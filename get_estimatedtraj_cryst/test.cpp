//make groundtruth
#include <iomanip>
#include <ctime>
#include <iostream>//for ios_base
#include <fstream>
#include <string>
#include <stdlib.h>
#include <vector>
//make associate
#include <dirent.h>
#include <algorithm>
//make trajectory
#include <Eigen/Core>
#include <Eigen/Geometry>
//make estimated trajectory from odometry data
#include <cmath>

//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui.hpp>
#include<opencv2/opencv.hpp>

//#include "g2o/core/sparse_optimizer.h"


using namespace std;

inline void check_directory(string& str);
inline void getpath(string directory,vector<string>& files);
void calc_a_alpha(double a[6],double delta[2]);
double gettotallength(string path,int jump_lines=0,int constzy=0);//0 means jump 0 lines,0 no const 1 for const z 2 for const y!

namespace std{
  using namespace Eigen;
Matrix3d skew(const Vector3d&v)
{
  Matrix3d m;
  m.fill(0.);
  m(0,1)  = -v(2);
  m(0,2)  =  v(1);
  m(1,2)  = -v(0);
  m(1,0)  =  v(2);
  m(2,0) = -v(1);
  m(2,1) = v(0);
  return m;
}
}
Eigen::Matrix3d getJacoright(double th,Eigen::Vector3d& a){
  Eigen::Matrix3d skewa=skew(a);
  if (th<1E-5){
    return (Eigen::Matrix3d::Identity()-th*skewa/2);
  }
  return (Eigen::Matrix3d::Identity()-(1-cos(th))/th*skewa+(1-sin(th)/th)*skewa*skewa);
}

int main(int argc,char** argv)
{ 
  //make a standard groundtruth file from a original one
  string str_old_truth,strtmp, path_old_truth;
  struct tm tmtmp;
  int ntmp;
  double dtimetmp,x,y,z,qx,qy,qz,qw,arrq[4];
  vector<double> reversedtmp;
  if (argc<2){
    cout<<"usage: ./test (directory of zzh's dataset) (2:EuRoC 3:leica/vicon / 2:TUM / 2:TUM2D / 2:zzh's dataset oldtruth.txt name, e.g. oldtruth4.txt)"<<endl;
    
    cout<<"Calculate Stereo's rectified Rc'cl(R1),Rc'cr(R2),Pclc'(P1),Pcrc'(P2) & Tbc(Tbc') used by VIO:"<<endl;
    cv::Mat Tbc1(4,4,CV_64F),Tbc2(4,4,CV_64F);
    Tbc1=(cv::Mat_<double>(4,4)<<0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
	  0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
	  -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
	  0.0, 0.0, 0.0, 1.0);
    Tbc2=(cv::Mat_<double>(4,4)<<0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
	  0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
	  -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
	  0.0, 0.0, 0.0, 1.0);
    cv::Mat K1,K2;K1=(cv::Mat_<double>(3,3)<<458.654, 0.0, 367.215, 0.0, 457.296, 248.375, 0.0, 0.0, 1.0);
    K2=(cv::Mat_<double>(3,3)<<457.587, 0.0, 379.999, 0.0, 456.134, 255.238, 0.0, 0.0, 1);
    cv::Mat D1,D2;D1=(cv::Mat_<double>(5,1)<<-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05, 0.0);
    D2=(cv::Mat_<double>(5,1)<<-0.28368365, 0.07451284, -0.00010473, -3.555907e-05, 0.0);
    cv::Mat R,T;
    cv::Mat Rbc1=Tbc1.rowRange(0,3).colRange(0,3),Rbc2=Tbc2.rowRange(0,3).colRange(0,3);
    cv::Mat tbc1=Tbc1.rowRange(0,3).col(3),tbc2=Tbc2.rowRange(0,3).col(3);
    R=Rbc2.t()*Rbc1;T=Rbc2.t()*tbc1-Rbc2.t()*tbc2;//R21
    cv::Mat R1,R2,P1,P2,Q;
    cv::stereoRectify(K1,D1,K2,D2,cv::Size(752,480),R,T,R1,R2,P1,P2,Q,cv::CALIB_ZERO_DISPARITY,0);//for no black area!
    cout<<"R1,R2:"<<endl;
    cout<<R1<<endl<<R2<<endl;
    cout<<"P1,P2:"<<endl;
    cout<<P1<<endl<<P2<<endl;
    /*
    cv::Mat Rl2=(cv::Mat_<double>(3,3)<<0.9999633526194376, -0.003625811871560086, 0.007755443660172947, 0.003680398547259526, 0.9999684752771629, -0.007035845251224894, -0.007729688520722713, 0.007064130529506649, 0.999945173484644);
    cv::Mat Rl1=(cv::Mat_<double>(3,3)<<0.999966347530033, -0.001422739138722922, 0.008079580483432283, 0.001365741834644127, 0.9999741760894847, 0.007055629199258132, -0.008089410156878961, -0.007044357138835809, 0.9999424675829176);
    cv::Mat t2l=(cv::Mat_<double>(3,1)<<-47.90639384423901/435.2046959714599,0,0),t1l=cv::Mat::zeros(3,1,CV_64F);
    cv::Mat R21,t21;
    R21=Rl2.t()*Rl1;
    t21=Rl2.t()*(-Rl1*t1l)+t2l;
    cout<<"Check!"<<endl;
    cout<<R21<<endl<<t21<<endl;
    cout<<R<<endl<<T<<endl;*/
    cout<<"Tbc:"<<endl;
    cv::Mat T1l=cv::Mat::eye(4,4,CV_64F);//Tl1.t() or Tc'c0.t()
    T1l.rowRange(0,3).colRange(0,3)=R1.t();//R1l=Rl1.t()
    T1l.rowRange(0,3).col(3)=-R1.t() * P1.rowRange(0,3).col(3)/P1.at<double>(0,0);//t1l=-R1l1*tl1_1*f/f or use t1l1*f/f or 0
//     cout<<T1l<<endl;
    cv::Mat Tbl=Tbc1*T1l;//Tbc'=Tbc0*Tc'c0.t()
    cout<<Tbl<<endl;
    
    return 0;
  }else{
    str_old_truth=argv[1];
    check_directory(str_old_truth);
    cout<<"rectified directory name: "<<str_old_truth<<endl;
  }
  path_old_truth = str_old_truth + "oldtruth4.txt";
  //test the length of the TUM dataset && we cannot estimate the trajectory from the 3 kinds of acceleration and we need the angle at least!
  if (argc>=3){
    if ((string(argv[2])=="TUM"||string(argv[2])=="TUM2D")){
      int constzy=0;
      if (string(argv[2])=="TUM2D") constzy=1; 
      cout<<"total length of groundtruth is "<<gettotallength(str_old_truth+"groundtruth.txt",3,constzy)<<endl;
      if (string(argv[2])=="TUM2D") constzy=2; 
      cout<<"total length of orbslam2 estimated path is "<<gettotallength(str_old_truth+"orbslam2/CameraTrajectory.txt",0,constzy)<<endl;
      ntmp=str_old_truth.rfind('/');
      double ntmp2=str_old_truth.rfind('/',ntmp-1);
      strtmp=str_old_truth.substr(ntmp2+1,ntmp-ntmp2-1);
      cout<<"total length of rgbdslamv2 estimated path is "<<gettotallength(str_old_truth+"rgbdslamv2/"+strtmp+".bagiteration_4_estimate.txt",1,constzy)<<endl;
      
      cout<<"estimate orb "<<gettotallength(str_old_truth+"orbslam2/CrystalTrajectory.txt",0,constzy)<<endl;
      if (string(argv[2])=="TUM2D") constzy=1; 
      cout<<"estimate "<<gettotallength(str_old_truth+"CrystalTrajectoryFromOdom.txt",2,constzy)<<endl;
      return 0;
    }else if (string(argv[2])=="EuRoC"){
      //make groundtruth_estimate0.txt
      ifstream fin_old_truth(str_old_truth+"/mav0/state_groundtruth_estimate0/data.csv",ios_base::in);
      ofstream fout_truth;
      int arrtmp[6];
      double total_length=0;
      if (!fin_old_truth.is_open()){
        cout<<"open oldtruth_estimate0 wrong!!!"<<endl;
        return -1;
      }
      getline(fin_old_truth,strtmp);//pass the 1st unused line #...
      fout_truth.open(str_old_truth+"/groundtruth_estimate0.txt",ios_base::out);
      fout_truth<<"# ground truth estimate0 trajectory\n# file: 'rgbd_dataset_zzh.bag'\n# timestamp tx ty tz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz\n";
      fout_truth<<fixed;
      int NUM_PER_LINE=17;//total 1+3+4+3*3 numbers per line
      int NUM_Q_OVER=8;//1+3+4
      while (!fin_old_truth.eof()){
        fin_old_truth>>strtmp;
        ntmp=strtmp.length();
        //if (strtmp[0]=='T') break;
        int posLast=0;double qwTmp;
        double len2=0;static double xyzLast[3],lineNum=0;
        for (int i=0;i<NUM_PER_LINE;++i){
          int pos=strtmp.find(',',posLast);
          string::size_type posNum;if (pos!=string::npos) posNum=pos-posLast;else posNum=string::npos;
          double dtmp=atof(strtmp.substr(posLast,posNum).c_str());
          if (i==4)
            qwTmp=dtmp;
          else{
            if (i==0) fout_truth<<setprecision(9)<<dtmp/1e9;
            else fout_truth<<setprecision(6)<<dtmp;//input time,x,y,z,qw,qx,qy,qz but we want output as time,x,y,z,qx,qy,qz,qw
            if (i==NUM_PER_LINE-1) fout_truth<<endl;
            else{
              fout_truth<<" ";
              if (i==NUM_Q_OVER-1) fout_truth<<qwTmp<<" ";
            }
            if (lineNum==0){
              xyzLast[i-1]=dtmp;
            }else if (i>0&&i<4){//x,y,z
              double del=dtmp-xyzLast[i-1];
              len2+=del*del;//delx^2+dely^2+delz^2
              xyzLast[i-1]=dtmp;
            }
          }
          posLast=pos+1;
        }
        total_length+=sqrt(len2);++lineNum;
      }
      fin_old_truth.close();
      fout_truth.close();
      cout<<"new groundtruth_estimate0.txt is made! && the total length is: "<<total_length<<endl;
      //make groundtruth_vicon0/leica0.txt
      string truthmethod="vicon0";
      if (argc>=4&&string(argv[3])=="leica") truthmethod="leica0";
      fin_old_truth.open(str_old_truth+"/mav0/"+truthmethod+"/data.csv",ios_base::in);
      total_length=0;
      if (!fin_old_truth.is_open()){
        cout<<"open oldtruth wrong!!!"<<endl;
        return -1;
      }
      getline(fin_old_truth,strtmp);//pass the 1st unused line #...
      fout_truth.open(str_old_truth+"/groundtruth_"+truthmethod+".txt",ios_base::out);
      fout_truth<<"# ground truth trajectory\n# file: 'rgbd_dataset_zzh.bag'\n# timestamp tx ty tz qx qy qz qw\n";
      fout_truth<<fixed;
      NUM_PER_LINE=8;//total 1+3+4 numbers per line
      while (!fin_old_truth.eof()){
        fin_old_truth>>strtmp;
        ntmp=strtmp.length();
        //if (strtmp[0]=='T') break;
        int posLast=0;double qwTmp;
        double len2=0;static double xyzLast[3],lineNum=0;
        for (int i=0;i<NUM_PER_LINE;++i){
          int pos=strtmp.find(',',posLast);
          string::size_type posNum;if (pos!=string::npos) posNum=pos-posLast;else posNum=string::npos;
          double dtmp=atof(strtmp.substr(posLast,posNum).c_str());
          if (i==4)
            qwTmp=dtmp;
          else{
            if (i==0) fout_truth<<setprecision(9)<<dtmp/1e9<<" ";
            else fout_truth<<setprecision(17)<<dtmp<<" ";//input time,x,y,z,qw,qx,qy,qz but we want output as time,x,y,z,qx,qy,qz,qw
            if (i==NUM_PER_LINE-1) fout_truth<<qwTmp<<endl;
            if (lineNum==0){
              xyzLast[i-1]=dtmp;
            }else if (i>0&&i<4){//x,y,z
              double del=dtmp-xyzLast[i-1];
              len2+=del*del;//delx^2+dely^2+delz^2
              xyzLast[i-1]=dtmp;
            }
          }
          posLast=pos+1;
          if (truthmethod=="leica0"&&i==3){//for leica0
            fout_truth<<"0 0 0 1"<<endl;
            break;
          }
        }
        total_length+=sqrt(len2);++lineNum;
      }
      fin_old_truth.close();
      fout_truth.close();
      cout<<"new groundtruth_"<<truthmethod<<".txt is made! && the total length is: "<<total_length<<endl;
      //make a true Twb from Twp using Tbp, p means prism, b means IMU
      fin_old_truth.open(str_old_truth+"/groundtruth_"+truthmethod+".txt",ios_base::in);
      total_length=0;
      fout_truth.open(str_old_truth+"/groundtruth.txt",ios_base::out);
      double dtmp;
      Eigen::Isometry3d Tpb;
      cout<<str_old_truth+"/mav0/"+truthmethod+"/sensor.yaml"<<endl;
      cv::FileStorage fsSettings(str_old_truth+"/mav0/"+truthmethod+"/sensor.yaml",cv::FileStorage::READ);
      if (!fsSettings.isOpened()) cout<<"Wrong in openning"<<truthmethod<<"/sensor.yaml!!!"<<endl;
      cv::FileNode fnT=fsSettings["T_BS"];
      if (fnT.empty()) cout<<"Empty Node!"<<endl;
      else{
        for (int i=0;i<4;++i)
          for (int j=0;j<4;++j)
            Tpb.matrix()(i,j)=fnT["data"][i*4+j];
        //cout<<Tbc.matrix()<<endl;
      }
      Tpb=Tpb.inverse();
      getline(fin_old_truth,strtmp);//pass 3 unused lines #...
      getline(fin_old_truth,strtmp);getline(fin_old_truth,strtmp);
      while (!fin_old_truth.eof()){
        fin_old_truth>>dtmp;
        fout_truth<<setprecision(6)<<dtmp<<" ";
        double x,y,z,qx,qy,qz,qw;
        fin_old_truth>>x>>y>>z>>qx>>qy>>qz>>qw;
        Eigen::Quaterniond q(qw,qx,qy,qz);
        Eigen::Vector3d t(x,y,z);
        Eigen::Isometry3d Twp(q);
        Twp.matrix().block<3,1>(0,3)=t;
        Eigen::Isometry3d Twb;
        Twb=Twp*Tpb;
        //cout<<Tbp.matrix()<<endl;
        Eigen::Quaterniond qout(Twb.rotation());
        if (qout.w()<0) qout.coeffs()*=-1;
        fout_truth<<setprecision(7)<<Twb(0,3)<<" "<<Twb(1,3)<<" "<<Twb(2,3);
        if (truthmethod=="leica0") fout_truth<<" 0 0 0 1"<<endl;
        else fout_truth<<" "<<qout.x()<<" "<<qout.y()<<" "<<qout.z()<<" "<<qout.w()<<endl;
      }
      fin_old_truth.close();
      fout_truth.close();
      cout<<"new groundtruth.txt is made! && the total length is: "<<total_length<<endl;
      //make trajectory Twb from Twc
      Eigen::Isometry3d Tcb;
      fsSettings.open(str_old_truth+"/mav0/cam0/sensor.yaml",cv::FileStorage::READ);
      if (!fsSettings.isOpened()) cout<<"Wrong!!!"<<endl;
      fnT=fsSettings["T_BS"];
      if (fnT.empty()) cout<<"Empty Node!"<<endl;
      else{
        for (int i=0;i<4;++i)
          for (int j=0;j<4;++j)
            Tcb.matrix()(i,j)=fnT["data"][i*4+j];
      }
      Tcb=Tcb.inverse();
      ifstream fin_traj(str_old_truth+"/orbslam2/KeyFrameTrajectory.txt",ios_base::in);
      if (fin_traj.is_open()){
        ofstream fout_traj(str_old_truth+"/orbslam2/KeyFrameTrajectoryIMU.txt",ios_base::out);
        fout_traj<<fixed;
        while (!fin_traj.eof()){
          fin_traj>>dtmp;
//          fout_traj<<setprecision(6)<<dtmp<<" ";
          fout_traj<<setprecision(6)<<dtmp/1e9<<" ";//for ORB_SLAM3
          double x,y,z,qx,qy,qz,qw;
          fin_traj>>x>>y>>z>>qx>>qy>>qz>>qw;
          getline(fin_traj,strtmp);//maybe recording pseudo velocity&bg,ba
          Eigen::Quaterniond q(qw,qx,qy,qz);
          Eigen::Vector3d t(x,y,z);
          Eigen::Isometry3d Twc(q);
          Twc.matrix().block<3,1>(0,3)=t;
          Eigen::Isometry3d Twb;
//          Twb=Twc*Tcb;
          Twb=Twc;
          //cout<<Tbp.matrix()<<endl;
          Eigen::Quaterniond qout(Twb.rotation());
          if (qout.w()<0) qout.coeffs()*=-1;
          fout_traj<<setprecision(7)<<Twb(0,3)<<" "<<Twb(1,3)<<" "<<Twb(2,3)
          <<" "<<qout.x()<<" "<<qout.y()<<" "<<qout.z()<<" "<<qout.w()
          <<endl;
        }
        fin_traj.close();
        fout_traj.close();
        cout<<"New KFTrajectoryIMU made from KFTrajectory of camera!"<<endl;
      }
      
      return 0;
    }else {
        path_old_truth = str_old_truth + argv[2];
    }
  }
  ifstream fin_old_truth(path_old_truth, ios_base::in);
  ofstream fout_truth;
  int arrtmp[6];
  double total_length=0;
  if (!fin_old_truth.is_open()){
    cout<<"open oldtruth wrong!!!"<<endl;
    goto labeljump1;
  }
  fout_truth.open(str_old_truth+"groundtruth.txt",ios_base::out);
  fout_truth<<"# ground truth trajectory\n# file: 'rgbd_dataset_zzh.bag'\n# timestamp tx ty tz qx qy qz qw\n";
  tmtmp.tm_year=2018-1900;//2017(for old VI dataset)
  cout<<fixed<<setprecision(4);
  fout_truth<<fixed<<setprecision(4);
  while (!fin_old_truth.eof()){
    fin_old_truth>>strtmp;
    //getline(fin_old_truth,strtmp);
    ntmp=strtmp.length();
    if (strtmp[0]=='T') break;
    strtmp=strtmp.substr(4);//delete 0471
    sscanf(strtmp.substr(0,12).c_str(),"%02d%02d%02d%02d%02d%02d",arrtmp,arrtmp+1,arrtmp+2,arrtmp+3,arrtmp+4,arrtmp+5);
    tmtmp.tm_mon=arrtmp[0]-1;tmtmp.tm_mday=arrtmp[1]+1;tmtmp.tm_hour=arrtmp[2]-1;tmtmp.tm_min=arrtmp[3];tmtmp.tm_sec=arrtmp[4];//+1 day
    dtimetmp=mktime(&tmtmp)+4+arrtmp[5]/100.0;//42(for old VI dataset)
    //cout<<dtimetmp<<" "<<endl;
    strtmp=strtmp.substr(13);
    //cout<<strtmp<<" ";
    int pos1=strtmp.find(',');
    int pos2=strtmp.find(',',pos1+1);
    x=atof(strtmp.substr(0,pos1).c_str());
    y=atof(strtmp.substr(pos1+1,pos2-pos1-1).c_str());
    z=atof(strtmp.substr(pos2+1).c_str());
    //cout<<x<<" "<<y<<" "<<z<<endl;
    reversedtmp.push_back(dtimetmp);reversedtmp.push_back(x);reversedtmp.push_back(y);reversedtmp.push_back(z);
  }
  ntmp=reversedtmp.size();
  for (int i=0;i<ntmp/4;++i){
    fout_truth<<reversedtmp[ntmp-i*4-4]<<" "<<reversedtmp[ntmp-i*4-3]<<" "<<reversedtmp[ntmp-i*4-2]<<" "<<reversedtmp[ntmp-i*4-1]
    <<" 0.0000 0.0000 0.0000 1.0000"<<endl;
    if (i>0){
      double delx_tmp=reversedtmp[ntmp-i*4-3]-reversedtmp[ntmp-(i-1)*4-3];
      double dely_tmp=reversedtmp[ntmp-i*4-2]-reversedtmp[ntmp-(i-1)*4-2];
      double delz_tmp=reversedtmp[ntmp-i*4-1]-reversedtmp[ntmp-(i-1)*4-1];
      total_length+=sqrt(delx_tmp*delx_tmp+dely_tmp*dely_tmp+delz_tmp*delz_tmp);
    }
  }
  fin_old_truth.close();
  fout_truth.close();
  cout<<"new groundtruth.txt is made! && the total length is: "<<total_length<<endl;
labeljump1:
  
  //make an associate file
  double minTime=-1,maxTime;//for estimated time ratio
  ofstream fout_associate(str_old_truth+"associate.txt"),fout_rgb(str_old_truth+"rgb.txt");
  vector<string> files,filesdepth;
  getpath(str_old_truth+"rgb",files);
  getpath(str_old_truth+"depth",filesdepth);
  int alignNum=files.size();
  if (files.size()>filesdepth.size()){
    alignNum=filesdepth.size();
  }
  fout_rgb<<"# color images\n# file: 'rgbd_dataset_zzh.bag'\n# timestamp filename\n";
  for (int i=0;i<alignNum;++i){
    //cout<<files[i]<<endl;
    strtmp=files[i].substr(0,files[i].rfind('.'));
    fout_associate<<strtmp<<" rgb/"<<files[i]<<" "<<strtmp<<" depth/"<<filesdepth[i]<<endl;
    fout_rgb<<strtmp<<" rgb/"<<files[i]<<endl;
    if (minTime==-1) minTime=atof(strtmp.c_str());
  }
  maxTime=atof(strtmp.c_str());
  fout_associate.close();
  cout<<"associate.txt && rgb.txt are made!"<<endl;
  
  //make a new estimated trajectory of the crystal from the one of the camera
  const double xccr[3]={-0.02,-0.027,-0.468};//the translational difference from camera to crystal
  Eigen::Isometry3d Tccr=Eigen::Isometry3d::Identity();
  Tccr.pretranslate(Eigen::Vector3d(xccr));
  ifstream fin_oldtraj(str_old_truth+"/orbslam2/CameraTrajectory.txt");
  ofstream fout_traj(str_old_truth+"orbslam2/CrystalTrajectory.txt");
  fout_traj<<fixed<<setprecision(9);
  bool flag_1st=true;
  double xold,yold,zold,told;
  total_length=0;double time_ratio=0;
  if (!fin_oldtraj.is_open()){
    cout<<"open orbslam2/CameraTrajectory.txt wrong!!!please run orbslam2 first!"<<endl;
    goto labeljump2;
  }
  while (!fin_oldtraj.eof()){
    fin_oldtraj>>strtmp;
    fout_traj<<strtmp<<" ";
    fin_oldtraj>>x>>y>>z>>arrq[0]>>arrq[1]>>arrq[2]>>arrq[3];
    Eigen::Quaterniond q(arrq);//arrq is the internal format
    Eigen::Isometry3d Twc=Eigen::Isometry3d::Identity(),Twcr;
    Twc.rotate(q);
    Twc.pretranslate(Eigen::Vector3d(x,y,z));
    Twcr=Twc*Tccr;
//     Twcr=Tccr.inverse()*Twc*Tccr;
    fout_traj<<Twcr(0,3)<<" "<<Twcr(1,3)<<" "<<Twcr(2,3)<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<1.0<<endl;
    if (flag_1st){
      flag_1st=false;
    }else{
      double delx_tmp=Twcr(0,3)-xold;
      double dely_tmp=Twcr(1,3)-yold;
      double delz_tmp=Twcr(2,3)-zold;
      if (atof(strtmp.c_str())-told<2){
        total_length+=sqrt(delx_tmp*delx_tmp+dely_tmp*dely_tmp+delz_tmp*delz_tmp);
        time_ratio+=atof(strtmp.c_str())-told;
      }
    }
    xold=Twcr(0,3);yold=Twcr(1,3);zold=Twcr(2,3);told=atof(strtmp.c_str());
  }
  fin_oldtraj.close();
  fout_traj.close();
  cout<<"new trajectory of the crystal is made! estimated length = "<<total_length<<"; time_ratio="<<time_ratio/(maxTime-minTime)<<", max_time="<<(maxTime-minTime)<<endl;
  //make a new estimated trajectory of the crystal from the one of the monocular camera
  fin_oldtraj.open(str_old_truth+"/orbslam2/Monocular/KeyFrameTrajectory.txt");
  fout_traj.open(str_old_truth+"orbslam2/Monocular/CrystalTrajectory.txt");
  fout_traj<<fixed<<setprecision(9);
  flag_1st=true;
  total_length=0;
  if (!fin_oldtraj.is_open()){
    cout<<"open orbslam2/Monocular/KeyFrameTrajectory.txt wrong!!!please run orbslam2 first!"<<endl;
    goto labeljump2;
  }
  while (!fin_oldtraj.eof()){
    fin_oldtraj>>strtmp;
    fout_traj<<strtmp<<" ";
    fin_oldtraj>>x>>y>>z>>arrq[0]>>arrq[1]>>arrq[2]>>arrq[3];
    Eigen::Quaterniond q(arrq);//arrq is the internal format
    Eigen::Isometry3d Twc=Eigen::Isometry3d::Identity(),Twcr;
    Twc.rotate(q);
    Twc.pretranslate(Eigen::Vector3d(x,y,z));
    Twcr=Tccr.inverse()*Twc*Tccr;
    fout_traj<<Twcr(0,3)<<" "<<Twcr(1,3)<<" "<<Twcr(2,3)<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<1.0<<endl;
    if (flag_1st){
      flag_1st=false;
    }else{
      double delx_tmp=Twcr(0,3)-xold;
      double dely_tmp=Twcr(1,3)-yold;
      double delz_tmp=Twcr(2,3)-zold;
      if (atof(strtmp.c_str())-told<2)
	total_length+=sqrt(delx_tmp*delx_tmp+dely_tmp*dely_tmp+delz_tmp*delz_tmp);
    }
    xold=Twcr(0,3);yold=Twcr(1,3);zold=Twcr(2,3);told=atof(strtmp.c_str());
  }
  fin_oldtraj.close();
  fout_traj.close();
  cout<<"new monocular trajectory of the crystal is made! estimated length = "<<total_length<<endl;
labeljump2:
  
  //calculate the pose from the encoder/hall sensor data
  //this world frame is rotaionally different from the one before,but still called the cr frame at the time 0
//#define estimate_mode 0//0 means Complementary Filter(for old VI), 1 means KF, 2means only encoder(for VIE dataset)
  int estimate_mode=2;
  const int ppr=400,datatime=10;//pulse per revolution,the span of the hall sensor data
  const double wheelradius=0.105,carradius=0.280,vscaleforhall=1.649336143e-4;//2.0/ppr*M_PI*wheelradius/datatime;//radius for the driving wheels(m),the half distance between two driving wheels
  const double wscaleforhall=vscaleforhall/carradius;
  const double xocr[3]={0.05,0.005,0.5};//the translational difference from the centre of two driving wheels to crystal
  const double xoi_o[3]={0.024,0.324,-0.461};
  double v[2],xt[3]={0,0,0},vtmp=0,wtmp=0,lasttime=-1;//v[0] is vl or left wheel velocity
  //xt is the pose of the centre of two driving wheels
  ifstream fin_odom;
  fin_odom.open(str_old_truth+"EncSensor.txt");
  if (fin_odom.fail()) {
    fin_odom.open(str_old_truth+"odometrysensor.txt");
    estimate_mode = 0;
  }
  ofstream fout_odomtraj(str_old_truth+"CrystalTrajectoryFromOdom.txt");
  fout_odomtraj<<fixed<<"# estimated trajectory of the crystal from odometry\n# timestamp tx ty tz qx qy qz qw\n";
  for (int i=0;i<3;++i)
    getline(fin_odom,strtmp);//read 3 senseless lines
  Eigen::Isometry3d Tocr(Eigen::Isometry3d::Identity()),Toi_o(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1)));
  Tocr.pretranslate(Eigen::Vector3d(xocr));
  Toi_o.prerotate(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(1,0,0)));
  //cout<<Toi_o.rotation().eulerAngles(2,1,0).transpose()<<endl;
  Toi_o.pretranslate(Eigen::Vector3d(xoi_o));
  Eigen::Isometry3d To0_wi(Eigen::Isometry3d::Identity());//transformation/change from  the first odometry frame(centre of two wheels) to IMU internal frame(ros version meaning,not slam or coordinate meaning)
  const double vari_wt=0.0001;
  double var_qt=1.5;//const variance for episl_ut, const variance for delta_tht_imu
  const double vari_rt2=0.000001,var_qt2=1;//const variance for episl_t, const variance for delta_t
  double vari_tht=0;//initial variance for theta/the state xt
  static double max[2]={0};
  while (!fin_odom.eof()){
    fin_odom>>dtimetmp;
    if (dtimetmp-lasttime==0){//||dtimetmp-lasttime<0.07){
      getline(fin_odom,strtmp);
      continue;
    }
    fout_odomtraj<<setprecision(6)<<dtimetmp<<" ";
    fin_odom>>v[0]>>v[1];
    if (estimate_mode!=2) fin_odom>>arrq[0]>>arrq[1]>>arrq[2]>>arrq[3];
    getline(fin_odom,strtmp);
    //if (abs(v[1]-v[0])<00) v[1]=v[0]=(v[0]+v[1])/2;
    //if (abs(v[1])<200) v[1]=0;if (abs(v[0])<200) v[0]=0;
    if (lasttime<0){
      //xt[2]=xt[1]=xt[0]=0;
      To0_wi.prerotate(Eigen::Quaterniond(arrq));//actually use quaterniond.inverse() has some numerical error, u'd better use conjugate()!
      To0_wi=(To0_wi*Toi_o).inverse();//Tw_wi=(Twi_oi*Toi_o)^(-1);
      //To0_wi=To0_wi.inverse();//Toi0_wi
      vtmp=(v[0]+v[1])/2*vscaleforhall;//v1=v-vw;v2=v+vw => v=(v1+v2)/2
      wtmp=(v[1]-v[0])/2*wscaleforhall;// => w=(v2-v1)/2/r
    }else{
      Eigen::Isometry3d To0_o(Toi_o);//suppose w means o0 not cr0
      To0_o.prerotate(Eigen::Quaterniond(arrq));//Tw_o=Tw_oi*Toi_o=Tw_wi*Twi_oi*Toi_o;here it's just Twi_o
      To0_o=To0_wi*To0_o;//Tw_o=Tw_wi*Twi_o;end suppose
      //Eigen::Isometry3d To0_o(Eigen::Isometry3d::Identity()),Twi_oi(Eigen::Quaterniond(arrq).toRotationMatrix());
      //To0_o=To0_wi*Twi_oi;
      Eigen::AngleAxisd rotatvec_tmp(To0_o.rotation());
      double theta_tmp=(rotatvec_tmp.angle()*rotatvec_tmp.axis()).dot(Eigen::Vector3d(0,0,1));
      if (theta_tmp>M_PI) theta_tmp-=2*M_PI;
      else if (theta_tmp<=-M_PI) theta_tmp+=2*M_PI;
      //double theta2=To0_o.rotation().eulerAngles(2,1,0)[0];
      ///hall sensor data(counts during 10s)/400*pi*2*radius(mm)/1000/10=velocity
      double deltat=dtimetmp-lasttime;
      vtmp=((v[0]+v[1])/2*vscaleforhall+vtmp)/2;//v1=v-vw;v2=v+vw => v=(v1+v2)/2
      wtmp=((v[1]-v[0])/2*wscaleforhall+wtmp)/2;// => w=(v2-v1)/2/r
      double delta_rect=theta_tmp-xt[2];
      if (delta_rect>M_PI) delta_rect-=2*M_PI;
      else if (delta_rect<=-M_PI) delta_rect+=2*M_PI;
      //Complementary Filter
      const double k_comple=0.008;//it's good between 0.006~0.01
      //Kalman Filter/KF
      double vari_rt=vari_wt*deltat*deltat;
      //vari_rt=vari_rt2;var_qt=var_qt2;//use const variances
      double kt_w=(vari_tht+vari_rt)/(vari_tht+var_qt+vari_rt)/deltat;//Kt/deltat
      switch (estimate_mode){
	case 0:
	  //Complementary Filter
	  wtmp+=k_comple*delta_rect;
	  break;
	case 1:
	  //Kalman Filter/KF
	  if (kt_w>1) kt_w=1;
	  wtmp+=delta_rect*kt_w;
	  vari_tht=(vari_tht+vari_rt)*var_qt/(vari_tht+var_qt+vari_rt);
      }
      if (abs(wtmp*deltat)>=1E-5){//!=0
	xt[0]=xt[0]-vtmp/wtmp*sin(xt[2])+vtmp/wtmp*sin(xt[2]+wtmp*deltat);//x'=x+-v/w*sinth+v/w*sin(th+w*dt) =v*cos(th)*dt;
	xt[1]=xt[1]+vtmp/wtmp*cos(xt[2])-vtmp/wtmp*cos(xt[2]+wtmp*deltat);//y'=y+v/w*costh-v/w*cos(th+w*dt) =v*sin(th)*dt;
	xt[2]=xt[2]+wtmp*deltat;//th(eta)'=th+w*dt;
	if (xt[2]>M_PI) xt[2]-=2*M_PI;
	else if (xt[2]<=-M_PI) xt[2]+=2*M_PI;
      }else{
	xt[0]=xt[0]+vtmp*cos(xt[2])*deltat;
	xt[1]=xt[1]+vtmp*sin(xt[2])*deltat;
	xt[2]=xt[2]+wtmp*deltat;//th(eta)'=th+w*dt;
      }
      /*double inputtmp[6]={vtmp,(v[0]+v[1])/2*vscaleforhall,wtmp,(v[1]-v[0])/2*wscaleforhall,deltat,xt[2]},deltaxy[2];
      calc_a_alpha(inputtmp,deltaxy);
      xt[2]=xt[2]+(wtmp+(v[1]-v[0])/2*wscaleforhall)/2*deltat;//th(eta)'=th+w*dt;
      if (xt[2]>M_PI) xt[2]-=2*M_PI;
      else if (xt[2]<=-M_PI) xt[2]+=2*M_PI;
      xt[0]+=deltaxy[0];xt[1]+=deltaxy[1];*/
      vtmp=(v[0]+v[1])/2*vscaleforhall;
      wtmp=(v[1]-v[0])/2*wscaleforhall;
      //cout<<dtimetmp<<" "<<theta_tmp<<" "<<xt[2]<<" "<<"! "<<kt_w<<" "<<rotatvec_tmp.axis().transpose()<<endl;
    }
    Eigen::Quaterniond q(Eigen::AngleAxisd(xt[2],Eigen::Vector3d(0,0,1)));
    Eigen::Isometry3d Two(Eigen::Isometry3d::Identity()),Twcr;
    Two.rotate(q);
    Two.pretranslate(Eigen::Vector3d(xt[0],xt[1],0));
    Twcr=Two*Tocr;//though Tocr.inverse() is independent of the shape of the trajectory!
    fout_odomtraj<<setprecision(9)<<Twcr(0,3)<<" "<<Twcr(1,3)<<" "<<Twcr(2,3)<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<1.0<<endl;
    
    //check the error
    istringstream strinTmp(strtmp);double dtmp;
    strinTmp>>dtmp;double bCheckIMU=true;
    if (strinTmp.eof()){
    }else{
      strinTmp>>dtmp>>dtmp;//read 3 mag data
      strinTmp>>dtmp>>dtmp;//read IMU data/ax,ay use ay to filter wrong record
      if (dtmp>-0.7||dtmp<-1.3){
	//cout<<"Wrong imu data! t: "<<dtimetmp<<endl;
	bCheckIMU=false;
      }
    }
    Eigen::Isometry3d Twb;//here w means oi0
    Twb=Toi_o*Two*Toi_o.inverse();//Toi0_oi=Toio*To0o*Tooi, it has the state x(t)=[Rwb(t),pwb(t)]
    
    //check the error of Encoder rEij
#define normalizeThetaz(x) (x>M_PI)?x-=2*M_PI:(x<=-M_PI)?x+=2*M_PI:0;
    Eigen::Vector3d rEij;Eigen::Matrix2d Sigmaetad(Eigen::Matrix2d::Identity()*0.0025);//sigmaetad=5cm/s for etald & etard(try 1cm/s)
    double deltat=dtimetmp-lasttime;
    static double inittime=-1;
    Eigen::Matrix3d SigmaModeldij(Eigen::Matrix3d::Identity()*0.0001*(dtimetmp-inittime));//here is deltatij
    static 
    Eigen::Matrix3d SigmaEij(Eigen::Matrix3d::Zero());//cannot give wx,wy,vz info(their Sigma is infinity/Omega row/col is 0)
    Eigen::Matrix3d Rwbj=Twb.rotation();static Eigen::Matrix3d Rwbi;
    Eigen::Vector3d pwbj=Twb.translation();static Eigen::Vector3d pwbi;
    static Eigen::Matrix3d Rbo(Toi_o.rotation()),Rob(Rbo.transpose());static Eigen::Vector3d pbo(Toi_o.translation()),pob(-Rob*pbo);
    static 
    Eigen::Vector2d deltaPijM(0,0);//deltaThetaii=0,deltaPii=0
    static
    double deltaThetaijMz=0;
    static double vLast[2];
    if (lasttime<0){
      Rwbi=Rwbj;pwbi=pwbj;
      inittime=dtimetmp;
      vLast[0]=v[0]*vscaleforhall;vLast[1]=v[1]*vscaleforhall;
    }else{
      //get deltaThetaij
      Eigen::Matrix3d Roioj=Rob*Rwbi.transpose()*Rwbj*Rbo;
      Eigen::AngleAxisd angaxTmp(Roioj);//Log(Rob*Rbiw*Rwbj*Rbo)
      double deltaThetaijz=angaxTmp.angle()*angaxTmp.axis()[2];
      //get vlj-1 && vrj-1 => vf w
      double vl=(v[0]*vscaleforhall+vLast[0])/2;
      double vr=(v[1]*vscaleforhall+vLast[1])/2;
      vLast[0]=v[0]*vscaleforhall;vLast[1]=v[1]*vscaleforhall;
      double w=-vl/carradius/2+vr/carradius/2,vf=vl/2+vr/2;//[vf;w]=1/2*[1 1;-1/rc 1/rc]*[vl;vr]
      double deltaThetaz=deltaThetaijMz;
      //update deltaThetaijM
      deltaThetaijMz+=w*deltat;//deltaThetaij-1M + woioj-1 * deltatj-1j
      normalizeThetaz(deltaThetaijz);normalizeThetaz(deltaThetaijMz);
      //calculate the first part(deltaTheijz) of rEij
      rEij[0]=deltaThetaijz-deltaThetaijMz;//error of wz
      normalizeThetaz(rEij[0]);
      
      Eigen::Vector3d deltaPij=Rob*(Rwbi.transpose()*(pwbj-pwbi)-pbo-Rbo*Roioj*pob);//Rob*(Rbiw*(pwbj-pwbi)-pbo-Rbo*Exp(deltaThetaij)*pob)
      double arrdTmp[4]={cos(deltaThetaz),sin(deltaThetaz),-sin(deltaThetaz),cos(deltaThetaz)};//row-major:{cos(deltaTheij-1Mz),-sin(deltaTheij-1Mz),0,sin(deltaTheij-1Mz),cos(deltaThetaij-1Mz),0,0,0,1}; but Eigen defaultly uses col-major!
      deltaPijM+=Eigen::Matrix2d(arrdTmp)*Eigen::Vector2d(vf*deltat,0);//deltaPijM+Roioj-1*voj-1oj-1*deltat
      for (int i=0;i<2;++i) rEij[i+1]=deltaPij[i]-deltaPijM[i];
      
      double arrdTmp2[9]={1,sin(deltaThetaz)*vf*deltat,-cos(deltaThetaz)*vf*deltat, 0,1,0, 0,0,1};
      double arrdTmp3[3]={deltat/carradius/2,cos(deltaThetaz)*deltat/2,sin(deltaThetaz)*deltat/2};
      double arrdTmp4[6]={arrdTmp3[0],arrdTmp3[1],arrdTmp3[2], -arrdTmp3[0],arrdTmp3[1],arrdTmp3[2]};
      Eigen::Matrix3d A(arrdTmp2);Eigen::Matrix<double,3,2> B(arrdTmp4);
      SigmaEij=A*SigmaEij*A.transpose()+B*Sigmaetad*B.transpose();
      
      double erBl_rEij=rEij.transpose()*(SigmaEij+SigmaModeldij).inverse()*rEij;
      double deltaEncoder=sqrt(7.815);
      if (erBl_rEij>deltaEncoder*deltaEncoder){//chi2(0.05,3)
	erBl_rEij=deltaEncoder*(sqrt(erBl_rEij)-deltaEncoder/2);
      }else{
	erBl_rEij/=2;
      }
      if (erBl_rEij>2){
//       cout<<fixed<<setprecision(9)<<erBl_rEij<<" time:"<<dtimetmp<<endl;
//       cout<<dtimetmp-inittime<<endl;
      //cout<<deltaPij[0]<<" "<<deltaPij[1]<<";"<<deltaPijM[0]<<" "<<deltaPijM[1]<<";"<<deltaThetaij[2]<<" "<<deltaThetaijM[2]<<endl;
      }
      if (max[1]<erBl_rEij) max[1]=erBl_rEij;
      //cout<<SigmaEij+SigmaModeldij<<" Sigma"<<endl;
    }
    //Rwbi=Rwbj;pwbi=pwbj;
    
    //check the error of IMU rIij
    if (bCheckIMU){
    Eigen::Vector3d rIij;Eigen::Matrix3d SigmaIij=Eigen::Matrix3d::Identity()*0.1;//use sigma=20,20,60 degree in wi frame/Sigmaetawi
    SigmaIij(2,2)=1;
    Eigen::Matrix3d RwIjM_IMU(Eigen::Isometry3d(Eigen::Quaterniond(arrq)).rotation());static Eigen::Matrix3d RwIiM_IMU;//Twb~(tj);Twb~(ti) or just Twib~(tj);Twib~(ti)
    RwIjM_IMU=RwIjM_IMU;//Twb~=Toi0_oi=Toi_o*To0_wi*Twi_oi; or just use Twib
    Eigen::Matrix3d Rwbj_IMU(Twb.rotation());static Eigen::Matrix3d Rwbi_IMU;
    Eigen::Matrix3d Jrj;static Eigen::Matrix3d Ai;
    bool bswitchmode=0;
    if (lasttime<0){
      if (!bswitchmode){
      RwIiM_IMU=RwIjM_IMU;Rwbi_IMU=Rwbj_IMU;
      Eigen::AngleAxisd angaxTmp(RwIiM_IMU);//phi~wIb=Log(R~wiw*R~wbi) or Log(R~wibi)
      Eigen::Matrix3d Jri=getJacoright(angaxTmp.angle(),angaxTmp.axis());
      Ai=RwIjM_IMU.transpose()*RwIiM_IMU*Jri;
      }
    }else{
      Eigen::AngleAxisd angaxTmp(RwIjM_IMU.transpose()*RwIiM_IMU*Rwbi_IMU.transpose()*Rwbj_IMU);
      if (angaxTmp.angle()>M_PI){//[0,2*M_PI)
	rIij=(-angaxTmp.angle()+M_PI)*angaxTmp.axis();
      }else
	rIij=angaxTmp.angle()*angaxTmp.axis();
      //get SigmaIij=Ai*Sigmaetawi*Ai.t()+Jrj*Sigmaetawi*Jrj.t(),Ai=(Rj.t()*Ri)*Jr(phiwIb(ti))
      angaxTmp=RwIjM_IMU;//phi~wIb=Log(R~wiw*R~wbj) or Log(R~wibj)
      Jrj=getJacoright(angaxTmp.angle(),angaxTmp.axis());
      SigmaIij=Ai*SigmaIij*Ai.transpose()+Jrj*SigmaIij*Jrj.transpose();
      //cout<<SigmaIij<<" Sigma"<<endl;
      //calculate error of rIij
      double erBl_rIij=rIij.transpose()*SigmaIij.inverse()*rIij;
      //Huber robust core: optimization target=KernelHuber(block)=H(e)={1/2*e sqrt(e)<=delta;delta(sqrt(e)-1/2*delta) others}
      double deltaImu=sqrt(7.815);
      if (erBl_rIij>deltaImu*deltaImu){//chi2(0.05,3)
	erBl_rIij=deltaImu*(sqrt(erBl_rIij)-deltaImu/2);
      }else{
	erBl_rIij/=2;
      }
      if (erBl_rIij>2){
//       cout<<fixed<<setprecision(9)<<erBl_rIij<<" time: "<<dtimetmp<<endl;
      }
      if (max[0]<erBl_rIij) max[0]=erBl_rIij;
    }
    if (bswitchmode){
    RwIiM_IMU=RwIjM_IMU;Rwbi_IMU=Rwbj_IMU;
    Ai=RwIjM_IMU.transpose()*RwIiM_IMU*Jrj;
    }
    }
    
    lasttime=dtimetmp;
  }
  cout<<"Max error block of rIij= "<<max[0]<<endl<<"Max error block of rEij="<<max[1]<<endl;
  fin_odom.close();
  fout_odomtraj.close();
  cout<<"new estimeated trajectory of the crystal from odom is made!"<<endl;
  
  
  //calculate the pose from the IMU sensor data(need gyro & acc data, 6*1 vector)
  //this world frame is same as the one before, called the cr frame at the time 0
  //connect odom(IMU) data Gauss G(9.81m/s^2) rad/s
  fin_odom.open(str_old_truth+"odometrysensor.txt");
  fout_odomtraj.open(str_old_truth+"CrystalTrajectoryFromIMU.txt");
  fout_odomtraj<<fixed<<"# estimated trajectory of the crystal from IMU\n# timestamp tx ty tz qx qy qz qw\n";
  for (int i=0;i<3;++i)
    getline(fin_odom,strtmp);//read 3 senseless lines
  //solve Toicr from the centre of IMU(oi/Frame B) to crystal
  /*const double xocr[3]={0.05,0.005,0.5};//the translational difference from the centre of two driving wheels to crystal
  const double xoi_o[3]={0.024,0.324,-0.461};
  Eigen::Isometry3d Tocr(Eigen::Isometry3d::Identity()),Toi_o(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1)));//no rotation between odom and crystal frame(x forward), \
  but from imu to odom frame need RY rotation
  Tocr.pretranslate(Eigen::Vector3d(xocr));
  Toi_o.prerotate(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(1,0,0)));
  //cout<<Toi_o.rotation().eulerAngles(2,1,0).transpose()<<endl;
  Toi_o.pretranslate(Eigen::Vector3d(xoi_o));*/
  lasttime=-1;//v[0] is vl or left wheel velocity \
  xt is the pose of the centre of two driving wheels
  Eigen::Isometry3d Toicr(Toi_o);
  Toicr=Toicr*Tocr;
  //cout<<Toicr.matrix()<<endl;//right xoicr checked

  //imu data container
  double dtmp,imu[6],lastimu[6];//0~2 for wx~z 3~5 for ax~z
  //state container
  Eigen::Isometry3d Twb(Eigen::Isometry3d::Identity());//Rwb(ti) pwb(ti) initially = I 0; here w means b0/inital IMU frame
  Eigen::Vector3d vwb,bg(0,0,0),ba(0.0,-0.45,0.0),gw(0,9.81,0);//vwb(ti), b(ti)=[bg(ti) ba(ti)]
  double inittime;
  //cyclic process
  while (0&&!fin_odom.eof()){
    //reading imu data
    fin_odom>>dtimetmp;
    if (fin_odom.eof()) break;
    fout_odomtraj<<setprecision(6)<<dtimetmp<<" ";
    fin_odom>>v[0]>>v[1]>>arrq[0]>>arrq[1]>>arrq[2]>>arrq[3];
    getline(fin_odom,strtmp);
    istringstream strinTmp(strtmp);
    strinTmp>>dtmp;
    if (strinTmp.eof()){
      cout<<"Nothing Left!"<<endl;
      break;
    }
    strinTmp>>dtmp>>dtmp;//read 3 mag data
    strinTmp>>imu[3]>>imu[4]>>imu[5]>>imu[0]>>imu[1]>>imu[2];//read IMU data/ax,ay,az,wx,wy,wz
    if (imu[4]>-0.5||imu[4]<-1.5){
      cout<<"Wrong imu data! t: "<<dtimetmp<<endl;
      continue;
    }
    //for (int i=0;i<6;++i) cout<<imu[i]<<" ";cout<<endl;
    //state updation/Iterative Preintegration, de means delta
    //deR~ij=deR~ij-1*deR~j-1j=deR~ij-1*Exp((w~j-1 - b_bar_g_i)*de_t_j-1_j), here use i=0
    if (lasttime<0){
      if (dtimetmp>1506926104)
      inittime=lasttime=dtimetmp;
      for (int i=0;i<6;++i) lastimu[i]=imu[i];
      continue;
    }else{
      //get w~j-1 & a~j-1
      for (int i=0;i<6;++i){
	if (i>2){
	  imu[i]*=9.81;//change ax~z's unit to m/s^2
	  dtmp=imu[i];
	  //imu[i]=(imu[i]*4+lastimu[i])/5;//alpha=t*2pi*f=1/15*2*3.14*2~=0.8
	  //lastimu[i]=imu[i];
	  imu[i]=lastimu[i];lastimu[i]=dtmp;
	}else{
	  dtmp=imu[i];
	  imu[i]=lastimu[i];
	  lastimu[i]=dtmp;
	}
      }
      if (dtimetmp-inittime>2) break;
    }
    double deltat=dtimetmp-lasttime;//de_t_j-1_j
    cout<<fixed<<setprecision(9)<<dtimetmp<<endl;
    Eigen::Vector3d vecTmp(imu[0]*0,imu[1],imu[2]*0);
    Eigen::Matrix3d RwbLast=Twb.rotation();//deR~0j-1
    cout<<"vec w: "<<vecTmp.transpose()<<endl;
    //if (vecTmp.norm()>0.2){
    vecTmp=(vecTmp-bg)*deltat;//phi=(w~j-1 - b_bar_g_i)*de_t_j-1_j, here b_bar_g_i=0
    double vecNorm=vecTmp.norm();
    double eps = 0.00001;//1E-5 cf. g2o
    if (vecNorm<0){
      cout<<"Wrong in norm()"<<endl;
      break;
    }
    if (vecNorm<eps){
      Eigen::Matrix3d Rwb=RwbLast+RwbLast*skew(vecTmp);//deR~ij=deR~ij-1*(I+phi^)
      Eigen::Quaterniond qtmp(Rwb);qtmp.normalize();Rwb=qtmp;//project R into the SO(3) manifold vy normalizing its quaternion
      for (int i=0;i<3;++i) for (int j=0;j<3;++j) Twb(i,j)=Rwb(i,j);
    }else{
      cout<<"w: "<<vecTmp.transpose()<<endl<<vecNorm<<" "<<(vecTmp/vecNorm).transpose()<<endl;
      Twb.rotate(Eigen::AngleAxisd(vecNorm,vecTmp/vecNorm));
    }
    //}
    cout<<"rotation: "<<Eigen::AngleAxisd(Twb.rotation()).angle()<<endl<<Eigen::AngleAxisd(Twb.rotation()).axis().transpose()<<endl;
    //de_v~ij=de_v~ij-1 + deR~ij-1*(a~j-1 - b_bar_a_i)*de_t_j-1_j, here use i=0
    Eigen::Vector3d accTmp(imu[3],imu[4]*0-9.81-0.45,imu[5]);
    Eigen::Vector3d vwbLast=vwb;
    vwb=vwb+RwbLast*(accTmp-ba)*deltat;
    Eigen::Vector3d vwb_r=vwb+gw*(dtimetmp-inittime);cout<<"acc/vel: "<<accTmp.transpose()<<endl;cout<<vwb_r.transpose()<<endl;
    //de_p~ij=de_p~ij-1 + de_v~ij-1*de_t_j-1_j + deR~ij-1*(a~j-1 - b_bar_a_i)*de_t_j-1_j^2/2, here use i=0
    Eigen::Vector3d pwb=Twb.translation();
    pwb=pwb+vwbLast*deltat+RwbLast*(accTmp-ba)*deltat*deltat/2;
    for (int i=0;i<3;++i) Twb(i,3)=pwb[i];
    Eigen::Vector3d pwb_r=pwb+gw*(dtimetmp-inittime)*(dtimetmp-inittime)/2;cout<<"pwb: "<<pwb_r.transpose()<<endl;
    //calculate Twcr
    Eigen::Isometry3d Twoi(Eigen::Isometry3d::Identity()),Twcr;
    Twoi.rotate(Twb.rotation());
    Twoi.pretranslate(pwb_r);
    Twcr=Toicr.inverse()*Twoi*Toicr;//though Tocr.inverse() is independent of the shape of the trajectory!
    Eigen::Quaterniond qwoi(Twcr.rotation());
    cout<<setprecision(9)<<Twcr(0,3)<<" "<<Twcr(1,3)<<" "<<Twcr(2,3)<<" "<<qwoi.x()<<" "<<qwoi.y()<<" "<<qwoi.z()<<" "<<qwoi.w()<<endl;//fout_odomtraj
    
    lasttime=dtimetmp;
  }
  fin_odom.close();
  fout_odomtraj.close();
  cout<<"new estimeated trajectory of the crystal from IMU by IMU Preintegration is made!"<<endl;
  
  /*static const double xbase_c[3]={0.293+0.225,-0.015,0.578-0.105};//from base frame to camera frame(ROS)/from camera coordinate to base coordinate(ORBSLAM)
  static Eigen::Isometry3d Tbase_c(Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d(1,0,0)));//Roll/R23
  Tbase_c.prerotate(Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d(0,0,1)));//Y/R12,notice the order R12*R23=R13/YPR!
  Tbase_c.pretranslate(Eigen::Vector3d(xbase_c));
  cout<<"Tbo="<<(Toi_o).matrix()<<endl;
  cout<<"Tco="<<(Tbase_c.inverse()).matrix()<<endl;*/
  
  return 0;
}

void check_directory(string& str)
{
  if (str[str.length()-1]!='/'){
    str+="/";
  }
}
void getpath(string directory, vector< string >& files)
{
  DIR* dir;//the handle or drectory pointer
  struct dirent *ptr;//struct pointer
  directory+="/";
  if ((dir=opendir(directory.c_str()))==nullptr){
    cerr<<"cannot open "<<directory<<endl;
    exit(1);
  }
  while ((ptr=readdir(dir))!=nullptr){
    if (ptr->d_type==DT_REG){
      files.push_back(ptr->d_name);
    }
  }
  sort(files.begin(),files.end());
}
void calc_a_alpha(double a[6], double delta[2])//a[4] is v0,vt,w0,wt,deltat,theta;delta[2] is deltax, deltay
{
  delta[0]=delta[1]=0;
  if (a[4]==0){
    return;
  }
  double acc=(a[1]-a[0])/a[4],alpha=(a[3]-a[2])/a[4];
  const double CALC_STEP=0.001;//0.00001; will converge at least 0.001 the similar accuracy with bar value!
  for (int i=0;i<a[4]/CALC_STEP;++i){//step is deltat/MAX_CALC_STEP_NUM
    double t=i*CALC_STEP;
    delta[0]+=(a[0]+acc*t)*cos(a[5]+a[2]*t+alpha/2*t*t)*CALC_STEP;
    delta[1]+=(a[0]+acc*t)*sin(a[5]+a[2]*t+alpha/2*t*t)*CALC_STEP;
  }
}
double gettotallength(string path,int jump_lines,int constzy)
{
  ifstream fin(path);
  bool flag_1st=true;
  double xold,yold,zold,told,tnow,x,y,z,arrq[4];
  double total_length=0;
  string strtmp;
  if (!fin.is_open()){
    cout<<"cannot open "<<path<<"!"<<endl;
    return 0;
  }
  for (int i=0;i<jump_lines;++i)
    getline(fin,strtmp);//read 3 senseless lines
  while (!fin.eof()){
    fin>>tnow;
    fin>>x>>y>>z>>arrq[0]>>arrq[1]>>arrq[2]>>arrq[3];
    if (flag_1st){
      flag_1st=false;
    }else{
      double delx_tmp=x-xold;
      double dely_tmp=y-yold;
      double delz_tmp=z-zold;
      if (constzy==1) delz_tmp=0;
      else if (constzy==2) dely_tmp=0;
      if (tnow-told<2){
	double delta=sqrt(delx_tmp*delx_tmp+dely_tmp*dely_tmp+delz_tmp*delz_tmp);
	//if (delta/(tnow-told)<1)
	  total_length+=delta;
      }
    }
    xold=x;yold=y;zold=z;told=tnow;
  }
  fin.close();
  return total_length;
}
