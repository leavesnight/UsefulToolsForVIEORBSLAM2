#include<iostream>
#include<fstream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Time.h>
#include<std_msgs/Header.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "Thirdparty/DLib/FileFunctions.h"

//#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
//#include <sensor_msgs/Image.h>

#include <boost/foreach.hpp>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BagFromImages");

    switch(argc)
    {
      case 5:case 6:case 7:case 8:
	break;
      default:
        cerr << "Usage: rosrun BagFromImages BagFromImages <path to image directory> <image extension .ext> <frequency> <path to output bag> (<depth extension .ext>) (number of IMU data) (color images' rosbag's name)" << endl;
        return 0;
    }

    ros::start();

    // Vector of paths to image
    vector<string> filenames,depthnames;
    ifstream finIMU;string strtmp;
    if (argc==5){
      filenames = DUtils::FileFunctions::Dir(argv[1], argv[2], true);
    }else{//6
      string strTmp(argv[1]);
      filenames = DUtils::FileFunctions::Dir((strTmp+"/rgb").c_str(), argv[2], true);
      depthnames = DUtils::FileFunctions::Dir((strTmp+"/depth").c_str(),argv[5],true);
      cout << "depth Images: " << depthnames.size() << endl;
      if (argc>6){//with odometry files
	finIMU.open(strTmp+"/IMUSensor.txt");
	getline(finIMU,strtmp);getline(finIMU,strtmp);getline(finIMU,strtmp);
	
	if (argc>7){//save images from referenced rosbag
	  rosbag::Bag bagin;bagin.open(string(argv[7]),rosbag::bagmode::Read);
	  vector<string> topics;
	  topics.push_back("/kinect2/qhd/image_color_rect");
	  topics.push_back("/kinect2/qhd/image_color");
	  rosbag::View view(bagin,rosbag::TopicQuery(topics));
	  int i=0;
	  BOOST_FOREACH(rosbag::MessageInstance const m,view){
	    sensor_msgs::ImageConstPtr simage=m.instantiate<sensor_msgs::Image>();
	    if (simage!=NULL){
	      const sensor_msgs::Image &img=*simage;
	      double tm_stamp=img.header.stamp.toSec();
	      char ch[25];//at least 10+1+6+4+1=22
	      sprintf(ch,"%.6f.",tm_stamp);//mpTracker->mCurrentFrame.mTimeStamp);
	      string rgbname=strTmp+"/rgb/"+ch+"bmp";
	      
	      cv_bridge::CvImageConstPtr cv_ptr=cv_bridge::toCvShare(simage);
	      cv::imwrite(rgbname,cv_ptr->image);
	      cout<<"Saved "<<++i<<" images~"<<endl;
	    }
	  }
	  vector<string> topicsD;
	  topicsD.push_back("/kinect2/qhd/image_depth_rect");
	  topicsD.push_back("/kinect2/qhd/image_depth");
	  rosbag::View viewD(bagin,rosbag::TopicQuery(topicsD));
	  i=0;
	  BOOST_FOREACH(rosbag::MessageInstance const m,viewD){
	    sensor_msgs::ImageConstPtr simage=m.instantiate<sensor_msgs::Image>();
	    if (simage!=NULL){
	      const sensor_msgs::Image &img=*simage;
	      double tm_stamp=img.header.stamp.toSec();
	      char ch[25];//at least 10+1+6+4+1=22
	      sprintf(ch,"%.6f.",tm_stamp);//mpTracker->mCurrentFrame.mTimeStamp);
	      string depthname=strTmp+"/depth/"+ch+"png";
	      
	      cv_bridge::CvImageConstPtr cv_ptr=cv_bridge::toCvShare(simage);
	      cv::imwrite(depthname,cv_ptr->image);
	      cout<<"Saved "<<++i<<" depth images~"<<endl;
	    }
	  }
	}
	
      }
    }

    cout << "Images: " << filenames.size() << endl;

    // Frequency
    double freq = atof(argv[3]);

    // Output bag
    rosbag::Bag bag_out(argv[4],rosbag::bagmode::Write);

    ros::Time t,t_old = ros::Time::now();
    t=t_old;

    const float T=1.0f/freq;
    ros::Duration d(T);

    for(size_t i=0;i<filenames.size();i++)
    {
        if(!ros::ok())
            break;

        cv::Mat im = cv::imread(filenames[i],CV_LOAD_IMAGE_COLOR);
        cv_bridge::CvImage cvImage;
        cvImage.image = im;
	if (argc==5){
	  cvImage.encoding = sensor_msgs::image_encodings::RGB8;
	}else{
	  //cvImage.encoding=sensor_msgs::image_encodings::BGR8;
	  cvImage.encoding = sensor_msgs::image_encodings::RGB8;
	  int pos=filenames[i].rfind('/')+1;
	  t=ros::Time(atof(filenames[i].substr(pos,filenames[i].rfind(argv[2])-pos).c_str()));
	}
        cvImage.header.stamp = t;
	if (argc==5){
	  bag_out.write("/camera/image_raw",ros::Time(t),cvImage.toImageMsg());
	}else{
	  bag_out.write("/cam0/rgb/image",ros::Time(t),cvImage.toImageMsg());
	}
        t+=d;
        cout << i+1 << " / " << filenames.size() << endl;
    }
    if (argc>5){//write depth images to rosbag::Bag
      t=t_old;
      for(size_t i=0;i<depthnames.size();i++)
      {
	  if(!ros::ok())
	      break;

	  cv::Mat im = cv::imread(depthnames[i],CV_LOAD_IMAGE_UNCHANGED);
	  cv_bridge::CvImage cvImage;
	  cvImage.image = im;
	  cvImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	  int pos=depthnames[i].rfind('/')+1;
	  t=ros::Time(atof((depthnames[i].substr(pos,depthnames[i].rfind(argv[5])-pos)).c_str()));
	  cvImage.header.stamp = t;
	  bag_out.write("/cam0/depth/image",ros::Time(t),cvImage.toImageMsg());
	  cout << i+1 << " / " << depthnames.size() << endl;
      }
      if (argc>6){//write IMU data to rosbag
	double tmstamp,data[9],quat[4];//9/13
	do{
	  finIMU>>tmstamp;
	  sensor_msgs::ImuPtr pdataIMU(new sensor_msgs::Imu());
	  sensor_msgs::Imu &dataIMU=*pdataIMU;
	  if (atoi(argv[6])==13){
	    finIMU>>quat[3]>>quat[0]>>quat[1]>>quat[2];//wxyz while q3 is w
	    dataIMU.orientation.w=quat[3];dataIMU.orientation.x=quat[0];dataIMU.orientation.y=quat[1];dataIMU.orientation.z=quat[2];
	  }
	  for (int i=0;i<9;++i) finIMU>>data[i];//magxyz axyz wxyz
	  getline(finIMU,strtmp);
	  dataIMU.angular_velocity.x=data[6];dataIMU.angular_velocity.y=data[7];dataIMU.angular_velocity.z=data[8];
	  for (int i=3;i<6;++i) data[i]*=9.81;
	  dataIMU.linear_acceleration.x=data[3];dataIMU.linear_acceleration.y=data[4];dataIMU.linear_acceleration.z=data[5];
// 	  tmstamp+=0.06;
	  t=ros::Time(tmstamp);
	  dataIMU.header.stamp=t;
	  bag_out.write("/imu0",t,pdataIMU);
	}while (!finIMU.eof());
      }
    }

    bag_out.close();

    ros::shutdown();

    return 0;
}
