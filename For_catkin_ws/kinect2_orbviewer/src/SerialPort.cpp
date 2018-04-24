#include "SerialPort.h"
//#include <cwchar>
#include <iostream>
//make trajectory
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
//publish ros topic
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>//for ros::Time

bool CSerialPort::st_bExit=false;//静态变量必须初始化
double CSerialPort::m_tm_stamp=0;
unsigned char CSerialPort::mbstartRecord=0;
std::map<double,std::vector<double>> CSerialPort::mmOdomHis[2]={std::map<double,std::vector<double>>(),std::map<double,std::vector<double>>()};
const UINT SLEEP_TIME_INTERVAL=10;//串口无数据时,sleep至下次查询间隔的时间(ms)

double gdCmdVel[2];//初始化为0
bool gbCmdChanged=false;
std::vector<NodeOdom<short> > gvecEnc;//for fast recording
std::vector<NodeOdom<double> > gvecIMU;

ORB_SLAM2::System* gpSLAM;

CSerialPort::CSerialPort(void):lpBytes(NULL)//初始化缓存区
{
	m_hComm=INVALID_HANDLE_VALUE;m_hListenThread=INVALID_HANDLE_VALUE;//初始化无串口&无监听线程(句柄)
	pthread_mutex_init(&m_csCommSync,NULL);//初始化互斥锁，用完需关闭，采用默认属性
}
CSerialPort::~CSerialPort(void)
{
	if (mfout.is_open()) mfout.close();
	CloseListenTread();
	ClosePort();
	pthread_mutex_destroy(&m_csCommSync);
}
CSerialPort::CSerialPort(ros::NodeHandle nh)
{
  m_hComm=INVALID_HANDLE_VALUE;m_hListenThread=INVALID_HANDLE_VALUE;//初始化无串口&无监听线程(句柄)
  pthread_mutex_init(&m_csCommSync,NULL);//初始化互斥锁，用完需关闭，采用默认属性
  this->nh=nh;
}

bool CSerialPort::InitPort(const char* portNo,UINT baud,char parity,UINT databits,UINT stopsbits)
{
	pthread_mutex_lock(&m_csCommSync);//给互斥锁等待式加锁,多线程同步解决手段之一,另一种如进入临界段
	//打开串口
	m_hComm=open(portNo,O_RDWR|O_NOCTTY|O_NDELAY);
	//O_RDWR 读写方式打开；O_NOCTTY 不允许进程管理串口（不太理解，一般都选上）；O_NDELAY 非阻塞（默认为阻塞，打开后也可以使用fcntl()重新设置）
	if (m_hComm==INVALID_HANDLE_VALUE){
	  std::cerr<<redSTR"Error in openning the port!"<<std::endl;
	  pthread_mutex_unlock(&m_csCommSync);
	  return false;
	}
	//串口参数配置 
	struct termios options;// 串口配置结构体
	tcgetattr(m_hComm,&options);// 获取串口原来的参数设置
	bzero(&options,sizeof(options));
	
	options.c_cflag|=B115200|CLOCAL|CREAD;// 设置波特率，本地连接，接收使能
	options.c_cflag&=~CSIZE;// 屏蔽数据位or清零数据位
	options.c_cflag|=CS8;// 数据位为 8 ，CS7 for 7 
	options.c_cflag&=~CSTOPB;// 一位停止位， 两位停止为 |= CSTOPB
	options.c_cflag&=~PARENB;//无校验
	options.c_cc[VTIME]=0;//设置等待时间和最少的接收字符
	options.c_cc[VMIN]=0;
	
	tcflush(m_hComm,TCIOFLUSH);// 清除所有正在发生的I/O数据 or 清空所有缓存区
	if (tcsetattr(m_hComm,TCSANOW,&options)!=0){ //TCSANOW 立刻对值进行修改
	  std::cerr<<redSTR"Error in configuring the port!"<<std::endl;
	  pthread_mutex_unlock(&m_csCommSync);
	  return false;
	}
	pthread_mutex_unlock(&m_csCommSync);
	return true;
}
void CSerialPort::ClosePort()
{
	if(m_hComm!=INVALID_HANDLE_VALUE)//如果串口被打开,关闭它
	{
		if (close(m_hComm)!=0){
		  std::cerr<<redSTR"Error in closing the serial port"<<std::endl;
		}
		m_hComm=INVALID_HANDLE_VALUE;
	}
	if (lpBytes!=NULL)
	{
		delete[]lpBytes;
		lpBytes=NULL;
	}
}

bool CSerialPort::OpenListenThread()
{
	if (m_hListenThread!=INVALID_HANDLE_VALUE)//线程已开启返回开启失败
	{
		return false;
	}
	st_bExit=false;//不需退出
	pthread_t threadID;//线程ID
	/*pthread_attr_t attr;//线程属性
	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);//设置使用显示调度属性，而不是继承父线程的属性
	pthread_attr_setschedpolicy(&attr,SCHED_RR);//使用实时轮转调度策略,需要root权限
	sched_param param;
	param.sched_priority=1;//above nomral 0
	pthread_attr_setschedparam(&attr,&param);//设置优先级，高于普通线程，得在创建线程前实现
	m_hListenThread=pthread_create(&threadID,&attr,(void*(*)(void*))ListenThread,this);//创建线程，成功返回0
	pthread_attr_destroy(&attr);//用完属性结构指针，清理和回收属性结构指针*/
	m_hListenThread=pthread_create(&threadID,NULL,(void*(*)(void*))ListenThread,this);//创建线程，成功返回0
	if (m_hListenThread!=0)
	{
		return false;
	}
	nFlag=0;//监听起始态
	lpNo=0;//start pos for lpBytes
	return true;
}
void CSerialPort::CloseListenTread()
{	
	if (m_hListenThread!=INVALID_HANDLE_VALUE)
	{
		st_bExit = true;//通知线程退出
		usleep(1e4);//等待线程退出，未用时间服务函数时系统精度默认为10ms
		m_hListenThread=INVALID_HANDLE_VALUE;
	}
}

bool CSerialPort::ReadChar(char &cRecv)
{
	DWORD BytesRead=0;
	if(m_hComm==INVALID_HANDLE_VALUE)
	{
		return false;
	}
	pthread_mutex_lock(&m_csCommSync);
	BytesRead=read(m_hComm,&cRecv,1);//从缓冲区读取一个字节
	if (BytesRead==-1)
	{
		//DWORD dwError=GetLastError();//获取错误码,可用来找原因
		tcflush(m_hComm,TCIFLUSH);//清空输入缓冲区
		pthread_mutex_unlock(&m_csCommSync);
		return false;
	}
	pthread_mutex_unlock(&m_csCommSync);
	return (BytesRead==1);
}
bool CSerialPort::ReadData(BYTE* pData,unsigned int* nLen)
{
	DWORD BytesRead=0;
	if(m_hComm==INVALID_HANDLE_VALUE)
	{
		return false;
	}
	pthread_mutex_lock(&m_csCommSync);
	BytesRead=read(m_hComm,pData,*nLen);//从缓冲区读取一个字节
	if (BytesRead==-1)
	{
		tcflush(m_hComm,TCIFLUSH);//清空输入缓冲区
		pthread_mutex_unlock(&m_csCommSync);
		return false;
	}
	pthread_mutex_unlock(&m_csCommSync);
	*nLen=BytesRead;
	return true;
}
bool CSerialPort::WriteData(BYTE* pData,unsigned int nLen)
{
	DWORD BytesToSend=0;
	if(m_hComm==INVALID_HANDLE_VALUE)
	{
		return false;
	}
	pthread_mutex_lock(&m_csCommSync);
	BytesToSend==write(m_hComm,pData,nLen);//向缓冲区写入指定量的数据
	if (BytesToSend==-1)
	{
		tcflush(m_hComm,TCOFLUSH);
		pthread_mutex_unlock(&m_csCommSync);
		return false;
	}
	pthread_mutex_unlock(&m_csCommSync);
	return true;
}
UINT CSerialPort::GetBytesInCOM(int mode)//读写操作前,通过本函数清除硬件通讯错误和获取通讯设备当前状态
{
	//COMSTAT comstat;//COMSTAT结构体,记录通信设备的状态信息
	//::memset(&comstat,0,sizeof(COMSTAT));
	UINT BytesInQue=0;
	/*if (ClearCommError(m_hComm,&dwError,&comstat))
	{
		BytesInQue=comstat.cbInQue;//获取在输入缓冲区中的字节数
	}*/
	switch (mode){
	  case 0:
	    tcflush(m_hComm,TCIOFLUSH);
	    break;
	  case 1:
	    tcflush(m_hComm,TCIFLUSH);
	    break;
	  case 2:
	    tcflush(m_hComm,TCOFLUSH);
	    break;
	}
	
	return BytesInQue;
}

UINT CSerialPort::ListenThread(void* pParam)//静态函数只需声明处加上即可
{
	CSerialPort *pSerialPort=reinterpret_cast<CSerialPort*>(pParam);//可映射回的强制转换
	// 线程循环,轮询方式读串口数据	
	int BytesInQue=100;
	BYTE* buff = new BYTE[BytesInQue];
	while (!pSerialPort->st_bExit)
	{
		BytesInQue=100;
		pSerialPort->ReadData(buff,(unsigned int*)&BytesInQue);
		if (BytesInQue==0)//串口输入缓冲区中无数据,则挂起一会再查询
		{
			usleep(5000);//sleep 5 ms
			continue;
		}
		int numRecv=0;//自定义缓冲区内数据位置
		do
		{
		  pSerialPort->BufferBytes(buff[numRecv++]);//缓存自定义缓冲区数据字节
		}while(--BytesInQue);//最多缓存BytesInQue个字节
		/*if (pSerialPort->nFlag==2){//读霍尔信息不用分段读
		  pSerialPort->lpBytes[pSerialPort->lpNo++]='\0';
		  //std::cout<<pSerialPort->lpBytes<<std::endl;
		  for (int i=0;i<pSerialPort->lpNo;++i)
		    std::cout<<(int)pSerialPort->lpBytes[i]<<" ";
		  std::cout<<std::endl;
		  std::cout<<" "<<pSerialPort->lpNo<<std::endl;
		}*/
	}
	delete []buff;
	return 0;
}
//extern timespec g_time;
bool CSerialPort::BufferBytes(BYTE chr)
{
	using namespace std;
	double odomData[15];//2+13 or 2+9
	static pthread_mutex_t mutexBB;
	if (lpNo<nSizeOfBytes)//只管读完,总长度最大为数据长度(含2字节校验码)或hall而言是所预想的最坏情况
	{
	  lpBytes[lpNo++]=chr;
	}else{
	  std::cerr<<redSTR"exceed the length! nFlag is: "<<(int)nFlag<<whiteSTR<<std::endl;
	}
	static int tmp_pos=0;//for hall process
	switch (nFlag){
	  case 1://process imu data
	    if (lpNo==nSizeOfBytes){//check bytes for safety
	      double tmstamp=ros::Time::now().toSec();
//	      cout << "In: "<< nSizeOfBytes << endl;
	      //convert the mode & clean the buff
	      unsigned short timerTicks;
	      const int DATA_NUM=COMMAND==0x03?9:4+3*3;
	      short data[DATA_NUM];
	      lpNo=0;
	      if (nSizeOfBytes==7){
		if (lpBytes[0]==0x29) break;
		static int nEEPROM=0;
		if (lpBytes[0]==0x28){
		  static BYTE utmpEEPROM[3]={0x28,0x00,0xEE};
		  cout<<"EEPROM "<<(int)utmpEEPROM[2]<<"="<<(lpBytes[1]<<8)+lpBytes[2]<<endl;//EEPROM
		  if (++nEEPROM<5){
		    utmpEEPROM[2]+=2;
		    if (nEEPROM==3) utmpEEPROM[2]+=2;
		    else if (nEEPROM==4) utmpEEPROM[2]=0x7a;
		    WriteData(utmpEEPROM,3);
		    break;
		  }
		}
		nSizeOfBytes=COMMAND==0x03?23:31;
		break;//we omit initial checksum
	      }else{//Checksum for 31 continuous mode
		if (lpBytes[0]!=COMMAND){ 
		  cerr<<"Wrong Command! "<<(int)lpBytes[0]<<endl;break;
		}
		short checksum=((signed char)lpBytes[nSizeOfBytes-2]<<8)+lpBytes[nSizeOfBytes-1];
		timerTicks=(lpBytes[nSizeOfBytes-4]<<8)+lpBytes[nSizeOfBytes-3];
		short sum=lpBytes[0]+timerTicks;
		for (int i=0;i<DATA_NUM;++i){
		  data[i]=((signed char)lpBytes[i*2+1]<<8)+lpBytes[i*2+2];
		  sum+=data[i];
		}
		if (checksum!=sum){ cerr<<"Wrong Checksum! "<<checksum<<"; "<<sum<<endl;break;}
	      }

	      /*timespec time;
	      clock_gettime(CLOCK_REALTIME,&time);
	      cout<<"finish time(imu):"<<time.tv_sec<<"."<<time.tv_nsec/1000<<endl;
	      cout<<"used time(imu):"<<(time.tv_sec-g_time.tv_sec)+(time.tv_nsec-g_time.tv_nsec)/1E9<<endl;*/
	      //about 15ms on Jetson TX1
	      double dIMUData[DATA_NUM];//odomData
	      if (COMMAND==0x0C){//0 is w,123 is xyz,456 is MagFieldxyz,789 is Accelxyz,101112 is AngRatexyz
	        for (int i=0;i<DATA_NUM;++i){
		  if (i<4) dIMUData[i]=data[i]/8192.;
		  else if (i<7) dIMUData[i]=data[i]/16384.;//*2000/32768000;
		  else if (i<10) dIMUData[i]=data[i]*7./32768;//*7000/32768000;
		  else dIMUData[i]=data[i]/3276.8;//*10000/32768000;
//	 	  cout<<dIMUData[i]<<" ";
	        }
		if (mbstartRecord<2){
		  pthread_mutex_lock(&mutexBB);
	          vector<double>& vecTmp=mmOdomHis[1][m_tm_stamp];
		  for (int i=0;i<3;++i){//xyz
		    vecTmp.push_back(dIMUData[i+1]);
		  }
		  vecTmp.push_back(dIMUData[0]);//w
	          for (int i=4;i<DATA_NUM;++i){//then Mag,Acc,AngRate
		    vecTmp.push_back(dIMUData[i]);
	          }
	          pthread_mutex_unlock(&mutexBB);
		}else{
		  if (mbstartRecord==3){  
		    gvecIMU.push_back(NodeOdom<double>(dIMUData,DATA_NUM,tmstamp));
		    //writeToFile(dIMUData,DATA_NUM,tmstamp);
		  }
		}
	      }else{//012 is MagFieldxyz,345 is Accelxyz,678 is AngRatexyz
	        for (int i=0;i<DATA_NUM;++i){
		  if (i<3) dIMUData[i]=data[i]/16384.;//*2000/32768000;
		  else if (i<6) dIMUData[i]=data[i]*7./32768;//*7000/32768000;
		  else dIMUData[i]=data[i]/3276.8;//*10000/32768000;
//		  cout<<dIMUData[i]<<" ";
	        }
		if (mbstartRecord<2){
		  //we don't use IMU data in laser-like SLAM without stabQ
		}else{
		  if (mbstartRecord==3){
		    gvecIMU.push_back(NodeOdom<double>(dIMUData,DATA_NUM,tmstamp));
		    //writeToFile(dIMUData,DATA_NUM,tmstamp);
		  }
		}
	      }
//	      cout << endl;
	    }
	    break;
	  case 2://check hall sensor data
	    //hall sensor gives the data format as "command(end with'\r')""return string"'\r'
	    if (chr=='\r'){
		lpBytes[lpNo-1]='\0';
		if (strcmp((char*)lpBytes,"?BS")==0){//check the response
		  lpBytes[lpNo-1]='\r';
		  nFlag=3;//goto hall data process mode
		  tmp_pos=lpNo;
		}else if (strcmp((char*)lpBytes,ENC_AUTO_START)==0){
		  nFlag=4;
		  cout<<blueSTR"Enter Automatic Sending mode for Encoders!"<<whiteSTR<<endl;
		}else{
		  lpNo=0;
		  cout<<redSTR"Wrong Response: "<<(char*)lpBytes<<whiteSTR<<endl;
		}
	    }
	    break;
	  case 3://process hall sensor data in polling mode
	    if (chr=='\r'){
	      //about 8ms on Jetson TX1 when polled
	      double tmstamp=ros::Time::now().toSec();

	      for (int i=0;i<lpNo-1;++i){
		if (lpBytes[i]=='\r'||lpBytes[i]=='\0') cout<<"*";
		else cout<<lpBytes[i];
	      }
	      //cout<<" "<<tmp_pos<<" "<<setprecision(9)<<m_tm_stamp<<endl;//don't use default '\r'
	      cout<<"*2*"<<endl;

	      double dEncData[2];
	      lpBytes[lpNo-1]='\0';
	      string str((const char*)lpBytes);
	      str=str.substr(tmp_pos);
	      int split_pos=str.find(':');
	      if (split_pos==string::npos){
		dEncData[1]=dEncData[0]=0;
		cout<<redSTR"There's a problem with hall sensor communication!"<<endl;
	      }else{
		dEncData[1]=atof(str.substr(3,split_pos-3).c_str());//the left wheel is after :
		dEncData[0]=atof(str.substr(split_pos+1).c_str());
	      }
	      if (mbstartRecord<2){//mode 0/1 for laser-like SLAM and control system, the later mode means recording the odometry data
	        vector<double>& vecTmp=mmOdomHis[0][m_tm_stamp];
 	        vecTmp.push_back(dEncData[0]);vecTmp.push_back(dEncData[1]);//vl then vr
	        //using namespace chrono;steady_clock::time_point tp1=steady_clock::now();
	        pthread_mutex_lock(&mutexBB);//commonly operate mmOdomHis[1]
	        for (map<double,vector<double>>::iterator it=mmOdomHis[0].begin(),itend=mmOdomHis[0].end();it!=itend;++it){
		  double tmHis=it->first;
		  odomData[0]=it->second[0];odomData[1]=it->second[1];
		  if (mmOdomHis[1].count(tmHis)){//if imu&&encoder both are recorded, then make odomData
		    for (int i=2;i<15;++i)
		      odomData[i]=mmOdomHis[1][tmHis][i-2];
		    publishTFOdom(odomData,tmHis,1);//process odomData[6]
		    cout <<setprecision(9)<<m_tm_stamp<< endl;
		    if (mbstartRecord==1) writeToFile(odomData,15);//mode 1: record enc&IMU data at the same file
		    if (tmHis==m_tm_stamp) mmOdomHis[0].clear();
		  }else{
		    if (tmHis<m_tm_stamp){
		      publishTFOdom(odomData,tmHis,0);//process odomData[2]
		    }else{
		      publishTFOdom(odomData,tmHis,2);//just publish without updating
		      mmOdomHis[0].erase(mmOdomHis[0].begin(),--mmOdomHis[0].end());
		    }
		  }
	        }
	        mmOdomHis[1].clear();
	        pthread_mutex_unlock(&mutexBB);
	        //cout<<"publishTF used time: "<<duration_cast<duration<double>>(steady_clock::now()-tp1).count()<<endl;
	      }else{//mode 2/3 for VIEORBSLAM and its control system, the later mode means recording the odometry data
		if (mbstartRecord==3){
		  short dataTmp[2]{dEncData[0],dEncData[1]};
		  gvecEnc.push_back(NodeOdom<short>(dataTmp,2,tmstamp));
		  //writeToFile(dEncData,2,tmstamp);
		}
	      }
	      
	      nFlag=2;//go back to hall data check mode
	      lpNo=0;
	      if (gbCmdChanged){//now changed cmd_vel can be sended to HBL2360
	        char pCommand[15];
	        sprintf(pCommand,"!VAR 1 %d\r",(int)gdCmdVel[0]);
	        WriteData((BYTE*)pCommand,strlen(pCommand));
	        sprintf(pCommand,"!VAR 2 %d\r",(int)gdCmdVel[1]);
	        WriteData((BYTE*)pCommand,strlen(pCommand));
		cout<<redSTR"WriteData0"<<whiteSTR<<endl;
	      }
	    }
	    break;
	  case 4://process hall sensor data in continuous mode
	    if (chr=='\r'){
	      double tmstamp=ros::Time::now().toSec();
	      //cout << "In: "<< nSizeOfBytes << endl;
	      if (lpNo>=6&&lpBytes[0]=='B'&&lpBytes[1]=='S'&&lpBytes[2]=='='){
		/*for (int i=0;i<lpNo-1;++i){
	   	  cout<<lpBytes[i];
	        }
	        cout<<endl;//don't use default '\r'*/

		double dEncData[2];
	        lpBytes[lpNo-1]='\0';
	        string str((const char*)lpBytes);
	        int split_pos=str.find(':');
	        if (split_pos==string::npos){
		  dEncData[1]=dEncData[0]=0;
		  cout<<redSTR"There's a problem with hall sensor communication!"<<endl;
	        }else{
		  dEncData[1]=atof(str.substr(3,split_pos-3).c_str());//the left wheel is after :
		  dEncData[0]=atof(str.substr(split_pos+1).c_str());
	        }
	        if (mbstartRecord<2){//mode 0/1 for laser-like SLAM and control system, the later mode means recording the odometry data
		  //we don't design laser-like SLAM in continuous mode for timestamp problem
		  pthread_mutex_lock(&mutexBB);//commonly operate mmOdomHis[1]
		  mmOdomHis[1].clear();
	          pthread_mutex_unlock(&mutexBB);
	        }else{//mode 2/3 for VIEORBSLAM and its control system, the later mode means recording the odometry data
		  if (mbstartRecord==3){
		    short dataTmp[2]{dEncData[0],dEncData[1]};
		    gvecEnc.push_back(NodeOdom<short>(dataTmp,2,tmstamp));
		    //writeToFile(dEncData,2,tmstamp);
		  }
		  gpSLAM->TrackOdom(tmstamp,dEncData,(char)ORB_SLAM2::System::ENCODER);//nTotalNum=2
	        }
	      }else{
                lpBytes[lpNo-1]='\0';
		if (strcmp((char*)lpBytes,"# C")==0){
		  nFlag=2;//go back to hall data check mode
		  cout<<blueSTR"Back to Pulling mode for Encoders!"<<whiteSTR<<endl;
	      	}
	      }

	      lpNo=0;
	      if (gbCmdChanged){
		gbCmdChanged=false;
	        char pCommand[15];
	        sprintf(pCommand,"!VAR 1 %d\r",(int)gdCmdVel[0]);
	        WriteData((BYTE*)pCommand,strlen(pCommand));
	        sprintf(pCommand,"!VAR 2 %d\r",(int)gdCmdVel[1]);
	        WriteData((BYTE*)pCommand,strlen(pCommand));
		cout<<redSTR"WriteData1"<<whiteSTR<<endl;
	      }
	    }
	}

	return true;
}
void CSerialPort::writeToFile(double data[],int num_data,const double &tmstamp){
	using namespace std;
	/*if (!mfout.is_open()){
	  cerr<<redSTR"open error! "<<m_tm_stamp<<whiteSTR<<endl;
	  return;
	}*/
  	mfout<<setprecision(6)<<(tmstamp==-1?m_tm_stamp:tmstamp)<<" "<<setprecision(3);
	for (int i=0;i<num_data-1;++i)
	  mfout<<data[i]<<" ";
	mfout<<data[num_data-1]<<endl;
}
void CSerialPort::publishTFOdom(double* odomdata,double tm_stamp,char mode){
  static const int ppr=400,datatime=10;//pulse per revolution,the span of the hall sensor data
  static const double wheelradius=0.105,carradius=0.280,vscaleforhall=2.0/ppr*M_PI*wheelradius/datatime;//radius for the driving wheels(m),the half distance between two driving wheels
  static const double wscaleforhall=vscaleforhall/carradius;
  static const double xo_base[3]={0,0,0};//the translational difference from the centre of two driving wheels to base_link frame(the same)
  static const double xoi_o[3]={0.024,0.324,-0.461};
  static int estimate_mode=0;
  static double lasttime=-1,st_vtmpt,st_wtmpt;
  double vtmpt,wtmpt;
  double deltat=0;
  double vtmp,wtmp,v[2]={odomdata[0],odomdata[1]},arrq[4]={odomdata[2],odomdata[3],odomdata[4],odomdata[5]};
  static double st_xt[3]={0};//x y theta
  double xt[3];//temporary pose
  
  Eigen::Isometry3d To_base(Eigen::Isometry3d::Identity()),Tbase_o;
  static Eigen::Isometry3d Toi_o(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1)));
  static Eigen::Isometry3d To0_wi(Eigen::Isometry3d::Identity());//transformation/change from  the first odometry frame(centre of two wheels) to IMU internal frame(ros version meaning,not slam or coordinate meaning)
  To_base.pretranslate(Eigen::Vector3d(xo_base));Tbase_o=To_base.inverse();
  
  //get the Tw_base/Todom_baselink
  if (lasttime<0){
    //xt[2]=xt[1]=xt[0]=0;
    if (mode==1){//if you may use mode==1, this must be true at first time
      Toi_o.prerotate(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(1,0,0)));
      //cout<<Toi_o.rotation().eulerAngles(2,1,0).transpose()<<endl;
      Toi_o.pretranslate(Eigen::Vector3d(xoi_o));
      
      To0_wi.prerotate(Eigen::Quaterniond(arrq));//actually use quaterniond.inverse() has some numerical error, u'd better use conjugate()!
      To0_wi=(To0_wi*Toi_o).inverse();//Tw_wi=(Twi_oi*Toi_o)^(-1);
      //std::cout<<arrq[0]<<" "<<arrq[1]<<" "<<arrq[2]<<" "<<arrq[3]<<std::endl;
      //To0_wi=To0_wi.inverse();//Toi0_wi
    }
    vtmpt=(v[0]+v[1])/2*vscaleforhall;//v1=v-vw;v2=v+vw => v=(v1+v2)/2
    wtmpt=(v[1]-v[0])/2*wscaleforhall;// => w=(v2-v1)/2/r
  }else{
    double theta_tmp;
    if (mode==1){
      Eigen::Isometry3d To0_o(Toi_o);//suppose w means o0 not cr0
      To0_o.prerotate(Eigen::Quaterniond(arrq));//Tw_o=Tw_oi*Toi_o=Tw_wi*Twi_oi*Toi_o;here it's just Twi_o
      To0_o=To0_wi*To0_o;//Tw_o=Tw_wi*Twi_o;end suppose
      //Eigen::Isometry3d To0_o(Eigen::Isometry3d::Identity()),Twi_oi(Eigen::Quaterniond(arrq).toRotationMatrix());
      //To0_o=To0_wi*Twi_oi;
      Eigen::AngleAxisd rotatvec_tmp(To0_o.rotation());
      theta_tmp=(rotatvec_tmp.angle()*rotatvec_tmp.axis()).dot(Eigen::Vector3d(0,0,1));
      if (theta_tmp>M_PI) theta_tmp-=2*M_PI;
      else if (theta_tmp<=-M_PI) theta_tmp+=2*M_PI;
      //double theta2=To0_o.rotation().eulerAngles(2,1,0)[0];
      std::cout<<std::setprecision(3)<<theta_tmp*180/M_PI<<" degrees"<<std::endl;
    }
    ///hall sensor data(counts during 10s)/400*pi*2*radius(mm)/1000/10=velocity
    deltat=tm_stamp-lasttime;
    vtmpt=(v[0]+v[1])/2*vscaleforhall;wtmpt=(v[1]-v[0])/2*wscaleforhall;
    vtmp=(vtmpt+st_vtmpt)/2;//v1=v-vw;v2=v+vw => v=(v1+v2)/2
    wtmp=(wtmpt+st_wtmpt)/2;// => w=(v2-v1)/2/r
    if (mode==1){
      double delta_rect=theta_tmp-st_xt[2];
      if (delta_rect>M_PI) delta_rect-=2*M_PI;
      else if (delta_rect<=-M_PI) delta_rect+=2*M_PI;
      //Complementary Filter
      const double k_comple=0.008;//it's good between 0.006~0.01
      /*//Kalman Filter/KF
      double vari_rt=vari_wt*deltat*deltat;
      //vari_rt=vari_rt2;var_qt=var_qt2;//use const variances
      double kt_w=(vari_tht+vari_rt)/(vari_tht+var_qt+vari_rt)/deltat;//Kt/deltat*/
      switch (estimate_mode){
	case 0:
	  //Complementary Filter
	  wtmp+=k_comple*delta_rect;
	  break;
	/*case 1:
	  //Kalman Filter/KF
	  if (kt_w>1) kt_w=1;
	  wtmp+=delta_rect*kt_w;
	  vari_tht=(vari_tht+vari_rt)*var_qt/(vari_tht+var_qt+vari_rt);*/
      }
    }
    if (wtmp!=0){
      double v_div_w=vtmp/wtmp,dtheta=wtmp*deltat;
      xt[0]=st_xt[0]+v_div_w*(-sin(st_xt[2])+sin(st_xt[2]+dtheta));//x'=x+-v/w*sinth+v/w*sin(th+w*dt) =v*cos(th)*dt;
      xt[1]=st_xt[1]+v_div_w*(cos(st_xt[2])-cos(st_xt[2]+dtheta));//y'=y+v/w*costh-v/w*cos(th+w*dt) =v*sin(th)*dt;
      xt[2]=st_xt[2]+dtheta;//th(eta)'=th+w*dt;
      if (xt[2]>M_PI) xt[2]-=2*M_PI;
      else if (xt[2]<=-M_PI) xt[2]+=2*M_PI;
    }else{
      xt[0]=st_xt[0]+vtmp*cos(st_xt[2])*deltat;
      xt[1]=st_xt[1]+vtmp*sin(st_xt[2])*deltat;
      xt[2]=st_xt[2];
    }
    //cout<<dtimetmp<<" "<<theta_tmp<<" "<<xt[2]<<" "<<"! "<<kt_w<<" "<<rotatvec_tmp.axis().transpose()<<endl;
  }
  Eigen::Quaterniond q(Eigen::AngleAxisd(xt[2],Eigen::Vector3d(0,0,1)));
  //thie o means the encoder odom frame not the ros odom frame(base_link frame)
  Eigen::Isometry3d Two(Eigen::Isometry3d::Identity()),Tw_base;//here world means the ros odom frame, not the crystal but the first frame of the base frame
  Two.rotate(q);
  Two.pretranslate(Eigen::Vector3d(xt[0],xt[1],0));
  Tw_base=Tbase_o*Two*To_base;//though Tocr.inverse() is independent of the shape of the trajectory!
  //first lasttime initialization must be mode==1
  if (mode==1||mode<2&&lasttime>=0||mode==0&&COMMAND==0x03){//2 just publish, don't update st_xt
    for (int i=0;i<3;++i)
      st_xt[i]=xt[i];
    lasttime=tm_stamp;
    st_wtmpt=wtmpt;st_vtmpt=vtmpt;
  }
  //cout<<setprecision(9)<<Twcr(0,3)<<" "<<Twcr(1,3)<<" "<<Twcr(2,3)<<" "<<0.0<<" "<<0.0<<" "<<0.0<<" "<<1.0<<endl;
  
  //publish i
  //here o means ros odom frame
  //initialize of the publisher of the odom topic "/odom"
  static ros::Publisher odom_pub=nh.advertise<nav_msgs::Odometry>("/odom",50);
  tf::TransformBroadcaster odom_broadcaster;
  nav_msgs::Odometry odom;
  for (int i=0;i<36;++i) 
      odom.pose.covariance[i]=odom.twist.covariance[i]=0.01;
  
  //publish the To_b to the "/tf" topic
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp=ros::Time(tm_stamp);
  odom_trans.header.frame_id="odom";
  odom_trans.child_frame_id="base_link";
  odom_trans.transform.translation.x=Tw_base(0,3);
  odom_trans.transform.translation.y=Tw_base(1,3);
  odom_trans.transform.translation.z=Tw_base(2,3);//0
  Eigen::Quaterniond qwb(Tw_base.rotation());
  odom_trans.transform.rotation.x=qwb.x();//qx=sin(t/2)*nx;
  odom_trans.transform.rotation.y=qwb.y();
  odom_trans.transform.rotation.z=qwb.z();
  odom_trans.transform.rotation.w=qwb.w();
  
  odom_broadcaster.sendTransform(odom_trans);
  
  //publish the To_b && deltaTo_b/deltat to the "/odom" topic
  if (deltat!=0){
    odom.header.stamp=odom_trans.header.stamp;
    //set the position
    odom.header.frame_id="odom";
    odom.child_frame_id="base_link";
    odom.pose.pose.position.x=odom_trans.transform.translation.x;
    odom.pose.pose.position.y=odom_trans.transform.translation.y;
    odom.pose.pose.position.z=odom_trans.transform.translation.z;
    odom.pose.pose.orientation=odom_trans.transform.rotation;
    //set the velocity
    //get the Tbt-1_o=To_bt-1.inverse()=>deltaTo_b=Tbt-1_bt=Tbt-1_o*To_bt
    static Eigen::Isometry3d Tw_base_last=Eigen::Isometry3d::Identity();
    Eigen::Isometry3d Tlb_b=Tw_base_last.inverse()*Tw_base;//T last baselink_baselink now
    if (mode<2)
      Tw_base_last=Tw_base;
    //printf("%f\n",dt);
    odom.twist.twist.linear.x=Tlb_b(0,3)/deltat;
    odom.twist.twist.linear.y=Tlb_b(1,3)/deltat;
    odom.twist.twist.linear.z=Tlb_b(2,3)/deltat;//0
    Eigen::AngleAxisd angaxis(Tlb_b.rotation());
    Eigen::Vector3d rotatvec=angaxis.angle()*angaxis.axis();
    odom.twist.twist.angular.x=rotatvec[0]/deltat;//0
    odom.twist.twist.angular.y=rotatvec[1]/deltat;//0
    odom.twist.twist.angular.z=rotatvec[2]/deltat;

    odom_pub.publish(odom);
  }
}
