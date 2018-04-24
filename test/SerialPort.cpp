#include "SerialPort.h"
//#include <cwchar>
#include <iostream>
/*//make trajectory
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
//publish ros topic
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>*/

extern unsigned long gnHz,gnHzEnc;

bool CSerialPort::st_bExit=false;//静态变量必须初始化
double CSerialPort::m_tm_stamp=0;
unsigned char CSerialPort::mbstartRecord=0;
const UINT SLEEP_TIME_INTERVAL=10;//串口无数据时,sleep至下次查询间隔的时间(ms)

double gdCmdVel[2];//初始化为0
bool gbCmdChanged=false;

CSerialPort::CSerialPort(void):lpBytes(NULL)//初始化缓存区
{
	m_hComm=INVALID_HANDLE_VALUE;m_hListenThread=INVALID_HANDLE_VALUE;//初始化无串口&无监听线程(句柄)
	pthread_mutex_init(&m_csCommSync,NULL);//初始化互斥锁，用完需关闭，采用默认属性
}
CSerialPort::~CSerialPort(void)
{
	if (mfout.is_open()) mfout.close();
	CloseListenThread();
	ClosePort();
	pthread_mutex_destroy(&m_csCommSync);
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
void CSerialPort::CloseListenThread()
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
extern timespec g_time;
bool CSerialPort::BufferBytes(BYTE chr)
{
	using namespace std;
	if (lpNo<nSizeOfBytes)//只管读完,总长度最大为数据长度(含2字节校验码)或hall而言是所预想的最坏情况
	{
	  lpBytes[lpNo++]=chr;
	}else{
	  std::cerr<<"exceed the length! nFlag is: "<<(int)nFlag<<std::endl;
	}
	static int tmp_pos=0;//for hall process
	switch (nFlag){
	  case 1://process imu datai
	    if (lpNo==nSizeOfBytes){//check bytes for safety
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

	      ++gnHz;
	      timespec time;
	      clock_gettime(CLOCK_REALTIME,&time);
//	      cout<<"finish time(imu):"<<time.tv_sec<<"."<<time.tv_nsec/1000<<endl;
//	      cout<<"used time(imu):"<<(time.tv_sec-g_time.tv_sec)+(time.tv_nsec-g_time.tv_nsec)/1E9<<endl;
	      double dIMUData[DATA_NUM];//odomData
	      if (COMMAND==0x0C){//0 is w,123 is xyz,456 is MagFieldxyz,789 is Accelxyz,101112 is AngRatexyz
	        for (int i=0;i<DATA_NUM;++i){
		  if (i<4) dIMUData[i]=data[i]/8192.;
		  else if (i<7) dIMUData[i]=data[i]/16384.;//*2000/32768000;
		  else if (i<10) dIMUData[i]=data[i]*7./32768;//*7000/32768000;
		  else dIMUData[i]=data[i]/3276.8;//*10000/32768000;
//		  cout<<dIMUData[i]<<" ";
	        }
	      }else{//012 is MagFieldxyz,345 is Accelxyz,678 is AngRatexyz
	        for (int i=0;i<DATA_NUM;++i){
	  	  if (i<3) dIMUData[i]=data[i]/16384.;//*2000/32768000;
		  else if (i<6) dIMUData[i]=data[i]*7./32768;//*7000/32768000;
		  else dIMUData[i]=data[i]/3276.8;//*10000/32768000;
//		  cout<<dIMUData[i]<<" ";
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
		  nFlag=3;//goto hall data process mode
		  tmp_pos=lpNo;
		}else if (strcmp((char*)lpBytes,ENC_AUTO_START)==0){
		  nFlag=4;
		  cout<<"Enter Automatic Sending mode for Encoders!"<<endl;
		}else{
		  lpNo=0;
		  cout<<"Wrong Response: "<<(char*)lpBytes<<endl;
		}
	    }
	    break;
	  case 3://process hall sensor data
	    if (chr=='\r'){
	      ++gnHzEnc;
	      timespec time;
	      clock_gettime(CLOCK_REALTIME,&time);
	      cout<<"finish time(hall):"<<time.tv_sec<<"."<<time.tv_nsec/1000<<endl;
	      cout<<"used time(hall):"<<(time.tv_sec-g_time.tv_sec)+(time.tv_nsec-g_time.tv_nsec)/1E9<<endl;
	      cout << "In: "<< nSizeOfBytes << endl;
	      for (int i=tmp_pos;i<lpNo-1;++i){
		cout<<lpBytes[i];
	      }
	      cout<<endl;//don't use default '\r'

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
	  case 4:
	    if (chr=='\r'){
	      //cout << "In: "<< nSizeOfBytes << endl;
	      if (lpNo>=6&&lpBytes[0]=='B'&&lpBytes[1]=='S'&&lpBytes[2]=='='){
	        ++gnHzEnc;
	        for (int i=0;i<lpNo-1;++i){
	   	  ;//cout<<lpBytes[i];
	        }
	        //cout<<endl;//don't use default '\r'
		
	      }else{
                lpBytes[lpNo-1]='\0';
		if (strcmp((char*)lpBytes,"# C")==0){
		  nFlag=2;//go back to hall data check mode
		  cout<<"Back to Pulling mode for Encoders!"<<endl;
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
