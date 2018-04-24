#ifndef SERIALPORT_H_
#define SERIALPORT_H_

#include <fcntl.h>//文件控制定义
#include <termios.h>//POSIX规范下的终端控制定义
#include <cstring>//使用bzero清0
#include <unistd.h>//unix 标准函数,如sleep()

#include <pthread.h>
#include <sched.h>//使用优先级参数

#include <string>
#include <iomanip>
#include <fstream>//write a file without useless open/close
#include <chrono>
//#include <time.h>//测量时间差
/*#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>*/
#include <ros/ros.h>//publish ros topic
#include <map>
#include <vector>//for fast recording

#include "System.h"

const int INVALID_HANDLE_VALUE=-1;
typedef int HANDLE;
typedef unsigned int UINT;
typedef unsigned char BYTE;
typedef unsigned long DWORD;
//zzh defined color cout, must after include opencv2
#define redSTR "\033[31m"
#define brightredSTR "\033[31;1m"
#define greenSTR "\e[32m"
#define brightgreenSTR "\e[32;1m"
#define blueSTR "\e[34m"
#define brightblueSTR "\e[34;1m"
#define yellowSTR "\e[33;1m"
#define brownSTR "\e[33m"
#define azureSTR "\e[36;1m"
#define whiteSTR "\e[0m"
const std::string gstrHome="";//we use "rosbag record topicname" to record images
//const std::string gstrHome="/home/ubuntu";
//const std::string gstrHome="/media/ubuntu/SD Root";
const std::string gstrORBFile="/home/ubuntu";//place for encoder/IMU data & input files
//const BYTE COMMAND=0x0C;//31 bytes
const BYTE COMMAND=0x03;//23 bytes
#define ENC_AUTO_START "# 5"//5ms

extern ORB_SLAM2::System* gpSLAM;
extern double gdCmdVel[2];//0 for y1, 1 for y2(right wheel target speed)
extern bool gbCmdChanged;
template<class DATATYPE>
struct NodeOdom{
  double tmstamp;
  DATATYPE* data;int size;
  NodeOdom(DATATYPE* pdata,int num,double tm){
    data=new DATATYPE[num];tmstamp=tm;size=num;
    for (int i=0;i<size;++i){data[i]=pdata[i];}
  }
  ~NodeOdom(){delete[] data;}
  NodeOdom& operator=(const NodeOdom& node){
    if (node.size!=size) return *this;
    tmstamp=node.tmstamp;
    for (int i=0;i<size;++i){data[i]=node.data[i];}
  }
  NodeOdom(const NodeOdom& node){
    data=new DATATYPE[node.size];
    tmstamp=node.tmstamp;size=node.size;
    for (int i=0;i<size;++i){data[i]=node.data[i];}
  }
};
extern std::vector<NodeOdom<short>> gvecEnc;//for fast recording
extern std::vector<NodeOdom<double>> gvecIMU;

class CSerialPort//操作成功返回非0；否则为0
{
private:
	HANDLE  m_hComm;//串口句柄
	volatile HANDLE m_hListenThread;//线程句柄
	static bool st_bExit;//线程退出标志变量
	//unsigned short tableCRC16[256];//CRC单字节值表
	static UINT ListenThread(void* pParam);
	/*串口监听线程:监听来自串口的数据和信息,在类中必须为静态,否则会传递第二个参数this指针
	pParam线程参数;return:UINT WINAPI线程返回值*/
	pthread_mutex_t m_csCommSync;//互斥锁，用来保护临界资源即当前串口缓冲区同时只被一个线程访问

	std::ofstream mfout;

public:
	static std::map<double,std::vector<double>> mmOdomHis[2];//pair<tm_stamp,st_odom[n]> n=2 for hall[0] n=13 for imu[1], for polling mode
	static unsigned char mbstartRecord;
	static double m_tm_stamp;//for polling mode
	BYTE *lpBytes;unsigned long lpNo;
	BYTE nFlag;unsigned long nSizeOfBytes;
	//数据缓冲区lpBytes(初始化为NULL,关闭时自动delete)及当前位置lpNo及状态标志nFlag及缓冲区大小
	//nFlag(开启监听时为0):(0为监听起始态,1为获取格式,2为刚获取完格式,3为正常读取态)
	ros::NodeHandle nh;//ros node handle, for publishing topics

	CSerialPort(void);
	CSerialPort(ros::NodeHandle nh);
	~CSerialPort(void);
	bool InitPort(const char* portNo,UINT baud=B115200,char parity='N',UINT databits=8,UINT stopsbits=1);
	/*初始化串口函数:portNo串口编号,默认值即COM1,尽量不要大于9;baud波特率;parity奇偶校验,'Y'表示需要;databits数据位
	stopsbits停止位;dwCommEvents默认为EV_RXCHAR,即只要收发任意一个字符,则产生一个事件;	
	PS:最先调用本函数进行串口初始化；若需自行设置详细的DCB参数,可使用重载函数；串口类析构时会自动关闭串口*/
	bool OpenListenThread();//开启监听线程:对串口数据监听,并将接收的数据打印到屏幕;
	void CloseListenTread();//关闭监听线程
	bool ReadChar(char& cRecv);//读取串口接收缓冲区的数据;cRecv存放数据的变量
	bool ReadData(BYTE* pData,unsigned int* nLen);
	bool WriteData(BYTE *pData,unsigned int nLen);//pData指向需要写入串口的数据缓冲区;nLen需要写入的数据长度
	//bool WriteDataCRC16(BYTE *pData,unsigned int nLen);//长度不包含CRC项
	//bool CheckCRC16(BYTE *pData,unsigned int nLen);//长度不包含CRC项,但缓存须留出2字节给CRC码
	UINT GetBytesInCOM(int mode=0);//获取串口缓冲区中的字节数,读写操作前最好加上这句来清除硬件错误,1为输入，2为输出，0两者都清除
	void ClosePort();
	//void MakeTableCRC16(bool LowBitFirst=1);//1表示低位先传送版本;默认都高字节先传送

	bool GetbListen()
	{
		return m_hListenThread!=INVALID_HANDLE_VALUE;
	}
	bool GetbLink()
	{
		return m_hComm!=INVALID_HANDLE_VALUE;
	}
	bool BufferBytes(BYTE chr);//将字节字符放入缓冲区
	void Openfout(const std::string &filename=""){
	  if (filename!="") mfout.open(filename,std::ios_base::out);
	  else mfout.open(gstrORBFile+"/test_save/odometrysensor.txt");
	  if (mbstartRecord==1)
	    mfout<<"# odometry data\n# file: 'rgbd_dataset_zzh.bag'\n# timestamp vl vr (qx qy qz qw) (magxyz axyz wxyz)"<<std::endl;
	  else
	    mfout<<"# Enc/IMU data\n# file: 'rgbd_dataset_zzh.bag'\n# timestamp [vl vr] / [(qw qx qy qz) magxyz axyz wxyz]"<<std::endl;//please notice different order in q
	  mfout<<std::fixed;
	}
	void Closefout(){mfout.close();}
	void writeToFile(double data[],int num_data,const double &tmstamp=-1);
	void publishTFOdom(double* odomdata,double tm_stamp,char mode);//mode==0 means only encoder estimation;1 means enc&IMU Complementary Filter fusion estimation
};

#endif //SERIALPORT_H_
