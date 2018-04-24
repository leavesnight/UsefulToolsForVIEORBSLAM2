#include <iostream>
#include "SerialPort.h"
#include <iomanip>
#include <thread>
#include <mutex>

using namespace std;

timespec g_time;
unsigned long gnHz=0,gnHzEnc;

CSerialPort sport,sport_enc;

mutex lock_cmd;
bool mb_speed_unlock=false;
volatile int last_cmd_time=-1;

void baseControlCallback(double x_linear,double z_angular){
  int x1=x_linear*1200,x2=z_angular*600;
  last_cmd_time=time(NULL);
  sport_enc.GetBytesInCOM(2);
  lock_cmd.lock();
  gdCmdVel[0]=x1+x2;gdCmdVel[1]=x1-x2;
  mb_speed_unlock=true;
  gbCmdChanged=true;
  lock_cmd.unlock();
}

bool gbRun=true;

void Run(){
  while (gbRun){
    baseControlCallback(0.3,0);
    cout<<gdCmdVel[0]<<" "<<gdCmdVel[1]<<endl;
    usleep(1e5);
  }
  lock_cmd.lock();
  gdCmdVel[0]=gdCmdVel[1]=0;mb_speed_unlock=false;
  gbCmdChanged=true;
  lock_cmd.unlock();
  cout<<gdCmdVel[0]<<" "<<gdCmdVel[1]<<endl;
}

int main(int argc,char** argv)
{
	//sport.InitPort("/dev/ttyACM0");
	sport.InitPort("/dev/ttyUSB0");
	sport_enc.InitPort("/dev/ttyACM0");
	//if (!sport.OpenListenThread()) {
	//	cerr << "Error in creating the thread!" << endl;
	//}
	
	if (!sport.OpenListenThread())
	  cerr<<redSTR"Error in creating the thread_imu!"<<endl;
	if (!sport_enc.OpenListenThread())
	  cerr<<redSTR"Error in creating the thread_enc!"<<endl;
	CSerialPort* pSerialPort=&sport;
	
	sport.nFlag=1;//进入IMU读取状态1
	sport_enc.nFlag=2;//进入转速读取状态2
	pSerialPort->nSizeOfBytes=31;//though we can read only ahead 9 bytes to speed up
	pSerialPort->lpBytes=new BYTE[pSerialPort->nSizeOfBytes];
	sport_enc.nSizeOfBytes=50;//21;//return at most "!VAR 1 1200\r+\r"*2=>28,please add the '\0'
      	//at most "?BS \r" and return "?BS \r""response""\r"=>21
      	//use total size for the the reply of the cmd_vel at the time of the reply for the next encoder!
	sport_enc.lpBytes=new BYTE[sport_enc.nSizeOfBytes];
	
	cout<<fixed<<setprecision(6);
	//cout<<(int)static_cast<signed char>(BYTE(-10))<<endl;

	//close continuous mode & clear buff
	pSerialPort->nSizeOfBytes=7;
	BYTE utmp[3]={0x10,0x00,0x00};
	pSerialPort->lpNo=0;
	pSerialPort->WriteData(utmp,3);
	sport_enc.lpNo=0;
	sport_enc.WriteData((BYTE*)"# C\r",strlen("# C\r"));//enc
	cin.get();

	//write & read EEPROM
	cout<<"Write EEPROM then input 4+1 numbers or with 0 to jump this step! or 2 to just read EEPROM!"<<endl;
	int byteTmp[5];
	cin>>byteTmp[0];cin.get();
	if (byteTmp[0]!=0){
	  if (byteTmp[0]!=2){
	    cin>>byteTmp[1]>>byteTmp[2]>>byteTmp[3]>>byteTmp[4];
	    cin.get();
	    pSerialPort->nSizeOfBytes=7;
  	    BYTE utmpEEPROM[7]={0x29,0x71,0x00,0xEE,0x00,0,0xAA};
	    double dSleep=1000;
	    for (int i=0;i<4;++i){
	      if (byteTmp[i]<256){ utmpEEPROM[5]=byteTmp[i];utmpEEPROM[4]=0x00;}
	      else{ utmpEEPROM[4]=byteTmp[i]>>8;utmpEEPROM[5]=byteTmp[i]&0xFF;}
	      pSerialPort->lpNo=0;
	      pSerialPort->WriteData(utmpEEPROM,7);
	      usleep(dSleep*1000);
	      utmpEEPROM[3]+=2;if (i==2) utmpEEPROM[3]+=2;
	    }
	    utmpEEPROM[4]=0x00;utmpEEPROM[5]=byteTmp[4];
	    utmpEEPROM[2]=0x00;utmpEEPROM[3]=0x7a;
	    pSerialPort->lpNo=0;
	    pSerialPort->WriteData(utmpEEPROM,7);
	    usleep(dSleep*1000);
	  }
	  pSerialPort->nSizeOfBytes=7;
  	  BYTE utmpEEPROM[3]={0x28,0x00,0xEE};
	  //int nTmp;cin>>nTmp;
	  //utmpEEPROM[2]=nTmp;cin.get();
	  pSerialPort->lpNo=0;
	  pSerialPort->WriteData(utmpEEPROM,3);
	  cin.get();
	}

	//set continuous mode for IMU
	utmp[2]=COMMAND;
	pSerialPort->lpNo=0;
	pSerialPort->nSizeOfBytes=7;

	//set encoder continuous mode(stop & add command to the buffer)
	sport_enc.lpNo=0;
	sport_enc.WriteData((BYTE*)"#\r",strlen("#\r"));
	sport_enc.WriteData((BYTE*)"?BS\r",strlen("?BS\r"));

	//start continuous mode
	pSerialPort->WriteData(utmp,3);//imu
	sport_enc.WriteData((BYTE*)ENC_AUTO_START"\r",strlen(ENC_AUTO_START"\r"));//enc
	
	timespec g_inittime;
	clock_gettime(CLOCK_REALTIME,&g_inittime);
	int nExecTimes=0;
	
	thread* pCtrlThread=NULL;
	pCtrlThread=new thread(&Run);
	while (1){
//		if (nExecTimes>5) break;
//		++nExecTimes;
	      	timespec tsTime;
	      	clock_gettime(CLOCK_REALTIME,&tsTime);
		double deltaTimeUsed=(tsTime.tv_sec-g_inittime.tv_sec)+(tsTime.tv_nsec-g_inittime.tv_nsec)/1E9;
		if (deltaTimeUsed>5){ 
		  cout<<"IMU Hz="<<gnHz/deltaTimeUsed<<endl;cout<<"Enc Hz="<<gnHzEnc/deltaTimeUsed<<endl;
		  gbRun=false;
		  break;
		}
		using namespace std;
		//cout << "put in command>" << endl;
		//int utmp=5;
		//cin>>utmp;
		//pSerialPort->lpNo=0;
//		pSerialPort->WriteData((BYTE*)&utmp,1);
//		sport_enc.lpNo=0;
//		sport_enc.WriteData((BYTE*)"?BS\r",strlen("?BS\r"));
		clock_gettime(CLOCK_REALTIME,&g_time);
		if (mb_speed_unlock&&(time(NULL)-last_cmd_time)>1){
		  lock_cmd.lock();
		  gdCmdVel[0]=gdCmdVel[1]=0;mb_speed_unlock=false;
		  gbCmdChanged=true;
		  lock_cmd.unlock();
		  cout<<"Output:"<<gdCmdVel[0]<<" "<<gdCmdVel[1]<<endl;
		}
		//cout<<"now time:"<<g_time.tv_sec<<"."<<g_time.tv_nsec/1000<<endl;
		
		/*static bool test = false;
		if (!test) {
			cout << "put in command>('# 500' to stop)" << endl;
			char* pCommand = new char[15];
			cin.getline(pCommand, 15);
			if (strcmp(pCommand, "# 500")==0)
				test = true;
			strcat(pCommand, "\r");//\n unnecessary
			int tmp_len = strlen(pCommand);
			sport_enc.WriteData((BYTE*)pCommand, tmp_len);
			delete pCommand;
		}*/
		usleep(6e4);
	}
		
	if (pCtrlThread!=NULL) pCtrlThread->join();

	//close continuous mode
	utmp[2]=0x00;
	pSerialPort->WriteData(utmp,3);//imu
	sport_enc.WriteData((BYTE*)"# C\r",strlen("# C\r"));//enc
	cin.get();

	cout<<"end"<<endl;
	
	return 0;
}
