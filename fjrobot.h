#ifndef _FJROBOT_H_
#define _FJROBOT_H_

//连接控制器ip 如：connect("192.168.11.10");
void connect(const char *ip);
//断开连接
void disconnect(void);
//连接状态 =0断 =1连接成功  参数rno不管，下同
int GetNetState(int rno = 0);
//读YX数据 如： int val=Yx(1700);  
int Yx(int iNo,int rno=0);
//读YC数据 如：double z=Yc(70)
double Yc(int iNo,int rno=0);
//写YX数据 如：SetYx(1699,1234);
void SetYx(int iNo,int iValue,int rno=0);
//写YC数据 如：SetYc(400,123.45);
void SetYc(int iNo,double dbValue,int rno=0);
//闭合开关 如：He(8);
void He(int iNo,int rno=0);
//断开开关 如：Fen(8);
void Fen(int iNo,int rno=0);
//使能/禁止
void ServoEnable();
//清除报警
void ClearAlarm();
//设置工作模式
void SetWorkMode(int mode);  // mode: 48 = 运行,   51 = 调节
//暂停运动
void PressPause();
//启动运行
void PressStart();	
//允许手动操作
void SetManualOP(int op);   //op=1 允许操作，op=0禁止操作

#endif  /*_FJROBOT_H_*/
