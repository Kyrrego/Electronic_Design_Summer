
#pragma once
#include <Kinect.h>
#include <opencv.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include "Client.h"


// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

/*小车*/
enum car {
	STOP,
	FORWARD,
	BACKWARD,
	TURNLEFT,
	TURNRIGHT,
	PAUSE,
	DETECTION
};

/*身体位置信息的类，参考Kinect SDK-BodyBasics D2D-CBodyBasics库*/
class CBodyBasics {
	//Kinect2.0的深度空间（copy自CBodyBasics）
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;

public:
	/// <summary>
	/// Constructor
	/// </summary>
	CBodyBasics();

	/// <summary>
	/// Destructor
	/// </summary>
	~CBodyBasics();

	/*获取最新的身体位置信息*/
	void   Update();

	/*初始化Kinect*/
	int   InitializeDefaultSensor();

	/*处理身体信息*/
	void  ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);

	

	static DWORD  framenumber;			//骨骼帧编号
	float spinemid_xin;		//重心
	float spinemid_yin;
	float spinemid_zin;
	float spinemid_xout;
	float spinemid_yout;
	float spinemid_zout;
	float spinemid_x;
	float spinemid_y;
	float spinemid_z;
	float spinebase_yin, spinebase_yout, spinebase_y;
	float rightfoot_yin = 0, rightfoot_yout = 0, rightfoot_y = 0;
	float rightAnkle_yin = 0, rightAnkle_yout = 0, rightAnkle_y = 0;
	float leftfoot_yin = 0, leftfoot_yout = 0, leftfoot_y = 0;
	float base_foot_in = 0, base_foot_out = 0, base_foot = 0;
	float spinetemp = 0;

	const double thresh_x = 0.15;
	const double thresh_z = 0.15;
	const double handdistance = 0.10;
	unsigned char Msg[10];
	char Msgsend[10];


	int flag = 0;		//下蹲标志位
	int handstate = 0;

	int armstate = 0; //1 for start
	int mapstate = 0; //1 for drawing map
	//car carstate = STOP;
	Send SendMsg;
	fstream file;

	char detectionstate = 0;

	bool Is_SendMsg = true;
	bool Is_SendPort = false;
	int close = 0;
	int Humiclose = 0;
	char TempData[4];

	static CBodyBasics* CurBodyBasics;
	//pthread_t threads_kinect;
	int rc_kinect;
	//pthread_t threads_humi;
	int rc_humi;

private:
	IKinectSensor*          m_pKinectSensor;//kinect源
	ICoordinateMapper*      m_pCoordinateMapper;//用于坐标变换
	IBodyFrameReader*       m_pBodyFrameReader;//用于骨架数据读取
	IDepthFrameReader*      m_pDepthFrameReader;//用于深度数据读取
	IBodyIndexFrameReader*  m_pBodyIndexFrameReader;//用于背景二值图读取

	//
	////画骨架函数
	//void DrawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1);
	////画手的状态函数
	//void DrawHandState(const DepthSpacePoint depthSpacePosition, HandState handState);
	void Detection(Joint joints[]);
	double  Distance(Joint p1, Joint p2);

	/*计算三个关节的角度*/
	void AngleHandle(Joint joints[]);
	int ElbowCalc(Joint Vertex, Joint Vertex1, Joint Vertex2);
	int HandCalc(Joint Vertex1, Joint Vertex2, Joint Vertex3);
	int ChestCalc(Joint Vertex1, Joint Vertex2, Joint Vertex3);
	void ShoulderCalc(Joint Vertex1, Joint Vertex2, int* Address);
	//显示图像的Mat
	cv::Mat skeletonImg;		//骨骼图
	cv::Mat depthImg;			//深度图
};