
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

/*С��*/
enum car {
	STOP,
	FORWARD,
	BACKWARD,
	TURNLEFT,
	TURNRIGHT,
	PAUSE,
	DETECTION
};

/*����λ����Ϣ���࣬�ο�Kinect SDK-BodyBasics D2D-CBodyBasics��*/
class CBodyBasics {
	//Kinect2.0����ȿռ䣨copy��CBodyBasics��
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

	/*��ȡ���µ�����λ����Ϣ*/
	void   Update();

	/*��ʼ��Kinect*/
	int   InitializeDefaultSensor();

	/*����������Ϣ*/
	void  ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);

	

	static DWORD  framenumber;			//����֡���
	float spinemid_xin;		//����
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


	int flag = 0;		//�¶ױ�־λ
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
	IKinectSensor*          m_pKinectSensor;//kinectԴ
	ICoordinateMapper*      m_pCoordinateMapper;//��������任
	IBodyFrameReader*       m_pBodyFrameReader;//���ڹǼ����ݶ�ȡ
	IDepthFrameReader*      m_pDepthFrameReader;//����������ݶ�ȡ
	IBodyIndexFrameReader*  m_pBodyIndexFrameReader;//���ڱ�����ֵͼ��ȡ

	//
	////���Ǽܺ���
	//void DrawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1);
	////���ֵ�״̬����
	//void DrawHandState(const DepthSpacePoint depthSpacePosition, HandState handState);
	void Detection(Joint joints[]);
	double  Distance(Joint p1, Joint p2);

	/*���������ؽڵĽǶ�*/
	void AngleHandle(Joint joints[]);
	int ElbowCalc(Joint Vertex, Joint Vertex1, Joint Vertex2);
	int HandCalc(Joint Vertex1, Joint Vertex2, Joint Vertex3);
	int ChestCalc(Joint Vertex1, Joint Vertex2, Joint Vertex3);
	void ShoulderCalc(Joint Vertex1, Joint Vertex2, int* Address);
	//��ʾͼ���Mat
	cv::Mat skeletonImg;		//����ͼ
	cv::Mat depthImg;			//���ͼ
};