
#pragma once
#include <Kinect.h>
#include <opencv.hpp>
#include <iostream>
#include <fstream>
#include <cmath>

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
	long int   InitializeDefaultSensor();

	/*����������Ϣ*/
	void  ProcessBody(INT64 nTime, int nBodyCount, IBody** ppBodies);

	/*������sample����֪����û����*/
	/// <summary>
	/// Converts a body point to screen space
	/// </summary>
	/// <param name="bodyPoint">body point to tranform</param>
	/// <param name="width">width (in pixels) of output buffer</param>
	/// <param name="height">height (in pixels) of output buffer</param>
	/// <returns>point in screen-space</returns>
	D2D1_POINT_2F           BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);

	/// <summary>
	/// Draws a body 
	/// </summary>
	/// <param name="pJoints">joint data</param>
	/// <param name="pJointPoints">joint positions converted to screen space</param>
	void                    DrawBody(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints);

	/// <summary>
	/// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
	/// </summary>
	/// <param name="handState">state of the hand</param>
	/// <param name="handPosition">position of the hand</param>
	void                    DrawHand(HandState handState, const D2D1_POINT_2F& handPosition);

	/// <summary>
	/// Draws one bone of a body (joint to joint)
	/// </summary>
	/// <param name="pJoints">joint data</param>
	/// <param name="pJointPoints">joint positions converted to screen space</param>
	/// <param name="pJointPoints">joint positions converted to screen space</param>
	/// <param name="joint0">one joint of the bone to draw</param>
	/// <param name="joint1">other joint of the bone to draw</param>
	void                    DrawBone(const Joint* pJoints, const D2D1_POINT_2F* pJointPoints, JointType joint0, JointType joint1);
};