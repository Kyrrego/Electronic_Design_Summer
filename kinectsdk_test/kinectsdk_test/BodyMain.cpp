#include "BodyMain.h"

using   namespace   std;
using   namespace   cv;
using   namespace   Sample;

CBodyBasics* CBodyBasics::CurBodyBasics = nullptr;

//构造函数，需要改
CBodyBasics::CBodyBasics(QObject* Parent) :
	QObject(Parent),
	m_pKinectSensor(NULL),
	m_pCoordinateMapper(NULL),
	m_pBodyFrameReader(NULL),
	m_pDepthFrameReader(NULL),
	m_pBodyIndexFrameReader(NULL)
{
	//pDlg = new CMFC_DEMO01Dlg();
}

// 析构函数，需要改
CBodyBasics::~CBodyBasics()
{
	SafeRelease(m_pBodyFrameReader);
	SafeRelease(m_pCoordinateMapper);
	SafeRelease(m_pDepthFrameReader);
	SafeRelease(m_pBodyIndexFrameReader);

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}
	SafeRelease(m_pKinectSensor);
}

//求2个骨骼点之间的距离
double CBodyBasics::Distance(Joint p1, Joint p2)
{
	return sqrt(pow(p2.Position.X - p1.Position.X, 2) + pow(p2.Position.Y - p1.Position.Y, 2) + pow(p2.Position.Z - p1.Position.Z, 2));
}

/*传输指定格式的所有角度数据*/
void CBodyBasics::AngleHandle(Joint joints[]) {
	int AngleElbow, AngleHand, AngleChest;
	int AngleSholder[2];
	if (armstate == 1 && (framenumber % 11 == 3 || framenumber % 11 == 8)) {
		for (int i = 0; i < 10; i++) {
			Msg[i] = 255;
		}
		if (joints[JointType_ElbowRight].Position.X - joints[JointType_ShoulderRight].Position.X < 0.05) {
			cout << "请将手臂放于身体外部" << endl;
			return;
		}
		if (handstate == 2 || handstate == 3) {

			AngleElbow = ElbowCalc(joints[JointType_ElbowRight], joints[JointType_ShoulderRight], joints[JointType_WristRight]);
			AngleHand = HandCalc(joints[JointType_WristRight], joints[JointType_ThumbRight], joints[JointType_HandTipRight]);
			ShoulderCalc(joints[JointType_ElbowRight], joints[JointType_ShoulderRight], AngleSholder);
			AngleChest = ChestCalc(joints[JointType_HipRight], joints[JointType_HipLeft], joints[JointType_SpineShoulder]);

			if (handstate == 2) {

				Msg[0] = 1;
				Msg[1] = AngleChest;
				Msg[2] = AngleSholder[0];
				Msg[3] = AngleSholder[1];
				Msg[4] = AngleElbow;
				Msg[5] = AngleElbow;
				Msg[6] = AngleHand;
				Msg[7] = 255;
			}
			else if (handstate == 3) {
				Msg[0] = 2;
				Msg[1] = AngleChest;
				Msg[2] = AngleSholder[0];
				Msg[3] = AngleSholder[1];
				Msg[4] = AngleElbow;
				Msg[5] = AngleElbow;
				Msg[6] = AngleHand;
				Msg[7] = 1;
			}
		}


		for (int i = 0; i < 10; i++) {
			Msgsend[i] = (char)Msg[i];
		}

		if (Is_SendMsg && mapstate == 0) {
			SendMsg.Send_msg(Msgsend);
			inttohex(Msg, 10);
		}


	}
}

/*三个关节角度的计算*/
int CBodyBasics::ElbowCalc(Joint ElbowRight, Joint ShoulderRight, Joint WristRight) {
	float vector1[3], vector2[3];
	float dotresult, length1, length2;
	float cosine, angle;
	vector1[0] = ShoulderRight.Position.X - ElbowRight.Position.X;
	vector1[1] = ShoulderRight.Position.Y - ElbowRight.Position.Y;
	vector1[2] = ShoulderRight.Position.Z - ElbowRight.Position.Z;
	vector2[0] = WristRight.Position.X - ElbowRight.Position.X;
	vector2[1] = WristRight.Position.Y - ElbowRight.Position.Y;
	vector2[2] = WristRight.Position.Z - ElbowRight.Position.Z;
	dotresult = vector1[0] * vector2[0] + vector1[1] * vector2[1] + vector1[2] * vector2[2];
	length1 = sqrt(pow(vector1[0], 2) + pow(vector1[1], 2) + pow(vector2[2], 2));
	length2 = sqrt(pow(vector2[0], 2) + pow(vector2[1], 2) + pow(vector2[2], 2));
	cosine = dotresult / (length1 * length2);
	angle = acos(cosine) * 180.0 / PI;
	if (angle < 20) {
		angle = 20;
	}
	if (angle > 160) {
		angle = 160;
	}
	angle = angle / 2.0;
	angle = angle * (254.0 / 70.0) - 35.285714;
	int intangle = (int)angle;
	if (intangle < 1) {
		intangle = 1;
	}
	if (intangle > 254) {
		intangle = 254;
	}
	intangle = -intangle + 256;

	return intangle;
}

int CBodyBasics::HandCalc(Joint Vertex1, Joint Vertex2, Joint Vertex3) {
	float edge1[3], edge2[3];
	edge1[0] = Vertex2.Position.X - Vertex1.Position.X;
	edge1[1] = Vertex2.Position.Y - Vertex1.Position.Y;
	edge1[2] = Vertex2.Position.Z - Vertex1.Position.Z;
	edge2[0] = Vertex3.Position.X - Vertex1.Position.X;
	edge2[1] = Vertex3.Position.Y - Vertex1.Position.Y;
	edge2[2] = Vertex3.Position.Z - Vertex1.Position.Z;
	float NormalVector[3];
	NormalVector[0] = edge1[1] * edge2[2] - edge1[2] * edge2[1];
	NormalVector[1] = edge1[2] * edge2[0] - edge1[0] * edge2[2];
	NormalVector[2] = edge1[0] * edge2[1] - edge1[1] * edge2[0];
	double length = sqrt(pow(NormalVector[0], 2) + pow(NormalVector[1], 2) + pow(NormalVector[2], 2));
	float UnitNormalVector[3];
	UnitNormalVector[0] = NormalVector[0] / length;
	UnitNormalVector[1] = NormalVector[1] / length;
	UnitNormalVector[2] = NormalVector[2] / length;
	float Yaxis[3] = { 0,1,0 };
	float dotresult;
	float angle;
	dotresult = UnitNormalVector[0] * Yaxis[0] + UnitNormalVector[1] * Yaxis[1] + UnitNormalVector[2] * Yaxis[2];
	angle = acos(dotresult) * 180.0 / PI;
	if (angle < 40) {
		angle = 40;
	}
	if (angle > 140) {
		angle = 140;
	}
	angle = angle * (254.0 / 100.0) - 100.6;
	int intangle = int(angle);
	if (intangle < 1) {
		intangle = 1;
	}
	if (intangle > 254) {
		intangle = 254;
	}
	return intangle;
}

int CBodyBasics::ChestCalc(Joint ShoulderRight, Joint ShoulderLeft, Joint SpineMid) {
	float edge1[3], edge2[3];
	edge1[0] = ShoulderLeft.Position.X - ShoulderRight.Position.X;
	edge1[1] = ShoulderLeft.Position.Y - ShoulderRight.Position.Y;
	edge1[2] = ShoulderLeft.Position.Z - ShoulderRight.Position.Z;
	edge2[0] = SpineMid.Position.X - ShoulderRight.Position.X;
	edge2[1] = SpineMid.Position.Y - ShoulderRight.Position.Y;
	edge2[2] = SpineMid.Position.Z - ShoulderRight.Position.Z;
	float NormalVector[3];
	NormalVector[0] = edge1[1] * edge2[2] - edge1[2] * edge2[1];
	NormalVector[1] = edge1[2] * edge2[0] - edge1[0] * edge2[2];
	NormalVector[2] = edge1[0] * edge2[1] - edge1[1] * edge2[0];
	double length = sqrt(pow(NormalVector[0], 2) + pow(NormalVector[1], 2) + pow(NormalVector[2], 2));
	float UnitNormalVector[3];
	UnitNormalVector[0] = NormalVector[0] / length;
	UnitNormalVector[1] = NormalVector[1] / length;
	UnitNormalVector[2] = NormalVector[2] / length;
	float Yaxis[3] = { 1,0,0 };
	float dotresult;
	float angle;
	int intangle;
	dotresult = UnitNormalVector[0] * Yaxis[0] + UnitNormalVector[1] * Yaxis[1] + UnitNormalVector[2] * Yaxis[2];
	angle = acos(dotresult) * 180.0 / PI;
	if (angle > 120) {
		angle = 120;
	}
	if (angle < 60) {
		angle = 60;
	}
	angle = angle * (254.0 / 60.0) - 253.0;
	intangle = (int)angle;
	if (intangle < 1) {
		intangle = 1;
	}
	if (intangle > 254) {
		intangle = 254;
	}
	return intangle;
}

void CBodyBasics::ShoulderCalc(Joint ElbowRight, Joint ShoulderRight, int* AngleShoulder) {
	float UnitVector[3];
	UnitVector[0] = ElbowRight.Position.X - ShoulderRight.Position.X;
	UnitVector[1] = ElbowRight.Position.Y - ShoulderRight.Position.Y;
	UnitVector[2] = ElbowRight.Position.Z - ShoulderRight.Position.Z;
	double length = sqrt(pow(UnitVector[0], 2) + pow(UnitVector[1], 2) + pow(UnitVector[2], 2));
	UnitVector[0] = UnitVector[0] / length;
	UnitVector[1] = UnitVector[1] / length;
	UnitVector[2] = UnitVector[2] / length;
	float SphericalAngle[2];

	SphericalAngle[0] = atan(UnitVector[0] / UnitVector[2]) * 180.0 / PI; //phi
	SphericalAngle[1] = acos(UnitVector[1]) * 180.0 / PI; //theta

	if (SphericalAngle[1] < 20) {
		SphericalAngle[1] = 20;
	}
	if (SphericalAngle[1] > 160) {
		SphericalAngle[1] = 160;
	}
	SphericalAngle[1] = SphericalAngle[1] * (254.0 / 140.0) - 35.285714;

	if (SphericalAngle[0] >= 0) {
		SphericalAngle[0] = SphericalAngle[0];
	}
	else if (SphericalAngle[0] < 0) {
		SphericalAngle[0] = SphericalAngle[0] + 180.0;
	}
	if (SphericalAngle[0] < 70) {
		SphericalAngle[0] = 70;
	}
	if (SphericalAngle[0] > 140) {
		SphericalAngle[0] = 140;
	}
	SphericalAngle[0] = SphericalAngle[0] * (254.0 / 70.0) - 253.0;

	AngleShoulder[0] = (int)SphericalAngle[0];
	AngleShoulder[1] = (int)SphericalAngle[1];



	if (AngleShoulder[0] < 1) {
		AngleShoulder[0] = 1;
	}
	if (AngleShoulder[0] > 254) {
		AngleShoulder[0] = 254;
	}
	if (AngleShoulder[1] < 1) {
		AngleShoulder[1] = 1;
	}
	if (AngleShoulder[1] > 254) {
		AngleShoulder[1] = 254;
	}
}
