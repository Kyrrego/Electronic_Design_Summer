/*��������kinect2+opencv2�Ĳ����ļ���������������kinect��opencv2�Ļ������õöԲ���*/


/*
#include "kinect.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

// ��ȫ�ͷ�ָ��
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int main()
{
	// ��ȡKinect�豸
	IKinectSensor* m_pKinectSensor;
	HRESULT hr;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	IMultiSourceFrameReader* m_pMultiFrameReader = nullptr;
	if (m_pKinectSensor)
	{
		hr = m_pKinectSensor->Open();
		if (SUCCEEDED(hr))
		{
			// ��ȡ������Դ����ȡ��
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Color |
				FrameSourceTypes::FrameSourceTypes_Infrared |
				FrameSourceTypes::FrameSourceTypes_Depth,
				&m_pMultiFrameReader);
		}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	// ��������֡������
	IDepthFrameReference* m_pDepthFrameReference = nullptr;
	IColorFrameReference* m_pColorFrameReference  = nullptr;
	IInfraredFrameReference* m_pInfraredFrameReference = nullptr;
	IInfraredFrame* m_pInfraredFrame = nullptr;
	IDepthFrame* m_pDepthFrame = nullptr;
	IColorFrame* m_pColorFrame = nullptr;
	// ����ͼƬ��ʽ
	Mat i_rgb(1080, 1920, CV_8UC4);      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	Mat i_depth(424, 512, CV_8UC1);
	Mat i_src_depth(424, 512, CV_16UC1);
	Mat i_ir(424, 512, CV_16UC1);

	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	while (true)
	{
		// ��ȡ�µ�һ����Դ����֡
		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
		if (FAILED(hr) || !m_pMultiFrame)
		{
			//cout << "!!!" << endl;
			continue;
		}

		// �Ӷ�Դ����֡�з������ɫ���ݣ�������ݺͺ�������
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_ColorFrameReference(&m_pColorFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pColorFrameReference->AcquireFrame(&m_pColorFrame);
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_DepthFrameReference(&m_pDepthFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pDepthFrameReference->AcquireFrame(&m_pDepthFrame);
		if (SUCCEEDED(hr))
			hr = m_pMultiFrame->get_InfraredFrameReference(&m_pInfraredFrameReference);
		if (SUCCEEDED(hr))
			hr = m_pInfraredFrameReference->AcquireFrame(&m_pInfraredFrame);

		// color������ͼƬ��
		UINT nColorBufferSize = 1920 * 1080 * 4;
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);

		// depth������ͼƬ��
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
			//for (int i = 0; i < 512 * 424; i++)
			//{
			//  // 0-255���ͼ��Ϊ����ʾ���ԣ�ֻȡ������ݵĵ�8λ
			//  BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
			//  reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
			//}

			// ʵ����16λunsigned int����
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_src_depth.data));
		}

		// infrared������ͼƬ��
		if (SUCCEEDED(hr))
		{
			hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
		}

		Mat i_rgb_resize = i_rgb.clone();       // ��С���㿴
		cv::resize(i_rgb_resize, i_rgb_resize, Size(512, 424));
		// ��ʾ
		imshow("rgb", i_rgb_resize);
		if (waitKey(1) == VK_ESCAPE)
			break;
		imshow("i_src_depth", i_src_depth);
		if (waitKey(1) == VK_ESCAPE)
			break;
		imshow("ir", i_ir);
		if (waitKey(1) == VK_ESCAPE)
			break;

		// �ͷ���Դ
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pInfraredFrame);
		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pInfraredFrameReference);
		SafeRelease(m_pMultiFrame);
	}
	// �رմ��ڣ��豸
	cv::destroyAllWindows();
	m_pKinectSensor->Close();
	std::system("pause");
	return 0;
}
*/


#include "pch.h"
#include <iostream>

// OpenCV ͷ�ļ�
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
// Kinect for Windows SDK ͷ�ļ�
#include <Kinect.h>

using namespace std;

int main()
{
	cout << "Hello World!\n";

	// 1a.��ȡ��Ӧ��
	IKinectSensor* pSensor = nullptr;
	GetDefaultKinectSensor(&pSensor);

	// 1b. �򿪸�Ӧ��
	pSensor->Open();

	// 2a. ȡ���������
	IDepthFrameSource* pFrameSource = nullptr;
	pSensor->get_DepthFrameSource(&pFrameSource);

	// 2b. ȡ��������ݵ�������Ϣ�����ߣ�
	int        iWidth = 0;
	int        iHeight = 0;
	IFrameDescription* pFrameDescription = nullptr;
	pFrameSource->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Width(&iWidth);
	pFrameDescription->get_Height(&iHeight);
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// 2c. ȡ�ø�Ӧ���ġ��ɿ���ȡ��������С����
	UINT16 uDepthMin = 0, uDepthMax = 0;
	pFrameSource->get_DepthMinReliableDistance(&uDepthMin);
	pFrameSource->get_DepthMaxReliableDistance(&uDepthMax);
	cout << "Reliable Distance: "
		<< uDepthMin << " �C " << uDepthMax << endl;

	// ����ͼ�����mDepthImg�����洢16λ��ͼ�����ݣ�ֱ��������ʾ��ӽ�ȫ��
	//������۲죬��mImg8bitת����8λ����ʾ
	cv::Mat mDepthImg(iHeight, iWidth, CV_16UC1);
	cv::Mat mImg8bit(iHeight, iWidth, CV_8UC1);
	cv::namedWindow("DepthImage");

	// 3a. ����������Ķ���
	IDepthFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);

	// ��ѭ��
	while (1)
	{
		// 4a. ȡ����������
		IDepthFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// 4c. �����ݴ���16λͼ�������
			pFrame->CopyFrameDataToArray(iWidth * iHeight,
				reinterpret_cast<UINT16*>(mDepthImg.data));//ǿ��ת����������

		   // 4d. ��16λת����8λ
			mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax);//converto()��һ��������������󣬵ڶ�����ת�����ͣ����������������ӣ�����4500��������ݵ�������
			cv::imshow("DepthImage", mImg8bit);
			//Ҫ�ı���ʾ����ɫ��Ч�����͸ı��mDepthImg��mImg8bit��ת��  
			// 4e. �ͷű���pFrame
			pFrame->Release();
		}

		if (cv::waitKey(30) == VK_ESCAPE) {
			break;
		}
	}

	// 3b. �ͷű���pFrameReader
	pFrameReader->Release();
	pFrameReader = nullptr;

	// 2d.�ͷű���pFramesSource
	pFrameSource->Release();
	pFrameSource = nullptr;

	// 1c.�رո�Ӧ��
	pSensor->Close();

	// 1d.�ͷŸ�Ӧ��
	pSensor->Release();
	pSensor = nullptr;

	return 0;


}

// ���г���: Ctrl + F5 ����� >����ʼִ��(������)���˵�
// ���Գ���: F5 ����� >����ʼ���ԡ��˵�

// ������ʾ: 
//   1. ʹ�ý��������Դ�������������/�����ļ�
//   2. ʹ���Ŷ���Դ�������������ӵ�Դ�������
//   3. ʹ��������ڲ鿴���������������Ϣ
//   4. ʹ�ô����б��ڲ鿴����
//   5. ת������Ŀ��>���������Դ����µĴ����ļ�����ת������Ŀ��>�����������Խ����д����ļ���ӵ���Ŀ
//   6. ��������Ҫ�ٴδ򿪴���Ŀ����ת�����ļ���>���򿪡�>����Ŀ����ѡ�� .sln �ļ�

