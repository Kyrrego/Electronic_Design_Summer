/*包含两个kinect2+opencv2的测试文件，可以用来测试kinect和opencv2的环境配置得对不对*/


/*
#include "kinect.h"
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

// 安全释放指针
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
	// 获取Kinect设备
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
			// 获取多数据源到读取器
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
	// 三个数据帧及引用
	IDepthFrameReference* m_pDepthFrameReference = nullptr;
	IColorFrameReference* m_pColorFrameReference  = nullptr;
	IInfraredFrameReference* m_pInfraredFrameReference = nullptr;
	IInfraredFrame* m_pInfraredFrame = nullptr;
	IDepthFrame* m_pDepthFrame = nullptr;
	IColorFrame* m_pColorFrame = nullptr;
	// 三个图片格式
	Mat i_rgb(1080, 1920, CV_8UC4);      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat i_depth(424, 512, CV_8UC1);
	Mat i_src_depth(424, 512, CV_16UC1);
	Mat i_ir(424, 512, CV_16UC1);

	UINT16 *depthData = new UINT16[424 * 512];
	IMultiSourceFrame* m_pMultiFrame = nullptr;
	while (true)
	{
		// 获取新的一个多源数据帧
		hr = m_pMultiFrameReader->AcquireLatestFrame(&m_pMultiFrame);
		if (FAILED(hr) || !m_pMultiFrame)
		{
			//cout << "!!!" << endl;
			continue;
		}

		// 从多源数据帧中分离出彩色数据，深度数据和红外数据
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

		// color拷贝到图片中
		UINT nColorBufferSize = 1920 * 1080 * 4;
		if (SUCCEEDED(hr))
			hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(i_rgb.data), ColorImageFormat::ColorImageFormat_Bgra);

		// depth拷贝到图片中
		if (SUCCEEDED(hr))
		{
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, depthData);
			//for (int i = 0; i < 512 * 424; i++)
			//{
			//  // 0-255深度图，为了显示明显，只取深度数据的低8位
			//  BYTE intensity = static_cast<BYTE>(depthData[i] % 256);
			//  reinterpret_cast<BYTE*>(i_depth.data)[i] = intensity;
			//}

			// 实际是16位unsigned int数据
			hr = m_pDepthFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_src_depth.data));
		}

		// infrared拷贝到图片中
		if (SUCCEEDED(hr))
		{
			hr = m_pInfraredFrame->CopyFrameDataToArray(424 * 512, reinterpret_cast<UINT16*>(i_ir.data));
		}

		Mat i_rgb_resize = i_rgb.clone();       // 缩小方便看
		cv::resize(i_rgb_resize, i_rgb_resize, Size(512, 424));
		// 显示
		imshow("rgb", i_rgb_resize);
		if (waitKey(1) == VK_ESCAPE)
			break;
		imshow("i_src_depth", i_src_depth);
		if (waitKey(1) == VK_ESCAPE)
			break;
		imshow("ir", i_ir);
		if (waitKey(1) == VK_ESCAPE)
			break;

		// 释放资源
		SafeRelease(m_pColorFrame);
		SafeRelease(m_pDepthFrame);
		SafeRelease(m_pInfraredFrame);
		SafeRelease(m_pColorFrameReference);
		SafeRelease(m_pDepthFrameReference);
		SafeRelease(m_pInfraredFrameReference);
		SafeRelease(m_pMultiFrame);
	}
	// 关闭窗口，设备
	cv::destroyAllWindows();
	m_pKinectSensor->Close();
	std::system("pause");
	return 0;
}
*/


#include "pch.h"
#include <iostream>

// OpenCV 头文件
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
// Kinect for Windows SDK 头文件
#include <Kinect.h>

using namespace std;

int main()
{
	cout << "Hello World!\n";

	// 1a.获取感应器
	IKinectSensor* pSensor = nullptr;
	GetDefaultKinectSensor(&pSensor);

	// 1b. 打开感应器
	pSensor->Open();

	// 2a. 取得深度数据
	IDepthFrameSource* pFrameSource = nullptr;
	pSensor->get_DepthFrameSource(&pFrameSource);

	// 2b. 取得深度数据的描述信息（宽、高）
	int        iWidth = 0;
	int        iHeight = 0;
	IFrameDescription* pFrameDescription = nullptr;
	pFrameSource->get_FrameDescription(&pFrameDescription);
	pFrameDescription->get_Width(&iWidth);
	pFrameDescription->get_Height(&iHeight);
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// 2c. 取得感应器的“可靠深度”的最大、最小距离
	UINT16 uDepthMin = 0, uDepthMax = 0;
	pFrameSource->get_DepthMinReliableDistance(&uDepthMin);
	pFrameSource->get_DepthMaxReliableDistance(&uDepthMax);
	cout << "Reliable Distance: "
		<< uDepthMin << " – " << uDepthMax << endl;

	// 建立图像矩阵，mDepthImg用来存储16位的图像数据，直接用来显示会接近全黑
	//不方便观察，用mImg8bit转换成8位再显示
	cv::Mat mDepthImg(iHeight, iWidth, CV_16UC1);
	cv::Mat mImg8bit(iHeight, iWidth, CV_8UC1);
	cv::namedWindow("DepthImage");

	// 3a. 打开深度数据阅读器
	IDepthFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);

	// 主循环
	while (1)
	{
		// 4a. 取得最新数据
		IDepthFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// 4c. 把数据存入16位图像矩阵中
			pFrame->CopyFrameDataToArray(iWidth * iHeight,
				reinterpret_cast<UINT16*>(mDepthImg.data));//强制转换数据类型

		   // 4d. 把16位转换成8位
			mDepthImg.convertTo(mImg8bit, CV_8U, 255.0f / uDepthMax);//converto()第一个参数是输出矩阵，第二个是转换类型，第三个是缩放因子，其中4500是深度数据的最大距离
			cv::imshow("DepthImage", mImg8bit);
			//要改变显示的颜色和效果，就改变从mDepthImg到mImg8bit的转换  
			// 4e. 释放变量pFrame
			pFrame->Release();
		}

		if (cv::waitKey(30) == VK_ESCAPE) {
			break;
		}
	}

	// 3b. 释放变量pFrameReader
	pFrameReader->Release();
	pFrameReader = nullptr;

	// 2d.释放变量pFramesSource
	pFrameSource->Release();
	pFrameSource = nullptr;

	// 1c.关闭感应器
	pSensor->Close();

	// 1d.释放感应器
	pSensor->Release();
	pSensor = nullptr;

	return 0;


}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门提示: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件

