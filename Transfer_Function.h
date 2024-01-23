#pragma once
#include "pcl_function.h"
#include "teAiExTypes.h"

class DynamicLabel;

class Transfer_Function
{
public:
	/// <summary>
	/// ����תΪcv::Mat
	/// </summary>
	/// <param name="datumPointCloud">��׼����</param>
	/// <param name="cloudin">��ת������</param>
	/// <param name="imageout">ת�����cv::Mat</param>
	static void Cloud2cvMat(pcl::PointCloud<pcl::PointXYZ>::Ptr datumPointCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, cv::Mat& imageout);

	/// <summary>
	/// cv::Mat תΪ ����
	/// </summary>
	/// <param name="imageIn">��ת����cv::Mat</param>
	/// <param name="cloudOut">ת����ĵ���</param>
	static void cvMat2Cloud(cv::Mat& imageIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut);
	
	/// <summary>
	/// cv::Mat ��������
	/// </summary>
	/// <param name="Matin">�����ҵ�cv::Mat</param>
	/// <param name="contours">�ҵ���������</param>
	static void cvMat2Contour(cv::Mat& Matin, std::vector<std::vector<cv::Point>>* contours);

};

