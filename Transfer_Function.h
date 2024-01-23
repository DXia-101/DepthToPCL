#pragma once
#include "pcl_function.h"
#include "teAiExTypes.h"

class DynamicLabel;

class Transfer_Function
{
public:
	/// <summary>
	/// 点云转为cv::Mat
	/// </summary>
	/// <param name="datumPointCloud">基准点云</param>
	/// <param name="cloudin">待转换点云</param>
	/// <param name="imageout">转换后的cv::Mat</param>
	static void Cloud2cvMat(pcl::PointCloud<pcl::PointXYZ>::Ptr datumPointCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, cv::Mat& imageout);

	/// <summary>
	/// cv::Mat 转为 点云
	/// </summary>
	/// <param name="imageIn">待转换的cv::Mat</param>
	/// <param name="cloudOut">转换后的点云</param>
	static void cvMat2Cloud(cv::Mat& imageIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut);
	
	/// <summary>
	/// cv::Mat 查找轮廓
	/// </summary>
	/// <param name="Matin">待查找的cv::Mat</param>
	/// <param name="contours">找到的轮廓集</param>
	static void cvMat2Contour(cv::Mat& Matin, std::vector<std::vector<cv::Point>>* contours);

};

