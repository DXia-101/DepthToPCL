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
	/// <param name="width">ת����ͼ��Ŀ���</param>
	/// <param name="height">ת����ͼ��ĸ߶�</param>
	/// <param name="originX">���Ƶ�ԭ���x����</param>
	/// <param name="originY">���Ƶ�ԭ���y����</param>
	/// <param name="cloudin">��ת������</param>
	/// <param name="imageout">ת�����cv::Mat</param>
	static void Cloud2cvMat(int width, int height, float originX, float originY, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, cv::Mat& imageout);

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

	/// <summary>
	/// ��ȡ�������ǵ�ͼ��
	/// </summary>
	/// <param name="Matin">����ȡͼ��</param>
	/// <param name="contour">����</param>
	/// <param name="extractedImages">��ȡ���cv::Mat</param>
	static void ExtractImage(cv::Mat& Matin, std::vector<cv::Point>* contour, cv::Mat* extractedImages);

	/// <summary>
	/// �ж�һ�����Ƿ�λ�������ڲ�
	/// </summary>
	/// <param name="x">���X����</param>
	/// <param name="y">���Y����</param>
	/// <param name="contour">�жϵ�����</param>
	/// <returns>�������ڷ���true�����ڷ���false</returns>
	static bool isPointInsideContour(int x, int y, std::vector<cv::Point>* contour);
	
	/// <summary>
	/// ���������������ڲ�ͼ��ת����
	/// </summary>
	/// <param name="imageIn">��Ҫ��ȡ��ͼ��</param>
	/// <param name="originX">���Ƶ�ԭ���x����</param>
	/// <param name="originY">���Ƶ�ԭ���y����</param>
	/// <param name="contour">��ȡ������</param>
	/// <param name="cloudOut">ת�����ĵ���</param>
	static void ExtractImage2Cloud(cv::Mat& imageIn, float originX, float originY, std::vector<cv::Point>* contour, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut);

	/// <summary>
	/// ����ͼƬ����������Χ�ڵĵ㵽������
	/// </summary>
	/// <param name="Matin">�ο�ͼƬ</param>
	/// <param name="contour">������Χ</param>
	static void AddPointsInsideContour(cv::Mat& Matin, std::vector<cv::Point>* contour);
};
