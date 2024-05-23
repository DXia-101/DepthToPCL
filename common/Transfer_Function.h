#pragma once
#include "pcl_function.h"
#include "teAiExTypes.h"
#include <QPixmap>
#include <QString>
#include <QMetaType>
#include <vector>

class DynamicLabel;

struct ImageInfo {
	QString ImgPath;
	QString ImgResolution;
	QString ImgName;
};

Q_DECLARE_METATYPE(ImageInfo)

class Transfer_Function
{
public:
	/// <summary>
	/// 点云转为cv::Mat
	/// </summary>
	/// <param name="width">转换后图像的宽度</param>
	/// <param name="height">转换后图像的高度</param>
	/// <param name="originX">点云的原点的x坐标</param>
	/// <param name="originY">点云的原点的y坐标</param>
	/// <param name="cloudin">待转换点云</param>
	/// <param name="imageout">转换后的cv::Mat</param>
	static void Cloud2cvMat(int width, int height, float originX, float originY, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin, cv::Mat& imageout);
	
	/// <summary>
	/// cv::Mat 转为 点云
	/// </summary>
	/// <param name="minHeight"></param>
	/// <param name="maxHeight"></param>
	/// <param name="imageIn">待转换的cv::Mat</param>
	/// <param name="cloudOut">转换后的点云</param>
	static void cvMat2Cloud(double minHeight, double maxHeight,cv::Mat& imageIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut);
	
	/// <summary>
	/// cv::Mat 查找轮廓
	/// </summary>
	/// <param name="Matin">待查找的cv::Mat</param>
	/// <param name="contours">找到的轮廓集</param>
	static void cvMat2Contour(cv::Mat& Matin, std::vector<std::vector<cv::Point>>* contours);

	/// <summary>
	/// 提取轮廓覆盖的图像
	/// </summary>
	/// <param name="Matin">待提取图像</param>
	/// <param name="contour">轮廓</param>
	/// <param name="extractedImages">提取后的cv::Mat</param>
	static void ExtractImage(cv::Mat& Matin, std::vector<cv::Point>* contour, cv::Mat* extractedImages);

	/// <summary>
	/// 判断一个点是否位于轮廓内部
	/// </summary>
	/// <param name="x">点的X坐标</param>
	/// <param name="y">点的Y坐标</param>
	/// <param name="contour">判断的轮廓</param>
	/// <returns>在轮廓内返回true，不在返回false</returns>
	static bool isPointInsideContour(int x, int y, std::vector<cv::Point>* contour);
	
	/// <summary>
	/// 根据轮廓将轮廓内部图像转点云
	/// </summary>
	/// <param name="imageIn">需要提取的图像</param>
	/// <param name="originX">点云的原点的x坐标</param>
	/// <param name="originY">点云的原点的y坐标</param>
	/// <param name="contour">提取的轮廓</param>
	/// <param name="cloudOut">转换出的点云</param>
	static void ExtractImage2Cloud(cv::Mat& imageIn, float originX, float originY, std::vector<cv::Point>* contour, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut);

	static void ExtractCloud2Cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, std::vector<cv::Point>* contour, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut);

	/// <summary>
	/// 添加图片中在轮廓范围内的点到轮廓里
	/// </summary>
	/// <param name="Matin">参考图片</param>
	/// <param name="contour">轮廓范围</param>
	static void AddPointsInsideContour(cv::Mat& Matin, std::vector<cv::Point>* contour);

	/// <summary>
	/// 加载图片为QPixmap格式
	/// </summary>
	/// <param name="filePath">图像路径</param>
	/// <returns>加载图片的QPixmap对象</returns>
	static QPixmap loadPixmap(const QString& filePath);

	/// <summary>
	/// 将三维Vector转为AiInstance
	/// </summary>
	/// <param name="Contours">需要转换的三维Vector</param>
	/// <returns>转换后的AiInstance</returns>
	static te::AiInstance VectorToAiInstance(std::vector<std::vector<cv::Point>>* Contours);
};

