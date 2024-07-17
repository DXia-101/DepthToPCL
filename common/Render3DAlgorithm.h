#pragma once
#include "pcl_function.h"
#include "Depth2RGB.h"
#include "teAiExTypes.h"
#include <QPixmap>
#include <QString>
#include <QMetaType>
#include <vector>

class Render_3D_Algorithm
{
public:
	/// <summary>
	/// 加载点云
	/// </summary>
	/// <param name="fileName">点云文件路径</param>
	/// <returns>加载的点云</returns>
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr LoadPointCloud(QString fileName);

	/// <summary>
	/// 保存点云
	/// </summary>
	/// <param name="fileName">需要保存的路径</param>
	/// <param name="saveCloud">需要保存的点云</param>
	/// <returns></returns>
	static bool SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud);

	/// <summary>
	/// 分割点云
	/// </summary>
	/// <param name="clipRange">当前渲染的前后界面范围</param>
	/// <param name="windowsize">当前vtk显示窗口的size</param>
	/// <param name="viewport">当前vtk的视口</param>
	/// <param name="cloudin">待分割的点云</param>
	/// <param name="pointSet">分割的轮廓</param>
	/// <param name="transmat">待分割点云的变化矩阵</param>
	/// <param name="rotatemat">待分割点云的旋转矩阵</param>
	/// <returns>分割后的点云</returns>
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr Segment(double* clipRange, int* windowsize, double* viewport, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin, const std::vector<QPointF>& pointSet, vtkMatrix4x4* transmat, vtkMatrix4x4* rotatemat);

	/**
	 * @brief inOrNot1
	 * @param poly_sides    平面上绘制多边形的顶点数
	 * @param poly_X        顶点的x坐标数组
	 * @param poly_Y        顶点的y坐标数组
	 * @param x             目标点云的x坐标
	 * @param y             目标点云的y坐标
	 * @return
	 */
	static int inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y);

	/// <summary>
	/// 按坐标轴渲染点云
	/// </summary>
	/// <param name="cloud">待渲染点云</param>
	/// <param name="curaxis">参照坐标轴</param>
	/// <param name="invalidthreshold">该轴的最小阈值</param>
	/// <param name="validthreshold">该轴的最大阈值</param>
	/// <returns></returns>
	static bool CoordinateAxisRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QString curaxis, double invalidthreshold, double validthreshold);

	/// <summary>
	/// 从点云中根据轮廓提取点云
	/// </summary>
	/// <param name="cloud">待提取点云</param>
	/// <param name="instance">提取的轮廓</param>
	/// <returns>提取出的点云</returns>
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr ExtractingPointCloudsBasedOnContours(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, te::AiInstance* instance);

	/// <summary>
	/// 将点从Canvas转到平面坐标
	/// </summary>
	/// <param name="windowsize">当前vtk显示窗口的size</param>
	/// <param name="viewport">当前vtk的视口</param>
	/// <param name="input3D">待转换的点</param>
	/// <param name="mat">坐标变化的复合矩阵</param>
	/// <param name="output2D">转换后的点</param>
	static void WorldToScreen(int* windowsize, double* viewport, pcl::PointXYZRGB* input3D, vtkMatrix4x4* mat, double* output2D);

	/// <summary>
	/// 将点从Canvas转到平面坐标
	/// </summary>
	/// <param name="windowsize">当前vtk显示窗口的size</param>
	/// <param name="viewport">当前vtk的视口</param>
	/// <param name="input3D">待转换的点</param>
	/// <param name="transform">坐标变化的复合矩阵</param>
	/// <param name="composit">点云的旋转矩阵</param>
	/// <param name="output2D">转换后的点</param>
	static void WorldToScreen(int* windowsize, double* viewport, pcl::PointXYZRGB* input3D, vtkMatrix4x4* transform, vtkMatrix4x4* composit, double* output2D);

	/// <summary>
	/// 计算点云AABB包围框
	/// </summary>
	/// <param name="cloud">需要计算的点云</param>
	/// <returns>返回点云AABB包围框的两个对顶角</returns>
	static std::pair<pcl::PointXYZRGB, pcl::PointXYZRGB> AxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

	/// <summary>
	/// 计算点云OBB包围框
	/// </summary>
	/// <param name="cloud">需要计算的点云</param>
	/// <returns>返回OBB包围框结构体</returns>
	static OBBStruct OrientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

	/// <summary>
	/// 剔除第一个点云中与第二个点云重合的点
	/// </summary>
	/// <param name="cloud1"></param>
	/// <param name="cloud2"></param>
	static void subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2);

	/// <summary>
	/// 计算点云透视到轴
	/// </summary>
	/// <param name="maxPt">点云的最大点</param>
	/// <param name="minPt">点云的最小点</param>
	/// <param name="cloud">待计算的点云</param>
	/// <returns>点云的中心点，偏移的距离</returns>
	static std::pair<Eigen::Vector3f, float> PerspectiveToAxis(pcl::PointXYZ maxPt, pcl::PointXYZ minPt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

	/// <summary>
	/// 对点云进行切割
	/// </summary>
	/// <param name="cloud_in">待切割点云</param>
	/// <param name="cloud_out">切割后点云</param>
	/// <param name="min">切割方向的最小值</param>
	/// <param name="max">切割方向的最大值</param>
	/// <param name="axis">切割的方向</param>
	/// <param name="is_save">保留内点还是外点</param>
	static void pcl_crossSection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, float min, float max, QString axis, float is_save);

	/// <summary>
	/// 对点云进行高度变换
	/// </summary>
	/// <param name="cloud">待变换的点云</param>
	/// <param name="factor">高度变换的参数</param>
	static void HeightTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int factor);
};