#include "Render3DAlgorithm.h"
#include "Transfer_Function.h"

/// <summary>
/// 加载点云
/// </summary>
/// <param name="fileName">点云文件路径</param>
/// <returns>加载的点云</returns>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Render_3D_Algorithm::LoadPointCloud(QString fileName)
{
	if (fileName.isEmpty())
	{
		return nullptr;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	if (fileName.endsWith("pcd"))
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileName.toStdString(), *cloud) == -1)
		{
			return nullptr;
		}
	}
	else {
		return nullptr;
	}
	return cloud;
}

/// <summary>
/// 保存点云
/// </summary>
/// <param name="fileName">需要保存的路径</param>
/// <param name="saveCloud">需要保存的点云</param>
/// <returns></returns>

bool Render_3D_Algorithm::SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud)
{
	if (saveCloud->empty()) {
		return false;
	}
	else {
		if (fileName.isEmpty()) {
			return false;
		}
		int return_status;
		if (fileName.endsWith(".pcd", Qt::CaseInsensitive))
			return_status = pcl::io::savePCDFileBinary(fileName.toStdString(), *saveCloud);
		else if (fileName.endsWith(".ply", Qt::CaseInsensitive))
			return_status = pcl::io::savePCDFileBinary(fileName.toStdString(), *saveCloud);
		else {
			fileName.append(".pcd");
			return_status = pcl::io::savePCDFileBinary(fileName.toStdString(), *saveCloud);
		}
		if (return_status != 0) {
			QString errorinfo = QString::fromStdString("Error writing point cloud" + fileName.toStdString());
			return false;
		}
	}
	return true;
}

/// <summary>
/// 分割点云
/// </summary>
/// <param name="renderer">待分割点云的渲染器</param>
/// <param name="cloudin">待分割的点云</param>
/// <param name="pointSet">分割的轮廓</param>
/// <param name="transmat">待分割点云的变化矩阵</param>
/// <returns>分割后的点云</returns>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Render_3D_Algorithm::Segment(double* clipRange, int* windowsize, double* viewport, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin, const std::vector<QPointF>& pointSet, vtkMatrix4x4* transform, vtkMatrix4x4* rotatemat)
{
	if (cloudin->empty()) {
		return nullptr;
	}
	double* FBRange = clipRange;

	double* PloyXarr = new double[pointSet.size()];
	double* PloyYarr = new double[pointSet.size()];
	for (int i = 0; i < pointSet.size(); ++i)
	{
		PloyXarr[i] = pointSet[i].x();
		PloyYarr[i] = pointSet[i].y();
	}
	const auto& mat = transform;
	const auto& transmat = rotatemat;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cliped = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();
	for (int i = 0; i < cloudin->points.size(); ++i)
	{
		pcl::PointXYZRGB P3D = cloudin->points.at(i);
		double P2D[2];
		WorldToScreen(windowsize, viewport, &P3D, transmat, mat, P2D);
		if (inOrNot1(pointSet.size(), PloyXarr, PloyYarr, P2D[0], P2D[1]))
		{
			cloud_cliped->points.push_back(cloudin->points.at(i));
		}
	}
	if (cloud_cliped->points.size() == 0)
		return nullptr;
	return cloud_cliped;
}

/**
* @brief inOrNot1
* @param poly_sides    平面上绘制多边形的顶点数
* @param poly_X        顶点的x坐标数组
* @param poly_Y        顶点的y坐标数组
* @param x             目标点云的x坐标
* @param y             目标点云的y坐标
* @return
*/

int Render_3D_Algorithm::inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y)
{
	int i, j;
	j = poly_sides - 1;
	int res = 0;
	for (i = 0; i < poly_sides; i++) {
		if (((poly_Y[i] < y && poly_Y[j] >= y) || (poly_Y[j] < y && poly_Y[i] >= y)) && (poly_X[i] <= x || poly_X[j] <= x))
		{
			res ^= ((poly_X[i] + (y - poly_Y[i]) / (poly_Y[j] - poly_Y[i]) * (poly_X[j] - poly_X[i])) < x);
		}
		j = i;
	}
	return res;
}

/// <summary>
/// 按坐标轴渲染点云
/// </summary>
/// <param name="cloud">待渲染点云</param>
/// <param name="curaxis">参照坐标轴</param>
/// <param name="invalidthreshold">该轴的最小阈值</param>
/// <param name="validthreshold">该轴的最大阈值</param>
/// <returns></returns>

bool Render_3D_Algorithm::CoordinateAxisRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, QString curaxis, double invalidthreshold, double validthreshold)
{
	if (cloud->empty()) {
		return false;
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Elevation_rendering(new pcl::PointCloud<pcl::PointXYZRGB>);

	double min = invalidthreshold;
	double max = validthreshold;

	TeJetColorCode trans;
	for (int index = 0; index < cloud->points.size(); ++index)
	{
		double value = 0;
		if (curaxis == "z")
		{
			value = cloud->points[index].z;
		}
		else if (curaxis == "x")
		{
			value = cloud->points[index].x;
		}
		else if (curaxis == "y")
		{
			value = cloud->points[index].y;
		}

		pcl::PointXYZRGB point = cloud->points[index];

		float absDepth = value > min ? value : min;
		absDepth = absDepth < max ? absDepth : max;
		float realDepth = max == min ? 0 : ((absDepth - min) / (max - min));
		int iIndex = 1023 * realDepth;//将[0-1.0]之间的数据映射到[0-1024)之间
		point.r = trans.m_pJetTab1024[iIndex].r;
		point.g = trans.m_pJetTab1024[iIndex].g;
		point.b = trans.m_pJetTab1024[iIndex].b;

		cloud_Elevation_rendering->push_back(point);
	}
	cloud->clear();
	pcl::copyPointCloud(*cloud_Elevation_rendering, *cloud);

	return true;
}

/// <summary>
/// 从点云中根据轮廓提取点云
/// </summary>
/// <param name="cloud">待提取点云</param>
/// <param name="instance">提取的轮廓</param>
/// <returns>提取出的点云</returns>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Render_3D_Algorithm::ExtractingPointCloudsBasedOnContours(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, te::AiInstance* instance)
{
	std::vector<cv::Point> contour;
	te::PolygonF maxpolygon = instance->contour.polygons.at(0);
	for (te::PolygonF polygon : instance->contour.polygons) {
		if (maxpolygon.size() < polygon.size()) {
			maxpolygon = polygon;
		}
	}
	for (const te::Point2f& point : maxpolygon) {
		contour.push_back(cv::Point(point.x, point.y));
	}
	cv::Mat extractImage;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_marked = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();

	Transfer_Function::ExtractCloud2Cloud(cloud, &contour, cloud_marked);
	return cloud_marked;
}

/// <summary>
/// 将点从Canvas转到平面坐标
/// </summary>
/// <param name="renderer">当前Canvas的渲染器</param>
/// <param name="input3D">待转换的点</param>
/// <param name="mat">坐标变化的复合矩阵</param>
/// <param name="output2D">转换后的点</param>

void Render_3D_Algorithm::WorldToScreen(int* windowsize, double* viewport, pcl::PointXYZRGB* input3D, vtkMatrix4x4* mat, double* output2D)
{
	double view[4];
	{
		view[0] = static_cast<double>(mat->GetElement(0, 0) * input3D->x + mat->GetElement(0, 1) * input3D->y + mat->GetElement(0, 2) * input3D->z + mat->GetElement(0, 3));
		view[1] = static_cast<double>(mat->GetElement(1, 0) * input3D->x + mat->GetElement(1, 1) * input3D->y + mat->GetElement(1, 2) * input3D->z + mat->GetElement(1, 3));
		view[2] = static_cast<double>(mat->GetElement(2, 0) * input3D->x + mat->GetElement(2, 1) * input3D->y + mat->GetElement(2, 2) * input3D->z + mat->GetElement(2, 3));
		view[3] = static_cast<double>(mat->GetElement(3, 0) * input3D->x + mat->GetElement(3, 1) * input3D->y + mat->GetElement(3, 2) * input3D->z + mat->GetElement(3, 3));
	};

	if (view[3] != 0.0)
	{
		input3D->x = view[0] / view[3];
		input3D->y = view[1] / view[3];
		input3D->z = view[2] / view[3];
	}


	double dx, dy;
	int sizex, sizey;

	const int* size = windowsize;
	if (!size)
	{
		return;
	}
	sizex = size[0];
	sizey = size[1];

	dx = (input3D->x + 1.0) * (sizex * (viewport[2] - viewport[0])) / 2.0 +
		sizex * viewport[0];
	dy = (input3D->y + 1.0) * (sizey * (viewport[3] - viewport[1])) / 2.0 +
		sizey * viewport[1];

	output2D[0] = dx;
	output2D[1] = dy;

}

/// <summary>
/// 将点从Canvas转到平面坐标
/// </summary>
/// <param name="renderer">当前Canvas的渲染器</param>
/// <param name="input3D">待转换的点</param>
/// <param name="transform">坐标变化的复合矩阵</param>
/// <param name="composit">点云的变换矩阵</param>
/// <param name="output2D">转换后的点</param>

void Render_3D_Algorithm::WorldToScreen(int* windowsize, double* viewport, pcl::PointXYZRGB* input3D, vtkMatrix4x4* transform, vtkMatrix4x4* composit, double* output2D)
{
	double trans[4];
	{
		trans[0] = static_cast<double>(transform->GetElement(0, 0) * input3D->x + transform->GetElement(0, 1) * input3D->y + transform->GetElement(0, 2) * input3D->z + transform->GetElement(0, 3));
		trans[1] = static_cast<double>(transform->GetElement(1, 0) * input3D->x + transform->GetElement(1, 1) * input3D->y + transform->GetElement(1, 2) * input3D->z + transform->GetElement(1, 3));
		trans[2] = static_cast<double>(transform->GetElement(2, 0) * input3D->x + transform->GetElement(2, 1) * input3D->y + transform->GetElement(2, 2) * input3D->z + transform->GetElement(2, 3));
		trans[3] = static_cast<double>(transform->GetElement(3, 0) * input3D->x + transform->GetElement(3, 1) * input3D->y + transform->GetElement(3, 2) * input3D->z + transform->GetElement(3, 3));
	}

	double view[4];
	{
		view[0] = static_cast<double>(composit->GetElement(0, 0) * trans[0] + composit->GetElement(0, 1) * trans[1] + composit->GetElement(0, 2) * trans[2] + composit->GetElement(0, 3));
		view[1] = static_cast<double>(composit->GetElement(1, 0) * trans[0] + composit->GetElement(1, 1) * trans[1] + composit->GetElement(1, 2) * trans[2] + composit->GetElement(1, 3));
		view[2] = static_cast<double>(composit->GetElement(2, 0) * trans[0] + composit->GetElement(2, 1) * trans[1] + composit->GetElement(2, 2) * trans[2] + composit->GetElement(2, 3));
		view[3] = static_cast<double>(composit->GetElement(3, 0) * trans[0] + composit->GetElement(3, 1) * trans[1] + composit->GetElement(3, 2) * trans[2] + composit->GetElement(3, 3));
	};

	if (view[3] != 0.0)
	{
		input3D->x = view[0] / view[3];
		input3D->y = view[1] / view[3];
		input3D->z = view[2] / view[3];
	}

	double dx, dy;
	int sizex, sizey;

	const int* size = windowsize;
	if (!size)
	{
		return;
	}
	sizex = size[0];
	sizey = size[1];

	dx = (input3D->x + 1.0) * (sizex * (viewport[2] - viewport[0])) / 2.0 +
		sizex * viewport[0];
	dy = (input3D->y + 1.0) * (sizey * (viewport[3] - viewport[1])) / 2.0 +
		sizey * viewport[1];

	output2D[0] = dx;
	output2D[1] = dy;
}

/// <summary>
/// 计算点云AABB包围框
/// </summary>
/// <param name="cloud">需要计算的点云</param>
/// <returns>返回点云AABB包围框的两个对顶角</returns>

std::pair<pcl::PointXYZRGB, pcl::PointXYZRGB> Render_3D_Algorithm::AxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	if (cloud->empty()) {
		return std::make_pair(pcl::PointXYZRGB(), pcl::PointXYZRGB());
	}
	pcl::PointXYZRGB min_point_AABB;
	pcl::PointXYZRGB max_point_AABB;

	pcl::getMinMax3D(*cloud, min_point_AABB, max_point_AABB);

	return std::make_pair(min_point_AABB, max_point_AABB);
}

/// <summary>
/// 计算点云AABB包围框
/// </summary>
/// <param name="cloud">需要计算的点云</param>
/// <returns>返回点云AABB包围框的两个对顶角</returns>

/// <summary>
/// 计算点云OBB包围框
/// </summary>
/// <param name="cloud">需要计算的点云</param>
/// <returns>返回OBB包围框结构体</returns>

OBBStruct Render_3D_Algorithm::OrientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	if (cloud->empty()) {
		return OBBStruct();
	}
	pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*cloud, *cloud_xyz);
	feature_extractor.setInputCloud(cloud_xyz);
	feature_extractor.compute();

	std::vector <float> moment_of_inertia;
	std::vector <float> eccentricity;
	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	float major_value, middle_value, minor_value;
	Eigen::Vector3f major_vector, middle_vector, minor_vector;
	Eigen::Vector3f mass_center;

	feature_extractor.getMomentOfInertia(moment_of_inertia);
	feature_extractor.getEccentricity(eccentricity);
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	feature_extractor.getEigenValues(major_value, middle_value, minor_value);
	feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
	feature_extractor.getMassCenter(mass_center);

	Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
	Eigen::Quaternionf quat(rotational_matrix_OBB);

	OBBStruct res{ position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z };

	return res;
}

/// <summary>
/// 剔除第一个点云中与第二个点云重合的点
/// </summary>
/// <param name="cloud1"></param>
/// <param name="cloud2"></param>

void Render_3D_Algorithm::subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
{
	//采样第一个点云
	pcl::VoxelGrid<pcl::PointXYZRGB> voxelGrid;
	voxelGrid.setInputCloud(cloud1);
	voxelGrid.setLeafSize(0.01f, 0.01f, 0.01f);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
	voxelGrid.filter(*cloud1_downsampled);

	// 创建 CropBox 滤波器，设置包围盒范围为第二个点云的边界框
	pcl::CropBox<pcl::PointXYZRGB> cropBox;
	cropBox.setInputCloud(cloud1_downsampled);
	pcl::PointXYZRGB min;
	pcl::PointXYZRGB max;
	pcl::getMinMax3D(*cloud2, min, max);
	cropBox.setMin(Eigen::Vector4f(min.x, min.y, min.z, 1.0f));
	cropBox.setMax(Eigen::Vector4f(max.x, max.y, max.z, 1.0f));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	cropBox.filter(*cloud1_filtered);

	// 提取被剔除的点
	pcl::PointIndices::Ptr indices(new pcl::PointIndices);
	cropBox.filter(indices->indices);
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	extract.setInputCloud(cloud1_downsampled);
	extract.setIndices(indices);
	extract.setNegative(true);
	extract.filter(*cloud1_filtered);

	cloud1->swap(*cloud1_filtered);
}

/// <summary>
/// 计算点云透视到轴
/// </summary>
/// <param name="maxPt">点云的最大点</param>
/// <param name="minPt">点云的最小点</param>
/// <param name="cloud">待计算的点云</param>
/// <returns>点云的中心点，偏移的距离</returns>

std::pair<Eigen::Vector3f, float> Render_3D_Algorithm::PerspectiveToAxis(pcl::PointXYZ maxPt, pcl::PointXYZ minPt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	if (!cloud->empty())
	{
		Eigen::Vector3f center((maxPt.x + minPt.x) / 2, (maxPt.y + minPt.y) / 2, (maxPt.z + minPt.z) / 2);
		Eigen::Vector3f diff = maxPt.getVector3fMap() - minPt.getVector3fMap();
		float distance = diff.norm();
		return std::make_pair(center, distance);
	}
	return std::make_pair(Eigen::Vector3f(), 0.0);
}

/// <summary>
/// 对点云进行切割
/// </summary>
/// <param name="cloud_in">待切割点云</param>
/// <param name="cloud_out">切割后点云</param>
/// <param name="min">切割方向的最小值</param>
/// <param name="max">切割方向的最大值</param>
/// <param name="axis">切割的方向</param>
/// <param name="is_save">保留内点还是外点</param>

void Render_3D_Algorithm::pcl_crossSection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, float min, float max, QString axis, float is_save)
{
	pcl::PassThrough<pcl::PointXYZRGB> pass;//设置滤波器对象

	pass.setInputCloud(cloud_in);
	pass.setFilterFieldName(axis.toStdString());
	pass.setFilterLimits(min, max);
	pass.setNegative(is_save);

	pass.filter(*cloud_out);
}

/// <summary>
/// 对点云进行高度变换
/// </summary>
/// <param name="cloud">待变换的点云</param>
/// <param name="factor">高度变换的参数</param>

void Render_3D_Algorithm::HeightTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int factor)
{
	if (factor != 1 && !cloud->empty())
	{
		for (auto& point : cloud->points) {
			point.z = point.z * factor;
		}
	}
}