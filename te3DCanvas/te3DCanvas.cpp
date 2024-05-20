#include "te3DCanvas.h"
#include <QMessageBox>
#include <QDebug>
#include <QWheelEvent>
#include <QEvent>
#include <QElapsedTimer>

#define DEBUG

#ifdef DEBUG
#include <QTime>
#endif
#include "Transfer_Function.h"
#include "teDataStorage.h"

te3DCanvas::te3DCanvas(QWidget *parent)
	: QVTKOpenGLNativeWidget(parent)
{
	PCL_Initalization();
    VTKCoordinateAxis();
}

te3DCanvas::~te3DCanvas()
{}

void te3DCanvas::PCL_Initalization()
{
    cloud = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    cloud_polygon = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    cloud_cliped = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    cloud_Filter_out = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    cloud_marked = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    cloud_Elevation_rendering = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();

    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_renderWindow = this->renderWindow();
    m_renderWindow->AddRenderer(m_renderer);
    //m_renderer->GetActiveCamera()->vtkCamera::ParallelProjectionOn();
    m_renderer->ResetCameraClippingRange();

    viewer.reset(new pcl::visualization::PCLVisualizer(m_renderer, m_renderWindow, "viewer", false));
    this->setRenderWindow(m_renderWindow);
    viewer->setupInteractor(m_renderWindow->GetInteractor(), m_renderWindow);
    viewer->setBackgroundColor(0.0, 0.3, 0.4);
    viewer->addCoordinateSystem(1.0);
    this->update();
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    currentCategory = "";
}

/// <summary>
/// 不规则框选的鼠标画线
/// </summary>
/// <param name="event"></param>
void te3DCanvas::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        if (m_member.isPickingMode) {
            double world_point[3];
            double displayPos[2];
            displayPos[0] = double(event->pos().x()), displayPos[1] = double(this->height() - event->pos().y() - 1);
            getScreentPos(displayPos, world_point, this);

            curP = pcl::PointXYZ(world_point[0], world_point[1], world_point[2]);
            if (!m_member.flag)m_member.flag = true;
            else {
                char str1[512];
                sprintf(str1, "line#%03d", m_member.line_id++);
                viewer->addLine(lastP, curP, str1);
            }
            lastP = curP;
            cloud_polygon->push_back(curP);
        }
    }
}

/**
 * @brief getScreentPos     屏幕坐标转换至世界坐标
 * @param displayPos        输入：屏幕坐标
 * @param world             输出：世界坐标
 * @param viewer_void       输入：pclViewer
 */
void te3DCanvas::getScreentPos(double* displayPos, double* world, void* viewer_void)
{
    double fp[4], tmp1[4], eventFPpos[4];
    m_renderer->GetActiveCamera()->GetFocalPoint(fp);
    fp[3] = 0.0;
    m_renderer->SetWorldPoint(fp);
    m_renderer->WorldToDisplay();
    m_renderer->GetDisplayPoint(tmp1);

    tmp1[0] = displayPos[0];
    tmp1[1] = displayPos[1];

    m_renderer->SetDisplayPoint(tmp1);
    m_renderer->DisplayToWorld();

    m_renderer->GetWorldPoint(eventFPpos);
    // Copy the result
    for (int i = 0; i < 3; i++)
    {
        world[i] = eventFPpos[i];
    }
}


/// <summary>
/// 不规则框选
/// </summary>
/// <param name="viewer_void"></param>
void te3DCanvas::PolygonSelect(void* viewer_void)
{
    double focal[3] = { 0 }; double pos[3] = { 0 };
    m_renderer->GetActiveCamera()->GetFocalPoint(focal);
    m_renderer->GetActiveCamera()->GetPosition(pos);

    pcl::PointXYZ eyeLine1 = pcl::PointXYZ(focal[0] - pos[0], focal[1] - pos[1], focal[2] - pos[2]);

    float mochang = sqrt(pow(eyeLine1.x, 2) + pow(eyeLine1.y, 2) + pow(eyeLine1.z, 2));//模长
    pcl::PointXYZ eyeLine = pcl::PointXYZ(eyeLine1.x / mochang, eyeLine1.y / mochang, eyeLine1.z / mochang);//单位向量 法向量

    //创建一个平面
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());//ax+by+cz+d=0
    coefficients->values.resize(4);
    coefficients->values[0] = eyeLine.x;// 法向量 x 分量
    coefficients->values[1] = eyeLine.y;// 法向量 y 分量
    coefficients->values[2] = eyeLine.z;// 法向量 z 分量
    coefficients->values[3] = 0;        // 平面偏移量

    //创建保存结果投影的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn_Prj(new pcl::PointCloud<pcl::PointXYZ>);//输入的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCiecle_result(new pcl::PointCloud<pcl::PointXYZ>);//绘制的线
    // 创建滤波器对象
    pcl::ProjectInliers<pcl::PointXYZ> proj;//建立投影对象
    proj.setModelType(pcl::SACMODEL_PLANE);//设置投影类型
    proj.setInputCloud(cloud_polygon);//设置输入点云
    proj.setModelCoefficients(coefficients);//加载投影参数
    proj.filter(*cloudCiecle_result);//执行程序，并将结果保存

    // 创建滤波器对象
    pcl::ProjectInliers<pcl::PointXYZ> projCloudIn;//建立投影对象
    projCloudIn.setModelType(pcl::SACMODEL_PLANE);//设置投影类型
    projCloudIn.setInputCloud(cloud);//设置输入点云
    projCloudIn.setModelCoefficients(coefficients);//加载投影参数
    projCloudIn.filter(*cloudIn_Prj);//执行程序，并将结果保存

    int ret = -1;
    double* PloyXarr = new double[cloudCiecle_result->points.size()];
    double* PloyYarr = new double[cloudCiecle_result->points.size()];
    for (int i = 0; i < cloudCiecle_result->points.size(); ++i)
    {
        PloyXarr[i] = cloudCiecle_result->points[i].x;
        PloyYarr[i] = cloudCiecle_result->points[i].y;
    }

    cloud_cliped->clear();
    for (int i = 0; i < cloudIn_Prj->points.size(); i++)
    {
        ret = inOrNot1(cloud_polygon->points.size(), PloyXarr, PloyYarr, cloudIn_Prj->points[i].x, cloudIn_Prj->points[i].y);
        if (1 == ret)//表示在里面
        {
            cloud_cliped->points.push_back(cloud->points[i]);
        }//表示在外面
    }
        
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> currentColor(cloud_cliped,currentColor.red(), currentColor.green(), currentColor.blue());
    QString CloudId;
    auto it = markerPCID.find(teDataStorage::getInstance()->getCurrentLabelCategory());
    if (it != markerPCID.end()) {
        CloudId = teDataStorage::getInstance()->getCurrentLabelCategory() + "marker" + QString::number(it->second.size());
        it->second.push_back(CloudId);
    }
    else {
        CloudId = teDataStorage::getInstance()->getCurrentLabelCategory() + "marker" + "0";
        markerPCID.insert(std::make_pair(teDataStorage::getInstance()->getCurrentLabelCategory(), std::vector<QString>{CloudId}));
    }

    teDataStorage::getInstance()->updateMarkersNumber();
    
    viewer->addPointCloud(cloud_cliped, currentColor, CloudId.toStdString());
    emit sig_3DCanvasMarkingCompleted(cloud_cliped);
    m_renderWindow->Render();
}

bool te3DCanvas::LoadPointCloud(QString fileName)
{
    if (fileName.isEmpty())
    {
        return false;
    }

    cloud->points.clear();

    if (fileName.endsWith("pcd"))
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *cloud) == -1)
        {
            qDebug() << "Couldn't read pcd file!\n";
            return false;
        }
    }
    else {
        QMessageBox::warning(this, "Warning", u8"点云读取格式错误！");
    }

    SetCoordinateSet();
}

bool te3DCanvas::SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr saveCloud)
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
            qDebug() << errorinfo;
            return false;
        }
    }
    return true;
}

void te3DCanvas::reRenderOriginCloud()
{
    reRendering(cloud);
}

void te3DCanvas::ShowDimension(int arg)
{
    if (arg > 0) {
        for (const auto& pair : markerPCID) {
            for (const QString& id : pair.second) {
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, id.toStdString());
            }
        }
    }
    else {
        for (const auto& pair : markerPCID) {
            for (const QString& id : pair.second) {
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, id.toStdString());
            }
        }
    }
    m_renderWindow->Render();
}

void te3DCanvas::ShowResult(int arg)
{
    if (arg > 0) {
        for (const auto& pair : resultPCID) {
            for (const QString& id : pair.second) {
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1.0, id.toStdString());
            }
        }
    }
    else {
        for (const auto& pair : resultPCID) {
            for (const QString& id : pair.second) {
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.0, id.toStdString());
            }
        }
    }
    m_renderWindow->Render();
}

void te3DCanvas::ReductionPointCloud()
{
    LoadPointCloud(QString::fromStdString(teDataStorage::getInstance()->getCurrentPointCloud()));
    reRenderOriginCloud();
}

bool te3DCanvas::SetBackgroundColor(QColor color)
{
    viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
    m_renderWindow->Render();
    return true;
}

bool te3DCanvas::CoordinateAxisRendering(QString curaxis)
{
    double Z_Max = axisset.maxPt.z;
    double Z_Min = axisset.minPt.z;
    double Z_Median1 = Z_Min + (Z_Max - Z_Min) / 3;
    double Z_Median2 = Z_Median1 + (Z_Max - Z_Min) / 3;

    for (int index = 0; index < cloud->points.size(); ++index)
    {
        if (cloud->points[index].z >= Z_Min && cloud->points[index].z < Z_Median1)
        {
            pcl::PointXYZRGB point;
            point.x = cloud->points[index].x;
            point.y = cloud->points[index].y;
            point.z = cloud->points[index].z;
            point.r = 128 - int(((Z_Median1 - cloud->points[index].z) / (Z_Median1 - Z_Min)) * 128);
            point.g = 255 - int(((Z_Median1 - cloud->points[index].z) / (Z_Median1 - Z_Min)) * 255);
            point.b = 0 + int(((Z_Median1 - cloud->points[index].z) / (Z_Median1 - Z_Min)) * 255);
            cloud_Elevation_rendering->push_back(point);
        }
        if (cloud->points[index].z >= Z_Median1 && cloud->points[index].z < Z_Median2)
        {
            pcl::PointXYZRGB point;
            point.x = cloud->points[index].x;
            point.y = cloud->points[index].y;
            point.z = cloud->points[index].z;
            point.r = 255 - int(((Z_Median2 - cloud->points[index].z) / (Z_Median2 - Z_Median1)) * 128);
            point.g = 255;
            point.b = 0;
            cloud_Elevation_rendering->push_back(point);
        }
        if (cloud->points[index].z >= Z_Median2 && cloud->points[index].z < Z_Max)
        {
            pcl::PointXYZRGB point;
            point.x = cloud->points[index].x;
            point.y = cloud->points[index].y;
            point.z = cloud->points[index].z;
            point.r = 255;
            point.g = 255 - int(((cloud->points[index].z - Z_Median2) / (Z_Max - Z_Median2)) * 255);
            point.b = 0;
            cloud_Elevation_rendering->push_back(point);
        }
    }
    viewer->updatePointCloud(cloud_Elevation_rendering, "cloud");
    m_renderWindow->Render();
    return true;
}

bool te3DCanvas::PointCloudColorSet(QColor color)
{
    QColor temp;
    temp.setRgb(143, 153, 159, 255);
    if (!cloud->empty() && (color != temp)) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>selected_color(cloud, color.redF() * 255, color.greenF() * 255, color.blueF() * 255);
        viewer->updatePointCloud(cloud, selected_color, "cloud");
        m_renderWindow->Render();
    }
    else {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>selected_color(cloud, 255, 255, 255);
        viewer->updatePointCloud(cloud, selected_color, "cloud");
        m_renderWindow->Render();
    }
    return true;
}

bool te3DCanvas::PointCloudPointSizeSet(int point_size)
{
    for (int i = 0; i < 1; ++i) {
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "cloud");
    }
    m_renderWindow->Render();
    return true;
}

void te3DCanvas::MarkersShowInCanvas(te::AiInstance* instance, cv::Mat& m_image,QColor color)
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
    cloud_marked->points.clear();
    Transfer_Function::ExtractCloud2Cloud(cloud, &contour, cloud_marked);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> currentColor(cloud_marked, color.red(), color.green(), color.blue());
    QString CloudId;
    auto it = markerPCID.find(teDataStorage::getInstance()->getCurrentLabelCategory());
    if (it != markerPCID.end()) {
        CloudId = teDataStorage::getInstance()->getCurrentLabelCategory() + "marker" + QString::number(it->second.size());
        it->second.push_back(CloudId);
    }
    else {
        CloudId = teDataStorage::getInstance()->getCurrentLabelCategory() + "marker" + "0";
        markerPCID.insert(std::make_pair(teDataStorage::getInstance()->getCurrentLabelCategory(), std::vector<QString>{CloudId}));
    }
    viewer->addPointCloud(cloud_marked, currentColor, CloudId.toStdString());
    m_renderWindow->Render();
}

void te3DCanvas::ResultsShowInCanvas(te::AiInstance* instance, cv::Mat& m_image, QColor color)
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
    cloud_marked->points.clear();
    Transfer_Function::ExtractCloud2Cloud(cloud, &contour, cloud_marked);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> currentColor(cloud_marked, color.red(), color.green(), color.blue());
    QString CloudId;
    auto it = resultPCID.find(teDataStorage::getInstance()->getCurrentLabelCategory());
    if (it != resultPCID.end()) {
        CloudId = teDataStorage::getInstance()->getCurrentLabelCategory() + "result" + QString::number(it->second.size());
        it->second.push_back(CloudId);
    }
    else {
        CloudId = teDataStorage::getInstance()->getCurrentLabelCategory() + "result" + "0";
        resultPCID.insert(std::make_pair(teDataStorage::getInstance()->getCurrentLabelCategory(), std::vector<QString>{CloudId}));
    }
    viewer->addPointCloud(cloud_marked, currentColor, CloudId.toStdString());
    m_renderWindow->Render();
}

void te3DCanvas::reRendering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin)
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    
    viewer->addPointCloud<pcl::PointXYZ>(cloudin, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    // 计算点云中心位置和对角线长度
    Eigen::Vector3f center((axisset.maxPt.x + axisset.minPt.x) / 2, (axisset.maxPt.y + axisset.minPt.y) / 2, (axisset.maxPt.z + axisset.minPt.z) / 2);
    Eigen::Vector3f diff = axisset.maxPt.getVector3fMap() - axisset.minPt.getVector3fMap();
    float distance = diff.norm();

    viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), 0, 0, 0); //耗时最多 2秒多
    //viewer->spinOnce();
    m_renderWindow->Render();
    setRotationCenter();
    m_CustomInteractor->ResetData();
    emit sig_CanvasreRender();
}

AxisSet te3DCanvas::getAxisSet()
{
    return axisset;
}

std::vector<double> te3DCanvas::getCloudCentroid()
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud,centroid);
    return { centroid.x(), centroid.y(), centroid.z() };
}

void te3DCanvas::SetCoordinateSet()
{
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(*cloud, min, max);
    int currentDisplayImageLength = max.x - min.x;
    int currentDisplayImageHeight = max.y - min.y;
    axisset = {
        currentDisplayImageLength,
        currentDisplayImageHeight,
        min.x,
        min.y,
        min,
        max,
    };
}

void te3DCanvas::VTKCoordinateAxis()
{
    axes_actor = vtkSmartPointer<vtkAxesActor>::New();
    axes_actor->SetAxisLabels(1);
    axes_actor->SetPosition(0, 0, 0);
    axes_actor->SetTotalLength(2, 2, 2);
    axes_actor->SetShaftType(0);
    axes_actor->SetCylinderRadius(0.02);

    m_CustomInteractor = CustomInteractorStyle::New();
    m_CustomInteractor->setRenderWindow(m_renderWindow, m_renderer, axes_actor);
    m_renderWindow->GetInteractor()->SetInteractorStyle(m_CustomInteractor);

    markerWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    markerWidget->SetOrientationMarker(axes_actor);
    markerWidget->SetInteractor(m_renderWindow->GetInteractor());
    markerWidget->SetEnabled(1);
    markerWidget->SetInteractive(false);

    m_renderWindow->Render();
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
int te3DCanvas::inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y)
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

void te3DCanvas::AxisAlignedBoundingBox()
{
    feature_extractor.setInputCloud(cloud); //提供指向输入数据集的指针。
    feature_extractor.compute();//启动所有特征的计算

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;

    pcl::PointXYZ min_point_AABB;//AABB包围盒
    pcl::PointXYZ max_point_AABB;

    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);

    viewer->addPointCloud<pcl::PointXYZ>(cloud, "points");
    viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

    m_renderWindow->Render();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

void te3DCanvas::OrientedBoundingBox()
{
    feature_extractor.setInputCloud(cloud);
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

    viewer->addPointCloud<pcl::PointXYZ>(cloud, /*RandomColor, */ "points");

    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);
    viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");
    m_renderWindow->Render();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

/// <summary>
/// 剔除第一个点云中与第二个点云重合的点
/// </summary>
/// <param name="cloud1"></param>
/// <param name="cloud2"></param>
void te3DCanvas::subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    //采样第一个点云
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud1);
    voxelGrid.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    voxelGrid.filter(*cloud1_downsampled);

    // 创建 CropBox 滤波器，设置包围盒范围为第二个点云的边界框
    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(cloud1_downsampled);
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(*cloud2, min, max);
    cropBox.setMin(Eigen::Vector4f(min.x, min.y, min.z, 1.0f));
    cropBox.setMax(Eigen::Vector4f(max.x, max.y, max.z, 1.0f));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    cropBox.filter(*cloud1_filtered);

    // 提取被剔除的点
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    cropBox.filter(indices->indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud1_downsampled);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*cloud1_filtered);

    cloud1->swap(*cloud1_filtered);
}

void te3DCanvas::PerspectiveToYaxis()
{
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    Eigen::Vector3f center((maxPt.x + minPt.x) / 2, (maxPt.y + minPt.y) / 2, (maxPt.z + minPt.z) / 2);
    Eigen::Vector3f diff = maxPt.getVector3fMap() - minPt.getVector3fMap();
    float distance = diff.norm();

    if (m_member.PositiveAndNegative_Y_axis)
        viewer->setCameraPosition(center(0), center(1), center(2) + distance, 0, center(1), 0, 1, 0, 0);
    else
        viewer->setCameraPosition(center(0), center(1), center(2) + distance, 0, center(1), 0, -1, 0, 0);
    m_member.PositiveAndNegative_Y_axis = !m_member.PositiveAndNegative_Y_axis;
    viewer->updateCamera();
    viewer->spinOnce();
}

void te3DCanvas::PerspectiveToXaxis()
{
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    Eigen::Vector3f center((maxPt.x + minPt.x) / 2, (maxPt.y + minPt.y) / 2, (maxPt.z + minPt.z) / 2);
    Eigen::Vector3f diff = maxPt.getVector3fMap() - minPt.getVector3fMap();
    float distance = diff.norm();

    if (m_member.PositiveAndNegative_X_axis)
        viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), 0, 0, 0, 1, 0);
    else
        viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), 0, 0, 0, -1, 0);
    m_member.PositiveAndNegative_X_axis = !m_member.PositiveAndNegative_X_axis;
    viewer->updateCamera();
    viewer->spinOnce();
}

void te3DCanvas::PerspectiveToZaxis()
{
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    Eigen::Vector3f center((maxPt.x + minPt.x) / 2, (maxPt.y + minPt.y) / 2, (maxPt.z + minPt.z) / 2);
    Eigen::Vector3f diff = maxPt.getVector3fMap() - minPt.getVector3fMap();
    float distance = diff.norm();

    if (m_member.PositiveAndNegative_Z_axis)
        viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), 1, 0, 0);
    else
        viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), -1, 0, 0);
    m_member.PositiveAndNegative_Z_axis = !m_member.PositiveAndNegative_Z_axis;
    viewer->updateCamera();
    viewer->spinOnce();
}

void te3DCanvas::GuassFilter(QString data1, QString data2, QString data3, QString data4)
{
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", "无点云输入");
        return;
    }
    else {
        if (data1.isEmpty() || data2.isEmpty() || data3.isEmpty() || data4.isEmpty()) {
            QMessageBox::warning(this, "Warning", "参数格式输入错误");
            return;
        }
        pcl_filter_guass(cloud, data1.toFloat(), data2.toFloat(), data3.toFloat(), data4.toFloat());
        *cloud = *cloud_Filter_out;

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->addPointCloud(cloud, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->resetCamera();
        m_renderWindow->Render();
    }
}

void te3DCanvas::pcl_filter_guass(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float paraA, float paraB, float paraC, float paraD)
{
    pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
    (*kernel).setSigma(paraA);
    (*kernel).setThresholdRelativeToSigma(paraB);
    //KdTree加速
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    (*kdtree).setInputCloud(cloud_in);
    pcl::filters::Convolution3D <pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> > convolution;
    convolution.setKernel(*kernel);
    convolution.setInputCloud(cloud_in);
    convolution.setSearchMethod(kdtree);
    convolution.setRadiusSearch(paraC);
    convolution.setNumberOfThreads(paraD);

    convolution.convolve(*cloud_Filter_out);
}

void te3DCanvas::DirectFilter(QString data1, QString data2, QString data3, QString data4)
{
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", "无点云输入");
        return;
    }
    else {
        if (data1.isEmpty() || data2.isEmpty() || data3.isEmpty() || data4.isEmpty()) {
            QMessageBox::warning(this, "Warning", "参数格式输入错误");
            return;
        }
        pcl_filter_direct(cloud, data1.toFloat(), data2.toFloat(), data3, data4.toFloat());
        *cloud = *cloud_Filter_out;
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->addPointCloud(cloud, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->resetCamera();
        m_renderWindow->Render();
    }
}
void te3DCanvas::LabelChanged(const QString& content, const QColor& fontColor)
{
    currentCategory = content;
    currentColor = fontColor;
}

void te3DCanvas::PointCloudHeightTransform(int factor)
{
    LoadPointCloud(QString::fromStdString(teDataStorage::getInstance()->getCurrentPointCloud()));
    for (int i = 0; i < cloud->size(); ++i) {
        cloud->at(i).z *= factor;
    }
    reRendering(cloud->makeShared());
}

void te3DCanvas::te3DCanvasStartMarking()
{
    m_member.isPickingMode = !m_member.isPickingMode;
    if (m_member.isPickingMode) {
        m_member.line_id++;
        cloud_polygon->clear();
        m_member.flag = false;
    }
    else {
        PolygonSelect(this);
    }
}

void te3DCanvas::pcl_filter_direct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float min, float max, QString axis, float is_save)
{
    pcl::PassThrough<pcl::PointXYZ> pass;//设置滤波器对象

    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName(axis.toStdString());
    pass.setFilterLimits(min, max);
    pass.setNegative(is_save);

    pass.filter(*cloud_Filter_out);
}

vtkRenderWindow* te3DCanvas::getvtkRenderWindow()
{
    return m_renderWindow;
}

vtkSmartPointer<vtkRenderer> te3DCanvas::getvtkRenderer()
{
    return vtkSmartPointer<vtkRenderer>();
}

void te3DCanvas::setRotationCenter()
{
    m_CustomInteractor->setRotationCenter(getCloudCentroid()[0], getCloudCentroid()[1], getCloudCentroid()[2]);
}

void te3DCanvas::SetClassBCallback(teMouseCircle& classB)
{
    m_CustomInteractor->SetCallback
    (
        [&classB]() 
        {
            classB.restitution();
        }
    );
}
