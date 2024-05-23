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
    cloud = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();
    cloud_polygon = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();
    cloud_cliped = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();
    cloud_Filter_out = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();
    cloud_marked = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();
    
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
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");

    currentCategory = "";
}

/// <summary>
/// 不规则框选的鼠标画线
/// </summary>
/// <param name="event"></param>
void te3DCanvas::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        if (m_member.isPickingMode) {
            double world_point[3];
            double displayPos[2];
            displayPos[0] = double(event->pos().x()), displayPos[1] = double(this->height() - event->pos().y() - 1);
            getScreentPos(displayPos, world_point, this);

            curP = pcl::PointXYZRGB(world_point[0], world_point[1], world_point[2],255,255,255);
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
    te3DCanvas* canvas = static_cast<te3DCanvas*> (viewer_void);
    vtkRenderer* renderer{ canvas->viewer->getRendererCollection()->GetFirstRenderer() };
    double fp[4], tmp1[4], eventFPpos[4];
    renderer->GetActiveCamera()->GetFocalPoint(fp);
    fp[3] = 0.0;
    renderer->SetWorldPoint(fp);
    renderer->WorldToDisplay();
    renderer->GetDisplayPoint(tmp1);

    tmp1[0] = displayPos[0];
    tmp1[1] = displayPos[1];

    renderer->SetDisplayPoint(tmp1);
    renderer->DisplayToWorld();

    renderer->GetWorldPoint(eventFPpos);
    for (int i = 0; i < 3; i++)
    {
        world[i] = eventFPpos[i];
    }
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

/// <summary>
/// 不规则框选
/// </summary>
/// <param name="viewer_void"></param>
void te3DCanvas::PolygonSelect(void* viewer_void)
{
    te3DCanvas* canvas = static_cast<te3DCanvas*> (viewer_void);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn_Prj(new pcl::PointCloud<pcl::PointXYZRGB>);//输入的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCiecle_result(new pcl::PointCloud<pcl::PointXYZRGB>);//绘制的线
#ifdef _Interactor_
    double A = canvas->m_CustomInteractor->getXActor()[1] * canvas->m_CustomInteractor->getYActor()[2] - canvas->m_CustomInteractor->getXActor()[2] * canvas->m_CustomInteractor->getYActor()[1];
    double B = canvas->m_CustomInteractor->getXActor()[2] * canvas->m_CustomInteractor->getYActor()[0] - canvas->m_CustomInteractor->getXActor()[0] * canvas->m_CustomInteractor->getYActor()[2];
    double C = canvas->m_CustomInteractor->getXActor()[0] * canvas->m_CustomInteractor->getYActor()[1] - canvas->m_CustomInteractor->getXActor()[1] * canvas->m_CustomInteractor->getYActor()[0];

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = A;
    coefficients->values[1] = B;
    coefficients->values[2] = C;
    coefficients->values[3] = 0;

    pcl::ProjectInliers<pcl::PointXYZRGB> projCloudIn;
    projCloudIn.setModelType(pcl::SACMODEL_PLANE);
    projCloudIn.setInputCloud(canvas->cloud);
    projCloudIn.setModelCoefficients(coefficients);
    projCloudIn.filter(*cloudIn_Prj);

    vtkSmartPointer<vtkMatrix4x4> vtk_matrix = canvas->m_CustomInteractor->m_pRotationTransform->GetMatrix();
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            transform(i, j) = static_cast<float>(vtk_matrix->GetElement(i, j));
        }
    }
    pcl::transformPointCloud(*cloudIn_Prj, *cloudIn_Prj, transform);
#else
    double focal[3] = { 0 }; double pos[3] = { 0 };
    vtkRenderer* renderer{ canvas->viewer->getRendererCollection()->GetFirstRenderer() };
    renderer->GetActiveCamera()->GetFocalPoint(focal);
    renderer->GetActiveCamera()->GetPosition(pos);
    pcl::PointXYZ eyeLine1 = pcl::PointXYZ(focal[0] - pos[0], focal[1] - pos[1], focal[2] - pos[2]);
    float mochang = sqrt(pow(eyeLine1.x, 2) + pow(eyeLine1.y, 2) + pow(eyeLine1.z, 2));
    pcl::PointXYZ eyeLine = pcl::PointXYZ(eyeLine1.x / mochang, eyeLine1.y / mochang, eyeLine1.z / mochang);
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    coefficients->values.resize(4);
    coefficients->values[0] = eyeLine.x;
    coefficients->values[1] = eyeLine.y;
    coefficients->values[2] = eyeLine.z;
    coefficients->values[3] = 0;
#endif
    double focal[3] = { 0 }; double pos[3] = { 0 };
    vtkRenderer* renderer{ canvas->viewer->getRendererCollection()->GetFirstRenderer() };
    renderer->GetActiveCamera()->GetFocalPoint(focal);
    renderer->GetActiveCamera()->GetPosition(pos);
    pcl::PointXYZ eyeLine1 = pcl::PointXYZ(focal[0] - pos[0], focal[1] - pos[1], focal[2] - pos[2]);
    float mochang = sqrt(pow(eyeLine1.x, 2) + pow(eyeLine1.y, 2) + pow(eyeLine1.z, 2));
    pcl::PointXYZ eyeLine = pcl::PointXYZ(eyeLine1.x / mochang, eyeLine1.y / mochang, eyeLine1.z / mochang);

    pcl::ModelCoefficients::Ptr coefficients_r(new pcl::ModelCoefficients());
    coefficients_r->values.resize(4);
    coefficients_r->values[0] = eyeLine.x;
    coefficients_r->values[1] = eyeLine.y;
    coefficients_r->values[2] = eyeLine.z;
    coefficients_r->values[3] = 0;

    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(canvas->cloud_polygon);
    proj.setModelCoefficients(coefficients_r);
    proj.filter(*cloudCiecle_result);

    //canvas->viewer->addPointCloud(cloudCiecle_result, "cloudCiecle_result");
    //canvas->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloudCiecle_result");
    //canvas->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 40, "cloudCiecle_result");
    //canvas->viewer->addPointCloud(cloudIn_Prj, "cloudIn_Prj");
    //canvas->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloudIn_Prj");
    //canvas->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "cloudIn_Prj");
    //canvas->viewer->getRenderWindow()->Render();

    double* PloyXarr = new double[cloudCiecle_result->points.size()];
    double* PloyYarr = new double[cloudCiecle_result->points.size()];
    for (int i = 0; i < cloudCiecle_result->points.size(); ++i)
    {
        PloyXarr[i] = cloudCiecle_result->points[i].x;
        PloyYarr[i] = cloudCiecle_result->points[i].y;
    }

    canvas->cloud_cliped->clear();
    int ret = -1;
    for (int i = 0; i < cloudIn_Prj->points.size(); i++)
    {
        ret = inOrNot1(canvas->cloud_polygon->points.size(), PloyXarr, PloyYarr, cloudIn_Prj->points[i].x, cloudIn_Prj->points[i].y);
        if (1 == ret)//表示在里面
        {
            canvas->cloud_cliped->points.push_back(canvas->cloud->points[i]);
        }//表示在外面
    }
    //canvas->viewer->removeAllPointClouds();
    //canvas->cloud->clear();
    //pcl::copyPointCloud(*canvas->cloud_cliped, *canvas->cloud);
    //canvas->viewer->addPointCloud(canvas->cloud, "cloud");
    //canvas->viewer->getRenderWindow()->Render();

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(canvas->currentColor.red(), canvas->currentColor.green(), canvas->currentColor.blue());
    QString CloudId;
    auto it = canvas->markerPCID.find(teDataStorage::getInstance()->getCurrentLabelCategory());
    if (it != canvas->markerPCID.end()) {
        CloudId = teDataStorage::getInstance()->getCurrentLabelCategory() + "marker" + QString::number(it->second.size());
        it->second.push_back(CloudId);
    }
    else {
        CloudId = teDataStorage::getInstance()->getCurrentLabelCategory() + "marker" + "0";
        canvas->markerPCID.insert(std::make_pair(teDataStorage::getInstance()->getCurrentLabelCategory(), std::vector<QString>{CloudId}));
    }
    //pcl::transformPointCloud(*canvas->cloud_cliped, *canvas->cloud_cliped, transform);
    //teDataStorage::getInstance()->updateMarkersNumber();
    canvas->viewer->addPointCloud(canvas->cloud_cliped, currentColor, CloudId.toStdString());
    emit canvas->sig_3DCanvasMarkingCompleted(canvas->cloud_cliped);
    canvas->m_renderWindow->Render();
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
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(fileName.toStdString(), *cloud) == -1)
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

bool te3DCanvas::SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud)
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

void te3DCanvas::reRenderOriginCloud(ReRenderMode mode)
{
    reRendering(cloud,mode);
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
    reRenderOriginCloud(ReSetCamera);
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Elevation_rendering;
    cloud_Elevation_rendering = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();

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
    cloud->clear();
    pcl::copyPointCloud(*cloud_Elevation_rendering, *cloud);
    viewer->updatePointCloud(cloud, "cloud");
    m_renderWindow->Render();
    return true;
}

bool te3DCanvas::PointCloudColorSet(QColor color)
{
    QColor temp;
    temp.setRgb(143, 153, 159, 255);
    if (!cloud->empty() && (color != temp)) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>selected_color(cloud, color.redF() * 255, color.greenF() * 255, color.blueF() * 255);
        viewer->updatePointCloud(cloud, selected_color, "cloud");
        m_renderWindow->Render();
    }
    else {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>selected_color(cloud, 255, 255, 255);
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

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(cloud_marked, color.red(), color.green(), color.blue());
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

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(cloud_marked, color.red(), color.green(), color.blue());
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

void te3DCanvas::reRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin,ReRenderMode mode)
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudin, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    if (mode == ReSetCamera) {
        // 计算点云中心位置和对角线长度
        Eigen::Vector3f center((axisset.maxPt.x + axisset.minPt.x) / 2, (axisset.maxPt.y + axisset.minPt.y) / 2, (axisset.maxPt.z + axisset.minPt.z) / 2);
        Eigen::Vector3f diff = axisset.maxPt.getVector3fMap() - axisset.minPt.getVector3fMap();
        float distance = diff.norm();

        viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), 0, 0, 0); //耗时最多 2秒多
    }

    //viewer->spinOnce();
    m_renderWindow->Render();
    setRotationCenter();

#ifdef _Interactor_
    m_CustomInteractor->ResetData();
#endif
    
    SetCoordinateSet();
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
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*cloud, *cloud_xyz);
    pcl::getMinMax3D(*cloud_xyz, min, max);
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

#ifdef _Interactor_
    m_CustomInteractor = CustomInteractorStyle::New();
    m_CustomInteractor->setRenderWindow(m_renderWindow, m_renderer, axes_actor);
    m_renderWindow->GetInteractor()->SetInteractorStyle(m_CustomInteractor);
#endif

    markerWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    markerWidget->SetOrientationMarker(axes_actor);
    markerWidget->SetInteractor(m_renderWindow->GetInteractor());
    markerWidget->SetEnabled(1);
    markerWidget->SetInteractive(false);

    m_renderWindow->Render();
}

void te3DCanvas::AxisAlignedBoundingBox()
{
    pcl::PointXYZRGB min_point_AABB;
    pcl::PointXYZRGB max_point_AABB;

    pcl::getMinMax3D(*cloud, min_point_AABB, max_point_AABB);

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
void te3DCanvas::subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
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

void te3DCanvas::PerspectiveToYaxis()
{
    pcl::PointXYZRGB minPt, maxPt;
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
    pcl::PointXYZRGB minPt, maxPt;
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
    pcl::PointXYZRGB minPt, maxPt;
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
        pcl::copyPointCloud(*cloud_Filter_out, *cloud);

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->addPointCloud(cloud, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->resetCamera();
        m_renderWindow->Render();
    }
}

void te3DCanvas::pcl_filter_guass(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float paraA, float paraB, float paraC, float paraD)
{
    pcl::filters::GaussianKernel<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZRGB, pcl::PointXYZRGB>);
    (*kernel).setSigma(paraA);
    (*kernel).setThresholdRelativeToSigma(paraB);
    //KdTree加速
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    (*kdtree).setInputCloud(cloud_in);
    pcl::filters::Convolution3D <pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::filters::GaussianKernel<pcl::PointXYZRGB, pcl::PointXYZRGB> > convolution;
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
        pcl::copyPointCloud(*cloud_Filter_out, *cloud);

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
    for (auto& point : cloud->points) {
        point.z = point.z * factor;
    }
    reRenderOriginCloud(ReSetCamera);
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

void te3DCanvas::pcl_filter_direct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float min, float max, QString axis, float is_save)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;//设置滤波器对象

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
#ifdef _Interactor_
    m_CustomInteractor->setRotationCenter(getCloudCentroid()[0], getCloudCentroid()[1], getCloudCentroid()[2]);
#endif
}
