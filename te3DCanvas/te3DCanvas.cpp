#include "te3DCanvas.h"
#include <QMessageBox>
#include <QDebug>
#include <QWheelEvent>
#include <QEvent>
#include <QElapsedTimer>
#include <QRegularExpression>

#include "Depth2RGB.h"

#define DEBUG

#ifdef DEBUG
#include <QTime>
#endif
#include "Transfer_Function.h"
#include "teDataStorage.h"

QString incrementNumber(const QString& input) 
{
    QRegularExpression regex("(\\d+)$");
    QRegularExpressionMatch match = regex.match(input);
    if (match.hasMatch()) {
        QString numberStr = match.captured(1);
        int number = numberStr.toInt();
        number++;
        return QString::number(number);
    }
    return QString();
}

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
    
    cloud_Filter_out = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();
    
    
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
void te3DCanvas::PolygonSelect()
{
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", u8"无点云输入");
        return;
    }
    double* FBRange = m_renderer->GetActiveCamera()->GetClippingRange();

    double* PloyXarr = new double[MarkerPointSet.size()];
    double* PloyYarr = new double[MarkerPointSet.size()];
    for (int i = 0; i < MarkerPointSet.size(); ++i)
    {
        PloyXarr[i] = MarkerPointSet[i].x();
        PloyYarr[i] = MarkerPointSet[i].y();
    }
    const auto& mat = m_renderer->GetActiveCamera()->GetCompositeProjectionTransformMatrix(m_renderer->GetTiledAspectRatio(), FBRange[0], FBRange[1]);
    const auto& transmat = m_CustomInteractor->m_pRotationTransform->GetMatrix();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cliped = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();
    for (int i = 0; i < cloud->points.size(); ++i)
    {
        pcl::PointXYZRGB P3D = cloud->points.at(i);
        double P2D[2];
        WorldToScreen(&P3D, transmat, mat, P2D);
        if (inOrNot1(MarkerPointSet.size(), PloyXarr, PloyYarr, P2D[0], P2D[1]))
        {
            cloud_cliped->points.push_back(cloud->points.at(i));
        }
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(cloud_cliped, currentColor.red(), currentColor.green(), currentColor.blue());
    QString CloudId;
    QString test = teDataStorage::getInstance()->getCurrentLabelCategory();
    auto it = markerPCID.find(teDataStorage::getInstance()->getCurrentLabelCategory());
    auto pct = resultPointCloud.find(teDataStorage::getInstance()->getCurrentLabelCategory());
    if (it != markerPCID.end()) {
        QString count = incrementNumber(it->second.back());
        CloudId = teDataStorage::getInstance()->getCurrentLabelCategory() + QString::number(teDataStorage::getInstance()->getCurrentIndex()) + "marker" + count;
        it->second.push_back(CloudId);
        pct->second.push_back(cloud_cliped);
    }
    else {
        CloudId = teDataStorage::getInstance()->getCurrentLabelCategory() + QString::number(teDataStorage::getInstance()->getCurrentIndex()) + "marker" + "0";
        markerPCID.insert(std::make_pair(teDataStorage::getInstance()->getCurrentLabelCategory(), std::vector<QString>{CloudId}));
        markerPointCloud.insert(std::make_pair(teDataStorage::getInstance()->getCurrentLabelCategory(), std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{cloud_cliped}));
    }
    viewer->addPointCloud(cloud_cliped, currentColor, CloudId.toStdString());

    vtkSmartPointer<vtkPropCollection> propCollection = m_renderer->GetViewProps();
    vtkProp3D* pActor = vtkProp3D::SafeDownCast(propCollection->GetLastProp());
    pActor->SetUserTransform(m_CustomInteractor->m_pRotationTransform);

    emit sig_3DCanvasMarkingCompleted(cloud_cliped);

    m_renderWindow->Render();
    teDataStorage::getInstance()->updateTrainWidget(teDataStorage::getInstance()->getCurrentTrainMarksNumber());
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

    //SetCoordinateSet();
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

void te3DCanvas::UpdateDimentsion()
{
    auto itStr = markerPCID.begin();
    auto itPC = markerPointCloud.begin();
    for (itStr, itPC; itStr != markerPCID.end(); ++itStr, ++itPC)
    {
        assert(itStr->first == itPC->first);
        for (int i = 0; i < itStr->second.size(); ++i)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(itPC->second.at(i), currentColor.red(), currentColor.green(), currentColor.blue());
            viewer->updatePointCloud(itPC->second.at(i), currentColor, itStr->second.at(i).toStdString());
        }
    }
}

void te3DCanvas::UpdateResult()
{
    auto itStr = resultPCID.begin();
    auto itPC = resultPointCloud.begin();
    for (itStr, itPC; itStr != resultPCID.end(); ++itStr, ++itPC)
    {
        assert(itStr->first == itPC->first);
        for (int i = 0; i < itStr->second.size(); ++i)
        {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(itPC->second.at(i), currentColor.red(), currentColor.green(), currentColor.blue());
            viewer->updatePointCloud(itPC->second.at(i), currentColor, itStr->second.at(i).toStdString());
        }
    }
}

void te3DCanvas::ReductionPointCloud()
{
    if (teDataStorage::getInstance()->getCurrentLoadImageNum() != 0)
    {
        LoadPointCloud(QString::fromStdString(teDataStorage::getInstance()->getCurrentPointCloud()));
        reRenderOriginCloud(ReSetCamera);
        emit sig_ShowAllItems();
    }
}

bool te3DCanvas::SetBackgroundColor(QColor color)
{
    viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
    m_renderWindow->Render();
    return true;
}

bool te3DCanvas::CoordinateAxisRendering(QString curaxis)
{
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", u8"无点云输入");
        return false;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Elevation_rendering(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 根据curaxis选择处理的坐标轴及其范围
    double min, max;
    if (curaxis == "z")
    {
        min = teDataStorage::getInstance()->getCurrentInvalidPointThreshold();
        max = teDataStorage::getInstance()->getCurrentValidPointThreshold();
    }
    else if (curaxis == "x")
    {
        min = axisset.minPt.x;
        max = axisset.maxPt.x;
    }
    else if (curaxis == "y")
    {
        min = axisset.minPt.y;
        max = axisset.maxPt.y;
    }

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
    viewer->updatePointCloud(cloud, "cloud");
    AutomaticallyAdjustCamera();
    m_renderWindow->Render();
    return true;
}

bool te3DCanvas::PointCloudColorSet(QColor color)
{
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", u8"无点云输入");
        return false;
    }
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
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", u8"无点云输入");
        return false;
    }
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_marked = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();

    Transfer_Function::ExtractCloud2Cloud(cloud, &contour, cloud_marked);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(cloud_marked, color.red(), color.green(), color.blue());
    QString CloudId;
    auto it = markerPCID.find(teDataStorage::getInstance()->getCurrentLabelCategory());
    auto pct = markerPointCloud.find(teDataStorage::getInstance()->getCurrentLabelCategory());
    if (it != markerPCID.end()) {
        QString count = incrementNumber(it->second.back());
        CloudId = QString::fromStdString(instance->name) + QString::number(teDataStorage::getInstance()->getCurrentIndex()) + "marker" + count;
        qDebug() << "CloudID: " << CloudId;
        it->second.push_back(CloudId);
        pct->second.push_back(cloud_marked);
    }
    else {
        CloudId = QString::fromStdString(instance->name) + QString::number(teDataStorage::getInstance()->getCurrentIndex()) + "marker" + "0";
        markerPCID.insert(std::make_pair(teDataStorage::getInstance()->getCurrentLabelCategory(), std::vector<QString>{CloudId}));
        markerPointCloud.insert(std::make_pair(teDataStorage::getInstance()->getCurrentLabelCategory(), std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{cloud_marked}));
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_marked = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();

    Transfer_Function::ExtractCloud2Cloud(cloud, &contour, cloud_marked);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(cloud_marked, color.red(), color.green(), color.blue());
    QString CloudId;
    auto it = resultPCID.find(teDataStorage::getInstance()->getCurrentLabelCategory());
    auto pct = resultPointCloud.find(teDataStorage::getInstance()->getCurrentLabelCategory());
    if (it != resultPCID.end()) {
        QString count = incrementNumber(it->second.back());
        CloudId = QString::fromStdString(instance->name) + QString::number(teDataStorage::getInstance()->getCurrentIndex()) + "result" + count;
        it->second.push_back(CloudId);
        pct->second.push_back(cloud_marked);
    }
    else {
        CloudId = QString::fromStdString(instance->name) + QString::number(teDataStorage::getInstance()->getCurrentIndex()) + "result" + "0";
        resultPCID.insert(std::make_pair(teDataStorage::getInstance()->getCurrentLabelCategory(), std::vector<QString>{CloudId}));
        resultPointCloud.insert(std::make_pair(teDataStorage::getInstance()->getCurrentLabelCategory(), std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{cloud_marked}));
    }
    viewer->addPointCloud(cloud_marked, currentColor, CloudId.toStdString());
    m_renderWindow->Render();
}

void te3DCanvas::reRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin,ReRenderMode mode)
{
    viewer->removeAllPointClouds();
    
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudin, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, "cloud");
    if (mode == ReSetCamera)
        AutomaticallyAdjustCamera();

    m_renderWindow->Render();
    setRotationCenter();

#ifdef _Interactor_
    m_CustomInteractor->ResetData();
#endif
    
    SetCoordinateSet();
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

vtkSmartPointer<vtkPolyData> te3DCanvas::createPlane(const pcl::ModelCoefficients& coefficients, float scale[2])
{
    vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();

    plane->SetNormal(coefficients.values[0], coefficients.values[1], coefficients.values[2]);
    double norm_sqr = coefficients.values[0] * coefficients.values[0]
        + coefficients.values[1] * coefficients.values[1]
        + coefficients.values[2] * coefficients.values[2];


    plane->Push(-coefficients.values[3] / sqrt(norm_sqr));
    plane->SetResolution(200, 200);
    plane->Update();

    double pt1[3], pt2[3], orig[3], center[3];
    plane->GetPoint1(pt1);
    plane->GetPoint2(pt2);
    plane->GetOrigin(orig);
    plane->GetCenter(center);

    double _pt1[3], _pt2[3];
    float scale1 = 3.0;
    float scale2 = 3.0;
    if (scale != nullptr)
    {
        scale1 = scale[0];
        scale2 = scale[1];
    }
    for (int i = 0; i < 3; i++) {
        _pt1[i] = scale1 * (pt1[i] - orig[i]);
        _pt2[i] = scale2 * (pt2[i] - orig[i]);
    }
    for (int i = 0; i < 3; ++i)
    {
        pt1[i] = orig[i] + _pt1[i];
        pt2[i] = orig[i] + _pt2[i];
    }
    plane->SetPoint1(pt1);
    plane->SetPoint2(pt2);

    plane->Update();
    return (plane->GetOutput());
}

void te3DCanvas::WorldToScreen(pcl::PointXYZRGB* input3D, vtkMatrix4x4* mat, double* output2D)
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

    if (m_renderer->GetVTKWindow())
    {
        double dx, dy;
        int sizex, sizey;

        const int* size = m_renderer->GetVTKWindow()->GetSize();
        if (!size)
        {
            return;
        }
        sizex = size[0];
        sizey = size[1];

        dx = (input3D->x + 1.0) * (sizex * (m_renderer->GetViewport()[2] - m_renderer->GetViewport()[0])) / 2.0 +
            sizex * m_renderer->GetViewport()[0];
        dy = (input3D->y + 1.0) * (sizey * (m_renderer->GetViewport()[3] - m_renderer->GetViewport()[1])) / 2.0 +
            sizey * m_renderer->GetViewport()[1];

        output2D[0] = dx;
        output2D[1] = dy;
    }
}

void te3DCanvas::WorldToScreen(pcl::PointXYZRGB* input3D, vtkMatrix4x4* transform, vtkMatrix4x4* composit, double* output2D)
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

    if (m_renderer->GetVTKWindow())
    {
        double dx, dy;
        int sizex, sizey;

        const int* size = m_renderer->GetVTKWindow()->GetSize();
        if (!size)
        {
            return;
        }
        sizex = size[0];
        sizey = size[1];

        dx = (input3D->x + 1.0) * (sizex * (m_renderer->GetViewport()[2] - m_renderer->GetViewport()[0])) / 2.0 +
            sizex * m_renderer->GetViewport()[0];
        dy = (input3D->y + 1.0) * (sizey * (m_renderer->GetViewport()[3] - m_renderer->GetViewport()[1])) / 2.0 +
            sizey * m_renderer->GetViewport()[1];

        output2D[0] = dx;
        output2D[1] = dy;
    }
}

void te3DCanvas::AxisAlignedBoundingBox()
{
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", u8"无点云输入");
        return;
    }
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
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", u8"无点云输入");
        return;
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
    if (teDataStorage::getInstance()->getCurrentLoadImageNum() != 0)
    {
        Eigen::Vector3f center((axisset.maxPt.x + axisset.minPt.x) / 2, (axisset.maxPt.y + axisset.minPt.y) / 2, (axisset.maxPt.z + axisset.minPt.z) / 2);
        Eigen::Vector3f diff = axisset.maxPt.getVector3fMap() - axisset.minPt.getVector3fMap();
        float distance = diff.norm();

        if (m_member.PositiveAndNegative_Y_axis)
            viewer->setCameraPosition(center(0), center(1) + distance, center(2), center(0), center(1), center(2), 1, 0, 0);
        else
            viewer->setCameraPosition(center(0), center(1) + distance, center(2), center(0), center(1), center(2), -1, 0, 0);
        m_member.PositiveAndNegative_Y_axis = !m_member.PositiveAndNegative_Y_axis;
        viewer->updateCamera();
        viewer->spinOnce();
    }
}

void te3DCanvas::PerspectiveToXaxis()
{
    if (teDataStorage::getInstance()->getCurrentLoadImageNum() != 0)
    {
        Eigen::Vector3f center((axisset.maxPt.x + axisset.minPt.x) / 2, (axisset.maxPt.y + axisset.minPt.y) / 2, (axisset.maxPt.z + axisset.minPt.z) / 2);
        Eigen::Vector3f diff = axisset.maxPt.getVector3fMap() - axisset.minPt.getVector3fMap();
        float distance = diff.norm();

        if (m_member.PositiveAndNegative_X_axis)
            viewer->setCameraPosition(center(0) + distance, center(1), center(2), center(0), center(1), center(2), 1, 0, 0);
        else
            viewer->setCameraPosition(center(0) + distance, center(1), center(2), center(0), center(1), center(2), -1, 0, 0);
        m_member.PositiveAndNegative_X_axis = !m_member.PositiveAndNegative_X_axis;
        viewer->updateCamera();
        viewer->spinOnce();
    }
}

void te3DCanvas::PerspectiveToZaxis()
{
    if (teDataStorage::getInstance()->getCurrentLoadImageNum() != 0)
    {
        Eigen::Vector3f center((axisset.maxPt.x + axisset.minPt.x) / 2, (axisset.maxPt.y + axisset.minPt.y) / 2, (axisset.maxPt.z + axisset.minPt.z) / 2);
        Eigen::Vector3f diff = axisset.maxPt.getVector3fMap() - axisset.minPt.getVector3fMap();
        float distance = diff.norm();

        if (m_member.PositiveAndNegative_Z_axis)
            viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), 1, 0, 0);
        else
            viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), -1, 0, 0);
        m_member.PositiveAndNegative_Z_axis = !m_member.PositiveAndNegative_Z_axis;
        viewer->updateCamera();
        viewer->spinOnce();
    }
}

void te3DCanvas::GuassFilter(QString data1, QString data2, QString data3, QString data4)
{
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", u8"无点云输入");
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
        QMessageBox::warning(this, "Warning", u8"无点云输入");
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

void te3DCanvas::HeightTransform(int factor)
{
    if (factor != 1 && !cloud->empty())
    {
        for (auto& point : cloud->points) {
            point.z = point.z * factor;
        }
    }
    reRenderOriginCloud(ReSetCamera);
}

void te3DCanvas::te3DCanvasStartMarking(QVector<QPointF>& pointlist)
{
    MarkerPointSet.clear();
    for (QPointF point : pointlist) {
        QPointF p;
        p.setX(point.x());
        p.setY(this->height() - point.y() - 1);
        MarkerPointSet.push_back(p);
    }
    PolygonSelect();
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

void te3DCanvas::AutomaticallyAdjustCamera()
{
    // 计算点云中心位置和对角线长度
    Eigen::Vector3f center((axisset.maxPt.x + axisset.minPt.x) / 2, (axisset.maxPt.y + axisset.minPt.y) / 2, (axisset.maxPt.z + axisset.minPt.z) / 2);
    Eigen::Vector3f diff = axisset.maxPt.getVector3fMap() - axisset.minPt.getVector3fMap();
    float distance = diff.norm();

    viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), 0, 0, 0); //耗时最多 2秒多
}
