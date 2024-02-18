#include "VTKOpenGLNativeWidget.h"
#include <QMessageBox>
#include <QDebug>
#include "Transfer_Function.h"

VTKOpenGLNativeWidget::VTKOpenGLNativeWidget(QWidget *parent)
	: QVTKOpenGLNativeWidget(parent)
{
	PCL_Initalization();
}

VTKOpenGLNativeWidget::~VTKOpenGLNativeWidget()
{}

std::string rand_str(const int len)  /*����Ϊ�ַ����ĳ���*/
{
    /*��ʼ��*/
    std::string str;                 /*����������������ַ�����str*/
    char c;                     /*�����ַ�c����������������ɵ��ַ�*/
    int idx;                    /*����ѭ���ı���*/
    /*ѭ�����ַ��������������ɵ��ַ�*/
    for (idx = 0; idx < len; idx++)
    {
        /*rand()%26��ȡ�࣬����Ϊ0~25����'a',������ĸa~z,���asc���*/
        c = 'a' + rand() % 26;
        str.push_back(c);       /*push_back()��string��β�庯���������������ַ�c*/
    }
    return str;                 /*�������ɵ�����ַ���*/
}

void VTKOpenGLNativeWidget::PCL_Initalization()
{
    cloud = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    Point_clicked_cloud = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    Frame_clicked_cloud = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    cloud_polygon = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    cloud_cliped = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    cloud_Filter_out = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    cloud_marked = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();
    mediancloud = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();

    m_renderer = vtkSmartPointer<vtkRenderer>::New();
    m_renderWindow = this->renderWindow();
    m_renderWindow->AddRenderer(m_renderer);

    viewer.reset(new pcl::visualization::PCLVisualizer(m_renderer, m_renderWindow, "viewer", false));
    this->setRenderWindow(m_renderWindow);
    viewer->setupInteractor(m_renderWindow->GetInteractor(), m_renderWindow);
    viewer->setBackgroundColor(0.0, 0.3, 0.4);
    viewer->addCoordinateSystem(1.0);
    this->update();
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    viewer->registerAreaPickingCallback(&VTKOpenGLNativeWidget::Frame_PickingCallBack, *this);
    viewer->registerPointPickingCallback(&VTKOpenGLNativeWidget::Point_PickingCallBack, *this);
    viewer->registerKeyboardCallback(&VTKOpenGLNativeWidget::keyboardEventOccurred, *this);
    viewer->registerKeyboardCallback(&VTKOpenGLNativeWidget::keyboardEventInvert, *this);
    viewer->registerMouseCallback(&VTKOpenGLNativeWidget::mouseEventOccurred, *this);

    currentdynamicLabel = nullptr;

    PositiveAndNegative_X_axis = true;
    PositiveAndNegative_Y_axis = true;
    PositiveAndNegative_Z_axis = true;
}

/// <summary>
/// �����ѡ�Ļص�����
/// </summary>
/// <param name="event"></param>
/// <param name="viewer_void"></param>
void VTKOpenGLNativeWidget::Frame_PickingCallBack(const pcl::visualization::AreaPickingEvent& event, void* viewer_void)
{
    std::vector< int > indices;
    if (event.getPointsIndices(indices) == -1)
        return;

    for (int i = 0; i < indices.size(); ++i)
    {
        Frame_clicked_cloud->points.push_back(cloud->points.at(indices[i]));
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(Frame_clicked_cloud, 255, 0, 0);

    viewer->removeShape("Frame_clicked_cloud");
    viewer->addPointCloud(Frame_clicked_cloud, red, "Frame_clicked_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Frame_clicked_cloud");
    m_renderWindow->Render();
}

/// <summary>
/// ȷ�Ͽ�ѡ����
/// </summary>
void VTKOpenGLNativeWidget::ComfirmFramePick()
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(cloud, 255, 255, 255);

    pcl::copyPointCloud(*Frame_clicked_cloud, *cloud); //����ѡ�ĵ��Ƹ�ֵ��Ҫ��ʾ�ĵ���
    Frame_clicked_cloud->points.clear();
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud(cloud, white, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    m_renderWindow->Render();
}

/// <summary>
/// �����ѡ�Ļص�����
/// </summary>
/// <param name="event"></param>
/// <param name="viewer_void"></param>
void VTKOpenGLNativeWidget::Point_PickingCallBack(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    if (event.getPointIndex() == -1)
        return;
    pcl::PointXYZ current_point;
    event.getPoint(current_point.x, current_point.y, current_point.z);
    Point_clicked_cloud->points.push_back(current_point);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(Point_clicked_cloud, 255, 0, 0);
    viewer->removePointCloud("Point_clicked_cloud");
    viewer->addPointCloud(Point_clicked_cloud, red, "Point_clicked_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Point_clicked_cloud");

    QString ptext = QString("X:%1 Y:%2 Z:%3").arg(current_point.x).arg(current_point.y).arg(current_point.z);
}

/// <summary>
/// ȷ�ϵ�ѡ����
/// </summary>
void VTKOpenGLNativeWidget::ComfirmPointPick()
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(cloud, 255, 255, 255);

    pcl::copyPointCloud(*Point_clicked_cloud, *cloud);
    Point_clicked_cloud->points.clear();
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    viewer->addPointCloud(cloud, white, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    m_renderWindow->Render();
}

/// <summary>
/// �������ѡ����C�����ã��ٰ�Cȷ�Ͽ�ѡ����
/// </summary>
/// <param name="event"></param>
/// <param name="viewer_void"></param>
void VTKOpenGLNativeWidget::keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
    if (event.getKeySym() == "c" && event.keyDown()) {
        isPickingMode = !isPickingMode;
        if (isPickingMode) {
            line_id = 0;
            cloud_polygon->clear();
            flag = false;
        }
        else {
            PolygonSelect(viewer_void, "Primary");
            viewer->removeAllShapes();
        }
    }
}

/// <summary>
/// �����򷴿�ѡ����v�����ã��ٰ�vȷ�Ϸ���ѡ����
/// </summary>
/// <param name="event"></param>
/// <param name="viewer_void"></param>
void VTKOpenGLNativeWidget::keyboardEventInvert(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
    if (event.getKeySym() == "v" && event.keyDown()) {
        isPickingMode = !isPickingMode;
        if (isPickingMode) {
            line_id = 0;
            cloud_polygon->clear();
            flag = false;
        }
        else {
            PolygonSelect(viewer_void, "Invert");
            viewer->removeAllShapes();
        }
    }
}

/// <summary>
/// �������ѡ����껭��
/// </summary>
/// <param name="event"></param>
/// <param name="viewer_void"></param>
void VTKOpenGLNativeWidget::mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* viewer_void)
{
    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
        event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        if (isPickingMode) {
            double world_point[3];
            double displayPos[2];
            displayPos[0] = double(event.getX()), displayPos[1] = double(event.getY());
            getScreentPos(displayPos, world_point, viewer_void);

            curP = pcl::PointXYZ(world_point[0], world_point[1], world_point[2]);
            if (!flag)flag = true;
            else {
                char str1[512];
                sprintf(str1, "line#%03d", line_id++);
                viewer->addLine(lastP, curP, str1);
            }
            lastP = curP;
            cloud_polygon->push_back(curP);
        }
    }
}

/**
 * @brief getScreentPos     ��Ļ����ת������������
 * @param displayPos        ���룺��Ļ����
 * @param world             �������������
 * @param viewer_void       ���룺pclViewer
 */
void VTKOpenGLNativeWidget::getScreentPos(double* displayPos, double* world, void* viewer_void)
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
/// �������ѡ
/// </summary>
/// <param name="viewer_void"></param>
/// <param name="mode">ģʽѡ��:Primary,Invert,Label1,Label2,Label3</param>
void VTKOpenGLNativeWidget::PolygonSelect(void* viewer_void, QString mode)
{
    double focal[3] = { 0 }; double pos[3] = { 0 };
    m_renderer->GetActiveCamera()->GetFocalPoint(focal);
    m_renderer->GetActiveCamera()->GetPosition(pos);

    pcl::PointXYZ eyeLine1 = pcl::PointXYZ(focal[0] - pos[0], focal[1] - pos[1], focal[2] - pos[2]);

    float mochang = sqrt(pow(eyeLine1.x, 2) + pow(eyeLine1.y, 2) + pow(eyeLine1.z, 2));//ģ��
    pcl::PointXYZ eyeLine = pcl::PointXYZ(eyeLine1.x / mochang, eyeLine1.y / mochang, eyeLine1.z / mochang);//��λ���� ������

    //����һ��ƽ��
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());//ax+by+cz+d=0
    coefficients->values.resize(4);
    coefficients->values[0] = eyeLine.x;// ������ x ����
    coefficients->values[1] = eyeLine.y;// ������ y ����
    coefficients->values[2] = eyeLine.z;// ������ z ����
    coefficients->values[3] = 0;        // ƽ��ƫ����

    //����������ͶӰ�ĵ���
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn_Prj(new pcl::PointCloud<pcl::PointXYZ>);//����ĵ���
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCiecle_result(new pcl::PointCloud<pcl::PointXYZ>);//���Ƶ���
    // �����˲�������
    pcl::ProjectInliers<pcl::PointXYZ> proj;//����ͶӰ����
    proj.setModelType(pcl::SACMODEL_PLANE);//����ͶӰ����
    proj.setInputCloud(cloud_polygon);//�����������
    proj.setModelCoefficients(coefficients);//����ͶӰ����
    proj.filter(*cloudCiecle_result);//ִ�г��򣬲����������

    // �����˲�������
    pcl::ProjectInliers<pcl::PointXYZ> projCloudIn;//����ͶӰ����
    projCloudIn.setModelType(pcl::SACMODEL_PLANE);//����ͶӰ����
    projCloudIn.setInputCloud(cloud);//�����������
    projCloudIn.setModelCoefficients(coefficients);//����ͶӰ����
    projCloudIn.filter(*cloudIn_Prj);//ִ�г��򣬲����������

    int ret = -1;
    double* PloyXarr = new double[cloudCiecle_result->points.size()];
    double* PloyYarr = new double[cloudCiecle_result->points.size()];
    for (int i = 0; i < cloudCiecle_result->points.size(); ++i)
    {
        PloyXarr[i] = cloudCiecle_result->points[i].x;
        PloyYarr[i] = cloudCiecle_result->points[i].y;
    }


    if(mode == "Primary") 
    {
        cloud_cliped->clear();
        for (int i = 0; i < cloudIn_Prj->points.size(); i++)
        {
            ret = inOrNot1(cloud_polygon->points.size(), PloyXarr, PloyYarr, cloudIn_Prj->points[i].x, cloudIn_Prj->points[i].y);
            if (1 == ret)//��ʾ������
            {
                cloud_cliped->points.push_back(cloud->points[i]);
            }//��ʾ������
        }
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        cloud->clear();
        pcl::copyPointCloud(*cloud_cliped, *cloud);
        viewer->addPointCloud(cloud, "aftercut");
        m_renderWindow->Render();
        //Display_Properites();
    }
    else if(mode == "Invert") {
        cloud_cliped->clear();
        for (int i = 0; i < cloudIn_Prj->points.size(); i++)
        {
            ret = inOrNot1(cloud_polygon->points.size(), PloyXarr, PloyYarr, cloudIn_Prj->points[i].x, cloudIn_Prj->points[i].y);
            if (1 != ret)//��ʾ������
            {
                cloud_cliped->points.push_back(cloud->points[i]);
            }
        }

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        cloud->clear();
        pcl::copyPointCloud(*cloud_cliped, *cloud);
        viewer->addPointCloud(cloud, "aftercut");
        m_renderWindow->Render();
    }
    else{
        cloud_cliped->clear();
        for (int i = 0; i < cloudIn_Prj->points.size(); i++)
        {
            ret = inOrNot1(cloud_polygon->points.size(), PloyXarr, PloyYarr, cloudIn_Prj->points[i].x, cloudIn_Prj->points[i].y);
            if (1 == ret)//��ʾ������
            {
                cloud_cliped->points.push_back(cloud->points[i]);
            }//��ʾ������
        }
        
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> currentColor(cloud_cliped, currentdynamicLabel->GetColor().red(), currentdynamicLabel->GetColor().green(), currentdynamicLabel->GetColor().blue());
        std::string CloudId = mode.toStdString() + rand_str(3);
        viewer->addPointCloud(cloud_cliped, currentColor, CloudId);
        emit PointCloudMarkingCompleted(cloud_cliped);
        m_renderWindow->Render();
    }
}

bool VTKOpenGLNativeWidget::LoadPointCloud(QString fileName)
{
    if (fileName.isEmpty())
    {
        return false;
    }

    cloud->points.clear();
    Frame_clicked_cloud->points.clear();
    Point_clicked_cloud->points.clear();

    int currentDisplayImageWidth, currentDisplayImageHeight;

    if (fileName.endsWith("pcd"))
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *cloud) == -1)
        {
            qDebug() << "Couldn't read pcd file  \n";
            return false;
        }
    }
    else if (fileName.endsWith("tif") || fileName.endsWith("tiff")) {
        cv::Mat image = cv::imread(fileName.toStdString(), cv::IMREAD_UNCHANGED);
        if (image.empty()) {
            QMessageBox::warning(this, "Warning", "�޷���ȡͼ���ļ�");
            return false;
        }
        Transfer_Function::cvMat2Cloud(image,cloud);
    }
    else {
        QMessageBox::warning(this, "Warning", "���ƶ�ȡ��ʽ����");
    }

    GetCoordinateSet();
    return true;
}

bool VTKOpenGLNativeWidget::SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr saveCloud)
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
            return_status = pcl::io::savePCDFileBinary(fileName.toStdString(),*saveCloud);
        else if (fileName.endsWith(".ply", Qt::CaseInsensitive))
            return_status = pcl::io::savePCDFileBinary(fileName.toStdString(),*saveCloud);
        else {
            fileName.append(".pcd");
            return_status = pcl::io::savePCDFileBinary(fileName.toStdString(),*saveCloud);
        }
        if (return_status != 0) {
            QString errorinfo = QString::fromStdString("Error writing point cloud" + fileName.toStdString());
            QMessageBox::warning(this, "Warning", errorinfo);
            return false;
        }
    }
    return true;
}

bool VTKOpenGLNativeWidget::SetBackgroundColor(QColor color)
{
    viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
    m_renderWindow->Render();
    return true;
}

bool VTKOpenGLNativeWidget::CoordinateAxisRendering(QString curaxis)
{
    if (!cloud->empty()) {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> render(cloud, curaxis.toStdString());
        viewer->updatePointCloud(cloud, render, "cloud");
        m_renderWindow->Render();
    }
    return true;
}

bool VTKOpenGLNativeWidget::PointCloudColorSet(QColor color)
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

bool VTKOpenGLNativeWidget::PointCloudPointSizeSet(int point_size)
{
    for (int i = 0; i < 1; ++i) {
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "cloud");
    }
    m_renderWindow->Render();
    return true;
}

bool VTKOpenGLNativeWidget::PointCloudHeightTransform(int factor)
{
    for (int i = 0; i < cloud->size(); ++i) {
        cloud->at(i).z *= factor;
    }
    reRendering(cloud->makeShared());
    return true;
}

void VTKOpenGLNativeWidget::AiInstance2Cloud(te::AiInstance* instance, cv::Mat& m_image,QColor color)
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
    //Transfer_Function::ExtractImage(m_image, &contour,&extractImage);
    Transfer_Function::ExtractImage2Cloud(m_image, axisset.OriginX, axisset.OriginY, &contour,cloud_marked);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> currentColor(cloud_marked, color.red(), color.green(), color.blue());
    std::string CloudId = instance->name + rand_str(3);
    viewer->addPointCloud(cloud_marked, currentColor, CloudId);
    m_renderWindow->Render();
}

void VTKOpenGLNativeWidget::reRendering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin)
{
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    viewer->addPointCloud<pcl::PointXYZ>(cloudin, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->resetCamera();
    //update();
    m_renderWindow->Render();//������Ⱦ
    viewer->resetCameraViewpoint("cloud");
}

void VTKOpenGLNativeWidget::GetCoordinateSet()
{
    pcl::PointXYZ min;//���ڴ�����������Сֵ
    pcl::PointXYZ max;//���ڴ������������ֵ
    pcl::getMinMax3D(*cloud, min, max);
    int currentDisplayImageLength = max.x - min.x;
    int currentDisplayImageHeight = max.y - min.y;
    axisset = {
        currentDisplayImageLength,
        currentDisplayImageHeight,
        min.x,
        min.y,
    };
}

/**
 * @brief inOrNot1
 * @param poly_sides    ƽ���ϻ��ƶ���εĶ�����
 * @param poly_X        �����x��������
 * @param poly_Y        �����y��������
 * @param x             Ŀ����Ƶ�x����
 * @param y             Ŀ����Ƶ�y����
 * @return
 */
int VTKOpenGLNativeWidget::inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y)
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
/// AABB��
/// </summary>
void VTKOpenGLNativeWidget::AxisAlignedBoundingBox()
{
    //pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud); //�ṩָ���������ݼ���ָ�롣
    feature_extractor.compute();//�������������ļ���
    //if (!computed) {
    //    QMessageBox::warning(this, "Warning", "��������У�");
    //    return;
    //}

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;

    pcl::PointXYZ min_point_AABB;//AABB��Χ��
    pcl::PointXYZ max_point_AABB;

    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia(moment_of_inertia);
    feature_extractor.getEccentricity(eccentricity);
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter(mass_center);

    //����AABB��Χ��
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "points");
    viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");

    m_renderWindow->Render();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

/// <summary>
/// ����OBB��
/// </summary>
void VTKOpenGLNativeWidget::OrientedBoundingBox()
{
    //pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();
    //if (!computed) {
    //    QMessageBox::warning(this, "Warning", "��������У�");
    //    return;
    //}

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
/// �޳���һ����������ڶ��������غϵĵ�
/// </summary>
/// <param name="cloud1"></param>
/// <param name="cloud2"></param>
void VTKOpenGLNativeWidget::subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2)
{
    // �²�����һ������
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud1);
    voxelGrid.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    voxelGrid.filter(*cloud1_downsampled);

    // ���� CropBox �˲��������ð�Χ�з�ΧΪ�ڶ������Ƶı߽��
    pcl::CropBox<pcl::PointXYZ> cropBox;
    cropBox.setInputCloud(cloud1_downsampled);
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    pcl::getMinMax3D(*cloud2, min, max);
    cropBox.setMin(Eigen::Vector4f(min.x, min.y, min.z, 1.0f));
    cropBox.setMax(Eigen::Vector4f(max.x, max.y, max.z, 1.0f));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    cropBox.filter(*cloud1_filtered);

    // ��ȡ���޳��ĵ�
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    cropBox.filter(indices->indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud1_downsampled);
    extract.setIndices(indices);
    extract.setNegative(true);
    extract.filter(*cloud1_filtered);

    // ���˲���ĵ��Ƹ�ֵ��ԭʼ�ĵ�һ�����ƶ���
    cloud1->swap(*cloud1_filtered);
}

/// <summary>
/// ��Y�ῴ��ȥ
/// </summary>
void VTKOpenGLNativeWidget::ViewYBtn()
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    if (PositiveAndNegative_Y_axis)
        viewer->setCameraPosition(centroid.x(), centroid.y(), centroid.z(), 0, 1, 0, 1, 0, 0);
    else
        viewer->setCameraPosition(centroid.x(), centroid.y(), centroid.z(), 0, 1, 0, -1, 0, 0);
    PositiveAndNegative_Y_axis = !PositiveAndNegative_Y_axis;
    viewer->updateCamera();
    viewer->spinOnce();
}

/// <summary>
/// ��X�ῴ��ȥ
/// </summary>
void VTKOpenGLNativeWidget::ViewXBtn()
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    if (PositiveAndNegative_X_axis)
        viewer->setCameraPosition(centroid.x(), centroid.y(), centroid.z(), 1, 0, 0, 0, 1, 0);
    else
        viewer->setCameraPosition(centroid.x(), centroid.y(), centroid.z(), 1, 0, 0, 0, -1, 0);
    PositiveAndNegative_X_axis = !PositiveAndNegative_X_axis;
    viewer->updateCamera();
    viewer->spinOnce();
}

/// <summary>
/// ��Z�ῴ��ȥ
/// </summary>
void VTKOpenGLNativeWidget::ViewZBtn()
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    if (PositiveAndNegative_Z_axis)
        viewer->setCameraPosition(0, 0, centroid.z(), 0, 0, -1, 0, 1, 0);
    else
        viewer->setCameraPosition(0, 0, -centroid.z(), 0, 0, -1,0, 1, 0);
        PositiveAndNegative_Z_axis = !PositiveAndNegative_Z_axis;
    viewer->updateCamera();
    viewer->spinOnce();
}

/// <summary>
/// ��˹�˲�
/// </summary>
/// <param name="data1">��׼��</param>
/// <param name="data2">��ֵ</param>
/// <param name="data3">�����뾶</param>
/// <param name="data4">openMP</param>
void VTKOpenGLNativeWidget::GuassFilter(QString data1, QString data2, QString data3, QString data4)
{
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", "�޵�������");
        return;
    }
    else {
        if (data1.isEmpty() || data2.isEmpty() || data3.isEmpty() || data4.isEmpty()) {
            QMessageBox::warning(this, "Warning", "������ʽ�������");
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

/// <summary>
/// �����˲�������
/// </summary>
/// <param name="cloud_in">�������</param>
/// <param name="paraA">��׼��</param>
/// <param name="paraB">��ֵ</param>
/// <param name="paraC">�����뾶</param>
/// <param name="paraD">openMP</param>
void VTKOpenGLNativeWidget::pcl_filter_guass(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float paraA, float paraB, float paraC, float paraD)
{
    //���þ����
    pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>::Ptr kernel(new pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ>);
    (*kernel).setSigma(paraA);
    (*kernel).setThresholdRelativeToSigma(paraB);
    //KdTree����
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    (*kdtree).setInputCloud(cloud_in);
    pcl::filters::Convolution3D <pcl::PointXYZ, pcl::PointXYZ, pcl::filters::GaussianKernel<pcl::PointXYZ, pcl::PointXYZ> > convolution;
    convolution.setKernel(*kernel);
    convolution.setInputCloud(cloud_in);
    convolution.setSearchMethod(kdtree);
    convolution.setRadiusSearch(paraC);//radius�������ɨ��İ뾶��������Ҫ�û�������
    convolution.setNumberOfThreads(paraD);//important! Set Thread number for openMP

    convolution.convolve(*cloud_Filter_out);
}

/// <summary>
/// ����ֱͨ�˲�
/// </summary>
/// <param name="data1">��С��Χ</param>
/// <param name="data2">���Χ</param>
/// <param name="data3">������</param>
/// <param name="data4">����orȥ��</param>
void VTKOpenGLNativeWidget::DirectFilter(QString data1, QString data2, QString data3, QString data4)
{
    if (cloud->empty()) {
        QMessageBox::warning(this, "Warning", "�޵�������");
        return;
    }
    else {
        if (data1.isEmpty() || data2.isEmpty() || data3.isEmpty() || data4.isEmpty()) {
            QMessageBox::warning(this, "Warning", "������ʽ�������");
            return;
        }
        pcl_filter_direct(cloud, data1.toFloat(), data2.toFloat(), data3, data4.toFloat());
        *cloud = *cloud_Filter_out;

        //Display_Properites();

        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        viewer->addPointCloud(cloud, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        viewer->resetCamera();
        m_renderWindow->Render();
    }
}
/// <summary>
/// �����˲�������
/// </summary>
/// <param name="cloud_in">�������</param>
/// <param name="min"></param>
/// <param name="max"></param>
/// <param name="axis">������</param>
/// <param name="is_save">����orȥ��</param>
void VTKOpenGLNativeWidget::pcl_filter_direct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float min, float max, QString axis, float is_save)
{
    pcl::PassThrough<pcl::PointXYZ> pass;//�����˲�������
    //��������
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName(axis.toStdString());//�˲����ֶΣ����˲��ķ��򡣿�����XYZ
    pass.setFilterLimits(min, max);//��������
    pass.setNegative(is_save);//����Ϊ��������ȥ����true��ȥ���������귶Χ�ڵĵ㣩

    pass.filter(*cloud_Filter_out);
}

