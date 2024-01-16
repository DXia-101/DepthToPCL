#include "DepthToPCL.h"

#pragma execution_character_set("utf-8")

#include <QDebug>
#include <QHBoxLayout>
#include <QColorDialog>
#include <QMessageBox>
#include <QFileDialog>
#include <QTime>
#include <QDir>
#include <QFile>
#include <QtMath>
#include <QDirIterator>
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QStatusBar>
#include <QKeyEvent>
//#include <QtConcurrent>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QImage>
#include <QPixmap>
#include <QPoint>
#include <QTextStream>
#include <QIODevice>

#include <vector>
#include <Windows.h>
#include <string>

#include "CategoryTag.h"
#include "Configure.h"
#include "DynamicLabel.h"


DepthToPCL::DepthToPCL(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);

    PCL_Initalization();

    Interface_Initialization();

    InitStateMachine();
}

DepthToPCL::~DepthToPCL()
{}

/**
 * @brief 界面初始化程序
 * @return
 */
void DepthToPCL::Interface_Initialization()
{
    QMenuBar* menu_bar = new QMenuBar(this);         
    ui.ToolBarHorizontalLayout->addWidget(menu_bar);
    menu_bar->setStyleSheet("font-size : 18px");

    QMenu* file_menu = new QMenu("文件", menu_bar);

    QAction* open_action = new QAction("读取");
    QAction* save_action = new QAction("保存");
    QAction* quit_action = new QAction("退出");

    file_menu->addAction(open_action);
    file_menu->addAction(save_action);
    file_menu->addSeparator();      
    file_menu->addAction(quit_action);

    menu_bar->addMenu(file_menu);

    connect(open_action, &QAction::triggered, this, &DepthToPCL::Open_clicked);//读取文件
    connect(save_action, &QAction::triggered, this, &DepthToPCL::Save_clicked);//保存文件
    connect(quit_action, &QAction::triggered, this, &DepthToPCL::close);//保存文件

    QMenu* Display_menu = new QMenu("显示", menu_bar);

    QAction* background_set = new QAction("背景设置");
    QAction* PointCloud_render = new QAction("点云渲染");
    QAction* color_set = new QAction("颜色设置");
    QAction* dot_size = new QAction("点的大小");

    Display_menu->addAction(background_set);
    Display_menu->addAction(PointCloud_render);
    Display_menu->addAction(color_set);
    Display_menu->addAction(dot_size);

    menu_bar->addMenu(Display_menu);
    
    connect(background_set, &QAction::triggered, this, &DepthToPCL::Background_Select);//背景颜色设置
    connect(PointCloud_render, &QAction::triggered, this, &DepthToPCL::PressBtn_rendering);//背景颜色设置
    connect(color_set, &QAction::triggered, this, &DepthToPCL::PressBtn_colorSelect);//点云颜色设置

    connect(dot_size, &QAction::triggered, this, &DepthToPCL::PressBtn_PointCloud_PointSize_slider);//点云点大小设置

    QMenu* Surround_menu = new QMenu("包围", menu_bar);

    QAction* AxisAlignedBounding = new QAction("AABB");
    QAction* OrientedBounding = new QAction("OBB");
    
    Surround_menu->addAction(AxisAlignedBounding);
    Surround_menu->addAction(OrientedBounding);

    menu_bar->addMenu(Surround_menu);

    connect(AxisAlignedBounding, &QAction::triggered, vtkWidget, &VTKOpenGLNativeWidget::AxisAlignedBoundingBox);
    connect(OrientedBounding, &QAction::triggered, vtkWidget, &VTKOpenGLNativeWidget::OrientedBoundingBox);

    QMenu* Filter_menu = new QMenu("滤波", menu_bar);

    QAction* Gaussian_filter = new QAction("高斯滤波");
    QAction* Direct_filter = new QAction("直通滤波");

    Filter_menu->addAction(Gaussian_filter);
    Filter_menu->addAction(Direct_filter);

    menu_bar->addMenu(Filter_menu);

    connect(Gaussian_filter, &QAction::triggered, this, &DepthToPCL::GuassFilterAction);
    connect(Direct_filter, &QAction::triggered, this, &DepthToPCL::DirectFilterAction);

    isKeyDown_shift = false;

    QMenu* Contour_menu = new QMenu("轮廓", menu_bar);

    QAction* Save_Contour = new QAction("保存轮廓");
    QAction* Load_Contour = new QAction("加载轮廓");

    Contour_menu->addAction(Save_Contour);
    Contour_menu->addAction(Load_Contour);

    menu_bar->addMenu(Contour_menu);

    connect(Save_Contour, &QAction::triggered, this, &DepthToPCL::SaveContour);
    connect(Load_Contour, &QAction::triggered, this, &DepthToPCL::LoadContour);

    connect(this, &DepthToPCL::MarkComplete, this, &DepthToPCL::SaveSampleLabel);
    connect(this, &DepthToPCL::MarkComplete, this, &DepthToPCL::SaveMat);

    labelVLayout = new QVBoxLayout(ui.scrollArea);
    ui.scrollAreaWidgetContents->setLayout(labelVLayout);
    labelVLayout->addStretch(1);

    totalHeight = 0;

    point_size = 1;
}

/**
 * @brief PCL初始化程序
 * @return
 */
void DepthToPCL::PCL_Initalization()
{
    vtkWidget = new VTKOpenGLNativeWidget(this);
    ui.VTKWidgetLayout->addWidget(vtkWidget);

    cvImageWidget = new ImageLabel(this);
    ui.VTKWidgetLayout->addWidget(cvImageWidget);
    cvImageWidget->hide();
}

void DepthToPCL::InitStateMachine()
{
    m_pStateMachine = new QStateMachine();
    TwoDState = new QState(m_pStateMachine);
    ThrDState = new QState(m_pStateMachine);
    TwoDState->assignProperty(cvImageWidget,"visible",true);
    TwoDState->assignProperty(vtkWidget,"visible",false);
    ThrDState->assignProperty(cvImageWidget, "visible", false);
    ThrDState->assignProperty(vtkWidget,"visible",true);
    TwoDState->addTransition(this, SIGNAL(ConversionBetween2Dand3D()), ThrDState);
    ThrDState->addTransition(this, SIGNAL(ConversionBetween2Dand3D()), TwoDState);

    m_pStateMachine->addState(TwoDState);
    m_pStateMachine->addState(ThrDState);

    m_pStateMachine->setInitialState(ThrDState);
    m_pStateMachine->start();
}

void DepthToPCL::CloudToContour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin,QString label)
{
    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*vtkWidget->cloud, min, max);

    cv::Mat image(cvImageWidget->height(), cvImageWidget->width(), CV_8UC3);
    for (int j = 0; j < image.rows; j++)
    {
        for (int i = 0; i < image.cols; i++)
        {
            image.at<cv::Vec3b>(j, i)[0] = 0;
            image.at<cv::Vec3b>(j, i)[1] = 0;
            image.at<cv::Vec3b>(j, i)[2] = 0;
        }
    }
    
    float l; //单个像素代表的实际长度
    float a = (max.x - min.x) / cvImageWidget->width();   //分辨率，根据实际需要设置，这里采用648*cvImageWidget->height()
    float b = (max.y - min.y) / cvImageWidget->height();
    if (a > b)
    {
        l = a;
    }
    else
    {
        l = b;
    }

    for (int i = 0; i < cloudin->size(); i++)
    {
        //计算点对应的像素坐标
        int x = (cloudin->points[i].x - min.x) / l;
        int y = (cloudin->points[i].y - min.y) / l;

        //将颜色信息赋予像素
        if (x > 0 && x < cvImageWidget->width() && y>0 && y < cvImageWidget->height())
        {
            image.at<cv::Vec3b>(y, x)[0] = cloudin->points[i].z;
            image.at<cv::Vec3b>(y, x)[1] = cloudin->points[i].z;
            image.at<cv::Vec3b>(y, x)[2] = cloudin->points[i].z;
        }
    }
    te::AiInstance instance;
    instance.name = label.toStdString();
    cv::Mat grayImg, binImg;
    cv::cvtColor(image, grayImg, cv::COLOR_BGR2GRAY);
    threshold(grayImg, binImg, 0, 255, cv::ThresholdTypes::THRESH_OTSU);
    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    findContours(binImg, contours, hierarchy, cv::RetrievalModes::RETR_TREE, cv::CHAIN_APPROX_NONE);
    
    te::PolygonF polygon;
    for (const std::vector<cv::Point>& contourPoints : contours) {

        te::PolygonF::PointType point;
        for (const cv::Point& contourPoint : contourPoints) {
            
            point.x = static_cast<float>(contourPoint.x);
            point.y = static_cast<float>(contourPoint.y);
            polygon.push_back(point);
            
        }
        instance.contour.polygons.push_back(polygon);
        polygon.clear();
    }
    allResultAiInstance.push_back(instance);
}

/// <summary>
/// 点云转图片
/// </summary>
void DepthToPCL::TDTo2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, cv::Mat& imageout)
{

    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*cloudin, min, max);

    cv::Mat image(cvImageWidget->height(), cvImageWidget->width(), CV_8UC3);
    for (int j = 0; j < image.rows; j++)
    {
        for (int i = 0; i < image.cols; i++)
        {
            image.at<cv::Vec3b>(j, i)[0] = 0;
            image.at<cv::Vec3b>(j, i)[1] = 0;
            image.at<cv::Vec3b>(j, i)[2] = 0;
        }
    }

    float l; //单个像素代表的实际长度
    float a = (max.x - min.x) / cvImageWidget->width();   //分辨率，根据实际需要设置，这里采用648*cvImageWidget->height()
    float b = (max.y - min.y) / cvImageWidget->height();

    if (a > b)
    {
        l = a;
    }
    else
    {
        l = b;
    }

    for (int i = 0; i < cloudin->size(); i++)
    {
        //计算点对应的像素坐标
        int x = (cloudin->points[i].x - min.x) / l;
        int y = (cloudin->points[i].y - min.y) / l;

        //将颜色信息赋予像素
        if (x > 0 && x < cvImageWidget->width() && y>0 && y < cvImageWidget->height())
        {
            image.at<cv::Vec3b>(y, x)[0] = cloudin->points[i].z;
            image.at<cv::Vec3b>(y, x)[1] = cloudin->points[i].z;
            image.at<cv::Vec3b>(y, x)[2] = cloudin->points[i].z;
        }
    }
    imageout = image;
}

//void DepthToPCL::ExtractContours(Te_Gt& contour, cv::Mat imgIn)
//{
//    cv::Mat grayImg, binImg;
//    cv::cvtColor(imgIn, grayImg, cv::COLOR_BGR2GRAY);
//    threshold(grayImg, binImg, 0, 255, cv::ThresholdTypes::THRESH_OTSU);
//
//    findContours(binImg, contour.GetContourVec(), contour.GetHierarchy(), cv::RetrievalModes::RETR_TREE, cv::CHAIN_APPROX_NONE);
//}

void DepthToPCL::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        QPoint pos = event->pos();
        QWidget* widget = childAt(pos);
        DynamicLabel* temp = qobject_cast<DynamicLabel*>(widget);
        if(nullptr != temp) {
            vtkWidget->currentdynamicLabel = temp;
            cvImageWidget->scene->currentdynamicLabel = temp;
            currentLabelNAme = vtkWidget->currentdynamicLabel->GetLabel();
            cvImageWidget->scene->currentColor = temp->GetColor();
            qDebug() << "当前标签：" << vtkWidget->currentdynamicLabel->GetLabel();
        }
    }
}

void DepthToPCL::paintEvent(QPaintEvent* event)
{
    //QWidget::paintEvent(event);
    //// 如果有选中的DynamicLabel对象
    //if (nullptr != vtkWidget->currentdynamicLabel)
    //{
    //    // 获取选中对象的位置和大小
    //    QPoint pos = vtkWidget->currentdynamicLabel->pos();
    //    QSize size = vtkWidget->currentdynamicLabel->size();

    //    // 创建一个QPainter对象，用于绘制虚线
    //    QPainter painter(this);
    //    painter.setPen(Qt::DashLine);
    //    painter.drawRect(pos.x() - 5, pos.y() - 5, size.width() + 10, size.height() + 10);
    //}
}

/// <summary>
/// 保存当前显示界面的点云
/// </summary>
void DepthToPCL::Save_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply)"));
    if (vtkWidget->cloud->empty()) {
        return;
    }
    else {
        if (filename.isEmpty()) {
            return;
        }
        int return_status;
        if (filename.endsWith(".pcd", Qt::CaseInsensitive))
            return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *vtkWidget->cloud);
        else if (filename.endsWith(".ply", Qt::CaseInsensitive))
            return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *vtkWidget->cloud);
        else {
            filename.append(".pcd");
            return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *vtkWidget->cloud);
        }
        if (return_status != 0) {
            QString errorinfo = QString::fromStdString("Error writing point cloud"+filename.toStdString());
            QMessageBox::warning(this, "Warning", errorinfo);
            return;
        }
    }
    
}


/// <summary>
/// 从Y轴看过去
/// </summary>
void DepthToPCL::on_ViewYBtn_clicked()
{
    vtkWidget->ViewYBtn();
}

/// <summary>
/// 从X轴看过去
/// </summary>
void DepthToPCL::on_ViewXBtn_clicked()
{
    vtkWidget->ViewXBtn();
}

/// <summary>
/// 从Z轴看过去
/// </summary>
void DepthToPCL::on_ViewZBtn_clicked()
{
    vtkWidget->ViewZBtn();
}

/// <summary>
/// 背景颜色选择
/// </summary>
void DepthToPCL::Background_Select()
{
    dialog_colorselect = new pcl_view_select_color();
    
    QColor color = dialog_colorselect->getColor();

    vtkWidget->viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
    vtkWidget->m_renderWindow->Render();
    return;
}

/// <summary>
/// XYZ方向渲染
/// </summary>
void DepthToPCL::PressBtn_rendering()
{
    dialog_render = new View_Render();
    connect(dialog_render, &View_Render::sendData, this, &DepthToPCL::Rendering_setting);
    if(dialog_render->exec() == QDialog::Accepted){}
    delete dialog_render;
}

void DepthToPCL::Rendering_setting(QString data)
{
    if (!vtkWidget->cloud->empty()) {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> render(vtkWidget->cloud, data.toStdString( ));
        vtkWidget->viewer->updatePointCloud(vtkWidget->cloud, render, "cloud");
        vtkWidget->m_renderWindow->Render();
    }
    return;
}

/// <summary>
/// 点云颜色设置
/// </summary>
void DepthToPCL::PressBtn_colorSelect()
{
    dialog_colorselect = new pcl_view_select_color();
    QColor color = dialog_colorselect->getColor();
    QColor temp;
    temp.setRgb(143, 153, 159, 255);
    if (!vtkWidget->cloud->empty() && (color != temp)) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>selected_color(vtkWidget->cloud,color.redF()*255,color.greenF()*255,color.blueF()*255);
        vtkWidget->viewer->updatePointCloud(vtkWidget->cloud, selected_color, "cloud");
        vtkWidget->m_renderWindow->Render();
    }
    else {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>selected_color(vtkWidget->cloud, 255, 255, 255);
        vtkWidget->viewer->updatePointCloud(vtkWidget->cloud, selected_color, "cloud");
        vtkWidget->m_renderWindow->Render();
    }
    return;
}

/// <summary>
/// 点云点大小设置Dialog
/// </summary>
void DepthToPCL::PressBtn_PointCloud_PointSize_slider()
{
    pointsize_set_dialog = new PointCloud_PointSize_Set_Dialog();
    connect(pointsize_set_dialog, SIGNAL(sendData(QString)), this, SLOT(PointCloud_PointSize_SliderSetting(QString)));
    if(pointsize_set_dialog->exec() == QDialog::Accepted){}
    delete pointsize_set_dialog;
}

/// <summary>
/// 点云点的大小设置
/// </summary>
void DepthToPCL::PointCloud_PointSize_SliderSetting(QString data)
{
    point_size = data.toInt();

    for (int i = 0; i < 1; ++i) {
        vtkWidget->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "cloud");
    }
}

/// <summary>
/// 开启框选，再次开启关闭
/// </summary>
void DepthToPCL::on_FramePickBtn_clicked()
{
    keybd_event('X', 0, 0, 0);
    keybd_event('X', 0, KEYEVENTF_KEYUP, 0);
}

/// <summary>
/// 开启点选，再次点击关闭
/// </summary>
void DepthToPCL::on_PointPickBtn_clicked()
{
    if (!isKeyDown_shift) {
        keybd_event(16, 0, 0, 0);
        isKeyDown_shift = true;
    }
    else {
        keybd_event(16, 0, KEYEVENTF_KEYUP, 0);
        isKeyDown_shift = false;
    }
}

void DepthToPCL::on_ComfirmFramePickBtn_clicked()
{
    vtkWidget->ComfirmFramePick();
}

void DepthToPCL::on_ComfirmPointPickBtn_clicked()
{
    vtkWidget->ComfirmPointPick();
}

/// <summary>
/// 高斯滤波注册动作
/// </summary>
void DepthToPCL::GuassFilterAction()
{
    dialog_Guass_filter = new Filter_Guass();
    connect(dialog_Guass_filter, &Filter_Guass::sendData, vtkWidget, &VTKOpenGLNativeWidget::GuassFilter);
    if (dialog_Guass_filter->exec() == QDialog::Accepted) {}
    delete dialog_Guass_filter;
}

/// <summary>
/// 直通滤波动作
/// </summary>
void DepthToPCL::DirectFilterAction()
{
    dialog_Direct_filter = new Filter_Direct();
    connect(dialog_Direct_filter, &Filter_Direct::sendData, vtkWidget, &VTKOpenGLNativeWidget::DirectFilter);
    if (dialog_Direct_filter->exec() == QDialog::Accepted) {}
    delete dialog_Direct_filter;
}

/// <summary>
/// 转换界面按钮
/// </summary>
void DepthToPCL::on_changeFormBtn_clicked()
{
    if (ThrDState->active()) {
        TDTo2D(vtkWidget->cloud,m_image);
        QImage qImg = QImage((unsigned char*)(m_image.data), m_image.cols, m_image.rows, m_image.cols * m_image.channels(), QImage::Format_RGB888);
        cvImageWidget->setImage(qImg);
        emit ConversionBetween2Dand3D();
    }
    else {
        emit ConversionBetween2Dand3D();
    }
}

/// <summary>
/// 绘制轮廓按钮
/// </summary>
void DepthToPCL::on_drawCounterBtn_clicked()
{
    cv::Mat grayImg,binImg;
    cv::cvtColor(m_image, grayImg, cv::COLOR_BGR2GRAY);

    //进行二值化
    threshold(grayImg, binImg, 0, 255, cv::ThresholdTypes::THRESH_OTSU);

    //提取二值化图像中的轮廓数据
    std::vector<std::vector<cv::Point> > contour_vec;
    std::vector<cv::Vec4i> hierarchy;
    findContours(binImg, contour_vec, hierarchy, cv::RetrievalModes::RETR_TREE, cv::CHAIN_APPROX_NONE);

    //绘制单通道轮廓图像，背景为白色，轮廓线条用黑色
    cv::Mat blkImg(binImg.size(), CV_8UC1, cv::Scalar(255));
    drawContours(blkImg, contour_vec, -1, cv::Scalar(0), 2);
    QImage qImg = QImage((unsigned char*)(blkImg.data), blkImg.cols, blkImg.rows, blkImg.cols * blkImg.channels(), QImage::Format_Grayscale8);
    //scene->addPixmap(QPixmap::fromImage(qImg.scaled(cvImageWidget->size(), Qt::KeepAspectRatio)));
    cvImageWidget->setImage(qImg);
}


/// <summary>
/// 标记完成
/// </summary>
void DepthToPCL::on_MarkCompletedBtn_clicked()
{
    for (int i = 0; i < labelVLayout->count(); ++i) {
        QWidget* widget = labelVLayout->itemAt(i)->widget();
        DynamicLabel* curlabel = qobject_cast<DynamicLabel*>(widget);
        if (nullptr != curlabel) {
            for (int j = 0; j < curlabel->GetCloudSize(); ++j) {
                CloudToContour(curlabel->CloudAt(j), curlabel->GetLabel());
            }
        }
    }
    emit MarkComplete();
}

/// <summary>
/// 绘制标记轮廓
/// </summary>
void DepthToPCL::on_drawMarkersBtn_clicked()
{
    cv::Mat blkImg(cvImageWidget->height(), cvImageWidget->width(), CV_8UC1, cv::Scalar(255));

    for (te::AiInstance aiContour : allResultAiInstance)
    {
        // 获取轮廓相关的数据
        std::vector<std::vector<cv::Point>> contour_vec;
        for (const te::PolygonF& polygon : aiContour.contour.polygons) {
            std::vector<cv::Point> contourPoints;
            for (const te::Point2f& point : polygon) {
                cv::Point contourPoint(static_cast<int>(point.x), static_cast<int>(point.y));
                contourPoints.push_back(contourPoint);
            }

            contour_vec.push_back(contourPoints);
        }

        drawContours(blkImg, contour_vec, -1, cv::Scalar(0), 2);
        QImage qImg = QImage((unsigned char*)(blkImg.data), blkImg.cols, blkImg.rows, blkImg.cols * blkImg.channels(), QImage::Format_Grayscale8);
        //scene->addPixmap(QPixmap::fromImage(qImg.scaled(cvImageWidget->size(), Qt::KeepAspectRatio)));
        cvImageWidget->setImage(qImg);
    }
}

void DepthToPCL::on_clearMarkersBtn_clicked()
{
}

//void DepthToPCL::on_clearMarkersBtn_clicked()
//{
//    Test1Label->ClearCloudVector();
//    Test2Label->ClearCloudVector();
//    Test3Label->ClearCloudVector();
//    ContourVector.clear();
//    viewer->removeAllPointClouds();
//    viewer->removeAllShapes();
//
//    viewer->addPointCloud<pcl::PointXYZ>(cloud->makeShared(), "cloud");
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
//    viewer->resetCamera();
//    vtkWidget->update();
//    renderWindow->Render();//重新渲染
//}

void DepthToPCL::SaveContour()
{
    //QString filePath = QFileDialog::getSaveFileName(nullptr, "选择保存路径和文件名", "", "(*.txt)");
    //if (filePath.isEmpty())
    //{
    //    return;
    //}

    //QFile file(filePath);
    //if (file.open(QIODevice::WriteOnly | QIODevice::Text))
    //{
    //    QTextStream out(&file);

    //    size_t arraySize = ContourVector.size();
    //    out << "Array Size: " << arraySize << "\n";

    //    for (auto labelContour : ContourVector)
    //    {
    //        QString label = labelContour->GetLabel();
    //        out << "Label: " << label << "\n";

    //        const std::vector<std::vector<cv::Point>>& contourVec = labelContour->GetContourVec();
    //        size_t contourVecSize = contourVec.size();
    //        out << "Contour Vec Size: " << contourVecSize << "\n";
    //        for (const std::vector<cv::Point>& contour : contourVec)
    //        {
    //            size_t contourSize = contour.size();
    //            out << "Contour Size: " << contourSize << "\n";
    //            for (const cv::Point& point : contour)
    //            {
    //                out << point.x << " " << point.y << "\n";
    //            }
    //        }
    //        const std::vector<cv::Vec4i>& hierarchy = labelContour->GetHierarchy();
    //        size_t hierarchySize = hierarchy.size();
    //        out << "Hierarchy Size: " << hierarchySize << "\n";
    //        for (const cv::Vec4i& item : hierarchy)
    //        {
    //            out << item[0] << " " << item[1] << " " << item[2] << " " << item[3] << "\n";
    //        }

    //        out << "\n";
    //    }
    //    file.close();
    //}
    //else
    //{
    //    qDebug() << "file open failed";
    //}
}

void DepthToPCL::LoadContour()
{
    //QString filePath = QFileDialog::getOpenFileName(nullptr, "选择文件", "", "(*.txt)");
    //if (filePath.isEmpty())
    //{
    //    return ;
    //}

    //QFile file(filePath);
    //if (file.open(QIODevice::ReadOnly | QIODevice::Text))
    //{
    //    QTextStream in(&file);

    //    size_t arraySize = 0;
    //    QString line;

    //    ContourVector.clear();

    //    if (in.readLineInto(&line))
    //    {
    //        size_t pos = line.indexOf(": ");
    //        if (pos != -1)
    //        {
    //            arraySize = line.mid(pos + 2).toULong();
    //        }
    //    }

    //    for (size_t i = 0; i < arraySize; ++i)
    //    {
    //        QString label;
    //        std::vector<std::vector<cv::Point>> contourVec;
    //        std::vector<cv::Vec4i> hierarchy;

    //        if (in.readLineInto(&line))
    //        {
    //            size_t pos = line.indexOf(": ");
    //            if (pos != -1)
    //            {
    //                label = line.mid(pos + 2);
    //            }
    //        }

    //        size_t contourVecSize = 0;
    //        if (in.readLineInto(&line))
    //        {
    //            size_t pos = line.indexOf(": ");
    //            if (pos != -1)
    //            {
    //                contourVecSize = line.mid(pos + 2).toULong();
    //            }
    //        }
    //        for (size_t j = 0; j < contourVecSize; ++j)
    //        {
    //            size_t contourSize = 0;
    //            if (in.readLineInto(&line))
    //            {
    //                size_t pos = line.indexOf(": ");
    //                if (pos != -1)
    //                {
    //                    contourSize = line.mid(pos + 2).toULong();
    //                }
    //            }

    //            std::vector<cv::Point> contour;
    //            for (size_t k = 0; k < contourSize; ++k)
    //            {
    //                if (in.readLineInto(&line))
    //                {
    //                    QStringList coordinates = line.split(' ');
    //                    if (coordinates.size() == 2)
    //                    {
    //                        int x = coordinates.at(0).toInt();
    //                        int y = coordinates.at(1).toInt();
    //                        contour.push_back(cv::Point(x, y));
    //                    }
    //                }
    //            }

    //            contourVec.push_back(std::move(contour));
    //        }

    //        size_t hierarchySize = 0;
    //        if (in.readLineInto(&line))
    //        {
    //            size_t pos = line.indexOf(": ");
    //            if (pos != -1)
    //            {
    //                hierarchySize = line.mid(pos + 2).toULong();
    //            }
    //        }
    //        for (size_t j = 0; j < hierarchySize; ++j)
    //        {
    //            if (in.readLineInto(&line))
    //            {
    //                QStringList items = line.split(' ');
    //                if (items.size() == 4)
    //                {
    //                    int item0 = items.at(0).toInt();
    //                    int item1 = items.at(1).toInt();
    //                    int item2 = items.at(2).toInt();
    //                    int item3 = items.at(3).toInt();
    //                    hierarchy.push_back(cv::Vec4i(item0, item1, item2, item3));
    //                }
    //            }
    //        }
    //        Te_Gt* temp = new Te_Gt(label);
    //        temp->GetContourVec() = contourVec;
    //        temp->GetHierarchy() = hierarchy;
    //        ContourVector.emplace_back(temp);

    //        in.readLineInto(&line);
    //    }

    //    file.close();
    //}
    //else
    //{
    //    qDebug() << "file open failed";
    //}

}

void DepthToPCL::SaveMat()
{
    /*vImg.push_back(m_image);*/
}

void DepthToPCL::SaveSampleLabel()
{
    /*vSampleLabel.push_back(ContourVector);*/
}

void DepthToPCL::on_addDynamicLabel_clicked()
{
    DynamicLabel* label = new DynamicLabel(tr("默认"));
    labelVLayout->addWidget(label);
    totalHeight += label->height();
    ui.scrollAreaWidgetContents->setGeometry(0, 0, label->width(), totalHeight);
}

void DepthToPCL::on_delDynamicLabel_clicked()
{

}

void DepthToPCL::on_startTagBtn_clicked()
{
    if (ThrDState->active()) {
        vtkWidget->isPickingMode = !vtkWidget->isPickingMode;
        if (vtkWidget->isPickingMode) {
            vtkWidget->line_id++;
            vtkWidget->cloud_polygon->clear();
            vtkWidget->flag = false;
        }
        else {
            vtkWidget->projectInliers(vtkWidget,currentLabelNAme);
        }
    }
    else if (TwoDState->active()) {

    }

}

/// <summary>
/// 打开文件
/// </summary>
void DepthToPCL::Open_clicked() {
    QString fileName = QFileDialog::getOpenFileName(this, tr("open  file"),
        "", tr("pcb files(*.pcd *.ply *.tif) ;;All files (*.*)"));

    if (fileName.isEmpty())
    {
        return;
    }

    vtkWidget->cloud->points.clear();
    vtkWidget->Frame_clicked_cloud->points.clear();
    vtkWidget->Point_clicked_cloud->points.clear();

    if (fileName.endsWith("pcd"))
    {
        qDebug() << fileName;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *vtkWidget->cloud) == -1)
        {
            qDebug() << "Couldn't read pcd file  \n";
            return;
        }
    }
    else if (fileName.endsWith("tif")) {
        cv::Mat image = cv::imread(fileName.toStdString(), cv::IMREAD_UNCHANGED);
        if (image.empty()) {
            QMessageBox::warning(this, "Warning", "无法读取图像文件");
            return;
        }
        vtkWidget->cloud->width = image.cols;
        vtkWidget->cloud->height = image.rows;
        vtkWidget->cloud->points.resize(vtkWidget->cloud->width * vtkWidget->cloud->height);

        cv::MatIterator_<float> pixel_it, pixel_end;
        pixel_it = image.begin<float>();
        pixel_end = image.end<float>();


        for (int i = 0; pixel_it != pixel_end; ++pixel_it, ++i) {
            int y = i / image.cols;
            int x = i % image.cols;
            float depth = *pixel_it;

            vtkWidget->cloud->at(x, y).x = static_cast<float>(x);
            vtkWidget->cloud->at(x, y).y = static_cast<float>(y);
            vtkWidget->cloud->at(x, y).z = depth*255;
        }

        //pcl::io::savePCDFile("output_cloud.pcd", cloud);
    }
    else {
        QMessageBox::warning(this, "Warning", "点云读取格式错误！");
    }

    //Display_Properites();

    //移除窗口点云
    vtkWidget->viewer->removeAllPointClouds();
    vtkWidget->viewer->removeAllShapes();
    
    vtkWidget->viewer->addPointCloud<pcl::PointXYZ>(vtkWidget->cloud->makeShared(), "cloud");
    vtkWidget->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    vtkWidget->viewer->resetCamera();
    vtkWidget->update();
    vtkWidget->m_renderWindow->Render();//重新渲染

    vtkWidget->viewer->resetCameraViewpoint("cloud");
}