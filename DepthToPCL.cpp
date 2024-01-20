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
#include <QPixmap>
#include <QPainter>
#include <QPainterPath>
#include <QImage>
#include <QVector>
#include <QRect>
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
#include <stdexcept>

#include "DynamicLabel.h"
#include "teRapidjsonObjectTree.h"


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
 * @brief 界面初始化
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

    labelVLayout = new QVBoxLayout(ui.scrollArea);
    ui.scrollAreaWidgetContents->setLayout(labelVLayout);
    labelVLayout->addStretch(1);

    totalHeight = 0;

    point_size = 1;

    currentDisplayImageLength = 0;
    currentDisplayImageHeight = 0;

    connect(vtkWidget, &VTKOpenGLNativeWidget::PointCloudMarkingCompleted, this, &DepthToPCL::ReceiveMarkedPointClouds);
    connect(cvImageWidget->scene, &GraphicsPolygonScene::GraphicsPolygonItemMarkingCompleted, this, &DepthToPCL::ReceiveMarkedPolygonItem);
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

/// <summary>
/// 初始化状态机
/// </summary>
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

void DepthToPCL::addAiInstance(DynamicLabel* curlabel,GraphicsPolygonItem* markedPolygon)
{
    te::AiInstance instance;
    instance.name = curlabel->GetLabel().toStdString();
    te::PolygonF polygon;
    te::PolygonF::PointType point;
    for (const QPointF& polygonPoint : markedPolygon->polygon()) {
        point.x = static_cast<float>(polygonPoint.x());
        point.y = static_cast<float>(polygonPoint.y());
        polygon.push_back(point);
    }
    instance.contour.polygons.push_back(polygon);
    curlabel->LabelAiInstSet.push_back(instance);
}

void DepthToPCL::addAiInstance(DynamicLabel* curlabel,pcl::PointCloud<pcl::PointXYZ>::Ptr markedCloud)
{
    cv::Mat image(currentDisplayImageHeight, currentDisplayImageLength, CV_8UC3);
    Cloud2cvMat(markedCloud, image);

    SaveMatContour2Label(image,curlabel);
}

void DepthToPCL::AiInstSet2Cloud(DynamicLabel* curlabel)
{
    for (te::AiInstance instance : curlabel->LabelAiInstSet) {
        vtkWidget->AiInstance2Cloud(&instance, curlabel->GetColor());
    }
}

void DepthToPCL::AiInstSet2PolygonItem(DynamicLabel* curlabel)
{
    for (te::AiInstance instance : curlabel->LabelAiInstSet) {
        cvImageWidget->scene->AiInstance2GraphicsPolygonItem(&instance, curlabel->GetColor());
    }
}

/// <summary>
/// 点云转换为cv::Mat
/// </summary>
/// <param name="cloudin">要转换的点云</param>
/// <param name="imageout">转换成功的图像</param>
void DepthToPCL::Cloud2cvMat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, cv::Mat& imageout)
{
    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*vtkWidget->cloud, min, max);
    
    if (!imageout.empty())
        imageout.release();
    imageout.create(currentDisplayImageHeight, currentDisplayImageLength, CV_8UC3);

    for (int i = 0; i < cloudin->size(); i++)
    {
        //计算点对应的像素坐标
        int x = (cloudin->points[i].x - min.x);
        int y = (cloudin->points[i].y - min.y);

        //将颜色信息赋予像素
        if (x > 0 && x < currentDisplayImageLength && y>0 && y < currentDisplayImageHeight)
        {
            imageout.at<cv::Vec3b>(y, x)[0] = cloudin->points[i].z;
            imageout.at<cv::Vec3b>(y, x)[1] = cloudin->points[i].z;
            imageout.at<cv::Vec3b>(y, x)[2] = cloudin->points[i].z;
        }
    }
}

/// <summary>
/// cv::Mat转为点云
/// </summary>
/// <param name="imageIn">待转换的cv::Mat</param>
/// <param name="cloudOut">转换完后的点云</param>
void DepthToPCL::cvMat2Cloud(cv::Mat& imageIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut)
{
    cloudOut->width = imageIn.cols;
    cloudOut->height = imageIn.rows;
    cloudOut->points.resize(cloudOut->width * cloudOut->height);

    cv::MatIterator_<float> pixel_it, pixel_end;
    pixel_it = imageIn.begin<float>();
    pixel_end = imageIn.end<float>();

    for (int i = 0; pixel_it != pixel_end; ++pixel_it, ++i) {
        int y = i / imageIn.cols;
        int x = i % imageIn.cols;
        float depth = *pixel_it;

        cloudOut->at(x, y).x = static_cast<float>(x);
        cloudOut->at(x, y).y = static_cast<float>(y);
        cloudOut->at(x, y).z = depth;
    }
}

/// <summary>
/// cv::Mat格式的图片提取轮廓
/// </summary>
/// <param name="Matin">需要提取的cv::Mat图片</param>
/// <param name="contours">提取出来的轮廓</param>
void DepthToPCL::cvMat2Contour(cv::Mat& Matin, std::vector<std::vector<cv::Point>>* contours)
{
    cv::Mat grayImg, binImg;
    cv::cvtColor(Matin, grayImg, cv::COLOR_BGR2GRAY);
    threshold(grayImg, binImg, 0, 255, cv::ThresholdTypes::THRESH_OTSU);
    std::vector<cv::Vec4i> hierarchy;
    findContours(binImg, *contours, hierarchy, cv::RetrievalModes::RETR_TREE, cv::CHAIN_APPROX_NONE);//会将整个图像的最外层轮廓算进去
}

/// <summary>
/// 保存图像轮廓到标签当中
/// </summary>
/// <param name="Matin">待查找轮廓的图像</param>
/// <param name="curlabel">保存轮廓的标签</param>
void DepthToPCL::SaveMatContour2Label(cv::Mat& Matin, DynamicLabel* curlabel)
{
    te::AiInstance instance;
    instance.name = curlabel->GetLabel().toStdString();
    std::vector<std::vector<cv::Point>> contours;
    cvMat2Contour(Matin, &contours);
    contours.erase(contours.begin());
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
    //allResultAiInstance.push_back(instance);
    curlabel->LabelAiInstSet.push_back(instance);
}

/// <summary>
/// 对轮廓覆盖到的图片部分进行提取
/// </summary>
/// <param name="imageToBeExtracted">待提取的图片</param>
/// <param name="extractedContours">提取的轮廓</param>
/// <param name="extractedImages">提取出的图片</param>
void DepthToPCL::ExtractImages(QImage* imageToBeExtracted, GraphicsPolygonItem* extractedContours, cv::Mat* extractedImages)
{
    cv::Mat currentImg = cv::Mat(imageToBeExtracted->height(), imageToBeExtracted->width(), CV_8UC(3), (void*)imageToBeExtracted->constBits(), imageToBeExtracted->bytesPerLine());
    
    cv::Mat selectionRangeImg = cv::Mat::zeros(currentImg.size(), CV_8UC3);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> contour;

    for (const QPointF& pointF : extractedContours->polygon()) {
        contour.push_back(cv::Point(pointF.x(), pointF.y()));
    }
    contours.push_back(contour);
    cv::drawContours(selectionRangeImg, contours, 0, cv::Scalar(255), -1);
    currentImg.copyTo(*extractedImages, selectionRangeImg);
}

/// <summary>
/// 重新渲染3D界面
/// </summary>
/// <param name="cloudin">需要显示的点云</param>
void DepthToPCL::reRendering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin)
{
    vtkWidget->viewer->removeAllPointClouds();
    vtkWidget->viewer->removeAllShapes();

    vtkWidget->viewer->addPointCloud<pcl::PointXYZ>(cloudin, "cloud");
    vtkWidget->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    vtkWidget->viewer->resetCamera();
    vtkWidget->update();
    vtkWidget->m_renderWindow->Render();//重新渲染
}

void DepthToPCL::ClearAllMarkedContent()
{
    for (int i = 0; i < labelVLayout->count(); ++i) {
        QWidget* widget = labelVLayout->itemAt(i)->widget();
        DynamicLabel* curlabel = qobject_cast<DynamicLabel*>(widget);
        if (nullptr != curlabel) {
            curlabel->LabelAiInstSet.clear();
        }
    }
}

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

void DepthToPCL::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Q) {
        on_startTagBtn_clicked();
    }
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
        cvImageWidget->setImage(currentDisplayImage);
        emit ConversionBetween2Dand3D();
    }
    else if(TwoDState->active()) {
        emit ConversionBetween2Dand3D();
    }
}

/// <summary>
/// 绘制轮廓按钮
/// </summary>
void DepthToPCL::on_drawCounterBtn_clicked()
{
    //提取二值化图像中的轮廓数据
    std::vector<std::vector<cv::Point> > contour_vec;
    cvMat2Contour(m_image, &contour_vec);
    contour_vec.erase(contour_vec.begin());
    //绘制单通道轮廓图像，背景为白色，轮廓线条用黑色
    cv::Mat blkImg(m_image.size(), CV_8UC1, cv::Scalar(255));
    drawContours(blkImg, contour_vec, -1, cv::Scalar(0), 2);
    QImage qImg = QImage((unsigned char*)(blkImg.data), blkImg.cols, blkImg.rows, blkImg.cols * blkImg.channels(), QImage::Format_Grayscale8);
    //scene->addPixmap(QPixmap::fromImage(qImg.scaled(cvImageWidget->size(), Qt::KeepAspectRatio)));
    cvImageWidget->setImage(qImg);
}


/// <summary>
/// 2D&3D状态下的标记完成
/// </summary>
void DepthToPCL::on_MarkCompleted_clicked()
{ 
    for (int i = 0; i < labelVLayout->count(); ++i) {
        QWidget* widget = labelVLayout->itemAt(i)->widget();
        DynamicLabel* curlabel = qobject_cast<DynamicLabel*>(widget);
        if (nullptr != curlabel) {
            for (int j = 0; j < curlabel->LabelAiInstSet.size(); ++j) {
                if (ThrDState->active()) {
                }
                else if (TwoDState->active()) {
                }
            }
        }
    }
}

/// <summary>
/// 绘制标记轮廓
/// </summary>
void DepthToPCL::on_drawMarkersBtn_clicked()
{
    for (int i = 0; i < labelVLayout->count(); ++i) {
        QWidget* widget = labelVLayout->itemAt(i)->widget();
        DynamicLabel* curlabel = qobject_cast<DynamicLabel*>(widget);
        if (nullptr != curlabel) {
            for (te::AiInstance aiContour : curlabel->LabelAiInstSet)
            {
                if (ThrDState->active()) {
                    AiInstSet2Cloud(curlabel);
                }
                else if (TwoDState->active()) {
                    AiInstSet2PolygonItem(curlabel);
                }
            }
        }
    }
}

void DepthToPCL::on_clearMarkersBtn_clicked()
{
    ClearAllMarkedContent();
    reRendering(vtkWidget->cloud->makeShared());
}

void DepthToPCL::SaveContour()
{
    on_MarkCompleted_clicked();
    te::Image obj;
    te::SampleMark gt_write;

    QString filePath = QFileDialog::getSaveFileName(nullptr, "选择保存路径和文件名", "", "(*.gt)");
    if (filePath.isEmpty())
    {
        return;
    }

    //gt_write.gtDataSet.assign(allResultAiInstance.begin(), allResultAiInstance.end());
    te::serializeJsonToOFStream(filePath.toStdString(), gt_write);
}

void DepthToPCL::LoadContour()
{
    te::Image obj;
    te::SampleMark gt_read;

    QString filePath = QFileDialog::getOpenFileName(nullptr, "选择文件", "", "(*.gt)");
    if (filePath.isEmpty())
    {
        return ;
    }

    te::deserializeJsonFromIFStream(filePath.toStdString(), &gt_read);
    //allResultAiInstance.assign(gt_read.gtDataSet.begin(), gt_read.gtDataSet.end());
}

/// <summary>
/// 高度系数的控制
/// </summary>
void DepthToPCL::on_ConfirmTransformationBtn_clicked()
{
    int factor = ui.HeightCoefficientSpinBox->value();
    for (int i = 0; i < vtkWidget->cloud->size(); ++i) {
        vtkWidget->cloud->at(i).z *= factor;
    }
    reRendering(vtkWidget->cloud->makeShared());
}

void DepthToPCL::ReceiveMarkedPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    addAiInstance(vtkWidget->currentdynamicLabel, cloud);
}

void DepthToPCL::ReceiveMarkedPolygonItem(GraphicsPolygonItem* polygonItem)
{
    addAiInstance(cvImageWidget->scene->currentdynamicLabel, polygonItem);
    AiInstSet2PolygonItem(cvImageWidget->scene->currentdynamicLabel);
}

/// <summary>
/// 添加标签按钮
/// </summary>
void DepthToPCL::on_addDynamicLabel_clicked()
{
    DynamicLabel* label = new DynamicLabel(tr("默认"));
    labelVLayout->addWidget(label);
    totalHeight += label->height();
    ui.scrollAreaWidgetContents->setGeometry(0, 0, label->width(), totalHeight);
}

/// <summary>
/// 删除标签按钮
/// </summary>
void DepthToPCL::on_delDynamicLabel_clicked()
{
    if (nullptr == vtkWidget->currentdynamicLabel)
        return;
    labelVLayout->removeWidget(vtkWidget->currentdynamicLabel);
    totalHeight -= vtkWidget->currentdynamicLabel->height();
    ui.scrollAreaWidgetContents->setGeometry(0, 0, vtkWidget->currentdynamicLabel->width(), totalHeight);
    vtkWidget->currentdynamicLabel->deleteLater();
    vtkWidget->currentdynamicLabel = nullptr;
}

/// <summary>
/// 开始标记按钮
/// </summary>
void DepthToPCL::on_startTagBtn_clicked()
{
    if (nullptr == vtkWidget->currentdynamicLabel)
        return;
    if (ThrDState->active()) {
        vtkWidget->isPickingMode = !vtkWidget->isPickingMode;
        if (vtkWidget->isPickingMode) {
            vtkWidget->line_id++;
            vtkWidget->cloud_polygon->clear();
            vtkWidget->flag = false;
            ui.startTagBtn->setText("结束标记");
        }
        else {
            ui.startTagBtn->setText("开始标记");
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
        "", tr("pcb files(*.pcd *.ply *.tif *.tiff) ;;All files (*.*)"));

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
        pcl::PointXYZ min;
        pcl::PointXYZ max;
        pcl::getMinMax3D(*vtkWidget->cloud, min, max);
        currentDisplayImageLength = max.x - min.x;
        currentDisplayImageHeight = max.y - min.y;
    }
    else if (fileName.endsWith("tif")|| fileName.endsWith("tiff")) {
        cv::Mat image = cv::imread(fileName.toStdString(), cv::IMREAD_UNCHANGED);
        if (image.empty()) {
            QMessageBox::warning(this, "Warning", "无法读取图像文件");
            return;
        }
        cvMat2Cloud(image, vtkWidget->cloud);

        currentDisplayImageLength = vtkWidget->cloud->width;
        currentDisplayImageHeight = vtkWidget->cloud->height;
    }
    else {
        QMessageBox::warning(this, "Warning", "点云读取格式错误！");
    }

    //移除窗口点云
    ClearAllMarkedContent();
    reRendering(vtkWidget->cloud->makeShared());
    Cloud2cvMat(vtkWidget->cloud, m_image);
    currentDisplayImage = QImage((unsigned char*)(m_image.data), m_image.cols, m_image.rows, m_image.cols * m_image.channels(), QImage::Format_RGB888);


    vtkWidget->viewer->resetCameraViewpoint("cloud");
}