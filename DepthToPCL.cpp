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
 * @brief �����ʼ��
 * @return
 */
void DepthToPCL::Interface_Initialization()
{
    QMenuBar* menu_bar = new QMenuBar(this);         
    ui.ToolBarHorizontalLayout->addWidget(menu_bar);
    menu_bar->setStyleSheet("font-size : 18px");

    QMenu* file_menu = new QMenu("�ļ�", menu_bar);

    QAction* open_action = new QAction("��ȡ");
    QAction* save_action = new QAction("����");
    QAction* quit_action = new QAction("�˳�");

    file_menu->addAction(open_action);
    file_menu->addAction(save_action);
    file_menu->addSeparator();      
    file_menu->addAction(quit_action);

    menu_bar->addMenu(file_menu);

    connect(open_action, &QAction::triggered, this, &DepthToPCL::Open_clicked);//��ȡ�ļ�
    connect(save_action, &QAction::triggered, this, &DepthToPCL::Save_clicked);//�����ļ�
    connect(quit_action, &QAction::triggered, this, &DepthToPCL::close);//�����ļ�

    QMenu* Display_menu = new QMenu("��ʾ", menu_bar);

    QAction* background_set = new QAction("��������");
    QAction* PointCloud_render = new QAction("������Ⱦ");
    QAction* color_set = new QAction("��ɫ����");
    QAction* dot_size = new QAction("��Ĵ�С");

    Display_menu->addAction(background_set);
    Display_menu->addAction(PointCloud_render);
    Display_menu->addAction(color_set);
    Display_menu->addAction(dot_size);

    menu_bar->addMenu(Display_menu);
    
    connect(background_set, &QAction::triggered, this, &DepthToPCL::Background_Select);//������ɫ����
    connect(PointCloud_render, &QAction::triggered, this, &DepthToPCL::PressBtn_rendering);//������ɫ����
    connect(color_set, &QAction::triggered, this, &DepthToPCL::PressBtn_colorSelect);//������ɫ����

    connect(dot_size, &QAction::triggered, this, &DepthToPCL::PressBtn_PointCloud_PointSize_slider);//���Ƶ��С����

    QMenu* Surround_menu = new QMenu("��Χ", menu_bar);

    QAction* AxisAlignedBounding = new QAction("AABB");
    QAction* OrientedBounding = new QAction("OBB");
    
    Surround_menu->addAction(AxisAlignedBounding);
    Surround_menu->addAction(OrientedBounding);

    menu_bar->addMenu(Surround_menu);

    connect(AxisAlignedBounding, &QAction::triggered, vtkWidget, &VTKOpenGLNativeWidget::AxisAlignedBoundingBox);
    connect(OrientedBounding, &QAction::triggered, vtkWidget, &VTKOpenGLNativeWidget::OrientedBoundingBox);

    QMenu* Filter_menu = new QMenu("�˲�", menu_bar);

    QAction* Gaussian_filter = new QAction("��˹�˲�");
    QAction* Direct_filter = new QAction("ֱͨ�˲�");

    Filter_menu->addAction(Gaussian_filter);
    Filter_menu->addAction(Direct_filter);

    menu_bar->addMenu(Filter_menu);

    connect(Gaussian_filter, &QAction::triggered, this, &DepthToPCL::GuassFilterAction);
    connect(Direct_filter, &QAction::triggered, this, &DepthToPCL::DirectFilterAction);

    isKeyDown_shift = false;

    QMenu* Contour_menu = new QMenu("����", menu_bar);

    QAction* Save_Contour = new QAction("��������");
    QAction* Load_Contour = new QAction("��������");

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
 * @brief PCL��ʼ������
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
/// ��ʼ��״̬��
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
/// ����ת��Ϊcv::Mat
/// </summary>
/// <param name="cloudin">Ҫת���ĵ���</param>
/// <param name="imageout">ת���ɹ���ͼ��</param>
void DepthToPCL::Cloud2cvMat(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, cv::Mat& imageout)
{
    pcl::PointXYZ min;//���ڴ�����������Сֵ
    pcl::PointXYZ max;//���ڴ������������ֵ
    pcl::getMinMax3D(*vtkWidget->cloud, min, max);
    
    if (!imageout.empty())
        imageout.release();
    imageout.create(currentDisplayImageHeight, currentDisplayImageLength, CV_8UC3);

    for (int i = 0; i < cloudin->size(); i++)
    {
        //������Ӧ����������
        int x = (cloudin->points[i].x - min.x);
        int y = (cloudin->points[i].y - min.y);

        //����ɫ��Ϣ��������
        if (x > 0 && x < currentDisplayImageLength && y>0 && y < currentDisplayImageHeight)
        {
            imageout.at<cv::Vec3b>(y, x)[0] = cloudin->points[i].z;
            imageout.at<cv::Vec3b>(y, x)[1] = cloudin->points[i].z;
            imageout.at<cv::Vec3b>(y, x)[2] = cloudin->points[i].z;
        }
    }
}

/// <summary>
/// cv::MatתΪ����
/// </summary>
/// <param name="imageIn">��ת����cv::Mat</param>
/// <param name="cloudOut">ת�����ĵ���</param>
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
/// cv::Mat��ʽ��ͼƬ��ȡ����
/// </summary>
/// <param name="Matin">��Ҫ��ȡ��cv::MatͼƬ</param>
/// <param name="contours">��ȡ����������</param>
void DepthToPCL::cvMat2Contour(cv::Mat& Matin, std::vector<std::vector<cv::Point>>* contours)
{
    cv::Mat grayImg, binImg;
    cv::cvtColor(Matin, grayImg, cv::COLOR_BGR2GRAY);
    threshold(grayImg, binImg, 0, 255, cv::ThresholdTypes::THRESH_OTSU);
    std::vector<cv::Vec4i> hierarchy;
    findContours(binImg, *contours, hierarchy, cv::RetrievalModes::RETR_TREE, cv::CHAIN_APPROX_NONE);//�Ὣ����ͼ���������������ȥ
}

/// <summary>
/// ����ͼ����������ǩ����
/// </summary>
/// <param name="Matin">������������ͼ��</param>
/// <param name="curlabel">���������ı�ǩ</param>
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
/// ���������ǵ���ͼƬ���ֽ�����ȡ
/// </summary>
/// <param name="imageToBeExtracted">����ȡ��ͼƬ</param>
/// <param name="extractedContours">��ȡ������</param>
/// <param name="extractedImages">��ȡ����ͼƬ</param>
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
/// ������Ⱦ3D����
/// </summary>
/// <param name="cloudin">��Ҫ��ʾ�ĵ���</param>
void DepthToPCL::reRendering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin)
{
    vtkWidget->viewer->removeAllPointClouds();
    vtkWidget->viewer->removeAllShapes();

    vtkWidget->viewer->addPointCloud<pcl::PointXYZ>(cloudin, "cloud");
    vtkWidget->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    vtkWidget->viewer->resetCamera();
    vtkWidget->update();
    vtkWidget->m_renderWindow->Render();//������Ⱦ
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
            qDebug() << "��ǰ��ǩ��" << vtkWidget->currentdynamicLabel->GetLabel();
        }
    }
}

void DepthToPCL::paintEvent(QPaintEvent* event)
{
    //QWidget::paintEvent(event);
    //// �����ѡ�е�DynamicLabel����
    //if (nullptr != vtkWidget->currentdynamicLabel)
    //{
    //    // ��ȡѡ�ж����λ�úʹ�С
    //    QPoint pos = vtkWidget->currentdynamicLabel->pos();
    //    QSize size = vtkWidget->currentdynamicLabel->size();

    //    // ����һ��QPainter�������ڻ�������
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
/// ���浱ǰ��ʾ����ĵ���
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
/// ��Y�ῴ��ȥ
/// </summary>
void DepthToPCL::on_ViewYBtn_clicked()
{
    vtkWidget->ViewYBtn();
}

/// <summary>
/// ��X�ῴ��ȥ
/// </summary>
void DepthToPCL::on_ViewXBtn_clicked()
{
    vtkWidget->ViewXBtn();
}

/// <summary>
/// ��Z�ῴ��ȥ
/// </summary>
void DepthToPCL::on_ViewZBtn_clicked()
{
    vtkWidget->ViewZBtn();
}

/// <summary>
/// ������ɫѡ��
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
/// XYZ������Ⱦ
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
/// ������ɫ����
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
/// ���Ƶ��С����Dialog
/// </summary>
void DepthToPCL::PressBtn_PointCloud_PointSize_slider()
{
    pointsize_set_dialog = new PointCloud_PointSize_Set_Dialog();
    connect(pointsize_set_dialog, SIGNAL(sendData(QString)), this, SLOT(PointCloud_PointSize_SliderSetting(QString)));
    if(pointsize_set_dialog->exec() == QDialog::Accepted){}
    delete pointsize_set_dialog;
}

/// <summary>
/// ���Ƶ�Ĵ�С����
/// </summary>
void DepthToPCL::PointCloud_PointSize_SliderSetting(QString data)
{
    point_size = data.toInt();

    for (int i = 0; i < 1; ++i) {
        vtkWidget->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "cloud");
    }
}

/// <summary>
/// ������ѡ���ٴο����ر�
/// </summary>
void DepthToPCL::on_FramePickBtn_clicked()
{
    keybd_event('X', 0, 0, 0);
    keybd_event('X', 0, KEYEVENTF_KEYUP, 0);
}

/// <summary>
/// ������ѡ���ٴε���ر�
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
/// ��˹�˲�ע�ᶯ��
/// </summary>
void DepthToPCL::GuassFilterAction()
{
    dialog_Guass_filter = new Filter_Guass();
    connect(dialog_Guass_filter, &Filter_Guass::sendData, vtkWidget, &VTKOpenGLNativeWidget::GuassFilter);
    if (dialog_Guass_filter->exec() == QDialog::Accepted) {}
    delete dialog_Guass_filter;
}

/// <summary>
/// ֱͨ�˲�����
/// </summary>
void DepthToPCL::DirectFilterAction()
{
    dialog_Direct_filter = new Filter_Direct();
    connect(dialog_Direct_filter, &Filter_Direct::sendData, vtkWidget, &VTKOpenGLNativeWidget::DirectFilter);
    if (dialog_Direct_filter->exec() == QDialog::Accepted) {}
    delete dialog_Direct_filter;
}

/// <summary>
/// ת�����水ť
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
/// ����������ť
/// </summary>
void DepthToPCL::on_drawCounterBtn_clicked()
{
    //��ȡ��ֵ��ͼ���е���������
    std::vector<std::vector<cv::Point> > contour_vec;
    cvMat2Contour(m_image, &contour_vec);
    contour_vec.erase(contour_vec.begin());
    //���Ƶ�ͨ������ͼ�񣬱���Ϊ��ɫ�����������ú�ɫ
    cv::Mat blkImg(m_image.size(), CV_8UC1, cv::Scalar(255));
    drawContours(blkImg, contour_vec, -1, cv::Scalar(0), 2);
    QImage qImg = QImage((unsigned char*)(blkImg.data), blkImg.cols, blkImg.rows, blkImg.cols * blkImg.channels(), QImage::Format_Grayscale8);
    //scene->addPixmap(QPixmap::fromImage(qImg.scaled(cvImageWidget->size(), Qt::KeepAspectRatio)));
    cvImageWidget->setImage(qImg);
}


/// <summary>
/// 2D&3D״̬�µı�����
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
/// ���Ʊ������
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

    QString filePath = QFileDialog::getSaveFileName(nullptr, "ѡ�񱣴�·�����ļ���", "", "(*.gt)");
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

    QString filePath = QFileDialog::getOpenFileName(nullptr, "ѡ���ļ�", "", "(*.gt)");
    if (filePath.isEmpty())
    {
        return ;
    }

    te::deserializeJsonFromIFStream(filePath.toStdString(), &gt_read);
    //allResultAiInstance.assign(gt_read.gtDataSet.begin(), gt_read.gtDataSet.end());
}

/// <summary>
/// �߶�ϵ���Ŀ���
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
/// ��ӱ�ǩ��ť
/// </summary>
void DepthToPCL::on_addDynamicLabel_clicked()
{
    DynamicLabel* label = new DynamicLabel(tr("Ĭ��"));
    labelVLayout->addWidget(label);
    totalHeight += label->height();
    ui.scrollAreaWidgetContents->setGeometry(0, 0, label->width(), totalHeight);
}

/// <summary>
/// ɾ����ǩ��ť
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
/// ��ʼ��ǰ�ť
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
            ui.startTagBtn->setText("�������");
        }
        else {
            ui.startTagBtn->setText("��ʼ���");
            vtkWidget->projectInliers(vtkWidget,currentLabelNAme);
        }
    }
    else if (TwoDState->active()) {

    }
}

/// <summary>
/// ���ļ�
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
            QMessageBox::warning(this, "Warning", "�޷���ȡͼ���ļ�");
            return;
        }
        cvMat2Cloud(image, vtkWidget->cloud);

        currentDisplayImageLength = vtkWidget->cloud->width;
        currentDisplayImageHeight = vtkWidget->cloud->height;
    }
    else {
        QMessageBox::warning(this, "Warning", "���ƶ�ȡ��ʽ����");
    }

    //�Ƴ����ڵ���
    ClearAllMarkedContent();
    reRendering(vtkWidget->cloud->makeShared());
    Cloud2cvMat(vtkWidget->cloud, m_image);
    currentDisplayImage = QImage((unsigned char*)(m_image.data), m_image.cols, m_image.rows, m_image.cols * m_image.channels(), QImage::Format_RGB888);


    vtkWidget->viewer->resetCameraViewpoint("cloud");
}