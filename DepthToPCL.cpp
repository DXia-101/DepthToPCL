#include "DepthToPCL.h"

#pragma execution_character_set("utf-8")

#include <QDebug>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QFileDialog>
#include <QDir>
#include <QFile>
#include <QDirIterator>
#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QStatusBar>
#include <QKeyEvent>
#include <QRandomGenerator>

#include <vector>
#include <Windows.h>
#include <string>
#include <stdexcept>

#include "DynamicLabel.h"
#include "teRapidjsonObjectTree.h"
#include "Transfer_Function.h"


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

void DepthToPCL::Interface_Initialization()
{
    QMenuBar* menu_bar = new QMenuBar(this);         
    ui.ToolBarHorizontalLayout->addWidget(menu_bar);
    menu_bar->setStyleSheet("font-size : 18px");

    QMenu* file_menu = new QMenu("文件", menu_bar);

    QAction* quit_action = new QAction("退出");

    file_menu->addAction(quit_action);

    menu_bar->addMenu(file_menu);

    connect(quit_action, &QAction::triggered, this, &DepthToPCL::close);//保存文件

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

    connect(vtkWidget, &VTKOpenGLNativeWidget::PointCloudMarkingCompleted, this, &DepthToPCL::ReceiveMarkedPointClouds);
    connect(cvImageWidget->scene, &GraphicsPolygonScene::GraphicsPolygonItemMarkingCompleted, this, &DepthToPCL::ReceiveMarkedPolygonItem);
    connect(cvImageWidget->scene, &GraphicsPolygonScene::UnselectedTags, this, &DepthToPCL::WarningForUnselectedTags);

    m_thrDMenuInterface = new _3DMenuInterface(vtkWidget);
    m_vtkToolBar = new VTKToolBar(vtkWidget);
    ui.vtkToolBarLayout->addWidget(m_vtkToolBar);
    ui.menuInterfaceLayout->addWidget(m_thrDMenuInterface);

    connect(m_vtkToolBar, &VTKToolBar::LoadingCompleted, this, &DepthToPCL::UpdatePointCloud2DImage);
    connect(m_thrDMenuInterface, &_3DMenuInterface::SizeChange, this, &DepthToPCL::UpdatePointCloud2DImage);
}

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
    
    //2D状态下显示
    TwoDState->assignProperty(cvImageWidget,"visible",true);
    TwoDState->assignProperty(vtkWidget,"visible",false);
    TwoDState->assignProperty(m_vtkToolBar,"visible",false);
    TwoDState->assignProperty(m_thrDMenuInterface,"visible",false);

    //3D状态下显示
    ThrDState->assignProperty(cvImageWidget, "visible", false);
    ThrDState->assignProperty(vtkWidget,"visible",true);
    ThrDState->assignProperty(m_vtkToolBar,"visible",true);
    ThrDState->assignProperty(m_thrDMenuInterface,"visible",true);
    
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
    cv::Mat image(0, 0, CV_32F);

    Transfer_Function::Cloud2cvMat(vtkWidget->axisset.curwidth, vtkWidget->axisset.curheight, vtkWidget->axisset.OriginX, vtkWidget->axisset.OriginY, markedCloud, image);
    SaveMatContour2Label(image,curlabel);
}

void DepthToPCL::AiInstSet2Cloud(DynamicLabel* curlabel)
{
    for (te::AiInstance instance : curlabel->LabelAiInstSet) {
        vtkWidget->AiInstance2Cloud(&instance,m_image,curlabel->GetColor());
    }
}

void DepthToPCL::AiInstSet2PolygonItem(DynamicLabel* curlabel)
{
    for (te::AiInstance instance : curlabel->LabelAiInstSet) {
        cvImageWidget->scene->AiInstance2GraphicsPolygonItem(&instance, curlabel->GetColor());
    }
}

void DepthToPCL::SaveMatContour2Label(cv::Mat& Matin, DynamicLabel* curlabel)
{
    te::AiInstance instance;
    instance.name = curlabel->GetLabel().toStdString();
    std::vector<std::vector<cv::Point>> contours;
    Transfer_Function::cvMat2Contour(Matin, &contours);
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
    curlabel->LabelAiInstSet.push_back(instance);
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
    Transfer_Function::cvMat2Contour(m_image, &contour_vec);
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
    if (TwoDState->active()) {
        ClearAllMarkedContent();
        cvImageWidget->scene->clear();
    }
    else if (ThrDState->active()) {
        ClearAllMarkedContent();
        vtkWidget->reRendering(vtkWidget->cloud->makeShared());
    }


}

void DepthToPCL::SaveContour()
{
    te::Image obj;
    QString filePath = QFileDialog::getSaveFileName(nullptr, "选择保存路径和文件名", "", "(*.gt)");
    std::cout << labelVLayout->count() << std::endl;
    for (int i = 0; i < labelVLayout->count()-1; ++i) {   //从1开始是因为Layout里面添加了一个Stretch，需要跳过他
        QWidget* widget = labelVLayout->itemAt(i)->widget();
        DynamicLabel* curlabel = qobject_cast<DynamicLabel*>(widget);
        te::SampleMark gt_write;
        //gt_write.gtDataSet = curlabel->LabelAiInstSet;
        gt_write.gtDataSet.assign(curlabel->LabelAiInstSet.begin(), curlabel->LabelAiInstSet.end());

        if (filePath.isEmpty())
        {
            return;
        }
        te::serializeJsonToOFStream(filePath.toStdString(), gt_write);
    }
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

    for (int i = 0; i < labelVLayout->count(); ++i) {
        QWidget* widget = labelVLayout->itemAt(i)->widget();
        DynamicLabel* curlabel = qobject_cast<DynamicLabel*>(widget);
        if (nullptr != curlabel) {
            curlabel->LabelAiInstSet.clear();
            curlabel->deleteLater();
        }
    }

    if (te::deserializeJsonFromIFStream(filePath.toStdString(), &gt_read)) {
        DynamicLabel* label = new DynamicLabel(QString::fromStdString(gt_read.gtDataSet.front().name));
        label->LabelAiInstSet = gt_read.gtDataSet;
        label->SetColor(QColor(QRandomGenerator::global()->bounded(256), QRandomGenerator::global()->bounded(256), QRandomGenerator::global()->bounded(256)));
        labelVLayout->insertWidget(0,label);
        totalHeight += label->height();
        ui.scrollAreaWidgetContents->setGeometry(0, 0, label->width(), totalHeight);
    }
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

void DepthToPCL::WarningForUnselectedTags()
{
    QMessageBox::warning(this, tr("错误"), tr("没有选择标签"));
}

void DepthToPCL::UpdatePointCloud2DImage()
{
    ClearAllMarkedContent();

    Transfer_Function::Cloud2cvMat(vtkWidget->axisset.curwidth, vtkWidget->axisset.curheight, vtkWidget->axisset.OriginX, vtkWidget->axisset.OriginY, vtkWidget->cloud, m_image);
    cv::Mat imgShow;
    m_image.convertTo(imgShow, CV_8UC1);/*
    currentDisplayImage = QImage(static_cast<uchar*>(imgShow.data), imgShow.cols, imgShow.rows, imgShow.cols * imgShow.channels(), QImage::Format_Grayscale8);*/
    currentDisplayImage = QImage((const unsigned char*)imgShow.data, imgShow.cols, imgShow.rows, imgShow.step, QImage::Format_Grayscale8).copy();
    
}

/// <summary>
/// 添加标签按钮
/// </summary>
void DepthToPCL::on_addDynamicLabel_clicked()
{
    DynamicLabel* label = new DynamicLabel(tr(""));
    labelVLayout->insertWidget(0, label);
    //labelVLayout->addWidget(label);
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
    if (nullptr == vtkWidget->currentdynamicLabel) {
        WarningForUnselectedTags();
        return;
    }
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
            vtkWidget->PolygonSelect(vtkWidget,currentLabelNAme);
        }
    }
    else if (TwoDState->active()) {

    }
}
