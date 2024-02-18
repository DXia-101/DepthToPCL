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
#include <QSettings>

#include <vector>
#include <Windows.h>
#include <string>
#include <stdexcept>

#include "DynamicLabel.h"
#include "teRapidjsonObjectTree.h"
#include "Transfer_Function.h"

#include "DataTransmission.h"

#include <time.h>
#include <thread>

#ifdef _WINDOWS

#include <direct.h>
#include <io.h>
#include <process.h>
#include <windows.h>

#undef min
#undef max
#else
#include <dirent.h>
#endif

#include "teTraining.h"
#include "tePrediction.h"
#include "teAugmentation.h"

#include "teImage.h"
#include "teObjectTreeWidget.h"
#include "teRapidjsonObjectTree.h"

#include "teEventLoopInvoke.h"
#include "teGeometricType.h"
#include "teTimer.h"

#include "Depth2RGB.h"

#include"teTableWidget.h"


DepthToPCL::DepthToPCL(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);

    PCL_Initalization();

    Interface_Initialization();

    InitStateMachine();
}

DepthToPCL::~DepthToPCL()
{
    delete workAiModel;
}

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

    QMenu* Train_menu = new QMenu("训练", menu_bar);

    QAction* Load_Images = new QAction("加载训练图片");
    QAction* Start_Train = new QAction("开始训练");
    QAction* Stop_Train = new QAction("停止训练");
    QAction* Start_Test = new QAction("开始测试");
    QAction* Train_Chart = new QAction("开始测试");

    Train_menu->addAction(Load_Images);
    Train_menu->addAction(Start_Train);
    Train_menu->addAction(Stop_Train);
    Train_menu->addAction(Start_Test);
    Train_menu->addAction(Train_Chart);

    menu_bar->addMenu(Train_menu);

    connect(Load_Images, &QAction::triggered, this, &DepthToPCL::LoadTrainingImages);
    connect(Start_Train, &QAction::triggered, this, &DepthToPCL::StartedTrainAction);
    connect(Stop_Train, &QAction::triggered, this, &DepthToPCL::StopTrainAction);
    connect(Start_Test, &QAction::triggered, this, &DepthToPCL::StartTestAction);
    connect(Train_Chart, &QAction::triggered, this, &DepthToPCL::ShowTrainChartAction);

    labelVLayout = new QVBoxLayout(ui.scrollArea);
    ui.scrollAreaWidgetContents->setLayout(labelVLayout);
    labelVLayout->addStretch(1);

    totalHeight = 0;
    point_size = 1;

    connect(vtkWidget, &VTKOpenGLNativeWidget::PointCloudMarkingCompleted, this, &DepthToPCL::ReceiveMarkedPointClouds);
    connect(teImageWidget, &ImageLabel::PolygonMarkingCompleted, this, &DepthToPCL::ReceiveMarkedPolygonItem);

    m_thrDMenuInterface = new _3DMenuInterface(vtkWidget);
    m_vtkToolBar = new VTKToolBar(vtkWidget);
    ui.vtkToolBarLayout->addWidget(m_vtkToolBar);
    ui.menuInterfaceLayout->addWidget(m_thrDMenuInterface);

    connect(m_vtkToolBar, &VTKToolBar::LoadingCompleted, this, &DepthToPCL::UpdatePointCloud2DImage);

    workAiModel = new AiModelInterface;
    trainChart = new TrainingStatisticsChart(this);
    trainChart->hide();

    DataTransmission::GetInstance()->connect(DataTransmission::GetInstance(), &DataTransmission::DataChanged, trainChart, &TrainingStatisticsChart::ReceiveData);

    this->showMaximized();

    connect(ui.AssertBrower, &TeSampWidget::sig_UpDateItem, this, [this](int* pIndex, int len)
        {
            for (int i = 0; i < len; i++) {
                ui.AssertBrower->teUpDateImg(pIndex[i], { QString::number(pIndex[i]) + "_thumb.bmp" }, QSize(256, 256), "Image");
            }
        });

    connect(ui.AssertBrower, &TeSampWidget::sig_ItemActive, this, [this](int* pIndex, int len)
        {
            for (int i = 0; i < len; i++) {
                if (!QFile::exists(QString::number(pIndex[i]) + "_thumb.bmp")) {
                    QFileInfo fileInfo(m_lstImgs[pIndex[i]]);
                    QString suffix = fileInfo.suffix().toLower();  // 获取并转换为小写
                    if ((suffix == "tif" || suffix == "tiff")){
                        std::string imgPath = m_lstImgs[pIndex[i]].toStdString();
                        cv::Mat image = cv::imread(imgPath, cv::IMREAD_UNCHANGED);

                        if (image.empty()) {
                            qDebug() << "Failed to load the TIF image.";
                            return;
                        }
                        cv::Mat median;
                        median.create(image.size(), CV_8UC3);
                        TeJetColorCode trans;
                        if (trans.cvt32F2BGR(image, median)) {
                            cv::cvtColor(median, median, cv::COLOR_BGR2RGB);
                            cv::Mat heatmap;
                            cv::applyColorMap(median, heatmap, cv::COLORMAP_JET);
                            cv::resize(heatmap, heatmap, cv::Size(80, 80));
                            cv::imwrite(std::to_string(pIndex[i]) + "_thumb.bmp", heatmap);
                        }
                    }
                    else {
                        te::Image img = te::Image::load(m_lstImgs[pIndex[i]].toStdString()).resize(te::Size(80, 80));
                        img.save(std::to_string(pIndex[i]) + "_thumb.bmp");
                        ui.AssertBrower->teUpdateThumb(pIndex[i], 0, QImage(QString::number(pIndex[i]) + "_thumb.bmp"), E_FORMAT_RGB);
                    }
                }
                if (!QFile::exists(QString::number(pIndex[i]) + "_thumb.pcd")) {
                    std::string imgPath = m_lstImgs[pIndex[i]].toStdString();
                    cv::Mat image = cv::imread(imgPath, cv::IMREAD_UNCHANGED);

                    if (image.empty()) {
                        qDebug() << "Failed to load the TIF image.";
                            return;
                    }
                    Transfer_Function::cvMat2Cloud(image, vtkWidget->mediancloud);
                    vtkWidget->SavePointCloud(QString::fromStdString(std::to_string(pIndex[i]) + "_thumb.pcd"), vtkWidget->mediancloud);
                }
            }
        });

    connect(ui.AssertBrower, &TeSampWidget::sig_SwitchImg, this, &DepthToPCL::SwitchDisplayItem);
}

void DepthToPCL::PCL_Initalization()
{
    vtkWidget = new VTKOpenGLNativeWidget(this);
    ui.VTKWidgetLayout->addWidget(vtkWidget);

    teImageWidget = new ImageLabel(this);
    ui.VTKWidgetLayout->addWidget(teImageWidget);
    teImageWidget->hide();

    SumPixNum = 0;
}

void DepthToPCL::InitStateMachine()
{
    m_pStateMachine = new QStateMachine();
    TwoDState = new QState(m_pStateMachine);
    ThrDState = new QState(m_pStateMachine);
    
    //2D状态下显示
    TwoDState->assignProperty(teImageWidget,"visible",true);
    TwoDState->assignProperty(vtkWidget,"visible",false);
    TwoDState->assignProperty(m_vtkToolBar,"visible",false);
    TwoDState->assignProperty(m_thrDMenuInterface,"visible",false);

    //3D状态下显示
    ThrDState->assignProperty(teImageWidget, "visible", false);
    ThrDState->assignProperty(vtkWidget,"visible",true);
    ThrDState->assignProperty(m_vtkToolBar,"visible",true);
    ThrDState->assignProperty(m_thrDMenuInterface,"visible",true);
    
    TwoDState->addTransition(this, SIGNAL(ConversionBetween2Dand3D()), ThrDState);
    ThrDState->addTransition(this, SIGNAL(ConversionBetween2Dand3D()), TwoDState);

    m_pStateMachine->addState(TwoDState);
    m_pStateMachine->addState(ThrDState);

    m_pStateMachine->setInitialState(TwoDState);
    m_pStateMachine->start();
}

void DepthToPCL::addAiInstance(DynamicLabel* curlabel, QList<QPolygonF>& Polygons)
{
    te::AiInstance instance;
    instance.name = curlabel->GetLabel().toStdString();
    te::PolygonF polygon;
    te::PolygonF::PointType point;
    for (const QPointF& polygonPoint : Polygons.front()) {
        point.x = static_cast<float>(polygonPoint.x());
        point.y = static_cast<float>(polygonPoint.y());
        polygon.push_back(point);
    }
    instance.contour.polygons.push_back(polygon);
    DataTransmission::GetInstance()->trainSamples[currentIndex].sampleMark.gtDataSet.push_back(instance);
    //curlabel->LabelAiInstSet.push_back(instance);
}

void DepthToPCL::addAiInstance(DynamicLabel* curlabel,pcl::PointCloud<pcl::PointXYZ>::Ptr markedCloud)
{
    cv::Mat image(0, 0, CV_32F);

    Transfer_Function::Cloud2cvMat(vtkWidget->axisset.curwidth, vtkWidget->axisset.curheight, vtkWidget->axisset.OriginX, vtkWidget->axisset.OriginY, markedCloud, image);
    SaveMatContour2Label(image,curlabel);
}

void DepthToPCL::AiInstSet2Cloud(DynamicLabel* curlabel)
{
    for (te::AiInstance instance : DataTransmission::GetInstance()->trainSamples[currentIndex].sampleMark.gtDataSet) {
        vtkWidget->AiInstance2Cloud(&instance,m_image,curlabel->GetColor());
    }
}

void DepthToPCL::AiInstSet2PolygonItem(DynamicLabel* curlabel)
{
    for (te::AiInstance instance : DataTransmission::GetInstance()->trainSamples[currentIndex].sampleMark.gtDataSet) {
        teImageWidget->AiInstance2GraphicsItem(&instance, curlabel->GetLabel(), curlabel->GetColor());
    }
}

void DepthToPCL::SaveMatContour2Label(cv::Mat& Matin, DynamicLabel* curlabel)
{
    te::AiInstance instance;
    instance.name = curlabel->GetLabel().toStdString();
    std::vector<std::vector<cv::Point>> contours;
    Transfer_Function::cvMat2Contour(Matin, &contours);
    //contours.erase(contours.begin());
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
    DataTransmission::GetInstance()->trainSamples[currentIndex].sampleMark.gtDataSet.push_back(instance);
    //curlabel->LabelAiInstSet.push_back(instance);
}

void DepthToPCL::ClearAllMarkedContent()
{
    for (auto sampleInfo : DataTransmission::GetInstance()->trainSamples) {
        sampleInfo.sampleMark.gtDataSet.clear();
    }
}

bool DepthToPCL::isLabelExist(QString curlabel)
{
    for (int i = 0; i < labelVLayout->count()-1; ++i) {
        QWidget* widget = labelVLayout->itemAt(i)->widget();
        DynamicLabel* m_curlabel = qobject_cast<DynamicLabel*>(widget);
        if (curlabel == m_curlabel->GetLabel()) {
            return true;
        }
        else {
            return false;
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
            teImageWidget->currentdynamicLabel = temp;
            currentLabelNAme = vtkWidget->currentdynamicLabel->GetLabel();
            teImageWidget->LabelChanged();
        }
    }
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
    QFileInfo fileInfo(m_lstImgs[currentIndex]);
    QString suffix = fileInfo.suffix().toLower();  // 获取并转换为小写
    if (TwoDState->active()) {
        vtkWidget->LoadPointCloud(QString::fromStdString(std::to_string(currentIndex) + "_thumb.pcd"));
        if (DataTransmission::GetInstance()->GetIsFilter()) {
            QSettings settings(QDir::currentPath() + "/config.ini");
            vtkWidget->DirectFilter(settings.value("StartLineEdit").toString(), settings.value("EndLineEdit").toString(), settings.value("Axis").toString(), settings.value("IsSave").toString());
        }
        vtkWidget->reRendering(vtkWidget->cloud->makeShared());
        
    }
    if (ThrDState->active()) {
        if ((suffix == "tif" || suffix == "tiff")) {
            cv::Mat image = cv::imread(m_lstImgs[currentIndex].toStdString(), cv::IMREAD_UNCHANGED);
            m_image = image.clone();
            if (image.empty()) {
                qDebug() << "Failed to load the TIF image.";
                return;
            }
            cv::Mat median;
            median.create(image.size(), CV_8UC3);
            TeJetColorCode trans;
            if (trans.cvt32F2BGR(image, median)) {
                cv::cvtColor(median, median, cv::COLOR_BGR2RGB);
                cv::Mat heatmap;
                cv::applyColorMap(median, heatmap, cv::COLORMAP_JET);
                currentDisplayImage = te::Image(heatmap).clone();
                teImageWidget->setImage(currentDisplayImage);
            }
        }
        else {
            currentDisplayImage.load(m_lstImgs[currentIndex].toStdString());
            teImageWidget->setImage(currentDisplayImage);
        }
    }
    emit ConversionBetween2Dand3D();
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
    //scene->addPixmap(QPixmap::fromImage(qImg.scaled(teImageWidget->size(), Qt::KeepAspectRatio)));
    te::Image displayImg = te::Image(blkImg, te::Image::E_Gray8);

    teImageWidget->setImage(displayImg);
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

                if (ThrDState->active()) {
                    AiInstSet2Cloud(curlabel);
                }
                else if (TwoDState->active()) {
                    AiInstSet2PolygonItem(curlabel);
                }
        }
    }
}

void DepthToPCL::on_clearMarkersBtn_clicked()
{
    if (TwoDState->active()) {
        ClearAllMarkedContent();
    }
    else if (ThrDState->active()) {
        ClearAllMarkedContent();
        vtkWidget->reRendering(vtkWidget->cloud->makeShared());
    }
}

void DepthToPCL::SaveContour()
{
    QString currentDir = QCoreApplication::applicationDirPath();
    te::Image obj;
    QFileInfo fileInfo(m_lstImgs[currentIndex]);
    QString fileName = fileInfo.baseName() + ".gt"; // 使用currentindex生成文件名
    QString filePath = currentDir + "/" + fileName; // 构建完整的文件路径
    te::serializeJsonToOFStream(filePath.toStdString(), DataTransmission::GetInstance()->trainSamples[currentIndex].sampleMark);
}

void DepthToPCL::LoadContour()
{
    te::Image obj;
    QString currentDir = QCoreApplication::applicationDirPath();

    for (int i = 0; i < labelVLayout->count(); ++i) {
        QWidget* widget = labelVLayout->itemAt(i)->widget();
        DynamicLabel* curlabel = qobject_cast<DynamicLabel*>(widget);
        if (nullptr != curlabel) {
            curlabel->deleteLater();
        }
    }
    for (auto sampleInfo : DataTransmission::GetInstance()->trainSamples) {
        sampleInfo.sampleMark.gtDataSet.clear();
    }

    for (int i = 0; i < SumPixNum; ++i) {
        QFileInfo fileInfo(m_lstImgs[i]);

        QString fileName = fileInfo.baseName() + ".gt";
        QString filePath = currentDir + "/" + fileName;

        if (te::deserializeJsonFromIFStream(filePath.toStdString(), &DataTransmission::GetInstance()->trainSamples[i].sampleMark)) {
            if (!isLabelExist(QString::fromStdString(DataTransmission::GetInstance()->trainSamples[i].sampleMark.gtDataSet.front().name))) {
                DynamicLabel* label = new DynamicLabel(QString::fromStdString(DataTransmission::GetInstance()->trainSamples[i].sampleMark.gtDataSet.front().name));
                label->SetColor(QColor(QRandomGenerator::global()->bounded(256), QRandomGenerator::global()->bounded(256), QRandomGenerator::global()->bounded(256)));
                labelVLayout->insertWidget(0, label);
                totalHeight += label->height();
                ui.scrollAreaWidgetContents->setGeometry(0, 0, label->width(), totalHeight);
            }
        }
    }
}

void DepthToPCL::ReceiveMarkedPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    addAiInstance(vtkWidget->currentdynamicLabel, cloud);
}

void DepthToPCL::ReceiveMarkedPolygonItem(QList<QPolygonF>& Polygons)
{
    addAiInstance(teImageWidget->currentdynamicLabel, Polygons);
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
    m_image.convertTo(imgShow, CV_8UC1);
    currentDisplayImage = te::Image(imgShow, te::Image::E_Gray8);
}

void DepthToPCL::LoadTrainingImages()
{
    m_lstImgs = QFileDialog::getOpenFileNames(nullptr);

    ui.AssertBrower->teUpDateSet(m_lstImgs.size(), 1);

    SumPixNum = ui.AssertBrower->teBrowserTable()->teGetArrayNum();
    DataTransmission::GetInstance()->InitTrainSamples(SumPixNum);
    /*QStringList fileNames = QFileDialog::getOpenFileNames(nullptr, "Select Training Files", QDir::homePath(), "TIFF Files (*.tif *.tiff)");

    TiffData.clear();
    GTData.clear();

    foreach(const QString & fileName, fileNames) {
        TiffData.push_back(fileName);
        QFileInfo fileInfo(fileName);
        QString baseName = fileInfo.baseName();

        QDir dir = fileInfo.absoluteDir();
        QStringList gtFileNames = dir.entryList(QStringList(baseName + ".gt"), QDir::Files);
        if (!gtFileNames.isEmpty()) {
            GTData.push_back(dir.absoluteFilePath(gtFileNames.first()));
        }
    }
    vTrainSamples.resize(TiffData.size());
    for (int i = 0; i < TiffData.size(); ++i) {
        std::string imagePath = TiffData.at(i).toStdString();
        std::string marksPath = GTData.at(i).toStdString();

        te::Image ss = te::Image::load(imagePath);

        vTrainSamples[i].sampleData.imageMatrix.push_back(ss);
        vTrainSamples[i].sampleData.roi = { 0,0,ss.width(), ss.height() };
        te::deserializeJsonFromIFStream(marksPath, &vTrainSamples[i].sampleMark);
    }*/
}

void DepthToPCL::StartedTrainAction()
{
    std::string fileName = "2.te";
    workAiModel->ParameterSettings(0, vTrainSamples, fileName.c_str());
    workAiModel->start();
}

void DepthToPCL::StopTrainAction()
{
}

void DepthToPCL::StartTestAction()
{
    std::string fileName = "2.te";
    int halfPrecise = 0;
    DeviceType deviceType = te::E_GPU;
    workAiModel->ParameterSettings(1, vTrainSamples, fileName.c_str(), halfPrecise, deviceType);

    workAiModel->start();
}

void DepthToPCL::ShowTrainChartAction()
{
    trainChart->show();
}

void DepthToPCL::SwitchDisplayItem(int iIndex, int iLayerIndex)
{
    currentIndex = iIndex;
    QFileInfo fileInfo(m_lstImgs[iIndex]);
    QString suffix = fileInfo.suffix().toLower();  // 获取并转换为小写
    if (ThrDState->active()) {
        vtkWidget->LoadPointCloud(QString::fromStdString(std::to_string(iIndex) + "_thumb.pcd"));
        if (DataTransmission::GetInstance()->GetIsFilter()) {
            QSettings settings(QDir::currentPath() + "/config.ini");
            vtkWidget->DirectFilter(settings.value("StartLineEdit").toString(), settings.value("EndLineEdit").toString(), settings.value("Axis").toString(), settings.value("IsSave").toString());
        }
        vtkWidget->reRendering(vtkWidget->cloud->makeShared());
    }
    if (TwoDState->active()) {
        if ((suffix == "tif" || suffix == "tiff")) {
            cv::Mat image = cv::imread(m_lstImgs[iIndex].toStdString(), cv::IMREAD_UNCHANGED);
            m_image = image.clone();
            if (image.empty()) {
                qDebug() << "Failed to load the TIF image.";
                return;
            }
            cv::Mat median;
            median.create(image.size(), CV_8UC3);
            TeJetColorCode trans;
            if (trans.cvt32F2BGR(image, median)) {
                cv::cvtColor(median, median, cv::COLOR_BGR2RGB);
                cv::Mat heatmap;
                cv::applyColorMap(median, heatmap, cv::COLORMAP_JET);
                currentDisplayImage = te::Image(heatmap).clone();
                teImageWidget->ClearMarks();
                teImageWidget->setImage(currentDisplayImage);
            }
        }
        else {
            currentDisplayImage.load(m_lstImgs[iIndex].toStdString());
            teImageWidget->setImage(currentDisplayImage);
        }
    }
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

    vtkWidget->currentdynamicLabel = label;
    teImageWidget->currentdynamicLabel = label;
    currentLabelNAme = label->GetLabel();
    teImageWidget->LabelChanged();
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
