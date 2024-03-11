#include "teImageBrowserController.h"
#include "Transfer_Function.h"
#include <QVBoxLayout>
#include <QFileInfo>
#include "pcl_function.h"
#include "Depth2RGB.h"
#include "teImage.h"
#include "te3DCanvasController.h"
#include "te2DCanvasController.h"
#include "teDataStorage.h"
#include <QSettings>
#include <QDir>

constexpr bool TwoD = false;
constexpr bool ThrD = true;

teImageBrowserController::Garbo teImageBrowserController::tmp;

teImageBrowserController* teImageBrowserController::instance = nullptr;

teImageBrowserController::teImageBrowserController(QObject *parent)
	: QObject(parent)
{
	ImageBrowser = new TeSampWidget();
    
    connect(te3DCanvasController::getInstance(), &te3DCanvasController::sig_GTShowSignalChange, this, &teImageBrowserController::ChangeGTShowFlag);
    connect(te3DCanvasController::getInstance(), &te3DCanvasController::sig_RSTShowSignalChange, this, &teImageBrowserController::ChangeRSTShowFlag);
    GTShowFlag = false;
    RSTShowFlag = false;
    CurrentState = TwoD;

    connect(ImageBrowser, &TeSampWidget::sig_SwitchImg, this, &teImageBrowserController::SwitchImg, Qt::DirectConnection);
    connect(ImageBrowser, &TeSampWidget::sig_UpDateItem, this, &teImageBrowserController::UpdateItem);
    connect(ImageBrowser, &TeSampWidget::sig_ItemActive, this, &teImageBrowserController::ItemActive);
}

teImageBrowserController::~teImageBrowserController()
{}

teImageBrowserController::teImageBrowserController(const teImageBrowserController&)
{
    
}

teImageBrowserController& teImageBrowserController::operator=(const teImageBrowserController&)
{
    return *this;
}

teImageBrowserController* teImageBrowserController::getInstance()
{
    if (!instance)
    {
        teImageBrowserController* pInstance = new teImageBrowserController();
        instance = pInstance;
    }
    return instance;
}

void teImageBrowserController::destroy()
{
    if (NULL != teImageBrowserController::instance) {
        delete teImageBrowserController::instance;
        teImageBrowserController::instance = NULL;
    }
}

void teImageBrowserController::displayUIInWidget(QVBoxLayout * layout)
{
	layout->addWidget(ImageBrowser);
	ImageBrowser->show();
}

void teImageBrowserController::UpdateItem(int* pIndex, int len)
{
    for (int i = 0; i < len; i++) {
        cv::Mat image = cv::imread(teDataStorage::getInstance()->getOriginImage()[pIndex[i]], cv::IMREAD_UNCHANGED);
        if (!image.empty()) {
            QSize imageSize(image.cols, image.rows);

            QFileInfo fileInfo(QString::fromStdString(teDataStorage::getInstance()->getOriginImage()[i]));
            QString fileName = fileInfo.fileName();

            ImageBrowser->teUpDateImg(pIndex[i], { QString::fromStdString(teDataStorage::getInstance()->getShrinkageChart()[i]) }, imageSize, fileName);
        }
    }
}

void teImageBrowserController::ItemActive(int* pIndex, int len)
{
    for (int i = 0; i < len; i++) {
        if (!QFile::exists(QString::fromStdString(teDataStorage::getInstance()->getShrinkageChart()[pIndex[i]]))) {
            QFileInfo fileInfo(QString::fromStdString(teDataStorage::getInstance()->getOriginImage()[pIndex[i]]));
            QString suffix = fileInfo.suffix().toLower();
            if ((suffix == "tif" || suffix == "tiff")) {
                std::string imgPath = teDataStorage::getInstance()->getOriginImage()[pIndex[i]];
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
                    teDataStorage::getInstance()->updateShrinkageChart(pIndex[i], std::to_string(pIndex[i]) + "_thumb.bmp");
                }
            }
            else {
                te::Image img = te::Image::load(teDataStorage::getInstance()->getOriginImage()[pIndex[i]]).resize(te::Size(80, 80));
                img.save(std::to_string(pIndex[i]) + "_thumb.bmp");
                teDataStorage::getInstance()->updateShrinkageChart(pIndex[i], std::to_string(pIndex[i]) + "_thumb.bmp");
                ImageBrowser->teUpdateThumb(pIndex[i], 0, QImage(QString::number(pIndex[i]) + "_thumb.bmp"), E_FORMAT_RGB);
            }
        }
        if (!QFile::exists(QString::fromStdString(teDataStorage::getInstance()->getPointCloud()[pIndex[i]]))) {
            std::string imgPath = teDataStorage::getInstance()->getOriginImage()[pIndex[i]];
            cv::Mat image = cv::imread(imgPath, cv::IMREAD_UNCHANGED);
            pcl::PointCloud<pcl::PointXYZ>::Ptr mediancloud = (new pcl::PointCloud<pcl::PointXYZ>())->makeShared();

            if (image.empty()) {
                qDebug() << "Failed to load the TIF image.";
                return;
            }
            Transfer_Function::cvMat2Cloud(image, mediancloud);
            emit te3DCanvasController::getInstance()->sig_SavePointCloud(QString::fromStdString(std::to_string(pIndex[i]) + "_thumb.pcd"), mediancloud);
            teDataStorage::getInstance()->updatePointCloud(pIndex[i], std::to_string(pIndex[i]) + "_thumb.pcd");
        }
    }
}

void teImageBrowserController::SwitchImg(int pIndex, int len)
{
    teDataStorage::getInstance()->setCurrentIndex(pIndex);
    if (CurrentState == ThrD) {
        emit te3DCanvasController::getInstance()->sig_LoadPointCloud(QString::fromStdString(teDataStorage::getInstance()->getPointCloud()[pIndex]));
        emit te3DCanvasController::getInstance()->sig_ReRenderOriginCloud();
    }
    else if (CurrentState == TwoD) {
        cv::Mat image = cv::imread(teDataStorage::getInstance()->getOriginImage()[pIndex], cv::IMREAD_UNCHANGED);
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
            emit te2DCanvasController::getInstance()->sig_ClearAll2DCanvasMarks();
            te2DCanvasController::getInstance()->setImage(te::Image(heatmap).clone());
            cv::waitKey(0);
        }
        emit sig_showAllItem();
    }
}

void teImageBrowserController::ChangeGTShowFlag(int index)
{
    if (index > 0) {
        GTShowFlag = true;
    }
    else {
        GTShowFlag = false;
    }
}

void teImageBrowserController::ChangeRSTShowFlag(int index)
{
    if (index > 0) {
        RSTShowFlag = true;
    }
    else {
        RSTShowFlag = false;
    }
}

void teImageBrowserController::ChangeCurrentState()
{
    CurrentState = !CurrentState;
}

void teImageBrowserController::teUpDataSet(int iNum, int iLayerNum, bool bReset)
{
    ImageBrowser->teUpDateSet(iNum,iLayerNum,bReset);
}
