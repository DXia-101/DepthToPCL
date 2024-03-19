#include "teImageBrowserController.h"
#include "Transfer_Function.h"
#include "pcl_function.h"
#include "Depth2RGB.h"
#include "teImage.h"
#include "te3DCanvasController.h"
#include "te2DCanvasController.h"
#include "teDataStorage.h"

#include <QSettings>
#include <QDir>
#include <QVBoxLayout>
#include <QFileInfo>BlockingQueuedConnection
#include <QDebug>
#include <QElapsedTimer>

#define DEBUG

teImageBrowserController::Garbo teImageBrowserController::tmp;

teImageBrowserController* teImageBrowserController::instance = nullptr;

constexpr bool TwoD = false;
constexpr bool ThrD = true;

teImageBrowserController::teImageBrowserController(QObject *parent)
	: QObject(parent)
{
	ImageBrowser = new TeSampWidget();
    worker = new teImageBrowserWorkThread();
    worker->setImageBrowser(ImageBrowser);
        
    CurrentState = TwoD;
    InvalidPointThreshold = 0;
    ValidPointThreshold = 800;

    connect(ImageBrowser, &TeSampWidget::sig_SwitchImg, this, &teImageBrowserController::SwitchImg, Qt::DirectConnection);
    connect(this, &teImageBrowserController::sig_ChangeCurrentState, this, &teImageBrowserController::ChangeCurrentState);
    connect(ImageBrowser, &TeSampWidget::sig_UpDateItem, this, &teImageBrowserController::UpdateItem);
    connect(ImageBrowser, &TeSampWidget::sig_ItemActive, this, &teImageBrowserController::ItemActive);
    connect(this, &teImageBrowserController::sig_InvalidPointThresholdChange, this, &teImageBrowserController::InvalidPointThresholdChange);
    connect(this, &teImageBrowserController::sig_ValidPointThresholdChange, this, &teImageBrowserController::ValidPointThresholdChange);

    connect(this, &teImageBrowserController::sig_teUpDataSet,worker, &teImageBrowserWorkThread::teUpDataSet, Qt::QueuedConnection);
    connect(this, &teImageBrowserController::sig_InvalidPointThresholdChange,worker, &teImageBrowserWorkThread::InvalidPointThresholdChange, Qt::QueuedConnection);
    connect(this, &teImageBrowserController::sig_ValidPointThresholdChange,worker, &teImageBrowserWorkThread::ValidPointThresholdChange, Qt::QueuedConnection);


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

            QFileInfo fileInfo(QString::fromStdString(teDataStorage::getInstance()->getOriginImage()[pIndex[i]]));
            QString fileName = fileInfo.fileName();

            ImageBrowser->teUpDateImg(pIndex[i], { QString::fromStdString(teDataStorage::getInstance()->getShrinkageChart()[pIndex[i]]) }, imageSize, fileName);
        }
    }
}


void teImageBrowserController::SwitchImg(int pIndex, int len)
{
    teDataStorage::getInstance()->setCurrentIndex(pIndex);
    if (CurrentState == ThrD) {
        te3DCanvasController::getInstance()->LoadPointCloud(QString::fromStdString(teDataStorage::getInstance()->getCurrentPointCloud()));

        te3DCanvasController::getInstance()->ReRenderOriginCloud();

        te3DCanvasController::getInstance()->ShowAllItems();

        te3DCanvasController::getInstance()->MaintainCoordinateAxis();

        emit sig_HeightTransform();
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
        if (trans.cvt32F2BGR(InvalidPointThreshold, ValidPointThreshold, image, median)) {
            cv::cvtColor(median, median, cv::COLOR_BGR2RGB);
            emit te2DCanvasController::getInstance()->sig_ClearAll2DCanvasMarks();
            te2DCanvasController::getInstance()->setImage(te::Image(median).clone());
            cv::waitKey(0);
        }
        emit sig_showAll2DItem();
    }
}

void teImageBrowserController::ItemActive(int* pIndex, int len)
{
    worker->setItemActive(pIndex, len);
    worker->run();
}

void teImageBrowserController::InvalidPointThresholdChange(int threshold)
{
    InvalidPointThreshold = threshold;
}

void teImageBrowserController::ValidPointThresholdChange(int threshold)
{
    ValidPointThreshold = threshold;
}

void teImageBrowserController::ChangeCurrentState()
{
    CurrentState = !CurrentState;
}
