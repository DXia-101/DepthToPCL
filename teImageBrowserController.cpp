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
#include <QTime>

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
    thread = new QThread();
    worker->moveToThread(thread);
        
    CurrentState = TwoD;
    InvalidPointThreshold = 0;
    ValidPointThreshold = 800;

    connect(ImageBrowser, &TeSampWidget::sig_SwitchImg, this, &teImageBrowserController::SwitchImg, Qt::DirectConnection);
    connect(this, &teImageBrowserController::sig_ChangeCurrentState, this, &teImageBrowserController::ChangeCurrentState);

    connect(ImageBrowser, &TeSampWidget::sig_UpDateItem, worker, &teImageBrowserWorkThread::UpdateItem,Qt::QueuedConnection);
    connect(ImageBrowser, &TeSampWidget::sig_ItemActive, worker, &teImageBrowserWorkThread::ItemActive,Qt::QueuedConnection);
    connect(worker, &teImageBrowserWorkThread::sig_showAll2DItem,this, &teImageBrowserController::sig_showAll2DItem, Qt::QueuedConnection);
    connect(worker, &teImageBrowserWorkThread::sig_SavePointCloud,this, &teImageBrowserController::sig_SavePointCloud, Qt::BlockingQueuedConnection);

    connect(this, &teImageBrowserController::sig_teUpDataSet,worker, &teImageBrowserWorkThread::teUpDataSet, Qt::QueuedConnection);
    connect(this, &teImageBrowserController::sig_InvalidPointThresholdChange,worker, &teImageBrowserWorkThread::InvalidPointThresholdChange, Qt::QueuedConnection);
    connect(this, &teImageBrowserController::sig_ValidPointThresholdChange,worker, &teImageBrowserWorkThread::ValidPointThresholdChange, Qt::QueuedConnection);

    thread->start();
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

void teImageBrowserController::SwitchImg(int pIndex, int len)
{
    teDataStorage::getInstance()->setCurrentIndex(pIndex);
    if (CurrentState == ThrD) {
        emit te3DCanvasController::getInstance()->sig_LoadPointCloud(QString::fromStdString(teDataStorage::getInstance()->getCurrentPointCloud()));
        emit te3DCanvasController::getInstance()->sig_ReRenderOriginCloud();
        emit te3DCanvasController::getInstance()->sig_ShowAllPointCloud();
        emit te3DCanvasController::getInstance()->sig_MaintainCoordinateAxis();
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
