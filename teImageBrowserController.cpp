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

    connect(ImageBrowser, &TeSampWidget::sig_SwitchImg, this, &teImageBrowserController::SwitchImg, Qt::DirectConnection);
    connect(this, &teImageBrowserController::sig_ChangeCurrentState, this, &teImageBrowserController::ChangeCurrentState);
    connect(ImageBrowser, &TeSampWidget::sig_UpDateItem, this, &teImageBrowserController::UpdateItem);
    connect(ImageBrowser, &TeSampWidget::sig_ItemActive, this, &teImageBrowserController::ItemActive);
    connect(this, &teImageBrowserController::sig_teUpDataSet,worker, &teImageBrowserWorkThread::teUpDataSet, Qt::QueuedConnection);
    connect(this, &teImageBrowserController::sig_GenerateCurrentData, worker, &teImageBrowserWorkThread::GenerateCurrentData);
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
    teDataStorage::getInstance()->updateTrainWidget(teDataStorage::getInstance()->getCurrentTrainMarksNumber());
    teDataStorage::getInstance()->updateResultWidget(teDataStorage::getInstance()->getCurrentResultMarksNumber());
    //mei ci tianjia tianjia biaoqian
    if (CurrentState == ThrD) {
#ifdef _Reckon_by_Time_
        timer.start();
#endif

        te3DCanvasController::getInstance()->LoadPointCloud(QString::fromStdString(teDataStorage::getInstance()->getCurrentPointCloud()));
#ifdef _Reckon_by_Time_
        qint64 elapsed1 = timer.elapsed();
        qDebug() << "LoadPointCloud took " << static_cast<double>(elapsed1) / 1000.0 << " seconds";
#endif

        te3DCanvasController::getInstance()->ShowAllItems();
#ifdef _Reckon_by_Time_
        qint64 elapsed2 = timer.elapsed() - elapsed1;
        qDebug() << "ShowAllItems took " << static_cast<double>(elapsed2) / 1000.0 << " seconds";
#endif
//
//        te3DCanvasController::getInstance()->MaintainCoordinateAxis();
//#ifdef _Reckon_by_Time_
//        qint64 elapsed3 = timer.elapsed() - elapsed2;
//        qDebug() << "Render to Coordinate took " << static_cast<double>(elapsed3) / 1000.0 << " seconds";
//#endif

        emit te3DCanvasController::getInstance()->sig_HeightTransform();
#ifdef _Reckon_by_Time_
        qint64 elapsed3 = timer.elapsed() - elapsed2;
        qDebug() << "HeightTransform took " << static_cast<double>(elapsed3) / 1000.0 << " seconds";
#endif

//        te3DCanvasController::getInstance()->SetCentroid();
//#ifdef _Reckon_by_Time_
//        qint64 elapsed4 = timer.elapsed() - elapsed3;
//        qDebug() << "SetCentroid took " << static_cast<double>(elapsed4) / 1000.0 << " seconds";
//#endif
    }
    else if (CurrentState == TwoD) {
        cv::Mat image = cv::imread(teDataStorage::getInstance()->getOriginImage()[pIndex], cv::IMREAD_UNCHANGED);
        if (image.empty()) {
            qDebug() << "Failed to load the TIF image.";
            return;
        }
        emit te2DCanvasController::getInstance()->sig_ClearAll2DCanvasSymbol();
        TeJetColorCode trans;
        trans.dealWithCvt(image, pIndex);
        te2DCanvasController::getInstance()->ShowAllItems();
    }
}

void teImageBrowserController::ItemActive(int* pIndex, int len)
{
    worker->setItemActive(pIndex, len);
    worker->run();
}

void teImageBrowserController::ChangeCurrentState()
{
    CurrentState = !CurrentState;
}
