#include "teImageBrowserController.h"
#include "Transfer_Function.h"
#include "pcl_function.h"
#include "Depth2RGB.h"

#include "teDataStorage.h"

#include <QSettings>
#include <QDir>
#include <QVBoxLayout>
#include <QFileInfo>
#include <QDebug>
#include <QElapsedTimer>
#include <QThread>

#define DEBUG

constexpr bool TwoD = false;
constexpr bool ThrD = true;

teImageBrowserController::teImageBrowserController(QObject *parent)
	: QObject(parent)
{
    CurrentState = TwoD;
	ImageBrowser = new TeSampWidget();
    thread = new QThread();
    worker = new teImageBrowserWorkThread();
    worker->setImageBrowser(ImageBrowser);
    worker->moveToThread(thread);
    
    connect(thread, &QThread::finished, worker, &QObject::deleteLater);
    connect(worker, &QObject::destroyed, thread, &QThread::quit);
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);

    connect(ImageBrowser, &TeSampWidget::sig_SwitchImg, this, &teImageBrowserController::SwitchImg);
    connect(this, &teImageBrowserController::sig_ChangeCurrentState, this, &teImageBrowserController::ChangeCurrentState);
    connect(ImageBrowser, &TeSampWidget::sig_UpDateItem, this, &teImageBrowserController::UpdateItem);
    connect(ImageBrowser, &TeSampWidget::sig_ItemActive, worker, &teImageBrowserWorkThread::ItemActive);
    connect(this, &teImageBrowserController::sig_teUpDataSet, this, &teImageBrowserController::teUpDataSet);
    
    thread->start();
}

teImageBrowserController::~teImageBrowserController()
{}

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
    emit sig_NeedReload();
    if (CurrentState == ThrD) {
        emit sig_LoadPointCloud(QString::fromStdString(teDataStorage::getInstance()->getCurrentPointCloud()));
    }
    else if (CurrentState == TwoD) {
        cv::Mat image = cv::imread(teDataStorage::getInstance()->getOriginImage()[pIndex], cv::IMREAD_UNCHANGED);
        if (image.empty()) {
            qDebug() << "Failed to load the TIF image.";
            return;
        }
        emit sig_ClearAll2DCanvasSymbol();
        TeJetColorCode trans;
        te::Image img = trans.dealWithCvt(image, pIndex);
        emit sig_SetImage(&img);
        emit sig_ShowAllItems();
    }
}

void teImageBrowserController::teUpDataSet(int iNum, int iLayerNum, bool bReset)
{
    ImageBrowser->teUpDateSet(iNum, iLayerNum, bReset);
}

void teImageBrowserController::ChangeCurrentState()
{
    CurrentState = !CurrentState;
}
