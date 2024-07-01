#include "teDataBrowserController.h"
#include "Transfer_Function.h"
#include "pcl_function.h"
#include "Depth2RGB.h"

#include "teAiModel.h"

#include <QSettings>
#include <QDir>
#include <QVBoxLayout>
#include <QFileInfo>
#include <QDebug>
#include <QThread>

#define DEBUG

constexpr bool TwoD = false;
constexpr bool ThrD = true;

teDataBrowserController::teDataBrowserController(QObject *parent)
	: QObject(parent)
{
    CurrentState = TwoD;
	ImageBrowser = new TeSampWidget();
    thread = new QThread();
    worker = new teDataBrowserWorkThread();
    worker->setImageBrowser(ImageBrowser);
    worker->moveToThread(thread);
    
    connect(thread, &QThread::finished, worker, &QObject::deleteLater);
    connect(worker, &QObject::destroyed, thread, &QThread::quit);
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);

    connect(ImageBrowser, &TeSampWidget::sig_SwitchImg, this, &teDataBrowserController::SwitchImg);
    connect(this, &teDataBrowserController::sig_ChangeCurrentState, this, &teDataBrowserController::ChangeCurrentState);
    connect(ImageBrowser, &TeSampWidget::sig_UpDateItem, this, &teDataBrowserController::UpdateItem);
    connect(ImageBrowser, &TeSampWidget::sig_ItemActive, worker, &teDataBrowserWorkThread::ItemActive);
    connect(this, &teDataBrowserController::sig_teUpDataSet, this, &teDataBrowserController::teUpDataSet);
    
    thread->start();
}

teDataBrowserController::~teDataBrowserController()
{}

void teDataBrowserController::displayUIInWidget(QVBoxLayout * layout)
{
	layout->addWidget(ImageBrowser);
	ImageBrowser->show();
}

void teDataBrowserController::setteAiModel(teAiModel* aiModel)
{
    m_teAiModel = aiModel;
    worker->setteAiModel(aiModel);
}

void teDataBrowserController::UpdateItem(int* pIndex, int len)
{
    for (int i = 0; i < len; i++) {
        int index = pIndex[i];
        cv::Mat image = cv::imread(m_teAiModel->getOriginImage()[index], cv::IMREAD_UNCHANGED);
        if (!image.empty()) {
            QSize imageSize(image.cols, image.rows);

            QFileInfo fileInfo(QString::fromStdString(m_teAiModel->getOriginImage()[pIndex[i]]));
            QString fileName = fileInfo.fileName();

            ImageBrowser->teUpDateImg(pIndex[i], { QString::fromStdString(m_teAiModel->getShrinkageChart()[pIndex[i]]) }, imageSize, fileName);
        }
    }
}

void teDataBrowserController::SwitchImg(int pIndex, int len)
{
    m_teAiModel->setCurrentIndex(pIndex);
    emit sig_IndexChanged();
    emit sig_updateTrainWidget();
    emit sig_updateResultWidget();
    emit sig_NeedReload();
    if (CurrentState == ThrD) {
        emit sig_LoadPointCloud(QString::fromStdString(m_teAiModel->getCurrentPointCloud()));
    }
    else if (CurrentState == TwoD) {
        emit sig_LoadOriginImage(QString::fromStdString(m_teAiModel->getCurrentOriginImage()));
    }
}

void teDataBrowserController::teUpDataSet(int iNum, int iLayerNum, bool bReset)
{
    ImageBrowser->teUpDateSet(iNum, iLayerNum, bReset);
}

void teDataBrowserController::ChangeCurrentState()
{
    CurrentState = !CurrentState;
}
