#include "teImageBrowserWorkThread.h"
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
#include <QFileInfo>
#include <QDebug>

teImageBrowserWorkThread::teImageBrowserWorkThread(QObject*parent)
	: QObject(parent)
{
    GTShowFlag = false;
    RSTShowFlag = false;

    InvalidPointThreshold = 0;
    ValidPointThreshold = 800;
}

teImageBrowserWorkThread::~teImageBrowserWorkThread()
{}

void teImageBrowserWorkThread::setImageBrowser(TeSampWidget * browser)
{
    ImageBrowser = browser;
}

void teImageBrowserWorkThread::UpdateItem(int* pIndex, int len)
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

void teImageBrowserWorkThread::ItemActive(int* pIndex, int len)
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
                if (trans.cvt32F2BGR(InvalidPointThreshold, ValidPointThreshold, image, median)) {
                    cv::cvtColor(median, median, cv::COLOR_BGR2RGB);
                    cv::resize(median, median, cv::Size(80, 80));
                    cv::imwrite(std::to_string(pIndex[i]) + "_thumb.bmp", median);
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
            Transfer_Function::cvMat2Cloud(InvalidPointThreshold, ValidPointThreshold, image, mediancloud);
            emit sig_SavePointCloud(QString::fromStdString(std::to_string(pIndex[i]) + "_thumb.pcd"), mediancloud);
            teDataStorage::getInstance()->updatePointCloud(pIndex[i], std::to_string(pIndex[i]) + "_thumb.pcd");
        }
    }
}

void teImageBrowserWorkThread::teUpDataSet(int iNum, int iLayerNum, bool bReset)
{
    ImageBrowser->teUpDateSet(iNum, iLayerNum, bReset);
}

void teImageBrowserWorkThread::InvalidPointThresholdChange(int threshold)
{
    InvalidPointThreshold = threshold;
}

void teImageBrowserWorkThread::ValidPointThresholdChange(int threshold)
{
    ValidPointThreshold = threshold;
}
