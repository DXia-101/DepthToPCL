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
}

teImageBrowserWorkThread::~teImageBrowserWorkThread()
{}

void teImageBrowserWorkThread::setImageBrowser(TeSampWidget * browser)
{
    ImageBrowser = browser;
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
                if (CV_MAT_DEPTH(image.type()) == CV_16U) {
                    if (!trans.cvt16Bit2BGR(teDataStorage::getInstance()->getCurrentInvalidPointThreshold(), teDataStorage::getInstance()->getCurrentValidPointThreshold(), image, median)) { return; }
                }
                else if (CV_MAT_DEPTH(image.type() == CV_32F)) {
                    if (!trans.cvt32F2BGR(teDataStorage::getInstance()->getCurrentInvalidPointThreshold(), teDataStorage::getInstance()->getCurrentValidPointThreshold(), image, median)) { return; }
                }
                cv::resize(median, median, cv::Size(80, 80));
                cv::imwrite(teDataStorage::getInstance()->GetCurrentPath().toStdString() + std::to_string(teDataStorage::getInstance()->getCurrentIndex()) + "_thumb.bmp", median);
                teDataStorage::getInstance()->updateShrinkageChart(teDataStorage::getInstance()->getCurrentIndex(), teDataStorage::getInstance()->GetCurrentPath().toStdString() + std::to_string(teDataStorage::getInstance()->getCurrentIndex()) + "_thumb.bmp");
            }
            else {
                te::Image img = te::Image::load(teDataStorage::getInstance()->getOriginImage()[pIndex[i]]).resize(te::Size(80, 80));
                img.save(std::to_string(pIndex[i]) + "_thumb.bmp");
                teDataStorage::getInstance()->updateShrinkageChart(pIndex[i], teDataStorage::getInstance()->GetCurrentPath().toStdString() + std::to_string(pIndex[i]) + "_thumb.bmp");
                ImageBrowser->teUpdateThumb(pIndex[i], 0, QImage(teDataStorage::getInstance()->GetCurrentPath() + QString::number(pIndex[i]) + "_thumb.bmp"), E_FORMAT_RGB);
            }
        }
        if (!QFile::exists(QString::fromStdString(teDataStorage::getInstance()->getPointCloud()[pIndex[i]]))) {
            std::string imgPath = teDataStorage::getInstance()->getOriginImage()[pIndex[i]];
            cv::Mat image = cv::imread(imgPath, cv::IMREAD_UNCHANGED);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr mediancloud = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();

            if (image.empty()) {
                qDebug() << "Failed to load the TIF image.";
                return;
            }
            Transfer_Function::cvMat2Cloud(teDataStorage::getInstance()->getSelectInvalidPointThreshold(pIndex[i]), teDataStorage::getInstance()->getSelectValidPointThreshold(pIndex[i]), image, mediancloud);
            te3DCanvasController::getInstance()->SavePointCloud(QString::fromStdString(teDataStorage::getInstance()->GetCurrentPath().toStdString() + std::to_string(pIndex[i]) + "_thumb.pcd"), mediancloud);
            teDataStorage::getInstance()->updatePointCloud(pIndex[i], teDataStorage::getInstance()->GetCurrentPath().toStdString() + std::to_string(pIndex[i]) + "_thumb.pcd");
        }
    }
}