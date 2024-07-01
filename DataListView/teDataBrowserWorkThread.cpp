#include "teDataBrowserWorkThread.h"
#include "Transfer_Function.h"
#include "pcl_function.h"
#include "Depth2RGB.h"
#include "teImage.h"

#include "teAiModel.h"

#include <QSettings>
#include <QDir>
#include <QVBoxLayout>
#include <QFileInfo>
#include <QDebug>

teDataBrowserWorkThread::teDataBrowserWorkThread(QObject*parent)
	: QObject(parent)
{
    GTShowFlag = false;
    RSTShowFlag = false;
}

teDataBrowserWorkThread::~teDataBrowserWorkThread()
{}

void teDataBrowserWorkThread::setteAiModel(teAiModel * aiModel)
{
    m_teAiModel = aiModel;
}

void teDataBrowserWorkThread::setImageBrowser(TeSampWidget * browser)
{
    ImageBrowser = browser;
}

bool teDataBrowserWorkThread::SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud)
{
    if (saveCloud->empty()) {
        return false;
    }
    else {
        if (fileName.isEmpty()) {
            return false;
        }
        int return_status;
        if (fileName.endsWith(".pcd", Qt::CaseInsensitive))
            return_status = pcl::io::savePCDFileBinary(fileName.toStdString(), *saveCloud);
        else if (fileName.endsWith(".ply", Qt::CaseInsensitive))
            return_status = pcl::io::savePCDFileBinary(fileName.toStdString(), *saveCloud);
        else {
            fileName.append(".pcd");
            return_status = pcl::io::savePCDFileBinary(fileName.toStdString(), *saveCloud);
        }
        if (return_status != 0) {
            QString errorinfo = QString::fromStdString("Error writing point cloud" + fileName.toStdString());
            qDebug() << errorinfo;
            return false;
        }
    }
    return true;
}

void teDataBrowserWorkThread::ItemActive(int* pIndex, int len)
{
    for (int i = 0; i < len; i++) 
    {
        if (!QFile::exists(QString::fromStdString(m_teAiModel->getShrinkageChart()[pIndex[i]]))) 
        {
            QFileInfo fileInfo(QString::fromStdString(m_teAiModel->getOriginImage()[pIndex[i]]));
            QString suffix = fileInfo.suffix().toLower();
            if ((suffix == "tif" || suffix == "tiff")) 
            {
                std::string imgPath = m_teAiModel->getOriginImage()[pIndex[i]];
                cv::Mat image = cv::imread(imgPath, cv::IMREAD_UNCHANGED);

                if (image.empty() || (image.type() != CV_32FC1 && image.type() != CV_16UC1)) {
                    qDebug() << "Failed to load the TIF image.";
                    return;
                }
                double minValue, maxValue;
                cv::minMaxLoc(image, &minValue, &maxValue);
                minValue = -(maxValue * 0.8);

                m_teAiModel->addInvalidPointThreshold(pIndex[i],minValue);
                m_teAiModel->addValidPointThreshold(pIndex[i],maxValue);

                cv::Mat median;
                median.create(image.size(), CV_8UC3);
                TeJetColorCode trans;
                if (CV_MAT_DEPTH(image.type()) == CV_16U) {
                    if (!trans.cvt16Bit2BGR(m_teAiModel->getCurrentInvalidPointThreshold(), m_teAiModel->getCurrentValidPointThreshold(), image, median)) { return; }
                }
                else if (CV_MAT_DEPTH(image.type() == CV_32F)) {
                    if (!trans.cvt32F2BGR(m_teAiModel->getCurrentInvalidPointThreshold(), m_teAiModel->getCurrentValidPointThreshold(), image, median)) { return; }
                }
                cv::resize(median, median, cv::Size(80, 80));

                cv::imwrite(m_teAiModel->GetCurrentPath().toStdString() + std::to_string(m_teAiModel->getCurrentIndex()) + "_thumb.bmp", median);
                m_teAiModel->updateShrinkageChart(m_teAiModel->getCurrentIndex(), m_teAiModel->GetCurrentPath().toStdString() + std::to_string(m_teAiModel->getCurrentIndex()) + "_thumb.bmp");
            }
            else 
            {
                te::Image img = te::Image::load(m_teAiModel->getOriginImage()[pIndex[i]]).resize(te::Size(80, 80));
                img.save(std::to_string(pIndex[i]) + "_thumb.bmp");
                m_teAiModel->updateShrinkageChart(pIndex[i], m_teAiModel->GetCurrentPath().toStdString() + std::to_string(pIndex[i]) + "_thumb.bmp");
                ImageBrowser->teUpdateThumb(pIndex[i], 0, QImage(m_teAiModel->GetCurrentPath() + QString::number(pIndex[i]) + "_thumb.bmp"), E_FORMAT_RGB);
            }
        }
        if (!QFile::exists(QString::fromStdString(m_teAiModel->getPointCloud()[pIndex[i]]))) 
        {
            std::string imgPath = m_teAiModel->getOriginImage()[pIndex[i]];
            cv::Mat image = cv::imread(imgPath, cv::IMREAD_UNCHANGED);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr mediancloud = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();

            if (image.empty()) 
            {
                qDebug() << "Failed to load the TIF image.";
                return;
            }
            Transfer_Function::cvMat2Cloud(m_teAiModel->getSelectInvalidPointThreshold(pIndex[i]), m_teAiModel->getSelectValidPointThreshold(pIndex[i]), image, mediancloud);
            SavePointCloud(QString::fromStdString(m_teAiModel->GetCurrentPath().toStdString() + std::to_string(pIndex[i]) + "_thumb.pcd"), mediancloud);
            m_teAiModel->updatePointCloud(pIndex[i], m_teAiModel->GetCurrentPath().toStdString() + std::to_string(pIndex[i]) + "_thumb.pcd");
        }
    }
}