#include "teImageBrowserController.h"
#include <QVBoxLayout>
#include <QFileInfo>
#include "pcl_function.h"
#include "Depth2RGB.h"
#include "teImage.h"

teImageBrowserController::teImageBrowserController(QObject *parent)
	: QObject(parent)
{
	ImageBrowser = new TeSampWidget();
    connect(ImageBrowser, &TeSampWidget::sig_SwitchImg, this, &teImageBrowserController::SwitchImg, Qt::DirectConnection);
    connect(ImageBrowser, &TeSampWidget::sig_UpDateItem, this, &teImageBrowserController::UpdateItem);
    connect(ImageBrowser, &TeSampWidget::sig_ItemActive, this, &teImageBrowserController::ItemActive);
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
    //for (int i = 0; i < len; i++) {
    //    cv::Mat image = cv::imread(m_lstImgs[pIndex[i]].toStdString(), cv::IMREAD_UNCHANGED);
    //    if (!image.empty()) {
    //        QSize imageSize(image.cols, image.rows);

    //        QFileInfo fileInfo((m_lstImgs[i]));
    //        QString fileName = fileInfo.fileName();

    //        ImageBrowser->teUpDateImg(pIndex[i], { QString::number(pIndex[i]) + "_thumb.bmp" }, imageSize, fileName);
    //    }
    //}
}

void teImageBrowserController::ItemActive(int* pIndex, int len)
{
    //for (int i = 0; i < len; i++) {
    //    if (!QFile::exists(QString::number(pIndex[i]) + "_thumb.bmp")) {
    //        QFileInfo fileInfo(m_lstImgs[pIndex[i]]);
    //        QString suffix = fileInfo.suffix().toLower();  // 获取并转换为小写
    //        if ((suffix == "tif" || suffix == "tiff")) {
    //            std::string imgPath = m_lstImgs[pIndex[i]].toStdString();
    //            cv::Mat image = cv::imread(imgPath, cv::IMREAD_UNCHANGED);

    //            if (image.empty()) {
    //                qDebug() << "Failed to load the TIF image.";
    //                return;
    //            }
    //            cv::Mat median;
    //            median.create(image.size(), CV_8UC3);
    //            TeJetColorCode trans;
    //            if (trans.cvt32F2BGR(image, median)) {
    //                cv::cvtColor(median, median, cv::COLOR_BGR2RGB);
    //                cv::Mat heatmap;
    //                cv::applyColorMap(median, heatmap, cv::COLORMAP_JET);
    //                cv::resize(heatmap, heatmap, cv::Size(80, 80));
    //                cv::imwrite(std::to_string(pIndex[i]) + "_thumb.bmp", heatmap);
    //            }
    //        }
    //        else {
    //            te::Image img = te::Image::load(m_lstImgs[pIndex[i]].toStdString()).resize(te::Size(80, 80));
    //            img.save(std::to_string(pIndex[i]) + "_thumb.bmp");
    //            ImageBrowser->teUpdateThumb(pIndex[i], 0, QImage(QString::number(pIndex[i]) + "_thumb.bmp"), E_FORMAT_RGB);
    //        }
    //    }
    //    if (!QFile::exists(QString::number(pIndex[i]) + "_thumb.pcd")) {
    //        std::string imgPath = m_lstImgs[pIndex[i]].toStdString();
    //        cv::Mat image = cv::imread(imgPath, cv::IMREAD_UNCHANGED);

    //        if (image.empty()) {
    //            qDebug() << "Failed to load the TIF image.";
    //            return;
    //        }
    //        //Transfer_Function::cvMat2Cloud(image, vtkWidget->mediancloud);
    //        //vtkWidget->SavePointCloud(QString::fromStdString(std::to_string(pIndex[i]) + "_thumb.pcd"), vtkWidget->mediancloud);
    //    }
    //}
}

void teImageBrowserController::SwitchImg(int pIndex, int len)
{
    //currentIndex = iIndex;
    //QFileInfo fileInfo(m_lstImgs[iIndex]);
    //QString suffix = fileInfo.suffix().toLower();  // 获取并转换为小写
    //if (ThrDState->active()) {
    //    vtkWidget->LoadPointCloud(QString::fromStdString(std::to_string(iIndex) + "_thumb.pcd"));
    //    if (DataTransmission::GetInstance()->GetIsFilter()) {
    //        QSettings settings(QDir::currentPath() + "/config.ini");
    //        vtkWidget->DirectFilter(settings.value("StartLineEdit").toString(), settings.value("EndLineEdit").toString(), settings.value("Axis").toString(), settings.value("IsSave").toString());
    //    }
    //    vtkWidget->reRendering(vtkWidget->cloud->makeShared());
    //}
    //if (TwoDState->active()) {
    //    if ((suffix == "tif" || suffix == "tiff")) {
    //        cv::Mat image = cv::imread(m_lstImgs[iIndex].toStdString(), cv::IMREAD_UNCHANGED);
    //        m_image = image.clone();
    //        if (image.empty()) {
    //            qDebug() << "Failed to load the TIF image.";
    //            return;
    //        }
    //        cv::Mat median;
    //        median.create(image.size(), CV_8UC3);
    //        TeJetColorCode trans;
    //        if (trans.cvt32F2BGR(image, median)) {
    //            cv::cvtColor(median, median, cv::COLOR_BGR2RGB);
    //            cv::Mat heatmap;
    //            cv::applyColorMap(median, heatmap, cv::COLORMAP_JET);
    //            currentDisplayImage = te::Image(heatmap).clone();

    //            teImageWidget->ClearMarks();
    //            teImageWidget->setImage(currentDisplayImage);

    //            cv::waitKey(0);//如果不添加这一句会出现显示的图片没有切换的问题
    //        }
    //    }
    //    else {
    //        currentDisplayImage.load(m_lstImgs[iIndex].toStdString());
    //        teImageWidget->setImage(currentDisplayImage);
    //    }
    //}
    //DrawTestMarkers();
}
