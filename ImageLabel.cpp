#include "ImageLabel.h"
#include <QMouseEvent>
#include <QGraphicsPolygonItem>

ImageLabel::ImageLabel(QWidget* parent)
    : QGraphicsView(parent)
{
    setDragMode(ScrollHandDrag);
    setRenderHint(QPainter::Antialiasing);
    scene = new GraphicsPolygonScene();
    setScene(scene);
    //connect(this, &ImageLabel::markedRegion, this, &ImageLabel::extractContours);
}

ImageLabel::~ImageLabel()
{
    delete scene;
}

void ImageLabel::setImage(const QImage& image)
{
    this->image = image;
    scene->clear();
    scene->addPixmap(QPixmap::fromImage(this->image));
    fitInView(sceneRect(), Qt::KeepAspectRatio);
}


//
//std::vector<std::vector<cv::Point>> ImageLabel::extractContours(const QImage& image, const QVector<QPolygonF>& polygons)
//{
//    cv::Mat cvImage(image.height(), image.width(), CV_8UC4, const_cast<uchar*>(image.constBits()), image.bytesPerLine());
//    cv::cvtColor(cvImage, cvImage, cv::COLOR_RGBA2GRAY);
//
//    cv::Mat mask(cvImage.size(), CV_8UC1, cv::Scalar(0));
//    std::vector<std::vector<cv::Point>> contours;
//
//    for (const QPolygonF& polygon : polygons) {
//        std::vector<cv::Point> cvPolygon;
//        for (const QPointF& point : polygon) {
//            cvPolygon.push_back(cv::Point(point.x(), point.y()));
//        }
//        contours.push_back(cvPolygon);
//    }
//    cv::drawContours(mask, contours, -1, cv::Scalar(255), cv::FILLED);
//
//    std::vector<std::vector<cv::Point>> extractedContours;
//    cv::findContours(mask, extractedContours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//
//    return extractedContours;
//}
