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
    fitInView(sceneRect(), Qt::KeepAspectRatio); //保持纵横比，适应到视图中但不改变纵横比。
}

