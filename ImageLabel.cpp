#include "ImageLabel.h"
#include <QMouseEvent>
#include <QGraphicsPolygonItem>

ImageLabel::ImageLabel(QWidget* parent)
    : te::GraphicsView(parent)
{
    this->addItemMgr(8);

    connect(&PolygonBrush, &te::PolygonGraphicsBrush::sig_DrawPolygon, this, &ImageLabel::DrawPolygonGraphics);
    connect(&RectBrush, &te::RectGraphicsBrush::sig_DrawRect, this, &ImageLabel::DrawRectGraphics);
    setAlignment(Qt::AlignJustify);
}

ImageLabel::~ImageLabel()
{

}

void ImageLabel::LabelChanged()
{
    RectBrush.setPen(QPen(Qt::green, 2));
    RectBrush.setBrush(QBrush(currentdynamicLabel->GetColor()));
    this->addBrush(&RectBrush);
    PolygonBrush.setPen(QPen(Qt::green, 2));
    PolygonBrush.setBrush(QBrush(currentdynamicLabel->GetColor()));
    this->addBrush(&PolygonBrush);
    
}

void ImageLabel::AiInstance2GraphicsItem(te::AiInstance* instance,QString label,QColor color)
{
    QList<QPolygonF> contours;
    QPolygonF polygonF;
    QPointF point;
    for (te::Point2f polygonPoint : instance->contour.polygons.front()) {
        point.setX(static_cast<float>(polygonPoint.x));
        point.setY(static_cast<float>(polygonPoint.y));
        polygonF.append(point);
    }
    contours.append(polygonF);
    te::ConnectedRegionGraphicsItem* polygonItem = new te::ConnectedRegionGraphicsItem({}, label);
    polygonItem->setPolygonList(contours);
    polygonItem->setPen(QColor(Qt::black));
    polygonItem->setBrush(QBrush(color));
    
    //添加该item
    this->itemMgr(0)->clipItem(polygonItem);
}

void ImageLabel::ClearMarks()
{
    this->itemMgr(0)->clearItems();
}

void ImageLabel::ShapeSelect(QString shape)
{
    if (shape.compare(QString::fromLocal8Bit("多边形")) == 0) {
        this->setCurrentBrush(1);    
    }
    else if (shape.compare(QString::fromLocal8Bit("矩形")) == 0) {
        this->setCurrentBrush(0);
    }
    
}

void ImageLabel::DrawPolygonGraphics(const QPolygonF& polygon)
{
    te::ConnectedRegionGraphicsItem* polygonItem = new te::ConnectedRegionGraphicsItem({ polygon }, currentdynamicLabel->GetLabel());
    polygonItem->setPen(QColor(Qt::black));
    polygonItem->setBrush(QBrush(currentdynamicLabel->GetColor()));

    QList<QPolygonF> contours = polygonItem->polygonList();

    //添加该item
    this->itemMgr(0)->clipItem(polygonItem);

    emit PolygonMarkingCompleted(contours);
}

void ImageLabel::DrawRectGraphics(const QRectF& rect)
{
    QPolygonF polygon;
    polygon << rect.topLeft()
        << rect.topRight()
        << rect.bottomRight()
        << rect.bottomLeft();

    te::ConnectedRegionGraphicsItem* polygonItem = new te::ConnectedRegionGraphicsItem({ polygon }, currentdynamicLabel->GetLabel());
    polygonItem->setPen(QColor(Qt::black));
    polygonItem->setBrush(QBrush(currentdynamicLabel->GetColor()));

    QList<QPolygonF> contours = polygonItem->polygonList();

    //添加该item
    this->itemMgr(0)->clipItem(polygonItem);

    emit PolygonMarkingCompleted(contours);
}
