#include "ImageLabel.h"
#include <QMouseEvent>
#include <QGraphicsPolygonItem>

ImageLabel::ImageLabel(QWidget* parent)
    : te::GraphicsView(parent)
{
    this->addItemMgr(8);

    connect(&curbrush, &te::PolygonGraphicsBrush::sig_DrawPolygon, this, &ImageLabel::DrawPolygonGraphics);
    setAlignment(Qt::AlignJustify);
}

ImageLabel::~ImageLabel()
{

}

void ImageLabel::LabelChanged()
{
    curbrush.setPen(QPen(Qt::green, 2));
    curbrush.setBrush(QBrush(currentdynamicLabel->GetColor()));
    this->addBrush(&curbrush);
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
