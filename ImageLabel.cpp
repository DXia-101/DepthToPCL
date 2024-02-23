#include "ImageLabel.h"
#include <QMouseEvent>
#include <QGraphicsPolygonItem>

ImageLabel::ImageLabel(QWidget* parent)
    : te::GraphicsView(parent)
{
    this->addItemMgr(8);

    setAlignment(Qt::AlignJustify);
    InitStateMachine();
}

ImageLabel::~ImageLabel()
{
    
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
    
    //��Ӹ�item
    this->itemMgr(0)->clipItem(polygonItem);
}

void ImageLabel::ClearMarks()
{
    this->itemMgr(0)->clearItems();
}

void ImageLabel::LabelChanged(const QString& content, const QColor& fontColor)
{
    graphicsBrush()[0]->setBrush(QBrush(fontColor));
    graphicsBrush()[1]->setBrush(QBrush(fontColor));
    graphicsBrush()[2]->setBrush(QBrush(fontColor));
    currentCategory = content;
    currentColor = fontColor;
}

void ImageLabel::StartMarked()
{
    te::RectGraphicsBrush* RectBrush = new te::RectGraphicsBrush;
    RectBrush->setPen(QPen(Qt::green, 2));
    this->addBrush(RectBrush);

    te::PolygonGraphicsBrush* PolygonBrush = new te::PolygonGraphicsBrush;
    PolygonBrush->setPen(QPen(Qt::green, 2));
    this->addBrush(PolygonBrush);

    te::PolyLineGraphicsBrush* LineBrush = new te::PolyLineGraphicsBrush;
    LineBrush->setPen(QPen(Qt::green, 2));
    LineBrush->setWidth(50);
    this->addBrush(LineBrush);
    connect(PolygonBrush, &te::PolygonGraphicsBrush::sig_DrawPolygon, this, &ImageLabel::DrawPolygonGraphics);
    connect(RectBrush, &te::RectGraphicsBrush::sig_DrawRect, this, &ImageLabel::DrawRectGraphics);
    connect(LineBrush, &te::PolyLineGraphicsBrush::sig_DrawPolyLine, this, &ImageLabel::DrawLineGraphics);
    
    this->setCurrentBrush(1);
}

void ImageLabel::ShapeSelect(QString shape)
{
    if (shape.compare(QString::fromLocal8Bit("�����")) == 0) {
        this->setCurrentBrush(1);    
    }
    else if (shape.compare(QString::fromLocal8Bit("����")) == 0) {
        this->setCurrentBrush(0);
    }else if (shape.compare(QString::fromLocal8Bit("����")) == 0) {
        this->setCurrentBrush(2);
    }
}

void ImageLabel::InitStateMachine()
{
    m_pStateMachine = new QStateMachine();
    DrawState = new QState(m_pStateMachine);
    EraseState = new QState(m_pStateMachine);

    DrawState->addTransition(this, SIGNAL(ReplaceToEraseState()), EraseState);
    EraseState->addTransition(this, SIGNAL(ReplaceToDrawState()), DrawState);

    m_pStateMachine->addState(DrawState);
    m_pStateMachine->addState(EraseState);

    m_pStateMachine->setInitialState(DrawState);
    m_pStateMachine->start();
}

void ImageLabel::DrawPolygonGraphics(const QPolygonF& polygon)
{
    DrawGraphics({ polygon });
}

void ImageLabel::DrawRectGraphics(const QRectF& rect)
{
    QPolygonF polygon;
    polygon << rect.topLeft()<< rect.topRight()<< rect.bottomRight()<< rect.bottomLeft();
    DrawGraphics({ polygon });
}

void ImageLabel::DrawLineGraphics(const QList<QPolygonF>& polyline)
{
    DrawGraphics(polyline);
}

void ImageLabel::DrawGraphics(const QList<QPolygonF>& region)
{
    te::ConnectedRegionGraphicsItem* polygonItem = new te::ConnectedRegionGraphicsItem(region, currentCategory);
    polygonItem->setPen(QColor(currentColor));
    polygonItem->setBrush(QBrush(currentColor));

    if (DrawState->active()) {
        //��Ӹ�item
        this->itemMgr(0)->clipItem(polygonItem);
    }
    else if (EraseState->active()) {
        QList<QPolygonF> contours = polygonItem->polygonList();
        this->itemMgr(0)->eraseItems(contours);
        emit ClearCurrentImageMarkers();
    }
    MarkingCompleted();
}

void ImageLabel::MarkingCompleted()
{
    for (te::GraphicsItem* item : this->itemMgr(0)->items()) {
        te::ConnectedRegionGraphicsItem* polygonItem = dynamic_cast<te::ConnectedRegionGraphicsItem*>(item);
        emit PolygonMarkingCompleted(polygonItem);
    }
}
