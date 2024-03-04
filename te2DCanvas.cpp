#include "te2DCanvas.h"
#include <QMouseEvent>
#include <QGraphicsPolygonItem>

te2DCanvas::te2DCanvas(QWidget* parent)
    : te::GraphicsView(parent)
{
    this->addItemMgr(8);

    setBrushPriority(false);

    setAlignment(Qt::AlignJustify);
    InitStateMachine();
}

te2DCanvas::~te2DCanvas()
{
    
}

void te2DCanvas::AiInstance2GraphicsItem(te::AiInstance* instance,QString label,QColor color)
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

void te2DCanvas::ClearAll2DCanvasMarks()
{
    this->itemMgr(0)->clearItems();
}

void te2DCanvas::LabelChanged(const QString& content, const QColor& fontColor)
{
    graphicsBrush()[0]->setBrush(QBrush(fontColor));
    graphicsBrush()[1]->setBrush(QBrush(fontColor));
    graphicsBrush()[2]->setBrush(QBrush(fontColor));
    currentCategory = content;
    currentColor = fontColor;
}

void te2DCanvas::StartMarked()
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
    connect(PolygonBrush, &te::PolygonGraphicsBrush::sig_DrawPolygon, this, &te2DCanvas::DrawPolygonGraphics);
    connect(RectBrush, &te::RectGraphicsBrush::sig_DrawRect, this, &te2DCanvas::DrawRectGraphics);
    connect(LineBrush, &te::PolyLineGraphicsBrush::sig_DrawPolyLine, this, &te2DCanvas::DrawLineGraphics);
    
    this->setCurrentBrush(1);
}

void te2DCanvas::Redo()
{
    itemMgr(0)->redoItems();
    emit ClearCurrent2DCanvasMarkers();
    te2DCanvasMarkingCompleted();
}

void te2DCanvas::Undo()
{
    itemMgr(0)->undoItems();
    emit ClearCurrent2DCanvasMarkers();
    te2DCanvasMarkingCompleted();
}

void te2DCanvas::ShapeSelect(QString shape)
{
    if (shape.compare(QString::fromLocal8Bit("多边形")) == 0) {
        this->setCurrentBrush(1);    
    }
    else if (shape.compare(QString::fromLocal8Bit("矩形")) == 0) {
        this->setCurrentBrush(0);
    }else if (shape.compare(QString::fromLocal8Bit("折线")) == 0) {
        this->setCurrentBrush(2);
    }
}

void te2DCanvas::InitStateMachine()
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

void te2DCanvas::DrawPolygonGraphics(const QPolygonF& polygon)
{
    DrawGraphics({ polygon });
}

void te2DCanvas::DrawRectGraphics(const QRectF& rect)
{
    QPolygonF polygon;
    polygon << rect.topLeft()<< rect.topRight()<< rect.bottomRight()<< rect.bottomLeft();
    DrawGraphics({ polygon });
}

void te2DCanvas::DrawLineGraphics(const QList<QPolygonF>& polyline)
{
    DrawGraphics(polyline);
}

void te2DCanvas::DrawGraphics(const QList<QPolygonF>& region)
{
    te::ConnectedRegionGraphicsItem* polygonItem = new te::ConnectedRegionGraphicsItem(region, currentCategory);
    polygonItem->setPen(QColor(currentColor));
    polygonItem->setBrush(QBrush(currentColor));

    if (DrawState->active()) {
        //添加该item
        this->itemMgr(0)->clipItem(polygonItem);
    }
    else if (EraseState->active()) {
        QList<QPolygonF> contours = polygonItem->polygonList();
        this->itemMgr(0)->eraseItems(contours);
        emit ClearCurrent2DCanvasMarkers();
    }
    te2DCanvasMarkingCompleted();
}

void te2DCanvas::te2DCanvasMarkingCompleted()
{
    for (te::GraphicsItem* item : this->itemMgr(0)->items()) {
        te::ConnectedRegionGraphicsItem* polygonItem = dynamic_cast<te::ConnectedRegionGraphicsItem*>(item);
        emit PolygonMarkingCompleted(polygonItem);
    }
}
