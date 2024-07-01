#include "te2DCanvas.h"
#include <QMouseEvent>
#include <QGraphicsPolygonItem>

te2DCanvas::te2DCanvas(QWidget* parent)
    : te::GraphicsView(parent)
{
    this->addItemMgr(8);
    this->addItemMgr(8);

    setBrushPriority(false);

    setAlignment(Qt::AlignJustify);
    InitStateMachine();
}

te2DCanvas::~te2DCanvas()
{
    
}

void te2DCanvas::setImg(te::Image* img)
{
    setImage(*img);
}

void te2DCanvas::ShowDimension(int arg)
{
    if (arg > 0) {
        this->itemMgr(0)->setItemsVisible(true);
    }
    else {
        this->itemMgr(0)->setItemsVisible(false);
    }
}

void te2DCanvas::ShowResult(int arg)
{
    if (arg > 0) {
        this->itemMgr(1)->setItemsVisible(true);
    }
    else {
        this->itemMgr(1)->setItemsVisible(false);
    }
}

void te2DCanvas::ShowLocalMask(int arg)
{

}

void te2DCanvas::ShowGlobalMask(int arg)
{

}

void te2DCanvas::CurrentStateChanged(const QString& category, const QColor& fontColor, const int& index, const double& valThreshold, const double& invalThreshold)
{
    if (isStartMark) 
    {
        graphicsBrush()[0]->setBrush(QBrush(fontColor));
        graphicsBrush()[1]->setBrush(QBrush(fontColor));
        graphicsBrush()[2]->setBrush(QBrush(fontColor));
    }
    m_curstate.currentCategory = category;
    m_curstate.currentColor = fontColor;
    m_curstate.currentIndex = index;
    m_curstate.currentValidPointThreshold = valThreshold;
    m_curstate.currentInvalidPointThreshold = invalThreshold;
}

void te2DCanvas::StartMarked()
{
    isStartMark = true;

    RectBrush = new te::RectGraphicsBrush;
    RectBrush->setPen(QPen(Qt::green, 2));
    this->addBrush(RectBrush);

    PolygonBrush = new te::PolygonGraphicsBrush;
    PolygonBrush->setPen(QPen(Qt::green, 2));
    this->addBrush(PolygonBrush);

    LineBrush = new te::PolyLineGraphicsBrush;
    LineBrush->setPen(QPen(Qt::green, 2));
    LineBrush->setWidth(70);
    this->addBrush(LineBrush);
    connect(PolygonBrush, &te::PolygonGraphicsBrush::sig_DrawPolygon, this, &te2DCanvas::DrawPolygonGraphics);
    connect(RectBrush, &te::RectGraphicsBrush::sig_DrawRect, this, &te2DCanvas::DrawRectGraphics);
    connect(LineBrush, &te::PolyLineGraphicsBrush::sig_DrawPolyLine, this, &te2DCanvas::DrawLineGraphics);
    
    this->setCurrentBrush(1);
}

void te2DCanvas::Redo()
{
    itemMgr(0)->redoItems();
    te2DCanvasMarkingCompleted();
}

void te2DCanvas::Undo()
{
    itemMgr(0)->undoItems();
    te2DCanvasMarkingCompleted();
}

void te2DCanvas::MarkersShowInCanvas(te::AiInstance* instance, QString label, QColor color)
{
    QList<QPolygonF> contours;
    
    for (int i = 0; i < instance->contour.polygons.size(); ++i) {
        QPolygonF polygonF;
        for (te::Point2f polygonPoint : instance->contour.polygons.at(i))
        {
            QPointF point;
            point.setX(static_cast<float>(polygonPoint.x));
            point.setY(static_cast<float>(polygonPoint.y));
            polygonF.append(point);
        }
        contours.append(polygonF);
    }

    te::ConnectedRegionGraphicsItem* polygonItem = new te::ConnectedRegionGraphicsItem({}, label);
    polygonItem->setPolygonList(contours);
    polygonItem->setPen(QColor(Qt::black));
    //color.setAlpha(80);
    polygonItem->setBrush(QBrush(color));

    //添加该item
    this->itemMgr(0)->clipItem(polygonItem);
}

void te2DCanvas::ResultsShowInCanvas(te::AiInstance* instance, QString label, QColor color)
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
    //color.setAlpha(80);
    polygonItem->setBrush(QBrush(color));

    //添加该item
    this->itemMgr(1)->clipItem(polygonItem);
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
    if (polygon.size() < 4)
        return;
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
    te::ConnectedRegionGraphicsItem* polygonItem = new te::ConnectedRegionGraphicsItem(region, m_curstate.currentCategory);
    //currentColor.setAlpha(100);
    polygonItem->setPen(QColor(m_curstate.currentColor));
    polygonItem->setBrush(QBrush(m_curstate.currentColor));

    if (DrawState->active()) {
        //添加该item
        this->itemMgr(0)->clipItem(polygonItem);
    }
    else if (EraseState->active()) {
        QList<QPolygonF> contours = polygonItem->polygonList();
        this->itemMgr(0)->eraseItems(contours);
    }
    te2DCanvasMarkingCompleted();
}

void te2DCanvas::te2DCanvasMarkingCompleted()
{
    emit sig_PolygonMarkingCompleted(this->itemMgr(0)->items());
    emit sig_updateTrainWidget();
}

void te2DCanvas::RemoveDimentsion()
{
    this->itemMgr(0)->clearItems();
}

void te2DCanvas::RemoveResult()
{
    this->itemMgr(1)->clearItems();
}
