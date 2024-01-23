#include "GraphicsPolygonScene.h"
#include "GraphicsPolygonItem.h"
#include <QCursor>
#include <QGraphicsSceneMouseEvent>
#include <QKeyEvent>
#include <QPolygon>
#include <QPolygonF>
#include <QPainterPath>
#include <QMessageBox>

GraphicsPolygonScene::GraphicsPolygonScene(QGraphicsScene* parent):QGraphicsScene(parent)
{
	InitStateMachine();
	isStart = true;
}

GraphicsPolygonScene::~GraphicsPolygonScene()
{
}

void GraphicsPolygonScene::InitStateMachine()
{
	m_pStateMachine = new QStateMachine();
	DragState = new QState(m_pStateMachine);
	MarkState = new QState(m_pStateMachine);
	BlankState = new QState(m_pStateMachine);
	DragState->addTransition(this, SIGNAL(StateChange()), BlankState);
	MarkState->addTransition(this, SIGNAL(StateChange()), DragState);
	BlankState->addTransition(this, SIGNAL(StateChange()), MarkState);

	m_pStateMachine->setInitialState(BlankState);
	m_pStateMachine->start();
}

void GraphicsPolygonScene::DrawPolygon(const te::PolygonF& polygon, QColor color)
{
	QPolygonF CurrentlyDawingPlygons;
	for (const te::Point2f& point : polygon) {
		QPointF contourPoint(point.x, point.y);
		CurrentlyDawingPlygons << contourPoint;
	}

	GraphicsPolygonItem* polyItem = new GraphicsPolygonItem();
	polyItem->setPen(QPen(Qt::white));
	polyItem->setBrush(QBrush(color));
	polyItem->setPolygon(CurrentlyDawingPlygons);
	addItem(polyItem);
}

void GraphicsPolygonScene::AiInstance2GraphicsPolygonItem(te::AiInstance* instance, QColor color)
{
	QPolygonF CurrentlyDawingPlygons;
	te::PolygonF maxpolygon = instance->contour.polygons[0];
	for (te::PolygonF polygon : instance->contour.polygons) {
		if (maxpolygon.size() < polygon.size()) {
			maxpolygon = polygon;
		}
	}
	for (const te::Point2f& point : maxpolygon) {
		QPointF contourPoint(point.x, point.y);
		CurrentlyDawingPlygons << contourPoint;
	}

	GraphicsPolygonItem* polyItem = new GraphicsPolygonItem();
	polyItem->setPen(QPen(Qt::white));
	polyItem->setBrush(QBrush(color));
	polyItem->setPolygon(CurrentlyDawingPlygons);
	addItem(polyItem);
}


void GraphicsPolygonScene::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
	QGraphicsScene::mousePressEvent(event);
	if (!event->isAccepted()) {
		if (event->button() == Qt::LeftButton) {
			if (MarkState->active()) {
				// 开始标记不规则框选区域
				if (isStart) {
					if (nullptr == currentdynamicLabel) {
						emit UnselectedTags();
						return;
					}
						
					currentMarkedPolygon << event->scenePos();
					polygon = new GraphicsPolygonItem();
					polygon->setPen(QPen(Qt::white));
					polygon->setBrush(QBrush(currentColor));
					polygon->setOpacity(0.5);
					addItem(polygon);
					//currentdynamicLabel->markedPolygons.append(polygon);
					isStart = false;
				}
			}
		}
		else if (event->button() == Qt::RightButton) {
			if (MarkState->active()) {
			}
		}
	}
}

void GraphicsPolygonScene::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
	if (MarkState->active() && !currentMarkedPolygon.isEmpty()) {
		// 更新不规则框选区域
		currentMarkedPolygon << event->scenePos();
		polygon->setPolygon(currentMarkedPolygon);
	}
	else{
		QGraphicsScene::mouseMoveEvent(event);
	}
}

void GraphicsPolygonScene::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
	if (event->button() == Qt::LeftButton) {
		
	}
	else if (event->button() == Qt::RightButton) {
		if (MarkState->active()) {
			isStart = true;
			currentMarkedPolygon.clear();
			emit GraphicsPolygonItemMarkingCompleted(polygon);
			removeItem(polygon);
		}
	}
	else {
		QGraphicsScene::mouseReleaseEvent(event);
	}
}

void GraphicsPolygonScene::keyPressEvent(QKeyEvent* event)
{
	if (event->key() == Qt::Key_Backspace) {
		// 移除所有选中的 items
		while (!selectedItems().isEmpty()) {
			removeItem(selectedItems().front());
		}
	} else if (event->key() == Qt::Key_C) {
		emit StateChange();
	}
	else {
		QGraphicsScene::keyPressEvent(event);
	}
}
