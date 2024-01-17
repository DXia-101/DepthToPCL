#include "GraphicsPolygonScene.h"
#include "GraphicsPolygonItem.h"
#include <QCursor>
#include <QGraphicsSceneMouseEvent>
#include <QKeyEvent>
#include <QPolygon>

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

void GraphicsPolygonScene::mousePressEvent(QGraphicsSceneMouseEvent* event)
{
	QGraphicsScene::mousePressEvent(event);
	if (!event->isAccepted()) {
		if (event->button() == Qt::LeftButton) {
			if (MarkState->active()) {
				// 开始标记不规则框选区域
				if (isStart) {
					currentMarkedPolygon << event->scenePos();
					GraphicsPolygonItem* polygon = new GraphicsPolygonItem();
					polygon->setPen(QPen(Qt::white));
					polygon->setBrush(QBrush(currentColor));
					polygon->setOpacity(0.5);
					addItem(polygon);
					currentdynamicLabel->markedPolygons.append(polygon);
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
		currentdynamicLabel->markedPolygons.back()->setPolygon(currentMarkedPolygon);
	}
	else {
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
