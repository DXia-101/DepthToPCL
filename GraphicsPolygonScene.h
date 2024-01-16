#pragma once

#include <QObject>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QKeyEvent>
#include <QStateMachine>
#include <QState>
#include <QPolygonF>
#include "GraphicsPolygonItem.h"
#include "DynamicLabel.h"

class GraphicsPolygonScene : public QGraphicsScene
{
	Q_OBJECT
public:
	GraphicsPolygonScene(QGraphicsScene* parent = nullptr);
	~GraphicsPolygonScene();
protected:
	// ×ó¼ü£ºÌí¼Óitem  ÓÒ¼ü£ºÒÆ³ýitem
	void mousePressEvent(QGraphicsSceneMouseEvent* event) override;
	void mouseMoveEvent(QGraphicsSceneMouseEvent* event) override;
	// Backspace¼üÒÆ³ýitem
	void keyPressEvent(QKeyEvent* event) override;

	void mouseReleaseEvent(QGraphicsSceneMouseEvent* event) override;

	void InitStateMachine();

signals:
	void markedRegion(const QPolygonF& polygon);
	void StateChange();
public:
	DynamicLabel* currentdynamicLabel;  //µ±Ç°±êÇ©¶ÔÏó
	QColor currentColor;
private:
	bool isStart;
	
	QStateMachine* m_pStateMachine;
	QState* MarkState;
	QState* DragState;
	QState* BlankState;

	QPoint markStartPoint;
	QPolygonF currentMarkedPolygon;
	
	GraphicsPolygonItem* draggingPolygonItem;
	QPointF dragStartPosition;
	QPointF prevDragPosition;
};
