#pragma once

#include <QObject>
#include <QGraphicsPolygonItem>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>
#include <QGraphicsItem>

enum STATE_FLAG {
	DEFAULT_FLAG = 0,
	MOV_RECT,
	ROTATE,
};

class GraphicsPolygonItem : public QGraphicsPolygonItem
{
public:
	explicit GraphicsPolygonItem(QGraphicsItem* parent = nullptr);
	explicit GraphicsPolygonItem(const QPolygonF& polygon,QGraphicsItem* parent = nullptr);
	~GraphicsPolygonItem();

	void mousePressEvent(QGraphicsSceneMouseEvent* event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent* event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent* event);
	int type() const;
private:
	QPointF m_centerPointF;
	bool m_bResizing;
	QPointF m_startPos;
	STATE_FLAG m_StateFlag;
	QPointF* pPointFofSmallRotateRect;
protected:
};
