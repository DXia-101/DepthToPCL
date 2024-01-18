#include "GraphicsPolygonItem.h"
#include <QCursor>

GraphicsPolygonItem::GraphicsPolygonItem(QGraphicsItem* parent):
	QGraphicsPolygonItem(parent),
	m_StateFlag(DEFAULT_FLAG)
{
	setCursor(Qt::ArrowCursor);
	setFlag(QGraphicsItem::ItemIsMovable);
	setFlag(QGraphicsItem::ItemIsSelectable);
}

GraphicsPolygonItem::GraphicsPolygonItem(const QPolygonF& polygon, QGraphicsItem* parent):
	QGraphicsPolygonItem(polygon, parent),
	m_StateFlag(DEFAULT_FLAG)
{
	setCursor(Qt::ArrowCursor);
	setFlag(QGraphicsItem::ItemIsMovable);
	setFlag(QGraphicsItem::ItemIsSelectable);
}

GraphicsPolygonItem::~GraphicsPolygonItem()
{}

void GraphicsPolygonItem::mousePressEvent(QGraphicsSceneMouseEvent * event)
{
	if (event->button() == Qt::LeftButton) {
		setSelected(true);
	}
	else if (event->button() == Qt::RightButton) {
		event->ignore();
	}
}

void GraphicsPolygonItem::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
	QGraphicsItem::mouseMoveEvent(event);
}

void GraphicsPolygonItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
	QGraphicsItem::mouseReleaseEvent(event);
}

int GraphicsPolygonItem::type() const
{
	return UserType + 1;
}
