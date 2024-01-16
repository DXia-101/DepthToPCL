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
		if (event->modifiers() == Qt::ShiftModifier) {
			setSelected(true);
		}
		else if (event->modifiers() == Qt::AltModifier) {
			// 重置 item 大小
			double radius = boundingRect().width() / 2.0;
			QPointF topLeft = boundingRect().topLeft();
			m_centerPointF = QPointF(topLeft.x() + pos().x() + radius, topLeft.y() + pos().y() + radius);
			QPointF pos = event->scenePos();
			double dist = sqrt(pow(m_centerPointF.x() - pos.x(), 2) + pow(m_centerPointF.y() - pos.y(), 2));
			if (dist / radius > 0.8) { //判断 鼠标拖拽item的边界 是否为边界
				m_bResizing = true;
			}
			else {
				m_bResizing = false;
			}
		}
		else {
			QGraphicsItem::mousePressEvent(event);
			event->accept();
		}
	}
	else if (event->button() == Qt::RightButton) {
		event->ignore();
	}
}

void GraphicsPolygonItem::mouseMoveEvent(QGraphicsSceneMouseEvent* event)
{
	if ((event->modifiers() == Qt::AltModifier) && m_bResizing) {
		//QPointF pos = event->scenePos();
		//double dist = sqrt(pow(m_centerPointF.x() - pos.x(), 2) + pow(m_centerPointF.y() - pos.y(), 2));
		//setPolygon(m_centerPointF.x() - this->pos().x() - dist, //位置和大小
		//	m_centerPointF.y() - this->pos().y() - dist,
		//	dist * 2, dist * 2);
	}
	else {
		QGraphicsItem::mouseMoveEvent(event);
	}
}

void GraphicsPolygonItem::mouseReleaseEvent(QGraphicsSceneMouseEvent* event)
{
	if ((event->modifiers() == Qt::AltModifier) && m_bResizing) {
		m_bResizing = false;
	}
	else if(event->modifiers() != Qt::AltModifier ) {
		setCursor(Qt::ArrowCursor);
		if (m_StateFlag == MOV_RECT)
		{
			m_StateFlag = DEFAULT_FLAG;
		}
	}
	else {
		QGraphicsItem::mouseReleaseEvent(event);
	}
}

int GraphicsPolygonItem::type() const
{
	return UserType + 1;
}
