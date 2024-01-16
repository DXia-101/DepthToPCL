#include "DropGraphicsView.h"

#include <QMouseEvent>
#include <QMimeData>
#include <QDebug>

#include "CategoryTag.h"

DropGraphicsView::DropGraphicsView(QWidget *parent)
	: QLabel(parent)
{
	setAcceptDrops(true);
}

DropGraphicsView::~DropGraphicsView()
{}

/// <summary>
/// 当该组件中有拖拽事件时触发
/// </summary>
/// <param name="event"></param>
void DropGraphicsView::dragEnterEvent(QDragEnterEvent * event)
{
	if (event->mimeData()->hasText()) {
		event->acceptProposedAction();
	}
}
/// <summary>
/// 当在该组件中有拖拽的鼠标按键松开时触发
/// </summary>
/// <param name="event"></param>
void DropGraphicsView::dropEvent(QDropEvent* event)
{
    if (event->mimeData()->hasText()) {
        QPoint position = event->pos();
        QString text = event->mimeData()->text();

        CategoryTag* label = new CategoryTag(text, false, this);
        QPoint labelPos = label->mapToParent(QPoint(0, 0));
        QVariant pointVariant;
        pointVariant.setValue(labelPos);
        emit pointSignal(pointVariant);
        emit labelSignal(label->text());
        label->move(position);
        label->show();
        event->acceptProposedAction();
    }
}

