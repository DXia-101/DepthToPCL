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
/// �������������ק�¼�ʱ����
/// </summary>
/// <param name="event"></param>
void DropGraphicsView::dragEnterEvent(QDragEnterEvent * event)
{
	if (event->mimeData()->hasText()) {
		event->acceptProposedAction();
	}
}
/// <summary>
/// ���ڸ����������ק����갴���ɿ�ʱ����
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

