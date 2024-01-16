#include "CategoryTag.h"

#include <QDrag>
#include <QMenu>
#include <QMimeData>
#include <QMouseEvent>
#include <QDebug>

CategoryTag::CategoryTag(const QString& text, bool status, QWidget* parent) : QLabel(text, parent)
{
    isBase = status;
    setStyleSheet("background-color: yellow; padding: 10px;");
    setAutoFillBackground(true);
    setAttribute(Qt::WA_DeleteOnClose);
    setContextMenuPolicy(Qt::CustomContextMenu);
    connect(this, &CategoryTag::customContextMenuRequested, this, &CategoryTag::showContextMenu);
}

CategoryTag::~CategoryTag()
{}
/// <summary>
/// ��ǩ����갴���¼�
/// ������ڸ�����ϰ���ʱ����
/// </summary>
/// <param name="event"></param>
void CategoryTag::mousePressEvent(QMouseEvent * event)
{
    if (event->button() == Qt::LeftButton) {
        qDebug() << "mousePressEvent";
        QDrag* drag = new QDrag(this);
        QMimeData* mimeData = new QMimeData;
        mimeData->setText(text());
        drag->setMimeData(mimeData);
        drag->exec(Qt::MoveAction);
        if (!isBase) {
            deleteLabel();
        }
    }
}

void CategoryTag::showContextMenu(const QPoint& pos)
{
    QMenu contextMenu(this);
    QAction* deleteAction = contextMenu.addAction("Delete");
    connect(deleteAction, &QAction::triggered, this, &CategoryTag::deleteLabel);
    contextMenu.exec(mapToGlobal(pos));
}

void CategoryTag::deleteLabel()
{
    close();
}
