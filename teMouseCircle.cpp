#include "teMouseCircle.h"
#include <QPainter>
#include <QApplication>

teMouseCircle::teMouseCircle(QWidget *parent)
	: QWidget(parent)
{
    setMouseTracking(true);
    setAttribute(Qt::WA_TranslucentBackground);
    setWindowFlags(windowFlags() | Qt::FramelessWindowHint);
    installEventFilter(this);
    radius = 10;
}

teMouseCircle::~teMouseCircle()
{}

void teMouseCircle::paintEvent(QPaintEvent * event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int centerX = width() / 2;
    int centerY = height() / 2;

    // ����͸���ı���
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(0, 0, 0, 0)); 
    painter.drawRect(rect());

    // ���Ʋ�͸����Բ������
    painter.setPen(Qt::green);
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(centerPoint, radius, radius);
}

void teMouseCircle::mouseMoveEvent(QMouseEvent* event)
{
    centerPoint = event->pos();
    update();
    transMouseEvents(event);
}

void teMouseCircle::mousePressEvent(QMouseEvent* event)
{
    transMouseEvents(event);
}

void teMouseCircle::mouseReleaseEvent(QMouseEvent* event)
{
    transMouseEvents(event);
}

void teMouseCircle::wheelEvent(QWheelEvent* event)
{
    transWheelEvents(event);
}

//��������¼�
void teMouseCircle::transMouseEvents(QMouseEvent* event)
{
    if (this->parentWidget()) {
        this->setAttribute(Qt::WA_TransparentForMouseEvents, true);

        QPoint pt = this->mapTo(this->parentWidget(), event->pos());
        QWidget* wid = this->parentWidget()->childAt(pt);
        if (wid) {
            pt = wid->mapFrom(this->parentWidget(), pt);
            QMouseEvent* mEvent = new QMouseEvent(event->type(), pt, event->button(), event->buttons(), event->modifiers());
            QApplication::postEvent(wid, mEvent);
        }

        this->setAttribute(Qt::WA_TransparentForMouseEvents, false);
    }
}

//���ݹ����¼�
void teMouseCircle::transWheelEvents(QWheelEvent* event)
{
    if (this->parentWidget()) {
        this->setAttribute(Qt::WA_TransparentForMouseEvents, true);

        QPoint pt = this->mapTo(this->parentWidget(), event->pos());
        QWidget* wid = this->parentWidget()->childAt(pt);
        if (wid) {
            pt = wid->mapFrom(this->parentWidget(), pt);
            QWheelEvent* wheelEvent = new QWheelEvent(pt, event->globalPos(), event->pixelDelta(), event->angleDelta(), event->buttons(), event->modifiers(), event->phase(), event->inverted());
            QApplication::postEvent(wid, wheelEvent);
        }

        this->setAttribute(Qt::WA_TransparentForMouseEvents, false);
    }
}
