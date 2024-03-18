#include "teMouseCircle.h"
#include <QPainter>
#include <QApplication>
#include <QPen>

QTransform getZoomMat(const QPoint& center, double dRatio)
{
    QTransform mat;

    mat.translate(center.x() * (1.0 - dRatio), center.y() * (1.0 - dRatio));

    mat.scale(dRatio, dRatio);

    return mat;
}

QTransform getResizeMat(Qt::Alignment align, const QSize& oldSize, const QSize& newSize)
{
    QTransform mat;

    double dDeltaSizeWidth = (newSize.width() - oldSize.width());
    double dDeltaSizeHeight = (newSize.height() - oldSize.height());

    double dDeltaX = 0.0;
    double dDeltaY = 0.0;

    if (align & Qt::AlignLeft) {
        dDeltaX = 0.0;
    }
    else if (align & Qt::AlignRight) {
        dDeltaX = dDeltaSizeWidth;
    }
    else if (align & Qt::AlignHCenter) {
        dDeltaX = dDeltaSizeWidth / 2.0;
    }

    if (align & Qt::AlignTop) {
        dDeltaY = 0.0;
    }
    else if (align & Qt::AlignBottom) {
        dDeltaY = dDeltaSizeHeight;
    }
    else if (align & Qt::AlignVCenter) {
        dDeltaY = dDeltaSizeHeight / 2.0;
    }

    mat.translate(dDeltaX, dDeltaY);

    return mat;
}


teMouseCircle::teMouseCircle(QWidget *parent)
	: QWidget(parent)
{
    setMouseTracking(true);
    setAttribute(Qt::WA_TranslucentBackground);
    setWindowFlags(windowFlags() | Qt::FramelessWindowHint);
    installEventFilter(this);
    radius = 10.0;
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

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(0, 0, 0, 0)); 
    painter.drawRect(rect());

    QPen pen(Qt::green);
    pen.setWidth(3);

    painter.setPen(pen);
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
    if (event->modifiers() == Qt::NoModifier) {
        int iDelta = event->delta();
        double dZoomRatio = 1.0 + iDelta / 1000.0;
        if ((radius * dZoomRatio) <= 0)
        {
            radius = 1.0;
        }
        else {
            radius *= dZoomRatio;
        }
    }
    transWheelEvents(event);
}

//传递鼠标事件
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

//传递滚轮事件
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

void teMouseCircle::receptiveFieldChange(int factor)
{
    radius = static_cast<float>(factor);
}