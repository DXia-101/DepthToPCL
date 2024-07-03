#include "teMouseCircle.h"
#include <QPainter>
#include <QApplication>
#include <QPen>
#include <QStateMachine>
#include <QState>
#include <QMouseEvent>
#include <QWheelEvent>
#include <iostream>

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
    ThrDradius = 10.0;
    TwoDradius = 10.0;
    ReduceTimes = -1;
    MaxState = false;
    InitStateMachine();
}

void teMouseCircle::InitStateMachine()
{
    m_pStateMachine = new QStateMachine();
    TwoDState = new QState(m_pStateMachine);
    ThrDState = new QState(m_pStateMachine);

    TwoDState->addTransition(this, &teMouseCircle::sig_enterThrD, ThrDState);
    ThrDState->addTransition(this, &teMouseCircle::sig_enterTwoD, TwoDState);

    m_pStateMachine->addState(TwoDState);
    m_pStateMachine->addState(ThrDState);

    m_pStateMachine->setInitialState(TwoDState);
    m_pStateMachine->start();
}

teMouseCircle::~teMouseCircle()
{}

void teMouseCircle::restitution()
{
    MaxState = true;
}

void teMouseCircle::paintEvent(QPaintEvent * event)
{
    if (circleVisible) {
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
        if(ThrDState->active())
            painter.drawEllipse(centerPoint, ThrDradius, ThrDradius);
        else if (TwoDState->active())
            painter.drawEllipse(centerPoint, TwoDradius, TwoDradius);
        update();
    }
}

void teMouseCircle::mouseMoveEvent(QMouseEvent* event)
{
    if (circleVisible) {
        centerPoint = event->pos();
        update();
        transMouseEvents(event);
    }
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
    if (circleVisible) {
        transWheelEvents(event);
        if (event->modifiers() == Qt::NoModifier) {
            int iDelta = event->delta();
            double delta = 1.0;
            if (iDelta < 0)
                delta = 0.80;
            else
                delta = 1.25;
            double dZoomRatio = delta;
            if (ThrDState->active())
            {
                if ((ThrDradius * dZoomRatio) <= 1)
                {
                    MaxState = false;
                    ThrDradius = ThrDradius;
                    ReduceTimes++;
                }
                else {
                    if (ReduceTimes > 0) {
                        ReduceTimes--;
                    }
                    else {
                        if (!MaxState) {
                            ThrDradius *= dZoomRatio;
                        }
                        else if (dZoomRatio <= 1) {
                            MaxState = false;
                            ThrDradius *= dZoomRatio;
                        }
                    }
                }
            }
            else if (TwoDState->active())
            {
                if ((TwoDradius * dZoomRatio) <= 1)
                {
                    MaxState = false;
                    TwoDradius = ThrDradius;
                }
                else {
                    if (!MaxState) {
                        TwoDradius *= dZoomRatio;
                    }
                    else if (dZoomRatio <= 1) {
                        MaxState = false;
                        TwoDradius *= dZoomRatio;
                    }

                }
            }
        }
        update();
    }
}

void teMouseCircle::enterEvent(QEvent* event) {
    circleVisible = true; // 鼠标进入Widget时，圆圈出现
    update();
}

void teMouseCircle::leaveEvent(QEvent* event) {
    circleVisible = false; // 鼠标离开Widget时，圆圈消失
    update();
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

void teMouseCircle::OutOfBounds()
{
    ThrDradius *= 0.80;
    update();
}

void teMouseCircle::receptiveFieldChange(double factor)
{
    ThrDradius = static_cast<float>(factor);
    TwoDradius = static_cast<float>(factor);
    ReduceTimes = -1;
    update();
}