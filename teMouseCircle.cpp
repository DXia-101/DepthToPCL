#include "teMouseCircle.h"
#include <QPainter>

teMouseCircle::teMouseCircle(QWidget *parent)
	: QWidget(parent)
{
    setMouseTracking(true);
    setAttribute(Qt::WA_TranslucentBackground);
    setWindowFlags(windowFlags() | Qt::FramelessWindowHint);
}

teMouseCircle::~teMouseCircle()
{}

void teMouseCircle::paintEvent(QPaintEvent * event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int radius = 10;
    int centerX = width() / 2;
    int centerY = height() / 2;

    // 绘制透明的背景
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(0, 0, 0, 0)); 
    painter.drawRect(rect());

    // 绘制不透明的圆形轮廓
    painter.setPen(Qt::green);
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(centerPoint, radius, radius);
}

void teMouseCircle::mouseMoveEvent(QMouseEvent* event)
{
    centerPoint = event->pos();
    update();
}
