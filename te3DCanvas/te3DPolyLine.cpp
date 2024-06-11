#include "te3DPolyLine.h"
#include <QPainter>
#include <QMouseEvent>

te3DPolyLine::te3DPolyLine(QWidget *parent)
	: QWidget(parent)
{}

te3DPolyLine::~te3DPolyLine()
{}

void te3DPolyLine::InitPolyLine()
{
	setAttribute(Qt::WA_TranslucentBackground);//允许透明背景

	setWindowFlags(windowFlags() | Qt::FramelessWindowHint);// 无边框和标题栏

	setMouseTracking(true);

	SetDraw(true);
}

void te3DPolyLine::SetDraw(bool bDraw)
{
	this->bDraw = bDraw;
	bLeftClick = false;
	bOverDraw = true;
	pointList.clear();
	update();
}

QVector<QPointF>& te3DPolyLine::GetPointList()
{
    return pointList;
}

void te3DPolyLine::enterEvent(QEvent* event)
{
    setMouseTracking(true);
}

void te3DPolyLine::leaveEvent(QEvent* event)
{
    setMouseTracking(false);
}

void te3DPolyLine::paintEvent(QPaintEvent* e)
{
    QPainter painter(this);

    if (bDraw)
    {
        painter.setPen(QColor(255, 0, 0));
        QVector<QLineF> lines;
        for (int i = 0; i < pointList.size(); i++)
        {
            if (i > 0)
            {
                QLineF line(QPointF(pointList[i - 1].x(), pointList[i - 1].y()), QPointF(pointList[i].x(), pointList[i].y()));
                lines.push_back(line);
            }
        }
        if (pointList.size() > 0)
        {
            QLineF line(QPointF(pointList.back().x(), pointList.back().y()), mapFromGlobal(QCursor::pos()));
            lines.push_back(line);
        }
        painter.drawLines(lines);
    }
    else if (!bDraw && bOverDraw)
    {
        painter.setPen(QColor(255, 0, 0));
        QVector<QLineF> lines;
        for (int i = 0; i < pointList.size(); i++)
        {
            if (i > 0)
            {
                QLineF line(QPointF(pointList[i - 1].x(), pointList[i - 1].y()), QPointF(pointList[i].x(), pointList[i].y()));
                lines.push_back(line);
            }
        }
        painter.drawLines(lines);
    }
}

void te3DPolyLine::mousePressEvent(QMouseEvent* e)
{
    if (bDraw)
    {
        if (e->button() == Qt::LeftButton)
        {
            pointList.push_back(e->pos());
            bLeftClick = true;
            bOverDraw = false;
            this->update();
        }
        else if (e->button() == Qt::RightButton)
        {
            if (pointList.size() > 0)
            {
                pointList.push_back(pointList[0]);
            }
            bDraw = false;
            bLeftClick = false;
            bOverDraw = true;
            this->update();
            sig_DrawOver();
        }
    }
}

void te3DPolyLine::mouseMoveEvent(QMouseEvent* e)
{
    if (bDraw)
        this->update();
}

void te3DPolyLine::mouseReleaseEvent(QMouseEvent* e)
{
    if (bDraw && bLeftClick)
    {
        bLeftClick = false;
        this->update();
    }
}
