#include "teThreeDMarkView.h"
using namespace te;
ThreeDMarkView::ThreeDMarkView(QWidget* parent)
	: QWidget(parent)
{
	initWidget();
	setDrawState(true);
}

ThreeDMarkView::~ThreeDMarkView()
{}

void ThreeDMarkView::initWidget()
{
	setAttribute(Qt::WA_TranslucentBackground);//允许透明背景

	setWindowFlags(windowFlags() | Qt::FramelessWindowHint);// 无边框和标题栏

	setMouseTracking(true);

	setDrawState(true);
}

void ThreeDMarkView::setDrawState(bool bDraw)
{
	this->bDraw = bDraw;
	bLeftClick = false;
	bOverDraw = true;
	pointList.clear();
	this->update();
}

void ThreeDMarkView::enterEvent(QEvent* event)
{
	setMouseTracking(true);
}

void ThreeDMarkView::leaveEvent(QEvent* event)
{
	setMouseTracking(false);
}

void ThreeDMarkView::paintEvent(QPaintEvent* e)
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
		if (pointList.size() > 1)
		{
			QLineF line(QPointF(pointList.front().x(), pointList.front().y()), mapFromGlobal(QCursor::pos()));
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

void ThreeDMarkView::mousePressEvent(QMouseEvent* e)
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
			if (pointList.size() < 3)
			{
				setDrawState(true);
				return;
			}
			if (pointList.size() > 2)
			{
				pointList.push_back(pointList[0]);
			}
			bDraw = false;
			bLeftClick = false;
			bOverDraw = true;
			this->update();
			if (pointList.size() > 3)
				viewModel.lock()->setThreeDMarkerPolygon(pointList);
		}
	}
}

void ThreeDMarkView::mouseMoveEvent(QMouseEvent* e)
{
	if (bDraw)
		this->update();
}

void ThreeDMarkView::mouseReleaseEvent(QMouseEvent* e)
{
	if (bDraw && bLeftClick)
	{
		bLeftClick = false;
		this->update();
	}
}

void ThreeDMarkView::refresh(ViewModel::updateMode mode)
{
	if (mode == ViewModel::ThreeDMark)
	{
		setDrawState(true);
	}
	else if (mode == ViewModel::HideThreeDMark)
	{
		this->hide();
	}
}

void ThreeDMarkView::bindViewModel(std::shared_ptr<ViewModel> vm)
{
	viewModel = vm;
	if (viewModel.lock())
	{
		connect(viewModel.lock().get(), &ViewModel::notified, this, &ThreeDMarkView::refresh);
	}
}
