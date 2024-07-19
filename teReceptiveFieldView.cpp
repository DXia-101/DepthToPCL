#include "teReceptiveFieldView.h"
#include <QPainter>
#include <QApplication>
#include <QPen>
#include <QStateMachine>
#include <QState>
#include <QMouseEvent>
#include <QWheelEvent>
#include "teReceptiveFieldViewMenber.h"
using namespace te;

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

ReceptiveFieldView::ReceptiveFieldView(QWidget* parent)
	: QWidget(parent)
{
	setMouseTracking(true);
	setAttribute(Qt::WA_TranslucentBackground);
	setWindowFlags(windowFlags() | Qt::FramelessWindowHint);
	installEventFilter(this);
	menber = new ReceptiveFieldViewMenber();
	menber->ThrDRadius = 10.0;
	menber->TwoDRadius = 10.0;
	menber->ReduceTimes = -1;
	menber->MaxState = false;
	menber->circleVisible = true;
	initStateMachine();
}

ReceptiveFieldView::~ReceptiveFieldView()
{
}

void ReceptiveFieldView::refresh(ViewModel::updateMode mode)
{
	if (mode == ViewModel::ReceptiveField)
	{
		receptiveFieldChange();
	}
	else if (mode == ViewModel::StateChange)
	{
		if (menber->ThrDState->active())
			emit sig_enterTwoD();
		else if (menber->TwoDState->active())
			emit sig_enterThrD();
	}
	else if (mode == ViewModel::OutOfBounds)
	{
		outOfBounds();
	}
}

void ReceptiveFieldView::initStateMachine()
{
	menber->stateMachine = new QStateMachine();
	menber->TwoDState = new QState(menber->stateMachine);
	menber->ThrDState = new QState(menber->stateMachine);

	menber->TwoDState->addTransition(this, &ReceptiveFieldView::sig_enterThrD, menber->ThrDState);
	menber->ThrDState->addTransition(this, &ReceptiveFieldView::sig_enterTwoD, menber->TwoDState);

	menber->stateMachine->addState(menber->TwoDState);
	menber->stateMachine->addState(menber->ThrDState);

	menber->stateMachine->setInitialState(menber->TwoDState);
	menber->stateMachine->start();
}

void te::ReceptiveFieldView::receptiveFieldChange()
{
	menber->ThrDRadius = static_cast<float>(viewModel.lock()->getReceptiveField());
	menber->TwoDRadius = static_cast<float>(viewModel.lock()->getReceptiveField());
	menber->ReduceTimes = -1;
	update();
}

void ReceptiveFieldView::paintEvent(QPaintEvent* event)
{
	if (menber->circleVisible) {
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
		if (menber->ThrDState->active())
			painter.drawEllipse(menber->centerPoint, menber->ThrDRadius, menber->ThrDRadius);
		else if (menber->TwoDState->active())
			painter.drawEllipse(menber->centerPoint, menber->TwoDRadius, menber->TwoDRadius);
		update();
	}
}

void ReceptiveFieldView::mouseMoveEvent(QMouseEvent* event)
{
	if (menber->circleVisible) {
		menber->centerPoint = event->pos();
		update();
		transMouseEvents(event);
	}
}

void ReceptiveFieldView::mousePressEvent(QMouseEvent* event)
{
	transMouseEvents(event);
}

void ReceptiveFieldView::mouseReleaseEvent(QMouseEvent* event)
{
	transMouseEvents(event);
}

void ReceptiveFieldView::wheelEvent(QWheelEvent* event)
{
	if (menber->circleVisible) {
		transWheelEvents(event);
		if (event->modifiers() == Qt::NoModifier) {
			int iDelta = event->delta();
			double delta = 1.0;
			if (iDelta < 0)
				delta = 0.80;
			else
				delta = 1.25;
			double dZoomRatio = delta;
			if (menber->ThrDState->active())
			{
				if ((menber->ThrDRadius * dZoomRatio) <= 1)
				{
					menber->MaxState = false;
					menber->ThrDRadius = menber->ThrDRadius;
					menber->ReduceTimes++;
				}
				else {
					if (menber->ReduceTimes > 0) {
						menber->ReduceTimes--;
					}
					else {
						if (!menber->MaxState) {
							menber->ThrDRadius *= dZoomRatio;
						}
						else if (dZoomRatio <= 1) {
							menber->MaxState = false;
							menber->ThrDRadius *= dZoomRatio;
						}
					}
				}
			}
			else if (menber->TwoDState->active())
			{
				if ((menber->TwoDRadius * dZoomRatio) <= 1)
				{
					menber->MaxState = false;
					menber->TwoDRadius = menber->ThrDRadius;
				}
				else {
					if (!menber->MaxState) {
						menber->TwoDRadius *= dZoomRatio;
					}
					else if (dZoomRatio <= 1) {
						menber->MaxState = false;
						menber->TwoDRadius *= dZoomRatio;
					}

				}
			}
		}
		update();
	}
}

void ReceptiveFieldView::enterEvent(QEvent* event)
{
	menber->circleVisible = true; // 鼠标进入Widget时，圆圈出现
	update();
}

void ReceptiveFieldView::leaveEvent(QEvent* event)
{
	menber->circleVisible = false; // 鼠标离开Widget时，圆圈消失
	update();
}

void ReceptiveFieldView::transMouseEvents(QMouseEvent* event)
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

void ReceptiveFieldView::transWheelEvents(QWheelEvent* event)
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

void ReceptiveFieldView::bindViewModel(std::shared_ptr<ViewModel> vm)
{
	viewModel = vm;
	if (viewModel.lock())
	{
		connect(viewModel.lock().get(), &ViewModel::notified, this, &ReceptiveFieldView::refresh);
	}
}

void ReceptiveFieldView::outOfBounds()
{
	menber->ThrDRadius *= 0.80;
	update();
}

