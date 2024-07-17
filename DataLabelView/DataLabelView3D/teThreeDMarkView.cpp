#include "teThreeDMarkView.h"
using namespace te;
ThreeDMarkView::ThreeDMarkView(QWidget* parent)
	: QWidget(parent)
{}

ThreeDMarkView::~ThreeDMarkView()
{}

void ThreeDMarkView::initWidget()
{
}

void ThreeDMarkView::setDrawState()
{
}

void ThreeDMarkView::enterEvent(QEvent* event)
{
}

void ThreeDMarkView::leaveEvent(QEvent* event)
{
}

void ThreeDMarkView::paintEvent(QPaintEvent* e)
{
}

void ThreeDMarkView::mousePressEvent(QMouseEvent* e)
{
}

void ThreeDMarkView::mouseMoveEvent(QMouseEvent* e)
{
}

void ThreeDMarkView::mouseReleaseEvent(QMouseEvent* e)
{
}

void ThreeDMarkView::refresh(ViewModel::updateMode mode)
{
	if (mode == ViewModel::ThreeDMark)
	{
		setDrawState();
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
