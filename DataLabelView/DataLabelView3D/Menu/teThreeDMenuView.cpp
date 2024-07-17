#include "teThreeDMenuView.h"
using namespace te;
ThreeDMenuView::ThreeDMenuView(QWidget* parent)
	: QWidget(parent)
	, ui(new Ui::ThreeDMenuViewClass())
{
	ui->setupUi(this);
}

ThreeDMenuView::~ThreeDMenuView()
{
	delete ui;
}

void ThreeDMenuView::bindViewModel(std::shared_ptr<ViewModel> vm)
{
	viewModel = vm;
	if (viewModel.lock())
	{
		connect(viewModel.lock().get(), &ViewModel::notified, this, &ThreeDMenuView::refresh);
	}
}

void te::ThreeDMenuView::setHeightCoefficientSpinBoxValue()
{
	int factor = viewModel.lock()->getPointCloudHeight();
	ui->HeightCoefficientSpinBox->setValue(factor);
}

void ThreeDMenuView::refresh(ViewModel::updateMode mode)
{
	if (mode == ViewModel::ThreeDViewMenu)
	{
		setHeightCoefficientSpinBoxValue();
	}
}

void ThreeDMenuView::on_reductionBtn_clicked()
{
}

void ThreeDMenuView::on_ViewYBtn_clicked()
{
	if (viewModel.lock()->getCoordinateAxisDire() != PositiveYaxis || viewModel.lock()->getCoordinateAxisDire() != NegativeYaxis)
	{
		viewModel.lock()->setCoordinateAxisDire(PositiveYaxis);
	}
	else
	{
		if (PositiveYaxis == viewModel.lock()->getCoordinateAxisDire())
			viewModel.lock()->setCoordinateAxisDire(NegativeYaxis);
		else if (NegativeYaxis == viewModel.lock()->getCoordinateAxisDire())
			viewModel.lock()->setCoordinateAxisDire(PositiveYaxis);
	}

	viewModel.lock()->notified(ViewModel::SetCameraPosition);
}

void ThreeDMenuView::on_ViewXBtn_clicked()
{
	if (viewModel.lock()->getCoordinateAxisDire() != PositiveXaxis || viewModel.lock()->getCoordinateAxisDire() != NegativeXaxis)
	{
		viewModel.lock()->setCoordinateAxisDire(PositiveXaxis);
	}
	else
	{
		if (PositiveXaxis == viewModel.lock()->getCoordinateAxisDire())
			viewModel.lock()->setCoordinateAxisDire(NegativeXaxis);
		else if (NegativeXaxis == viewModel.lock()->getCoordinateAxisDire())
			viewModel.lock()->setCoordinateAxisDire(PositiveXaxis);
	}

	viewModel.lock()->notified(ViewModel::SetCameraPosition);
}

void ThreeDMenuView::on_ViewZBtn_clicked()
{
	if (viewModel.lock()->getCoordinateAxisDire() != PositiveZaxis || viewModel.lock()->getCoordinateAxisDire() != NegativeZaxis)
	{
		viewModel.lock()->setCoordinateAxisDire(PositiveZaxis);
	}
	else
	{
		if (PositiveZaxis == viewModel.lock()->getCoordinateAxisDire())
			viewModel.lock()->setCoordinateAxisDire(NegativeZaxis);
		else if (NegativeZaxis == viewModel.lock()->getCoordinateAxisDire())
			viewModel.lock()->setCoordinateAxisDire(PositiveZaxis);
	}
	viewModel.lock()->notified(ViewModel::SetCameraPosition);
}

void ThreeDMenuView::on_startMarkBtn_clicked()
{
	if (ViewModel::ThreeDMarkView == viewModel.lock()->getCurrentWidgetType())
		viewModel.lock()->setCurrentWidgetType(ViewModel::ReceptiveFieldView);
	else if (ViewModel::ReceptiveFieldView == viewModel.lock()->getCurrentWidgetType())
		viewModel.lock()->setCurrentWidgetType(ViewModel::ThreeDMarkView);
	viewModel.lock()->notified(ViewModel::StartMark);
}

void ThreeDMenuView::on_showGTCheckBox_stateChanged(int arg)
{
	viewModel.lock()->setThreeDisShowMarkers(arg);
	viewModel.lock()->notified(ViewModel::MarkerPointCloudVisible);
}

void ThreeDMenuView::on_showRSTcheckBox_stateChanged(int arg)
{
	viewModel.lock()->setThreeDisShowMarkers(arg);
	viewModel.lock()->notified(ViewModel::ResultPointCloudVisible);
}

void ThreeDMenuView::on_HighTransformBtn_clicked()
{
	viewModel.lock()->setPointCloudHeight(ui->HeightCoefficientSpinBox->value());
	viewModel.lock()->notified(ViewModel::PointCloudHeight);
}
