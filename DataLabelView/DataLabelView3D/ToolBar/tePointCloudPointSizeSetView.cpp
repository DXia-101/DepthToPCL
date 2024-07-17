#include "tePointCloudPointSizeSetView.h"

using namespace te;

PointCloudPointSizeSetView::PointCloudPointSizeSetView(QWidget* parent)
	: QDialog(parent)
	, ui(new Ui::PointCloudPointSizeSetViewClass())
{
	ui->setupUi(this);
	ui->PointSize_SpinBox->setMinimum(0);
	ui->PointSize_SpinBox->setMaximum(10);
	ui->PointSize_SpinBox->setSingleStep(1);
	ui->PointSize_SpinBox->setValue(5);

	ui->PointSizeHSlider->setMinimum(0);
	ui->PointSizeHSlider->setMaximum(10);
	ui->PointSizeHSlider->setSingleStep(1);
	ui->PointSizeHSlider->setValue(5);
}

PointCloudPointSizeSetView::~PointCloudPointSizeSetView()
{
	delete ui;
}

int te::PointCloudPointSizeSetView::getSize()
{
	return pointCloudSize;
}

void te::PointCloudPointSizeSetView::on_buttonBox_accepted()
{
	pointCloudSize = QString("%1").arg(ui->PointSizeHSlider->value()).toInt();
	this->close();
}

void te::PointCloudPointSizeSetView::on_PointSize_SpinBox_valueChanged(int value)
{
	ui->PointSizeHSlider->setValue(value);
}

void te::PointCloudPointSizeSetView::on_PointSizeHSlider_valueChanged(int value)
{
	ui->PointSize_SpinBox->setValue(value);
}
