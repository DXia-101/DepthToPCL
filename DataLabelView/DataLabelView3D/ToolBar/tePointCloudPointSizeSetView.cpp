#include "tePointCloudPointSizeSetView.h"

using namespace te;

PointCloudPointSizeSetView::PointCloudPointSizeSetView(QWidget* parent)
	: QDialog(parent)
	, ui(new Ui::PointCloudPointSizeSetViewClass())
{
	ui->setupUi(this);
	ui->PointSize_SpinBox->setMinimum(1);
	ui->PointSize_SpinBox->setMaximum(10);
	ui->PointSize_SpinBox->setSingleStep(1);
	ui->PointSize_SpinBox->setValue(1);

	ui->PointSizeHSlider->setMinimum(1);
	ui->PointSizeHSlider->setMaximum(10);
	ui->PointSizeHSlider->setSingleStep(1);
	ui->PointSizeHSlider->setValue(1);

	connect(ui->PointSize_SpinBox, qOverload<int>(&QSpinBox::valueChanged), this, &PointCloudPointSizeSetView::spinBoxValueChanged);
	connect(ui->PointSizeHSlider, &QSlider::valueChanged, this, &PointCloudPointSizeSetView::sliderValueChanged);
	connect(ui->buttonBox, &QDialogButtonBox::accepted, this, &PointCloudPointSizeSetView::ValueAccepted);
	connect(ui->buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
}

PointCloudPointSizeSetView::~PointCloudPointSizeSetView()
{
	delete ui;
}

int te::PointCloudPointSizeSetView::getSize()
{
	return pointCloudSize;
}

void te::PointCloudPointSizeSetView::ValueAccepted()
{
	pointCloudSize = QString("%1").arg(ui->PointSize_SpinBox->value()).toInt();
	this->close();
}

void te::PointCloudPointSizeSetView::spinBoxValueChanged(int value)
{
	ui->PointSizeHSlider->blockSignals(true);
	ui->PointSizeHSlider->setValue(value);
	ui->PointSizeHSlider->blockSignals(false);
}

void te::PointCloudPointSizeSetView::sliderValueChanged(int value)
{
	ui->PointSize_SpinBox->blockSignals(true);
	ui->PointSize_SpinBox->setValue(value);
	ui->PointSize_SpinBox->blockSignals(false);
}
