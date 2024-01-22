#include "PointCloud_PointSize_Set_Dialog.h"

PointCloud_PointSize_Set_Dialog::PointCloud_PointSize_Set_Dialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	ui.PointSize_SpinBox->setMinimum(0);
	ui.PointSize_SpinBox->setMaximum(10);
	ui.PointSize_SpinBox->setSingleStep(1);
	ui.PointSize_SpinBox->setValue(5);

	ui.PointSizeHSlider->setMinimum(0);
	ui.PointSizeHSlider->setMaximum(10);
	ui.PointSizeHSlider->setSingleStep(1);
	ui.PointSizeHSlider->setValue(5);

	//connect(ui.PointSize_SpinBox, SIGNAL(valueChanged(int)), ui.PointSizeHSlider, SIGNAL(setValue(int)));
	//connect(ui.PointSizeHSlider, SIGNAL(valueChanged(int)), ui.PointSize_SpinBox, SIGNAL(setValue(int)));
	//
}

PointCloud_PointSize_Set_Dialog::~PointCloud_PointSize_Set_Dialog()
{}

int PointCloud_PointSize_Set_Dialog::GetSize()
{
	return pointCloudSize;
}

void PointCloud_PointSize_Set_Dialog::on_PointSize_SpinBox_valueChanged(int value)
{
	ui.PointSizeHSlider->setValue(value);
}

void PointCloud_PointSize_Set_Dialog::on_PointSizeHSlider_valueChanged(int value)
{
	ui.PointSize_SpinBox->setValue(value);
}


void PointCloud_PointSize_Set_Dialog::on_buttonBox_accepted() {
	pointCloudSize = QString("%1").arg(ui.PointSizeHSlider->value()).toInt();
	this->close();
}