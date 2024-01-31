#include "ImageInfoDisplay.h"

ImageInfoDisplay::ImageInfoDisplay(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::ImageInfoDisplayClass())
{
	ui->setupUi(this);
}

ImageInfoDisplay::~ImageInfoDisplay()
{
	delete ui;
}

void ImageInfoDisplay::setResolutionLabelText(QString resolution)
{
	ui->resolutionLabel->setText(resolution);
}

void ImageInfoDisplay::setImageNameEditText(QString resolution)
{
	ui->imageNameEdit->setText(resolution);
}

QString ImageInfoDisplay::GetImageNameEditText()
{
	return ui->imageNameEdit->text();
}
