#pragma once

#include <QWidget>
#include "ui_ImageInfoDisplay.h"

QT_BEGIN_NAMESPACE
namespace Ui { class ImageInfoDisplayClass; };
QT_END_NAMESPACE

class ImageInfoDisplay : public QWidget
{
	Q_OBJECT

public:
	ImageInfoDisplay(QWidget *parent = nullptr);
	~ImageInfoDisplay();
	void setResolutionLabelText(QString resolution);
	void setImageNameEditText(QString resolution);
	QString GetImageNameEditText();
private:
	Ui::ImageInfoDisplayClass *ui;
};
