#include "te3DCanvasPointCloudColorSelectDialog.h"
#include <QColorDialog>

te3DCanvasPointCloudColorSelectDialog::te3DCanvasPointCloudColorSelectDialog()
{
	QColor col = QColorDialog::getColor(Qt::white, this);
	if (col.isValid())
		color = col;
	else {
		col.setRgb(143, 153, 159, 255);
		color = col;
	}
}

te3DCanvasPointCloudColorSelectDialog::~te3DCanvasPointCloudColorSelectDialog()
{}

void te3DCanvasPointCloudColorSelectDialog::setColor(const QColor & col)
{
	if (col.isValid()) {
		color = col;
	}
}

QColor te3DCanvasPointCloudColorSelectDialog::getColor()
{
	return color;
}
