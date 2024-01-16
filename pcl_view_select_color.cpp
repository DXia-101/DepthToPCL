#include "pcl_view_select_color.h"
#include <QColorDialog>

pcl_view_select_color::pcl_view_select_color()
{
	QColor col = QColorDialog::getColor(Qt::white, this);
	if (col.isValid())
		color = col;
	else {
		col.setRgb(143, 153, 159, 255);
		color = col;
	}
}

pcl_view_select_color::~pcl_view_select_color()
{}

void pcl_view_select_color::setColor(const QColor & col)
{
	if (col.isValid()) {
		color = col;
	}
}

QColor pcl_view_select_color::getColor()
{
	return color;
}
