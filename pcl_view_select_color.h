#pragma once

#include <QWidget>

class pcl_view_select_color  : public QWidget
{
	Q_OBJECT

public:
	pcl_view_select_color();
	~pcl_view_select_color();

	void setColor(const QColor& col);
	QColor getColor();

private:
	QColor color;
};
