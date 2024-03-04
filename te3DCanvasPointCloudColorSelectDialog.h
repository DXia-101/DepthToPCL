#pragma once

#include <QWidget>

class te3DCanvasPointCloudColorSelectDialog  : public QWidget
{
	Q_OBJECT

public:
	te3DCanvasPointCloudColorSelectDialog();
	~te3DCanvasPointCloudColorSelectDialog();

	void setColor(const QColor& col);
	QColor getColor();

private:
	QColor color;
};
