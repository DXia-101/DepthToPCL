#pragma once

#include <QWidget>
#include <QMouseEvent>
#include <QWheelEvent>

class teMouseCircle  : public QWidget
{
	Q_OBJECT

public:
	teMouseCircle(QWidget *parent = nullptr);
	~teMouseCircle();

protected:
	void paintEvent(QPaintEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;
	void transMouseEvents(QMouseEvent* event);
	void transWheelEvents(QWheelEvent* event);
private:
	QPoint centerPoint;
	int radius;
};
