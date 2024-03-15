#pragma once

#include <QWidget>
#include <QMouseEvent>

class teMouseCircle  : public QWidget
{
	Q_OBJECT

public:
	teMouseCircle(QWidget *parent = nullptr);
	~teMouseCircle();

protected:
	void paintEvent(QPaintEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;

private:
	QPoint centerPoint;
};
