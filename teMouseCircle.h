#pragma once

#include <QWidget>

class QMouseEvent;
class QWheelEvent;
class QStateMachine;
class QState;

class teMouseCircle  : public QWidget
{
	Q_OBJECT

public:
	teMouseCircle(QWidget *parent = nullptr);
	~teMouseCircle();

	void InitStateMachine();
public:
	void restitution();

protected:
	void paintEvent(QPaintEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;
	void enterEvent(QEvent* event) override;
	void leaveEvent(QEvent* event) override;
	void transMouseEvents(QMouseEvent* event);
	void transWheelEvents(QWheelEvent* event);

public slots:
	void receptiveFieldChange(int factor);

signals:
	void sig_enterThrD();
	void sig_enterTwoD();

private:
	QPointF centerPoint;
	float ThrDradius;
	float TwoDradius;
	bool MaxState;
	bool circleVisible;
	int ReduceTimes;

	QStateMachine* m_pStateMachine;
	QState* TwoDState;
	QState* ThrDState;
};
