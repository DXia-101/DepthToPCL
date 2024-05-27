#pragma once

#include <QWidget>
#include <QCloseEvent>
#include <QMouseEvent>
#include "ui_teTrainStatisticsChart.h"


QT_BEGIN_NAMESPACE
namespace Ui { class teTrainStatisticsChartClass; };
QT_END_NAMESPACE

class QStatusBar;
class QCustomPlot;

class teTrainStatisticsChart : public QWidget
{
	Q_OBJECT

public:
	teTrainStatisticsChart(QWidget *parent = nullptr);
	~teTrainStatisticsChart();

private:
	Ui::teTrainStatisticsChartClass *ui;
	
public slots:
	void ReceiveData(int iteration, float fAvgLoss, float fPosAcc);

protected slots:
	void on_iteration_checkBox_stateChanged(int arg);
	void on_fAvgLoss_checkBox_stateChanged(int arg);
	void on_fPosAcc_checkBox_stateChanged(int arg);

public:
	void isShow(int arg);

protected:
	void QPlot_init(QCustomPlot* customPlot);

	void Show_Plot(QCustomPlot* customPlot, int iterationNum, float fAvgLossNum, float fPosAccNum);
	void closeEvent(QCloseEvent* event) override;
	void handlePositionToolTip(QMouseEvent* event);
	bool eventFilter(QObject* watched, QEvent* event);
signals:
	void sig_closeteTrainStatisticsChart();

private:
	QCustomPlot* customPlot;
	QStatusBar* customBar;
	QCPGraph* iteration;
	QCPGraph* fAvgLoss;
	QCPGraph* fPosAcc;
};
