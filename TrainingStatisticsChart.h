#pragma once

#include <QMainWindow>
#include "ui_TrainingStatisticsChart.h"
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE
namespace Ui { class TrainingStatisticsChartClass; };
QT_END_NAMESPACE

class TrainingStatisticsChart : public QMainWindow
{
	Q_OBJECT

public:
	TrainingStatisticsChart(QWidget *parent = nullptr);
	~TrainingStatisticsChart();

public slots:
	void ReceiveData(int iteration, float fAvgLoss, float fPosAcc);

protected slots:
	void on_iteration_checkBox_stateChanged(int arg);
	void on_fAvgLoss_checkBox_stateChanged(int arg);
	void on_fPosAcc_checkBox_stateChanged(int arg);

protected:
	void QPlot_init(QCustomPlot* customPlot);
	/// <summary>
	/// ÇúÏß¸üÐÂ»æÍ¼
	/// </summary>
	/// <param name="customPlot"></param>
	/// <param name="num"></param>
	void Show_Plot(QCustomPlot* customPlot, int iterationNum, float fAvgLossNum, float fPosAccNum);

private:
	Ui::TrainingStatisticsChartClass *ui;

	QCustomPlot* customPlot;
	QStatusBar* customBar;
	QCPGraph* iteration;
	QCPGraph* fAvgLoss;
	QCPGraph* fPosAcc;
};
