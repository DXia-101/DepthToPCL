#pragma once

#include <QWidget>
#include <QComboBox>
#include "ui_teTrainParameterDisplay.h"

#include "TrainParamRegister.h"

QT_BEGIN_NAMESPACE
namespace Ui { class teTrainParameterDisplayClass; };
QT_END_NAMESPACE

class teTrainParameterDisplay : public QWidget
{
	Q_OBJECT

public:
	teTrainParameterDisplay(QWidget *parent = nullptr);
	~teTrainParameterDisplay();

public slots:
	void SaveteTrainParameter();
	void on_TrainingCurveCBox_stateChanged(int arg);
	void teTrainStatisticsChartClose();

signals:
	void sig_ShowTrainStatisticsChart(int arg);
	void sig_receptiveFieldChange(int factor);

public:
	void getTrainParam(te::TrainParamRegister* train);

private:
	Ui::teTrainParameterDisplayClass *ui;
};