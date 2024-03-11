#pragma once

#include <QWidget>
#include <QComboBox>
#include "ui_teTrainParameter.h"

#include "TrainParam.h"

QT_BEGIN_NAMESPACE
namespace Ui { class teTrainParameterClass; };
QT_END_NAMESPACE

class teTrainParameter : public QWidget
{
	Q_OBJECT

public:
	teTrainParameter(QWidget *parent = nullptr);
	~teTrainParameter();

public slots:
	void SaveteTrainParameter();
	void on_SaveButton_clicked(); 
	void on_TrainingCurveCBox_stateChanged(int arg);

signals:
	void sig_ShowTrainStatisticsChart(int arg);

public:
	void getTrainParam(te::TrainParam* train);

private:
	Ui::teTrainParameterClass *ui;
};