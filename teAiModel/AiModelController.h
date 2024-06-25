#pragma once

#include <QObject>
#include "AiModelInterface.h"
#include "teTrainParameterDisplay.h"
#include "teTestParameterDisplay.h"

class QVBoxLayout;

class AiModelController  : public QObject
{
	Q_OBJECT

public:
	AiModelController(QObject *parent = nullptr);
	~AiModelController();

public:
	void displayUIInWidget(QVBoxLayout* layout);
	void PrepareTrain();
	void PrepareTest();
	int getReceptiveField();

signals:
	void sig_SaveParameter();
	void sig_PrepareTrain();
	void sig_PrepareTest();
	void sig_isShowTSChart(int arg);
	void sig_TSChartClose();
	void sig_receptiveFieldChange(int factor);
	void sig_StopTrain();
	void sig_TestCompleted();
private:
	teTrainParameterDisplay* m_teTrainPara;
	teTestParameterDisplay* m_teTestPara;
	AiModelInterface* m_AiModel;
};
