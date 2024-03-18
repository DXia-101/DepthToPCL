#pragma once

#include <QObject>
#include "AiModelInterface.h"
#include "teTrainParameter.h"
#include "teTestParameter.h"

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
	void sig_SaveTrianParameter();
	void sig_PrepareTrain();
	void sig_PrepareTest();
	void sig_isShowTSChart(int arg);
	void sig_TSChartClose();
	void sig_receptiveFieldChange(int factor);
	void sig_StopTrain();

private:
	teTrainParameter* m_teTrainPara;
	teTestParameter* m_teTestPara;
	AiModelInterface* m_AiModel;
};
