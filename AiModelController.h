#pragma once

#include <QObject>
#include "AiModelInterface.h"
#include "teTrainParameter.h"
#include "TrainParam.h"

class QVBoxLayout;

class AiModelController  : public QObject
{
	Q_OBJECT

public:
	AiModelController(QObject *parent = nullptr);
	~AiModelController();

signals:
	void sig_SaveTrianParameter();
	void sig_PrepareTrain();
	void sig_PrepareTest();

public:
	void displayUIInWidget(QVBoxLayout* layout);
	void PrepareTrain();
	void PrepareTest();

private:
	teTrainParameter* m_teTrainPara;
	AiModelInterface* m_AiModel;
	te::TrainParam* train;
};
