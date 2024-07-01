#pragma once

#include <QObject>


class QVBoxLayout;
class QThread;
class teAiModel;
class teAlgorithmInterface;
class teTrainParameterDisplay;
class teTestParameterDisplay;

class teAlgorithmController  : public QObject
{
	Q_OBJECT

public:
	teAlgorithmController(QObject *parent = nullptr);
	~teAlgorithmController();

public:
	void displayUIInWidget(QVBoxLayout* layout);
	void PrepareTrain();
	void PrepareTest();
	void setteAiModel(teAiModel*);
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
	void sig_StartTrain();
	void sig_StartTest();
private:
	teTrainParameterDisplay* m_teTrainPara;
	teTestParameterDisplay* m_teTestPara;
	teAlgorithmInterface* worker;
	QThread* thread;
};
