#include "teAlgorithmController.h"
#include <QVBoxLayout>
#include <QThread>
#include "TrainParamRegister.h"
#include "TestParamRegister.h"
#include "teAlgorithmInterface.h"
#include "teTrainParameterDisplay.h"
#include "teTestParameterDisplay.h"

teAlgorithmController::teAlgorithmController(QObject *parent)
	: QObject(parent)
{
	m_teTrainPara = new teTrainParameterDisplay();
	m_teTestPara = new teTestParameterDisplay();
	thread = new QThread();
	worker = new teAlgorithmInterface();
	worker->moveToThread(thread);
	connect(thread, &QThread::finished, worker, &QObject::deleteLater);
	connect(worker, &QObject::destroyed, thread, &QThread::quit);
	connect(thread, &QThread::finished, thread, &QObject::deleteLater);
	connect(this, &teAlgorithmController::sig_PrepareTrain, this, &teAlgorithmController::PrepareTrain);
	connect(this, &teAlgorithmController::sig_PrepareTest, this, &teAlgorithmController::PrepareTest);
	connect(this, &teAlgorithmController::sig_SaveParameter, m_teTrainPara, &teTrainParameterDisplay::SaveteTrainParameter);
	connect(this, &teAlgorithmController::sig_SaveParameter, m_teTestPara, &teTestParameterDisplay::SaveteTestParameter);
	connect(this, &teAlgorithmController::sig_StopTrain, worker, &teAlgorithmInterface::StopTrain);
	connect(this, &teAlgorithmController::sig_TSChartClose, m_teTrainPara, &teTrainParameterDisplay::teTrainStatisticsChartClose);
	connect(this, &teAlgorithmController::sig_StartTrain, worker, &teAlgorithmInterface::StartTrain);
	connect(this, &teAlgorithmController::sig_StartTest, worker, &teAlgorithmInterface::StartTest);
	connect(m_teTrainPara, &teTrainParameterDisplay::sig_receptiveFieldChange , this, &teAlgorithmController::sig_receptiveFieldChange);
	connect(worker, &teAlgorithmInterface::sig_TestingCompleted, this, &teAlgorithmController::sig_TestCompleted);
	
	thread->start();
}	

teAlgorithmController::~teAlgorithmController()
{

}

void teAlgorithmController::displayUIInWidget(QVBoxLayout* layout)
{
	layout->addWidget(m_teTrainPara);
	layout->addWidget(m_teTestPara);
	m_teTrainPara->show();
	m_teTestPara->show();
}

void teAlgorithmController::PrepareTrain()
{
	std::string fileName = "2.te";
	worker->TrainParameterSettings(fileName.c_str());
	m_teTrainPara->SaveteTrainParameter();
	te::TrainParamRegister* train = new te::TrainParamRegister();
	m_teTrainPara->getTrainParam(train);
	worker->InitTrainConfig(train);
	emit sig_StartTrain();
}

void teAlgorithmController::PrepareTest()
{
	std::string fileName = "2.te";
	worker->TestParameterSettings(fileName.c_str());
	m_teTestPara->SaveteTestParameter();
	te::TestParamRegister* test = new te::TestParamRegister();
	m_teTestPara->getTestParam(test);
	worker->InitTestConfig(test);
	emit sig_StartTest();
}

void teAlgorithmController::setteAiModel(teAiModel* model)
{
	worker->setteAiModel(model);
}

int teAlgorithmController::getReceptiveField()
{
	te::TrainParamRegister* train = new te::TrainParamRegister();
	m_teTrainPara->getTrainParam(train);
	return train->receptiveField;
}
