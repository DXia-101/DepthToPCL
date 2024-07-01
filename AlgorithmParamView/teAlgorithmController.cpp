#include "teAlgorithmController.h"
#include <QVBoxLayout>
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
	m_teAlgorithm = new teAlgorithmInterface();
	connect(this, &teAlgorithmController::sig_PrepareTrain, this, &teAlgorithmController::PrepareTrain);
	connect(this, &teAlgorithmController::sig_PrepareTest, this, &teAlgorithmController::PrepareTest);
	connect(this, &teAlgorithmController::sig_SaveParameter, m_teTrainPara, &teTrainParameterDisplay::SaveteTrainParameter);
	connect(this, &teAlgorithmController::sig_SaveParameter, m_teTestPara, &teTestParameterDisplay::SaveteTestParameter);
	connect(this, &teAlgorithmController::sig_StopTrain, m_teAlgorithm, &teAlgorithmInterface::StopTrain);
	connect(this, &teAlgorithmController::sig_TSChartClose, m_teTrainPara, &teTrainParameterDisplay::teTrainStatisticsChartClose);
	connect(m_teTrainPara, &teTrainParameterDisplay::sig_receptiveFieldChange , this, &teAlgorithmController::sig_receptiveFieldChange);
	connect(m_teAlgorithm, &teAlgorithmInterface::sig_TestingCompleted, this, &teAlgorithmController::sig_TestCompleted);
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
	m_teAlgorithm->TrainParameterSettings(fileName.c_str());
	m_teTrainPara->SaveteTrainParameter();
	te::TrainParamRegister* train = new te::TrainParamRegister();
	m_teTrainPara->getTrainParam(train);
	m_teAlgorithm->InitTrainConfig(train);
	m_teAlgorithm->start();
}

void teAlgorithmController::PrepareTest()
{
	std::string fileName = "2.te";
	m_teAlgorithm->TestParameterSettings(fileName.c_str());
	m_teTestPara->SaveteTestParameter();
	te::TestParamRegister* test = new te::TestParamRegister();
	m_teTestPara->getTestParam(test);
	m_teAlgorithm->InitTestConfig(test);
	m_teAlgorithm->start();
}

void teAlgorithmController::setteAiModel(teAiModel* model)
{
	m_teAlgorithm->setteAiModel(model);
}

int teAlgorithmController::getReceptiveField()
{
	te::TrainParamRegister* train = new te::TrainParamRegister();
	m_teTrainPara->getTrainParam(train);
	return train->receptiveField;
}
