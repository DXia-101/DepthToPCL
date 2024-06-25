#include "AiModelController.h"
#include <QVBoxLayout>
#include "teDataStorage.h"
#include "TrainParamRegister.h"
#include "TestParamRegister.h"

AiModelController::AiModelController(QObject *parent)
	: QObject(parent)
{
	m_teTrainPara = new teTrainParameterDisplay();
	m_teTestPara = new teTestParameterDisplay();
	m_AiModel = new AiModelInterface();
	connect(this, &AiModelController::sig_PrepareTrain, this, &AiModelController::PrepareTrain);
	connect(this, &AiModelController::sig_PrepareTest, this, &AiModelController::PrepareTest);
	connect(this, &AiModelController::sig_SaveParameter, m_teTrainPara, &teTrainParameterDisplay::SaveteTrainParameter);
	connect(this, &AiModelController::sig_SaveParameter, m_teTestPara, &teTestParameterDisplay::SaveteTestParameter);
	connect(this, &AiModelController::sig_StopTrain, m_AiModel, &AiModelInterface::StopTrain);
	connect(this, &AiModelController::sig_TSChartClose, m_teTrainPara, &teTrainParameterDisplay::teTrainStatisticsChartClose);
	connect(m_teTrainPara, &teTrainParameterDisplay::sig_ShowTrainStatisticsChart, this, &AiModelController::sig_isShowTSChart);
	connect(m_teTrainPara, &teTrainParameterDisplay::sig_receptiveFieldChange , this, &AiModelController::sig_receptiveFieldChange);
	connect(m_AiModel, &AiModelInterface::sig_TestingCompleted, this, &AiModelController::sig_TestCompleted);
}	

AiModelController::~AiModelController()
{

}

void AiModelController::displayUIInWidget(QVBoxLayout* layout)
{
	layout->addWidget(m_teTrainPara);
	layout->addWidget(m_teTestPara);
	m_teTrainPara->show();
	m_teTestPara->show();
}

void AiModelController::PrepareTrain()
{
	std::string fileName = "2.te";
	m_AiModel->TrainParameterSettings(fileName.c_str());
	m_teTrainPara->SaveteTrainParameter();
	te::TrainParamRegister* train = new te::TrainParamRegister();
	m_teTrainPara->getTrainParam(train);
	m_AiModel->InitTrainConfig(train);
	m_AiModel->start();
}

void AiModelController::PrepareTest()
{
	std::string fileName = "2.te";
	m_AiModel->TestParameterSettings(fileName.c_str());
	m_teTestPara->SaveteTestParameter();
	te::TestParamRegister* test = new te::TestParamRegister();
	m_teTestPara->getTestParam(test);
	m_AiModel->InitTestConfig(test);
	m_AiModel->start();
}

int AiModelController::getReceptiveField()
{
	te::TrainParamRegister* train = new te::TrainParamRegister();
	m_teTrainPara->getTrainParam(train);
	return train->receptiveField;
}
