#include "AiModelController.h"
#include <QVBoxLayout>
#include "teDataStorage.h"
#include "TrainParam.h"
#include "TestParam.h"

AiModelController::AiModelController(QObject *parent)
	: QObject(parent)
{
	m_teTrainPara = new teTrainParameter();
	m_teTestPara = new teTestParameter();
	m_AiModel = new AiModelInterface();
	connect(this, &AiModelController::sig_PrepareTrain, this, &AiModelController::PrepareTrain);
	connect(this, &AiModelController::sig_PrepareTest, this, &AiModelController::PrepareTest);
	connect(this, &AiModelController::sig_SaveTrianParameter, m_teTrainPara, &teTrainParameter::SaveteTrainParameter);
	connect(m_teTrainPara, &teTrainParameter::sig_ShowTrainStatisticsChart, this, &AiModelController::sig_isShowTSChart);
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
	te::TrainParam* train = new te::TrainParam();
	m_teTrainPara->getTrainParam(train);
	m_AiModel->InitTrainConfig(train);
	m_AiModel->start();
}

void AiModelController::PrepareTest()
{
	std::string fileName = "2.te";
	m_AiModel->TestParameterSettings(fileName.c_str());
	te::TestParam* test = new te::TestParam();
	m_teTestPara->getTestParam(test);
	m_AiModel->InitTestConfig(test);
	m_AiModel->start();
}
