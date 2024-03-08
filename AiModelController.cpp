#include "AiModelController.h"
#include <QVBoxLayout>
#include "teDataStorage.h"

AiModelController::AiModelController(QObject *parent)
	: QObject(parent)
{
	m_teTrainPara = new teTrainParameter();
	m_AiModel = new AiModelInterface();
	connect(this, &AiModelController::sig_PrepareTrain, this, &AiModelController::PrepareTrain);
	connect(this, &AiModelController::sig_PrepareTest, this, &AiModelController::PrepareTest);
	connect(this, &AiModelController::sig_SaveTrianParameter, m_teTrainPara, &teTrainParameter::SaveteTrainParameter);
}

AiModelController::~AiModelController()
{

}

void AiModelController::displayUIInWidget(QVBoxLayout* layout)
{
	layout->addWidget(m_teTrainPara);
	m_teTrainPara->show();
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

}
