#include "teTrainParameter.h"
#include "teRapidjsonObjectTree.h"

teTrainParameter::teTrainParameter(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::teTrainParameterClass())
{
	ui->setupUi(this);

	ui->treeView->setWriter(std::make_shared<te::TrainParamWriter>());

	te::TrainParam param;
	te::deserializeJsonFromIFStream("./paraconfig.ini", &param);
	
	ui->treeView->writeObject_t(param);

	ui->treeView->setExpandedRecursive(true);
}

teTrainParameter::~teTrainParameter()
{
	delete ui;
}

void teTrainParameter::SaveteTrainParameter()
{
	te::TrainParam param;
	ui->treeView->readObject_t(&param);

	te::serializeJsonToOFStream("./paraconfig.ini", param);
}

void teTrainParameter::on_SaveButton_clicked()
{
	SaveteTrainParameter();
}

void teTrainParameter::getTrainParam(te::TrainParam* train)
{
	ui->treeView->readObject_t(train);
}
