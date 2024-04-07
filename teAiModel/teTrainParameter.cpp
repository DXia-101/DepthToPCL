#include "teTrainParameter.h"
#include "teRapidjsonObjectTree.h"

#include<QDebug>

teTrainParameter::teTrainParameter(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::teTrainParameterClass())
{
	ui->setupUi(this);

	ui->treeView->setWriter(std::make_shared<te::TrainParamWriter>());

	connect(ui->treeView->getWriter(), &te::ObjectTreeWidgetWriter::sig_ItemChange, this, [this](te::ObjectTreeWidgetItem* pItem)
	{
		if (pItem->key() == "receptiveField") {

			int value = 0;
			ui->treeView->readObject_t(&value, pItem);

			emit sig_receptiveFieldChange(value);
		}
	});

	te::TrainParam param;
	te::deserializeJsonFromIFStream("./TrainParaconfig.ini", &param);
	
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

	te::serializeJsonToOFStream("./TrainParaconfig.ini", param);
	ui->treeView->checkItemChange();
}

void teTrainParameter::on_TrainingCurveCBox_stateChanged(int arg)
{
	emit sig_ShowTrainStatisticsChart(arg);
}

void teTrainParameter::teTrainStatisticsChartClose()
{
	ui->TrainingCurveCBox->setChecked(false);
}

void teTrainParameter::getTrainParam(te::TrainParam* train)
{
	ui->treeView->readObject_t(train);
}

