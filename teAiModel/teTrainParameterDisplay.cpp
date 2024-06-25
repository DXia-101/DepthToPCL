#include "teTrainParameterDisplay.h"
#include "teRapidjsonObjectTree.h"

#include<QDebug>

teTrainParameterDisplay::teTrainParameterDisplay(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::teTrainParameterDisplayClass())
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

	te::TrainParamRegister param;
	te::deserializeJsonFromIFStream("./TrainParaconfig.ini", &param);
	
	ui->treeView->writeObject_t(param);

	ui->treeView->setExpandedRecursive(true);
}

teTrainParameterDisplay::~teTrainParameterDisplay()
{
	delete ui;
}

void teTrainParameterDisplay::SaveteTrainParameter()
{
	te::TrainParamRegister param;
	ui->treeView->readObject_t(&param);

	te::serializeJsonToOFStream("./TrainParaconfig.ini", param);
	ui->treeView->checkItemChange();
}

void teTrainParameterDisplay::on_TrainingCurveCBox_stateChanged(int arg)
{
	emit sig_ShowTrainStatisticsChart(arg);
}

void teTrainParameterDisplay::teTrainStatisticsChartClose()
{
	ui->TrainingCurveCBox->setChecked(false);
}

void teTrainParameterDisplay::getTrainParam(te::TrainParamRegister* train)
{
	ui->treeView->readObject_t(train);
}

