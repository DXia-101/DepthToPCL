#include "ParameterDesignWidget.h"
#include "teBaseDevelopmentKit.h"
#include "teTraining.h"
#include "teAiExTypes.h"

#include "DataTransmission.h"

ParameterDesignWidget::ParameterDesignWidget(AiModelInterface* workaimodel,QWidget *parent)
	: workAiModel(workaimodel),QWidget(parent)
	, ui(new Ui::ParameterDesignWidgetClass())
{
	ui->setupUi(this);
	connect(workAiModel, &AiModelInterface::StartInitTrainConfigSignal, this, &ParameterDesignWidget::StartInitTrainConfigSlots, Qt::DirectConnection);

	trainChart = new TrainingStatisticsChart(this);
	trainChart->hide();
	DataTransmission::GetInstance()->connect(DataTransmission::GetInstance(), &DataTransmission::DataChanged, trainChart, &TrainingStatisticsChart::ReceiveData);
}

ParameterDesignWidget::~ParameterDesignWidget()
{
	delete ui;
}

void ParameterDesignWidget::on_NetworkListCBox_currentTextChanged(const QString& arg1)
{
	workAiModel->config.netName = arg1.toStdString();
}

void ParameterDesignWidget::on_ChannelsCBox_currentTextChanged(const QString& arg1)
{
	workAiModel->config.sampleDesc[0].channel = arg1.toInt();
}

void ParameterDesignWidget::on_dtypeCBox_currentIndexChanged(int index)
{
	te::BaseType selectedTool = static_cast<te::BaseType>(index);
	workAiModel->config.sampleDesc[0].dtype = selectedTool;
}

void ParameterDesignWidget::on_eToolTypeCBox_currentIndexChanged(int index)
{
	ToolType selectedTool = static_cast<ToolType>(index);
	workAiModel->config.eToolType = selectedTool;
	workAiModel->netNames = Training::getNetworkList(selectedTool);

	QStringList list;
	for (std::string str : workAiModel->netNames) {
		list.push_back(QString::fromStdString(str));
	}
	ui->NetworkListCBox->addItems(list);
}

void ParameterDesignWidget::on_eTrainModeCBox_currentIndexChanged(int index)
{
	te::TrainMode selectedTool = static_cast<te::TrainMode>(index);
	workAiModel->config.eTrainMode = selectedTool;
}

void ParameterDesignWidget::on_locateTypeBox_currentIndexChanged(int index)
{
	te::LocateType selectedTool = static_cast<te::LocateType>(index);
	workAiModel->config.locateType = selectedTool;
}

void ParameterDesignWidget::on_receptiveFieldEdit_textEdited(const QString& arg1)
{
	workAiModel->config.receptiveField_A = arg1.toInt();
}

void ParameterDesignWidget::on_heapIDEdit_textEdited(const QString& arg1)
{
	workAiModel->config.sampleDesc[0].heapId = arg1.toInt();
}

void ParameterDesignWidget::on_trainIterCntEdit_textEdited(const QString& arg1)
{
	workAiModel->config.trainIterCnt = arg1.toInt();
}

void ParameterDesignWidget::on_BatchSizeEdit_textEdited(const QString& arg1)
{
	workAiModel->config.batchSize = arg1.toInt();
}

void ParameterDesignWidget::on_PatchWidthEdit_textEdited(const QString& arg1)
{
	if (arg1 == "Default") {
		workAiModel->config.patchWidth = 0;
		return;
	}
	workAiModel->config.patchWidth = arg1.toInt();
}

void ParameterDesignWidget::on_DeviceIDEdit_textEdited(const QString& arg1)
{
	workAiModel->DeviceID = arg1.toInt();
}

void ParameterDesignWidget::on_PatchHeightEdit_textEdited(const QString& arg1)
{
	if (arg1 == "Default") {
		workAiModel->config.patchHeight = 0;
		return;
	}
	workAiModel->config.patchHeight = arg1.toInt();
}

void ParameterDesignWidget::on_saveFrequencyEdit_textEdited(const QString& arg1)
{
	workAiModel->config.saveFrequency = arg1.toInt();
}

void ParameterDesignWidget::on_locateSideEdit_textEdited(const QString& arg1)
{
	workAiModel->config.locateSide = arg1.toInt();
}

void ParameterDesignWidget::on_TrainingCurveCBox_stateChanged(int arg1)
{
	if (ui->TrainingCurveCBox->isChecked()) {
		trainChart->show();
	}
	else {
		trainChart->hide();
	}
	
}

void ParameterDesignWidget::on_AutomaticCBox_stateChanged(int arg1)
{

}

void ParameterDesignWidget::StartInitTrainConfigSlots()
{
	int batchsize;
	if (ui->BatchSizeEdit->text().isEmpty()) {
		batchsize = 6;
	}
	else {
		batchsize = ui->BatchSizeEdit->text().toInt();
	}

	int patchWidth;
	if (ui->PatchWidthEdit->text() == "Default") {
		patchWidth = 0;
	}
	else {
		patchWidth = ui->PatchWidthEdit->text().toInt();
	}

	int patchHeight;
	if (ui->PatchHeightEdit->text() == "Default") {
		patchHeight = 0;
	}
	else {
		patchHeight = ui->PatchHeightEdit->text().toInt();
	}

	int receptiveField_A;
	if (ui->receptiveFieldEdit->text().isEmpty()) {
		receptiveField_A = 64;
	}
	else {
		receptiveField_A = ui->receptiveFieldEdit->text().toInt();
	}

	int trainIterCnt;
	if (ui->trainIterCntEdit->text().isEmpty()) {
		trainIterCnt = 128;
	}
	else {
		trainIterCnt = ui->trainIterCntEdit->text().toInt();
	}

	int saveFrequency;
	if (ui->saveFrequencyEdit->text().isEmpty()) {
		saveFrequency = 3;
	}
	else {
		saveFrequency = ui->saveFrequencyEdit->text().toInt();
	}

	te::ToolType eToolType;
	eToolType = static_cast<te::ToolType>(ui->eToolTypeCBox->currentIndex());


	te::TrainMode eTrainMode;
	eTrainMode = static_cast<te::TrainMode>(ui->eTrainModeCBox->currentIndex());

	te::LocateType locateType;
	locateType = static_cast<te::LocateType>(ui->locateTypeBox->currentIndex());

	int locateSide;
	if (ui->locateSideEdit->text().isEmpty()) {
		locateSide = 0;
	}
	else {
		locateSide = ui->locateSideEdit->text().toInt();
	}

	std::string netname;
	if (ui->NetworkListCBox->count() == 0) {
		workAiModel->netNames = Training::getNetworkList(eToolType);
		netname = workAiModel->netNames[1];
	}
	else {
		netname = ui->NetworkListCBox->currentText().toStdString();
	}

	te::BaseType dtype;
	dtype = static_cast<te::BaseType>(ui->dtypeCBox->currentIndex());
	

	int channel = ui->ChannelsCBox->currentText().toInt();

	int heapId;
	if (ui->heapIDEdit->text().isEmpty()) {
		heapId = 0;
	}
	else {
		heapId = ui->heapIDEdit->text().toInt();
	}

	if (ui->DeviceIDEdit->text().isEmpty()) {
		workAiModel->DeviceID = 0;
	}
	else {
		workAiModel->DeviceID = ui->DeviceIDEdit->text().toInt();
	}
	
	workAiModel->InitTrainConfig(
		batchsize, patchWidth, patchHeight, receptiveField_A, trainIterCnt,
		saveFrequency, eToolType, eTrainMode, locateType, locateSide,
		netname, dtype, channel, heapId
	);
}
