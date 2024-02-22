#include "TestParameterSetting.h"
#include "tePrediction.h"
#include "teAiExTypes.h"

TestParameterSetting::TestParameterSetting(AiModelInterface* workaimodel, QWidget *parent)
	: workAiModel(workaimodel), QWidget(parent)
	, ui(new Ui::TestParameterSettingClass())
{
	ui->setupUi(this);

	connect(workAiModel, &AiModelInterface::StartInitTestConfigSignal, this, &TestParameterSetting::StartInitTestConfigSlots, Qt::DirectConnection);
}

TestParameterSetting::~TestParameterSetting()
{
	delete ui;
}

void TestParameterSetting::on_MaxBatchSizeEdit_textEdited(const QString& arg1)
{
	workAiModel->infer_.setMaxBatchSize(arg1.toInt());
}

void TestParameterSetting::on_BatchSizeEdit_textEdited(const QString& arg1)
{
	workAiModel->infer_.setBatchSize(arg1.toInt());
}

void TestParameterSetting::on_maxContourCountEdit_textEdited(const QString& arg1)
{
	workAiModel->contrDesc.maxContourCount = arg1.toInt();
}

void TestParameterSetting::on_maxContourPointCountEdit_textEdited(const QString& arg1)
{
	workAiModel->contrDesc.maxContourPointCount = arg1.toInt();
}

void TestParameterSetting::on_maxInnerContourCountEdit_textEdited(const QString& arg1)
{
	workAiModel->contrDesc.maxInnerContourCount = arg1.toInt();
}

void TestParameterSetting::on_DeviceIDEdit_textEdited(const QString& arg1)
{
	workAiModel->devInfo.deviceID = arg1.toInt();
}

void TestParameterSetting::on_DeviceTypeComboBox_currentIndexChanged(int index)
{
	te::DeviceType devicetype = static_cast<te::DeviceType>(index);
	workAiModel->devInfo.deviceType = devicetype;
}

void TestParameterSetting::on_ComputePrecisionComboBox_currentIndexChanged(int index)
{
	te::ComputePrecision precision = static_cast<te::ComputePrecision>(index);
	workAiModel->infer_.setPrecisionType(precision);
}

void TestParameterSetting::StartInitTestConfigSlots()
{
	int MaxBatchSize;
	if (ui->MaxBatchSizeEdit->text().isEmpty()) {
		MaxBatchSize = 1;
	}
	else {
		MaxBatchSize = ui->MaxBatchSizeEdit->text().toInt();
	}

	int BatchSize;
	if (ui->BatchSizeEdit->text().isEmpty()) {
		BatchSize = 1;
	}
	else {
		BatchSize = ui->BatchSizeEdit->text().toInt();
	}

	int maxContourCount;
	if (ui->maxContourCountEdit->text().isEmpty()) {
		maxContourCount = 12;
	}
	else {
		maxContourCount = ui->maxContourCountEdit->text().toInt();
	}

	int maxContourPointCount;
	if (ui->maxContourPointCountEdit->text().isEmpty()) {
		maxContourPointCount = 5;
	}
	else {
		maxContourPointCount = ui->maxContourPointCountEdit->text().toInt();
	}

	int maxInnerContourCount;
	if (ui->maxInnerContourCountEdit->text().isEmpty()) {
		maxInnerContourCount = 256;
	}
	else {
		maxInnerContourCount = ui->maxInnerContourCountEdit->text().toInt();
	}

	int deviceID;
	if (ui->DeviceIDEdit->text().isEmpty()) {
		deviceID = 0;
	}
	else {
		deviceID = ui->DeviceIDEdit->text().toInt();
	}

	te::DeviceType devicetype;
	devicetype = static_cast<te::DeviceType>(ui->DeviceTypeComboBox->currentIndex());

	te::ComputePrecision precision;
	precision = static_cast<te::ComputePrecision>(ui->ComputePrecisionComboBox->currentIndex());

	workAiModel->InitTestConfig(
		MaxBatchSize, BatchSize,
		maxContourCount, maxContourPointCount, maxInnerContourCount,
		deviceID, devicetype, precision
	);
}
