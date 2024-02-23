#include "TestParameterSetting.h"
#include "tePrediction.h"
#include "teAiExTypes.h"

#include <QSettings>
#include <QDir>

TestParameterSetting::TestParameterSetting(AiModelInterface* workaimodel, QWidget *parent)
	: workAiModel(workaimodel), QWidget(parent)
	, ui(new Ui::TestParameterSettingClass())
{
	ui->setupUi(this);

	configFilePath = QDir::currentPath() + "/config.ini";
	QSettings settings(configFilePath, QSettings::IniFormat);
	ui->MaxBatchSizeEdit->setText(settings.value("MaxBatchSizeEdit").toString());
	ui->BatchSizeEdit->setText(settings.value("BatchSizeEdit").toString());
	ui->maxContourCountEdit->setText(settings.value("maxContourCountEdit").toString());
	ui->maxContourPointCountEdit->setText(settings.value("maxContourPointCountEdit").toString());
	ui->maxInnerContourCountEdit->setText(settings.value("maxInnerContourCountEdit").toString());
	ui->DeviceIDEdit->setText(settings.value("DeviceIDEdit").toString());
	ui->DeviceTypeComboBox->setCurrentIndex(settings.value("DeviceTypeComboBox").toInt());
	ui->ComputePrecisionComboBox->setCurrentIndex(settings.value("ComputePrecisionComboBox").toInt());

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
	QSettings settings(configFilePath, QSettings::IniFormat);

	int MaxBatchSize;
	if (ui->MaxBatchSizeEdit->text().isEmpty()) {
		MaxBatchSize = 1;
	}
	else {
		MaxBatchSize = ui->MaxBatchSizeEdit->text().toInt();
	}
	settings.setValue("MaxBatchSizeEdit", ui->MaxBatchSizeEdit->text());

	int BatchSize;
	if (ui->BatchSizeEdit->text().isEmpty()) {
		BatchSize = 1;
	}
	else {
		BatchSize = ui->BatchSizeEdit->text().toInt();
	}
	settings.setValue("BatchSizeEdit", ui->BatchSizeEdit->text());

	int maxContourCount;
	if (ui->maxContourCountEdit->text().isEmpty()) {
		maxContourCount = 12;
	}
	else {
		maxContourCount = ui->maxContourCountEdit->text().toInt();
	}
	settings.setValue("maxContourCountEdit", ui->maxContourCountEdit->text());

	int maxContourPointCount;
	if (ui->maxContourPointCountEdit->text().isEmpty()) {
		maxContourPointCount = 5;
	}
	else {
		maxContourPointCount = ui->maxContourPointCountEdit->text().toInt();
	}
	settings.setValue("maxContourPointCountEdit", ui->maxContourPointCountEdit->text());

	int maxInnerContourCount;
	if (ui->maxInnerContourCountEdit->text().isEmpty()) {
		maxInnerContourCount = 256;
	}
	else {
		maxInnerContourCount = ui->maxInnerContourCountEdit->text().toInt();
	}
	settings.setValue("maxInnerContourCountEdit", ui->maxInnerContourCountEdit->text());

	int deviceID;
	if (ui->DeviceIDEdit->text().isEmpty()) {
		deviceID = 0;
	}
	else {
		deviceID = ui->DeviceIDEdit->text().toInt();
	}
	settings.setValue("DeviceIDEdit", ui->DeviceIDEdit->text());

	te::DeviceType devicetype;
	devicetype = static_cast<te::DeviceType>(ui->DeviceTypeComboBox->currentIndex());
	settings.setValue("DeviceTypeComboBox", ui->DeviceTypeComboBox->currentIndex());

	te::ComputePrecision precision;
	precision = static_cast<te::ComputePrecision>(ui->ComputePrecisionComboBox->currentIndex());
	settings.setValue("ComputePrecisionComboBox", ui->ComputePrecisionComboBox->currentIndex());

	workAiModel->InitTestConfig(
		MaxBatchSize, BatchSize,
		maxContourCount, maxContourPointCount, maxInnerContourCount,
		deviceID, devicetype, precision
	);
}
