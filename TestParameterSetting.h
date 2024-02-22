#pragma once

#include <QWidget>
#include "ui_TestParameterSetting.h"
#include "AiModelInterface.h"

QT_BEGIN_NAMESPACE
namespace Ui { class TestParameterSettingClass; };
QT_END_NAMESPACE

class TestParameterSetting : public QWidget
{
	Q_OBJECT

public:
	TestParameterSetting(AiModelInterface* workaimodel,QWidget *parent = nullptr);
	~TestParameterSetting();

protected slots:
	void on_MaxBatchSizeEdit_textEdited(const QString& arg1);
	void on_BatchSizeEdit_textEdited(const QString& arg1);
	void on_maxContourCountEdit_textEdited(const QString& arg1);
	void on_maxContourPointCountEdit_textEdited(const QString& arg1);
	void on_maxInnerContourCountEdit_textEdited(const QString& arg1);
	void on_DeviceIDEdit_textEdited(const QString& arg1);

	void on_DeviceTypeComboBox_currentIndexChanged(int index);
	void on_ComputePrecisionComboBox_currentIndexChanged(int index);

	void StartInitTestConfigSlots();
private:
	Ui::TestParameterSettingClass *ui;
	AiModelInterface* workAiModel;
};
