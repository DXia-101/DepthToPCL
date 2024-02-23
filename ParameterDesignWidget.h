#pragma once

#include <QWidget>
#include "ui_ParameterDesignWidget.h"
#include "AiModelInterface.h"
#include "TrainingStatisticsChart.h"


QT_BEGIN_NAMESPACE
namespace Ui { class ParameterDesignWidgetClass; };
QT_END_NAMESPACE

class ParameterDesignWidget : public QWidget
{
	Q_OBJECT

public:
	ParameterDesignWidget(AiModelInterface* workaimodel,QWidget *parent = nullptr);
	~ParameterDesignWidget();

protected slots:
	void on_NetworkListCBox_currentTextChanged(const QString& arg1);
	void on_ChannelsCBox_currentTextChanged(const QString& arg1);
	void on_dtypeCBox_currentIndexChanged(int index);
	void on_eToolTypeCBox_currentIndexChanged(int index);
	void on_eTrainModeCBox_currentIndexChanged(int index);
	void on_locateTypeBox_currentIndexChanged(int index);


	void on_receptiveFieldEdit_textEdited(const QString& arg1);
	void on_heapIDEdit_textEdited(const QString& arg1);
	void on_trainIterCntEdit_textEdited(const QString& arg1);
	void on_BatchSizeEdit_textEdited(const QString& arg1);
	void on_PatchWidthEdit_textEdited(const QString& arg1);
	void on_DeviceIDEdit_textEdited(const QString& arg1);
	void on_PatchHeightEdit_textEdited(const QString& arg1);
	void on_saveFrequencyEdit_textEdited(const QString& arg1);
	void on_locateSideEdit_textEdited(const QString& arg1);

	void on_TrainingCurveCBox_stateChanged(int arg1);
	void on_AutomaticCBox_stateChanged(int arg1);

	void StartInitTrainConfigSlots();
private:
	Ui::ParameterDesignWidgetClass *ui;

	AiModelInterface* workAiModel;
	TrainingStatisticsChart* trainChart;
	QString configFilePath;
};
