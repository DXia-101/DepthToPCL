#pragma once

#include <QWidget>
#include "ui_MainInterface.h"

#include "AiModelController.h"
#include "teTrainStatisticsChart.h"

#include <QStateMachine>
#include <QState>

QT_BEGIN_NAMESPACE
namespace Ui { class MainInterfaceClass; };
QT_END_NAMESPACE

class MainInterface : public QWidget
{
	Q_OBJECT

public:
	MainInterface(QWidget *parent = nullptr);
	~MainInterface();

	void InitStateMachine();
	void InitToolBar();

private slots:
	void LoadTrainingImages();
	void on_InvalidPointThresholdSpinBox_valueChanged(int arg);
	void ConnectHeightTransform();
	void DisconnectHeightTransform();

private:
	Ui::MainInterfaceClass *ui;
	
signals:
	void sig_LoadTrainingImages(const QStringList& filePaths);
	void sig_InvalidPointThresholdChange(int threshold);

private:
	QStateMachine* m_pStateMachine;
	QState* TwoDState;
	QState* ThrDState;
	AiModelController* m_AiModelController;
	teTrainStatisticsChart* m_SChart;
};
