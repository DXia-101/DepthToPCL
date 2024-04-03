#pragma once

#include <QWidget>
#include "ui_MainInterface.h"

#include "AiModelController.h"
#include "teTrainStatisticsChart.h"
#include "teMouseCircle.h"

#include <QStateMachine>
#include <QState>
#include <QEvent>

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
	void ClearAllCaches();

private slots:
	void LoadTrainingImages();
	void on_InvalidPointThresholdSpinBox_valueChanged(int arg);
	void on_ValidPointThresholdSpinBox_valueChanged(int arg);
	void on_clearDatabaseBtn_clicked();
	void ConnectHeightTransform();
	void DisconnectHeightTransform();
	void ChangeBtnTextTo2D();
	void ChangeBtnTextTo3D();
	void ResetMouseRadius();

private:
	void SetThreshold(QString filePath);
	
private:
	Ui::MainInterfaceClass *ui;
	
signals:
	void sig_LoadTrainingImages(const QStringList& filePaths);
	void sig_InvalidPointThresholdChange(int threshold);
	void sig_ValidPointThresholdChange(int threshold);
	void sig_SaveParameter();

private:
	QStateMachine* m_pStateMachine;
	QState* TwoDState;
	QState* ThrDState;
	AiModelController* m_AiModelController;
	teTrainStatisticsChart* m_SChart;
	teMouseCircle* m_mouseCircle;
};
