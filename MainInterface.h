#pragma once

#include <QWidget>
#include "ui_MainInterface.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainInterfaceClass; };
QT_END_NAMESPACE

class QStackedLayout;
class te2DCanvasController;
class te3DCanvasController;
class AiModelController;
class teTrainStatisticsChart;
class teLabelBrowser;
class QStateMachine;
class QState;

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
	void on_InvalidPointThresholdSpinBox_valueChanged(double arg);
	void on_ValidPointThresholdSpinBox_valueChanged(double arg);
	void on_ThresholdBtn_clicked();
	void on_clearDatabaseBtn_clicked();
	void ChangeBtnTextTo2D();
	void ChangeBtnTextTo3D();
	void ManagePolyLine();
	
private:
	void SetThreshold(QString filePath);
	
private:
	Ui::MainInterfaceClass *ui;
	
signals:
	void sig_LoadTrainingImages(const QStringList& filePaths);
	void sig_SaveParameter();
	void sig_setHeightCoefficientFactor(int factor);

private:
	QStateMachine* m_pStateMachine;
	QState* TwoDState;
	QState* ThrDState;
	AiModelController* m_AiModelController;
	teTrainStatisticsChart* m_SChart;
	QStackedLayout* stacklayout;
	teLabelBrowser* m_teLabelBrowser;
	te2DCanvasController* m_te2DController;
	te3DCanvasController* m_te3DController;

	bool HastheImageBeenLoaded = false;
};
