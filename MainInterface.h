#pragma once

#include <QWidget>
#include "ui_MainInterface.h"
#include "pcl_function.h"
QT_BEGIN_NAMESPACE
namespace Ui { class MainInterfaceClass; };
QT_END_NAMESPACE

class QStackedLayout;
class te2DCanvasController;
class te3DCanvasController;
class teAlgorithmController;
class teDataBrowserController;
class QStateMachine;
class QState;
class teAiModel;
class teLabelBrowser;
class teMouseCircle;

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
	void labelChange(const QString & category, const QColor & fontColor);
	void ColorChange(const QColor& fontColor);
	void updateResultOperate();
	void IndexChanged();
	void updateTrainWidget();
	void updateResultWidget();
	void SetreceptiveFieldCurrrentWidget();
	void ResetMouseRadius();

private:
	void SetThreshold(QString filePath);
	void SetClassBCallback(teMouseCircle& classB);
	
private:
	Ui::MainInterfaceClass *ui;
	
signals:
	void sig_LoadTrainingImages(const QStringList& filePaths);
	void sig_SaveParameter();
	void sig_setHeightCoefficientFactor(int factor);
	void sig_teUpDataSet(int iNum, int iLayerNum, bool bReset);
	void sig_LoadTrainImagesComplete();
	void sig_ColorChanged();
	void sig_CurrentStateChanged(const QString& category, const QColor& fontColor, const int& index, const double& valThreshold, const double& invalThreshold);
	void sig_enterThrD();
	void sig_enterTwoD();

private:
	QStateMachine* m_pStateMachine;
	QState* TwoDState;
	QState* ThrDState;
	QStackedLayout* stacklayout;
	teAiModel* m_teAiModel;
	teAlgorithmController* m_teAlgorithmController;
	teDataBrowserController* m_teIBController;
	te2DCanvasController* m_te2DController;
	te3DCanvasController* m_te3DController;
	teLabelBrowser* m_teLabelBrowser;
	teMouseCircle* m_teMouseCircle;

	bool HastheImageBeenLoaded = false;
	struct currentState m_curstate;
};
