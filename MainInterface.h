#pragma once

#include <QWidget>
#include "ui_MainInterface.h"
#include "te3DCanvasController.h"
#include "te2DCanvasController.h"
#include "teLabelBrowser.h"

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

private:
	Ui::MainInterfaceClass *ui;
	te3DCanvasController* m_te3DCanvasController;
	te2DCanvasController* m_te2DCanvasController;
	teLabelBrowser* m_teLabelBrowser;

signals:
	void ConversionBetween2Dand3D();

private:
	QStateMachine* m_pStateMachine;
	QState* TwoDState;
	QState* ThrDState;
};
