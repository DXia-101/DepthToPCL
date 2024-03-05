#pragma once

#include <QWidget>
#include "ui_MainInterface.h"

#include "teLabelBrowser.h"
#include "teImageBrowserController.h"

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
	teLabelBrowser* m_teLabelBrowser;
	teImageBrowserController* m_teImageBrowserController;

signals:
	void ConversionBetween2Dand3D();

private:
	QStateMachine* m_pStateMachine;
	QState* TwoDState;
	QState* ThrDState;
};
