#include "MainInterface.h"

MainInterface::MainInterface(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::MainInterfaceClass())
{
	ui->setupUi(this);
	m_te3DCanvasController = new te3DCanvasController();
	m_te3DCanvasController->displayUIInWidget(ui->canvasLayout);
	m_te2DCanvasController = new te2DCanvasController();
	m_te2DCanvasController->displayUIInWidget(ui->canvasLayout);
	m_te3DCanvasController->hideAllUI();
	m_teLabelBrowser = new teLabelBrowser();
	ui->labelLayout->addWidget(m_teLabelBrowser);
	m_teImageBrowserController = new teImageBrowserController();
	m_teImageBrowserController->displayUIInWidget(ui->browserLayout);

	InitStateMachine();
}

MainInterface::~MainInterface()
{
	delete ui;
}

void MainInterface::InitStateMachine()
{
    m_pStateMachine = new QStateMachine();
    TwoDState = new QState(m_pStateMachine);
    ThrDState = new QState(m_pStateMachine);

	connect(TwoDState, &QState::entered, m_te2DCanvasController, &te2DCanvasController::showAllUI);
	connect(TwoDState, &QState::exited, m_te2DCanvasController, &te2DCanvasController::hideAllUI);
	connect(ThrDState, &QState::entered, m_te3DCanvasController, &te3DCanvasController::showAllUI);
	connect(ThrDState, &QState::exited, m_te3DCanvasController, &te3DCanvasController::hideAllUI);

	TwoDState->addTransition(ui->convertBtn, &QPushButton::clicked, ThrDState);
	ThrDState->addTransition(ui->convertBtn, &QPushButton::clicked, TwoDState);

    m_pStateMachine->addState(TwoDState);
    m_pStateMachine->addState(ThrDState);

    m_pStateMachine->setInitialState(TwoDState);
    m_pStateMachine->start();
}
