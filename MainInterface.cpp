#include "MainInterface.h"
#include "te3DCanvasController.h"
#include "te2DCanvasController.h"

MainInterface::MainInterface(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::MainInterfaceClass())
{
	ui->setupUi(this);

	te3DCanvasController::getInstance()->displayUIInWidget(ui->canvasLayout);
	te3DCanvasController::getInstance()->hideAllUI();
	te2DCanvasController::getInstance()->displayUIInWidget(ui->canvasLayout);
	te2DCanvasController::getInstance()->showAllUI();
	m_teLabelBrowser = new teLabelBrowser();
	ui->labelLayout->addWidget(m_teLabelBrowser);
	m_teImageBrowserController = new teImageBrowserController();
	m_teImageBrowserController->displayUIInWidget(ui->browserLayout);

	InitStateMachine();

	connect(te3DCanvasController::getInstance(), &te3DCanvasController::sig_GTShowSignalChange, m_teImageBrowserController, &teImageBrowserController::ChangeGTShowFlag);
	connect(te3DCanvasController::getInstance(), &te3DCanvasController::sig_RSTShowSignalChange, m_teImageBrowserController, &teImageBrowserController::ChangeRSTShowFlag);
	connect(ui->convertBtn, &QPushButton::clicked, m_teImageBrowserController, &teImageBrowserController::ChangeCurrentState);
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

	connect(TwoDState, &QState::entered, te2DCanvasController::getInstance(), &te2DCanvasController::showAllUI);
	connect(TwoDState, &QState::exited, te2DCanvasController::getInstance(), &te2DCanvasController::hideAllUI);
	connect(ThrDState, &QState::entered, te3DCanvasController::getInstance(), &te3DCanvasController::showAllUI);
	connect(ThrDState, &QState::exited, te3DCanvasController::getInstance(), &te3DCanvasController::hideAllUI);

	TwoDState->addTransition(ui->convertBtn, &QPushButton::clicked, ThrDState);
	ThrDState->addTransition(ui->convertBtn, &QPushButton::clicked, TwoDState);

    m_pStateMachine->addState(TwoDState);
    m_pStateMachine->addState(ThrDState);

    m_pStateMachine->setInitialState(TwoDState);
    m_pStateMachine->start();
}