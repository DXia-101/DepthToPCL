#include "MainInterface.h"
#include "te3DCanvasController.h"
#include "te2DCanvasController.h"
#include "teImageBrowserController.h"
#include "teDataStorage.h"

#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QAction>
#include <QFileDialog>

MainInterface::MainInterface(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::MainInterfaceClass())
{
	ui->setupUi(this);
	te3DCanvasController::getInstance()->displayUIInWidget(ui->canvasLayout);
	te3DCanvasController::getInstance()->hideAllUI();
	te2DCanvasController::getInstance()->displayUIInWidget(ui->canvasLayout);
	te2DCanvasController::getInstance()->showAllUI();
	teDataStorage::getInstance()->displayUIInWidget(ui->labelLayout);
	teImageBrowserController::getInstance()->displayUIInWidget(ui->browserLayout);

	m_AiModelController = new AiModelController();
	m_AiModelController->displayUIInWidget(ui->labelLayout);

	InitStateMachine();
	InitToolBar();

	connect(ui->convertBtn, &QPushButton::clicked, teImageBrowserController::getInstance(), &teImageBrowserController::ChangeCurrentState);
	connect(this, &MainInterface::sig_InvalidPointThresholdChange, teImageBrowserController::getInstance(), &teImageBrowserController::InvalidPointThresholdChange);

	connect(teDataStorage::getInstance(), &teDataStorage::sig_teUpDataSet, teImageBrowserController::getInstance(), &teImageBrowserController::teUpDataSet);
	connect(teDataStorage::getInstance(), &teDataStorage::sig_LoadTrainImagesComplete, te2DCanvasController::getInstance(),&te2DCanvasController::sig_StartMarking);
	
	connect(teDataStorage::getInstance(), &teDataStorage::sig_currentLabelChange, te2DCanvasController::getInstance(), &te2DCanvasController::sig_currentLabelChange);
	connect(te2DCanvasController::getInstance(), &te2DCanvasController::sig_ClearCurrentTrainGT,teDataStorage::getInstance(), &teDataStorage::clearCurrentMarkersGT);

	connect(teImageBrowserController::getInstance(), &teImageBrowserController::sig_showAll2DItem, te2DCanvasController::getInstance(), &te2DCanvasController::ShowAllItems);

	m_SChart = new teTrainStatisticsChart();
	m_SChart->hide();
	connect(teDataStorage::getInstance(), &teDataStorage::sig_DataChangeDuringTraining, m_SChart, &teTrainStatisticsChart::ReceiveData);
	connect(m_AiModelController, &AiModelController::sig_isShowTSChart, m_SChart, &teTrainStatisticsChart::isShow);
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

void MainInterface::InitToolBar()
{
	QMenuBar* menu_bar = new QMenuBar(this);
	ui->ToolBarLayout->addWidget(menu_bar);
	menu_bar->setStyleSheet("font-size : 18px");

	QMenu* Train_menu = new QMenu(u8"ÑµÁ·", menu_bar);

	QAction* Load_Images = new QAction(u8"¼ÓÔØÑµÁ·Í¼Æ¬");
	QAction* Start_Train = new QAction(u8"¿ªÊ¼ÑµÁ·");
	QAction* Stop_Train = new QAction(u8"Í£Ö¹ÑµÁ·");
	QAction* Start_Test = new QAction(u8"¿ªÊ¼²âÊÔ");

	Train_menu->addAction(Load_Images);
	Train_menu->addAction(Start_Train);
	Train_menu->addAction(Stop_Train);
	Train_menu->addAction(Start_Test);

	menu_bar->addMenu(Train_menu);

	connect(Load_Images, &QAction::triggered, this, &MainInterface::LoadTrainingImages);
	connect(this, &MainInterface::sig_LoadTrainingImages, teDataStorage::getInstance(), &teDataStorage::LoadTrainingImages);
	connect(Start_Train, &QAction::triggered, m_AiModelController, &AiModelController::sig_PrepareTrain);
	//connect(Stop_Train, &QAction::triggered, this, &DepthToPCL::StopTrainAction);
	connect(Start_Test, &QAction::triggered, m_AiModelController, &AiModelController::sig_PrepareTest);

}

void MainInterface::on_InvalidPointThresholdSpinBox_valueChanged(int arg)
{
	emit sig_InvalidPointThresholdChange(arg);
}

void MainInterface::LoadTrainingImages()
{
	QStringList filepaths = QFileDialog::getOpenFileNames(nullptr, u8"Ñ¡ÔñÎÄ¼þ", "", "TIFF Files (*.tif *.tiff)");
	emit sig_LoadTrainingImages(filepaths);
	teDataStorage::getInstance()->setCurrentLoadImageNum(filepaths.size());
}