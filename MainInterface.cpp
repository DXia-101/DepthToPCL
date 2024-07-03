#include "MainInterface.h"
#include "te3DCanvasController.h"
#include "te2DCanvasController.h"
#include "teDataBrowserController.h"
#include "teAiModel.h"
#include "teAi3DStorage.h"
#include "teAlgorithmController.h"
#include "teLabelBrowser.h"
#include "teMouseCircle.h"

#include <QMenuBar>
#include <QMenu>
#include <QToolBar>
#include <QAction>
#include <QFileDialog>
#include <QStackedLayout>
#include <QKeyEvent>
#include <QDir>
#include <QStateMachine>
#include <QState>
#include <QEvent>
#include <QPushButton>
#include <QMessageBox>

MainInterface::MainInterface(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::MainInterfaceClass())
{
	QDir currentDir(QDir::currentPath()); // 获取当前工作目录

	currentDir.mkdir("workspace");

	ui->setupUi(this);
	
	stacklayout = new QStackedLayout();
	ui->CanvasLayout->addLayout(stacklayout);
	stacklayout->setStackingMode(QStackedLayout::StackAll);

	auto dbStore = std::make_unique<teAi3DStorage>();
	m_teAiModel = new teAiModel(std::move(dbStore));

	m_teIBController = new teDataBrowserController();
	m_teIBController->displayUIInWidget(ui->browserLayout);
	m_teIBController->setteAiModel(m_teAiModel);

	m_teLabelBrowser = new teLabelBrowser();
	m_teLabelBrowser->displayUIInWidget(ui->labelLayout);

	m_te3DController = new te3DCanvasController();
	m_te3DController->setteAiModel(m_teAiModel);
	m_te3DController->setteLabelBrowser(m_teLabelBrowser);

	m_te2DController = new te2DCanvasController();
	m_te2DController->setteAiModel(m_teAiModel);
	m_te2DController->setteLabelBrowser(m_teLabelBrowser);

	m_te3DController->displayCanvasInWidget(stacklayout);
	m_te2DController->displayCanvasInWidget(stacklayout);
	m_te3DController->displayToolBarInWidget(ui->CanvasToolBarLayout);
	m_te2DController->displayToolBarInWidget(ui->CanvasToolBarLayout);
	m_teAlgorithmController = new teAlgorithmController();
	m_teAlgorithmController->displayUIInWidget(ui->labelLayout);
	m_teAlgorithmController->setteAiModel(m_teAiModel);

	m_teMouseCircle = new teMouseCircle();
	m_teMouseCircle->show();
	
	stacklayout->addWidget(m_teMouseCircle);
	stacklayout->setCurrentWidget(m_teMouseCircle);


	InitStateMachine();
	InitToolBar();
	
	m_te3DController->hideAllUI();
	this->showMaximized();
	connect(ui->convertBtn, &QPushButton::clicked, m_teIBController, &teDataBrowserController::sig_ChangeCurrentState);
	connect(this, &MainInterface::sig_teUpDataSet, m_teIBController, &teDataBrowserController::sig_teUpDataSet);
	connect(this, &MainInterface::sig_LoadTrainImagesComplete, m_te2DController, &te2DCanvasController::sig_ShowFirstImage);
	connect(this, &MainInterface::sig_setHeightCoefficientFactor, m_te3DController, &te3DCanvasController::sig_setHeightCoefficientFactor);
	connect(this, &MainInterface::sig_ColorChanged, m_te3DController, &te3DCanvasController::ReLoadGTAndRST);
	connect(this, &MainInterface::sig_ColorChanged, m_te2DController, &te2DCanvasController::ReLoadGTAndRST);
	connect(this, &MainInterface::sig_CurrentStateChanged, m_te3DController, &te3DCanvasController::sig_CurrentStateChanged);
	connect(this, &MainInterface::sig_CurrentStateChanged, m_te2DController, &te2DCanvasController::sig_CurrentStateChanged);
	connect(this, &MainInterface::sig_enterThrD, m_teMouseCircle, &teMouseCircle::sig_enterThrD);
	connect(this, &MainInterface::sig_enterTwoD, m_teMouseCircle, &teMouseCircle::sig_enterTwoD);

	connect(m_teLabelBrowser, &teLabelBrowser::sig_currentRowSelected, this, &MainInterface::labelChange);
	connect(m_teLabelBrowser, &teLabelBrowser::sig_ColorChanged, this, &MainInterface::ColorChange);
	connect(m_teLabelBrowser, &teLabelBrowser::sig_StartMark, m_te2DController, &te2DCanvasController::sig_StartMark);

	connect(m_te3DController, &te3DCanvasController::sig_ManagePolyLine, this, &MainInterface::ManagePolyLine);
	connect(m_te3DController, &te3DCanvasController::sig_updateTrainWidget, this, &MainInterface::updateTrainWidget);
	connect(m_te3DController, &te3DCanvasController::sig_ReLoadGTAndRST, this, &MainInterface::ResetMouseRadius);
	connect(m_te3DController, &te3DCanvasController::sig_updateTrainWidget, this, &MainInterface::updateTrainWidget);
	connect(m_te3DController, &te3DCanvasController::sig_MarkerComplete, this, &MainInterface::SetreceptiveFieldCurrrentWidget);
	connect(m_te3DController, &te3DCanvasController::sig_OutOfBounds, m_teMouseCircle, &teMouseCircle::OutOfBounds);

	connect(m_te2DController, &te2DCanvasController::sig_updateTrainWidget, m_te3DController, &te3DCanvasController::ShowAllMarkers);
	connect(m_te2DController, &te2DCanvasController::sig_eraseMarkers, m_te3DController, &te3DCanvasController::ShowAllMarkers);
	connect(m_te2DController, &te2DCanvasController::sig_updateTrainWidget,this, &MainInterface::updateTrainWidget);
	connect(m_te2DController, &te2DCanvasController::sig_AfterFirstImageShow,this, &MainInterface::ResetMouseRadius);

	connect(m_teAlgorithmController, &teAlgorithmController::sig_TestCompleted, this, &MainInterface::updateResultOperate);
	connect(m_teAlgorithmController, &teAlgorithmController::sig_TestCompleted, m_te3DController, &te3DCanvasController::ShowAllItems);
	connect(m_teAlgorithmController, &teAlgorithmController::sig_TestCompleted, m_te2DController, &te2DCanvasController::ShowAllItems);
	connect(m_teAlgorithmController, &teAlgorithmController::sig_receptiveFieldChange, m_te2DController, &te2DCanvasController::receptiveFieldChange);
	connect(m_te2DController, &te2DCanvasController::sig_receptiveFieldChange, m_teMouseCircle, &teMouseCircle::receptiveFieldChange);

	connect(m_teIBController, &teDataBrowserController::sig_IndexChanged, this, &MainInterface::IndexChanged);
	connect(m_teIBController, &teDataBrowserController::sig_IndexChanged, this, &MainInterface::ResetMouseRadius);
	connect(m_teIBController, &teDataBrowserController::sig_updateTrainWidget, this, &MainInterface::updateTrainWidget);
	connect(m_teIBController, &teDataBrowserController::sig_updateResultWidget, this, &MainInterface::updateResultWidget);
	connect(m_teIBController, &teDataBrowserController::sig_NeedReload, m_te3DController, &te3DCanvasController::NeedReload);
	connect(m_teIBController, &teDataBrowserController::sig_NeedReload, m_te2DController, &te2DCanvasController::NeedReload);
	connect(m_teIBController, &teDataBrowserController::sig_LoadPointCloud, m_te3DController, &te3DCanvasController::LoadPointCloud);
	connect(m_teIBController, &teDataBrowserController::sig_LoadOriginImage, m_te2DController, &te2DCanvasController::LoadOriginImage);

	QAction* saveAction = new QAction(tr("Save"), this);
	saveAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_S));
	connect(saveAction, &QAction::triggered, this, &MainInterface::sig_SaveParameter);
	addAction(saveAction);

	m_curstate.currentCategory = "";
	m_curstate.currentColor = Qt::black;
	m_curstate.currentIndex = 0;
	m_curstate.currentValidPointThreshold = 0.0;
	m_curstate.currentInvalidPointThreshold = 0.0;
	emit sig_CurrentStateChanged(m_curstate.currentCategory, m_curstate.currentColor, m_curstate.currentIndex, m_curstate.currentValidPointThreshold, m_curstate.currentInvalidPointThreshold);
}

MainInterface::~MainInterface()
{
	delete m_teMouseCircle;
	delete ui;
}

void MainInterface::InitStateMachine()
{
    m_pStateMachine = new QStateMachine();
    TwoDState = new QState(m_pStateMachine);
    ThrDState = new QState(m_pStateMachine);

	connect(TwoDState, &QState::entered, m_te2DController, &te2DCanvasController::showAllUI);
	connect(TwoDState, &QState::entered, this, &MainInterface::sig_enterTwoD);
	connect(TwoDState, &QState::entered, this, &MainInterface::ChangeBtnTextTo2D);
	connect(TwoDState, &QState::exited, m_te2DController, &te2DCanvasController::hideAllUI);
	connect(ThrDState, &QState::entered, m_te3DController, &te3DCanvasController::showAllUI);
	connect(ThrDState, &QState::entered, this, &MainInterface::sig_enterThrD);
	connect(ThrDState, &QState::entered, this, &MainInterface::ChangeBtnTextTo3D);
	connect(ThrDState, &QState::exited, m_te3DController, &te3DCanvasController::hideAllUI);

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

	QMenu* operate_menu = new QMenu(u8"文件", menu_bar);
	QAction* Load_Images = new QAction(u8"加载图片");
	operate_menu->addAction(Load_Images);

	menu_bar->addMenu(operate_menu);

	QMenu* Train_menu = new QMenu(u8"训练", menu_bar);
	QAction* Start_Train = new QAction(u8"开始训练");
	QAction* Stop_Train = new QAction(u8"停止训练");
	QAction* Start_Test = new QAction(u8"开始测试");

	Train_menu->addAction(Start_Train);
	Train_menu->addAction(Start_Test);
	menu_bar->addMenu(Train_menu);

	connect(Load_Images, &QAction::triggered, this, &MainInterface::LoadTrainingImages);
	connect(Start_Train, &QAction::triggered, m_teAlgorithmController, &teAlgorithmController::sig_PrepareTrain);
	connect(Stop_Train, &QAction::triggered, m_teAlgorithmController, &teAlgorithmController::sig_StopTrain);
	connect(Start_Test, &QAction::triggered, m_teAlgorithmController, &teAlgorithmController::sig_PrepareTest);
	connect(this, &MainInterface::sig_SaveParameter, m_teAlgorithmController, &teAlgorithmController::sig_SaveParameter);
}

void MainInterface::ClearAllCaches()
{
	if (!m_teAiModel->clearAllPointCloud())
		std::cerr << "clearAllPointCloud fail" << std::endl;
	if (!m_teAiModel->clearAllThumbnail())
		std::cerr << "clearAllThumbnail fail" << std::endl;
}

void MainInterface::on_InvalidPointThresholdSpinBox_valueChanged(double arg)
{
	if (!HastheImageBeenLoaded) {
		m_teAiModel->InvalidPointThresholdsChange(arg);
	}
}

void MainInterface::on_ValidPointThresholdSpinBox_valueChanged(double arg)
{
	if (!HastheImageBeenLoaded) {
		m_teAiModel->ValidPointThresholdsChange(arg);
	}
}

void MainInterface::on_ThresholdBtn_clicked()
{
	if (m_teAiModel->getCurrentLoadImageNum() != 0)
	{
		m_teAiModel->clearCurrentPointCloudAndThumbnail();

		m_teAiModel->ValidPointThresholdChange(ui->ValidPointThresholdSpinBox->value());
		m_teAiModel->InvalidPointThresholdChange(ui->InvalidPointThresholdSpinBox->value());

		if (TwoDState->active()) {
			m_te2DController->ShowCurrentImages();
			m_te2DController->NeedReload();
			m_te2DController->showAllUI();
		}
		else if (ThrDState->active()) {
			m_te3DController->NeedReload();
			m_te3DController->showAllUI();
		}
	}
}

void MainInterface::on_clearDatabaseBtn_clicked()
{
	m_teAiModel->DropAllTables();
	ClearAllCaches();
}

void MainInterface::ChangeBtnTextTo2D()
{
	ui->convertBtn->setText(u8"转换到3D");
}

void MainInterface::ChangeBtnTextTo3D()
{
	ui->convertBtn->setText(u8"转换到2D");
}

void MainInterface::ManagePolyLine()
{
	if (m_curstate.currentCategory == "") 
	{
		QMessageBox::information(nullptr, u8"提示", u8"请先选择一个标签！", QMessageBox::Ok);
		emit m_te3DController->sig_MarkerButtonRecovery();
		return;
	}
	m_te3DController->ManagePolyLine(stacklayout);
}

void MainInterface::labelChange(const QString& category, const QColor& fontColor)
{
	m_curstate.currentCategory = category;
	m_curstate.currentColor = fontColor;
	emit sig_CurrentStateChanged(m_curstate.currentCategory, m_curstate.currentColor, m_curstate.currentIndex, m_curstate.currentValidPointThreshold, m_curstate.currentInvalidPointThreshold);
}

void MainInterface::ColorChange(const QColor& fontColor)
{
	m_curstate.currentColor = fontColor;
	emit sig_CurrentStateChanged(m_curstate.currentCategory, m_curstate.currentColor, m_curstate.currentIndex, m_curstate.currentValidPointThreshold, m_curstate.currentInvalidPointThreshold);
	emit sig_ColorChanged();
}

void MainInterface::updateResultOperate()
{
	m_teLabelBrowser->updateTrainWidget(&m_teAiModel->getCurrentResultMarksNumber());
}

void MainInterface::IndexChanged()
{
	m_curstate.currentIndex = m_teAiModel->getCurrentIndex();

	m_curstate.currentValidPointThreshold = m_teAiModel->getCurrentValidPointThreshold();
	m_curstate.currentInvalidPointThreshold = m_teAiModel->getCurrentInvalidPointThreshold();

	ui->ValidPointThresholdSpinBox->setValue(m_curstate.currentValidPointThreshold);
	ui->InvalidPointThresholdSpinBox->setValue(m_curstate.currentInvalidPointThreshold);

	emit sig_CurrentStateChanged(m_curstate.currentCategory, m_curstate.currentColor, m_curstate.currentIndex, m_curstate.currentValidPointThreshold, m_curstate.currentInvalidPointThreshold);
}

void MainInterface::updateTrainWidget()
{
	m_teLabelBrowser->updateTrainWidget(&m_teAiModel->getCurrentTrainMarksNumber());
}

void MainInterface::updateResultWidget()
{
	m_teLabelBrowser->updateResultWidget(&m_teAiModel->getCurrentResultMarksNumber());
}

void MainInterface::SetreceptiveFieldCurrrentWidget()
{
	stacklayout->setCurrentWidget(m_teMouseCircle);
}

void MainInterface::SetThreshold(QString filePath)
{
	//计算阈值
	cv::Mat image = cv::imread(filePath.toStdString(), cv::IMREAD_UNCHANGED);

	if (image.empty() || (image.type() != CV_32FC1 && image.type() != CV_16UC1)) {
		return;
	}

	double minValue, maxValue;
	cv::minMaxLoc(image, &minValue, &maxValue);

	minValue = -(maxValue*0.8);

	ui->ValidPointThresholdSpinBox->setValue(maxValue);
	ui->InvalidPointThresholdSpinBox->setValue(minValue);

	m_curstate.currentValidPointThreshold = maxValue;
	m_curstate.currentInvalidPointThreshold = minValue;
	emit sig_CurrentStateChanged(m_curstate.currentCategory, m_curstate.currentColor, m_curstate.currentIndex, m_curstate.currentValidPointThreshold, m_curstate.currentInvalidPointThreshold);
	m_teAiModel->addInvalidPointThreshold(m_curstate.currentIndex,minValue);
	m_teAiModel->addValidPointThreshold(m_curstate.currentIndex,maxValue);

	int factor = maxValue - minValue;
	if (factor >= 1 && factor < 255) {
		factor = 255 / factor;
	}
	else {
		factor = 1;
	}
	emit sig_setHeightCoefficientFactor(factor);
}

void MainInterface::LoadTrainingImages()
{
	QStringList filepaths = QFileDialog::getOpenFileNames(nullptr, u8"选择文件", "", "TIFF Files (*.tif *.tiff)");
	m_teAiModel->setCurrentIndex(0);
	m_teAiModel->InitThreasholds(filepaths.size());
	m_te3DController->NeedReload();
	if (!filepaths.isEmpty())
	{
		if (ui->AutomaticCheckBox->isChecked()) {
			SetThreshold(filepaths[0]);
		}
		HastheImageBeenLoaded = true;
		m_teAiModel->LoadTrainingImages(filepaths);
		emit sig_teUpDataSet(filepaths.size(), 1, true);
		emit sig_LoadTrainImagesComplete();
	}
}

void MainInterface::ResetMouseRadius()
{
	m_te2DController->receptiveFieldChange(m_teAlgorithmController->getReceptiveField());
}