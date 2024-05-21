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
#include <QStackedLayout>
#include <QKeyEvent>

MainInterface::MainInterface(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::MainInterfaceClass())
{
	ui->setupUi(this);
	this->showMaximized();
	QStackedLayout* stacklayout = new QStackedLayout();
	ui->CanvasLayout->addLayout(stacklayout);
	stacklayout->setStackingMode(QStackedLayout::StackAll);
	te3DCanvasController::getInstance()->displayCanvasInWidget(stacklayout);
	te2DCanvasController::getInstance()->displayCanvasInWidget(stacklayout);
	te3DCanvasController::getInstance()->displayToolBarInWidget(ui->CanvasToolBarLayout);
	te2DCanvasController::getInstance()->displayToolBarInWidget(ui->CanvasToolBarLayout);
	te3DCanvasController::getInstance()->hideAllUI();
	
	te2DCanvasController::getInstance()->showAllUI();
	teDataStorage::getInstance()->displayUIInWidget(ui->labelLayout);
	teImageBrowserController::getInstance()->displayUIInWidget(ui->browserLayout);

	m_AiModelController = new AiModelController();
	m_AiModelController->displayUIInWidget(ui->labelLayout);

	m_mouseCircle = new teMouseCircle();
	m_mouseCircle->show();
	m_mouseCircle->receptiveFieldChange(m_AiModelController->getReceptiveField());
	stacklayout->addWidget(m_mouseCircle);
	stacklayout->setCurrentWidget(m_mouseCircle);
	te3DCanvasController::getInstance()->SetClassBCallback(*m_mouseCircle);

	InitStateMachine();
	InitToolBar();
	connect(te3DCanvasController::getInstance(), &te3DCanvasController::sig_DisonnectHeightTransform, this, &MainInterface::ResetMouseRadius);
	connect(ui->convertBtn, &QPushButton::clicked, teImageBrowserController::getInstance(), &teImageBrowserController::sig_ChangeCurrentState);
	connect(this, &MainInterface::sig_setHeightCoefficientFactor, te3DCanvasController::getInstance(), &te3DCanvasController::sig_setHeightCoefficientFactor);

	connect(teDataStorage::getInstance(), &teDataStorage::sig_teUpDataSet, teImageBrowserController::getInstance(), &teImageBrowserController::sig_teUpDataSet);
	connect(teDataStorage::getInstance(), &teDataStorage::sig_LoadTrainImagesComplete, te2DCanvasController::getInstance(),&te2DCanvasController::sig_StartMarking);
	
	connect(teDataStorage::getInstance(), &teDataStorage::sig_currentLabelChange, te2DCanvasController::getInstance(), &te2DCanvasController::sig_currentLabelChange);
	connect(te2DCanvasController::getInstance(), &te2DCanvasController::sig_ClearCurrentTrainGT,teDataStorage::getInstance(), &teDataStorage::clearCurrentMarkersGT);

	connect(teImageBrowserController::getInstance(), &teImageBrowserController::sig_showAll2DItem, te2DCanvasController::getInstance(), &te2DCanvasController::ShowAllItems);

	m_SChart = new teTrainStatisticsChart();
	m_SChart->hide();
	connect(teDataStorage::getInstance(), &teDataStorage::sig_DataChangeDuringTraining, m_SChart, &teTrainStatisticsChart::ReceiveData);
	connect(m_AiModelController, &AiModelController::sig_isShowTSChart, m_SChart, &teTrainStatisticsChart::isShow);
	connect(m_SChart, &teTrainStatisticsChart::sig_closeteTrainStatisticsChart, m_AiModelController, &AiModelController::sig_TSChartClose);
	connect(m_AiModelController, &AiModelController::sig_receptiveFieldChange, m_mouseCircle, &teMouseCircle::receptiveFieldChange);

	QAction* saveAction = new QAction(tr("Save"), this);
	saveAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_S));
	connect(saveAction, &QAction::triggered, this, &MainInterface::sig_SaveParameter);
	addAction(saveAction);
}

MainInterface::~MainInterface()
{
	ClearAllCaches();
	delete m_mouseCircle;
	delete ui;
}

void MainInterface::InitStateMachine()
{
    m_pStateMachine = new QStateMachine();
    TwoDState = new QState(m_pStateMachine);
    ThrDState = new QState(m_pStateMachine);

	connect(TwoDState, &QState::entered, te2DCanvasController::getInstance(), &te2DCanvasController::showAllUI);
	connect(TwoDState, &QState::entered, this, &MainInterface::ChangeBtnTextTo2D);
	connect(TwoDState, &QState::entered, this, &MainInterface::ResetMouseRadius);
	connect(TwoDState, &QState::exited, te2DCanvasController::getInstance(), &te2DCanvasController::hideAllUI);
	connect(ThrDState, &QState::entered, te3DCanvasController::getInstance(), &te3DCanvasController::showAllUI);
	connect(ThrDState, &QState::entered, this, &MainInterface::ChangeBtnTextTo3D);
	connect(ThrDState, &QState::entered, this, &MainInterface::ResetMouseRadius);
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
	connect(this, &MainInterface::sig_LoadTrainingImages, teDataStorage::getInstance(), &teDataStorage::LoadTrainingImages);
	connect(Start_Train, &QAction::triggered, m_AiModelController, &AiModelController::sig_PrepareTrain);
	connect(Stop_Train, &QAction::triggered, m_AiModelController, &AiModelController::sig_StopTrain);
	connect(Start_Test, &QAction::triggered, m_AiModelController, &AiModelController::sig_PrepareTest);
	connect(this, &MainInterface::sig_SaveParameter, m_AiModelController, &AiModelController::sig_SaveParameter);
}

void MainInterface::ClearAllCaches()
{
	QDir currentDir = QDir::current();
	QStringList filters;
	filters << "*.pcd" << "*.bmp";
	QFileInfoList fileList = currentDir.entryInfoList(filters, QDir::Files);

	foreach(QFileInfo fileInfo, fileList) {
		QString filePath = fileInfo.absoluteFilePath();
		currentDir.remove(filePath);
	}
}

void MainInterface::on_InvalidPointThresholdSpinBox_valueChanged(double arg)
{
	if (!HastheImageBeenLoaded) {
		teDataStorage::getInstance()->InvalidPointThresholdsChange(arg);
	}
}

void MainInterface::on_ValidPointThresholdSpinBox_valueChanged(double arg)
{
	if (!HastheImageBeenLoaded) {
		teDataStorage::getInstance()->ValidPointThresholdsChange(arg);
	}
}

void MainInterface::on_ThresholdBtn_clicked()
{
	teDataStorage::getInstance()->DeleteCurrentPointCloudAndThumbnail();

	teDataStorage::getInstance()->ValidPointThresholdChange(ui->ValidPointThresholdSpinBox->value());
	teDataStorage::getInstance()->InvalidPointThresholdChange(ui->InvalidPointThresholdSpinBox->value());

	emit teImageBrowserController::getInstance()->sig_GenerateCurrentData();

	if (TwoDState->active()) {
		te2DCanvasController::getInstance()->ShowCurrentImages();
		te2DCanvasController::getInstance()->showAllUI();
	}
	else if(ThrDState->active()) {
		te3DCanvasController::getInstance()->showAllUI();
	}
}

void MainInterface::on_clearDatabaseBtn_clicked()
{
	teDataStorage::getInstance()->DropAllTables();
}

void MainInterface::ChangeBtnTextTo2D()
{
	ui->convertBtn->setText(u8"转换到3D");
}

void MainInterface::ChangeBtnTextTo3D()
{
	ui->convertBtn->setText(u8"转换到2D");
}

void MainInterface::ResetMouseRadius()
{
	m_mouseCircle->receptiveFieldChange(m_AiModelController->getReceptiveField());
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

	minValue = -maxValue;

	ui->ValidPointThresholdSpinBox->setValue(maxValue);
	ui->InvalidPointThresholdSpinBox->setValue(minValue);

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
	//ui->clearDatabaseBtn->setVisible(false);
	QStringList filepaths = QFileDialog::getOpenFileNames(nullptr, u8"选择文件", "", "TIFF Files (*.tif *.tiff)");
	teDataStorage::getInstance()->setCurrentLoadImageNum(filepaths.size());
	teDataStorage::getInstance()->InitThreasholds(filepaths.size());
	if (!filepaths.isEmpty())
	{
		if (ui->AutomaticCheckBox->isChecked()) {
			SetThreshold(filepaths[0]);
		}
		HastheImageBeenLoaded = true;
		emit sig_LoadTrainingImages(filepaths);
	}
}
