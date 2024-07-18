#include "teMainView.h"
#include <QStackedLayout>
#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
using namespace te;

MainView::MainView(QWidget* parent)
	: QWidget(parent)
	, ui(new Ui::MainViewClass())
{
	ui->setupUi(this);
	stackLayout = new QStackedLayout();
	ui->CanvasLayout->addLayout(stackLayout);
	stackLayout->setStackingMode(QStackedLayout::StackAll);//显示所有stackedLayout中的元素
	initToolbar();
}

MainView::~MainView()
{
	delete ui;
}

void MainView::bindViewModel(std::shared_ptr<ViewModel> vm)
{
	viewModel = vm;
	if (viewModel.lock().get())
	{
		connect(viewModel.lock().get(), &ViewModel::notified, this, &MainView::refresh);
	}
}

void MainView::addWidgetToStackLayout(QWidget* widget, ViewModel::TypeWidget type)
{
	stackLayout->addWidget(widget);
	widgetMap[type] = widget;
}

void MainView::addWidgetToLabelLayout(QWidget* widget)
{
	ui->labelLayout->addWidget(widget);
}

void MainView::addWidgetToBrowserLayout(QWidget* widget)
{
	ui->browserLayout->addWidget(widget);
}

void MainView::addWidgetToToolBarLayout(QWidget* widget)
{
	ui->CanvasToolBarLayout->addWidget(widget);
}

void te::MainView::initToolbar()
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
	Train_menu->addAction(Stop_Train);
	menu_bar->addMenu(Train_menu);

	connect(Load_Images, &QAction::triggered, this, &MainView::loadTrainingImages);
	connect(Start_Train, &QAction::triggered, this, &MainView::prepareTrain);
	connect(Stop_Train, &QAction::triggered, this, &MainView::stopTrain);
	connect(Start_Test, &QAction::triggered, this, &MainView::prepareTest);

	QAction* saveAction = new QAction(tr("Save"), this);
	saveAction->setShortcut(QKeySequence(Qt::CTRL + Qt::Key_S));
	connect(saveAction, &QAction::triggered, this, &MainView::saveParameter);
	addAction(saveAction);
}

void MainView::setCurrentWidgetForStackLayout()
{
	ViewModel::TypeWidget type = viewModel.lock()->getCurrentWidgetType();
	QWidget* widget = widgetMap[type];
	if (widget)
	{
		stackLayout->setCurrentWidget(widget);
	}
}

void te::MainView::loadTrainingImages()
{
	QStringList filepaths = QFileDialog::getOpenFileNames(nullptr, u8"选择文件", "", "TIFF Files (*.tif *.tiff)");
	viewModel.lock()->loadTrainingImages(filepaths);
	viewModel.lock()->notified(ViewModel::InitPointCloud);
}

void te::MainView::prepareTrain()
{
	std::string fileName = "2.te";
	viewModel.lock()->notified(ViewModel::TrainPara);
	viewModel.lock()->prepareTrain(fileName);
}

void te::MainView::stopTrain()
{
	viewModel.lock()->stopTrain();
}

void te::MainView::prepareTest()
{
	std::string fileName = "2.te";
	viewModel.lock()->notified(ViewModel::TestPara);
	viewModel.lock()->prepareTest(fileName);
}

void te::MainView::saveParameter()
{
	viewModel.lock()->notified(ViewModel::TestPara);
	viewModel.lock()->notified(ViewModel::TrainPara);
}

void MainView::setThresholdSpinBox()
{
	double val = viewModel.lock()->getCurrentValidPointThreshold();
	double inval = viewModel.lock()->getCurrentInvalidPointThreshold();
	ui->ValidPointThresholdSpinBox->setValue(val);
	ui->InvalidPointThresholdSpinBox->setValue(inval);
}

void MainView::refresh(ViewModel::updateMode mode)
{
	if (mode == ViewModel::StartMark)
	{
		setCurrentWidgetForStackLayout();
	}
}
