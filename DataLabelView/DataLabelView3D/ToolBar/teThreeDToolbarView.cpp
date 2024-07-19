#include "teThreeDToolbarView.h"
#include "teCoordinateAxisRenderSetView.h"
#include "teCrossSectionSetView.h"
#include "tePointCloudPointSizeSetView.h"

#include <QColorDialog>
#include <QHBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QColor>

using namespace te;

ThreeDToolbarView::ThreeDToolbarView(QWidget* parent)
	: QWidget(parent)
	, ui(new Ui::ThreeDToolbarViewClass())
{
	ui->setupUi(this);
	initToolbar();
}

ThreeDToolbarView::~ThreeDToolbarView()
{
	delete ui;
}

void ThreeDToolbarView::bindViewModel(std::shared_ptr<ViewModel> vm)
{
	viewModel = vm;
	if (viewModel.lock())
	{
		connect(viewModel.lock().get(), &ViewModel::notified, this, &ThreeDToolbarView::refresh);
	}
}

void te::ThreeDToolbarView::initToolbar()
{
	QHBoxLayout* layout = new QHBoxLayout(this);
	QMenuBar* menu_bar = new QMenuBar(this);
	layout->addWidget(menu_bar);
	menu_bar->setStyleSheet("font-size : 18px");

	QMenu* Display_menu = new QMenu("显示", menu_bar);

	QAction* backgroundSet = new QAction("背景设置");
	QAction* CoordinateAxisRender = new QAction("点云渲染");
	QAction* pointCloudColorSet = new QAction("颜色设置");
	QAction* pointCloudSizeSet = new QAction("点云大小设置");

	Display_menu->addAction(backgroundSet);
	Display_menu->addAction(CoordinateAxisRender);
	Display_menu->addAction(pointCloudColorSet);
	Display_menu->addAction(pointCloudSizeSet);

	menu_bar->addMenu(Display_menu);

	connect(backgroundSet, &QAction::triggered, this, &ThreeDToolbarView::setBackgroundColor);
	connect(CoordinateAxisRender, &QAction::triggered, this, &ThreeDToolbarView::setAxisRender);
	connect(pointCloudColorSet, &QAction::triggered, this, &ThreeDToolbarView::setPointCloudColor);
	connect(pointCloudSizeSet, &QAction::triggered, this, &ThreeDToolbarView::setPointSize);

	QMenu* Surround_menu = new QMenu("包围", menu_bar);

	QAction* AxisAlignedBounding = new QAction("AABB");
	QAction* OrientedBounding = new QAction("OBB");

	Surround_menu->addAction(AxisAlignedBounding);
	Surround_menu->addAction(OrientedBounding);

	menu_bar->addMenu(Surround_menu);

	connect(AxisAlignedBounding, &QAction::triggered, this, &ThreeDToolbarView::setAABB);
	connect(OrientedBounding, &QAction::triggered, this, &ThreeDToolbarView::setOBB);

	QMenu* Filter_menu = new QMenu("工具", menu_bar);

	//QAction* Gaussian_filter = new QAction("Gaussian filter");
	QAction* CrossSection = new QAction("Cross Section");

	//Filter_menu->addAction(Gaussian_filter);
	Filter_menu->addAction(CrossSection);

	menu_bar->addMenu(Filter_menu);

	connect(CrossSection, &QAction::triggered, this, &ThreeDToolbarView::setCrossSection);
}

void ThreeDToolbarView::refresh(ViewModel::updateMode)
{

}

void ThreeDToolbarView::setBackgroundColor()
{
	QColor col = QColorDialog::getColor(Qt::white, this);
	viewModel.lock()->setPointCloudBackgroundColor(col);
	viewModel.lock()->notified(ViewModel::BackgroundColor);
}

void ThreeDToolbarView::setAxisRender()
{
	axisRender = new CoordinateAxisRenderSetView();
	QString axis = axisRender->getAxis();
	if (axisRender->exec() == QDialog::Accepted) {}
	viewModel.lock()->setRenderAxis(axis);
	delete axisRender;
	viewModel.lock()->notified(ViewModel::AxisRender);
}

void ThreeDToolbarView::setPointCloudColor()
{
	QColor col = QColorDialog::getColor(Qt::white, this);
	viewModel.lock()->setPointCloudColor(col);
	viewModel.lock()->notified(ViewModel::PointCloudColor);
}

void ThreeDToolbarView::setPointSize()
{
	pointSize = new PointCloudPointSizeSetView();
	int point_size = pointSize->getSize();
	if (pointSize->exec() == QDialog::Accepted) {}
	viewModel.lock()->setPointCloudPointSize(point_size);
	delete pointSize;
	viewModel.lock()->notified(ViewModel::PointCloudPointSize);
}

void ThreeDToolbarView::setCrossSection()
{
}

void te::ThreeDToolbarView::setAABB()
{
}

void te::ThreeDToolbarView::setOBB()
{
}
