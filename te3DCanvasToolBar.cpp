#include "te3DCanvasToolBar.h"

#pragma execution_character_set("utf-8")

#include <QHBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QColor>

te3DCanvasToolBar::te3DCanvasToolBar(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::te3DCanvasToolBarClass())
{
	ui->setupUi(this);
}

te3DCanvasToolBar::~te3DCanvasToolBar()
{
	delete ui;
}

void te3DCanvasToolBar::InitInterface()
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

    connect(backgroundSet, &QAction::triggered, this, &te3DCanvasToolBar::sig_BackgroundColorSetting);
    connect(CoordinateAxisRender, &QAction::triggered, this, &te3DCanvasToolBar::sig_CoordinateAxisRendering);
    connect(pointCloudColorSet, &QAction::triggered, this, &te3DCanvasToolBar::sig_PointCloudColorSetting);
    connect(pointCloudSizeSet, &QAction::triggered, this, &te3DCanvasToolBar::sig_PointCloudPointSizeSetting);

    QMenu* Surround_menu = new QMenu("包围", menu_bar);

    QAction* AxisAlignedBounding = new QAction("AABB");
    QAction* OrientedBounding = new QAction("OBB");

    Surround_menu->addAction(AxisAlignedBounding);
    Surround_menu->addAction(OrientedBounding);

    menu_bar->addMenu(Surround_menu);

    connect(AxisAlignedBounding, &QAction::triggered, this, &te3DCanvasToolBar::sig_AABBSurrounding);
    connect(OrientedBounding, &QAction::triggered, this, &te3DCanvasToolBar::sig_OBBSurrounding);

    QMenu* Filter_menu = new QMenu("滤波", menu_bar);

    QAction* Gaussian_filter = new QAction("高斯滤波");
    QAction* Direct_filter = new QAction("直通滤波");

    Filter_menu->addAction(Gaussian_filter);
    Filter_menu->addAction(Direct_filter);

    menu_bar->addMenu(Filter_menu);

    connect(Gaussian_filter, &QAction::triggered, this, &te3DCanvasToolBar::sig_GaussFilter);
    connect(Direct_filter, &QAction::triggered, this, &te3DCanvasToolBar::sig_DirectFilter);
}
