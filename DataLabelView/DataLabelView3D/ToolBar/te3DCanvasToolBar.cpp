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
    InitInterface();
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

    connect(backgroundSet, &QAction::triggered, this, &te3DCanvasToolBar::BackgroundColorSelect);
    connect(CoordinateAxisRender, &QAction::triggered, this, &te3DCanvasToolBar::CoordinateAxisSelect);
    connect(pointCloudColorSet, &QAction::triggered, this, &te3DCanvasToolBar::PointCloudColorSelect);
    connect(pointCloudSizeSet, &QAction::triggered, this, &te3DCanvasToolBar::PointCloudPointSizeSelect);

    QMenu* Surround_menu = new QMenu("包围", menu_bar);

    QAction* AxisAlignedBounding = new QAction("AABB");
    QAction* OrientedBounding = new QAction("OBB");

    Surround_menu->addAction(AxisAlignedBounding);
    Surround_menu->addAction(OrientedBounding);

    menu_bar->addMenu(Surround_menu);

    connect(AxisAlignedBounding, &QAction::triggered, this, &te3DCanvasToolBar::sig_AABBSurrounding);
    connect(OrientedBounding, &QAction::triggered, this, &te3DCanvasToolBar::sig_OBBSurrounding);

    QMenu* Filter_menu = new QMenu("工具", menu_bar);

    //QAction* Gaussian_filter = new QAction("Gaussian filter");
    QAction* CrossSction = new QAction("Cross Section");

    //Filter_menu->addAction(Gaussian_filter);
    Filter_menu->addAction(CrossSction);

    menu_bar->addMenu(Filter_menu);

    connect(CrossSction, &QAction::triggered, this, &te3DCanvasToolBar::CrossSectionAction);
}

void te3DCanvasToolBar::MaintainCoordinateAxis()
{
    emit sig_CoordinateAxis(this->axis);
}

void te3DCanvasToolBar::BackgroundColorSelect()
{
    dialog_colorselect = new te3DCanvasPointCloudColorSelectDialog();
    QColor color = dialog_colorselect->getColor();

    emit sig_BackgroundColor(color);
}

void te3DCanvasToolBar::CoordinateAxisSelect()
{
    dialog_render = new te3DCanvasCoordinateAxisRenderDialog();
    connect(dialog_render, &te3DCanvasCoordinateAxisRenderDialog::sig_CoordinateAxisRender, this, &te3DCanvasToolBar::sig_CoordinateAxis);
    connect(dialog_render, &te3DCanvasCoordinateAxisRenderDialog::sig_CoordinateAxisRender, this, &te3DCanvasToolBar::SaveAxis);
    if (dialog_render->exec() == QDialog::Accepted) {}
    delete dialog_render;
}

void te3DCanvasToolBar::PointCloudColorSelect()
{
    dialog_colorselect = new te3DCanvasPointCloudColorSelectDialog();
    QColor color = dialog_colorselect->getColor();
    delete dialog_colorselect;

    emit sig_PointCloudColor(color);
}

void te3DCanvasToolBar::PointCloudPointSizeSelect()
{
    pointsize_set_dialog = new PointCloud_PointSize_Set_Dialog();
    int point_size = pointsize_set_dialog->GetSize();
    if (pointsize_set_dialog->exec() == QDialog::Accepted) {}
    delete pointsize_set_dialog;

    emit sig_PointCloudPointSize(point_size);
}

void te3DCanvasToolBar::CrossSectionAction()
{
    dialog_CrossSection = new CrossSection();
    connect(dialog_CrossSection, &CrossSection::sendData, this, &te3DCanvasToolBar::sig_CrossSection);
    if (dialog_CrossSection->exec() == QDialog::Accepted) {}
    delete dialog_CrossSection;
}

void te3DCanvasToolBar::SaveAxis(QString axis)
{
    this->axis = axis;
}
