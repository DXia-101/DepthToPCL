#include "VTKToolBar.h"

#pragma execution_character_set("utf-8")

#include <QHBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QColor>
#include <QDebug>

#include "Transfer_Function.h"

VTKToolBar::VTKToolBar(VTKOpenGLNativeWidget* vtkWidget,QWidget *parent)
	: vtkWidget(vtkWidget),QWidget(parent)
	, ui(new Ui::VTKToolBarClass())
{
	ui->setupUi(this);
    InterfaceInit();
}

void VTKToolBar::InterfaceInit()
{
    QHBoxLayout* layout = new QHBoxLayout(this);
    QMenuBar* menu_bar = new QMenuBar(this);
    layout->addWidget(menu_bar);
    menu_bar->setStyleSheet("font-size : 18px");

    QMenu* file_menu = new QMenu("�ļ�", menu_bar);

    QAction* open_action = new QAction("��ȡ");
    QAction* save_action = new QAction("����");

    file_menu->addAction(open_action);
    file_menu->addAction(save_action);

    menu_bar->addMenu(file_menu);

    connect(open_action, &QAction::triggered, this, &VTKToolBar::LoadPointCloud);//��ȡ�ļ�
    connect(save_action, &QAction::triggered, this, &VTKToolBar::SavePointCloud);//�����ļ�

    QMenu* Display_menu = new QMenu("��ʾ", menu_bar);

    QAction* backgroundSet = new QAction("��������");
    QAction* CoordinateAxisRender = new QAction("������Ⱦ");
    QAction* pointCloudColorSet = new QAction("��ɫ����");
    QAction* pointCloudSizeSet = new QAction("���ƴ�С����");

    Display_menu->addAction(backgroundSet);
    Display_menu->addAction(CoordinateAxisRender);
    Display_menu->addAction(pointCloudColorSet);
    Display_menu->addAction(pointCloudSizeSet);

    menu_bar->addMenu(Display_menu);

    connect(backgroundSet, &QAction::triggered, this, &VTKToolBar::BackgroundColorSettingAction);
    connect(CoordinateAxisRender, &QAction::triggered, this, &VTKToolBar::CoordinateAxisRenderingAction);
    connect(pointCloudColorSet, &QAction::triggered, this, &VTKToolBar::PointCloudColorSettingsAction);
    connect(pointCloudSizeSet, &QAction::triggered, this, &VTKToolBar::PointCloudPointSizeSettingsAction);

    QMenu* Surround_menu = new QMenu("��Χ", menu_bar);

    QAction* AxisAlignedBounding = new QAction("AABB");
    QAction* OrientedBounding = new QAction("OBB");

    Surround_menu->addAction(AxisAlignedBounding);
    Surround_menu->addAction(OrientedBounding);

    menu_bar->addMenu(Surround_menu);

    connect(AxisAlignedBounding, &QAction::triggered, vtkWidget, &VTKOpenGLNativeWidget::AxisAlignedBoundingBox);
    connect(OrientedBounding, &QAction::triggered, vtkWidget, &VTKOpenGLNativeWidget::OrientedBoundingBox);

    QMenu* Filter_menu = new QMenu("�˲�", menu_bar);

    QAction* Gaussian_filter = new QAction("��˹�˲�");
    QAction* Direct_filter = new QAction("ֱͨ�˲�");

    Filter_menu->addAction(Gaussian_filter);
    Filter_menu->addAction(Direct_filter);

    menu_bar->addMenu(Filter_menu);

    connect(Gaussian_filter, &QAction::triggered, this, &VTKToolBar::GaussFilterAction);
    connect(Direct_filter, &QAction::triggered, this, &VTKToolBar::DirectFilterAction);
}

VTKToolBar::~VTKToolBar()
{
	delete ui;
}

void VTKToolBar::LoadPointCloud()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("open  file"),
        "", tr("pcb files(*.pcd *.ply *.tif *.tiff) ;;All files (*.*)"));
    if(vtkWidget->LoadPointCloud(fileName))
        emit LoadingCompleted();
    else
        QMessageBox::warning(this, "Warning", "����ʧ��");
}

void VTKToolBar::SavePointCloud()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply)"));
    if(!vtkWidget->SavePointCloud(filename))
        QMessageBox::warning(this, "Warning", "����ʧ��");
}

void VTKToolBar::BackgroundColorSettingAction()
{
    dialog_colorselect = new pcl_view_select_color();
    QColor color = dialog_colorselect->getColor();
    
    vtkWidget->SetBackgroundColor(color);
    return;
}

void VTKToolBar::CoordinateAxisRenderingAction()
{
    dialog_render = new View_Render();
    QString curaxis;
    connect(dialog_render, &View_Render::determine, this, [&](QString axis) {
        curaxis = axis;
        });
    if (dialog_render->exec() == QDialog::Accepted) {}
    delete dialog_render;

    vtkWidget->CoordinateAxisRendering(curaxis);
}

void VTKToolBar::PointCloudColorSettingsAction()
{
    dialog_colorselect = new pcl_view_select_color();
    QColor color = dialog_colorselect->getColor();
    delete dialog_colorselect;

    vtkWidget->PointCloudColorSet(color);
}

void VTKToolBar::PointCloudPointSizeSettingsAction()
{
    pointsize_set_dialog = new PointCloud_PointSize_Set_Dialog();
    int point_size = pointsize_set_dialog->GetSize();
    if (pointsize_set_dialog->exec() == QDialog::Accepted) {}
    delete pointsize_set_dialog;

    vtkWidget->PointCloudPointSizeSet(point_size);
}

void VTKToolBar::GaussFilterAction()
{
    dialog_Guass_filter = new Filter_Guass();
    connect(dialog_Guass_filter, &Filter_Guass::sendData, vtkWidget, &VTKOpenGLNativeWidget::GuassFilter);
    if (dialog_Guass_filter->exec() == QDialog::Accepted) {}
    delete dialog_Guass_filter;
}

void VTKToolBar::DirectFilterAction()
{
    dialog_Direct_filter = new Filter_Direct();
    connect(dialog_Direct_filter, &Filter_Direct::sendData, vtkWidget, &VTKOpenGLNativeWidget::DirectFilter);
    if (dialog_Direct_filter->exec() == QDialog::Accepted) {}
    delete dialog_Direct_filter;
}