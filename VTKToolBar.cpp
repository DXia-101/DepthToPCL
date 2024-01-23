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

    QMenu* file_menu = new QMenu("文件", menu_bar);

    QAction* open_action = new QAction("读取");
    QAction* save_action = new QAction("保存");

    file_menu->addAction(open_action);
    file_menu->addAction(save_action);

    menu_bar->addMenu(file_menu);

    connect(open_action, &QAction::triggered, this, &VTKToolBar::LoadPointCloud);//读取文件
    connect(save_action, &QAction::triggered, this, &VTKToolBar::SavePointCloud);//保存文件

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

    connect(backgroundSet, &QAction::triggered, this, &VTKToolBar::BackgroundColorSettingAction);
    connect(CoordinateAxisRender, &QAction::triggered, this, &VTKToolBar::CoordinateAxisRenderingAction);
    connect(pointCloudColorSet, &QAction::triggered, this, &VTKToolBar::PointCloudColorSettingsAction);
    connect(pointCloudSizeSet, &QAction::triggered, this, &VTKToolBar::PointCloudPointSizeSettingsAction);

    QMenu* Surround_menu = new QMenu("包围", menu_bar);

    QAction* AxisAlignedBounding = new QAction("AABB");
    QAction* OrientedBounding = new QAction("OBB");

    Surround_menu->addAction(AxisAlignedBounding);
    Surround_menu->addAction(OrientedBounding);

    menu_bar->addMenu(Surround_menu);

    connect(AxisAlignedBounding, &QAction::triggered, vtkWidget, &VTKOpenGLNativeWidget::AxisAlignedBoundingBox);
    connect(OrientedBounding, &QAction::triggered, vtkWidget, &VTKOpenGLNativeWidget::OrientedBoundingBox);

    QMenu* Filter_menu = new QMenu("滤波", menu_bar);

    QAction* Gaussian_filter = new QAction("高斯滤波");
    QAction* Direct_filter = new QAction("直通滤波");

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

    if (fileName.isEmpty())
    {
        return;
    }

    vtkWidget->cloud->points.clear();
    vtkWidget->Frame_clicked_cloud->points.clear();
    vtkWidget->Point_clicked_cloud->points.clear();

    int currentDisplayImageWidth, currentDisplayImageHeight;

    if (fileName.endsWith("pcd"))
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *vtkWidget->cloud) == -1)
        {
            qDebug() << "Couldn't read pcd file  \n";
            return;
        }
    }
    else if (fileName.endsWith("tif") || fileName.endsWith("tiff")) {
        cv::Mat image = cv::imread(fileName.toStdString(), cv::IMREAD_UNCHANGED);
        if (image.empty()) {
            QMessageBox::warning(this, "Warning", "无法读取图像文件");
            return;
        }
        Transfer_Function::cvMat2Cloud(image, vtkWidget->cloud);
    }
    else {
        QMessageBox::warning(this, "Warning", "点云读取格式错误！");
    }

    //移除窗口点云
    //ClearAllMarkedContent();
    emit LoadingCompleted();
    vtkWidget->reRendering(vtkWidget->cloud->makeShared());
    vtkWidget->viewer->resetCameraViewpoint("cloud");
}

void VTKToolBar::SavePointCloud()
{
    QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply)"));
    if (vtkWidget->cloud->empty()) {
        return;
    }
    else {
        if (filename.isEmpty()) {
            return;
        }
        int return_status;
        if (filename.endsWith(".pcd", Qt::CaseInsensitive))
            return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *vtkWidget->cloud);
        else if (filename.endsWith(".ply", Qt::CaseInsensitive))
            return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *vtkWidget->cloud);
        else {
            filename.append(".pcd");
            return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *vtkWidget->cloud);
        }
        if (return_status != 0) {
            QString errorinfo = QString::fromStdString("Error writing point cloud" + filename.toStdString());
            QMessageBox::warning(this, "Warning", errorinfo);
            return;
        }
    }
}

void VTKToolBar::BackgroundColorSettingAction()
{
    dialog_colorselect = new pcl_view_select_color();
    QColor color = dialog_colorselect->getColor();

    vtkWidget->viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
    vtkWidget->m_renderWindow->Render();
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

    if (!vtkWidget->cloud->empty()) {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> render(vtkWidget->cloud, curaxis.toStdString());
        vtkWidget->viewer->updatePointCloud(vtkWidget->cloud, render, "cloud");
        vtkWidget->m_renderWindow->Render();
    }
    return;
}

void VTKToolBar::PointCloudColorSettingsAction()
{
    dialog_colorselect = new pcl_view_select_color();
    QColor color = dialog_colorselect->getColor();
    QColor temp;
    temp.setRgb(143, 153, 159, 255);
    if (!vtkWidget->cloud->empty() && (color != temp)) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>selected_color(vtkWidget->cloud, color.redF() * 255, color.greenF() * 255, color.blueF() * 255);
        vtkWidget->viewer->updatePointCloud(vtkWidget->cloud, selected_color, "cloud");
        vtkWidget->m_renderWindow->Render();
    }
    else {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>selected_color(vtkWidget->cloud, 255, 255, 255);
        vtkWidget->viewer->updatePointCloud(vtkWidget->cloud, selected_color, "cloud");
        vtkWidget->m_renderWindow->Render();
    }
    return;
}

void VTKToolBar::PointCloudPointSizeSettingsAction()
{
    pointsize_set_dialog = new PointCloud_PointSize_Set_Dialog();
    int point_size = pointsize_set_dialog->GetSize();
    if (pointsize_set_dialog->exec() == QDialog::Accepted) {}
    delete pointsize_set_dialog;

    for (int i = 0; i < 1; ++i) {
        vtkWidget->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "cloud");
    }

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