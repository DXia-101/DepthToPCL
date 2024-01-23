#pragma once

#include <QWidget>
#include "ui_VTKToolBar.h"
#include "VTKOpenGLNativeWidget.h"
#include "pcl_function.h"
#include "pcl_view_select_color.h"
#include "View_Render.h"
#include "PointCloud_PointSize_Set_Dialog.h"
#include "Filter_Guass.h"
#include "Filter_Direct.h"

QT_BEGIN_NAMESPACE
namespace Ui { class VTKToolBarClass; };
QT_END_NAMESPACE

class VTKToolBar : public QWidget
{
	Q_OBJECT

public:
	VTKToolBar(VTKOpenGLNativeWidget* vtkWidget,QWidget *parent = nullptr);
	~VTKToolBar();
	
public:
	void InterfaceInit();

public:
	/// <summary>
	/// ��ȡ����
	/// </summary>
	void LoadPointCloud();

	/// <summary>
	/// �������
	/// </summary>
	void SavePointCloud();

	/// <summary>
	/// //������ɫ����
	/// </summary>
	void BackgroundColorSettingAction();

	/// <summary>
	/// ��������Ⱦ
	/// </summary>
	void CoordinateAxisRenderingAction();

	/// <summary>
	/// ������ɫ����
	/// </summary>
	void PointCloudColorSettingsAction();

	/// <summary>
	/// ���Ƶ��С����
	/// </summary>
	void PointCloudPointSizeSettingsAction();

	/// <summary>
	/// ��˹�˲�
	/// </summary>
	void GaussFilterAction();
	
	/// <summary>
	/// ֱͨ�˲�
	/// </summary>
	void DirectFilterAction();

signals:
	void LoadingCompleted();

private:
	Ui::VTKToolBarClass *ui;
	VTKOpenGLNativeWidget* vtkWidget;

	pcl_view_select_color* dialog_colorselect;
	View_Render* dialog_render;
	PointCloud_PointSize_Set_Dialog* pointsize_set_dialog;

	Filter_Guass* dialog_Guass_filter;
	Filter_Direct* dialog_Direct_filter;
};
