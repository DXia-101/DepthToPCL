#include "_3DMenuInterface.h"

_3DMenuInterface::_3DMenuInterface(VTKOpenGLNativeWidget* vtkWidget, QWidget *parent)
	: vtkWidget(vtkWidget), QWidget(parent)
	, ui(new Ui::_3DMenuInterfaceClass())
{
	ui->setupUi(this);
	isKeyDown_shift = false;
}

_3DMenuInterface::~_3DMenuInterface()
{
	delete ui;
}

/// <summary>
/// 高度系数的控制
/// </summary>
void _3DMenuInterface::on_ConfirmTransformationBtn_clicked()
{
	int factor = ui->HeightCoefficientSpinBox->value();
	vtkWidget->PointCloudHeightTransform(factor);
}

void _3DMenuInterface::on_ViewYBtn_clicked()
{
	vtkWidget->ViewYBtn();
}

void _3DMenuInterface::on_ViewXBtn_clicked()
{
	vtkWidget->ViewXBtn();
}

void _3DMenuInterface::on_ViewZBtn_clicked()
{
	vtkWidget->ViewZBtn();
}

void _3DMenuInterface::on_FramePickBtn_clicked()
{
	keybd_event('X', 0, 0, 0);
	keybd_event('X', 0, KEYEVENTF_KEYUP, 0);
}

void _3DMenuInterface::on_PointPickBtn_clicked()
{
	if (!isKeyDown_shift) {
		keybd_event(16, 0, 0, 0);
		isKeyDown_shift = true;
	}
	else {
		keybd_event(16, 0, KEYEVENTF_KEYUP, 0);
		isKeyDown_shift = false;
	}
}

void _3DMenuInterface::on_ComfirmFramePickBtn_clicked()
{
	vtkWidget->ComfirmFramePick();
}

void _3DMenuInterface::on_ComfirmPointPickBtn_clicked()
{
	vtkWidget->ComfirmPointPick();
}
