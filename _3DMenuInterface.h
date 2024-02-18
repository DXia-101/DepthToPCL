#pragma once

#include <QWidget>
#include "ui__3DMenuInterface.h"
#include "VTKOpenGLNativeWidget.h"

QT_BEGIN_NAMESPACE
namespace Ui { class _3DMenuInterfaceClass; };
QT_END_NAMESPACE

class _3DMenuInterface : public QWidget
{
	Q_OBJECT

public:
	_3DMenuInterface(VTKOpenGLNativeWidget* vtkWidget, QWidget *parent = nullptr);
	~_3DMenuInterface();

public slots:
    void on_ConfirmTransformationBtn_clicked();
    
    void on_ViewYBtn_clicked();
    void on_ViewXBtn_clicked();
    void on_ViewZBtn_clicked();

    void on_FramePickBtn_clicked();
    void on_PointPickBtn_clicked();

    void on_ComfirmFramePickBtn_clicked();
    void on_ComfirmPointPickBtn_clicked();

private:
	Ui::_3DMenuInterfaceClass *ui;
    VTKOpenGLNativeWidget* vtkWidget;
    bool isKeyDown_shift; //模拟shift是否按下
};
