#pragma once

#include <QWidget>
#include "ui_te3DCanvasMenu.h"

QT_BEGIN_NAMESPACE
namespace Ui { class te3DCanvasMenuClass; };
QT_END_NAMESPACE

class te3DCanvasMenu : public QWidget
{
	Q_OBJECT

public:
	te3DCanvasMenu(QWidget *parent = nullptr);
	~te3DCanvasMenu();

public slots:
	void on_ConfirmTransformationBtn_clicked();

	void on_ViewYBtn_clicked();
	void on_ViewXBtn_clicked();
	void on_ViewZBtn_clicked();

	void on_startMarkBtn_clicked();

signals:
	void sig_HeightTransform(int facetor);
	void sig_PerspectiveToXaxis();
	void sig_PerspectiveToYaxis();
	void sig_PerspectiveToZaxis();
	void sig_StartMarking();
private:
	Ui::te3DCanvasMenuClass *ui;
};
