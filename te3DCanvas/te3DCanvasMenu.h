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

public:
	bool isDimensionShow();
	bool isResultShow();

public slots:
	void on_ConfirmTransformationBtn_clicked();
	void on_reductionBtn_clicked();

	void on_ViewYBtn_clicked();
	void on_ViewXBtn_clicked();
	void on_ViewZBtn_clicked();

	void on_startMarkBtn_clicked();

	void on_showGTCheckBox_stateChanged(int arg);
	void on_showRSTcheckBox_stateChanged(int arg);

	void setHeightCoefficientFactor(int factor);
signals:
	void sig_HeightTransform(int facetor);
	void sig_PerspectiveToXaxis();
	void sig_PerspectiveToYaxis();
	void sig_PerspectiveToZaxis();
	void sig_StartMarking();
	void sig_GtCheckStateChanged(int arg);
	void sig_RSTCheckStateChanged(int arg);
	void sig_ConnectHeightTransForm();
	void sig_DisconnectHeightTransForm();
	void sig_CoordinateAxisRender();
private:
	Ui::te3DCanvasMenuClass *ui;
};
