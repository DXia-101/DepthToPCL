#pragma once

#include <QWidget>
#include "ui_te3DCanvasCoordinateAxisRenderDialog.h"

QT_BEGIN_NAMESPACE
namespace Ui { class te3DCanvasCoordinateAxisRenderDialogClass; };
QT_END_NAMESPACE

class te3DCanvasCoordinateAxisRenderDialog : public QWidget
{
	Q_OBJECT

public:
	te3DCanvasCoordinateAxisRenderDialog(QWidget *parent = nullptr);
	~te3DCanvasCoordinateAxisRenderDialog();

private:
	Ui::te3DCanvasCoordinateAxisRenderDialogClass *ui;

public:
	QString GetAxis();

signals:
	void sig_CoordinateAxisRender(QString data);

private slots:
	void on_XRadioButton_clicked();
	void on_YRadioButton_clicked();
	void on_ZRadioButton_clicked();
	void on_cancelButton_clicked();
	void on_okButton_clicked();

private:
	QString axis;
};
