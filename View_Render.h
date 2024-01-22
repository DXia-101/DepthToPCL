#pragma once

#include <QDialog>
#include "ui_View_Render.h"

class View_Render : public QDialog
{
	Q_OBJECT

public:
	View_Render(QWidget *parent = nullptr);
	~View_Render();

public:
	QString GetAxis();

signals:
	void determine();

private slots:
	void on_XRadioButton_clicked();
	void on_YRadioButton_clicked();
	void on_ZRadioButton_clicked();
	void on_cancelButton_clicked();
	void on_okButton_clicked();

private:
	Ui::View_RenderClass ui;
	QString axis;
};
