#pragma once

#include <QDialog>
#include "ui_Filter_Direct.h"

class Filter_Direct : public QDialog
{
	Q_OBJECT

public:
	Filter_Direct(QWidget *parent = nullptr);
	~Filter_Direct();

private slots:
	void on_cancelButton_clicked();
	void on_okButton_clicked();
	void on_XradioButton_clicked();
	void on_YradioButton_clicked();
	void on_ZradioButton_clicked();
	void on_YesRadioButton_clicked();
	void on_NoRadioButton_clicked();

signals:
	void sendData(QString data1, QString data2, QString data3, QString data4);

private:
	Ui::Filter_DirectClass ui;

	QString axis, isSave;
};
