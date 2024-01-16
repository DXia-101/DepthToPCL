#pragma once

#include <QDialog>
#include "ui_Filter_Guass.h"

class Filter_Guass : public QDialog
{
	Q_OBJECT

public:
	Filter_Guass(QWidget *parent = nullptr);
	~Filter_Guass();
signals:
	void sendData(QString data1,QString data2,QString data3,QString data4);

private slots:
	void on_cancelButton_clicked();
	void on_okButton_clicked();
private:
	Ui::Filter_GuassClass ui;
};
