#pragma once

#include <QDialog>
#include "ui_CrossSection.h"

class CrossSection : public QDialog
{
	Q_OBJECT

public:
	CrossSection(QWidget *parent = nullptr);
	~CrossSection();

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
	Ui::CrossSectionClass ui;

	QString axis, isSave;
	QString configFilePath;
};
