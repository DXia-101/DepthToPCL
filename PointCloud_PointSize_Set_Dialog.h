#pragma once

#include <QDialog>
#include "ui_PointCloud_PointSize_Set_Dialog.h"

class PointCloud_PointSize_Set_Dialog : public QDialog
{
	Q_OBJECT

public:
	PointCloud_PointSize_Set_Dialog(QWidget *parent = nullptr);
	~PointCloud_PointSize_Set_Dialog();

public:
	int GetSize();

signals:
	void sendData(QString data);

private slots:
	void on_buttonBox_accepted();
	void on_PointSize_SpinBox_valueChanged(int value);
	void on_PointSizeHSlider_valueChanged(int value);

private:
	Ui::PointCloud_PointSize_Set_DialogClass ui;
	int pointCloudSize;
};
