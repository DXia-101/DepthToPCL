#include "Filter_Guass.h"

Filter_Guass::Filter_Guass(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

Filter_Guass::~Filter_Guass()
{}

void Filter_Guass::on_cancelButton_clicked()
{
	this->close();
}

void Filter_Guass::on_okButton_clicked()
{
	emit sendData(ui.lineEdit->text(), ui.lineEdit_2->text(), ui.lineEdit_3->text(), ui.lineEdit_4->text());
	this->close();
}