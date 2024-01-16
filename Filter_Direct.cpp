#include "Filter_Direct.h"

Filter_Direct::Filter_Direct(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

Filter_Direct::~Filter_Direct()
{}

void Filter_Direct::on_cancelButton_clicked()
{
	this->close();
}

void Filter_Direct::on_okButton_clicked()
{
	emit sendData(ui.StartLineEdit->text(), ui.EndLineEdit->text(), axis, isSave);
	this->close();
}

void Filter_Direct::on_XradioButton_clicked()
{
	if (ui.XradioButton->isChecked()) {
		ui.YradioButton->setChecked(false);
		ui.ZradioButton->setChecked(false);
		axis = "x";
	}
}

void Filter_Direct::on_YradioButton_clicked()
{
	if (ui.YradioButton->isChecked()) {
		ui.XradioButton->setChecked(false);
		ui.ZradioButton->setChecked(false);
		axis = "y";
	}
}

void Filter_Direct::on_ZradioButton_clicked()
{
	if (ui.ZradioButton->isChecked()) {
		ui.XradioButton->setChecked(false);
		ui.YradioButton->setChecked(false);
		axis = "z";
	}
}

void Filter_Direct::on_YesRadioButton_clicked()
{
	if (ui.YesRadioButton->isChecked()) {
		ui.NoRadioButton->setChecked(false);
		isSave = "0";
	}
}

void Filter_Direct::on_NoRadioButton_clicked()
{
	if (ui.NoRadioButton->isChecked()) {
		ui.YesRadioButton->setChecked(false);
		isSave = "1";
	}
}
