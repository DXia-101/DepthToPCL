#include "CrossSection.h"
#include <QSettings>
#include <QDir>

CrossSection::CrossSection(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	configFilePath = QDir::currentPath() + "/config.ini";

	QSettings settings(configFilePath, QSettings::IniFormat);
	ui.StartLineEdit->setText(settings.value("StartLineEdit").toString());
	ui.EndLineEdit->setText(settings.value("EndLineEdit").toString());
	axis = settings.value("Axis").toString();
	isSave = settings.value("IsSave").toString();

	if (axis == "x") {
		ui.XradioButton->setChecked(true);
		ui.YradioButton->setChecked(false);
		ui.ZradioButton->setChecked(false);
	}
	else if (axis == "y") {
		ui.XradioButton->setChecked(false);
		ui.YradioButton->setChecked(true);
		ui.ZradioButton->setChecked(false);
	}
	else if (axis == "z") {
		ui.XradioButton->setChecked(false);
		ui.YradioButton->setChecked(false);
		ui.ZradioButton->setChecked(true);
	}

	if (isSave == "0") {
		ui.YesRadioButton->setChecked(true);
		ui.NoRadioButton->setChecked(false);
	}
	else if (isSave == "1") {
		ui.YesRadioButton->setChecked(false);
		ui.NoRadioButton->setChecked(true);
	}
}

CrossSection::~CrossSection()
{}

void CrossSection::on_cancelButton_clicked()
{
	this->close();
}

void CrossSection::on_okButton_clicked()
{
	emit sendData(ui.StartLineEdit->text(), ui.EndLineEdit->text(), axis, isSave);
	QSettings settings(configFilePath, QSettings::IniFormat);
	settings.setValue("StartLineEdit", ui.StartLineEdit->text());
	settings.setValue("EndLineEdit", ui.EndLineEdit->text());
	settings.setValue("Axis", axis);
	settings.setValue("IsSave", isSave);
	this->close();
}

void CrossSection::on_XradioButton_clicked()
{
	if (ui.XradioButton->isChecked()) {
		ui.YradioButton->setChecked(false);
		ui.ZradioButton->setChecked(false);
		axis = "x";
	}
}

void CrossSection::on_YradioButton_clicked()
{
	if (ui.YradioButton->isChecked()) {
		ui.XradioButton->setChecked(false);
		ui.ZradioButton->setChecked(false);
		axis = "y";
	}
}

void CrossSection::on_ZradioButton_clicked()
{
	if (ui.ZradioButton->isChecked()) {
		ui.XradioButton->setChecked(false);
		ui.YradioButton->setChecked(false);
		axis = "z";
	}
}

void CrossSection::on_YesRadioButton_clicked()
{
	if (ui.YesRadioButton->isChecked()) {
		ui.NoRadioButton->setChecked(false);
		isSave = "0";
	}
}

void CrossSection::on_NoRadioButton_clicked()
{
	if (ui.NoRadioButton->isChecked()) {
		ui.YesRadioButton->setChecked(false);
		isSave = "1";
	}
}
