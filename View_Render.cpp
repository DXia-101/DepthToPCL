#include "View_Render.h"

View_Render::View_Render(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

View_Render::~View_Render()
{}

QString View_Render::GetAxis()
{
	return axis;
}

void View_Render::on_XRadioButton_clicked()
{
	if (ui.XRadioButton->isChecked()) {
		ui.YRadioButton->setChecked(false);
		ui.ZRadioButton->setChecked(false);
		axis = "x";
	}
}

void View_Render::on_YRadioButton_clicked()
{
	if (ui.YRadioButton->isChecked()) {
		ui.XRadioButton->setChecked(false);
		ui.ZRadioButton->setChecked(false);
		axis = "y";
	}
}

void View_Render::on_ZRadioButton_clicked()
{
	if (ui.ZRadioButton->isChecked()) {
		ui.YRadioButton->setChecked(false);
		ui.XRadioButton->setChecked(false);
		axis = "z";
	}
}

void View_Render::on_cancelButton_clicked()
{
	emit determine();
	this->close();
}

void View_Render::on_okButton_clicked()
{
	
	this->close();
}
