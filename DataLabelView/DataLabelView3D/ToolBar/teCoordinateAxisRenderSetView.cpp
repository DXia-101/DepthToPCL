#include "teCoordinateAxisRenderSetView.h"

using namespace te;

CoordinateAxisRenderSetView::CoordinateAxisRenderSetView(QWidget* parent)
	: QDialog(parent)
	, ui(new Ui::CoordinateAxisRenderSetViewClass())
{
	ui->setupUi(this);
}

CoordinateAxisRenderSetView::~CoordinateAxisRenderSetView()
{
	delete ui;
}

QString te::CoordinateAxisRenderSetView::getAxis()
{
	return axis;
}

void te::CoordinateAxisRenderSetView::on_XRadioButton_clicked()
{
	if (ui->XRadioButton->isChecked()) {
		ui->YRadioButton->setChecked(false);
		ui->ZRadioButton->setChecked(false);
		axis = "x";
	}
}

void te::CoordinateAxisRenderSetView::on_YRadioButton_clicked()
{
	if (ui->YRadioButton->isChecked()) {
		ui->XRadioButton->setChecked(false);
		ui->ZRadioButton->setChecked(false);
		axis = "y";
	}
}

void te::CoordinateAxisRenderSetView::on_ZRadioButton_clicked()
{
	if (ui->ZRadioButton->isChecked()) {
		ui->YRadioButton->setChecked(false);
		ui->XRadioButton->setChecked(false);
		axis = "z";
	}
}

void te::CoordinateAxisRenderSetView::on_cancelButton_clicked()
{
	this->close();
}

void te::CoordinateAxisRenderSetView::on_okButton_clicked()
{
	this->close();
}
