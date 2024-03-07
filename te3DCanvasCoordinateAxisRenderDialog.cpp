#include "te3DCanvasCoordinateAxisRenderDialog.h"

te3DCanvasCoordinateAxisRenderDialog::te3DCanvasCoordinateAxisRenderDialog(QDialog*parent)
	: QDialog(parent)
	, ui(new Ui::te3DCanvasCoordinateAxisRenderDialogClass())
{
	ui->setupUi(this);
	this->show();
}

te3DCanvasCoordinateAxisRenderDialog::~te3DCanvasCoordinateAxisRenderDialog()
{
	delete ui;
}

QString te3DCanvasCoordinateAxisRenderDialog::GetAxis()
{
	return axis;
}

void te3DCanvasCoordinateAxisRenderDialog::on_XRadioButton_clicked()
{
	if (ui->XRadioButton->isChecked()) {
		ui->YRadioButton->setChecked(false);
		ui->ZRadioButton->setChecked(false);
		axis = "x";
	}
}

void te3DCanvasCoordinateAxisRenderDialog::on_YRadioButton_clicked()
{
	if (ui->YRadioButton->isChecked()) {
		ui->XRadioButton->setChecked(false);
		ui->ZRadioButton->setChecked(false);
		axis = "y";
	}
}

void te3DCanvasCoordinateAxisRenderDialog::on_ZRadioButton_clicked()
{
	if (ui->ZRadioButton->isChecked()) {
		ui->YRadioButton->setChecked(false);
		ui->XRadioButton->setChecked(false);
		axis = "z";
	}
}

void te3DCanvasCoordinateAxisRenderDialog::on_cancelButton_clicked()
{
	this->close();
}

void te3DCanvasCoordinateAxisRenderDialog::on_okButton_clicked()
{
	emit sig_CoordinateAxisRender(axis);
	this->close();
}
