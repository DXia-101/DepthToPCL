#include "te3DCanvasMenu.h"

te3DCanvasMenu::te3DCanvasMenu(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::te3DCanvasMenuClass())
{
	ui->setupUi(this);
}

te3DCanvasMenu::~te3DCanvasMenu()
{
	delete ui;
}

bool te3DCanvasMenu::isDimensionShow()
{
	return ui->showGTCheckBox->isChecked();
}

bool te3DCanvasMenu::isResultShow()
{
	return ui->showRSTcheckBox->isChecked();
}

void te3DCanvasMenu::on_ConfirmTransformationBtn_clicked()
{
	int factor = ui->HeightCoefficientSpinBox->value();
	emit sig_HeightTransform(factor);
	emit sig_ConnectHeightTransForm();
}

void te3DCanvasMenu::on_reductionBtn_clicked()
{
	emit sig_DisconnectHeightTransForm();
}

void te3DCanvasMenu::on_ViewYBtn_clicked()
{
	emit sig_PerspectiveToYaxis();
}

void te3DCanvasMenu::on_ViewXBtn_clicked()
{
	emit sig_PerspectiveToXaxis();
}

void te3DCanvasMenu::on_ViewZBtn_clicked()
{
	emit sig_PerspectiveToZaxis();
}

void te3DCanvasMenu::on_startMarkBtn_clicked()
{
	if (ui->startMarkBtn->isChecked()) {
		ui->startMarkBtn->setChecked(true);
		ui->startMarkBtn->setText(u8"完成标记");
	}
	else {
		ui->startMarkBtn->setChecked(false);
		ui->startMarkBtn->setText(u8"开始标记");
	}
	emit sig_StartMarking();
}

void te3DCanvasMenu::on_showGTCheckBox_stateChanged(int arg)
{
	emit sig_GtCheckStateChanged(arg);
}

void te3DCanvasMenu::on_showRSTcheckBox_stateChanged(int arg)
{
	emit sig_RSTCheckStateChanged(arg);
}

void te3DCanvasMenu::setHeightCoefficientFactor(int factor)
{
	ui->HeightCoefficientSpinBox->setValue(factor);
}
