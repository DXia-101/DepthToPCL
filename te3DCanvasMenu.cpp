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

void te3DCanvasMenu::on_ConfirmTransformationBtn_clicked()
{
	int factor = ui->HeightCoefficientSpinBox->value();
	emit sig_HeightTransform(factor);
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
	emit sig_StartMarking();
}
