#include "te2DCanvasToolBar.h"

#pragma execution_character_set("utf-8")

#include <QHBoxLayout>
#include <QMenuBar>
#include <QFileDialog>

te2DCanvasToolBar::te2DCanvasToolBar(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::te2DCanvasToolBarClass())
{
	ui->setupUi(this);
	InitInterface();
}

te2DCanvasToolBar::~te2DCanvasToolBar()
{
	delete ui;
}

void te2DCanvasToolBar::InitInterface()
{
	QMenuBar* menu_bar = new QMenuBar(this);
	ui->ToolBarLayout->addWidget(menu_bar);
	menu_bar->setStyleSheet("font-size : 18px");
}

void te2DCanvasToolBar::on_redoButton_clicked()
{
	
}

void te2DCanvasToolBar::on_undoButton_clicked()
{
}

void te2DCanvasToolBar::on_eraserButton_clicked()
{
	if (!ui->eraserButton->isChecked()) {
		emit sig_te2DCanvasDrawStatus();
	}
	else {
		emit sig_te2DCanvasEraseStatus();
	}
}

void te2DCanvasToolBar::on_ShapeCBox_currentTextChanged(const QString& arg1)
{
	emit sig_te2DCanvasShapeSelected(arg1);
}

void te2DCanvasToolBar::on_DimensionCheckBox_stateChanged(int arg)
{
	emit sig_ShowDimension(arg);
}

void te2DCanvasToolBar::on_ResultCheckBox_stateChanged(int arg)
{
	emit sig_ShowResult(arg);
}

void te2DCanvasToolBar::on_LocalMaskCheckBox_stateChanged(int arg)
{
	emit sig_ShowLocalMask(arg);
}

void te2DCanvasToolBar::on_GlobalMaskCheckBox_stateChanged(int arg)
{
	emit sig_ShowGlobalMask(arg);
}
