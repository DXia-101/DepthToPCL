#include "ImageDisplayToolBar.h"

#pragma execution_character_set("utf-8")

#include <QHBoxLayout>
#include <QMenuBar>
#include <QFileDialog>
#include <QMessageBox>

ImageDisplayToolBar::ImageDisplayToolBar(ImageLabel* imagelabel,QWidget *parent)
	: m_imageLabel(imagelabel),QWidget(parent)
	, ui(new Ui::ImageDisplayToolBarClass())
{
	ui->setupUi(this);
	InterfaceInit();
	connect(this, &ImageDisplayToolBar::ShapeSelected, m_imageLabel, &ImageLabel::ShapeSelect);
}

ImageDisplayToolBar::~ImageDisplayToolBar()
{
	delete ui;
}

void ImageDisplayToolBar::InterfaceInit()
{
	QMenuBar* menu_bar = new QMenuBar(this);
	ui->ToolBarLayout->addWidget(menu_bar);
	menu_bar->setStyleSheet("font-size : 18px");
}

void ImageDisplayToolBar::on_ShapeCBox_currentTextChanged(const QString& arg1) 
{
	emit ShapeSelected(arg1);
}