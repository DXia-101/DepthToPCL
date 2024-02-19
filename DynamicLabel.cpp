#include "DynamicLabel.h"

#include <QColorDialog>
#include <QPainter>

DynamicLabel::DynamicLabel(QString tag, QWidget *parent)
	: label(tag),QWidget(parent)
	, ui(new Ui::DynamicLabelClass())
{
	ui->setupUi(this);
	ui->LabelNameEdit->setText(tag);
	penColor = Qt::white;
	ui->label->setStyleSheet("background-color: " + penColor.name() + ";");
	connect(ui->LabelNameEdit, &QLineEdit::textEdited, this, &DynamicLabel::setLabelName);
	setAutoFillBackground(true);
	

	installEventFilter(this);
}

DynamicLabel::~DynamicLabel()
{
	delete ui;
}

QString DynamicLabel::GetLabel()
{
	return this->label;
}

QColor DynamicLabel::GetColor()
{
	return penColor;
}

void DynamicLabel::SetColor(QColor color)
{
	penColor = color;
	ui->label->setStyleSheet("background-color: " + penColor.name() + ";");
}

void DynamicLabel::setSelected(bool selected)
{
	QPalette palette = this->palette();
	palette.setColor(QPalette::Background, selected ? Qt::red : Qt::white);
	setPalette(palette);
}

void DynamicLabel::setLabelName(QString tag)
{
	this->label = tag;
}

void DynamicLabel::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	painter.setPen(Qt::DashLine);
	painter.drawRect(rect().adjusted(0, 0, -1, -1));  // »æÖÆÐéÏß¿ò
	QWidget::paintEvent(event);
}

void DynamicLabel::on_colorSelectBtn_clicked() 
{
	penColor = QColorDialog::getColor(Qt::white, this);
	if (!penColor.isValid()) {
		return;
	}
	else {
		ui->label->setStyleSheet("background-color: " + penColor.name() + ";");
		//QRgb rgb = penColor.rgb();
		//qDebug() << "rgb== " << qRed(rgb) << qGreen(rgb) << qBlue(rgb);

		//qDebug() << "m_winColor== " << penColor.name();
	}
}