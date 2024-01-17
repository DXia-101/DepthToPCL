#include "DynamicLabel.h"

#include <QColorDialog>

DynamicLabel::DynamicLabel(QString tag, QWidget *parent)
	: label(tag),QWidget(parent)
	, ui(new Ui::DynamicLabelClass())
{
	ui->setupUi(this);
	penColor = Qt::white;
	connect(ui->LabelNameEdit, &QLineEdit::textEdited, this, &DynamicLabel::setLabelName);
}

DynamicLabel::~DynamicLabel()
{
	delete ui;
}

void DynamicLabel::PushCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ> tempcloud;
	pcl::copyPointCloud(*cloud, tempcloud);
	clouds.emplace_back(tempcloud);
}

void DynamicLabel::ClearCloudVector()
{
	clouds.clear();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DynamicLabel::GetBack()
{
	return clouds.back().makeShared();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DynamicLabel::CloudAt(int index)
{
	return clouds.at(index).makeShared();
}

int DynamicLabel::GetCloudSize()
{
	return clouds.size();
}

QString DynamicLabel::GetLabel()
{
	return this->label;
}

QColor DynamicLabel::GetColor()
{
	return penColor;
}

void DynamicLabel::setLabelName(QString tag)
{
	this->label = tag;
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