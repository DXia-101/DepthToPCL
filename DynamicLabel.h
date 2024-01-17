#pragma once

#include <QWidget>
#include "ui_DynamicLabel.h"
#include "pcl_function.h"
#include "Configure.h"
#include "GraphicsPolygonItem.h"
#include <vector>

QT_BEGIN_NAMESPACE
namespace Ui { class DynamicLabelClass; };
QT_END_NAMESPACE

class DynamicLabel : public QWidget
{
	Q_OBJECT

public:
	DynamicLabel(QString tag,QWidget *parent = nullptr);
	~DynamicLabel();

	void PushCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void ClearCloudVector();

	pcl::PointCloud<pcl::PointXYZ>::Ptr GetBack();
	pcl::PointCloud<pcl::PointXYZ>::Ptr CloudAt(int index);

	int GetCloudSize();

	QString GetLabel();
	QColor GetColor();

private slots:
	void on_colorSelectBtn_clicked();
	void setLabelName(QString tag);

public:
	QVector<GraphicsPolygonItem*> markedPolygons;
private:
	Ui::DynamicLabelClass *ui;
	
	std::vector<pcl::PointCloud<pcl::PointXYZ>> clouds;
	
	
	QString label;
	QColor penColor;
	
};
