#pragma once

#include <QWidget>
#include "ui_DynamicLabel.h"
#include "pcl_function.h"
#include "teAiExTypes.h"
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

	QString GetLabel();
	QColor GetColor();
	void SetColor(QColor color);

	void setSelected(bool selected);
private slots:
	void on_colorSelectBtn_clicked();
	void setLabelName(QString tag);
protected:
public:
	te::AiInstSet LabelAiInstSet;

private:
	Ui::DynamicLabelClass *ui;
	
	QString label;
	QColor penColor;
	
};
