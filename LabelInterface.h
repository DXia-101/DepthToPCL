#pragma once

#include <QWidget>
#include "ui_LabelInterface.h"
#include <QTableWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class LabelInterfaceClass; };
QT_END_NAMESPACE

class LabelInterface : public QWidget
{
	Q_OBJECT

public:
	LabelInterface(QWidget *parent = nullptr);
	~LabelInterface();

public:
    QColor getSelectedRowFontColor();
	QString getSelectedRowCategory();

	void clearTableWidget();

	void addRowToTable(const QString& content, const QColor& fontColor);

	bool checkFirstColumn(const QString& searchString);

signals:
	void currentRowSelected(const QString& content, const QColor& fontColor);

protected slots:
	void on_addLabelButton_clicked();
	void on_deleteLabelButton_clicked();
	void addLabel(QString category);

	void handleSelectionChanged();

	void ColorSelect();
private:
	void InitInterface();

private:
	Ui::LabelInterfaceClass *ui;

public:
	QTableWidget* LabelWidget;
};
