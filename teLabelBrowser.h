#pragma once

#include <QWidget>
#include "ui_teLabelBrowser.h"

#include <QTableWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class teLabelBrowserClass; };
QT_END_NAMESPACE

class teLabelBrowser : public QWidget
{
	Q_OBJECT

public:
	teLabelBrowser(QWidget *parent = nullptr);
	~teLabelBrowser();
	QTableWidget* LabelWidget;
public:
	QColor getSelectedRowFontColor();
	QString getSelectedRowCategory();

	void clearTableWidget();

	void addRowToTable(const QString& content, const QColor& fontColor);

	bool checkFirstColumn(const QString& searchString);

	QColor getFontColorByFirstColumnValue(const QString& searchString);

signals:
	void sig_currentRowSelected(const QString& content, const QColor& fontColor);
	void sig_ColorChanged();

protected slots:
	void on_addLabelButton_clicked();
	void on_deleteLabelButton_clicked();
	void addLabel(QString category);

	void handleSelectionChanged();

	void ColorSelect();
private:
	void InitInterface();
	void SendCurrentItemInfo(QTableWidgetItem* item);
	void saveTableWidget();
	void loadTableWidget();

private:
	Ui::teLabelBrowserClass *ui;

	
};
