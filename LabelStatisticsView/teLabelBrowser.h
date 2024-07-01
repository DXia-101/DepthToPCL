#pragma once

#include <QWidget>
#include "ui_teLabelBrowser.h"

#include <QTableWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class teLabelBrowserClass; };
QT_END_NAMESPACE

class QVBoxLayout;

class teLabelBrowser : public QWidget
{
	Q_OBJECT

public:
	teLabelBrowser(QWidget *parent = nullptr);
	~teLabelBrowser();

	void displayUIInWidget(QVBoxLayout* layout);
public:
	QColor getSelectedRowFontColor();
	QString getSelectedRowCategory();

	void clearTableWidget();

	void addRowToTable(const QString& content, const QColor& fontColor);

	bool checkFirstColumn(const QString& searchString);

	QColor getFontColorByFirstColumnValue(const QString& searchString);

signals:
	void sig_currentRowSelected(const QString& content, const QColor& fontColor);
	void sig_ColorChanged(const QColor& fontColor);
	void sig_StartMark();

public slots:
	/// <summary>
	/// 更新标记浏览器中标记的数目
	/// </summary>
	/// <param name="nameCounts">各标记的数目</param>
	void updateTrainWidget(QMap<QString, int>* nameCounts);
	/// <summary>
	/// 更新标记浏览器中测试标的数目
	/// </summary>
	/// <param name="nameCounts">各测试标的数目</param>
	void updateResultWidget(QMap<QString, int>* nameCounts);

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

	QTableWidget* LabelWidget;
	bool isStartMark = false;
};
