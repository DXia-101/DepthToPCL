#pragma once

#include <QWidget>
#include "ui_AssetBrowser.h"
#include "pcl_function.h"
#include "DisplayTableModel.h"
#include "NameDelegate.h"
#include "ImageDelegate.h"

QT_BEGIN_NAMESPACE
namespace Ui { class AssetBrowserClass; };
QT_END_NAMESPACE

class AssetBrowser : public QWidget
{
	Q_OBJECT

public:
	AssetBrowser(QWidget *parent = nullptr);
	~AssetBrowser();

protected:
	void InitTable();
	void setInsertRowData(int _row);
	bool isSelectionRows();

protected slots:
	void insertData();

private:
	Ui::AssetBrowserClass *ui;

	DisplayTableModel* m_DisplayTableModel;

	int m_no; //No.ап
	QModelIndexList m_showList;

	NameDelegate* m_NameDelegate;
	ImageDelegate* m_ImageDelegate;
};
