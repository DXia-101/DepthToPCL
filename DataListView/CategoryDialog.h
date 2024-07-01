#pragma once

#include <QDialog>
#include "ui_CategoryDialog.h"

QT_BEGIN_NAMESPACE
namespace Ui { class CategoryDialogClass; };
QT_END_NAMESPACE

class CategoryDialog : public QDialog
{
	Q_OBJECT

public:
	CategoryDialog(QWidget *parent = nullptr);
	~CategoryDialog();

protected slots:
	void on_okButton_clicked();
	void on_deleteButton_clicked();

signals:
	void CategoryName(QString category);

private:
	Ui::CategoryDialogClass *ui;
};
