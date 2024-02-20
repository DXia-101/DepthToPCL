#include "CategoryDialog.h"

CategoryDialog::CategoryDialog(QWidget *parent)
	: QDialog(parent)
	, ui(new Ui::CategoryDialogClass())
{
	ui->setupUi(this);
}

CategoryDialog::~CategoryDialog()
{
	delete ui;
}

void CategoryDialog::on_okButton_clicked()
{
	if (!ui->categoryLineEdit->text().isEmpty()) {
		emit CategoryName(ui->categoryLineEdit->text());
	}
	this->close();
}

void CategoryDialog::on_deleteButton_clicked()
{
	this->close();
}
