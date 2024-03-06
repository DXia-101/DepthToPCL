#include "teLabelBrowser.h"
#include "CategoryDialog.h"

#include <QDialog>
#include <QLabel>
#include <QLineEdit>
#include <QColorDialog>
#include <QHeaderView>

teLabelBrowser::teLabelBrowser(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::teLabelBrowserClass())
{
	ui->setupUi(this);
	InitInterface();
}

teLabelBrowser::~teLabelBrowser()
{
	delete ui;
}

void teLabelBrowser::InitInterface()
{
	LabelWidget = new QTableWidget();
	ui->TableWidgetLayout->addWidget(LabelWidget);

	LabelWidget->setColumnCount(4);
	LabelWidget->setHorizontalHeaderLabels({ u8"类别", u8"标记", u8"找到", u8"匹配" });
	LabelWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	LabelWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
	LabelWidget->verticalHeader()->setVisible(false);
	LabelWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

	QWidget* parentWidget = LabelWidget->parentWidget();
	int parentWidth = parentWidget->width(); // 获取父窗口的宽度

	int columnCount = LabelWidget->columnCount();
	int columnWidth = parentWidth / (columnCount + 1);

	for (int column = 0; column < columnCount; ++column) {
		LabelWidget->setColumnWidth(column, columnWidth);
		LabelWidget->horizontalHeader()->setSectionResizeMode(column, QHeaderView::Fixed);
	}

	LabelWidget->insertRow(0);
	LabelWidget->setItem(0, 0, new QTableWidgetItem("LMask"));
	LabelWidget->setItem(0, 1, new QTableWidgetItem("0"));
	LabelWidget->setItem(0, 2, new QTableWidgetItem("0"));
	LabelWidget->setItem(0, 3, new QTableWidgetItem("0"));

	LabelWidget->insertRow(1);
	LabelWidget->setItem(1, 0, new QTableWidgetItem("GMask"));
	LabelWidget->setItem(1, 1, new QTableWidgetItem("0"));
	LabelWidget->setItem(1, 2, new QTableWidgetItem("0"));
	LabelWidget->setItem(1, 3, new QTableWidgetItem("0"));

	LabelWidget->insertRow(2);
	LabelWidget->setItem(2, 0, new QTableWidgetItem("BG"));
	LabelWidget->setItem(2, 1, new QTableWidgetItem("0"));
	LabelWidget->setItem(2, 2, new QTableWidgetItem("0"));
	LabelWidget->setItem(2, 3, new QTableWidgetItem("0"));

	QTableWidgetItem* currentItem = LabelWidget->item(0, 0);
	LabelWidget->setCurrentItem(currentItem);

	connect(LabelWidget, &QTableWidget::cellDoubleClicked, this, &teLabelBrowser::ColorSelect);
	connect(LabelWidget, &QTableWidget::itemSelectionChanged, this, &teLabelBrowser::handleSelectionChanged);
}

QColor teLabelBrowser::getSelectedRowFontColor()
{
	if (LabelWidget->selectedItems().isEmpty()) {
		return QColor();
	}

	int row = LabelWidget->selectedItems().first()->row();
	QTableWidgetItem* item = LabelWidget->item(row, 0);
	if (item) {
		QColor fontColor = item->foreground().color();
		return fontColor;
	}

	return QColor();
}

QString teLabelBrowser::getSelectedRowCategory()
{
	if (LabelWidget->selectedItems().isEmpty()) {
		return QString();
	}
	int row = LabelWidget->selectedItems().first()->row();
	QString content = LabelWidget->item(row, 0)->text();

	return content;
}

void teLabelBrowser::clearTableWidget()
{
	int rowCount = LabelWidget->rowCount();

	for (int row = rowCount - 1; row >= 3; --row) {
		LabelWidget->removeRow(row);
	}
}

void teLabelBrowser::addRowToTable(const QString& content, const QColor& fontColor)
{
	if (checkFirstColumn(content)) {
		return;
	}
	int row = LabelWidget->rowCount();
	LabelWidget->insertRow(row);

	QTableWidgetItem* itemContent = new QTableWidgetItem(content);
	itemContent->setForeground(fontColor);

	LabelWidget->setItem(row, 0, itemContent);
}

bool teLabelBrowser::checkFirstColumn(const QString& searchString)
{
	for (int row = 0; row < LabelWidget->rowCount(); ++row) {
		QTableWidgetItem* item = LabelWidget->item(row, 0);

		if (item && item->text() == searchString) {
			return true;
		}
	}
	return false;
}

QColor teLabelBrowser::getFontColorByFirstColumnValue(const QString& searchString)
{
	for (int row = 0; row < LabelWidget->rowCount(); ++row) {
		QTableWidgetItem* item = LabelWidget->item(row, 0);

		if (item && item->text() == searchString) {
			return item->foreground().color();
		}
	}
	return QColor(Qt::black);
}

void teLabelBrowser::on_addLabelButton_clicked()
{
	CategoryDialog* dialog = new CategoryDialog();
	connect(dialog, &CategoryDialog::CategoryName, this, &teLabelBrowser::addLabel);
	dialog->exec();
}

void teLabelBrowser::on_deleteLabelButton_clicked()
{
	int row = LabelWidget->currentRow();
	if (row >= 0) {
		LabelWidget->removeRow(row);
	}
}

void teLabelBrowser::addLabel(QString category)
{
	addRowToTable(category, Qt::black);
}

void teLabelBrowser::handleSelectionChanged()
{
	if (!LabelWidget->selectedItems().isEmpty()) {
		int row = LabelWidget->selectedItems().first()->row();
		QTableWidgetItem* item = LabelWidget->item(row, 0); // 第一列的索引为0

		if (item) {
			QString content = item->text();
			QColor fontColor = item->foreground().color();
			emit sig_currentRowSelected(content, fontColor);
		}
	}
}

void teLabelBrowser::ColorSelect()
{
	QTableWidgetItem* item = LabelWidget->currentItem();
	if (item) {
		QColorDialog colorDialog;
		if (colorDialog.exec() == QDialog::Accepted) {
			QColor selectedColor = colorDialog.selectedColor();
			item->setForeground(QBrush(selectedColor));
			QString content = item->text();
			QColor fontColor = item->foreground().color();
			emit sig_currentRowSelected(content, fontColor);
		}
	}
}


