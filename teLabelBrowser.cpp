#include "teLabelBrowser.h"
#include "CategoryDialog.h"

#include <QAbstractItemDelegate>
#include <QColorDialog>
#include <QDialog>
#include <QFile>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QTextStream>

teLabelBrowser::teLabelBrowser(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::teLabelBrowserClass())
{
	ui->setupUi(this);
	InitInterface();
	loadTableWidget();
}

teLabelBrowser::~teLabelBrowser()
{
	saveTableWidget();
	delete ui;
}

void teLabelBrowser::InitInterface()
{
	LabelWidget = new QTableWidget();
	ui->TableWidgetLayout->addWidget(LabelWidget);

	LabelWidget->setColumnCount(4);
	QStringList headerLabels;
	headerLabels << u8"类别" << u8"标记" << u8"找到" << u8"匹配";
	LabelWidget->setHorizontalHeaderLabels(headerLabels);

	LabelWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	LabelWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
	LabelWidget->verticalHeader()->setVisible(false);
	LabelWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	LabelWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);//自适应
	LabelWidget->setAlternatingRowColors(true);
	LabelWidget->setPalette(QPalette(Qt::gray));
	connect(LabelWidget, &QTableWidget::cellDoubleClicked, this, &teLabelBrowser::ColorSelect);
	connect(LabelWidget, &QTableWidget::itemSelectionChanged, this, &teLabelBrowser::handleSelectionChanged);
}

void teLabelBrowser::SendCurrentItemInfo(QTableWidgetItem* item)
{
	if (item) {
		QString content = item->text();
		QColor fontColor = item->foreground().color();
		emit sig_currentRowSelected(content, fontColor);
	}
}

void teLabelBrowser::saveTableWidget()
{
	QFile file("./tableconfig.ini");
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
		return;

	QTextStream stream(&file);
	int rowCount = LabelWidget->rowCount();
	int columnCount = LabelWidget->columnCount();

	for (int row = 0; row < rowCount; ++row)
	{
		QTableWidgetItem* item = LabelWidget->item(row, 0);
		if (item)
		{
			QString data = item->data(Qt::DisplayRole).toString();
			stream << data << "\t";
		}

		QTableWidgetItem* itemcolor = LabelWidget->item(row, 0);
		if (itemcolor)
		{
			stream << itemcolor->foreground().color().name() << "\t";
		}
		stream << "\n";
	}
	file.close();
}

void teLabelBrowser::loadTableWidget()
{
	QFile file("./tableconfig.ini");
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
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
		SendCurrentItemInfo(currentItem);

		return;
	}

	LabelWidget->clearContents();

	QTextStream stream(&file);
	while (!stream.atEnd())
	{
		QString line = stream.readLine();
		QStringList items = line.split("\t");

		int row = LabelWidget->rowCount();
		LabelWidget->insertRow(row);

		QString data = items[0];
		QTableWidgetItem* item = new QTableWidgetItem(data);
		LabelWidget->setItem(row, 0, item);

		int columnCount = items.size();
		for (int column = 1; column < columnCount-1; ++column)
		{
			if (column < LabelWidget->columnCount())
			{
				QString data = "0";
				QTableWidgetItem* item = new QTableWidgetItem(data);
				LabelWidget->setItem(row, column, item);
			}
		}
		QTableWidgetItem* delegate = LabelWidget->item(row, 0);
		if (delegate)
		{
			QString colorName = items[columnCount - 2];
			QColor color(colorName);
			delegate->setForeground(color);
		}
	}

	file.close();
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
	LabelWidget->setItem(row, 1, new QTableWidgetItem("0"));
	LabelWidget->setItem(row, 2, new QTableWidgetItem("0"));
	LabelWidget->setItem(row, 3, new QTableWidgetItem("0"));
	
	QTableWidgetItem* currentItem = LabelWidget->item(row, 0);
	LabelWidget->setCurrentItem(currentItem);
	SendCurrentItemInfo(currentItem);
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

		SendCurrentItemInfo(item);
	}
}

void teLabelBrowser::ColorSelect()
{
	QTableWidgetItem* selectedItem = nullptr;
	QList<QTableWidgetItem*> selectedItems = LabelWidget->selectedItems();
	selectedItem = selectedItems.first();
	if (selectedItem) {
		QColorDialog colorDialog;
		if (colorDialog.exec() == QDialog::Accepted) {
			QColor selectedColor = colorDialog.selectedColor();
				
			int selectedRow = selectedItem->row();
			QTableWidgetItem* firstColumnItem = LabelWidget->item(selectedRow, 0);
			if (firstColumnItem) {
				firstColumnItem->setForeground(QBrush(selectedColor));
				SendCurrentItemInfo(firstColumnItem);
			}
			
		}
	}
}


