#include "teLabelBrowserView.h"
#include "teViewModel.h"
#include "teRapidjsonObjectTree.h"
#include <QAbstractItemDelegate>
#include <QColorDialog>
#include <QDialog>
#include <QFile>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QTextStream>
#include <QVBoxLayout>
#include <QTableWidget>
#include <QHBoxLayout>
#include <QPushButton>

using namespace te;
LabelBrowserView::LabelBrowserView(QWidget* parent)
	: QWidget(parent)
	, ui(new Ui::LabelBrowserViewClass())
{
	ui->setupUi(this);
	initInterface();
	setAttribute(Qt::WA_DeleteOnClose);
}

LabelBrowserView::~LabelBrowserView()
{
	saveTableWidget();
	delete ui;
}

void LabelBrowserView::refresh(ViewModel::updateMode mode)
{
	if (mode == ViewModel::updateTrainCount)
	{
		updateTrainCount();
	}
	else if (mode == ViewModel::updateResultCount)
	{
		updateResultCount();
	}
}

void LabelBrowserView::initInterface()
{
	labelWidget = new QTableWidget();
	ui->TableWidgetLayout->addWidget(labelWidget);

	labelWidget->setColumnCount(4);
	QStringList headerLabels;
	headerLabels << u8"类别" << u8"标记" << u8"找到" << u8"匹配";
	labelWidget->setHorizontalHeaderLabels(headerLabels);

	labelWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
	labelWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
	labelWidget->verticalHeader()->setVisible(false);
	labelWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	labelWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);//自适应
	labelWidget->setAlternatingRowColors(true);
	labelWidget->setPalette(QPalette(Qt::gray));
	connect(labelWidget, &QTableWidget::cellDoubleClicked, this, &LabelBrowserView::selectLabelColor);
	connect(labelWidget, &QTableWidget::itemSelectionChanged, this, &LabelBrowserView::selectCurrentLabel);
}

void LabelBrowserView::changeCurrentLabelInfo(QTableWidgetItem* item)
{
	if (item) {
		QString content = item->text();
		QColor fontColor = item->foreground().color();

		viewModel.lock()->changeCurrentLabelInfo(content, fontColor);
		viewModel.lock()->setCurrentLabel(content);
	}
}

void LabelBrowserView::saveTableWidget()
{
	QString filePath = "./workspace/tableconfig.ini";

	QFile file(filePath);

	if (!file.exists()) {
		if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
			file.close();
		}
	}

	JsonFile jsonfile;
	jsonfile.open("./workspace/tableconfig.ini");
	std::map<std::string, std::string> labelconfig;
	int rowCount = labelWidget->rowCount();
	int columnCount = labelWidget->columnCount();

	for (int row = 0; row < rowCount; ++row)
	{
		QTableWidgetItem* item = labelWidget->item(row, 0);
		std::string category = "", fontcolor = "";
		if (item)
		{
			category = item->data(Qt::DisplayRole).toString().toStdString();
		}

		QTableWidgetItem* itemcolor = labelWidget->item(row, 0);
		if (itemcolor)
		{
			fontcolor = itemcolor->foreground().color().name().toStdString();
		}
		labelconfig[category] = fontcolor;
	}

	jsonfile.write({ "LabelBrowser" }, labelconfig);
}

void LabelBrowserView::loadTableWidget()
{
	JsonFile file;
	if (!file.open("./workspace/tableconfig.ini"))
	{
		labelWidget->insertRow(0);
		labelWidget->setItem(0, 0, new QTableWidgetItem("LMask"));
		labelWidget->setItem(0, 1, new QTableWidgetItem("0"));
		labelWidget->setItem(0, 2, new QTableWidgetItem("0"));
		labelWidget->setItem(0, 3, new QTableWidgetItem("0"));

		labelWidget->insertRow(1);
		labelWidget->setItem(1, 0, new QTableWidgetItem("GMask"));
		labelWidget->setItem(1, 1, new QTableWidgetItem("0"));
		labelWidget->setItem(1, 2, new QTableWidgetItem("0"));
		labelWidget->setItem(1, 3, new QTableWidgetItem("0"));

		labelWidget->insertRow(2);
		labelWidget->setItem(2, 0, new QTableWidgetItem("BG"));
		labelWidget->setItem(2, 1, new QTableWidgetItem("0"));
		labelWidget->setItem(2, 2, new QTableWidgetItem("0"));
		labelWidget->setItem(2, 3, new QTableWidgetItem("0"));

		QTableWidgetItem* currentItem = labelWidget->item(0, 0);
		viewModel.lock()->addLabelInfo(currentItem->text(), currentItem->foreground().color());
		currentItem = labelWidget->item(1, 0);
		viewModel.lock()->addLabelInfo(currentItem->text(), currentItem->foreground().color());
		currentItem = labelWidget->item(2, 0);
		viewModel.lock()->addLabelInfo(currentItem->text(), currentItem->foreground().color());

		labelWidget->setCurrentItem(currentItem);
		return;
	}

	labelWidget->clearContents();
	std::map<std::string, std::string> labelconfig;
	file.read({ "LabelBrowser" }, &labelconfig);
	for (auto& info : labelconfig)
	{
		int row = labelWidget->rowCount();
		labelWidget->insertRow(row);

		QTableWidgetItem* item = new QTableWidgetItem(QString::fromStdString(info.first));
		labelWidget->setItem(row, 0, item);

		for (int column = 1; column < 4; ++column)
		{
			if (column < labelWidget->columnCount())
			{
				QString data = "0";
				QTableWidgetItem* item = new QTableWidgetItem(data);
				labelWidget->setItem(row, column, item);
			}
		}
		QTableWidgetItem* delegate = labelWidget->item(row, 0);
		if (delegate)
		{
			QString colorName = QString::fromStdString(info.second);
			QColor color(colorName);
			delegate->setForeground(color);
		}
		viewModel.lock()->addLabelInfo(item->text(), item->foreground().color());
	}

}

void LabelBrowserView::addRowToTable(const QString& content, const QColor& fontColor)
{
	if (checkFirstColumn(content)) {
		return;
	}
	int row = labelWidget->rowCount();
	labelWidget->insertRow(row);

	QTableWidgetItem* itemContent = new QTableWidgetItem(content);
	itemContent->setForeground(fontColor);

	labelWidget->setItem(row, 0, itemContent);
	labelWidget->setItem(row, 1, new QTableWidgetItem("0"));
	labelWidget->setItem(row, 2, new QTableWidgetItem("0"));
	labelWidget->setItem(row, 3, new QTableWidgetItem("0"));

	QTableWidgetItem* currentItem = labelWidget->item(row, 0);
	labelWidget->setCurrentItem(currentItem);
	viewModel.lock()->addLabelInfo(currentItem->text(), currentItem->foreground().color());
	viewModel.lock()->setCurrentLabel(content);
}

bool LabelBrowserView::checkFirstColumn(const QString& searchString)
{
	for (int row = 0; row < labelWidget->rowCount(); ++row) {
		QTableWidgetItem* item = labelWidget->item(row, 0);

		if (item && item->text() == searchString) {
			return true;
		}
	}
	return false;
}

void LabelBrowserView::selectCurrentLabel()
{
	if (!labelWidget->selectedItems().isEmpty()) {
		int row = labelWidget->selectedItems().first()->row();
		QTableWidgetItem* item = labelWidget->item(row, 0); // 第一列的索引为0

		changeCurrentLabelInfo(item);
	}
}

void LabelBrowserView::selectLabelColor()
{
	QTableWidgetItem* selectedItem = nullptr;
	QList<QTableWidgetItem*> selectedItems = labelWidget->selectedItems();
	selectedItem = selectedItems.first();
	if (selectedItem) {
		QColorDialog colorDialog;
		if (colorDialog.exec() == QDialog::Accepted) {
			QColor selectedColor = colorDialog.selectedColor();

			int selectedRow = selectedItem->row();
			QTableWidgetItem* firstColumnItem = labelWidget->item(selectedRow, 0);
			if (firstColumnItem) {
				firstColumnItem->setForeground(QBrush(selectedColor));
			}
			changeCurrentLabelInfo(firstColumnItem);
			viewModel.lock()->notified(ViewModel::UpdateMarkerPointCloud);
			viewModel.lock()->notified(ViewModel::UpdateResultPointCloud);
		}
	}
}

void LabelBrowserView::on_addLabelButton_clicked()
{
	QDialog dialog;
	dialog.setWindowTitle("标记输入");

	QVBoxLayout* mainLayout = new QVBoxLayout(&dialog);

	QLabel* label = new QLabel("标记:", &dialog);
	QLineEdit* lineEdit = new QLineEdit(&dialog);
	QHBoxLayout* inputLayout = new QHBoxLayout();
	inputLayout->addWidget(label);
	inputLayout->addWidget(lineEdit);
	mainLayout->addLayout(inputLayout);

	QPushButton* okButton = new QPushButton("确定", &dialog);
	QPushButton* cancelButton = new QPushButton("取消", &dialog);
	QHBoxLayout* buttonLayout = new QHBoxLayout();
	buttonLayout->addStretch(1);
	buttonLayout->addWidget(okButton);
	buttonLayout->addWidget(cancelButton);
	mainLayout->addLayout(buttonLayout);

	QString markerText;

	QObject::connect(okButton, &QPushButton::clicked, [&dialog, lineEdit, &markerText]() {
		markerText = lineEdit->text();
		dialog.accept();
		});

	QObject::connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

	int result = dialog.exec();
	if (result == QDialog::Accepted) {
		addRowToTable(markerText, Qt::black);
	}
}

void LabelBrowserView::on_deleteLabelButton_clicked()
{
	int row = labelWidget->currentRow();
	if (row >= 0) {
		labelWidget->removeRow(row);
	}
}

void te::LabelBrowserView::bindViewModel(std::shared_ptr<ViewModel> vm)
{
	viewModel = vm;
	if (viewModel.lock())
	{
		connect(viewModel.lock().get(), &ViewModel::notified, this, &LabelBrowserView::refresh);
	}

	loadTableWidget();
}

void LabelBrowserView::updateTrainCount()
{
	QMap<QString, int> nameCounts = viewModel.lock()->getTrainMarkerCount();
	for (int row = 0; row < labelWidget->rowCount(); ++row)
	{
		QTableWidgetItem* nameItem = labelWidget->item(row, 0);
		if (nameItem)
		{
			QString name = nameItem->text();
			if (nameCounts.contains(name))
			{
				int count = nameCounts.value(name);
				QTableWidgetItem* countItem = labelWidget->item(row, 1);
				if (!countItem)
				{
					countItem = new QTableWidgetItem();
					labelWidget->setItem(row, 1, countItem);
				}
				countItem->setData(Qt::DisplayRole, count);
			}
			else if (nameCounts.size() == 0 || !(nameCounts.contains(name)))
			{
				int count = 0;
				QTableWidgetItem* countItem = labelWidget->item(row, 1);
				if (!countItem)
				{
					countItem = new QTableWidgetItem();
					labelWidget->setItem(row, 1, countItem);
				}
				countItem->setData(Qt::DisplayRole, count);
			}
		}
	}
}

void LabelBrowserView::updateResultCount()
{
	QMap<QString, int> nameCounts = viewModel.lock()->getTestResultCount();
	for (int row = 0; row < labelWidget->rowCount(); ++row)
	{
		QTableWidgetItem* nameItem = labelWidget->item(row, 0);
		if (nameItem)
		{
			QString name = nameItem->text();
			if (nameCounts.contains(name))
			{
				int count = nameCounts.value(name);
				QTableWidgetItem* countItem = labelWidget->item(row, 2);
				if (!countItem)
				{
					countItem = new QTableWidgetItem();
					labelWidget->setItem(row, 2, countItem);
				}
				countItem->setData(Qt::DisplayRole, count);
			}
		}
	}
}