#include "AssetBrowser.h"
#include <QMessageBox>


AssetBrowser::AssetBrowser(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::AssetBrowserClass())
{
	ui->setupUi(this);
	InitTable();
	insertData();
}

AssetBrowser::~AssetBrowser()
{
	delete ui;
}

void AssetBrowser::InitTable()
{
	this->m_no = 1;

	int headWidth[COLUMN_HEAD_INDEX::COLUMN] = { 80, 250};

	this->m_DisplayTableModel = new DisplayTableModel();
	ui->DisplayLableView->setModel(this->m_DisplayTableModel);

	m_ImageDelegate = new ImageDelegate();

	// 添加委托
	ui->DisplayLableView->setItemDelegate(m_ImageDelegate);

	// 设置水平头文本居中
	ui->DisplayLableView->horizontalHeader()->setDefaultAlignment(Qt::AlignHCenter);
	//设置隔行变色
	ui->DisplayLableView->setAlternatingRowColors(true);
	// 设置最后一列填满表格剩余空间
	ui->DisplayLableView->horizontalHeader()->setStretchLastSection(true);
	// 设置表格行高
	ui->DisplayLableView->verticalHeader()->setDefaultSectionSize(80);

	for (int i = 0; i < COLUMN_HEAD_INDEX::COLUMN; i++) {
		// 设置表格列宽
		ui->DisplayLableView->setColumnWidth(i, headWidth[i]);
	}
	// 设置固定行高
	ui->DisplayLableView->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
	ui->DisplayLableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
	// 设置行表头背景颜色样式为浅黄色
	ui->DisplayLableView->horizontalHeader()->setStyleSheet("QHeaderView::section{background:#ffffff;}");
	//取消显示行头
	ui->DisplayLableView->verticalHeader()->setVisible(false);
	// 设置左上角两个表头相交的区域的颜色样式为浅灰色
	ui->DisplayLableView->setStyleSheet("QTableCornerButton::section{background:#c2c2c2;}");
}

bool AssetBrowser::isSelectionRows()
{
	// 获取所有选中的索引
	QModelIndexList list = ui->DisplayLableView->selectionModel()->selectedIndexes();

	// 存储被选中的所有行
	QVector<int> selectedRows;

	// 获得所有选中单元格的行
	for each (QModelIndex index in list) {
		selectedRows.append(index.row());
	}
	// 进行排序
	std::sort(selectedRows.begin(), selectedRows.end());
	// 去除容器内重复元素
	auto it = std::unique(selectedRows.begin(), selectedRows.end());
	selectedRows.erase(it, selectedRows.end());

	// 遍历所有行的单元格，存在有一个单元格没有被选中，则返回false
	bool selectedFlag = true;
	for (int row = 0; row < selectedRows.count(); row++) {
		for (int column = 0; column < COLUMN_HEAD_INDEX::COLUMN; column++) {
			QModelIndex index = m_DisplayTableModel->index(selectedRows.at(row), column);
			// 检测该索引是否被选中
			if (ui->DisplayLableView->selectionModel()->isSelected(index) == false) {
				selectedFlag = false;
				break;
			}
		}

		// 为false，直接返回结果
		if (selectedFlag == false) return  selectedFlag;

	}

	/* 此处为良好BUG ：当用于没有选中任何单元格，也会返回true，是为了默认在行末尾连续插入行 */
	return selectedFlag;
}

void AssetBrowser::setInsertRowData(int _row)
{
	QString value = "";

	QModelIndex index0 = m_DisplayTableModel->index(_row, COLUMN_HEAD_INDEX::ImgLable);
	m_DisplayTableModel->setData(index0, QString("C:/Users/Administrator/Desktop/LenaRGB.bmp"));

	static int ref = 1;
	QModelIndex index1 = m_DisplayTableModel->index(_row, COLUMN_HEAD_INDEX::InfoLable);
	ImageInfo info{"","Test","Test"};
	m_DisplayTableModel->setData(index1, QVariant::fromValue(info));

}

void AssetBrowser::insertData()
{
	if (!isSelectionRows()) {
		QMessageBox::information(this, "提示", "请选中一行！");
		return;
	}

	// 获取所有选中的索引
	QModelIndexList indexs = ui->DisplayLableView->selectionModel()->selectedIndexes();
	if (indexs.size() == 0) {
		m_DisplayTableModel->insertRows(m_DisplayTableModel->rowCount(), 1);
		setInsertRowData(m_DisplayTableModel->rowCount() - 1);
		return;
	}

	// 获得最后一个索引的行
	int row = indexs.last().row();
	m_DisplayTableModel->insertRows(row + 1, 1);
	setInsertRowData(row + 1);
}