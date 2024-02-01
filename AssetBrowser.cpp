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

	// ���ί��
	ui->DisplayLableView->setItemDelegate(m_ImageDelegate);

	// ����ˮƽͷ�ı�����
	ui->DisplayLableView->horizontalHeader()->setDefaultAlignment(Qt::AlignHCenter);
	//���ø��б�ɫ
	ui->DisplayLableView->setAlternatingRowColors(true);
	// �������һ���������ʣ��ռ�
	ui->DisplayLableView->horizontalHeader()->setStretchLastSection(true);
	// ���ñ���и�
	ui->DisplayLableView->verticalHeader()->setDefaultSectionSize(80);

	for (int i = 0; i < COLUMN_HEAD_INDEX::COLUMN; i++) {
		// ���ñ���п�
		ui->DisplayLableView->setColumnWidth(i, headWidth[i]);
	}
	// ���ù̶��и�
	ui->DisplayLableView->verticalHeader()->setSectionResizeMode(QHeaderView::Fixed);
	ui->DisplayLableView->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);
	// �����б�ͷ������ɫ��ʽΪǳ��ɫ
	ui->DisplayLableView->horizontalHeader()->setStyleSheet("QHeaderView::section{background:#ffffff;}");
	//ȡ����ʾ��ͷ
	ui->DisplayLableView->verticalHeader()->setVisible(false);
	// �������Ͻ�������ͷ�ཻ���������ɫ��ʽΪǳ��ɫ
	ui->DisplayLableView->setStyleSheet("QTableCornerButton::section{background:#c2c2c2;}");
}

bool AssetBrowser::isSelectionRows()
{
	// ��ȡ����ѡ�е�����
	QModelIndexList list = ui->DisplayLableView->selectionModel()->selectedIndexes();

	// �洢��ѡ�е�������
	QVector<int> selectedRows;

	// �������ѡ�е�Ԫ�����
	for each (QModelIndex index in list) {
		selectedRows.append(index.row());
	}
	// ��������
	std::sort(selectedRows.begin(), selectedRows.end());
	// ȥ���������ظ�Ԫ��
	auto it = std::unique(selectedRows.begin(), selectedRows.end());
	selectedRows.erase(it, selectedRows.end());

	// ���������еĵ�Ԫ�񣬴�����һ����Ԫ��û�б�ѡ�У��򷵻�false
	bool selectedFlag = true;
	for (int row = 0; row < selectedRows.count(); row++) {
		for (int column = 0; column < COLUMN_HEAD_INDEX::COLUMN; column++) {
			QModelIndex index = m_DisplayTableModel->index(selectedRows.at(row), column);
			// ���������Ƿ�ѡ��
			if (ui->DisplayLableView->selectionModel()->isSelected(index) == false) {
				selectedFlag = false;
				break;
			}
		}

		// Ϊfalse��ֱ�ӷ��ؽ��
		if (selectedFlag == false) return  selectedFlag;

	}

	/* �˴�Ϊ����BUG ��������û��ѡ���κε�Ԫ��Ҳ�᷵��true����Ϊ��Ĭ������ĩβ���������� */
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
		QMessageBox::information(this, "��ʾ", "��ѡ��һ�У�");
		return;
	}

	// ��ȡ����ѡ�е�����
	QModelIndexList indexs = ui->DisplayLableView->selectionModel()->selectedIndexes();
	if (indexs.size() == 0) {
		m_DisplayTableModel->insertRows(m_DisplayTableModel->rowCount(), 1);
		setInsertRowData(m_DisplayTableModel->rowCount() - 1);
		return;
	}

	// ������һ����������
	int row = indexs.last().row();
	m_DisplayTableModel->insertRows(row + 1, 1);
	setInsertRowData(row + 1);
}