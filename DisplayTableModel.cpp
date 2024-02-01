#include "DisplayTableModel.h"


DisplayTableModel::DisplayTableModel(QAbstractTableModel *parent)
	: QAbstractTableModel(parent)
{
	this->m_HorizontalHead.append("缩略图");
	this->m_HorizontalHead.append("图片信息");
}

DisplayTableModel::~DisplayTableModel()
{}

int DisplayTableModel::rowCount(const QModelIndex & parent) const
{
	return this->m_ImgInfoVector.size();
}

int DisplayTableModel::columnCount(const QModelIndex& parent) const
{
	return COLUMN_HEAD_INDEX::COLUMN;
}

QVariant DisplayTableModel::headerData(int section, Qt::Orientation orientation, int role) const
{
	if (role != Qt::DisplayRole)
		return QAbstractTableModel::headerData(section, orientation, role);
	if (orientation == Qt::Orientation::Horizontal) {
		return this->m_HorizontalHead[section];
	}

	return QAbstractTableModel::headerData(section, orientation, role);
}

QVariant DisplayTableModel::data(const QModelIndex& index, int role) const
{
	if (!index.isValid()) return QVariant();

	if (role == Qt::DisplayRole || role == Qt::EditRole) {
		switch (index.column()) {
		case COLUMN_HEAD_INDEX::ImgLable: {
			return this->m_ImgInfoVector[index.row()].ImgPath;
			break;
		}
		case COLUMN_HEAD_INDEX::InfoLable:
			return QVariant::fromValue(this->m_ImgInfoVector[index.row()]);
			break;

		default:
			return QVariant();
			break;
		}
	}

	return QVariant();
}

QModelIndex DisplayTableModel::index(int row, int column, const QModelIndex& parent) const
{
	// 行和列的合法性检查
	if (row < 0 || row >= this->m_ImgInfoVector.size() || column < 0 || column >= COLUMN_HEAD_INDEX::COLUMN) {
		return QModelIndex();
	}

	switch (column) {
	case COLUMN_HEAD_INDEX::ImgLable:
		// 获取对应行和列单元格的数据
		return createIndex(row, column);
		break;

	case COLUMN_HEAD_INDEX::InfoLable:
		return createIndex(row, column);
		break;

	default:
		return createIndex(row, column);
		break;
	}

	return createIndex(row, column);
}

Qt::ItemFlags DisplayTableModel::flags(const QModelIndex& index) const
{
	if (!index.isValid()) {
		// 返回用户可以与界面进行交互
		return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
	}

	switch (index.column()) {
	case COLUMN_HEAD_INDEX::ImgLable:
		// 返回用户可以与界面进行交互
		return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
		break;
	case COLUMN_HEAD_INDEX::InfoLable:
		// 返回用户可以进行编辑
		return QAbstractItemModel::flags(index) | Qt::ItemIsEnabled | Qt::ItemIsEditable | Qt::ItemIsSelectable;
		break;
	default:
		return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
		break;
	}
	return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

bool DisplayTableModel::setData(const QModelIndex& index, const QVariant& Invalue, int role)
{
	if (index.isValid() && role == Qt::EditRole) {
		switch (index.column()) {
		case COLUMN_HEAD_INDEX::ImgLable:
			this->m_ImgInfoVector[index.row()].ImgPath = Invalue.toString();
			break;
		case COLUMN_HEAD_INDEX::InfoLable: {
			ImageInfo info = Invalue.value<ImageInfo>();
			this->m_ImgInfoVector[index.row()].ImgName = info.ImgName;
			this->m_ImgInfoVector[index.row()].ImgResolution = info.ImgResolution; 
		}
			break;
		default:
			break;
		}
		// 设置完成后发射信号告诉视图数据有修改
		emit dataChanged(index, index);	// 参数是左上角和右下角的模型索引（这里是相同的）
		return true;
	}
	return false;
}

bool DisplayTableModel::insertRows(int row, int count, const QModelIndex& parent)
{
	if (count == 0) return false;

	if (!parent.isValid()) {
		beginInsertRows(QModelIndex(), row, row + count - 1);
	}
	else {
		beginInsertRows(parent, row, row + count - 1);
	}

	// 按照位置在链表对应位置进行插入数据
	for (int addCount = 0; addCount < count; addCount++) {
		this->m_ImgInfoVector.insert(row + addCount, {});
	}
	// 结束插入行
	endInsertRows();

	return true;
}

bool DisplayTableModel::removeRows(int row, int count, const QModelIndex& parent)
{
	if (count == 0) return false;

	if (!parent.isValid()) {
		beginRemoveRows(QModelIndex(), row, row + count - 1);
	}
	else {
		beginInsertRows(parent, row, row + count - 1);
	}

	// 按照位置在链表对应位置进行移除数据
	for (int removeCount = 0; removeCount < count; removeCount++) {
		this->m_ImgInfoVector.removeAt(row);
	}

	endRemoveRows();
	return true;
}
