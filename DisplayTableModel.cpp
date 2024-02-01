#include "DisplayTableModel.h"


DisplayTableModel::DisplayTableModel(QAbstractTableModel *parent)
	: QAbstractTableModel(parent)
{
	this->m_HorizontalHead.append("����ͼ");
	this->m_HorizontalHead.append("ͼƬ��Ϣ");
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
	// �к��еĺϷ��Լ��
	if (row < 0 || row >= this->m_ImgInfoVector.size() || column < 0 || column >= COLUMN_HEAD_INDEX::COLUMN) {
		return QModelIndex();
	}

	switch (column) {
	case COLUMN_HEAD_INDEX::ImgLable:
		// ��ȡ��Ӧ�к��е�Ԫ�������
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
		// �����û������������н���
		return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
	}

	switch (index.column()) {
	case COLUMN_HEAD_INDEX::ImgLable:
		// �����û������������н���
		return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
		break;
	case COLUMN_HEAD_INDEX::InfoLable:
		// �����û����Խ��б༭
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
		// ������ɺ����źŸ�����ͼ�������޸�
		emit dataChanged(index, index);	// ���������ϽǺ����½ǵ�ģ����������������ͬ�ģ�
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

	// ����λ���������Ӧλ�ý��в�������
	for (int addCount = 0; addCount < count; addCount++) {
		this->m_ImgInfoVector.insert(row + addCount, {});
	}
	// ����������
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

	// ����λ���������Ӧλ�ý����Ƴ�����
	for (int removeCount = 0; removeCount < count; removeCount++) {
		this->m_ImgInfoVector.removeAt(row);
	}

	endRemoveRows();
	return true;
}
