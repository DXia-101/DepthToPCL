#pragma once

#include <QAbstractTableModel>
#include "ImageInfoDisplay.h"
#include <QPixmap>

#pragma execution_character_set("utf-8")

enum COLUMN_HEAD_INDEX {
	ImgLable = 0,
	InfoLable = 1,
	COLUMN,
};

class DisplayTableModel  : public QAbstractTableModel
{
	Q_OBJECT

public:
	DisplayTableModel(QAbstractTableModel *parent = nullptr);
	~DisplayTableModel();
	
	// �����и���
	virtual int rowCount(const QModelIndex& parent = QModelIndex()) const override;
	// �����и���
	virtual int columnCount(const QModelIndex& parent = QModelIndex()) const override;
	// ����ͷ�ı�
	virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
	// ������������
	virtual QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
	// ����ģ������
	virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
	// ����ģ�������ı༭��ʽ
	virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
	// ����ģ����������
	virtual bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;
	// ������
	virtual bool insertRows(int row, int count, const QModelIndex& parent = QModelIndex()) override;
	// ɾ����
	virtual bool removeRows(int row, int count, const QModelIndex& parent = QModelIndex()) override;

private:
	QList<QPixmap> m_ImgLable;
	QStringList m_InfoLable;

	QStringList m_HorizontalHead;
};
