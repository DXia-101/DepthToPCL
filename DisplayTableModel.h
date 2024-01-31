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
	
	// 返回行个数
	virtual int rowCount(const QModelIndex& parent = QModelIndex()) const override;
	// 返回列个数
	virtual int columnCount(const QModelIndex& parent = QModelIndex()) const override;
	// 返回头文本
	virtual QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const override;
	// 返回索引数据
	virtual QVariant data(const QModelIndex& index, int role = Qt::DisplayRole) const override;
	// 返回模型索引
	virtual QModelIndex index(int row, int column, const QModelIndex& parent = QModelIndex()) const override;
	// 返回模型索引的编辑方式
	virtual Qt::ItemFlags flags(const QModelIndex& index) const override;
	// 设置模型索引数据
	virtual bool setData(const QModelIndex& index, const QVariant& value, int role = Qt::EditRole) override;
	// 插入行
	virtual bool insertRows(int row, int count, const QModelIndex& parent = QModelIndex()) override;
	// 删除行
	virtual bool removeRows(int row, int count, const QModelIndex& parent = QModelIndex()) override;

private:
	QList<QPixmap> m_ImgLable;
	QStringList m_InfoLable;

	QStringList m_HorizontalHead;
};
