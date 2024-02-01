#pragma once

#include <QItemDelegate>
#include <QImage>



class ImageDelegate  : public QItemDelegate
{
	Q_OBJECT

public:
	ImageDelegate(QItemDelegate* parent = nullptr);
	~ImageDelegate();

	void paint(QPainter* painter, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// 创建编辑器
	virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// 自定义编辑器数据设置逻辑
	virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
	// 更新编辑器集合属性
	virtual void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// 自定义编辑器数据保存逻辑
	virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;

private:
	QString m_Resolution;
	QString m_ImageName;
};
