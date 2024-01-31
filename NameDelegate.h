#pragma once

#include <QItemDelegate>

class NameDelegate  : public QItemDelegate
{
	Q_OBJECT

public:
	NameDelegate(QItemDelegate *parent = nullptr);
	NameDelegate(QString resolution,QString imageName, QItemDelegate* parent = nullptr);
	~NameDelegate();

	// 创建编辑器，创建自己需要的控件，进行返回。
	virtual QWidget *createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// 设置编辑器数据，为创建的控件设值。
	virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
	// 设置控件的属性，例如将其设置为矩形、圆形等。
	virtual void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// 设置控件对应单元格中显示的数据。
	virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;

private:
	// 存储ImageInfoDisplay的数据
	QString m_Resolution;
	QString m_ImageName;
};
