#pragma once

#include <QItemDelegate>
#include <QImage>

class ImageDelegate  : public QItemDelegate
{
	Q_OBJECT

public:
	ImageDelegate(QItemDelegate*parent = nullptr);
	ImageDelegate(QImage img, QItemDelegate* parent = nullptr);
	~ImageDelegate();

	// �����༭��
	virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// ���ñ༭������
	virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
	// ���±༭����������
	virtual void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// ����ģ������
	virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;

private:
	QImage m_DisplayImage;
};
