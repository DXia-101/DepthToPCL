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
	// �����༭��
	virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// �Զ���༭�����������߼�
	virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
	// ���±༭����������
	virtual void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// �Զ���༭�����ݱ����߼�
	virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;

private:
	QString m_Resolution;
	QString m_ImageName;
};
