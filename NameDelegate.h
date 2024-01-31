#pragma once

#include <QItemDelegate>

class NameDelegate  : public QItemDelegate
{
	Q_OBJECT

public:
	NameDelegate(QItemDelegate *parent = nullptr);
	NameDelegate(QString resolution,QString imageName, QItemDelegate* parent = nullptr);
	~NameDelegate();

	// �����༭���������Լ���Ҫ�Ŀؼ������з��ء�
	virtual QWidget *createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// ���ñ༭�����ݣ�Ϊ�����Ŀؼ���ֵ��
	virtual void setEditorData(QWidget* editor, const QModelIndex& index) const override;
	// ���ÿؼ������ԣ����罫������Ϊ���Ρ�Բ�εȡ�
	virtual void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const override;
	// ���ÿؼ���Ӧ��Ԫ������ʾ�����ݡ�
	virtual void setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const override;

private:
	// �洢ImageInfoDisplay������
	QString m_Resolution;
	QString m_ImageName;
};
