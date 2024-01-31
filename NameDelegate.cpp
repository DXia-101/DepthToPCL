#include "NameDelegate.h"
#include "ImageInfoDisplay.h"

NameDelegate::NameDelegate(QItemDelegate*parent)
	: QItemDelegate(parent)
{
	this->m_ImageName = "";
	this->m_Resolution = "";
}

NameDelegate::NameDelegate(QString resolution, QString imageName, QItemDelegate * parent) :QItemDelegate(parent)
{
	this->m_ImageName = imageName;
	this->m_Resolution = resolution;
}

NameDelegate::~NameDelegate()
{}

QWidget* NameDelegate::createEditor(QWidget * parent, const QStyleOptionViewItem & option, const QModelIndex & index) const
{
	ImageInfoDisplay* editor = new ImageInfoDisplay(parent);
	return editor;
}

void NameDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
	ImageInfoDisplay* Display = static_cast<ImageInfoDisplay*>(editor);
	Display->setResolutionLabelText(m_Resolution);
	Display->setImageNameEditText(m_ImageName);
}

void NameDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	// ���༭������Ϊ��������
	editor->setGeometry(option.rect);
}

void NameDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
	ImageInfoDisplay* Display = static_cast<ImageInfoDisplay*>(editor);	// ����ת��
	// ģ�ͣ���Ԫ����ʾ������
	model->setData(index, Display->GetImageNameEditText());
}