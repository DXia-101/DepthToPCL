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
	// 将编辑器设置为矩形属性
	editor->setGeometry(option.rect);
}

void NameDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
	ImageInfoDisplay* Display = static_cast<ImageInfoDisplay*>(editor);	// 类型转换
	// 模型（单元格）显示的数据
	model->setData(index, Display->GetImageNameEditText());
}