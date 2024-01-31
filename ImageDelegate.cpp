#include "ImageDelegate.h"
#include <QLabel>

ImageDelegate::ImageDelegate(QItemDelegate *parent)
	: QItemDelegate(parent)
{
}

ImageDelegate::ImageDelegate(QImage img, QItemDelegate* parent) :QItemDelegate(parent)
{
	this->m_DisplayImage = img;
}

ImageDelegate::~ImageDelegate()
{}

QWidget* ImageDelegate::createEditor(QWidget * parent, const QStyleOptionViewItem & option, const QModelIndex & index) const
{
	QLabel* editor = new QLabel(parent);
	return editor;
}

void ImageDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
	QLabel* label = static_cast<QLabel*>(editor);
	label->setPixmap(QPixmap::fromImage(m_DisplayImage));
}

void ImageDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	editor->setGeometry(option.rect);
}

void ImageDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
	QLabel* label = static_cast<QLabel*>(editor);	// 类型转换
	// 模型（单元格）显示的数据
	model->setData(index, label->pixmap());
}