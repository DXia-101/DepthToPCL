#include "ImageDelegate.h"
#include <QLabel>
#include <QPainter>
#include <QLineEdit>
#include "Transfer_Function.h"


ImageDelegate::ImageDelegate(QItemDelegate *parent)
	: QItemDelegate(parent)
{
}

ImageDelegate::~ImageDelegate()
{}

void ImageDelegate::paint(QPainter * painter, const QStyleOptionViewItem & option, const QModelIndex & index) const
{
	if (index.column() == 0)
	{
		QString imagePath = index.data().toString();
		QPixmap* pixmap = new QPixmap(imagePath);
		*pixmap = pixmap->scaled(80, 80);
		int nWidth = pixmap->width();
		int nHeight = pixmap->height();
		QRect rect = option.rect;
		int x = rect.x() + rect.width() / 2 - nWidth / 2;  //���ڵ�Ԫ����м�
		int y = rect.y() + rect.height() / 2 - nHeight / 2;
		painter->drawPixmap(x, y, *pixmap);
	}
	else
	{
		painter->setPen(Qt::black);  // ���û�����ɫ
		painter->setFont(QFont("Arial", 12));  // ��������

		QString text = index.data().value<ImageInfo>().ImgResolution;
		painter->drawText(option.rect, Qt::AlignCenter, text);
	}
}

QWidget* ImageDelegate::createEditor(QWidget * parent, const QStyleOptionViewItem & option, const QModelIndex & index) const
{
	if (index.column() == 1)
	{
		QLineEdit* editor = new QLineEdit(parent);
		return editor;
	}
}

void ImageDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
	if (index.column() == 1)
	{
		ImageInfo info = index.data().value<ImageInfo>();
		QLineEdit* disply = static_cast<QLineEdit*>(editor);
		disply->setText(info.ImgName);
	}
}

void ImageDelegate::updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
	editor->setGeometry(option.rect);
}

void ImageDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
	if (index.column() == 1)
	{
		QLineEdit* disply = static_cast<QLineEdit*>(editor);	// ����ת��
		// ģ�ͣ���Ԫ����ʾ������
		ImageInfo info;
		if (disply->text().isEmpty()) {
			info = { "",info.ImgResolution,info.ImgName };
		}
		else {
			info = { "",info.ImgResolution,disply->text()};
		}
		
		model->setData(index, QVariant::fromValue(info));
	}
}