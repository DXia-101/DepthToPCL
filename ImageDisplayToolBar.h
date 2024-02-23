#pragma once

#include <QWidget>
#include "ui_ImageDisplayToolBar.h"
#include "ImageLabel.h"

QT_BEGIN_NAMESPACE
namespace Ui { class ImageDisplayToolBarClass; };
QT_END_NAMESPACE

class ImageDisplayToolBar : public QWidget
{
	Q_OBJECT

public:
	ImageDisplayToolBar(ImageLabel* imagelabel, QWidget* parent = nullptr);
	~ImageDisplayToolBar();

public:
	void InterfaceInit();

public slots:
	void on_redoButton_clicked();
	void on_undoButton_clicked();

protected slots:
	void on_ShapeCBox_currentTextChanged(const QString& arg1);
	void on_eraserButton_clicked();

signals:
	void ShapeSelected(QString shape);
	void DrawStatus();
	void EraseStatus();

private:
	Ui::ImageDisplayToolBarClass *ui;

	ImageLabel* m_imageLabel;
};
