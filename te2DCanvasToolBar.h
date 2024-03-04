#pragma once

#include <QWidget>
#include "ui_te2DCanvasToolBar.h"

QT_BEGIN_NAMESPACE
namespace Ui { class te2DCanvasToolBarClass; };
QT_END_NAMESPACE

class te2DCanvasToolBar : public QWidget
{
	Q_OBJECT

public:
	te2DCanvasToolBar(QWidget *parent = nullptr);
	~te2DCanvasToolBar();

public:
	void InitInterface();

private slots:
	void on_redoButton_clicked();
	void on_undoButton_clicked();
	void on_eraserButton_clicked();
	void on_ShapeCBox_currentTextChanged(const QString& arg1);

signals:
	void sig_te2DCanvasShapeSelected(QString shape);
	void sig_te2DCanvasDrawStatus();
	void sig_te2DCanvasEraseStatus();
	void sig_te2DCanvasRedo();
	void sig_te2DCanvasUndo();

private:
	Ui::te2DCanvasToolBarClass *ui;
};
