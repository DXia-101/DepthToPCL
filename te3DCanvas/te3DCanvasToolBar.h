#pragma once

#include <QWidget>
#include "ui_te3DCanvasToolBar.h"
#include "te3DCanvasPointCloudColorSelectDialog.h"

QT_BEGIN_NAMESPACE
namespace Ui { class te3DCanvasToolBarClass; };
QT_END_NAMESPACE

class te3DCanvasToolBar : public QWidget
{
	Q_OBJECT

public:
	te3DCanvasToolBar(QWidget *parent = nullptr);
	~te3DCanvasToolBar();

private:
	void InitInterface();

signals:
	void sig_BackgroundColorSetting();
	void sig_CoordinateAxisRendering();
	void sig_PointCloudColorSetting();
	void sig_PointCloudPointSizeSetting();
	void sig_GaussFilter();
	void sig_DirectFilter();
	void sig_AABBSurrounding();
	void sig_OBBSurrounding();

private:
	Ui::te3DCanvasToolBarClass *ui;
};
