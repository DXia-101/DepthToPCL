#pragma once

#include <QWidget>
#include "ui_te3DCanvasToolBar.h"
#include "te3DCanvasPointCloudColorSelectDialog.h"
#include "CrossSection.h"
#include "te3DCanvasPointCloudColorSelectDialog.h"
#include "te3DCanvasCoordinateAxisRenderDialog.h"
#include "PointCloud_PointSize_Set_Dialog.h"

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

public slots:
	void MaintainCoordinateAxis();

private slots:
	void BackgroundColorSelect();
	void CoordinateAxisSelect();
	void PointCloudColorSelect();
	void PointCloudPointSizeSelect();
	//void GaussFilterAction();
	void CrossSectionAction();
	void SaveAxis(QString axis);
signals:
	void sig_BackgroundColor(QColor color);
	void sig_CoordinateAxis(QString curaxis);
	void sig_PointCloudColor(QColor color);
	void sig_PointCloudPointSize(int size);
	void sig_AABBSurrounding();
	void sig_OBBSurrounding();
	void sig_CrossSection(QString data1, QString data2, QString data3, QString data4);

private:
	Ui::te3DCanvasToolBarClass *ui;

	te3DCanvasPointCloudColorSelectDialog* dialog_colorselect;
	te3DCanvasCoordinateAxisRenderDialog* dialog_render;
	PointCloud_PointSize_Set_Dialog* pointsize_set_dialog;

	CrossSection* dialog_CrossSection;

	QString axis = "z";
};
