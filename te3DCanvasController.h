#pragma once

#include <QObject>
#include "te3DCanvas.h"
#include "te3DCanvasMenu.h"
#include "te3DCanvasToolBar.h"
#include "Filter_Guass.h"
#include "Filter_Direct.h"
#include "te3DCanvasPointCloudColorSelectDialog.h"
#include "te3DCanvasCoordinateAxisRenderDialog.h"
#include "PointCloud_PointSize_Set_Dialog.h"

class te3DCanvasController  : public QObject
{
	Q_OBJECT

public:
	te3DCanvasController(QObject *parent = nullptr);
	~te3DCanvasController();

	void displayUIInWidget(QVBoxLayout* layout);

private slots:
	void BackgroundColorSelect();
	void CoordinateAxisSelect();
	void PointCloudColorSelect();
	void PointCloudPointSizeSelect();
	void GaussFilterAction();
	void DirectFilterAction();

public slots:
	void hideAllUI();
	void showAllUI();

signals:
	void sig_BackgroundColor(QColor color);
	void sig_CoordinateAxis(QString curaxis);
	void sig_PointCloudColor(QColor color);
	void sig_PointCloudPointSize(int size);
	void sig_AABBSurrounding();
	void sig_OBBSurrounding();
	void sig_GuassFilter(QString data1, QString data2, QString data3, QString data4);
	void sig_DirectFilter(QString data1, QString data2, QString data3, QString data4);
	void sig_PointCloudMarkingCompleted(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
	void sig_LabelChanged(const QString& content, const QColor& fontColor);

private:
	te3DCanvas* m_te3DCanvas;
	te3DCanvasMenu* m_te3DCanvasMenu;
	te3DCanvasToolBar* m_te3DCanvasToolBar;

	te3DCanvasPointCloudColorSelectDialog* dialog_colorselect;
	te3DCanvasCoordinateAxisRenderDialog* dialog_render;
	PointCloud_PointSize_Set_Dialog* pointsize_set_dialog;

	Filter_Guass* dialog_Guass_filter;
	Filter_Direct* dialog_Direct_filter;
};
