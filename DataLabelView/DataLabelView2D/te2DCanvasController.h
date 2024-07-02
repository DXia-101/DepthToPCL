#pragma once

#include <QObject>
#include "te2DCanvas.h"
#include "te2DCanvasToolBar.h"
#include "teImage.h"
#include <QStackedLayout>

class teAiModel;
class teLabelBrowser;

class te2DCanvasController  : public QObject
{
	Q_OBJECT

public:
	te2DCanvasController(QObject* parent = nullptr);
	~te2DCanvasController();

	void displayToolBarInWidget(QVBoxLayout* layout);
	void displayCanvasInWidget(QStackedLayout* layout);
	void setImage(const te::Image& img, bool resetView = true);
	void setteAiModel(teAiModel* aiModel);
	void setteLabelBrowser(teLabelBrowser* browser);
	
signals:
	void sig_ShowFirstImage();
	void sig_CurrentStateChanged(const QString& category, const QColor& fontColor, const int& index, const double& valThreshold, const double& invalThreshold);

	void sig_StartMark();
	void sig_updateTrainWidget();
	void sig_eraseMarkers();

public slots:
	void hideAllUI();
	void showAllUI();
	void add2DAiInstance(QList<te::GraphicsItem*> polygonItems);
	void ShowFirstImage();
	void ShowAllItems();
	void ShowCurrentImages();
	void ReLoadGTAndRST();
	void NeedReload();
	void slotSetImage(te::Image*);
	void LoadOriginImage(QString);

private:
	void ShowAllResults();
	void ShowAllMarkers();

private:
	teAiModel* m_teAiModel;
	teLabelBrowser* m_teLabelBrowser;

	te2DCanvas* m_te2DCanvas;
	te2DCanvasToolBar* m_te2DCanvasToolBar;
	bool IsNeedReload = false;
	bool IsFirstShow = true;
};
