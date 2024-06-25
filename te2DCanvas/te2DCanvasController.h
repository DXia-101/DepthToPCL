#pragma once

#include <QObject>
#include "te2DCanvas.h"
#include "te2DCanvasToolBar.h"
#include "teImage.h"
#include <QStackedLayout>

class te2DCanvasController  : public QObject
{
	Q_OBJECT

public:
	te2DCanvasController(QObject* parent = nullptr);
	~te2DCanvasController();

	void displayToolBarInWidget(QVBoxLayout* layout);
	void displayCanvasInWidget(QStackedLayout* layout);
	void setImage(const te::Image& img, bool resetView = true);

signals:
	void sig_ClearAll2DCanvasSymbol();
	void sig_StartMarking();
	void sig_currentLabelChange(const QString& category, const QColor& color);
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

private:
	void ShowAllResults();
	void ShowAllMarkers();

private:
	static te2DCanvasController* instance;

	te2DCanvas* m_te2DCanvas;
	te2DCanvasToolBar* m_te2DCanvasToolBar;
	bool IsNeedReload = false;
	bool IsFirstShow = true;
};
