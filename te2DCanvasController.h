#pragma once

#include <QObject>
#include "te2DCanvas.h"
#include "te2DCanvasToolBar.h"

class te2DCanvasController  : public QObject
{
	Q_OBJECT

public:
	te2DCanvasController(QObject* parent = nullptr);
	~te2DCanvasController();

	void displayUIInWidget(QVBoxLayout* layout);

public slots:
	void hideAllUI();
	void showAllUI();

private:
	te2DCanvas* m_te2DCanvas;
	te2DCanvasToolBar* m_te2DCanvasToolBar;
};
