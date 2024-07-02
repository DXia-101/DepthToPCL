#pragma once

#include <QGraphicsView>
#include "pcl_function.h"
#include <vector>
#include <QStateMachine>
#include <QState>
#include <QGraphicsScene>
#include "teImage.h"
#include "teGraphicsViewV2.h"

#include <QMap>
#include "teGraphicsBrushV2.h"
#include "teGraphicsItemMgrV2.h"
#include "teAiExTypes.h"

#include <QStateMachine>
#include <QState>


class te2DCanvas : public te::GraphicsView
{
	Q_OBJECT

public:
	te2DCanvas(QWidget *parent = nullptr);
	~te2DCanvas();

	void te2DCanvasMarkingCompleted();

public:
	void RemoveDimentsion();
	void RemoveResult();

public slots:
	void MarkersShowInCanvas(te::AiInstance* instance, QString label, QColor color);
	void ResultsShowInCanvas(te::AiInstance* instance, QString label, QColor color);
	void ShapeSelect(QString shape);
	void CurrentStateChanged(const QString& category, const QColor& fontColor, const int& index = 0, const double& valThreshold = 0, const double& invalThreshold = 0);
	void StartMarked();
	void Redo();
	void Undo();
	void setImg(te::Image* img);
	void ShowDimension(int arg);
	void ShowResult(int arg);
	void ShowLocalMask(int arg);
	void ShowGlobalMask(int arg);

signals:
	void sig_PolygonMarkingCompleted(QList<te::GraphicsItem*> polygonItems);
	void sig_ReplaceToEraseState();
	void sig_ReplaceToDrawState();
	void sig_updateTrainWidget();
	void sig_eraseMarkers();

private:
	void InitStateMachine();
	void DrawPolygonGraphics(const QPolygonF& polygon);
	void DrawRectGraphics(const QRectF& rect);
	void DrawLineGraphics(const QList<QPolygonF>& polyline);
	void DrawGraphics(const QList<QPolygonF>& region);
private:
	QStateMachine* m_pStateMachine;
	QState* DrawState;
	QState* EraseState;

private:
	struct currentState m_curstate;

	te::RectGraphicsBrush* RectBrush;
	te::PolygonGraphicsBrush* PolygonBrush;
	te::PolyLineGraphicsBrush* LineBrush;

	bool isStartMark = false;
};