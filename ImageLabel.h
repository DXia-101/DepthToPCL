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


class ImageLabel  : public te::GraphicsView
{
	Q_OBJECT

public:
	ImageLabel(QWidget *parent);
	~ImageLabel();

	void AiInstance2GraphicsItem(te::AiInstance* instance,QString label, QColor color);
	void ClearMarks();

public slots:
	void ShapeSelect(QString shape);
	void LabelChanged(const QString& content, const QColor& fontColor);
	void StartMarked();

signals:
	void PolygonMarkingCompleted(te::ConnectedRegionGraphicsItem* polygonItem);
	void ReplaceToEraseState();
	void ReplaceToDrawState();
	void ClearCurrentImageMarkers();

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

public:
	QString currentCategory;
	QColor currentColor;
};
