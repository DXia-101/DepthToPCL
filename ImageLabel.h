#pragma once

#include <QGraphicsView>
#include "pcl_function.h"
#include <vector>
#include <QStateMachine>
#include <QState>
#include <QGraphicsScene>
#include "teImage.h"
#include "teGraphicsViewV2.h"
#include "DynamicLabel.h"
#include <QMap>
#include "teGraphicsBrushV2.h"
#include "teGraphicsItemMgrV2.h"
#include "teAiExTypes.h"


class ImageLabel  : public te::GraphicsView
{
	Q_OBJECT

public:
	ImageLabel(QWidget *parent);
	~ImageLabel();

	void LabelChanged();
	void AiInstance2GraphicsItem(te::AiInstance* instance,QString label, QColor color);
	void ClearMarks();

public slots:
	void ShapeSelect(QString shape);

signals:
	void PolygonMarkingCompleted(QList<QPolygonF>& Polygons);

public:
	te::Image image;
	DynamicLabel* currentdynamicLabel;  //当前标签对象

	te::PolygonGraphicsBrush PolygonBrush;
	te::RectGraphicsBrush RectBrush;
private:
	void DrawPolygonGraphics(const QPolygonF& polygon);
	void DrawRectGraphics(const QRectF& rect);
};
