#pragma once

#include <QGraphicsView>
#include "pcl_function.h"
#include <vector>
#include <QStateMachine>
#include <QState>
#include <QGraphicsScene>
#include "GraphicsPolygonScene.h"
class ImageLabel  : public QGraphicsView
{
	Q_OBJECT

public:
	ImageLabel(QWidget *parent);
	~ImageLabel();

	void setImage(const QImage& image);

protected:

private:
	//std::vector<std::vector<cv::Point>> extractContours(const QImage& image, const QVector<QPolygonF>& polygons);

signals:
	void markedRegion(const QPolygonF& polygon);
	void StateChange();
public:
	GraphicsPolygonScene* scene;
private:
	

	QImage image;
};
