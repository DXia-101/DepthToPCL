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

signals:
	void markedRegion(const QPolygonF& polygon);
	void StateChange();
public:
	GraphicsPolygonScene* scene;
	QImage image;
private:
	
};
