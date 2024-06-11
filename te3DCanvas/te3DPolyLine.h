#pragma once

#include <QWidget>

class te3DPolyLine  : public QWidget
{
	Q_OBJECT

public:
	te3DPolyLine(QWidget *parent = nullptr);
	~te3DPolyLine();

	void InitPolyLine();
	void SetDraw(bool bDraw);

    QVector<QPointF>& GetPointList();

signals:
    void sig_DrawOver();

protected:
    void enterEvent(QEvent* event) override;
    void leaveEvent(QEvent* event) override;
    void paintEvent(QPaintEvent* e) override;     //绘制
    void mousePressEvent(QMouseEvent* e) override;       //按下
    void mouseMoveEvent(QMouseEvent* e) override;        //移动
    void mouseReleaseEvent(QMouseEvent* e) override;     //松开

    bool bDraw;             //是否处于绘制状态
    bool bLeftClick;            //是否已经开始左键点击，同时标识是否开始进行绘制
    bool bOverDraw;

    QVector<QPointF> pointList;
};
