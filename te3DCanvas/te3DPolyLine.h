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
    void paintEvent(QPaintEvent* e) override;     //����
    void mousePressEvent(QMouseEvent* e) override;       //����
    void mouseMoveEvent(QMouseEvent* e) override;        //�ƶ�
    void mouseReleaseEvent(QMouseEvent* e) override;     //�ɿ�

    bool bDraw;             //�Ƿ��ڻ���״̬
    bool bLeftClick;            //�Ƿ��Ѿ���ʼ��������ͬʱ��ʶ�Ƿ�ʼ���л���
    bool bOverDraw;

    QVector<QPointF> pointList;
};
