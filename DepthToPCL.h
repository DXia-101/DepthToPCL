#pragma once

#include <QtWidgets/QWidget>
#include "ui_DepthToPCL.h"

#include "pcl_function.h"
#include "teAiExTypes.h"

#include "VTKOpenGLNativeWidget.h"
#include "ImageLabel.h"
#include "teGraphicsViewV2.h"
#include "_3DMenuInterface.h"
#include "VTKToolBar.h"

#include <QThread>
#include <QVBoxLayout>
#include <QStateMachine>
#include <QPaintEvent>
#include <QKeyEvent>
#include <QState>

class DepthToPCL : public QWidget
{
    Q_OBJECT

public:
    DepthToPCL(QWidget *parent = nullptr);
    ~DepthToPCL();

    /// <summary>
    /// ��ʼ������
    /// </summary>
    void Interface_Initialization();

    /// <summary>
    /// ��ʼ��PCL
    /// </summary>
    void PCL_Initalization();

    /// <summary>
    /// ��ʼ��״̬��
    /// </summary>
    void InitStateMachine();
protected:
    /// <summary>
    /// ��ӱ�ǵ���ǰѡ���ǩ
    /// </summary>
    /// <param name="curlabel">��ǰѡ���ǩ</param>
    /// <param name="Polygons">��ǵĶ����������</param>
    void addAiInstance(DynamicLabel* curlabel, QList<QPolygonF>& Polygons);

    /// <summary>
    /// ��ӱ�ǵ���ǰѡ���ǩ
    /// </summary>
    /// <param name="curlabel">��ǰѡ���ǩ</param>
    /// <param name="markedCloud">��ǵĵ���</param>
    void addAiInstance(DynamicLabel* curlabel,pcl::PointCloud<pcl::PointXYZ>::Ptr markedCloud);

    /// <summary>
    /// ����ǰ��ǵ�����AiInstanceת��Ϊ����
    /// </summary>
    /// <param name="curlabel">��ǰ���</param>
    void AiInstSet2Cloud(DynamicLabel* curlabel);

    /// <summary>
    /// ����ǰ��ǵ�����AiInstanceת��Ϊ�����
    /// </summary>
    /// <param name="curlabel">��ǰ���</param>
    void AiInstSet2PolygonItem(DynamicLabel* curlabel);
    
private:
    void mousePressEvent(QMouseEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    /// <summary>
    /// ����ͼ����������ǩ����
    /// </summary>
    /// <param name="Matin">������������ͼ��</param>
    /// <param name="curlabel">���������ı�ǩ</param>
    void SaveMatContour2Label(cv::Mat& Matin, DynamicLabel* curlabel);

    /// <summary>
    /// ������б������
    /// </summary>
    void ClearAllMarkedContent();

private slots:
    void on_addDynamicLabel_clicked(); //��ӱ��
    void on_delDynamicLabel_clicked(); //ɾ�����
    void on_startTagBtn_clicked(); //��ʼ���

    void on_changeFormBtn_clicked(); //ת�����水ť
    void on_drawCounterBtn_clicked(); //����ԭͼ������ť

    void on_MarkCompleted_clicked(); //��ɱ��
    void on_drawMarkersBtn_clicked(); //���Ʊ��
    void on_clearMarkersBtn_clicked(); //������б��

    void SaveContour();
    void LoadContour();

    void ReceiveMarkedPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void ReceiveMarkedPolygonItem(QList<QPolygonF>& Polygons);

    void WarningForUnselectedTags();

    void UpdatePointCloud2DImage();
signals:
    void ConversionBetween2Dand3D();

private:
    Ui::DepthToPCLClass ui;

    int totalHeight; //����ҳ�������߶�

    int point_size = 1; //���ƴ�С

    cv::Mat m_image;

    ImageLabel* teImageWidget;
    VTKOpenGLNativeWidget* vtkWidget;

    VTKToolBar* m_vtkToolBar;
    _3DMenuInterface* m_thrDMenuInterface;

    QVBoxLayout* labelVLayout;
    QStateMachine* m_pStateMachine;
    QState* TwoDState;
    QState* ThrDState;

    QString currentLabelNAme;

    te::Image currentDisplayImage;
};
