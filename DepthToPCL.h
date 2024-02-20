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
#include "AiModelInterface.h"
#include "TrainingStatisticsChart.h"
#include "ImageDisplayToolBar.h"

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
    void addAiInstance(QList<QPolygonF>& Polygons);
    void addAiInstance(pcl::PointCloud<pcl::PointXYZ>::Ptr markedCloud);

    /// <summary>
    /// ����ǰ��ǵ�����AiInstanceת��Ϊ����
    /// </summary>
    void AiInstSet2Cloud(QColor color);

    /// <summary>
    /// ����ǰ��ǵ�����AiInstanceת��Ϊ�����
    /// </summary>
    void AiInstSet2PolygonItem(QString category, QColor color);
    
private:
    void keyPressEvent(QKeyEvent* event) override;

private:
    /// <summary>
    /// ����ͼ����������ǩ����
    /// </summary>
    /// <param name="Matin">������������ͼ��</param>
    /// <param name="LabelName">���������ı�ǩ</param>
    void SaveMatContour2Label(cv::Mat& Matin, QString LabelName);

    /// <summary>
    /// ������б������
    /// </summary>
    void ClearAllMarkedContent();

    bool isLabelExist(QString curlabel);

private slots:
    void on_startTagBtn_clicked(); //��ʼ���

    void on_changeFormBtn_clicked(); //ת�����水ť
    void on_drawCounterBtn_clicked(); //����ԭͼ������ť

    void on_drawMarkersBtn_clicked(); //���Ʊ��
    void on_clearMarkersBtn_clicked(); //������б��

    void SaveContour();
    void LoadContour();

    void ReceiveMarkedPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void ReceiveMarkedPolygonItem(QList<QPolygonF>& Polygons);

    void UpdatePointCloud2DImage();

    void LoadTrainingImages();
    void StartedTrainAction();
    void StopTrainAction();
    void StartTestAction();
    void ShowTrainChartAction();

    void SwitchDisplayItem(int iIndex, int iLayerIndex = 0);

signals:
    void ConversionBetween2Dand3D();
    
private:
    Ui::DepthToPCLClass ui;

    int point_size = 1; //���ƴ�С

    cv::Mat m_image;

    ImageLabel* teImageWidget;
    VTKOpenGLNativeWidget* vtkWidget;

    VTKToolBar* m_vtkToolBar;
    _3DMenuInterface* m_thrDMenuInterface;
    ImageDisplayToolBar* m_imageDisplayToolBar;

    QStateMachine* m_pStateMachine;
    QState* TwoDState;
    QState* ThrDState;

    te::Image currentDisplayImage;

    std::vector<te::SampleInfo> vTrainSamples;
    std::vector<QString> TiffData;
    std::vector<QString> GTData;
    AiModelInterface* workAiModel;
    TrainingStatisticsChart* trainChart; 

    QStringList m_lstImgs;
    int currentIndex;

    int SumPixNum;
};
