#pragma once

#include <QtWidgets/QWidget>
#include "ui_DepthToPCL.h"
#include "pcl_function.h"
#include "VTKOpenGLNativeWidget.h"
#include "pcl_view_select_color.h"
#include "View_Render.h"
#include "PointCloud_PointSize_Set_Dialog.h"
#include "Filter_Guass.h"
#include "Filter_Direct.h"
#include "DropGraphicsView.h"
#include "ImageLabel.h"
#include "teAiExTypes.h"


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

    void Interface_Initialization();
    void PCL_Initalization();
    void InitStateMachine();
    void TDTo2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin,cv::Mat& imageout);
    //void ExtractContours(Te_Gt& contour, cv::Mat imgIn);
    
private:
    void mousePressEvent(QMouseEvent* event) override;
    void paintEvent(QPaintEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void CloudToContour(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, QString label);
    

private slots:
    void Open_clicked(); //�򿪵���
    void Save_clicked(); //�������

    void on_ViewYBtn_clicked();
    void on_ViewXBtn_clicked();
    void on_ViewZBtn_clicked();

    void Background_Select();
    void PressBtn_rendering();
    void Rendering_setting(QString data);
    void PressBtn_colorSelect();

    void PressBtn_PointCloud_PointSize_slider();
    void PointCloud_PointSize_SliderSetting(QString data);

    void on_FramePickBtn_clicked();
    void on_PointPickBtn_clicked();

    void on_ComfirmFramePickBtn_clicked();
    void on_ComfirmPointPickBtn_clicked();

    void GuassFilterAction();
    void DirectFilterAction();

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

    void SaveMat();
    void SaveSampleLabel();
signals:
    void MarkComplete();
    void ConversionBetween2Dand3D();

private:
    Ui::DepthToPCLClass ui;

    int totalHeight; //����ҳ�������߶�
    bool isKeyDown_shift; //ģ��shift�Ƿ���
    int point_size = 1; //���ƴ�С

    cv::Mat m_image;
    ImageLabel* cvImageWidget;
    VTKOpenGLNativeWidget* vtkWidget;

    pcl_view_select_color* dialog_colorselect;
    View_Render* dialog_render;
    PointCloud_PointSize_Set_Dialog* pointsize_set_dialog;

    Filter_Guass* dialog_Guass_filter;
    Filter_Direct* dialog_Direct_filter;

    //std::vector<cv::Mat> vImg; //����ͼƬ��Mat����
    //SampleLabel ContourVector; //����ͼƬ������������
    //std::vector<SampleLabel> vSampleLabel;

    te::AiInstSet allResultAiInstance;

    QVBoxLayout* labelVLayout;

    QStateMachine* m_pStateMachine;
    QState* TwoDState;
    QState* ThrDState;

    QString currentLabelNAme;

    float currentDisplayImageLength;
    float currentDisplayImageHeight;
};
