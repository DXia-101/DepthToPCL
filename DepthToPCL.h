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

#include "ImageDisplayToolBar.h"
#include "ParameterDesignWidget.h"
#include "TestParameterSetting.h"

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
protected:
    void addAiInstance(QList<QPolygonF>& Polygons);
    void addAiInstance(pcl::PointCloud<pcl::PointXYZ>::Ptr markedCloud);
    
private:
    void keyPressEvent(QKeyEvent* event) override;

private:
    void SaveMatContour2Label(cv::Mat& Matin, QString LabelName);// 保存图像轮廓到标签当中

    void ClearAllMarkedContent();
private slots:
    void on_startTagBtn_clicked(); //开始标记

    void on_changeFormBtn_clicked(); //转换界面按钮
    void on_drawCounterBtn_clicked(); //绘制原图轮廓按钮

    void on_drawMarkersBtn_clicked(); //绘制标记
    void on_clearMarkersBtn_clicked(); //清空所有标记

    void SaveContour();
    void LoadContour();

    void ReceiveMarkedPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void ReceiveMarkedPolygonItem(QList<QPolygonF>& Polygons);

    void UpdatePointCloud2DImage();

    void LoadTrainingImages();
    void StartedTrainAction();
    void StopTrainAction();
    void StartTestAction();

    void SwitchDisplayItem(int iIndex, int iLayerIndex = 0);
    void EndTest();
    void DrawTestMarkers();
    void SelectTestMarkColor();
signals:
    void ConversionBetween2Dand3D();
    void LoadingImagesCompleted();
    
private:
    Ui::DepthToPCLClass ui;

    int point_size = 1; //点云大小

    cv::Mat m_image;

    ImageLabel* teImageWidget;
    VTKOpenGLNativeWidget* vtkWidget;

    VTKToolBar* m_vtkToolBar;
    _3DMenuInterface* m_thrDMenuInterface;
    ImageDisplayToolBar* m_imageDisplayToolBar;

    ParameterDesignWidget* m_parameterDesignWidget;
    TestParameterSetting* m_TestParameterSetting;

    QStateMachine* m_pStateMachine;
    QState* TwoDState;
    QState* ThrDState;

    te::Image currentDisplayImage;

    std::vector<te::SampleInfo> vTrainSamples;
    std::vector<QString> TiffData;
    std::vector<QString> GTData;
    AiModelInterface* workAiModel;
    

    QStringList m_lstImgs;
    int currentIndex;

    int SumPixNum;

    QPushButton* DrawTestContour;
    ContoursSet* TestContoursSet;
    QColor TestMarkColor;
};
