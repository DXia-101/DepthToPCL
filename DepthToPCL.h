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
    /// 初始化界面
    /// </summary>
    void Interface_Initialization();

    /// <summary>
    /// 初始化PCL
    /// </summary>
    void PCL_Initalization();

    /// <summary>
    /// 初始化状态机
    /// </summary>
    void InitStateMachine();
protected:
    void addAiInstance(DynamicLabel* curlabel, QList<QPolygonF>& Polygons);
    void addAiInstance(DynamicLabel* curlabel, pcl::PointCloud<pcl::PointXYZ>::Ptr markedCloud);

    /// <summary>
    /// 将当前标记的所有AiInstance转换为点云
    /// </summary>
    /// <param name="curlabel">当前标记</param>
    void AiInstSet2Cloud(DynamicLabel* curlabel);

    /// <summary>
    /// 将当前标记的所有AiInstance转换为多边形
    /// </summary>
    /// <param name="curlabel">当前标记</param>
    void AiInstSet2PolygonItem(DynamicLabel* curlabel);
    
private:
    void mousePressEvent(QMouseEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    /// <summary>
    /// 保存图像轮廓到标签当中
    /// </summary>
    /// <param name="Matin">待查找轮廓的图像</param>
    /// <param name="curlabel">保存轮廓的标签</param>
    void SaveMatContour2Label(cv::Mat& Matin, DynamicLabel* curlabel);

    /// <summary>
    /// 清除所有标记内容
    /// </summary>
    void ClearAllMarkedContent();

    bool isLabelExist(QString curlabel);

private slots:
    void on_addDynamicLabel_clicked(); //添加标记
    void on_delDynamicLabel_clicked(); //删除标记
    void on_startTagBtn_clicked(); //开始标记

    void on_changeFormBtn_clicked(); //转换界面按钮
    void on_drawCounterBtn_clicked(); //绘制原图轮廓按钮

    void on_drawMarkersBtn_clicked(); //绘制标记
    void on_clearMarkersBtn_clicked(); //清空所有标记

    void SaveContour();
    void LoadContour();

    void ReceiveMarkedPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void ReceiveMarkedPolygonItem(QList<QPolygonF>& Polygons);

    void WarningForUnselectedTags();

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

    int totalHeight; //滚动页面的总体高度

    int point_size = 1; //点云大小

    cv::Mat m_image;

    ImageLabel* teImageWidget;
    VTKOpenGLNativeWidget* vtkWidget;

    VTKToolBar* m_vtkToolBar;
    _3DMenuInterface* m_thrDMenuInterface;
    ImageDisplayToolBar* m_imageDisplayToolBar;

    QVBoxLayout* labelVLayout;
    QStateMachine* m_pStateMachine;
    QState* TwoDState;
    QState* ThrDState;

    QString currentLabelNAme;

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
