#pragma once

#include <QtWidgets/QWidget>
#include "ui_DepthToPCL.h"

#include "pcl_function.h"
#include "teAiExTypes.h"


#include "VTKOpenGLNativeWidget.h"
#include "ImageLabel.h"
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
    /// <summary>
    /// 添加标记到当前选择标签
    /// </summary>
    /// <param name="curlabel">当前选择标签</param>
    /// <param name="markedPolygon">标记的多边形</param>
    void addAiInstance(DynamicLabel* curlabel,GraphicsPolygonItem* markedPolygon);

    /// <summary>
    /// 添加标记到当前选择标签
    /// </summary>
    /// <param name="curlabel">当前选择标签</param>
    /// <param name="markedCloud">标记的点云</param>
    void addAiInstance(DynamicLabel* curlabel,pcl::PointCloud<pcl::PointXYZ>::Ptr markedCloud);

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
    void paintEvent(QPaintEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    /// <summary>
    /// 保存图像轮廓到标签当中
    /// </summary>
    /// <param name="Matin">待查找轮廓的图像</param>
    /// <param name="curlabel">保存轮廓的标签</param>
    void SaveMatContour2Label(cv::Mat& Matin, DynamicLabel* curlabel);

    /// <summary>
    /// 对轮廓覆盖到的图片部分进行提取
    /// </summary>
    /// <param name="imageToBeExtracted">待提取的图片</param>
    /// <param name="extractedContours">提取的轮廓</param>
    /// <param name="extractedImages">提取出的图片</param>
    void ExtractImages(QImage* imageToBeExtracted, GraphicsPolygonItem* extractedContours,cv::Mat* extractedImages);

    
    /// <summary>
    /// 清除所有标记内容
    /// </summary>
    void ClearAllMarkedContent();

private slots:
    void on_addDynamicLabel_clicked(); //添加标记
    void on_delDynamicLabel_clicked(); //删除标记
    void on_startTagBtn_clicked(); //开始标记

    void on_changeFormBtn_clicked(); //转换界面按钮
    void on_drawCounterBtn_clicked(); //绘制原图轮廓按钮

    void on_MarkCompleted_clicked(); //完成标记
    void on_drawMarkersBtn_clicked(); //绘制标记
    void on_clearMarkersBtn_clicked(); //清空所有标记

    void SaveContour();
    void LoadContour();

    void ReceiveMarkedPointClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void ReceiveMarkedPolygonItem(GraphicsPolygonItem* polygonItem);

    void WarningForUnselectedTags();

    void UpdatePointCloud2DImage();
signals:
    void ConversionBetween2Dand3D();

private:
    Ui::DepthToPCLClass ui;

    int totalHeight; //滚动页面的总体高度

    int point_size = 1; //点云大小

    cv::Mat m_image;
    ImageLabel* cvImageWidget;
    VTKOpenGLNativeWidget* vtkWidget;

    VTKToolBar* m_vtkToolBar;
    _3DMenuInterface* m_thrDMenuInterface;

    QVBoxLayout* labelVLayout;
    QStateMachine* m_pStateMachine;
    QState* TwoDState;
    QState* ThrDState;

    QString currentLabelNAme;

    QImage currentDisplayImage;
};
