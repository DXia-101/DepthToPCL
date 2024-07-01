#pragma once
#include "pcl_function.h"

#include "teAiExTypes.h"
#include "te3DCanvasMember.h"
#include "CustomInteractorStyle.h"

#include <map>
#include <vector>

#define _Interactor_

class te3DCanvas  : public QVTKOpenGLNativeWidget
{
	Q_OBJECT

public:
	te3DCanvas(QWidget *parent = nullptr);
	~te3DCanvas();

    void PCL_Initalization();
    void VTKCoordinateAxis();
    void AutomaticallyAdjustCamera();
    void setRotationCenter();
    void reRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin, ReRenderMode mode);
public:
    void PerspectiveToYaxis();
    void PerspectiveToXaxis();
    void PerspectiveToZaxis();

    void pcl_filter_guass(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, float paraA, float paraB, float paraC, float paraD);
    void pcl_crossSection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, float min, float max, QString axis, float is_save);

    vtkRenderWindow* getvtkRenderWindow();
    vtkSmartPointer<vtkRenderer> getvtkRenderer();
public:
    void MarkersShowInCanvas(te::AiInstance* instance, cv::Mat& m_image, QColor color);
    void ResultsShowInCanvas(te::AiInstance* instance, cv::Mat& m_image, QColor color);
    
    struct AxisSet getAxisSet();
    std::vector<double> getCloudCentroid();

    void ClearmarkerPCID();
    void ClearresultPCID();
    void CloudToMarkers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
protected:
    static int inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Segment(const std::vector<QPointF>& pointSet);

    void subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2);
    
    vtkSmartPointer<vtkPolyData> createPlane(const pcl::ModelCoefficients& coefficients, float scale[2] = nullptr);

    void WorldToScreen(pcl::PointXYZRGB* input3D, vtkMatrix4x4* mat, double* output2D);
    void WorldToScreen(pcl::PointXYZRGB* input3D, vtkMatrix4x4* transform, vtkMatrix4x4* composit, double* output2D);
public slots:
    bool SetBackgroundColor(QColor color);
    bool CoordinateAxisRendering(QString curaxis);
    bool PointCloudColorSet(QColor color);
    bool PointCloudPointSizeSet(int point_size);

    void AxisAlignedBoundingBox();
    void OrientedBoundingBox();

    void GuassFilter(QString data1, QString data2, QString data3, QString data4);
    void CrossSection(QString data1, QString data2, QString data3, QString data4);
    void CurrentStateChanged(const QString& category, const QColor& fontColor, const int& index,const double& valThreshold,const double& invalThreshold);

    void HeightTransform(int factor);
    void te3DCanvasStartMarking(QVector<QPointF>& pointlist);

    bool LoadPointCloud(QString fileName);
    bool SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud);

    void reRenderOriginCloud(ReRenderMode mode);

    void ShowDimension(int arg);
    void ShowResult(int arg);
    void UpdateDimentsion();
    void UpdateResult();

    void ReductionPointCloud(QString fileName);
    void SetCoordinateSet();
signals:
    void sig_3DCanvasMarkingCompleted(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void sig_ShowAllItems();

private:
    std::map<QString,std::vector<QString>> markerPCID;
    std::map<QString,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> markerPointCloud;
    std::map<QString, std::vector<QString>> resultPCID;
    std::map<QString,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> resultPointCloud;

    vtkRenderWindow* m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkRenderer> m_Axes_renderer;
    
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    
    pcl::visualization::PCLVisualizer::Ptr viewer;

#ifdef _Interactor_
    CustomInteractorStyle* m_CustomInteractor;
#endif

    vtkSmartPointer<vtkOrientationMarkerWidget> markerWidget;
    vtkSmartPointer<vtkAxesActor> axes_actor;
    
private:
    struct AxisSet axisset;
    struct te3DCanvasMember m_member;
    struct currentState m_curstate;
};
