#pragma once
#include "pcl_function.h"
#include "Filter_Guass.h"
#include "Filter_Direct.h"

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
    
public:
    void PCL_Initalization();

    void PerspectiveToYaxis();
    void PerspectiveToXaxis();
    void PerspectiveToZaxis();

    void pcl_filter_guass(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float paraA, float paraB, float paraC, float paraD);
    void pcl_filter_direct(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, float min, float max, QString axis, float is_save);

    vtkRenderWindow* getvtkRenderWindow();
    vtkSmartPointer<vtkRenderer> getvtkRenderer();

    void setRotationCenter();
    void AutomaticallyAdjustCamera();
public:
    void MarkersShowInCanvas(te::AiInstance* instance, cv::Mat& m_image, QColor color);
    void ResultsShowInCanvas(te::AiInstance* instance, cv::Mat& m_image, QColor color);
    void reRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin, ReRenderMode mode);
    struct AxisSet getAxisSet();
    std::vector<double> getCloudCentroid();
protected:
    static int inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y);
    void PolygonSelect();

    void subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2);
    void VTKCoordinateAxis();
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
    void DirectFilter(QString data1, QString data2, QString data3, QString data4);
    void LabelChanged(const QString& content, const QColor& fontColor);

    void HeightTransform(int factor);
    void te3DCanvasStartMarking(QVector<QPointF>& pointlist);

    bool LoadPointCloud(QString fileName);
    bool SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud);

    void reRenderOriginCloud(ReRenderMode mode);

    void ShowDimension(int arg);
    void ShowResult(int arg);
    void UpdateDimentsion();
    void UpdateResult();

    void ReductionPointCloud();

    void SetCoordinateSet();
signals:
    void sig_3DCanvasMarkingCompleted(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void sig_ShowAllItems();

public:
    std::map<QString,std::vector<QString>> markerPCID;
    std::map<QString,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> markerPointCloud;
    std::map<QString, std::vector<QString>> resultPCID;
    std::map<QString,std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> resultPointCloud;


    vtkRenderWindow* m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;
    vtkSmartPointer<vtkRenderer> m_Axes_renderer;
    
public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Filter_out;

    pcl::visualization::PCLVisualizer::Ptr viewer;

#ifdef _Interactor_
    CustomInteractorStyle* m_CustomInteractor;
#endif
    Filter_Guass* dialog_Guass_filter;
    Filter_Direct* dialog_Direct_filter;

    vtkSmartPointer<vtkOrientationMarkerWidget> markerWidget;
    vtkSmartPointer<vtkAxesActor> axes_actor;
    
    QVector<QPointF> MarkerPointSet;
public:
    struct AxisSet axisset;
    struct te3DCanvasMember m_member;

    QString currentCategory;
    QColor currentColor;
};
