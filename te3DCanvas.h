#pragma once
#include "pcl_function.h"
#include "Filter_Guass.h"
#include "Filter_Direct.h"

#include "teAiExTypes.h"
#include "te3DCanvasMember.h"


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

    void PolygonSelect(void* viewer_void);

    void pcl_filter_guass(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float paraA, float paraB, float paraC, float paraD);
    void pcl_filter_direct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float min, float max, QString axis, float is_save);
public:
    void AiInstance2Cloud(te::AiInstance* instance, cv::Mat& m_image, QColor color);
    void reRendering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin);
    struct AxisSet getAxisSet();
protected:
    void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* viewer_void); //不规则框选的鼠标画线

    void getScreentPos(double* displayPos, double* world, void* viewer_void);
    int inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y);
    
    void subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

    void SetCoordinateSet();
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

    void PointCloudHeightTransform(int factor);
    void te3DCanvasStartMarking();

    bool LoadPointCloud(QString fileName);
    bool SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr saveCloud);

    void reRenderOriginCloud();
signals:
    void sig_3DCanvasMarkingCompleted(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void CloudChanged();
private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Point_clicked_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Frame_clicked_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_polygon;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Filter_out;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_marked;

    pcl::PointXYZ curP, lastP; //画线
    
    pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkRenderWindow* m_renderWindow;
    vtkSmartPointer<vtkRenderer> m_renderer;

    Filter_Guass* dialog_Guass_filter;
    Filter_Direct* dialog_Direct_filter;
private:
    struct AxisSet axisset;
    struct te3DCanvasMember m_member;

    QString currentCategory;
    QColor currentColor;
};
