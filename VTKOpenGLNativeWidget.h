#pragma once
#include "pcl_function.h"
#include "Filter_Guass.h"
#include "Filter_Direct.h"

#include "teAiExTypes.h"

struct AxisSet {
    int curwidth;//��ǰ��ʾ�Ŀ��
    int curheight;//��ǰ��ʾ�ĸ߶�
    float OriginX;//����ԭ���X����
    float OriginY;//����ԭ���Y����
};

class VTKOpenGLNativeWidget  : public QVTKOpenGLNativeWidget
{
	Q_OBJECT

public:
	VTKOpenGLNativeWidget(QWidget *parent);
	~VTKOpenGLNativeWidget();

    void PCL_Initalization();
public:
    void ViewYBtn();
    void ViewXBtn();
    void ViewZBtn();
    void ComfirmFramePick();
    void ComfirmPointPick();
    void PolygonSelect(void* viewer_void, QString mode);
    bool LoadPointCloud(QString fileName);
    bool SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr saveCloud);
    bool SetBackgroundColor(QColor color);
    bool CoordinateAxisRendering(QString curaxis);
    bool PointCloudColorSet(QColor color);
    bool PointCloudPointSizeSet(int point_size);
    bool PointCloudHeightTransform(int factor);

    void pcl_filter_guass(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float paraA, float paraB, float paraC, float paraD);
    void pcl_filter_direct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float min, float max, QString axis, float is_save);

public:
    void AiInstance2Cloud(te::AiInstance* instance, cv::Mat& m_image, QColor color);

    /// <summary>
    /// ������Ⱦ3D����
    /// </summary>
    /// <param name="cloudin">��Ҫ��ʾ�ĵ���</param>
    void reRendering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin);

    void GetCoordinateSet();
protected:
    void Frame_PickingCallBack(const pcl::visualization::AreaPickingEvent& event, void* viewer_void);
    void Point_PickingCallBack(const pcl::visualization::PointPickingEvent& event, void* viewer_void);

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void); //�������ѡ����C�����ã��ٰ�Cȷ�Ͽ�ѡ����
    void keyboardEventInvert(const pcl::visualization::KeyboardEvent& event, void* viewer_void); //�����򷴿�ѡ����v�����ã��ٰ�vȷ�Ϸ���ѡ����
    void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* viewer_void); //�������ѡ����껭��

    void getScreentPos(double* displayPos, double* world, void* viewer_void);
    int inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y);
    
    void subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
    

public slots:
    void AxisAlignedBoundingBox();
    void OrientedBoundingBox();

    void GuassFilter(QString data1, QString data2, QString data3, QString data4);
    void DirectFilter(QString data1, QString data2, QString data3, QString data4);
    void LabelChanged(const QString& content, const QColor& fontColor);
signals:
    void PointCloudMarkingCompleted(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void CloudChanged();
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Point_clicked_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Frame_clicked_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_polygon;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Filter_out;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_marked;
    pcl::PointCloud<pcl::PointXYZ>::Ptr mediancloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkRenderWindow* m_renderWindow;
    bool isPickingMode = false;
    bool flag = false;//�ж��ǲ��ǵ�һ�ε��
    unsigned int line_id = 0;

    struct AxisSet axisset;
private:
    vtkSmartPointer<vtkRenderer> m_renderer;
    pcl::PointXYZ curP, lastP;
    
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    Filter_Guass* dialog_Guass_filter;
    Filter_Direct* dialog_Direct_filter;

    bool PositiveAndNegative_X_axis;
    bool PositiveAndNegative_Y_axis;
    bool PositiveAndNegative_Z_axis;

public:
    QString currentCategory;
    QColor currentColor;
};
