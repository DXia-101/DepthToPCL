#pragma once
#include "pcl_function.h"
#include "Filter_Guass.h"
#include "Filter_Direct.h"
#include "DynamicLabel.h"

class VTKOpenGLNativeWidget  : public QVTKOpenGLNativeWidget
{
	Q_OBJECT

public:
	VTKOpenGLNativeWidget(QWidget *parent);
	~VTKOpenGLNativeWidget();

    void PCL_Initalization();

    void ViewYBtn();
    void ViewXBtn();
    void ViewZBtn();
    void ComfirmFramePick();
    void ComfirmPointPick();
    void projectInliers(void* viewer_void, QString mode);

protected:
    void Frame_PickingCallBack(const pcl::visualization::AreaPickingEvent& event, void* viewer_void);
    void Point_PickingCallBack(const pcl::visualization::PointPickingEvent& event, void* viewer_void);

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void); //�������ѡ����C�����ã��ٰ�Cȷ�Ͽ�ѡ����
    void keyboardEventInvert(const pcl::visualization::KeyboardEvent& event, void* viewer_void); //�����򷴿�ѡ����v�����ã��ٰ�vȷ�Ϸ���ѡ����
    void mouseEventOccurred(const pcl::visualization::MouseEvent& event, void* viewer_void); //�������ѡ����껭��

    void getScreentPos(double* displayPos, double* world, void* viewer_void);
    int inOrNot1(int poly_sides, double* poly_X, double* poly_Y, double x, double y);
    
    void subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);
    
    void pcl_filter_guass(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float paraA, float paraB, float paraC, float paraD);
    void pcl_filter_direct(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, float min, float max, QString axis, float is_save);
public slots:
    void AxisAlignedBoundingBox();
    void OrientedBoundingBox();

    void GuassFilter(QString data1, QString data2, QString data3, QString data4);
    void DirectFilter(QString data1, QString data2, QString data3, QString data4);
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Point_clicked_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Frame_clicked_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_polygon;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cliped;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_Filter_out;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    vtkRenderWindow* m_renderWindow;
    bool isPickingMode = false;
    bool flag = false;//�ж��ǲ��ǵ�һ�ε��
    unsigned int line_id = 0;

    DynamicLabel* currentdynamicLabel;  //��ǰ��ǩ����
private:
    vtkSmartPointer<vtkRenderer> m_renderer;
    pcl::PointXYZ curP, lastP;
    
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    Filter_Guass* dialog_Guass_filter;
    Filter_Direct* dialog_Direct_filter;
};
