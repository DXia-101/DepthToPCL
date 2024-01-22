#pragma once
#define PCL_FUNCTION_H

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkCamera.h>
#include <vtkAutoInit.h>

#include <pcl/common/io.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/sample_consensus/method_types.h> //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h> //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/mouse_event.h>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>