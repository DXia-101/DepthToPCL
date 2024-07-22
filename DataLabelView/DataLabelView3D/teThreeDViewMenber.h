#pragma once

#include "pcl_function.h"
#include <map>
#include <QString>
#include <vector>

namespace te {
	class InteractorStyle;

	class ThreeDViewMenber
	{
	public:
		vtkSmartPointer<vtkRenderWindow> renderWindow;
		vtkSmartPointer<vtkRenderer> renderer;
		vtkSmartPointer<InteractorStyle> customInteractor;
		vtkSmartPointer<vtkOrientationMarkerWidget> markerWidget;
		vtkSmartPointer<vtkAxesActor> axes_actor;

		pcl::visualization::PCLVisualizer::Ptr viewer;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
		std::map<QString, std::vector<QString>> markerPCID;
		std::map<QString, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> markerPointCloud;
		std::map<QString, std::vector<QString>> resultPCID;
		std::map<QString, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>> resultPointCloud;

		enum ReRenderMode {
			ReSetCamera,
			NoSetCamera,
		};
	};
}
