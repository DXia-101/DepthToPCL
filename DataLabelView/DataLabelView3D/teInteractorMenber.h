#pragma once
#include "pcl_function.h"

class InteractorMenber
{
public:
	vtkRenderWindow* renderwindow;
	vtkSmartPointer<vtkRenderer> renderer;

	vtkSmartPointer<vtkAxesActor> axes_actor;
	vtkSmartPointer<vtkTransform> axesTransform;

	std::vector<vtkActor*> selectedActor = { nullptr };
	vtkSmartPointer<vtkTransform> rotationTransform = nullptr;

	bool bLBtnDown = false;

	int nOldMousePosX;
	int nOldMousePosY;

	vtkProp3D* InteractionProp;

	cv::Mat mat;
	cv::Mat submatrix;

	double rotationCenter[3] = { 0.0,0.0,0.0 };
	std::vector<double> Actor_xAxis = { 1.0,0.0,0.0 };
	std::vector<double> Actor_yAxis = { 0.0,1.0,0.0 };
	std::vector<double> Axes_xAxis = { 1.0,0.0,0.0 };
	std::vector<double> Axes_yAxis = { 0.0,1.0,0.0 };
};