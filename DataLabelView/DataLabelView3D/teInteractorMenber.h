#pragma once
#include "pcl_function.h"

class InteractorMenber
{
public:
	vtkRenderWindow* m_rendererwindow;
	vtkSmartPointer<vtkRenderer> m_renderer;

	vtkSmartPointer<vtkAxesActor> axes_actor;
	vtkSmartPointer<vtkTransform> axesTransform;

	std::vector<vtkActor*> m_pSelectedActor = { nullptr };
	vtkSmartPointer<vtkTransform> m_pRotationTransform = nullptr;

	bool m_bLBtnDown = false;

	int m_nOldMousePosX;
	int m_nOldMousePosY;

	vtkProp3D* InteractionProp;

	cv::Mat mat;
	cv::Mat submatrix;

	double rotationCenter[3] = { 0.0,0.0,0.0 };
	std::vector<double> Actor_xAxis = { 1.0,0.0,0.0 };
	std::vector<double> Actor_yAxis = { 0.0,1.0,0.0 };
	std::vector<double> Axes_xAxis = { 1.0,0.0,0.0 };
	std::vector<double> Axes_yAxis = { 0.0,1.0,0.0 };
};