#include "CustomInteractorStyle.h"
#include <vtkInteractorStyle.h>

#include "RotateAroundAxis.h"
#include "teStereoCamera.h"

#include "vtkCamera.h"
#include "vtkCellPicker.h"
#include "vtkCallbackCommand.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkProp3D.h"
#include <cmath>
vtkStandardNewMacro(CustomInteractorStyle);

void CustomInteractorStyle::setRenderWindow(vtkRenderWindow* window, vtkSmartPointer<vtkRenderer> render, vtkSmartPointer<vtkAxesActor> axes)
{
	m_rendererwindow = window;
	m_renderer = render;
	axes_actor = axes;
}

//void CustomInteractorStyle::OnMouseWheelForward() 
//{
//	if (Zoomflag) {
//		Dolly(1.12);
//	}
//	else {
//		vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
//	}
//}
//
//void CustomInteractorStyle::OnMouseWheelBackward() 
//{
//	if (Zoomflag) {
//		Dolly(0.88);
//	}
//	else {
//		vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
//	}
//}
//
//void CustomInteractorStyle::OnMiddleButtonDown()
//{
//	vtkInteractorStyleTrackballCamera::OnMiddleButtonDown();
//}
//
//void CustomInteractorStyle::OnMiddleButtonUp()
//{
//	vtkInteractorStyleTrackballCamera::OnMiddleButtonUp();
//}

void CustomInteractorStyle::rotateAroundAxis(double angle, std::vector<double>& axis, std::vector<double>* point)
{
	cv::Point3d OutPoint = cv::Point3d(0, 0, 0);
	mat = cv::Mat();
	RotateAroundAxis(cv::Point3d(point->at(0), point->at(1), point->at(2)), cv::Point3d(0, 0, 0),
		cv::Vec3d(axis.at(0), axis.at(1), axis.at(2)), cv::saturate_cast<double>(angle * CV_PI / 180.0),
		OutPoint, mat);

	mat = mat(cv::Rect(0, 0, 3, 3));
	cv::invert(mat, submatrix);
	cv::Mat matrix1_mat(3, 1, CV_64F);

	matrix1_mat.at<double>(0, 0) = point->at(0);
	matrix1_mat.at<double>(1, 0) = point->at(1);
	matrix1_mat.at<double>(2, 0) = point->at(2);

	cv::Mat result;
	result = submatrix * matrix1_mat;

	point->at(0) = result.at<double>(0, 0);
	point->at(1) = result.at<double>(1, 0);
	point->at(2) = result.at<double>(2, 0);
}

void CustomInteractorStyle::OnMouseMove()
{
	if (!m_bLBtnDown) //没有按下鼠标左键
	{
		vtkInteractorStyleTrackballCamera::OnMouseMove();
		return;
	}
	
	int X = this->Interactor->GetEventPosition()[0];
	int Y = this->Interactor->GetEventPosition()[1];
	double deltX = (X - m_nOldMousePosX)*0.1;
	double deltY = (Y - m_nOldMousePosY)*0.1;
	if (abs(deltX) > 10 || abs(deltY) > 10)
	{
		m_nOldMousePosX = X;
		m_nOldMousePosY = Y;
	}

	if (axes_actor != nullptr)
	{
		if (!axesTransform)
		{
			axesTransform = vtkSmartPointer<vtkTransform>::New();
			axesTransform->Identity();
		}
		double xAngle, yAngle, zAngle;

		double* axesCenter = axes_actor->GetCenter();
		axesTransform->Translate(axesCenter[0], axesCenter[1], axesCenter[2]);

		rotateAroundAxis(-deltY,Axes_xAxis ,&Axes_yAxis);
		teCalcEulerAngle(mat, E_ORDER_XYZ, &xAngle, &yAngle, &zAngle);
		axesTransform->RotateX(xAngle);
		axesTransform->RotateY(yAngle);
		axesTransform->RotateZ(zAngle);
		
		rotateAroundAxis(deltX,Axes_yAxis, &Axes_xAxis);
		teCalcEulerAngle(mat, E_ORDER_XYZ, &xAngle, &yAngle, &zAngle);
		axesTransform->RotateX(xAngle);
		axesTransform->RotateY(yAngle);
		axesTransform->RotateZ(zAngle);
		
		axesTransform->Translate(-axesCenter[0], -axesCenter[1], -axesCenter[2]);
		axes_actor->SetUserTransform(axesTransform);
	}
	
	if (!m_pRotationTransform)
	{
		m_pRotationTransform = vtkSmartPointer<vtkTransform>::New();
		m_pRotationTransform->Identity();
	}

	m_pRotationTransform->Translate(rotationCenter[0], rotationCenter[1], rotationCenter[2]);

	double xAngle, yAngle, zAngle;

	rotateAroundAxis(-deltY, Actor_xAxis, &Actor_yAxis);
	teCalcEulerAngle(mat, E_ORDER_XYZ, &xAngle, &yAngle, &zAngle);

	m_pRotationTransform->RotateX(xAngle);
	m_pRotationTransform->RotateY(yAngle);
	m_pRotationTransform->RotateZ(zAngle);

	rotateAroundAxis(deltX, Actor_yAxis, &Actor_xAxis);
	teCalcEulerAngle(mat, E_ORDER_XYZ, &xAngle, &yAngle, &zAngle);

	m_pRotationTransform->RotateX(xAngle);
	m_pRotationTransform->RotateY(yAngle);
	m_pRotationTransform->RotateZ(zAngle);

	m_pRotationTransform->Translate(-rotationCenter[0], -rotationCenter[1], -rotationCenter[2]);

	for (int i = 0; i < m_pSelectedActor.size(); ++i)
	{
		if (m_pSelectedActor[i] == nullptr) {
			continue;
		}
		m_pSelectedActor[i]->SetUserTransform(m_pRotationTransform);
	}
	m_nOldMousePosX = X;
	m_nOldMousePosY = Y;
	m_renderer->ResetCameraClippingRange();
	this->Interactor->GetRenderWindow()->Render();
}

void CustomInteractorStyle::OnLeftButtonDown()
{
	m_bLBtnDown = true;

	vtkSmartPointer<vtkPropCollection> propCollection = m_renderer->GetViewProps();
	propCollection->InitTraversal();
	vtkProp* prop = nullptr;
	vtkProp3D* pActor = nullptr;
	int pickNum = propCollection->GetNumberOfItems();
	m_pSelectedActor.clear();
	while (pickNum > 0)
	{
		prop = propCollection->GetNextProp();
		pActor = vtkProp3D::SafeDownCast(prop);
		m_pSelectedActor.push_back(vtkActor::SafeDownCast(pActor));
		pickNum--;
	}
	this->Interactor->GetRenderWindow()->Render();
	m_nOldMousePosX = this->Interactor->GetEventPosition()[0];
	m_nOldMousePosY = this->Interactor->GetEventPosition()[1];

	vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void CustomInteractorStyle::OnLeftButtonUp()
{
	m_bLBtnDown = false;
	vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

void CustomInteractorStyle::setRotationCenter(double x, double y, double z)
{
	rotationCenter[0] = x;
	rotationCenter[1] = y;
	rotationCenter[2] = z;
}

CustomInteractorStyle::CustomInteractorStyle()
{
	this->InteractionProp = nullptr;
}

CustomInteractorStyle::~CustomInteractorStyle()
{
}

void CustomInteractorStyle::Dolly(double factor)
{
	double initialPosition[3];
	vtkCamera* cam = m_renderer->GetActiveCamera();
	cam->GetPosition(initialPosition);
	double oldFocalPoint[3];
	cam->GetFocalPoint(oldFocalPoint);

	DollyToPosition(factor, this->Interactor->GetEventPosition(), m_renderer);

	if (IsCameraOutOfBounds(cam))
	{
		cam->SetPosition(initialPosition);
		cam->SetFocalPoint(oldFocalPoint);
		TriggerCallback();
	}
	m_renderer->ResetCameraClippingRange();
	m_rendererwindow->Render();
}

bool CustomInteractorStyle::IsCameraOutOfBounds(vtkCamera* camera)
{
	double position[3];
	camera->GetPosition(position);

	if (position[2] < rotationCenter[2])
	{
		return true;
	}

	return false;
}

void CustomInteractorStyle::DollyToPosition(double fact, int* position, vtkRenderer* renderer)
{
	vtkCamera* cam = renderer->GetActiveCamera();
	if (cam->GetParallelProjection())
	{
		int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
		// 相对于光标缩放
		int* aSize = renderer->GetRenderWindow()->GetSize();
		int w = aSize[0];
		int h = aSize[1];
		x0 = w / 2;
		y0 = h / 2;
		x1 = position[0];
		y1 = position[1];
		TranslateCamera(renderer, x0, y0, x1, y1);
		cam->SetParallelScale(cam->GetParallelScale() / fact);
		TranslateCamera(renderer, x1, y1, x0, y0);
	}
	else
	{
		// 相对于光标位置进行缩放
		double viewFocus[4], originalViewFocus[3], cameraPos[3], newCameraPos[3];
		double newFocalPoint[4], norm[3];

		// 将焦点移动到光标位置
		cam->GetPosition(cameraPos);
		cam->GetFocalPoint(viewFocus);
		cam->GetFocalPoint(originalViewFocus);
		cam->GetViewPlaneNormal(norm);

		CustomInteractorStyle::ComputeWorldToDisplay(
			renderer, viewFocus[0], viewFocus[1], viewFocus[2], viewFocus);

		CustomInteractorStyle::ComputeDisplayToWorld(
			renderer, double(position[0]), double(position[1]), viewFocus[2], newFocalPoint);

		cam->SetFocalPoint(newFocalPoint);

		// 沿投影方向移入/移出相机
		cam->Dolly(fact);

		// 寻找新的焦点
		cam->GetPosition(newCameraPos);

		double newPoint[3];
		newPoint[0] = originalViewFocus[0] + newCameraPos[0] - cameraPos[0];
		newPoint[1] = originalViewFocus[1] + newCameraPos[1] - cameraPos[1];
		newPoint[2] = originalViewFocus[2] + newCameraPos[2] - cameraPos[2];

		cam->SetFocalPoint(newPoint);
	}
}

void CustomInteractorStyle::TranslateCamera(vtkRenderer* renderer, int toX, int toY, int fromX, int fromY)
{
	vtkCamera* cam = renderer->GetActiveCamera();
	double viewFocus[4], focalDepth, viewPoint[3];
	double newPickPoint[4], oldPickPoint[4], motionVector[3];
	cam->GetFocalPoint(viewFocus);

	CustomInteractorStyle::ComputeWorldToDisplay(
		renderer, viewFocus[0], viewFocus[1], viewFocus[2], viewFocus);
	focalDepth = viewFocus[2];

	CustomInteractorStyle::ComputeDisplayToWorld(
		renderer, double(toX), double(toY), focalDepth, newPickPoint);
	CustomInteractorStyle::ComputeDisplayToWorld(
		renderer, double(fromX), double(fromY), focalDepth, oldPickPoint);

	//摄像机运动反向
	motionVector[0] = oldPickPoint[0] - newPickPoint[0];
	motionVector[1] = oldPickPoint[1] - newPickPoint[1];
	motionVector[2] = oldPickPoint[2] - newPickPoint[2];

	cam->GetFocalPoint(viewFocus);
	cam->GetPosition(viewPoint);
	cam->SetFocalPoint(motionVector[0] + viewFocus[0], motionVector[1] + viewFocus[1], motionVector[2] + viewFocus[2]);
	cam->SetPosition(motionVector[0] + viewPoint[0], motionVector[1] + viewPoint[1], motionVector[2] + viewPoint[2]);
}

void CustomInteractorStyle::SetCallback(CallbackFunction callback)
{
	callback_ = callback;
}

void CustomInteractorStyle::TriggerCallback()
{
	if (callback_)
	{
		callback_();
	}
}

void CustomInteractorStyle::ResetData()
{
	Actor_xAxis = { 1.0,0.0,0.0 };
	Actor_yAxis = { 0.0,1.0,0.0 };
	Axes_xAxis = { 1.0,0.0,0.0 };
	Axes_yAxis = { 0.0,1.0,0.0 };
	m_pRotationTransform = vtkSmartPointer<vtkTransform>::New();
	m_pRotationTransform->Identity();
	axesTransform = vtkSmartPointer<vtkTransform>::New();
	axesTransform->Identity();
}
