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

CustomInteractorStyle::CustomInteractorStyle()
{
	this->InteractionProp = nullptr;
}

CustomInteractorStyle::~CustomInteractorStyle()
{
}

void CustomInteractorStyle::setRenderWindow(vtkRenderWindow* window, vtkSmartPointer<vtkRenderer> render, vtkSmartPointer<vtkAxesActor> axes)
{
	m_rendererwindow = window;
	m_renderer = render;
	axes_actor = axes;
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

void CustomInteractorStyle::rotateAroundAxis(double dx, double dy, std::vector<double>* xAxis, std::vector<double>* yAxis)
{
	if ((dx != 0 || dy != 0))
	{
		double delta = pow((dx * dx + dy * dy), 0.5) * 0.1;
		cv::Point3d OutPoint = cv::Point3d(0, 0, 0);
		std::vector<double> baseAxis = {
			dx * xAxis->at(0) + dy * yAxis->at(0),
			dx * xAxis->at(1) + dy * yAxis->at(1),
			dx * xAxis->at(2) + dy * yAxis->at(2)
		};
		std::vector<double> zAxis = {
			(xAxis->at(1) * yAxis->at(2) - xAxis->at(2) * yAxis->at(1)),
			(xAxis->at(2) * yAxis->at(0) - xAxis->at(0) * yAxis->at(2)),
			(xAxis->at(0) * yAxis->at(1) - xAxis->at(1) * yAxis->at(0))
		};
		std::vector<double> rotateAxis = {
			(zAxis.at(1) * baseAxis.at(2) - zAxis.at(2) * baseAxis.at(1)),
			(zAxis.at(2) * baseAxis.at(0) - zAxis.at(0) * baseAxis.at(2)),
			(zAxis.at(0) * baseAxis.at(1) - zAxis.at(1) * baseAxis.at(0))
		};
		mat = cv::Mat();
		RotateAroundAxis(cv::Point3d(yAxis->at(0), yAxis->at(1), yAxis->at(2)), cv::Point3d(0, 0, 0),
			cv::Vec3d(rotateAxis.at(0), rotateAxis.at(1), rotateAxis.at(2)), delta * (M_PI / 180.0),
			OutPoint, mat);

		mat = mat(cv::Rect(0, 0, 3, 3));
		cv::invert(mat, submatrix);
		cv::Mat matrix1_mat(3, 1, CV_64F);

		matrix1_mat.at<double>(0, 0) = yAxis->at(0);
		matrix1_mat.at<double>(1, 0) = yAxis->at(1);
		matrix1_mat.at<double>(2, 0) = yAxis->at(2);

		cv::Mat result;
		result = submatrix * matrix1_mat;

		yAxis->at(0) = result.at<double>(0, 0);
		yAxis->at(1) = result.at<double>(1, 0);
		yAxis->at(2) = result.at<double>(2, 0);

		matrix1_mat.at<double>(0, 0) = xAxis->at(0);
		matrix1_mat.at<double>(1, 0) = xAxis->at(1);
		matrix1_mat.at<double>(2, 0) = xAxis->at(2);

		result = submatrix * matrix1_mat;

		xAxis->at(0) = result.at<double>(0, 0);
		xAxis->at(1) = result.at<double>(1, 0);
		xAxis->at(2) = result.at<double>(2, 0);
	}
}

#ifndef _CC_
void CustomInteractorStyle::OnMouseMove()
{
	if (!m_bLBtnDown) //没有按下鼠标左键
	{
		vtkInteractorStyleTrackballCamera::OnMouseMove();
		return;
	}
	
	int X = this->Interactor->GetEventPosition()[0];
	int Y = this->Interactor->GetEventPosition()[1];
	double deltX = (X - m_nOldMousePosX);
	double deltY = (Y - m_nOldMousePosY);
	rotateAroundAxis(deltX, deltY, &Actor_xAxis, &Actor_yAxis);
	vtkSmartPointer<vtkMatrix4x4> AxesMatrix = vtkSmartPointer<vtkMatrix4x4>::New();

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			AxesMatrix->SetElement(i, j, mat.at<double>(i, j));
		}
	}

	if (axes_actor != nullptr)
	{
		if (!axesTransform)
		{
			axesTransform = vtkSmartPointer<vtkTransform>::New();
			axesTransform->Identity();
		}
		double* axesCenter = axes_actor->GetCenter();
		axesTransform->Translate(axesCenter[0], axesCenter[1], axesCenter[2]);
		axesTransform->Concatenate(AxesMatrix);
		axesTransform->Translate(-axesCenter[0], -axesCenter[1], -axesCenter[2]);
		axes_actor->SetUserTransform(axesTransform);
	}
	
	if (!m_pRotationTransform)
	{
		m_pRotationTransform = vtkSmartPointer<vtkTransform>::New();
		m_pRotationTransform->Identity();
	}

	m_pRotationTransform->Translate(rotationCenter[0], rotationCenter[1], rotationCenter[2]);
	m_pRotationTransform->Concatenate(AxesMatrix);
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
#else
void CustomInteractorStyle::OnMouseMove()
{
	if (!m_bLBtnDown) // 没有按下鼠标左键
	{
		vtkInteractorStyleTrackballCamera::OnMouseMove();
		return;
	}

	int X = this->Interactor->GetEventPosition()[0];
	int Y = this->Interactor->GetEventPosition()[1];
	double deltX = (X - m_nOldMousePosX) * 0.1;
	double deltY = (Y - m_nOldMousePosY) * 0.1;

	// 获取当前相机的视图方向
	vtkSmartPointer<vtkCamera> camera = m_renderer->GetActiveCamera();
	double viewUp[3];
	camera->GetViewUp(viewUp);
	double focalPoint[3];
	camera->GetFocalPoint(focalPoint);
	double position[3];
	camera->GetPosition(position);

	double viewDirection[3];
	for (int i = 0; i < 3; ++i)
	{
		viewDirection[i] = focalPoint[i] - position[i];
	}
	vtkMath::Normalize(viewDirection);

	// 计算左右翻转的轴（与视图方向和viewUp向量垂直）
	double rightAxis[3];
	vtkMath::Cross(viewDirection, viewUp, rightAxis);
	vtkMath::Normalize(rightAxis);

	if (!m_pRotationTransform)
	{
		m_pRotationTransform = vtkSmartPointer<vtkTransform>::New();
		m_pRotationTransform->Identity();
	}

	// 处理前后翻转（绕右轴旋转）
	rotateByQuaternion(-deltY, std::vector<double>{rightAxis[0], rightAxis[1], rightAxis[2]}, m_pRotationTransform);

	// 处理左右翻转（绕viewUp轴旋转）
	rotateByQuaternion(deltX, std::vector<double>{viewUp[0], viewUp[1], viewUp[2]}, m_pRotationTransform);

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
#endif

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
		if(pActor != nullptr)
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

void CustomInteractorStyle::OnRightButtonDown()
{
}

void CustomInteractorStyle::OnRightButtonUp()
{
}

void CustomInteractorStyle::setRotationCenter(double x, double y, double z)
{
	rotationCenter[0] = x;
	rotationCenter[1] = y;
	rotationCenter[2] = z;
}

void CustomInteractorStyle::OnMouseWheelForward()
{
	Dolly(1.12);
}

void CustomInteractorStyle::OnMouseWheelBackward()
{
	Dolly(0.88);
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

	// 相对于光标位置进行缩放
	double viewFocus[4], originalViewFocus[3], cameraPos[3], newCameraPos[3];
	double medianFocus[4];

	// 将焦点移动到光标位置
	cam->GetPosition(cameraPos);
	cam->GetFocalPoint(viewFocus);
	cam->GetFocalPoint(originalViewFocus);

	CustomInteractorStyle::ComputeWorldToDisplay(
		renderer, viewFocus[0], viewFocus[1], viewFocus[2], viewFocus);

	CustomInteractorStyle::ComputeDisplayToWorld(
		renderer, double(position[0]), double(position[1]), viewFocus[2], medianFocus);

	cam->SetFocalPoint(medianFocus);

	cam->Dolly(fact);

	cam->GetPosition(newCameraPos);

	double newFocus[3];
	newFocus[0] = originalViewFocus[0] + newCameraPos[0] - cameraPos[0];
	newFocus[1] = originalViewFocus[1] + newCameraPos[1] - cameraPos[1];
	newFocus[2] = originalViewFocus[2];

	cam->SetFocalPoint(newFocus);
}