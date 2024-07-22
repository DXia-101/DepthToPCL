#include "teInteractorStyle.h"
#include <vtkInteractorStyle.h>
//#include "RotateAroundAxis.h"
//#include "teStereoCamera.h"
#include "vtkCamera.h"
#include "vtkCellPicker.h"
#include "vtkCallbackCommand.h"
#include "vtkMath.h"
#include "vtkMatrix4x4.h"
#include "vtkObjectFactory.h"
#include "vtkProp3D.h"
#include <cmath>

namespace te {
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
}
using namespace te;
vtkStandardNewMacro(InteractorStyle);
InteractorStyle::InteractorStyle()
{
	menber = new InteractorMenber();
	menber->InteractionProp = nullptr;
}

InteractorStyle::~InteractorStyle()
{
}

void InteractorStyle::setRenderWindow(vtkRenderWindow* window, vtkSmartPointer<vtkRenderer> render, vtkSmartPointer<vtkAxesActor> axes)
{
	menber->renderwindow = window;
	menber->renderer = render;
	menber->axes_actor = axes;
}

void InteractorStyle::OnMouseWheelForward()
{
	Dolly(1.25);
}

void InteractorStyle::OnMouseWheelBackward()
{
	Dolly(0.80);
}

//void InteractorStyle::OnMouseMove()
//{
//	if (!menber->bLBtnDown) //没有按下鼠标左键
//	{
//		vtkInteractorStyleTrackballCamera::OnMouseMove();
//		return;
//	}
//
//	int X = this->Interactor->GetEventPosition()[0];
//	int Y = this->Interactor->GetEventPosition()[1];
//	double deltX = (X - menber->nOldMousePosX);
//	double deltY = (Y - menber->nOldMousePosY);
//	rotateAroundAxis(deltX, deltY, &menber->Actor_xAxis, &menber->Actor_yAxis);
//	vtkSmartPointer<vtkMatrix4x4> AxesMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
//
//	for (int i = 0; i < 3; ++i)
//	{
//		for (int j = 0; j < 3; ++j)
//		{
//			AxesMatrix->SetElement(i, j, menber->mat.at<double>(i, j));
//		}
//	}
//
//	if (menber->axes_actor != nullptr)
//	{
//		if (!menber->axesTransform)
//		{
//			menber->axesTransform = vtkSmartPointer<vtkTransform>::New();
//			menber->axesTransform->Identity();
//		}
//		double* axesCenter = menber->axes_actor->GetCenter();
//		menber->axesTransform->Translate(axesCenter[0], axesCenter[1], axesCenter[2]);
//		menber->axesTransform->Concatenate(AxesMatrix);
//		menber->axesTransform->Translate(-axesCenter[0], -axesCenter[1], -axesCenter[2]);
//		menber->axes_actor->SetUserTransform(menber->axesTransform);
//	}
//
//	if (!menber->rotationTransform)
//	{
//		menber->rotationTransform = vtkSmartPointer<vtkTransform>::New();
//		menber->rotationTransform->Identity();
//	}
//
//	menber->rotationTransform->Translate(menber->rotationCenter[0], menber->rotationCenter[1], menber->rotationCenter[2]);
//	menber->rotationTransform->Concatenate(AxesMatrix);
//	menber->rotationTransform->Translate(-menber->rotationCenter[0], -menber->rotationCenter[1], -menber->rotationCenter[2]);
//
//	for (int i = 0; i < menber->selectedActor.size(); ++i)
//	{
//		if (menber->selectedActor[i] == nullptr) {
//			continue;
//		}
//		menber->selectedActor[i]->SetUserTransform(menber->rotationTransform);
//	}
//	menber->nOldMousePosX = X;
//	menber->nOldMousePosY = Y;
//	menber->renderer->ResetCameraClippingRange();
//	this->Interactor->GetRenderWindow()->Render();
//}
//
//void InteractorStyle::OnLeftButtonDown()
//{
//	menber->bLBtnDown = true;
//
//	vtkSmartPointer<vtkPropCollection> propCollection = menber->renderer->GetViewProps();
//	propCollection->InitTraversal();
//	vtkProp* prop = nullptr;
//	vtkProp3D* pActor = nullptr;
//	int pickNum = propCollection->GetNumberOfItems();
//	menber->selectedActor.clear();
//	while (pickNum > 0)
//	{
//		prop = propCollection->GetNextProp();
//		pActor = vtkProp3D::SafeDownCast(prop);
//		if (pActor != nullptr)
//			menber->selectedActor.push_back(vtkActor::SafeDownCast(pActor));
//		pickNum--;
//	}
//	this->Interactor->GetRenderWindow()->Render();
//	menber->nOldMousePosX = this->Interactor->GetEventPosition()[0];
//	menber->nOldMousePosY = this->Interactor->GetEventPosition()[1];
//
//	vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
//}
//
//void InteractorStyle::OnLeftButtonUp()
//{
//	menber->bLBtnDown = false;
//	vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
//}

void InteractorStyle::OnRightButtonDown()
{
}

void InteractorStyle::OnRightButtonUp()
{
}

void InteractorStyle::setRotationCenter(double x, double y, double z)
{
	menber->rotationCenter[0] = x;
	menber->rotationCenter[1] = y;
	menber->rotationCenter[2] = z;
}

void InteractorStyle::setCallback(CallbackFunction callback)
{
	callback_ = callback;
}

void InteractorStyle::resetData()
{
	menber->Actor_xAxis = { 1.0,0.0,0.0 };
	menber->Actor_yAxis = { 0.0,1.0,0.0 };
	menber->Axes_xAxis = { 1.0,0.0,0.0 };
	menber->Axes_yAxis = { 0.0,1.0,0.0 };
	menber->rotationTransform = vtkSmartPointer<vtkTransform>::New();
	menber->rotationTransform->Identity();
	menber->axesTransform = vtkSmartPointer<vtkTransform>::New();
	menber->axesTransform->Identity();
}

vtkSmartPointer<vtkTransform> InteractorStyle::getRotationTransform()
{
	return menber->rotationTransform;
}

void InteractorStyle::Dolly(double factor)
{
	double initialPosition[3];
	vtkCamera* cam = menber->renderer->GetActiveCamera();
	cam->GetPosition(initialPosition);
	double oldFocalPoint[3];
	cam->GetFocalPoint(oldFocalPoint);

	DollyToPosition(factor, this->Interactor->GetEventPosition(), menber->renderer);

	if (isCameraOutOfBounds(cam))
	{
		cam->SetPosition(initialPosition);
		cam->SetFocalPoint(oldFocalPoint);
		triggerCallback();
	}
	menber->renderer->ResetCameraClippingRange();
	menber->renderwindow->Render();
}

bool InteractorStyle::isCameraOutOfBounds(vtkCamera* camera)
{
	double position[3];
	camera->GetPosition(position);

	if (position[2] < menber->rotationCenter[2])
	{
		return true;
	}

	return false;
}

void InteractorStyle::DollyToPosition(double fact, int* position, vtkRenderer* renderer)
{
	vtkCamera* cam = renderer->GetActiveCamera();

	// 相对于光标位置进行缩放
	double viewFocus[4], originalViewFocus[3], cameraPos[3], newCameraPos[3];
	double medianFocus[4];

	// 将焦点移动到光标位置
	cam->GetPosition(cameraPos);
	cam->GetFocalPoint(viewFocus);
	cam->GetFocalPoint(originalViewFocus);

	InteractorStyle::ComputeWorldToDisplay(
		renderer, viewFocus[0], viewFocus[1], viewFocus[2], viewFocus);

	InteractorStyle::ComputeDisplayToWorld(
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

void InteractorStyle::rotateAroundAxis(double dx, double dy, std::vector<double>* xAxis, std::vector<double>* yAxis)
{
	//if ((dx != 0 || dy != 0))
	//{
	//	double delta = pow((dx * dx + dy * dy), 0.5) * 0.1;
	//	cv::Point3d OutPoint = cv::Point3d(0, 0, 0);
	//	std::vector<double> baseAxis = {
	//		dx * xAxis->at(0) + dy * yAxis->at(0),
	//		dx * xAxis->at(1) + dy * yAxis->at(1),
	//		dx * xAxis->at(2) + dy * yAxis->at(2)
	//	};
	//	std::vector<double> zAxis = {
	//		(xAxis->at(1) * yAxis->at(2) - xAxis->at(2) * yAxis->at(1)),
	//		(xAxis->at(2) * yAxis->at(0) - xAxis->at(0) * yAxis->at(2)),
	//		(xAxis->at(0) * yAxis->at(1) - xAxis->at(1) * yAxis->at(0))
	//	};
	//	std::vector<double> rotateAxis = {
	//		(zAxis.at(1) * baseAxis.at(2) - zAxis.at(2) * baseAxis.at(1)),
	//		(zAxis.at(2) * baseAxis.at(0) - zAxis.at(0) * baseAxis.at(2)),
	//		(zAxis.at(0) * baseAxis.at(1) - zAxis.at(1) * baseAxis.at(0))
	//	};
	//	menber->mat = cv::Mat();
	//	RotateAroundAxis(cv::Point3d(yAxis->at(0), yAxis->at(1), yAxis->at(2)), cv::Point3d(0, 0, 0),
	//		cv::Vec3d(rotateAxis.at(0), rotateAxis.at(1), rotateAxis.at(2)), delta * (M_PI / 180.0),
	//		OutPoint, menber->mat);

	//	menber->mat = menber->mat(cv::Rect(0, 0, 3, 3));
	//	cv::invert(menber->mat, menber->submatrix);
	//	cv::Mat matrix1_mat(3, 1, CV_64F);

	//	matrix1_mat.at<double>(0, 0) = yAxis->at(0);
	//	matrix1_mat.at<double>(1, 0) = yAxis->at(1);
	//	matrix1_mat.at<double>(2, 0) = yAxis->at(2);

	//	cv::Mat result;
	//	result = menber->submatrix * matrix1_mat;

	//	yAxis->at(0) = result.at<double>(0, 0);
	//	yAxis->at(1) = result.at<double>(1, 0);
	//	yAxis->at(2) = result.at<double>(2, 0);

	//	matrix1_mat.at<double>(0, 0) = xAxis->at(0);
	//	matrix1_mat.at<double>(1, 0) = xAxis->at(1);
	//	matrix1_mat.at<double>(2, 0) = xAxis->at(2);

	//	result = menber->submatrix * matrix1_mat;

	//	xAxis->at(0) = result.at<double>(0, 0);
	//	xAxis->at(1) = result.at<double>(1, 0);
	//	xAxis->at(2) = result.at<double>(2, 0);
	//}
}

void InteractorStyle::triggerCallback()
{
	if (callback_)
	{
		callback_();
	}
}
