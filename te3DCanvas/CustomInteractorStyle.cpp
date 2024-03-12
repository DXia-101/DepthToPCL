#include "CustomInteractorStyle.h"
#include <vtkInteractorStyle.h>

CustomInteractorStyle* CustomInteractorStyle::New() {
	CustomInteractorStyle* self = new CustomInteractorStyle();
	return self;
}

void CustomInteractorStyle::setRenderWindow(vtkRenderWindow* window, vtkSmartPointer<vtkRenderer> render)
{
	m_rendererwindow = window;
	m_renderer = render;
}

void CustomInteractorStyle::OnMouseWheelForward() {
	if (Zoomflag) {
		Dolly(1.1);
	}
	else {
		vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
	}
}

void CustomInteractorStyle::OnMouseWheelBackward() {
	if (Zoomflag) {
		Dolly(0.9);
	}
	else {
		vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
	}
}

void CustomInteractorStyle::Dolly(double factor)
{
	DollyToPosition(factor, this->Interactor->GetEventPosition(), m_renderer);

	//这一句必须加，不然图像不出现，得拖拽一下
	m_renderer->ResetCameraClippingRange();

	m_rendererwindow->Render();
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
