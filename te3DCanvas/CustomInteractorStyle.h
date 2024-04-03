#include "pcl_function.h"
#include <vtkTransform.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vector>

class CustomInteractorStyle :public vtkInteractorStyleTrackballCamera 
{
public:
	static CustomInteractorStyle* New();
	vtkTypeMacro(CustomInteractorStyle, vtkInteractorStyleTrackballCamera);

	void setRenderWindow(vtkRenderWindow* window, vtkSmartPointer<vtkRenderer> render, vtkSmartPointer<vtkAxesActor> axes);
	//virtual void OnMouseWheelForward()override;
	//virtual void OnMouseWheelBackward()override;
	//virtual void OnMiddleButtonDown()override;
	//virtual void OnMiddleButtonUp()override;
	virtual void OnMouseMove()override;
	virtual void OnLeftButtonDown() override;
	virtual void OnLeftButtonUp() override;
	void setRotationCenter(double x,double y,double z);

	using CallbackFunction = std::function<void()>;

	void SetCallback(CallbackFunction callback);

	void TriggerCallback();

protected:
	CustomInteractorStyle();
	~CustomInteractorStyle() override;

	void Dolly(double factor);
	bool IsCameraOutOfBounds(vtkCamera* camera);
	void DollyToPosition(double fact, int* position, vtkRenderer* renderer);
	void TranslateCamera(vtkRenderer* renderer, int toX, int toY, int fromX, int fromY);
	void rotateAroundAxis(double angle, std::vector<double>& axis, std::vector<double>* point);

	vtkRenderWindow* m_rendererwindow;
	vtkSmartPointer<vtkRenderer> m_renderer;

	vtkSmartPointer<vtkAxesActor> axes_actor;
	vtkSmartPointer<vtkTransform> axesTransform;

	std::vector<vtkActor*> m_pSelectedActor = { nullptr };
	vtkSmartPointer<vtkTransform> m_pRotationTransform = nullptr;
private:
	CustomInteractorStyle(const CustomInteractorStyle&) = delete;
	void operator=(const CustomInteractorStyle&) = delete;

	CallbackFunction callback_;

	bool Zoomflag = false;
	bool m_bLBtnDown = false;

	double MotionFactor;

	int m_nOldMousePosX;
	int m_nOldMousePosY;

	vtkProp3D* InteractionProp;
	
	double m_pickedPos[3] = { 0.0, 0.0, 0.0 }; // 初始化为0
	double rotationCenter[3] = { 0.0,0.0,0.0 }; // 旋转中心点

	cv::Mat mat;
	cv::Mat submatrix;
	std::vector<double> Actor_xAxis = { 1.0,0.0,0.0 };
	std::vector<double> Actor_yAxis = { 0.0,1.0,0.0 };
	std::vector<double> Axes_xAxis = { 1.0,0.0,0.0 };
	std::vector<double> Axes_yAxis = { 0.0,1.0,0.0 };
};
