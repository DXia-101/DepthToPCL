#include "pcl_function.h"
#include <vtkTransform.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vector>
class CustomInteractorStyle :public vtkInteractorStyleTrackballCamera {
public:
	static CustomInteractorStyle* New();
	vtkTypeMacro(CustomInteractorStyle, vtkInteractorStyleTrackballCamera);

	void setRenderWindow(vtkRenderWindow* window, vtkSmartPointer<vtkRenderer> render);
	//virtual void OnMouseWheelForward()override;
	//virtual void OnMouseWheelBackward()override;
	//virtual void OnMiddleButtonDown()override;
	//virtual void OnMiddleButtonUp()override;
	virtual void OnMouseMove()override;
	virtual void OnLeftButtonDown() override;
	virtual void OnLeftButtonUp() override;
	void setRotationCenter(double x,double y,double z);
protected:
	CustomInteractorStyle();
	~CustomInteractorStyle() override;

	//void Dolly(double factor);
	//void DollyToPosition(double fact, int* position, vtkRenderer* renderer);
	//void TranslateCamera(vtkRenderer* renderer, int toX, int toY, int fromX, int fromY);

	double MotionFactor;

	vtkProp3D* InteractionProp;
	vtkCellPicker* InteractionPicker;

private:
	bool Zoomflag = false;
	vtkRenderWindow* m_rendererwindow;
	vtkSmartPointer<vtkRenderer> m_renderer;

	CustomInteractorStyle(const CustomInteractorStyle&) = delete;
	void operator=(const CustomInteractorStyle&) = delete;
	std::vector<vtkSmartPointer<vtkTransform>> m_pRotationTransform = { nullptr };
	bool m_bLBtnDown = false;
	int m_nOldMousePosX;
	int m_nOldMousePosY;
	std::vector<vtkActor*> m_pSelectedActor = { nullptr };
	double m_pickedPos[3] = { 0.0, 0.0, 0.0 }; // 初始化为0
	double rotationCenter[3] = { 0.0,0.0,0.0 }; // 旋转中心点
};
