#include "pcl_function.h"

class CustomInteractorStyle :public vtkInteractorStyleTrackballCamera {
public:
	static CustomInteractorStyle* New();
	vtkTypeMacro(CustomInteractorStyle, vtkInteractorStyleTrackballCamera);

	void setRenderWindow(vtkRenderWindow* window, vtkSmartPointer<vtkRenderer> render);
	virtual void OnMouseWheelForward()override;
	virtual void OnMouseWheelBackward()override;
protected:
	void Dolly(double factor);
	void DollyToPosition(double fact, int* position, vtkRenderer* renderer);
	void TranslateCamera(vtkRenderer* renderer, int toX, int toY, int fromX, int fromY);
private:
	bool Zoomflag = false;
	vtkRenderWindow* m_rendererwindow;
	vtkSmartPointer<vtkRenderer> m_renderer;
};
