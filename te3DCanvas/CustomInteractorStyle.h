#include "pcl_function.h"
#include <vtkTransform.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vector>

//#define _CC_

class CustomInteractorStyle :public vtkInteractorStyleTrackballCamera 
{
public:
	static CustomInteractorStyle* New();
	vtkTypeMacro(CustomInteractorStyle, vtkInteractorStyleTrackballCamera);

	void setRenderWindow(vtkRenderWindow* window, vtkSmartPointer<vtkRenderer> render, vtkSmartPointer<vtkAxesActor> axes);

	std::vector<double>& getXActor();
	std::vector<double>& getYActor();

	virtual void OnMouseMove()override;
	virtual void OnLeftButtonDown() override;
	virtual void OnLeftButtonUp() override;
	virtual void OnRightButtonDown() override;
	virtual void OnRightButtonUp() override;
	void setRotationCenter(double x,double y,double z);

	using CallbackFunction = std::function<void()>;

	void SetCallback(CallbackFunction callback);

	void TriggerCallback();
	void ResetData();
protected:
	CustomInteractorStyle();
	~CustomInteractorStyle() override;

	void Dolly(double factor);
	bool IsCameraOutOfBounds(vtkCamera* camera);
	void DollyToPosition(double fact, int* position, vtkRenderer* renderer);
	void TranslateCamera(vtkRenderer* renderer, int toX, int toY, int fromX, int fromY);
	void rotateAroundAxis(double angle, std::vector<double>& axis, std::vector<double>* point);
	void rotateByQuaternion(double angle, std::vector<double>& axis, vtkSmartPointer<vtkTransform>& transform);
	
	vtkRenderWindow* m_rendererwindow;
	vtkSmartPointer<vtkRenderer> m_renderer;

	vtkSmartPointer<vtkAxesActor> axes_actor;
	vtkSmartPointer<vtkTransform> axesTransform;
public:
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

struct Quaternion {
    double w, x, y, z;

    Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {}

    // 从轴和角度生成四元数
	static Quaternion fromAxisAngle(const std::vector<double>& axis, double angle) {
		double halfAngle = angle / 2.0;
		double s = sin(halfAngle);
		return Quaternion(cos(halfAngle), axis[0] * s, axis[1] * s, axis[2] * s);
	}

	// 将四元数转换为 4x4 矩阵
	std::vector<std::vector<double>> toMatrix() const {
		std::vector<std::vector<double>> mat(4, std::vector<double>(4, 0));

		mat[0][0] = 1 - 2 * (y * y + z * z);
		mat[0][1] = 2 * (x * y - w * z);
		mat[0][2] = 2 * (x * z + w * y);
		mat[1][0] = 2 * (x * y + w * z);
		mat[1][1] = 1 - 2 * (x * x + z * z);
		mat[1][2] = 2 * (y * z - w * x);
		mat[2][0] = 2 * (x * z - w * y);
		mat[2][1] = 2 * (y * z + w * x);
		mat[2][2] = 1 - 2 * (x * x + y * y);
		mat[3][3] = 1;

		return mat;
	}
};