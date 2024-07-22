#pragma once
#include "pcl_function.h"
#include <vtkTransform.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

namespace te {
	class InteractorMenber;
	class InteractorStyle : public vtkInteractorStyleTrackballCamera
	{
	public:
		static InteractorStyle* New();
		vtkTypeMacro(InteractorStyle, vtkInteractorStyleTrackballCamera);

		void setRenderWindow(vtkRenderWindow* window, vtkSmartPointer<vtkRenderer> render, vtkSmartPointer<vtkAxesActor> axes);
	protected:
		virtual void OnMouseWheelForward()override;
		virtual void OnMouseWheelBackward()override;
		//virtual void OnMouseMove()override;
		//virtual void OnLeftButtonDown() override;
		//virtual void OnLeftButtonUp() override;
		virtual void OnRightButtonDown() override;
		virtual void OnRightButtonUp() override;

	public:
		void setRotationCenter(double x, double y, double z);
		using CallbackFunction = std::function<void()>;
		void setCallback(CallbackFunction callback);
		void resetData();
		vtkSmartPointer<vtkTransform> getRotationTransform();
	protected:
		InteractorStyle();
		~InteractorStyle() override;

		void Dolly(double factor);
		bool isCameraOutOfBounds(vtkCamera* camera);
		void DollyToPosition(double fact, int* position, vtkRenderer* renderer);
		void rotateAroundAxis(double dx, double dy, std::vector<double>* xAxis, std::vector<double>* yAxis);

		using CallbackFunction = std::function<void()>;
		void triggerCallback();
	private:
		InteractorStyle(const InteractorStyle&) = delete;
		void operator=(const InteractorStyle&) = delete;

		CallbackFunction callback_;

	private:
		InteractorMenber* menber;
	};

}