#pragma once

#include "pcl_function.h"
#include "teViewModel.h"
#include <memory>

namespace te {
	class ThreeDViewMenber;
	class ThreeDView : public QVTKOpenGLNativeWidget
	{
		Q_OBJECT

	public:
		ThreeDView(QWidget* parent = nullptr);
		~ThreeDView();

	public:
		void outOfBounds();

	protected:
		void RerenderWindow();

	private:
		void initWidget();
		void initCoordinateAxis();
		void initPointCloud();

		void setRotationCenter();
		void setCoordinateInfo();
		void setCameraPosition();
		void adjustCamera();
		void adjustPointCloudTransform();
		void setMarkerPointCloudVisible();
		void setResultPointCloudVisible();

		void updateMarkerPointCloud();
		void updateResultPointCloud();
		void addMarkerPointCloud();
		void showMarkerPointCloud();
		void showResultPointCloud();
		void clearMarkerPointCloud();
		void clearResultPointCloud();
		void clearMarkerVector();
		void clearResultVector();

		void crossSection();
		void setPointCloudHeight();
		void setBackgroundColor();
		void setPointCloudColor();
		void setAxisRender();
		void setPointSize();
		void setAxisAlignedBoundingBox();
		void setOrientedBoundingBox();

		void saveSegmentPara();

	private:
		std::vector<double> getCloudCentroid();

	private:
		void setInteractorCallback(ThreeDView& ptr);

	public:
		void bindViewModel(std::shared_ptr<ViewModel>);

	protected slots:
		void refresh(ViewModel::updateMode);

	private:
		std::weak_ptr<ViewModel> viewModel;
		ThreeDViewMenber* menber;
	};

}
