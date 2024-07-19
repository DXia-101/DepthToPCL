#pragma once

#include <QObject>
#include <QMap>
#include <QString>
#include <QVector>
#include <QPointF>
#include <any>
#include <memory>
#include "pcl_function.h"
#include "teTrainParaRegister.h"
#include "teTestParaRegister.h"
#include "teAiExTypes.h"
#include "teMatrix.h"
#include "teGraphicsItemV2.h"

namespace te {
	class Model;

	enum CoordinateAxisDirection
	{
		PositiveXaxis,
		NegativeXaxis,
		PositiveYaxis,
		NegativeYaxis,
		PositiveZaxis,
		NegativeZaxis,
	};

	class TrainStatisticsViewMenber
	{
	public:
		int iteration;
		float fAvgLoss;
		float fPosAcc;
	};

	class ViewModel : public QObject
	{
		Q_OBJECT

	public:
		ViewModel(QObject* parent = nullptr);
		~ViewModel();

	public:
		enum TypeWidget {
			ReceptiveFieldView,
			ThreeDMarkView,
			ThreeDView,
			TwoDView,
		};

		enum updateMode {
			StartMark,
			ThreeDViewMenu,
			ReceptiveField,
			updateTrainCount,
			updateResultCount,
			ThreeDMark,
			StateChange,
			HideThreeDMark,
			ShowTrainStateChart,
			Segement,
			OutOfBounds,
			UnCheckTrainBox,
			TrainStatistics,
			InitPointCloud,
			SetCameraPosition,
			AutoAdjustCamera,
			MarkerPointCloudVisible,
			ResultPointCloudVisible,
			UpdateMarkerPointCloud,
			UpdateResultPointCloud,
			AddMarkerPointCloud,
			ShowMarkerPointCloud,
			ShowResultPointCloud,
			CrossSection,
			PointCloudHeight,
			BackgroundColor,
			PointCloudColor,
			AxisRender,
			PointCloudPointSize,
			AABB,
			OBB,
			TrainPara,
			TrainCurveBox,
			TestPara,
		};

	public:
		void setModel(Model*);

	public:
		void loadTrainingImages(const QStringList& filePaths);
		void prepareTrain(std::string);
		void prepareTest(std::string);
		void stopTrain();

		QColor getLabelColor(const QString&);
		std::pair<QString, QColor> getCurrentLabelInfo();
		void changeCurrentLabelInfo(const QString&, const QColor&);
		void addLabelInfo(const QString&, const QColor&);
		QMap<QString, int> getTrainMarkerCount();
		QMap<QString, int> getTestResultCount();
		QString getCurrentLabel();
		void setCurrentLabel(QString);
		void removeLabelInfo(QString);

		void setThreshold(std::pair<double, double>);

		void add3DAiInstance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void add2DAiInstance(QList<te::GraphicsItem*> polygonItems);

		void trainModel();
		void testModel();
		void initTrainConfig(TrainParaRegister* para);
		void initTestConfig(TestParaRegister* para);

		double getCurrentInvalidPointThreshold();
		double getCurrentValidPointThreshold();

		std::string getSelectShrinkageChart(int index);
		std::string getCurrentShrinkageChart();
		std::string getSelectOriginImage(int index);
		std::string getCurrentOriginImage();
		std::string getSelectPointCloud(int index);
		std::string getCurrentPointCloud();
		std::string getSelectTrainGt(int index);
		std::string getSelectResultGt(int index);
		SampleMark getSelectTrainSampleInfo(int index);
		SampleMark getSelectResultSampleInfo(int index);
		SampleMark getCurrentTrainSampleInfo();
		SampleMark getCurrentResultSampleInfo();

		bool updateShrinkageChart(int index, std::string& filepath);
		bool updatePointCloud(int index, std::string& filepath);
		bool updateTrainGtFilePath(int index, std::string& filepath);
		bool updateResultGtFilePath(int index, std::string& filepath);

		bool clearCurrentTrainSampleMark();
		bool clearAllTestSampleMark();
		bool clearAllPointCloud();
		bool clearAllShrinkageChart();
		void DropAllTables();

		void getTrainSamples(std::vector<SampleInfo>* trainSamples);
		void getResultSamples(std::vector<SampleInfo>* resultSamples);

		void segment();
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr coordinateAxisRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void extractingPointCloudsBasedOnContours(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void axisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void orientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2);
		void perspectiveToAxis(pcl::PointXYZ maxPt, pcl::PointXYZ minPt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void pcl_crossSection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudByContour(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, AiInstance*);

	public:
		int getCurrentIndex();
		void setCurrentIndex(int);
		int getCurrentDisplayImageLength();
		void setCurrentDisplayImageLength(int);
		int getCurrentDisplayImageHeight();
		void setCurrentDisplayImageHeight(int);
		pcl::PointXYZ getCurrentMaxPt();
		void setCurrentMaxPt(pcl::PointXYZ);
		pcl::PointXYZ getCurrentMinPt();
		void setCurrentMinPt(pcl::PointXYZ);
		enum CoordinateAxisDirection getCoordinateAxisDire();
		void setCoordinateAxisDire(CoordinateAxisDirection);
		int getThreeDisShowMarkers();
		void setThreeDisShowMarkers(int);
		int getThreeDisShowResults();
		void setThreeDisShowResults(int);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudToSegmented();
		void setPointCloudToSegmented(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
		vtkSmartPointer<vtkMatrix4x4> getCompositeProjectionTransform();
		void setCompositeProjectionTransform(vtkSmartPointer<vtkMatrix4x4>);
		vtkSmartPointer<vtkMatrix4x4> getThreeDTransmat();
		void setThreeDTransmat(vtkSmartPointer<vtkMatrix4x4>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSegmentedPointCloud();
		void setSegmentedPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
		double getPointCloudHeight();
		void setPointCloudHeight(double);
		QColor getPointCloudBackgroundColor();
		void setPointCloudBackgroundColor(QColor);
		QColor getPointCloudColor();
		void setPointCloudColor(QColor);
		int getPointCloudPointSize();
		void setPointCloudPointSize(int);
		QString getRenderAxis();
		void setRenderAxis(QString);
		enum TypeWidget getCurrentWidgetType();
		void setCurrentWidgetType(TypeWidget);
		float getReceptiveField();
		void setReceptiveField(float);
		TestParaRegister getTestPara();
		void setTestPara(TestParaRegister);
		TrainParaRegister getTrainPara();
		void setTrainPara(TrainParaRegister);
		int getTrainStatisticsState();
		void setTrainStatisticsState(int);
		TrainStatisticsViewMenber getTrainStateMenber();
		void setTrainStateMenber(TrainStatisticsViewMenber);
		QVector<QPointF> getThreeDMarkerPolygon();
		void setThreeDMarkerPolygon(QVector<QPointF>);
		std::vector<int> getVtkWindowSize();
		void setVtkWindowSize(std::vector<int>);
		std::vector<double> getRenderViewport();
		void setRenderViewport(std::vector<double>);

	private:
		void setTrainStatisticsCallback(ViewModel& ptr);

	signals:
		void notified(updateMode);

	private:
		Model* model;
	};

}