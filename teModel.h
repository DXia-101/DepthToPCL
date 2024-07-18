#pragma once
#include <unordered_map>
#include <variant>
#include <memory>
#include <any>
#include <QMap>
#include "pcl_function.h"
#include "teAiExTypes.h"
#include "teMatrix.h"
#include "teGraphicsItemV2.h"
#include "teTrainParaRegister.h"
#include "teTestParaRegister.h"
#include "teTraining.h"
#include "teModelMenber.h"

#include"teDataTypeRegistration.h"

TE_BEGIN_NAMESPACE

RTTR_REGISTRATION
{
	TE_REGISTER_TYPE(ConnectedRegion)
	TE_REGISTER_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(ConnectedRegion, polygons)
	TE_REGISTER_END

	TE_REGISTER_TYPE(AiInstance)
	TE_REGISTER_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(AiInstance, id)
	TE_REGISTER_PROPERTY(AiInstance, area)
	TE_REGISTER_PROPERTY(AiInstance, angle)
	TE_REGISTER_PROPERTY(AiInstance, probit)
	TE_REGISTER_PROPERTY(AiInstance, name)
	TE_REGISTER_PROPERTY(AiInstance, contour)
	TE_REGISTER_END

	TE_REGISTER_TYPE(SampleMark)
	TE_REGISTER_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(SampleMark, gtDataSet)
	TE_REGISTER_PROPERTY(SampleMark, localMask)
	TE_REGISTER_END
}

TE_END_NAMESPACE

namespace te
{
	class IDataStore;
	class ModelMenber;
	class Model
	{
	public:
		Model(std::unique_ptr<IDataStore> dataStore);
		~Model();
	public:
		//Ai训练相关
		void trainModel();
		void testModel();
		void initTrainConfig(TrainParaRegister* para);
		void initTestConfig(TestParaRegister* para);
		void setmodelpath(std::string modelpath);
		void stopTrain();
		Training getTrainHandle();

	private:
		static int teException(void* pParam, AiStatus eStatus);
		static void teTrainStateCallBack(AiStatus status, TrainState& stateinfo, void* param);
		static void teAiInferResult(AiResult& inferResult, DynamicMatrix& hotmap, void* pCallbackParam);

		bool isImageAlreadyExist(std::string filepath);
		void initThreasholds(int size);
		bool updateTrainSampleMark(int index, SampleMark& samplemark);
		bool updateResultSampleMark(int index, SampleMark& samplemark);
		bool updateCurrentTrainSampleMark(SampleMark& samplemark);
		bool updateCurrentResultSampleMark(SampleMark& samplemark);

		int getCurrentIndex();
		int getCurrentLoadImageNum();
		void setCurrentLoadImageNum(int);

	public:
		//数据管理
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
		QMap<QString, int> getCurrentTrainMarksNumber();
		QMap<QString, int> getCurrentResultMarksNumber();

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

		void updateInvalidPointThreshold(double threshold);
		double getSelectInvalidPointThreshold(int index);
		double getCurrentInvalidPointThreshold();
		void addInvalidPointThreshold(int index, double threshold);

		void updateValidPointThreshold(double threshold);
		double getSelectValidPointThreshold(int index);
		double getCurrentValidPointThreshold();
		void addValidPointThreshold(int index, double threshold);

		//当前工作相关
		void loadTrainingImages(const QStringList& filePaths);

		//与训练状态统计图相关
		using CallbackFunction = std::function<void(int, float, float)>;
		void setCallback(CallbackFunction callback);
		void triggerCallback(int iteration, float fAvgLoss, float fPosAcc);

		//3DCanvas计算相关
		bool savePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud);
		void segment(double* clipRange, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin, vtkMatrix4x4* transmat);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr coordinateAxisRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void extractingPointCloudsBasedOnContours(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void worldToScreen(int* windowsize, double* viewport, pcl::PointXYZRGB* input3D, vtkMatrix4x4* mat, double* output2D);
		void worldToScreen(int* windowsize, double* viewport, pcl::PointXYZRGB* input3D, vtkMatrix4x4* transform, vtkMatrix4x4* composit, double* output2D);
		void axisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void orientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2);
		void perspectiveToAxis(pcl::PointXYZ maxPt, pcl::PointXYZ minPt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void pcl_crossSection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointCloudByContour(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, AiInstance*);

		//标记相关
		void add3DAiInstance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
		void add2DAiInstance(QList<te::GraphicsItem*> polygonItems);

		void addLabelInfo(const QString&, const QColor&);
		void changeLabelInfo(const QString&, const QColor&);
		void removeLabelInfo(const QString&);
		std::pair<QString, QColor> getCurrentLabelInfo();
		QColor getLabelColor(const QString&);

		void setThrDMarkPointList(QVector<QPointF>);
		QVector<QPointF> getThrDMarkPointList();
	public:
		//与训练状态统计图相关
		using CallbackFunction = std::function<void(int, float, float)>;
		void SetCallback(CallbackFunction callback);
		void TriggerCallback(int iteration, float fAvgLoss, float fPosAcc);

	public:
		template<typename T>
		void setData(const std::string& id, T&& value) {
			storage[id] = std::make_any<std::decay_t<T>>(std::forward<T>(value));
		}

		template<typename T>
		bool getData(std::string id, T& outValue) const {
			auto it = storage.find(id);
			if (it != storage.end()) {
				try {
					outValue = std::any_cast<T>(it->second);
					return true;
				}
				catch (const std::bad_any_cast&) {
					return false;
				}
			}
			return false;
		}

	private:
		std::unique_ptr<IDataStore> m_dataStore;

		std::unordered_map<std::string, std::any> storage;
		QMap<QString, QColor> labelstore;
		CallbackFunction callback_;

		ModelMenber* menber;

	};
}