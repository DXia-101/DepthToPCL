#pragma once

#include <QObject>
#include <QStackedLayout>
#include "pcl_function.h"

class teAiModel;
class teLabelBrowser;
class te3DCanvas;
class te3DCanvasMenu;
class te3DCanvasToolBar;
class te3DPolyLine;

class te3DCanvasController  : public QObject
{
	Q_OBJECT

public:
	te3DCanvasController(QObject* parent = nullptr);
	~te3DCanvasController();

	void displayToolBarInWidget(QVBoxLayout* layout);
	void displayCanvasInWidget(QStackedLayout* layout);
	void SavePointCloud(QString filepath, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcr);
	void setteAiModel(teAiModel*);
	void setteLabelBrowser(teLabelBrowser*);

	QRect getGeometry();

	void ManagePolyLine(QStackedLayout* layout);

	void NeedReload();

public slots:
	void hideAllUI();
	void showAllUI();
	void add3DAiInstance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
	void ShowAllItems();
	void LoadPointCloud(QString fileName);
	void ReLoadGTAndRST();
	void SetCentroid();
	void StartDrawPolyLine();
	void ReloadPointCloud();

signals:
	void sig_ManagePolyLine();
	void sig_setHeightCoefficientFactor(int factor);
	void sig_CurrentStateChanged(const QString& category, const QColor& fontColor, const int& index, const double& valThreshold, const double& invalThreshold);
	void sig_MarkerButtonRecovery();
	void sig_updateTrainWidget();
	void sig_ReLoadGTAndRST();
	void sig_MarkerComplete();

public:
	void ShowAllResults();
	void ShowAllMarkers();

private:
	te3DCanvas* m_te3DCanvas;
	te3DCanvasMenu* m_te3DCanvasMenu;
	te3DCanvasToolBar* m_te3DCanvasToolBar;
	teAiModel* m_teAiModel;
	teLabelBrowser* m_teLabelBrowser;
	te3DPolyLine* m_te3DPolyLine;

	bool IsNeedReload = false;
};
