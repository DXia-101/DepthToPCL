#ifndef TESAMPWIDGET_H
#define TESAMPWIDGET_H

#include <QWidget>
#include <QTimer>
#include "tewidget_global.h"
#include <functional>
#include <algorithm>

namespace Ui {
class TeSampWidget;
}

class TeTableWidget;

class TeSampWidget : public QWidget
{
    Q_OBJECT

public:

	typedef std::function<QString(int)> FieldNameTable;

    explicit TeSampWidget(QWidget *parent = nullptr);
    ~TeSampWidget();

	void teSetField(FieldNameTable table, int iLength);

	void teShowField();

	void teShowField(int iField);

	void teHideField();

    void teUpDateSet(int iNum,int iLayerNum,bool bReset=true);

    //单个更新和一批更新一样(显示才更新)
    void teUpDateImg(int iIndex,const QStringList& qlstThumbnailPath,QSize& size,const QString& qstrImgName);

    void teSetSampleField(int iIndex,int iField,QString qstrValue, QColor qClolor);

	void teUpdateThumb(int iIndex, int iLayer,const QImage& qImg, TeImgFormat eFormat);

	bool teIsThumbNull(int iIndex,int iLayer);

	TeTableWidget* teBrowserTable();

    int teCurSampleIndex();

	int currentLayerIndex();


signals:
    //需要更新显示样本
    void sig_UpDateItem(int* piIndex,int iLength);
    //样本属性更新了
    void sig_SamplePropChange(int iIndex,int iField);

    void sig_BatchSamplePropChange(int iField);
    //切换图片
    void sig_SwitchImg(int iIndex,int iLayerIndex=0);

	//更新当前可见的项目
	void sig_ItemActive(int* piIndex, int iLength);

protected:
	bool eventFilter(QObject *watched, QEvent *event)override;

private slots:
    void on_toolButton_clicked();

    void on_m_NumBox_editingFinished();

	void activeTimeout();

private:
    
    Ui::TeSampWidget *ui;

	enum ChangeReason
	{
		E_User,
		E_Update,
		E_Box,
		E_ScrollBar,
	};

	ChangeReason m_eChangeReason = E_User;

	FieldNameTable m_NameTable = [](int iIndex)->QString {return QString();};

	QTimer m_ActiveTimer;

	int m_iFocusWidget = 0;
};

#endif // TESAMPWIDGET_H
