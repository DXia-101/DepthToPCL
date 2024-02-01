#ifndef TETABLEWIDGET_H
#define TETABLEWIDGET_H


#include<QKeyEvent>
#include<QResizeEvent>

#include"tewidget_global.h"
#include"qxttablewidget.h"


//Only For Debug
class TeTableWidgetItem:public QxtTableWidgetItem
{
public:

    ~TeTableWidgetItem()
    {
       qDebug()<<"Release Table Item";
    }

};


//写死了只能按行浏览
class  TeTableWidget:public QxtTableWidget
{
    Q_OBJECT

public:

    enum TeBrowseMode
    {
        E_BROWSE_FRAME=0,
        E_BROWSE_PAGE=E_BROWSE_FRAME+1,
    };


    TeTableWidget(QWidget* parent=nullptr,
                 TeBrowseMode eBackMode=E_BROWSE_PAGE,
                 TeBrowseMode eForwardMode=E_BROWSE_PAGE);


    ~TeTableWidget()override;

    int teGetItemNum(){return m_iItemNum;}

    void teSetArray(int iNum);

    int teGetArrayNum(){return m_iArrayNum;}

    void teSwitchItemByIndex(int iIndex);

    void teSwitchByHook(QKeyEvent * qevent)
    {
        keyPressEvent(qevent);
    }

    int teGetRowByIndex(int iIndex);

    int teGetItemIndex(int iRow){return m_piItemIndex[iRow];}

    int* teGetVisibleIndex(int& iNum);

    void teSetCurrentRow(int iRow)
    {
        setCurrentCell(iRow,0);

    }

signals:
    //piIndex表示要更新的Index
    void teUpDateItem(int* piIndex,int iLength);
    void teSwitchItem(int iIndex);

protected:

    int m_iItemNum=50;
    int m_iItemHeight=80;


    //E_BROWSE_FRAME E_BROWSE_PAGE

    TeBrowseMode m_eBackwardBrowseMode;//=E_BROWSE_PAGE;//向后浏览策略
    TeBrowseMode m_eForwardBrowseMode;//=E_BROWSE_PAGE;//向前浏览策略

    void keyPressEvent(QKeyEvent * qevent)override;

    void resizeEvent(QResizeEvent *event)override;

	void wheelEvent(QWheelEvent *event)override;

private:

    enum TeUpdateEvent
    {
        E_SWITCH_SET=0,//更新集合
        E_BROWSE_BACKWARD=E_SWITCH_SET+1,//向后浏览
        E_BROWSE_FORWARD=E_BROWSE_BACKWARD+1,//向前浏览
        E_RESIZE_WIDGET=E_BROWSE_FORWARD+1,//缩放控件
        E_SWITCH_INDEX=E_RESIZE_WIDGET+1,//查询特定Index
    };

    static const int iMaxItemNum=512;

    int m_iArrayNum=0;//真实总数
    int m_iVisibleItemNum=0;//可见的数量
    int m_piItemIndex[iMaxItemNum];//缩略图控件中缩略图Index


    typedef void(TeTableWidget::* pFuncUpdateIndex)(int);

    static pFuncUpdateIndex pUpDateIndexArray[];


    void teUpDateIndexBrowseBackWard(int iImgIndex=-1);
    void teUpDateIndexBrowseForWard(int iImgIndex=-1);
    void teUpDateIndexSwitchSet(int iImgIndex=-1);
    void teUpDateIndexResize(int iImgIndex=-1);
    void teUpDateSwitchIndex(int iImgIndex);

    void teUpdateItem(TeUpdateEvent eUpDateEvent,int iIndex=-1);


};


#endif // TETABLEWIDGET_H
