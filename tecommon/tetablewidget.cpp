#include"tetablewidget.h"
#include<QHeaderView>
#include<QApplication>

TeTableWidget::pFuncUpdateIndex TeTableWidget::pUpDateIndexArray[]=
{
    &TeTableWidget::teUpDateIndexSwitchSet,
    &TeTableWidget::teUpDateIndexBrowseBackWard,
    &TeTableWidget::teUpDateIndexBrowseForWard,
    &TeTableWidget::teUpDateIndexResize,
    &TeTableWidget::teUpDateSwitchIndex,
};


TeTableWidget::TeTableWidget(QWidget* parent,TeBrowseMode eBackMode,
                             TeBrowseMode eForwardMode):QxtTableWidget(parent),
                                                      m_eBackwardBrowseMode(eBackMode),
                                                      m_eForwardBrowseMode(eForwardMode)
{

    setSelectionBehavior(QAbstractItemView::SelectRows);
    setSelectionMode(QAbstractItemView::SingleSelection);

    setRowCount(m_iItemNum);


    for(int i=0;i<iMaxItemNum;i++)
        m_piItemIndex[i]=-1;

    connect(this,&QTableWidget::currentCellChanged,this,
            [this](int currentRow, int currentColumn, int previousRow, int previousColumn)
    {
        //行变化
        if(currentRow!=previousRow)
        {
            int iRow=currentRow;

            if(m_piItemIndex[iRow]>=0)
                emit teSwitchItem(m_piItemIndex[iRow]);
        }

    });


}


TeTableWidget::~TeTableWidget()
{

}

void TeTableWidget::teSetArray(int iNum)
{
    m_iArrayNum=iNum;
    this->teUpdateItem(E_SWITCH_SET);


    if(m_iVisibleItemNum>0&&m_iArrayNum>0)
    {
        int iSrcRow=currentRow();
        if(iSrcRow==0)
        {//qDebug()<<" Emit 1";
            if(m_piItemIndex[0]>=0)
                emit teSwitchItem(m_piItemIndex[0]);
        }
        else
        {
            this->teSetCurrentRow(0);
        }
    }

}

void TeTableWidget::teSwitchItemByIndex(int iIndex)
{
//    int iFirst=qMin(iIndex,m_iThumbnailSyllogeNum-m_iVisibleCapacityNum);
//    this->teUpdateThumbnail(E_SWITCH_INDEX,iIndex);

    int iFirst=qMin(iIndex,qMax(0,m_iArrayNum-m_iVisibleItemNum));
    this->teUpdateItem(E_SWITCH_INDEX,iIndex);

    int iSrcRow=this->currentRow();
    if(iSrcRow==iIndex-iFirst)
    {//qDebug()<<" Switch Index Bug";
        //qDebug()<<" Emit 5";
        if(m_piItemIndex[iIndex-iFirst]>=0)
            emit teSwitchItem(m_piItemIndex[iIndex-iFirst]);
    }
    else
    {
        this->teSetCurrentRow(iIndex-iFirst);
    }

}

int TeTableWidget::teGetRowByIndex(int iIndex)
{
    int iRow=-1;

    int iVisibleLength=0;
    int* piIndex=teGetVisibleIndex(iVisibleLength);

    int* it = std::find(piIndex, piIndex + iVisibleLength,iIndex);
    if(it!=piIndex + iVisibleLength){//是否在可见区域内
         iRow=iIndex-piIndex[0];
    }

    return iRow;

}


int* TeTableWidget::teGetVisibleIndex(int& iNum)
{
    int iIndexLength=0;
    for(int i=0;i<m_iVisibleItemNum;i++)
    {
        if(-1<m_piItemIndex[i]&&m_piItemIndex[i]<m_iArrayNum)
        {
            iIndexLength++;
        }
    }
    iNum=iIndexLength;
    return m_piItemIndex;

}

void TeTableWidget::keyPressEvent(QKeyEvent * qevent)
{
    switch (qevent->key())
    {
    case(Qt::Key_Up):
    {

        if(0==currentRow()&&m_piItemIndex[0]>0)//没翻到头，更换项目
        {
            /*
             * 逐帧模式退到第0行
             * 翻页模式退到可见最后一行和当前m_piItemIndex[0]-1的最小值（为尽可能多的显示项目，不一定退到最后一行）
             */
            int iRow=(m_eBackwardBrowseMode==E_BROWSE_FRAME)?0:qMin(m_iVisibleItemNum,m_piItemIndex[0])-1;
            this->teUpdateItem(E_BROWSE_BACKWARD);

            int iSrcRow=this->currentRow();
            if(iSrcRow==iRow)//项目没变Index变了
            {//qDebug()<<" Row Change Bug";
                //qDebug()<<" Emit 4";
                if(m_piItemIndex[iRow]>=0)
                    emit teSwitchItem(m_piItemIndex[iRow]);
            }
            else
            {
                this->teSetCurrentRow(iRow);
            }

        }
        else
        {   //qDebug()<<"TeList KeyPress"<<qevent;
            QxtTableWidget::keyPressEvent(qevent);
        }


        break;
    }
    case(Qt::Key_Down):
    {

        if(currentRow()>=m_iVisibleItemNum-1&&qMax(currentRow()+1-m_iVisibleItemNum,0)
           +m_piItemIndex[m_iVisibleItemNum-1]<m_iArrayNum-1)//当前行>=可见项目且没切到头
        {


            int iNext=m_piItemIndex[m_iVisibleItemNum-1]+qMax(currentRow()+1-m_iVisibleItemNum,0)+1;
            int iFirst=(m_eForwardBrowseMode==E_BROWSE_FRAME)?iNext-m_iVisibleItemNum+1:
                        qMin(iNext,m_iArrayNum-m_iVisibleItemNum);

            /*
             * 逐帧模式切到可见行最后一行
             * 翻页模式切到0行或NextIndex+可见数量-总数量行（尽可能多的显示项目，不一定切到第0行）
             */
            int iRow=(m_eForwardBrowseMode==E_BROWSE_FRAME)?m_iVisibleItemNum-1:iNext-iFirst;
            this->teUpdateItem(E_BROWSE_FORWARD);

            int iSrcRow=this->currentRow();
            if(iSrcRow==iRow)
            {//qDebug()<<" Row Change Bug";
                //qDebug()<<" Emit 3";
                if(m_piItemIndex[iRow]>=0)
                    emit teSwitchItem(m_piItemIndex[iRow]);
            }
            else
            {
                this->teSetCurrentRow(iRow);
            }

        }
        else
        {   //qDebug()<<"TeList KeyPress"<<qevent;
            QxtTableWidget::keyPressEvent(qevent);
        }


        break;
    }

    default:break;

    }

}

void TeTableWidget::resizeEvent(QResizeEvent *event)
{

    if((event->size().height()-horizontalHeader()->size().height())/
        m_iItemHeight!=m_iVisibleItemNum)
    {
        int iItemLength=event->size().height()-horizontalHeader()->size().height();
        m_iVisibleItemNum=qMin(iItemLength/m_iItemHeight,
                               m_iItemNum);
        //qDebug()<<"Visible Num"<<m_iVisibleCapacityNum;
        this->teUpdateItem(E_RESIZE_WIDGET);

    }

    QxtTableWidget::resizeEvent(event);

}

void TeTableWidget::wheelEvent(QWheelEvent *event)
{
	int iEventNum= qAbs(event->delta()) / 120;
    int iKey = (event->delta() > 0) ? Qt::Key_Up : Qt::Key_Down;
	QKeyEvent keyEvent(QEvent::KeyPress,iKey,Qt::NoModifier);

	setFocus();

	for(int i=0;i< iEventNum;i++)
		QApplication::sendEvent(this, &keyEvent);
}

/*
逐帧浏览时First为当前Index[0]-1
翻页浏览时First为当前Index[0]-可见数量（为负数时Index取0，
下一行为Index[0]-1与可见数量的最小值，以尽可能多显示项目）
*/
void TeTableWidget::teUpDateIndexBrowseBackWard(int iIndex)
{
    Q_UNUSED(iIndex)

    if (0 < m_piItemIndex[0])
    {

        int iFirst=(m_eBackwardBrowseMode==E_BROWSE_FRAME)?
                    m_piItemIndex[0]-1:qMax(m_piItemIndex[0] - m_iVisibleItemNum,0);//缩放时若当前Index被隐藏，左切不连续，就这么地吧
        for(int i=0;i<m_iVisibleItemNum;i++)
        {
            if(iFirst+i<m_iArrayNum)
                m_piItemIndex[i]=iFirst+i;
            else
                m_piItemIndex[i]=-1;
        }

        for(int i=m_iVisibleItemNum;i<m_iItemNum;i++)
            m_piItemIndex[i]=-1;

    }

}

/*
NextIndex为当前Index[0]+可见数量+当前行-可见数量（当前行被隐藏时）
逐帧浏览时First为NextIndex-可见数量
翻页浏览时First为NextIndex与总数-可见数量的最小值（以尽可能多的显示项目），下一行为第0行或NextIndex+可见数量-总数
*/

void TeTableWidget::teUpDateIndexBrowseForWard(int iIndex)
{
    Q_UNUSED(iIndex)

    if (m_piItemIndex[0]+ m_iVisibleItemNum < m_iArrayNum)
    {
        /*为尽可能多的显示项目，NextIndex不一定等于FirstIndex
         *qMax(currentRow()+1-m_iVisibleItemNum,0)：当前行可能不可见
         */

        int iNext=m_piItemIndex[m_iVisibleItemNum-1]+qMax(currentRow()+1-m_iVisibleItemNum,0)+1;
        int iFirst=(m_eForwardBrowseMode==E_BROWSE_FRAME)?iNext-m_iVisibleItemNum+1:
                    qMin(iNext,m_iArrayNum-m_iVisibleItemNum);

        for(int i=0;i<m_iVisibleItemNum;i++)
        {
            if(iFirst+i<m_iArrayNum)
                m_piItemIndex[i]=iFirst+i;
            else
                m_piItemIndex[i]=-1;
        }

        for(int i=m_iVisibleItemNum;i<m_iItemNum;i++)
            m_piItemIndex[i]=-1;
    }


}

void TeTableWidget::teUpDateIndexSwitchSet(int iIndex)
{
    Q_UNUSED(iIndex)

    int iFirst=0;
    for(int i=0;i<m_iVisibleItemNum;i++)
    {
        if(iFirst+i<m_iArrayNum)
            m_piItemIndex[i]=iFirst+i;
        else
            m_piItemIndex[i]=-1;
    }

    for(int i=m_iVisibleItemNum;i<m_iItemNum;i++)
        m_piItemIndex[i]=-1;

}

void TeTableWidget::teUpDateIndexResize(int iIndex)
{
    Q_UNUSED(iIndex)

    int iFirst=qMax(m_piItemIndex[0],0);
    for(int i=0;i<m_iVisibleItemNum;i++)
    {
        if(iFirst+i<m_iArrayNum)
            m_piItemIndex[i]=iFirst+i;
        else
            m_piItemIndex[i]=-1;
    }

    for(int i=m_iVisibleItemNum;i<m_iItemNum;i++)
        m_piItemIndex[i]=-1;
}

void TeTableWidget::teUpDateSwitchIndex(int iIndex)
{
    if(iIndex>-1&&iIndex<m_iArrayNum)
    {
        int iFirst=qMin(iIndex,qMax(0,m_iArrayNum-m_iVisibleItemNum));
        //int iFirst=iImgIndex;
        for(int i=0;i<m_iVisibleItemNum;i++)
        {
            if(iFirst+i<m_iArrayNum)
                m_piItemIndex[i]=iFirst+i;
            else
                m_piItemIndex[i]=-1;
        }

        for(int i=m_iVisibleItemNum;i<m_iItemNum;i++)
            m_piItemIndex[i]=-1;

    }

}



void TeTableWidget::teUpdateItem(TeUpdateEvent eUpDateEvent,int iIndex)
{

    (this->*pUpDateIndexArray[eUpDateEvent])(iIndex);

    int iIndexLength=0;
    for(int i=0;i<m_iVisibleItemNum;i++)
    {
        if(-1<m_piItemIndex[i]&&m_piItemIndex[i]<m_iArrayNum)
        {
            //this->item(i)->setHidden(false);
            setRowHidden(i,false);
            iIndexLength++;
        }
        else
        {
            //this->item(i)->setHidden(true);
            setRowHidden(i,true);
        }

    }
    for(int i=m_iVisibleItemNum;i<m_iItemNum;i++)
    {
        //this->item(i)->setHidden(true);
        setRowHidden(i,true);
    }

    emit teUpDateItem(m_piItemIndex,iIndexLength);

}

