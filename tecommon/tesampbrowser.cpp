#include"tesampbrowser.h"
#include<QHeaderView>
#include<QGridLayout>
#include<QLabel>

TeSampBrowser::TeSampBrowser(QWidget* parent,TeBrowseMode eBackMode,
                             TeBrowseMode eForwardMode):TeTableWidget(parent,eBackMode,eForwardMode)

{

    setColumnCount(2);

    setIconSize(QSize(m_iItemHeight,m_iItemHeight));

    for(int j=0;j<columnCount();j++)
    {
        setHorizontalHeaderItem(j,new QxtTableWidgetItem());

        for(int i=0;i<m_iItemNum;i++)
        {

            setItem(i,j,new QxtTableWidgetItem());

            if(j==0)
            {
                item(i,j)->setSizeHint(QSize(m_iItemHeight,m_iItemHeight));
                static_cast<QxtTableWidgetItem*>(item(i,j))->setFlag(Qt::ItemIsEditable,false);
                setVerticalHeaderItem(i,new QxtTableWidgetItem());
                setRowHeight(i,m_iItemHeight);
            }
            if(j==1)
            {
                static_cast<QxtTableWidgetItem*>(item(i,j))->setFlag(Qt::ItemIsEditable,false);
                setIndexWidget(indexFromItem(item(i,j)),teCreateWidget(item(i,j)));
            }

        }

    }

    horizontalHeaderItem(0)->setText(u8"缩略图");
    horizontalHeaderItem(1)->setText(u8"图片信息");


    //QHeaderView::ResizeToContents QHeaderView::Stretch
    horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);//其余列最佳大小
    horizontalHeader()->setSectionResizeMode(1,QHeaderView::Stretch);//第二列自适应大小


    verticalHeader()->hide();
//    connect(this,&TeTableWidget::teUpDateItem,this,[this](int* piIndex,int iLength)
//    {
//        for(int i=0;i<iLength;i++)
//        {
//            verticalHeaderItem(i)->setText(QString::number(piIndex[i]+1));

//        }

//    });


}

TeSampBrowser::~TeSampBrowser()
{

}



void TeSampBrowser::teSetThumb(int iRow,const QImage& qImg,TeImgFormat eFormat)
{
	if (qImg.isNull()) {
		item(iRow, 0)->setIcon(QIcon());
		return;
	}

    QImage qShowImg;
    switch(eFormat)
    {
    case(E_FORMAT_RGB):{
        qShowImg=qImg;//浅拷贝
        break;
    }
    case(E_FORMAT_BGR):{
        qShowImg=qImg.rgbSwapped();//深拷贝
        break;
    }
    }

    if(qImg.width()!=m_iItemHeight||qImg.height()!=m_iItemHeight)
        qShowImg=qShowImg.scaled(m_iItemHeight,m_iItemHeight);//深拷贝

    QPixmap qShowPixmap=QPixmap::fromImage(qShowImg);

    item(iRow,0)->setIcon(qShowPixmap);
}


void TeSampBrowser::teSetThumbInfo(int iRow,int iWidth,int iHeight,const QString& qstr)
{
    item(iRow,1)->data(Qt::UserRole+1).value<QLabel*>()->setNum(iWidth);
    item(iRow,1)->data(Qt::UserRole+2).value<QLabel*>()->setNum(iHeight);
    item(iRow,1)->data(Qt::UserRole+3).value<TeEditLabel*>()->teSetText(qstr);

}

bool TeSampBrowser::teIsThumbNull(int iRow)
{
	return item(iRow, 0)->icon().isNull();
}

void TeSampBrowser::teAddField(QString qstrName)
{
    QFont qFont;
    qFont.setBold(true);
    qFont.setPointSize(12);

    setColumnCount(columnCount()+1);

    QxtTableWidgetItem* pHeadItem=new QxtTableWidgetItem(qstrName);
    setHorizontalHeaderItem(columnCount()-1,pHeadItem);

    for(int i=0;i<m_iItemNum;i++)
    {
        QxtTableWidgetItem* pItem=new QxtTableWidgetItem;
        pItem->setFlag(Qt::ItemIsEditable,false);
        setItem(i,columnCount()-1,pItem);
    }

}

void TeSampBrowser::teSetFieldValue(int iRow,int iFieldIndex,QString qstrValue,
                                   QColor qClolor)
{
    item(iRow,2+iFieldIndex)->setText(qstrValue);

    item(iRow,2+iFieldIndex)->setForeground(QBrush(qClolor));

    item(iRow,2+iFieldIndex)->setData(Qt::UserRole,qstrValue);

}


QString TeSampBrowser::teGetFieldValue(int iRow,int iFieldIndex)
{
    return item(iRow,2+iFieldIndex)->data(Qt::UserRole).toString();
}


void TeSampBrowser::keyPressEvent(QKeyEvent * qevent)
{
    if(currentRow()>=0){

        switch (qevent->key())
        {
        //切换项目文本框退出编辑
        case(Qt::Key_Up):case(Qt::Key_Down):{
            if(TeEditLabel::E_EDIT_TEXT==item(currentRow(),1)->data(Qt::UserRole+3)
               .value<TeEditLabel*>()->teGetEditType())
            {
                setFocus();
            }

            break;
        }

        default:break;

        }

    }

    TeTableWidget::keyPressEvent(qevent);
}

void TeSampBrowser::mousePressEvent(QMouseEvent *event)
{
    if(currentRow()>=0&& itemAt(event->pos())!=nullptr){
        //处于当前行时，单机文本框进入编辑
       if(itemAt(event->pos())->row()==currentRow())
       {
           if(item(currentRow(),1)->data(Qt::UserRole+3)
                         .value<TeEditLabel*>()->underMouse())
           {
               item(currentRow(),1)->data(Qt::UserRole+3)
                          .value<TeEditLabel*>()->teSwitchText();
           }
       }
    }

    TeTableWidget::mousePressEvent(event);

}


QWidget* TeSampBrowser::teCreateWidget(QTableWidgetItem* pItem)
{
    QWidget *pWidget = new QWidget;
    QGridLayout *pLayout=new QGridLayout(pWidget);

    //pLayout->setContentsMargins(0,10,0,0);

    QFont qFont;
    qFont.setBold(true);
    qFont.setPointSize(12);


    //QLabel* pSizeWidget=new QLabel("分辨率:",pWidget);
    //pSizeWidget->setAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
    //pSizeWidget->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));

    QLabel* pWidthWidget=new QLabel(pWidget);
    pWidthWidget->setAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
    pWidthWidget->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));


    QLabel* pXLabel=new QLabel(u8"×",pWidget);
    pXLabel->setAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
    pXLabel->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));

    QLabel* pHeightWidget=new QLabel(pWidget);
    pHeightWidget->setAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
    pHeightWidget->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));


    TeEditLabel* pEditLabel=new TeEditLabel(pWidget);
    pEditLabel->setSizePolicy(QSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed));


    pItem->setData(Qt::UserRole+1,QVariant::fromValue(pWidthWidget));
    pItem->setData(Qt::UserRole+2,QVariant::fromValue(pHeightWidget));
    pItem->setData(Qt::UserRole+3,QVariant::fromValue(pEditLabel));

    //pLayout->addWidget(pSizeWidget,0,0,1,1);
    pLayout->addWidget(pWidthWidget,0,0,1,1);
    pLayout->addWidget(pXLabel,0,1,1,1);
    pLayout->addWidget(pHeightWidget,0,2,1,1);
    pLayout->addWidget(pEditLabel,1,0,1,4);


    pWidget->setLayout(pLayout);


    return pWidget;

}


