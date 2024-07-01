#include "teTrainStatisticsChart.h"

#include <QStatusBar>
#include <QTimer>
#include <qcustomplot.h>
#include <QToolTip>

teTrainStatisticsChart::Garbo teTrainStatisticsChart::tmp;
teTrainStatisticsChart* teTrainStatisticsChart::instance = nullptr;

teTrainStatisticsChart::teTrainStatisticsChart(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::teTrainStatisticsChartClass())
{
	ui->setupUi(this);
    customPlot = ui->ChartDisplayWidget;
    customBar = new QStatusBar();
    ui->statusLayout->addWidget(customBar);
    customPlot->installEventFilter(this);
    
    QPlot_init(customPlot);

}

teTrainStatisticsChart::teTrainStatisticsChart(const teTrainStatisticsChart&)
{
}

teTrainStatisticsChart::~teTrainStatisticsChart()
{
    delete ui;
}

teTrainStatisticsChart& teTrainStatisticsChart::operator=(const teTrainStatisticsChart&)
{
    return *this;
}


teTrainStatisticsChart* teTrainStatisticsChart::getInstance()
{
    if (!instance)
    {
        teTrainStatisticsChart* pInstance = new teTrainStatisticsChart();
        instance = pInstance;
    }
    return instance;
}

void teTrainStatisticsChart::destroy()
{
    if (NULL != teTrainStatisticsChart::instance) {
        delete teTrainStatisticsChart::instance;
        teTrainStatisticsChart::instance = NULL;
    }
}

void teTrainStatisticsChart::ReceiveData(int iteration, float fAvgLoss, float fPosAcc)
{
    Show_Plot(customPlot, iteration, fAvgLoss, fPosAcc);
}

void teTrainStatisticsChart::on_iteration_checkBox_stateChanged(int arg)
{
    if (arg) {
        iteration->setPen(QPen(Qt::red));
    }
    else {
        iteration->setPen(QColor(0, 0, 0, 0));
    }
    customPlot->replot();
}

void teTrainStatisticsChart::on_fAvgLoss_checkBox_stateChanged(int arg)
{
    if (arg) {
        fAvgLoss->setPen(QPen(Qt::black));
    }
    else {
        fAvgLoss->setPen(QColor(0, 0, 0, 0));
    }
    customPlot->replot();
}

void teTrainStatisticsChart::on_fPosAcc_checkBox_stateChanged(int arg)
{
    if (arg) {
        fPosAcc->setPen(QPen(Qt::green));
    }
    else {
        fPosAcc->setPen(QColor(0, 0, 0, 0));
    }
    customPlot->replot();
}

void teTrainStatisticsChart::isShow(int arg)
{
    if (arg > 0) {
        show();
    }
    else {
        hide();
    }
}

void teTrainStatisticsChart::RestoreInitialState()
{
    customPlot->clearGraphs();
    customPlot->clearItems();
    QPlot_init(customPlot);
}

void teTrainStatisticsChart::QPlot_init(QCustomPlot* customPlot)
{
    // ͼ�������������
    iteration = customPlot->addGraph();
    fAvgLoss = customPlot->addGraph();
    fPosAcc = customPlot->addGraph();

    // ����������ɫ
    iteration->setPen(QPen(Qt::red));
    fAvgLoss->setPen(QPen(Qt::black));
    fPosAcc->setPen(QPen(Qt::green));

    // ��������������
    customPlot->xAxis->setLabel("X");
    customPlot->yAxis->setLabel("Y");

    // ����y��������ʾ��Χ
    customPlot->yAxis->setRange(0, 1);

    // ��ʾͼ���ͼ��
    customPlot->legend->setVisible(true);
    // �����������
    iteration->setName("iteration");
    fAvgLoss->setName("fAvgLoss");
    fPosAcc->setName("fPosAcc");

    // ���ò������ߵĸ�ѡ��������ɫ
    ui->iteration_checkBox->setStyleSheet("QCheckBox{color:rgb(255,0,0)}");
    ui->fAvgLoss_checkBox->setStyleSheet("QCheckBox{color:rgb(0,0,0)}");
    ui->fPosAcc_checkBox->setStyleSheet("QCheckBox{color:rgb(0,255,0)}");

    // �����û�������϶��᷶Χ�������������ţ����ѡ��ͼ��:
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

void teTrainStatisticsChart::Show_Plot(QCustomPlot* customPlot, int iterationNum, float fAvgLossNum, float fPosAccNum)
{
    static double cnt;

    cnt++;
    // �������������
    iteration->addData(cnt, iterationNum);
    fAvgLoss->addData(cnt, fAvgLossNum);
    fPosAcc->addData(cnt, fPosAccNum);

    // ����x��������ʾ��Χ��ʹ������Ӧ����x�ᣬx�������ʾ1000����
    //customPlot->xAxis->setRange((iteration->dataCount() > 1000) ? (iteration->dataCount() - 1000) : 0, iteration->dataCount());
    customPlot->rescaleAxes();
    customPlot->replot(QCustomPlot::rpQueuedReplot);

    static QTime time(QTime::currentTime());
    double key = time.elapsed() / 1000.0; // ��ʼ�����ڵ�ʱ�䣬��λ��
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    if (key - lastFpsKey > 1) // ÿ1����һ��ƽ��ֵ
    {
        //״̬����ʾ֡������������
        customBar->showMessage(
            QString("%1 FPS, Total Data points: %2")
            .arg(frameCount / (key - lastFpsKey), 0, 'f', 0)
            .arg(customPlot->graph(0)->data()->size() + customPlot->graph(1)->data()->size())
            , 0);
        lastFpsKey = key;
        frameCount = 0;
    }
}

void teTrainStatisticsChart::closeEvent(QCloseEvent* event)
{
    emit sig_closeteTrainStatisticsChart();
    event->accept();
}

void teTrainStatisticsChart::handlePositionToolTip(QMouseEvent* event)
{
    double dValueX = customPlot->xAxis->pixelToCoord(event->pos().x());
    double dValueY = customPlot->yAxis->pixelToCoord(event->pos().y());

    if (dValueX < customPlot->xAxis->range().lower || dValueX > customPlot->xAxis->range().upper
        || dValueY < customPlot->yAxis->range().lower || dValueY > customPlot->yAxis->range().upper)
        return;

    double dRatioX = customPlot->xAxis->axisRect()->width() / (customPlot->xAxis->range().upper - customPlot->xAxis->range().lower);
    double dRatioY = customPlot->yAxis->axisRect()->height() / (customPlot->yAxis->range().upper - customPlot->yAxis->range().lower);

    for (QCPGraph* Graph : { iteration ,fAvgLoss ,fPosAcc })
    {
        if (!Graph->visible())
            continue;

        QSharedPointer<QCPGraphDataContainer> dataContainer = Graph->data();
        if (dataContainer->at(dataContainer->size() - 1)->key < ((double)(2.0 / dRatioX) + dValueX))
            continue;

        QVector<double> vecValue;
        QVector<double> vecKey;
        //�ҵ�����������x��Χ�ڵĵ㣨���2�����ص����ڣ�
        int iIndex1 = Graph->findBegin(dValueX - (double)(2.0 / dRatioX));
        int iIndex2 = Graph->findEnd((double)(2.0 / dRatioX) + dValueX);
        for (int k = iIndex1; k <= iIndex2; k++)
        {
            vecKey.push_back(dataContainer->at(k)->key);
            vecValue.push_back(dataContainer->at(k)->value);
        }

        double dFinalY = 0;
        double dFinalX = 0;
        double dy = qAbs(dValueY);
        bool bExist = false;
        for (int k = 0; k < vecValue.count(); k++)
        {
            if (qAbs(vecValue.at(k) - dValueY) * dRatioY > 2)
                continue;

            if (qAbs(vecValue.at(k) - dValueY) < dy)
            {
                dy = qAbs(vecValue.at(k) - dValueY);
                dFinalY = vecValue.at(k);
                dFinalX = vecKey.at(k);
                bExist = true;
            }
        }
        QString GraphName;
        if (Graph == iteration)
            GraphName = "iteration";
        if (Graph == fAvgLoss)
            GraphName = "fAvgLoss";
        if (Graph == fPosAcc)
            GraphName = "fPosAcc";
        if (bExist)
        {
            QString strToolTip = QString("%1\nx=%2\ny=%3").arg(GraphName).arg(dFinalX).arg(dFinalY);
            QToolTip::showText(cursor().pos(), strToolTip, customPlot);
            return;
        }
    }

    if (QToolTip::isVisible())
        QToolTip::hideText();
}

bool teTrainStatisticsChart::eventFilter(QObject* watched, QEvent* event)
{
    if (watched == customPlot)
    {
        if (event->type() == QEvent::MouseMove)
        {
            QMouseEvent* pMouseEvent = (QMouseEvent*)event;
            handlePositionToolTip(pMouseEvent);
        }
    }
    return QWidget::eventFilter(watched, event);
}
