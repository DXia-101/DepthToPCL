#include "teTrainStatisticsChart.h"
#include <QStatusBar>
#include <QTimer>

teTrainStatisticsChart::teTrainStatisticsChart(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::teTrainStatisticsChartClass())
{
	ui->setupUi(this);
    customPlot = ui->ChartDisplayWidget;
    customBar = new QStatusBar();
    ui->statusLayout->addWidget(customBar);
    
    QPlot_init(customPlot);

}

teTrainStatisticsChart::~teTrainStatisticsChart()
{
    delete ui;
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
    customPlot->xAxis->setRange((iteration->dataCount() > 1000) ? (iteration->dataCount() - 1000) : 0, iteration->dataCount());
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
