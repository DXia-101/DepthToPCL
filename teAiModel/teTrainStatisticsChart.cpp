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
    // 图表添加两条曲线
    iteration = customPlot->addGraph();
    fAvgLoss = customPlot->addGraph();
    fPosAcc = customPlot->addGraph();

    // 设置曲线颜色
    iteration->setPen(QPen(Qt::red));
    fAvgLoss->setPen(QPen(Qt::black));
    fPosAcc->setPen(QPen(Qt::green));

    // 设置坐标轴名称
    customPlot->xAxis->setLabel("X");
    customPlot->yAxis->setLabel("Y");

    // 设置y坐标轴显示范围
    customPlot->yAxis->setRange(0, 1);

    // 显示图表的图例
    customPlot->legend->setVisible(true);
    // 添加曲线名称
    iteration->setName("iteration");
    fAvgLoss->setName("fAvgLoss");
    fPosAcc->setName("fPosAcc");

    // 设置波形曲线的复选框字体颜色
    ui->iteration_checkBox->setStyleSheet("QCheckBox{color:rgb(255,0,0)}");
    ui->fAvgLoss_checkBox->setStyleSheet("QCheckBox{color:rgb(0,0,0)}");
    ui->fPosAcc_checkBox->setStyleSheet("QCheckBox{color:rgb(0,255,0)}");

    // 允许用户用鼠标拖动轴范围，用鼠标滚轮缩放，点击选择图形:
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
}

void teTrainStatisticsChart::Show_Plot(QCustomPlot* customPlot, int iterationNum, float fAvgLossNum, float fPosAccNum)
{
    static double cnt;

    cnt++;
    // 给曲线添加数据
    iteration->addData(cnt, iterationNum);
    fAvgLoss->addData(cnt, fAvgLossNum);
    fPosAcc->addData(cnt, fPosAccNum);

    // 设置x坐标轴显示范围，使其自适应缩放x轴，x轴最大显示1000个点
    customPlot->xAxis->setRange((iteration->dataCount() > 1000) ? (iteration->dataCount() - 1000) : 0, iteration->dataCount());
    customPlot->replot(QCustomPlot::rpQueuedReplot);

    static QTime time(QTime::currentTime());
    double key = time.elapsed() / 1000.0; // 开始到现在的时间，单位秒
    static double lastFpsKey;
    static int frameCount;
    ++frameCount;
    if (key - lastFpsKey > 1) // 每1秒求一次平均值
    {
        //状态栏显示帧数和数据总数
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
