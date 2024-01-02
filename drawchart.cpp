#include "drawchart.h"

QChart *creatChart(QList<QPointF> points, QString Joint) {
    // 创建一个线
    QLineSeries *pLineSeries = new QLineSeries();
    pLineSeries->append(points);
    //pLineSeries->pointLabelsVisible(false);
    pLineSeries->setColor(Qt::red);
    //pLineSeries->setPointsVisible(true);
    // 创建一个chart

    QValueAxis *xAxis = new QValueAxis();
    QValueAxis *yAxis = new QValueAxis();

    xAxis->setTitleText("t(s)");
    xAxis->setLineVisible(true);
    yAxis->setTitleText(Joint);
    yAxis->setLineVisible(true);
    if (Joint == "power(w)") {
        yAxis->setRange(0, MAX_POWER);
    }


    xAxis->setGridLineVisible(true);
    yAxis->setGridLineVisible(true);

    QPen pen;
    pen.setStyle(Qt::DotLine);
    pen.setColor(Qt::lightGray);
    xAxis->setGridLinePen(pen);
    yAxis->setGridLinePen(pen);

    xAxis->setLinePenColor(Qt::black);
    yAxis->setLinePenColor(Qt::black);

    QChart *pChart = new QChart();
    pChart->addSeries(pLineSeries);
    pChart->legend()->hide();
    pChart->addAxis(xAxis, Qt::AlignBottom);
    pChart->addAxis(yAxis, Qt::AlignLeft);
    //pChart->setTitle("测试Qchart运行");
    //pChart->createDefaultAxes();

    pLineSeries->attachAxis(xAxis);
    pLineSeries->attachAxis(yAxis);


    //pChart->axisX()->setRange(0, 15);
    //pChart->axisY()->setRange(-10, 30);

    pChart->setTheme(QChart::ChartThemeLight);

    /*
    //创建ChartView
    QChartView *pView = new QChartView();
    pView->setChart(pChart);
    pView->setRenderHint(QPainter::Antialiasing);
    */

    pChart->setContentsMargins(0, 0, 0, 0);  //设置外边界全部为0
    pChart->setMargins(QMargins(0, 0, 0, 0));//设置内边界全部为0
    pChart->setBackgroundVisible(true);
    pChart->setBackgroundRoundness(0);       //设置背景区域无圆角

    return pChart;
}



QChart *creatChart(QList<QPointF> points, QString yLabel, QString xLabel, double rlo, double rhi) {
    // 创建一个线
    QLineSeries *pLineSeries = new QLineSeries();
    pLineSeries->append(points);
    //pLineSeries->pointLabelsVisible(false);
    pLineSeries->setColor(Qt::red);
    //pLineSeries->setPointsVisible(true);
    // 创建一个chart

    QValueAxis *xAxis = new QValueAxis();
    QValueAxis *yAxis = new QValueAxis();

    xAxis->setTitleText(xLabel);
    xAxis->setLineVisible(true);
    yAxis->setTitleText(yLabel);
    yAxis->setLineVisible(true);

    yAxis->setRange(rlo, rhi);


    xAxis->setGridLineVisible(true);
    yAxis->setGridLineVisible(true);

    QPen pen;
    pen.setStyle(Qt::DotLine);
    pen.setColor(Qt::lightGray);
    xAxis->setGridLinePen(pen);
    yAxis->setGridLinePen(pen);

    xAxis->setLinePenColor(Qt::black);
    yAxis->setLinePenColor(Qt::black);

    QChart *pChart = new QChart();
    pChart->addSeries(pLineSeries);
    pChart->legend()->hide();
    pChart->addAxis(xAxis, Qt::AlignBottom);
    pChart->addAxis(yAxis, Qt::AlignLeft);
    //pChart->setTitle("测试Qchart运行");
    //pChart->createDefaultAxes();

    pLineSeries->attachAxis(xAxis);
    pLineSeries->attachAxis(yAxis);


    //pChart->axisX()->setRange(0, 15);
    //pChart->axisY()->setRange(-10, 30);

    pChart->setTheme(QChart::ChartThemeLight);

    /*
    //创建ChartView
    QChartView *pView = new QChartView();
    pView->setChart(pChart);
    pView->setRenderHint(QPainter::Antialiasing);
    */

    pChart->setContentsMargins(0, 0, 0, 0);  //设置外边界全部为0
    pChart->setMargins(QMargins(0, 0, 0, 0));//设置内边界全部为0
    pChart->setBackgroundVisible(true);
    pChart->setBackgroundRoundness(0);       //设置背景区域无圆角

    return pChart;
}

QChart *creatChart(QList<QList<QPointF>> pointsMatrix)
{
    QList<QPointF> points[6];
    int i = 0;
    foreach(QList<QPointF> element, pointsMatrix) {
        // 处理元素
        points[i] = element;
        //QList<QPointF> point = element;
        i++;
    }

    QLineSeries *pLineSeries1 = new QLineSeries();
    QLineSeries *pLineSeries2 = new QLineSeries();
    QLineSeries *pLineSeries3 = new QLineSeries();
    QLineSeries *pLineSeries4 = new QLineSeries();
    QLineSeries *pLineSeries5 = new QLineSeries();
    QLineSeries *pLineSeries6 = new QLineSeries();
    pLineSeries1->append(points[0]);
    pLineSeries2->append(points[1]);
    pLineSeries3->append(points[2]);
    pLineSeries4->append(points[3]);
    pLineSeries5->append(points[4]);
    pLineSeries6->append(points[5]);
    //pLineSeries->pointLabelsVisible(false);
    QPen pen1(Qt::SolidLine);
    QPen pen2(Qt::DashLine);
    QPen pen3(Qt::DotLine);
    QPen pen4(Qt::DashDotLine);
    QPen pen5(Qt::DashDotDotLine);
    QPen pen6(Qt::SolidLine);
    /*
    pen1.setWidth(2);
    pen2.setWidth(2);
    pen3.setWidth(2);
    pen4.setWidth(2);
    pen5.setWidth(2);
    pen6.setWidth(3);
*/
    pen1.setColor(Qt::red);
    pen2.setColor(Qt::darkYellow);
    pen3.setColor(Qt::blue);
    pen4.setColor(Qt::green);
    pen5.setColor(Qt::magenta);
    pen6.setColor(Qt::yellow);


    pLineSeries1->setPen(pen1);
    pLineSeries2->setPen(pen2);
    pLineSeries3->setPen(pen3);
    pLineSeries4->setPen(pen4);
    pLineSeries5->setPen(pen5);
    pLineSeries6->setPen(pen6);

    pLineSeries1->setName("q1");
    pLineSeries2->setName("q2");
    pLineSeries3->setName("q3");
    pLineSeries4->setName("q4");
    pLineSeries5->setName("q5");
    pLineSeries6->setName("q6");
    //pLineSeries->setPointsVisible(true);
    // 创建一个chart

    QValueAxis *xAxis = new QValueAxis();
    QValueAxis *yAxis = new QValueAxis();

    xAxis->setTitleText("time(s)");
    xAxis->setLineVisible(true);
    yAxis->setTitleText("pos(deg)");
    yAxis->setLineVisible(true);

    yAxis->setRange(-360, 360);


    xAxis->setGridLineVisible(true);
    yAxis->setGridLineVisible(true);

    QPen pen;
    pen.setStyle(Qt::DotLine);
    pen.setColor(Qt::lightGray);
    xAxis->setGridLinePen(pen);
    yAxis->setGridLinePen(pen);

    xAxis->setLinePenColor(Qt::black);
    yAxis->setLinePenColor(Qt::black);

    QChart *pChart = new QChart();
    pChart->addSeries(pLineSeries1);
    pChart->addSeries(pLineSeries2);
    pChart->addSeries(pLineSeries3);
    pChart->addSeries(pLineSeries4);
    pChart->addSeries(pLineSeries5);
    pChart->addSeries(pLineSeries6);

    pChart->legend()->setVisible(true);

    pChart->legend()->setMarkerShape(QLegend::MarkerShapeFromSeries);
    //pChart->legend()->setMinimumWidth(200);


    pChart->addAxis(xAxis, Qt::AlignBottom);
    pChart->addAxis(yAxis, Qt::AlignLeft);
    //pChart->setTitle("测试Qchart运行");
    //pChart->createDefaultAxes();

    pLineSeries1->attachAxis(xAxis);
    pLineSeries1->attachAxis(yAxis);

    pLineSeries2->attachAxis(xAxis);
    pLineSeries2->attachAxis(yAxis);

    pLineSeries3->attachAxis(xAxis);
    pLineSeries3->attachAxis(yAxis);

    pLineSeries4->attachAxis(xAxis);
    pLineSeries4->attachAxis(yAxis);

    pLineSeries5->attachAxis(xAxis);
    pLineSeries5->attachAxis(yAxis);

    pLineSeries6->attachAxis(xAxis);
    pLineSeries6->attachAxis(yAxis);



    pChart->setTheme(QChart::ChartThemeLight);

    /*
    //创建ChartView
    QChartView *pView = new QChartView();
    pView->setChart(pChart);
    pView->setRenderHint(QPainter::Antialiasing);
    */

    pChart->setContentsMargins(0, 0, 0, 0);  //设置外边界全部为0
    pChart->setMargins(QMargins(0, 0, 0, 0));//设置内边界全部为0
    pChart->setBackgroundVisible(true);
    pChart->setBackgroundRoundness(0);       //设置背景区域无圆角
    return pChart;
}
