#ifndef DRAWCHART_H
#define DRAWCHART_H
#include <QChart>
#include <QLegend>
#include <QLineSeries>
#include <QChartView>
#include <QValueAxis>
#include <QtCharts/QLegendMarker>

#define MAX_POWER 3000

//此句必备
//QT_CHARTS_USE_NAMESPACE

QT_BEGIN_NAMESPACE

QChart *creatChart(QList<QPointF> points, QString Joint) ;
QChart *creatChart(QList<QPointF> points, QString yLabel, QString xLabel, double rlo, double rhi);
QChart *creatChart(QList<QList<QPointF>> pointsMatrix);

QT_END_NAMESPACE
#endif // DRAWCHART_H
