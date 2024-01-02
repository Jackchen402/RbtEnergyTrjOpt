#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QListView>
#include <QStandardItemModel>
#include <QVBoxLayout>
#include <QCheckBox>
#include <iostream>
#include <QTcpSocket>
#include <QTcpServer>
#include <QHostAddress>
#include <QtDebug>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <thread>
#include <chrono>
#include <map>
#include "drawchart.h"

extern "C" {

#include "modbus/modbus.h"

}

using namespace std;

extern QString sampling_data_path;
extern QString kuku_ptpsrc_path;


namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    void init_Server();
    QByteArray readData();
    void sendData(QString str);
    int ConnectToRobot();
    int closeConnection();

    void InitRobotControl(); // 建立六轴联动的槽函数和信号
    ~Widget();
private slots:
    //void on_pushButton_8_clicked();
    void on_StartServer_clicked();
    void on_StopServer_clicked();
    void ClientDisconnected();
    void receivedData();
    void delegateFunc();
    void on_sendBtn_clicked();
    void on_recordPower_clicked();
    void on_stopRecordPower_clicked();
    void ExitSafely_func();
    void receivePower();
    void traject_add_func();
    void traject_clear_func();
    void drawChartView();
    void cal_Energy_func();
    void cal_min_Energy_func();
    void optimal_Drawing_func();
    void generate_opt_file_func();
    void draw_para_graph_func();
    void traject_compare_func();
    //自定义按钮对应的slot
    void Axis_Control();
    void Position_Reset();
    void Position_set();



signals:
    void receivePowerSignal();

private:
    Ui::Widget *ui;

private:
    QTcpServer *tcpserver = NULL;
    QTcpSocket *tcpsocket = NULL;  // 只有一个客户端，就是机器人控制中心
    modbus_t *mb = NULL;
    QStandardItemModel *model = NULL;
    QList<int> *selectedItems = NULL;
    QPushButton *drawPic_btn = NULL;
    QWidget *window = NULL;

};
void recordPower(Widget *w);
void startRecordPower(Widget *w);
void generate_trajectory();


#endif // WIDGET_H
