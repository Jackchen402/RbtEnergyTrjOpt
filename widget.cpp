#include <QThread>
#include <QtCore>
#include "widget.h"
#include "ui_widget.h"
#include "drawchart.h"
#include "trajectory.h"

bool threadRun = false;
bool time_clock_run = false;
double time_clock = 0;
map<double, double> optimal_result;
QString sampling_data_path;
QString kuku_ptpsrc_path;
VectorXd optimal_time;
MatrixXd joints;

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget),
    mb(NULL)
{
    ui->setupUi(this);
    ui->tabWidget->setStyleSheet("background-color:rgb(231,230,229);");
    ui->Input_IP->setText("172.31.1.11");
    ui->Input_PORT->setText("8080");
    ui->Input_IP_2->setText("172.31.1.146");
    ui->Input_PORT_2->setText("502");
    ui->datatoSend->setStyleSheet("background-color:rgb(255,255,255);");
    ui->dynm_factor->setText("0.35");
    ui->optimal_factor->setText("0.35");
    ui->E_Consumation->setText("0");
    ui->Opt_Consumation->setText("0");
    ui->delta_text->setText("10");
    ui->sampling_num->setText("5");

    qDebug() << sampling_data_path << " \n";
    InitRobotControl();

//    QList<QPointF> points;
//    points << QPointF(0, 6) << QPointF(2, 4) << QPointF(3, 8) << QPointF(7, 4)<< QPointF(10, 5)
//              << QPointF(11, 1) << QPointF(13, 3) << QPointF(17, 6) << QPointF(18, 3) << QPointF(20, 2);
//    drawChartView(points);

}

Widget::~Widget()
{
    delete ui;
}
/****************************************************/
// 画图部分
void Widget::drawChartView() {


    int size = selectedItems->size();
    if (size > 1) {
        //qSort(selectedItems->begin(), selectedItems->end());
        sort(selectedItems->begin(), selectedItems->end());
    }



    QString str1, str2, str3, str4, str5, str6, t_str, p_str;
    QList<QPointF> points1, points2, points3, points4, points5, points6, energy_points;

    for (int i = 0; i < size; ++i) {
        int row = (*selectedItems)[i];
        QModelIndex index = model->index(row, 0);
        QString buff = model->data(index).toString();


        int Startpos = buff.indexOf("<time>"), Endpos = buff.indexOf("</time>"), num = Endpos - Startpos;
        t_str = buff.mid(Startpos + 6, num - 6);


        Startpos = buff.indexOf("<AX1>"), Endpos = buff.indexOf("</AX1>"), num = Endpos - Startpos;
        str1 = buff.mid(Startpos + 5, num - 5);
        points1 << QPointF(t_str.toDouble(), str1.toDouble());


        Startpos = buff.indexOf("<AX2>"), Endpos = buff.indexOf("</AX2>"), num = Endpos - Startpos;
        str2 = buff.mid(Startpos + 5, num - 5);
        points2 << QPointF(t_str.toDouble(), str2.toDouble());

        Startpos = buff.indexOf("<AX3>"), Endpos = buff.indexOf("</AX3>"), num = Endpos - Startpos;
        str3 = buff.mid(Startpos + 5, num - 5);
        points3 << QPointF(t_str.toDouble(), str3.toDouble());

        Startpos = buff.indexOf("<AX4>"), Endpos = buff.indexOf("</AX4>"), num = Endpos - Startpos;
        str4 = buff.mid(Startpos + 5, num - 5);
        points4 << QPointF(t_str.toDouble(), str4.toDouble());

        Startpos = buff.indexOf("<AX5>"), Endpos = buff.indexOf("</AX5>"), num = Endpos - Startpos;
        str5 = buff.mid(Startpos + 5, num - 5);
        points5 << QPointF(t_str.toDouble(), str5.toDouble());

        Startpos = buff.indexOf("<AX6>"), Endpos = buff.indexOf("</AX6>"), num = Endpos - Startpos;
        str6 = buff.mid(Startpos + 5, num - 5);
        points6 << QPointF(t_str.toDouble(), str6.toDouble());

        Startpos = buff.indexOf("<power>"), Endpos = buff.indexOf("</power>"), num = Endpos - Startpos;
        p_str = buff.mid(Startpos + 7, num - 7);
        energy_points << QPointF(t_str.toDouble(), p_str.toDouble());

    }

    QChart *chart1 = creatChart(points1, "q1(deg)");
    ui->Joint1_Graph->setChart(chart1);
    ui->Joint1_Graph->setRenderHint(QPainter::Antialiasing);


    QChart *chart2 = creatChart(points2, "q2(deg)");
    ui->Joint2_Graph->setChart(chart2);
    ui->Joint2_Graph->setRenderHint(QPainter::Antialiasing);

    QChart *chart3 = creatChart(points3, "q3(deg)");
    ui->Joint3_Graph->setChart(chart3);
    ui->Joint3_Graph->setRenderHint(QPainter::Antialiasing);

    QChart *chart4 = creatChart(points4, "q4(deg)");
    ui->Joint4_Graph->setChart(chart4);
    ui->Joint4_Graph->setRenderHint(QPainter::Antialiasing);

    QChart *chart5 = creatChart(points5, "q5(deg)");
    ui->Joint5_Graph->setChart(chart5);
    ui->Joint5_Graph->setRenderHint(QPainter::Antialiasing);

    QChart *chart6 = creatChart(points6, "q6(deg)");
    ui->Joint6_Graph->setChart(chart6);
    ui->Joint6_Graph->setRenderHint(QPainter::Antialiasing);


    QChart *chart7 = creatChart(energy_points, "power(w)");
    ui->Energy_Graph->setChart(chart7);
    ui->Energy_Graph->setRenderHint(QPainter::Antialiasing);

    window->close();
}

// 画最优轨迹
void Widget::optimal_Drawing_func() {
    if (optimal_time.size() == 0) {
        return;
    }
    int SEGMENTS = *Init_SEGMENTS();
    int size = selectedItems->size();
    QString str1, str2, str3, str4, str5, str6, t_str, p_str;
    QList<QPointF> points1, points2, points3, points4, points5, points6, energy_points;
    /*
    for (int i = 0; i < size; ++i) {
        int row = (*selectedItems)[i * SEGMENTS];
        QModelIndex index = model->index(row, 0);
        QString buff = model->data(index).toString();

        int Startpos = buff.indexOf("<AX1>"), Endpos = buff.indexOf("</AX1>"), num = Endpos - Startpos;
        str1 = buff.mid(Startpos + 5, num - 5);
        points1 << QPointF(optimal_time(i), str1.toDouble());

        Startpos = buff.indexOf("<AX2>"), Endpos = buff.indexOf("</AX2>"), num = Endpos - Startpos;
        str2 = buff.mid(Startpos + 5, num - 5);
        points2 << QPointF(optimal_time(i), str2.toDouble());

        Startpos = buff.indexOf("<AX3>"), Endpos = buff.indexOf("</AX3>"), num = Endpos - Startpos;
        str3 = buff.mid(Startpos + 5, num - 5);
        points3 << QPointF(optimal_time(i), str3.toDouble());

        Startpos = buff.indexOf("<AX4>"), Endpos = buff.indexOf("</AX4>"), num = Endpos - Startpos;
        str4 = buff.mid(Startpos + 5, num - 5);
        points4 << QPointF(optimal_time(i), str4.toDouble());

        Startpos = buff.indexOf("<AX5>"), Endpos = buff.indexOf("</AX5>"), num = Endpos - Startpos;
        str5 = buff.mid(Startpos + 5, num - 5);
        points5 << QPointF(optimal_time(i), str5.toDouble());

        Startpos = buff.indexOf("<AX6>"), Endpos = buff.indexOf("</AX6>"), num = Endpos - Startpos;
        str6 = buff.mid(Startpos + 5, num - 5);
        points6 << QPointF(optimal_time(i), str6.toDouble());

    }
    */
        // 1/nu * (tm+1 - tm) + tm  对时间点进行插值
        // 利用原轨迹
        for (int i = 0; i < size; ++i) {
            int row = (*selectedItems)[i];
            QModelIndex index = model->index(row, 0);
            QString buff = model->data(index).toString();

            int t_index = i / SEGMENTS;
            //qDebug() << "t_index = " << t_index << "optimal_time.size() = " << optimal_time.size() << endl;
            int count = i % SEGMENTS;
            double interval = 0;
            if (t_index == size / SEGMENTS) {
                break; //后面的点舍弃
            }
            if (t_index < optimal_time.size() - 1)    {
                interval = (optimal_time(t_index + 1) - optimal_time(t_index)) / SEGMENTS;
            }
            double d_interval = double(count) * interval;

            int Startpos = buff.indexOf("<AX1>"), Endpos = buff.indexOf("</AX1>"), num = Endpos - Startpos;
            str1 = buff.mid(Startpos + 5, num - 5);
            points1 << QPointF(optimal_time(t_index) + d_interval, str1.toDouble());


            Startpos = buff.indexOf("<AX2>"), Endpos = buff.indexOf("</AX2>"), num = Endpos - Startpos;
            str2 = buff.mid(Startpos + 5, num - 5);
            points2 << QPointF(optimal_time(t_index)  + d_interval, str2.toDouble());

            Startpos = buff.indexOf("<AX3>"), Endpos = buff.indexOf("</AX3>"), num = Endpos - Startpos;
            str3 = buff.mid(Startpos + 5, num - 5);
            points3 << QPointF(optimal_time(t_index) + d_interval, str3.toDouble());

            Startpos = buff.indexOf("<AX4>"), Endpos = buff.indexOf("</AX4>"), num = Endpos - Startpos;
            str4 = buff.mid(Startpos + 5, num - 5);
            points4 << QPointF(optimal_time(t_index) + d_interval, str4.toDouble());

            Startpos = buff.indexOf("<AX5>"), Endpos = buff.indexOf("</AX5>"), num = Endpos - Startpos;
            str5 = buff.mid(Startpos + 5, num - 5);
            points5 << QPointF(optimal_time(t_index)  + d_interval, str5.toDouble());

            Startpos = buff.indexOf("<AX6>"), Endpos = buff.indexOf("</AX6>"), num = Endpos - Startpos;
            str6 = buff.mid(Startpos + 5, num - 5);
            points6 << QPointF(optimal_time(t_index)  + d_interval, str6.toDouble());


        }




    QChart *chart1 = creatChart(points1, "q1(deg)");
    ui->Joint1_Graph_2->setChart(chart1);
    ui->Joint1_Graph_2->setRenderHint(QPainter::Antialiasing);


    QChart *chart2 = creatChart(points2, "q2(deg)");
    ui->Joint2_Graph_2->setChart(chart2);
    ui->Joint2_Graph_2->setRenderHint(QPainter::Antialiasing);

    QChart *chart3 = creatChart(points3, "q3(deg)");
    ui->Joint3_Graph_2->setChart(chart3);
    ui->Joint3_Graph_2->setRenderHint(QPainter::Antialiasing);

    QChart *chart4 = creatChart(points4, "q4(deg)");
    ui->Joint4_Graph_2->setChart(chart4);
    ui->Joint4_Graph_2->setRenderHint(QPainter::Antialiasing);

    QChart *chart5 = creatChart(points5, "q5(deg)");
    ui->Joint5_Graph_2->setChart(chart5);
    ui->Joint5_Graph_2->setRenderHint(QPainter::Antialiasing);

    QChart *chart6 = creatChart(points6, "q6(deg)");
    ui->Joint6_Graph_2->setChart(chart6);
    ui->Joint6_Graph_2->setRenderHint(QPainter::Antialiasing);
}


void Widget::traject_compare_func() {
    int size = selectedItems->size();
    if (size == 0) {
        return;
    }
    if (size > 1) {
        //qSort(selectedItems->begin(), selectedItems->end());
        sort(selectedItems->begin(), selectedItems->end());
    }



    QString str1, str2, str3, str4, str5, str6, t_str;
    QList<QPointF> points1, points2, points3, points4, points5, points6;

    for (int i = 0; i < size; ++i) {
        int row = (*selectedItems)[i];
        QModelIndex index = model->index(row, 0);
        QString buff = model->data(index).toString();


        int Startpos = buff.indexOf("<time>"), Endpos = buff.indexOf("</time>"), num = Endpos - Startpos;
        t_str = buff.mid(Startpos + 6, num - 6);


        Startpos = buff.indexOf("<AX1>"), Endpos = buff.indexOf("</AX1>"), num = Endpos - Startpos;
        str1 = buff.mid(Startpos + 5, num - 5);
        points1 << QPointF(t_str.toDouble(), str1.toDouble());


        Startpos = buff.indexOf("<AX2>"), Endpos = buff.indexOf("</AX2>"), num = Endpos - Startpos;
        str2 = buff.mid(Startpos + 5, num - 5);
        points2 << QPointF(t_str.toDouble(), str2.toDouble());

        Startpos = buff.indexOf("<AX3>"), Endpos = buff.indexOf("</AX3>"), num = Endpos - Startpos;
        str3 = buff.mid(Startpos + 5, num - 5);
        points3 << QPointF(t_str.toDouble(), str3.toDouble());

        Startpos = buff.indexOf("<AX4>"), Endpos = buff.indexOf("</AX4>"), num = Endpos - Startpos;
        str4 = buff.mid(Startpos + 5, num - 5);
        points4 << QPointF(t_str.toDouble(), str4.toDouble());

        Startpos = buff.indexOf("<AX5>"), Endpos = buff.indexOf("</AX5>"), num = Endpos - Startpos;
        str5 = buff.mid(Startpos + 5, num - 5);
        points5 << QPointF(t_str.toDouble(), str5.toDouble());

        Startpos = buff.indexOf("<AX6>"), Endpos = buff.indexOf("</AX6>"), num = Endpos - Startpos;
        str6 = buff.mid(Startpos + 5, num - 5);
        points6 << QPointF(t_str.toDouble(), str6.toDouble());

    }

    QList<QList<QPointF>> points;
    points << points1 << points2 << points3 << points4 << points5 << points6;

    QChart *chart = creatChart(points);
    ui->original_traject->setChart(chart);
    ui->original_traject->setRenderHint(QPainter::Antialiasing);

    if (optimal_time.size() == 0) {
        return;
    }

    points1.clear();
    points2.clear();
    points3.clear();
    points4.clear();
    points5.clear();
    points6.clear();

    int SEGMENTS = *Init_SEGMENTS();
        // 1/nu * (tm+1 - tm) + tm  对时间点进行插值
        // 利用原轨迹
        for (int i = 0; i < size; ++i) {
            int row = (*selectedItems)[i];
            QModelIndex index = model->index(row, 0);
            QString buff = model->data(index).toString();

            int t_index = i / SEGMENTS;
            //qDebug() << "t_index = " << t_index << "optimal_time.size() = " << optimal_time.size() << endl;
            int count = i % SEGMENTS;
            double interval = 0;
            if (t_index == size / SEGMENTS) {
                break; //后面的点舍弃
            }
            if (t_index < optimal_time.size() - 1)    {
                interval = (optimal_time(t_index + 1) - optimal_time(t_index)) / SEGMENTS;
            }
            double d_interval = double(count) * interval;

            int Startpos = buff.indexOf("<AX1>"), Endpos = buff.indexOf("</AX1>"), num = Endpos - Startpos;
            str1 = buff.mid(Startpos + 5, num - 5);
            points1 << QPointF(optimal_time(t_index) + d_interval, str1.toDouble());


            Startpos = buff.indexOf("<AX2>"), Endpos = buff.indexOf("</AX2>"), num = Endpos - Startpos;
            str2 = buff.mid(Startpos + 5, num - 5);
            points2 << QPointF(optimal_time(t_index)  + d_interval, str2.toDouble());

            Startpos = buff.indexOf("<AX3>"), Endpos = buff.indexOf("</AX3>"), num = Endpos - Startpos;
            str3 = buff.mid(Startpos + 5, num - 5);
            points3 << QPointF(optimal_time(t_index) + d_interval, str3.toDouble());

            Startpos = buff.indexOf("<AX4>"), Endpos = buff.indexOf("</AX4>"), num = Endpos - Startpos;
            str4 = buff.mid(Startpos + 5, num - 5);
            points4 << QPointF(optimal_time(t_index) + d_interval, str4.toDouble());

            Startpos = buff.indexOf("<AX5>"), Endpos = buff.indexOf("</AX5>"), num = Endpos - Startpos;
            str5 = buff.mid(Startpos + 5, num - 5);
            points5 << QPointF(optimal_time(t_index)  + d_interval, str5.toDouble());

            Startpos = buff.indexOf("<AX6>"), Endpos = buff.indexOf("</AX6>"), num = Endpos - Startpos;
            str6 = buff.mid(Startpos + 5, num - 5);
            points6 << QPointF(optimal_time(t_index)  + d_interval, str6.toDouble());


        }

        QList<QList<QPointF>> pointsOpt;
        pointsOpt << points1 << points2 << points3 << points4 << points5 << points6;

        QChart *chartOpt = creatChart(pointsOpt);
        ui->optimal_traject->setChart(chartOpt);
        ui->optimal_traject->setRenderHint(QPainter::Antialiasing);

}


// 轨迹添加
void Widget::traject_add_func() {
    model = new QStandardItemModel();
    QList<QStandardItem*> list;

    QFile file(sampling_data_path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "文件打开失败\n";
    }
    QString buff;
    double time_point;
    bool start_record = false;
    while (!file.atEnd()) {
        buff = file.readLine();

        QString str1, str2, str3, str4, str5, str6, t_str, p_str;
        int Startpos = buff.indexOf("<time>"), Endpos = buff.indexOf("</time>"), num = Endpos - Startpos;
        t_str = buff.mid(Startpos + 6, num - 6);


        Startpos = buff.indexOf("<AX1>"), Endpos = buff.indexOf("</AX1>"), num = Endpos - Startpos;
        str1 = buff.mid(Startpos + 5, num - 5);

        Startpos = buff.indexOf("<AX2>"), Endpos = buff.indexOf("</AX2>"), num = Endpos - Startpos;
        str2 = buff.mid(Startpos + 5, num - 5);

        Startpos = buff.indexOf("<AX3>"), Endpos = buff.indexOf("</AX3>"), num = Endpos - Startpos;
        str3 = buff.mid(Startpos + 5, num - 5);

        Startpos = buff.indexOf("<AX4>"), Endpos = buff.indexOf("</AX4>"), num = Endpos - Startpos;
        str4 = buff.mid(Startpos + 5, num - 5);

        Startpos = buff.indexOf("<AX5>"), Endpos = buff.indexOf("</AX5>"), num = Endpos - Startpos;
        str5 = buff.mid(Startpos + 5, num - 5);

        Startpos = buff.indexOf("<AX6>"), Endpos = buff.indexOf("</AX6>"), num = Endpos - Startpos;
        str6 = buff.mid(Startpos + 5, num - 5);

        Startpos = buff.indexOf("<power>"), Endpos = buff.indexOf("</power>"), num = Endpos - Startpos;
        p_str = buff.mid(Startpos + 7, num - 7);

        if (qAbs(str1.toDouble()) < 3 && qAbs(str4.toDouble()) < 3 && qAbs(str5.toDouble()) < 3 && qAbs(str6.toDouble()) < 3) {
            continue;
        }
        if (start_record == false) {
            time_point = t_str.toDouble();
            start_record = true;
        }


        QString item_str = "<time>" + QString::number(t_str.toDouble() - time_point, 'f', 4) + "</time><AX1>" + str1 + "</AX1><AX2>" + str2 + "</AX2><AX3>" + str3
                    +"</AX3><AX4>" + str4 + "</AX4><AX5>" + str5 + "</AX5><AX6>" + str6 + "</AX6><power>" + p_str + "</power>";

        QStandardItem* item = new QStandardItem(item_str);
        item->setCheckable(true);
        list << item;
    }


    file.close();


    // 创建模型并设置数据
    model->appendColumn(list);

    // 创建列表视图并设置模型
    QListView *listView = new QListView();
    listView->setModel(model);

    // 创建勾选框和选中列表
    QCheckBox *checkBox = new QCheckBox("Select All(适用于少量轨迹时)");
    selectedItems = new QList<int>();

    drawPic_btn = new QPushButton();

    QObject::connect(drawPic_btn, &QPushButton::clicked, this, &Widget::drawChartView);
    drawPic_btn->setText("确定");
    drawPic_btn->resize(48, 18);

    // 创建布局并将列表视图和勾选框添加进去
    QHBoxLayout *hlayout = new QHBoxLayout();
    hlayout->addWidget(checkBox);
    hlayout->addWidget(drawPic_btn);
    hlayout->setSpacing(1000);
    QVBoxLayout *layout = new QVBoxLayout();


    layout->addWidget(listView);
    layout->addLayout(hlayout);

    // 创建窗口并设置布局
    window = new QWidget();
    window->setLayout(layout);

    // 连接勾选框的状态改变信号和槽函数
    QObject::connect(checkBox, &QCheckBox::stateChanged, [this](int state){
        selectedItems->clear();
        if (state == Qt::Checked) {
            for (int i = 0; i < model->rowCount(); i++) {
                QModelIndex index = model->index(i, 0);
                model->setData(index, Qt::Checked, Qt::CheckStateRole);
                if (!selectedItems->contains(i)) {
                    selectedItems->append(i);
                }
            }
        } else {
            for (int i = 0; i < model->rowCount(); i++) {
                QModelIndex index = model->index(i, 0);
                model->setData(index, Qt::Unchecked, Qt::CheckStateRole);
            }
        }

    });

    // 连接列表项的勾选框的状态改变信号和槽函数
    QObject::connect(model, &QAbstractItemModel::dataChanged, [this](const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles){
        for (int i = topLeft.row(); i <= bottomRight.row(); i++) {
            QModelIndex index = model->index(i, 0);
            QVariant data = model->data(index, Qt::CheckStateRole);

            if (data == Qt::Checked) {
                if (!selectedItems->contains(i)) { // 选中了没有被包含
                    selectedItems->append(i);
                }

            } else if (selectedItems->contains(i)) { //没有被选中但是包含了的
                selectedItems->removeOne(i);
            }

        }
    });




    window->resize(1270, 655);
    window->show();
}


//轨迹清除
void Widget::traject_clear_func() {
    QList<QPointF> points;
    QChart *chart1 = creatChart(points, "q1(deg)");
    ui->Joint1_Graph->setChart(chart1);
    ui->Joint1_Graph->setRenderHint(QPainter::Antialiasing);


    QChart *chart2 = creatChart(points, "q2(deg)");
    ui->Joint2_Graph->setChart(chart2);
    ui->Joint2_Graph->setRenderHint(QPainter::Antialiasing);

    QChart *chart3 = creatChart(points, "q3(deg)");
    ui->Joint3_Graph->setChart(chart3);
    ui->Joint3_Graph->setRenderHint(QPainter::Antialiasing);

    QChart *chart4 = creatChart(points, "q4(deg)");
    ui->Joint4_Graph->setChart(chart4);
    ui->Joint4_Graph->setRenderHint(QPainter::Antialiasing);

    QChart *chart5 = creatChart(points, "q5(deg)");
    ui->Joint5_Graph->setChart(chart5);
    ui->Joint5_Graph->setRenderHint(QPainter::Antialiasing);

    QChart *chart6 = creatChart(points, "q6(deg)");
    ui->Joint6_Graph->setChart(chart6);
    ui->Joint6_Graph->setRenderHint(QPainter::Antialiasing);
}

/****************************************************/
// TCP server 部分


QByteArray Widget::readData() {
    char buf[9]={0};
    // 每次取8个字节长度

    tcpsocket->read(buf, 8);
    QByteArray row = QByteArray::fromRawData(buf, 8);
    // QByteArray 转换为 QString
    // QString str = QString(row);

    return row;

    /*
    QByteArray temp = tcpsocket->readAll();
    ui->textEditRead->append(temp);
    */
}

void Widget::sendData(QString str) {
    if (tcpsocket == NULL) {
       QMessageBox::warning(this, "错误", "未与机器人连接！");
       return;
    }
    string data = str.toStdString();
    // 发送数据
    tcpsocket->write(data.c_str(), data.size());

}


int Widget::ConnectToRobot() {
    if (tcpserver == NULL) {
        tcpserver = new QTcpServer();
    }

    QString address_text = ui->Input_IP->text();
    QHostAddress address;
    if (address_text.isEmpty()) {

        ui->Input_IP->setText("172.31.1.11");
        address = QHostAddress("172.31.1.11");

    } else {

        address = QHostAddress(address_text);

    }

    unsigned short port;
    if (ui->Input_PORT->text().isEmpty()) {
        port = 8080;
        ui->Input_PORT->setText("8080");
    } else {
        port = ui->Input_PORT->text().toUShort();
    }

    if (tcpserver->listen(address, port)) {
        ui->Input_IP->setEnabled(false);
        ui->Input_PORT->setEnabled(false);

    } else {

        ui->StartServer->setEnabled(true);
        if (tcpserver != NULL) {
            delete tcpserver;
            tcpserver = NULL;
        }
        if (tcpsocket != NULL) {
            delete tcpsocket;
            tcpsocket = NULL;
        }



        QMessageBox::warning(this, "错误", "IP或端口输入错误！");
        return -1;
    }

    connect(tcpserver, SIGNAL(newConnection()), this, SLOT(delegateFunc()));

    return 0;
}

void Widget::delegateFunc() {
    tcpsocket = tcpserver->nextPendingConnection(); // 取到这个连接
    //QHostAddress clientIP = tcpsocket->peerAddress();
    //short clientPORT = tcpsocket->peerPort();
    connect(tcpsocket, SIGNAL(disconnected()), this, SLOT(ClientDisconnected()));//客户端掉线处理
    connect(tcpsocket, SIGNAL(readyRead()), this, SLOT(receivedData()));//客户端发来的数据处理

}


void Widget::ClientDisconnected() {
    QTcpSocket *tcpSocket = static_cast<QTcpSocket *>( QObject::sender() );
    time_clock_run = false;
    tcpSocket->close();

}

void Widget::receivedData() {

    QDateTime timeDate = QDateTime::currentDateTime();  // 获取当前时间
    long long timeStr = timeDate.toMSecsSinceEpoch();   				// 将当前时间转为时间戳
    double timePoint = timeStr / 1000.0;

    if (time_clock_run == false) {
        time_clock = timePoint;
        time_clock_run = true;
    }

    QTcpSocket *tcpSocket = static_cast<QTcpSocket *>( QObject::sender() );
    qDebug() <<QString("收到来自客户端 %1 的消息：").arg(tcpSocket->peerAddress().toString());
    //QByteArray ba = tcpSocket->readLine(300);
    QByteArray ba = tcpSocket->readAll();

    timePoint -= time_clock;

    QString text = "<time>" + QString::number(timePoint, 10, 4) + "</time>" + QString(ba);

    QFile file(sampling_data_path);

    if (mb != NULL) {
        uint16_t active_power[2]{0}; // 有功功率


        int ret = modbus_read_input_registers(mb, 65, 2, active_power);
        //modbus_write_registers(mb, 65, 2, active_power);
        if (ret == -1) {
            qDebug() << "read error \n";
        }
        float power = modbus_get_float_abcd(active_power);


        text +=  "<power>"+QString("%1").arg(power) + "</power>\n";
    }


    //设定文件的打开方式
    file.open(QIODevice::Append | QIODevice::Text);


    //向文件中写入内容
    qDebug() << text;
    QTextStream stream(&file);
    stream << text;



    file.close();

    ui->traject_receive->clear();
    ui->traject_receive->append(text);
     //qDebug() << ba;//打印收到的来自客户端的消息
    QString str;
    int Startpos = text.indexOf("<AX1>"), Endpos = text.indexOf("</AX1>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 5, num - 5);
    ui->A1_Joint->setText(str);

    Startpos = text.indexOf("<AX2>"), Endpos = text.indexOf("</AX2>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 5, num - 5);
    ui->A2_Joint->setText(str);

    Startpos = text.indexOf("<AX3>"), Endpos = text.indexOf("</AX3>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 5, num - 5);
    ui->A3_Joint->setText(str);

    Startpos = text.indexOf("<AX4>"), Endpos = text.indexOf("</AX4>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 5, num - 5);
    ui->A4_Joint->setText(str);

    Startpos = text.indexOf("<AX5>"), Endpos = text.indexOf("</AX5>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 5, num - 5);
    ui->A5_Joint->setText(str);

    Startpos = text.indexOf("<AX6>"), Endpos = text.indexOf("</AX6>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 5, num - 5);
    ui->A6_Joint->setText(str);


    Startpos = text.indexOf("<PX>"), Endpos = text.indexOf("</PX>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 4, num - 4);
    ui->OUT_PX->setText(str);

    Startpos = text.indexOf("<PY>"), Endpos = text.indexOf("</PY>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 4, num - 4);
    ui->OUT_PY->setText(str);

    Startpos = text.indexOf("<PZ>"), Endpos = text.indexOf("</PZ>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 4, num - 4);
    ui->OUT_PZ->setText(str);

    Startpos = text.indexOf("<PA>"), Endpos = text.indexOf("</PA>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 4, num - 4);
    ui->OUT_PA->setText(str);

    Startpos = text.indexOf("<PB>"), Endpos = text.indexOf("</PB>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 4, num - 4);
    ui->OUT_PB->setText(str);

    Startpos = text.indexOf("<PC>"), Endpos = text.indexOf("</PC>"), num = Endpos - Startpos;
    str = text.mid(Startpos + 4, num - 4);
    ui->OUT_PC->setText(str);




}

int Widget::closeConnection() {

    ui->Input_IP->setEnabled(true);
    ui->Input_PORT->setEnabled(true);
    return 0;
}



void Widget::on_StartServer_clicked()
{
    if (tcpserver == NULL) {
        ui->StartServer->setEnabled(false);
        ConnectToRobot();
    }
}

void Widget::on_StopServer_clicked()
{



    if (!ui->StartServer->isEnabled()) {
        closeConnection();
        time_clock_run = false;
        ui->StartServer->setEnabled(true);
        delete tcpserver;
        tcpserver = NULL;
    }

}



void Widget::on_sendBtn_clicked()
{
    sendData(ui->datatoSend->toPlainText());
}



// 控制机器人运动部分
/************************************************************/
void Widget::InitRobotControl() {

    connect(ui->AXIS1_P, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS1_M, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS2_P, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS2_M, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS3_P, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS3_M, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS4_P, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS4_M, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS5_P, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS5_M, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS6_P, &QPushButton::clicked, this, &Widget::Axis_Control);
    connect(ui->AXIS6_M, &QPushButton::clicked, this, &Widget::Axis_Control);
    //关联世界坐标控制
    connect(ui->ToTarg_Pos, &QPushButton::clicked, this, &Widget::Position_set);
    connect(ui->ToInit_Pos, &QPushButton::clicked, this, &Widget::Position_Reset);
    connect(ui->safeExit, &QPushButton::clicked, this, &Widget::ExitSafely_func);
    connect(ui->traject_add, &QPushButton::clicked, this, &Widget::traject_add_func);
    connect(ui->traject_clear, &QPushButton::clicked, this, &Widget::traject_clear_func);
    connect(ui->cal_Energy, &QPushButton::clicked, this, &Widget::cal_Energy_func);
    connect(ui->cal_min_Energy, &QPushButton::clicked, this, &Widget::cal_min_Energy_func);
    connect(ui->optimal_Drawing, &QPushButton::clicked, this, &Widget::optimal_Drawing_func);
    //generate_opt_file_func
    connect(ui->generate_opt_file, &QPushButton::clicked, this, &Widget::generate_opt_file_func);
    connect(ui->draw_para_graph, &QPushButton::clicked, this, &Widget::draw_para_graph_func);
    connect(ui->traject_compare, &QPushButton::clicked, this, &Widget::traject_compare_func);
}


void Widget::Axis_Control() {
    if (tcpsocket == NULL) {
       QMessageBox::warning(this, "错误", "未与机器人连接！");
       return;
    }
     QPushButton *btn = static_cast<QPushButton *>( QObject::sender() );
     qDebug() << "Axis_Control" << " \n";
     if (btn->text() == "A1+") {
        string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>0</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
        tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A1-") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>1</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A2+") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>2</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A2-") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>3</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A3+") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>4</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A3-") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>5</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A4+") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>6</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A4-") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>7</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A5+") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>8</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A5-") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>9</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A6+") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>10</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     } else if (btn->text() == "A6-") {
         string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>11</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
         tcpsocket->write(sendStr.c_str(), sendStr.size());
     }
}

void Widget::Position_Reset() {
    //1340, 0, 1810, 180, 90, 180
    string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>13</M_MODE><PX>0</PX><PY>0</PY><PZ>0</PZ><PA>0</PA><PB>0</PB><PC>0</PC></Sensor>";
    if (tcpsocket != NULL) {
        tcpsocket->write(sendStr.c_str(), sendStr.size());
    } else {
        QMessageBox::warning(this, "错误", "未与机器人连接！");
    }



}

void Widget::Position_set() {
    if (tcpsocket == NULL) {
       QMessageBox::warning(this, "错误", "未与机器人连接！");
       return;
    }
    if (ui->PX->text().isEmpty() || ui->PY->text().isEmpty() || ui->PZ->text().isEmpty() || ui->PA->text().isEmpty()
           || ui->PB->text().isEmpty() || ui->PC->text().isEmpty()) {
        QMessageBox::warning(this, "错误", "文本框不能为空！");
    } else {
        string sendStr = "<Sensor><Status><IsExit>FALSE</IsExit></Status><M_MODE>12</M_MODE><PX>" + ui->PX->text().toStdString()
                + "</PX><PY>" + ui->PY->text().toStdString() + "</PY><PZ>" + ui->PZ->text().toStdString() + "</PZ><PA>"
                + ui->PA->text().toStdString() + "</PA><PB>" + ui->PB->text().toStdString() + "</PB><PC>"
                + ui->PC->text().toStdString() + "</PC></Sensor>";
        tcpsocket->write(sendStr.c_str(), sendStr.size());
    }
}



// 功率采集
/************************************************************/

void Widget::on_recordPower_clicked()
{
    threadRun = true;

    QString address = ui->Input_IP_2->text();

    if (address.isEmpty()) {
        ui->Input_IP_2->setText("172.31.1.146");
        address = "172.31.1.146";
    }

    unsigned short port;
    if (ui->Input_PORT_2->text().isEmpty()) {
        port = 502;
        ui->Input_PORT_2->setText("502");
    } else {
        port = ui->Input_PORT_2->text().toUShort();
    }


    ui->recordPower->setEnabled(false);

    mb = modbus_new_tcp(address.toStdString().c_str(), port);
    qDebug() << "address  " << address << "port" << port << " \n";

    modbus_set_debug(mb, TRUE);

    if (modbus_connect(mb) == -1) {
        qDebug() << "connect error\n";
    } else {
        qDebug() << "connect successfully\n";
    }
    modbus_set_byte_timeout(mb, 1, 0);
    connect(this, SIGNAL(receivePowerSignal()), this, SLOT(receivePower()));

    startRecordPower(this);
}

void Widget::receivePower() {
    uint16_t active_power[2]{0}; // 有功功率

    QDateTime timeDate = QDateTime::currentDateTime();  // 获取当前时间
    int ret = modbus_read_input_registers(mb, 65, 2, active_power);
    //modbus_write_registers(mb, 65, 2, active_power);
    if (ret == -1) {
        qDebug() << "read error \n";
    }


    float power = modbus_get_float_abcd(active_power);


    QString text = timeDate.toString() + "  " + QString("%1").arg(power) + "\n";
    ui->power_receive->clear();
    ui->power_receive->append(text);

}


//不能操作widget,但是可以发送信号
void startRecordPower(Widget *w) {

    class thread t(recordPower, w);

    t.detach();
}


void recordPower(Widget *w) {
    while (threadRun) {
        emit w->receivePowerSignal();
        this_thread::sleep_for(chrono::milliseconds(40));// 每40ms采集一次功率
    }
}

void Widget::on_stopRecordPower_clicked()
{
    threadRun = false;
    ui->recordPower->setEnabled(true);
}

//轨迹、能耗相关计算
/*****************************************************************/

void Widget::cal_Energy_func() {
    if (selectedItems == NULL) {
        return;
    }
    // 两点之间当做梯形计算面积，（上底+下底）* 高/2
    int size = selectedItems->size();


    QString str1, str2, str3, str4, str5, str6, t_str, p_str;
    VectorXd t_vec, p_vec;
    t_vec.resize(size);
    p_vec.resize(size);

    for (int i = 0; i < size; ++i) {
        int row = (*selectedItems)[i];
        QModelIndex index = model->index(row, 0);
        QString buff = model->data(index).toString();

        int Startpos = buff.indexOf("<time>"), Endpos = buff.indexOf("</time>"), num = Endpos - Startpos;
        t_str = buff.mid(Startpos + 6, num - 6);
        t_vec(i) = t_str.toDouble();


        Startpos = buff.indexOf("<power>"), Endpos = buff.indexOf("</power>"), num = Endpos - Startpos;
        p_str = buff.mid(Startpos + 7, num - 7);
        p_vec(i) = p_str.toDouble();
    }

    double Total_Energy = 0;
    for (int i = 0; i < size - 1; ++i) {
        Total_Energy += ((p_vec(i) + p_vec(i + 1)) * (t_vec(i + 1) - t_vec(i)) / 2);
    }

    ui->E_Consumation->setText(QString::number(Total_Energy));
    if (optimal_result.empty()) {
        ui->Opt_Consumation->setText(QString::number(Total_Energy));
    } else {
        ui->Opt_Consumation->setText(QString::number(optimal_result.begin()->first));
    }


}



void Widget::cal_min_Energy_func() {
    if (selectedItems == NULL) {
        return;
    }
    // 两点之间当做梯形计算面积，（上底+下底）* 高/2
    int size = selectedItems->size();
    double *factor = Init_dyn_factor();
    double factor_set = ui->dynm_factor->text().toDouble();
    *factor = factor_set;

    double *delta = Init_DELTA();
    *delta = ui->delta_text->text().toDouble();

    int *segments = Init_SEGMENTS();
    int segments_set = ui->sampling_num->text().toInt();
    *segments = segments_set;



    //*segments = segments_set;

    int *nu = Init_NU();
    *nu = size / segments_set;


    QString str1, str2, str3, str4, str5, str6, t_str, p_str;
    VectorXd t_vec, p_vec, q1_vec, q2_vec, q3_vec, q4_vec, q5_vec, q6_vec;
    t_vec.resize(size);
    p_vec.resize(size);
    q1_vec.resize(size);
    q2_vec.resize(size);
    q3_vec.resize(size);
    q4_vec.resize(size);
    q5_vec.resize(size);
    q6_vec.resize(size);

    for (int i = 0; i < size; ++i) {
        int row = (*selectedItems)[i];
        QModelIndex index = model->index(row, 0);
        QString buff = model->data(index).toString();

        int Startpos = buff.indexOf("<time>"), Endpos = buff.indexOf("</time>"), num = Endpos - Startpos;
        t_str = buff.mid(Startpos + 6, num - 6);
        t_vec(i) = t_str.toDouble();

        Startpos = buff.indexOf("<AX1>"), Endpos = buff.indexOf("</AX1>"), num = Endpos - Startpos;
        str1 = buff.mid(Startpos + 5, num - 5);
        q1_vec(i) = str1.toDouble();

        Startpos = buff.indexOf("<AX2>"), Endpos = buff.indexOf("</AX2>"), num = Endpos - Startpos;
        str2 = buff.mid(Startpos + 5, num - 5);
        q2_vec(i) = str2.toDouble();

        Startpos = buff.indexOf("<AX3>"), Endpos = buff.indexOf("</AX3>"), num = Endpos - Startpos;
        str3 = buff.mid(Startpos + 5, num - 5);
        q3_vec(i) = str3.toDouble();

        Startpos = buff.indexOf("<AX4>"), Endpos = buff.indexOf("</AX4>"), num = Endpos - Startpos;
        str4 = buff.mid(Startpos + 5, num - 5);
        q4_vec(i) = str4.toDouble();

        Startpos = buff.indexOf("<AX5>"), Endpos = buff.indexOf("</AX5>"), num = Endpos - Startpos;
        str5 = buff.mid(Startpos + 5, num - 5);
        q5_vec(i) = str5.toDouble();

        Startpos = buff.indexOf("<AX6>"), Endpos = buff.indexOf("</AX6>"), num = Endpos - Startpos;
        str6 = buff.mid(Startpos + 5, num - 5);
        q6_vec(i) = str6.toDouble();

        Startpos = buff.indexOf("<power>"), Endpos = buff.indexOf("</power>"), num = Endpos - Startpos;
        p_str = buff.mid(Startpos + 7, num - 7);
        p_vec(i) = p_str.toDouble();
    }


    VectorXd E_vec = VectorXd::Zero(size);
    for (int i = 0; i < size - 1; ++i) {
        E_vec(i) = ((p_vec(i) + p_vec(i + 1)) * (t_vec(i + 1) - t_vec(i)) / 2);
    }

    //cout << E_vec << endl;
    int NU = size / segments_set;
    double ***costMatrix = new double**[NU];
    for (int i = 0; i < NU; ++i) {
        costMatrix[i] = new double*[NT];
        for (int j = 0; j < NT; ++j) {
            costMatrix[i][j] = new double[NV]{0};
        }
    }

    //double costMartrix[NU][NT][NV]{{{0}}};
    Path ***path_back = new Path**[NU];
    for (int i = 0; i < NU; ++i) {
        path_back[i] = new Path*[NT];
        for (int j = 0; j < NT; ++j) {
            path_back[i][j] = new Path[NV];
        }
    }
    //Path path_back[NU][NT][NV];
    MatrixXd totalEnergy = MatrixXd::Zero(NT, NV);
    VectorXd hm = VectorXd::Zero(NU);

    VectorXd q1, q2, q3, q4, q5, q6;
    q1.resize(NU);
    q2.resize(NU);
    q3.resize(NU);
    q4.resize(NU);
    q5.resize(NU);
    q6.resize(NU);
    for (int i = 0; i < NU; ++i) {
        q1(i) = q1_vec(segments_set * i);
        q2(i) = q2_vec(segments_set * i);
        q3(i) = q3_vec(segments_set * i);
        q4(i) = q4_vec(segments_set * i);
        q5(i) = q5_vec(segments_set * i);
        q6(i) = q6_vec(segments_set * i);
    }

    //MatrixXd joints(size, 6);

    joints.resize(NU, 6);
    //joints << q1_vec, q2_vec, q3_vec, q4_vec, q5_vec, q6_vec;
    joints << q1, q2, q3, q4, q5, q6;
    VectorXd t;
    t.resize(NU);
    for (int i = 0; i < NU; ++i) {
        t(i) = t_vec(segments_set * i);
    }


    vector<vector<vector<bool>>> isScaled (NU, vector<vector<bool>>(NT, vector<bool>(NV)));

    MainAlogrithm(costMatrix,path_back, isScaled, joints, t, E_vec);
    /*
    for (int k = 0; k < NT; ++k) {
        for (int h = 0; h < NV; ++h) { // h用来求出m+1时的t以及u'
            int j = k, i = h;
            int m = NU - 1;
            double t_E = 0;
            while (m > 0) {
                 t_E += costMatrix[m][j][i];
                 j = path_back[m][j][i].t_index;
                 i = path_back[m][j][i].v_index;
                 --m;
            }
            totalEnergy(k, h) = t_E;
        }
    }

    std::ptrdiff_t i, j;
    double result = totalEnergy.minCoeff(&j, &i);
    */
    double result = Calculatehm(costMatrix, path_back, hm, isScaled);
    qDebug() << "最小能量是 --- " << result << " \n";

    optimal_time.resize(NU);
    optimal_time(0) = 0;
    for (int i = 1; i < NU; ++i) {
        optimal_time(i) = hm(i- 1) + optimal_time(i - 1);
    }
    //cout << optimal_time << endl;
    optimal_result.insert(pair<double, double>(result, factor_set));
    ui->optimal_factor->setText(QString::number(optimal_result.begin()->second));

    ui->E_Consumation->setText(QString::number(result));
    ui->Opt_Consumation->setText(QString::number(optimal_result.begin()->first));

}


void Widget::draw_para_graph_func() {
    if (optimal_time.size() == 0) {
        return;
    }
    MatrixXd bMatrix = getParameterB();
    int numOfControl = bMatrix.cols();
    QList<QPointF> points;
    vector<double> sumOfBVec(numOfControl);
    for (int i = 0; i < numOfControl - 1; ++i) {
        points << QPointF(i, bMatrix.col(i).sum());
        qDebug() << bMatrix.col(i).sum() << " \n";
    }

    QChart *chart = creatChart(points, "BVec-approx", "i", -0.5, 0.5);
    ui->parameter_graph->setChart(chart);
    ui->parameter_graph->setRenderHint(QPainter::Antialiasing);


}


// 生成机器人最优轨迹文件
void Widget::generate_opt_file_func() {
    QFile file(kuku_ptpsrc_path);
    if (!file.open(QIODevice::ReadWrite | QIODevice::Text))
    {
        cout << "打开文件失败" << endl;
        // 打开文件失败
        return;
    }



    if (selectedItems == NULL) {
            return;
    }
    // 两点之间当做梯形计算面积，（上底+下底）* 高/2
    int SEGMENTS = *Init_SEGMENTS();
    int size = selectedItems->size();
    QString str1, str2, str3, str4, str5, str6;



    QTextStream out(&file);
    out << "PTP_SPLINE\n";

    out << "  SPTP {A1 0.000000,A2 -90.000000,A3 90.000000,A4 0.000000,A5 0.000000,A6 0.000000}\n";
    out << "TIME_BLOCK START\n";

    double refer_time = -1;
    for (int i = 0; i < size; ++i) {
        int row = (*selectedItems)[i];
        QModelIndex index = model->index(row, 0);
        QString buff = model->data(index).toString();

        int t_index = i / SEGMENTS;
        qDebug() << "t_index = " << t_index << "optimal_time.size() = " << optimal_time.size() << " \n";
        int count = i % SEGMENTS;
        double interval = 0;
        if (t_index == size / SEGMENTS - 1) {
            break; //后面的点舍弃
        }
        if (t_index < optimal_time.size() - 1)    {
            interval = (optimal_time(t_index + 1) - optimal_time(t_index)) / SEGMENTS;
        }
        double d_interval = double(count) * interval;

        int Startpos = buff.indexOf("<AX1>"), Endpos = buff.indexOf("</AX1>"), num = Endpos - Startpos;
        str1 = buff.mid(Startpos + 5, num - 5);



        Startpos = buff.indexOf("<AX2>"), Endpos = buff.indexOf("</AX2>"), num = Endpos - Startpos;
        str2 = buff.mid(Startpos + 5, num - 5);


        Startpos = buff.indexOf("<AX3>"), Endpos = buff.indexOf("</AX3>"), num = Endpos - Startpos;
        str3 = buff.mid(Startpos + 5, num - 5);


        Startpos = buff.indexOf("<AX4>"), Endpos = buff.indexOf("</AX4>"), num = Endpos - Startpos;
        str4 = buff.mid(Startpos + 5, num - 5);


        Startpos = buff.indexOf("<AX5>"), Endpos = buff.indexOf("</AX5>"), num = Endpos - Startpos;
        str5 = buff.mid(Startpos + 5, num - 5);


        Startpos = buff.indexOf("<AX6>"), Endpos = buff.indexOf("</AX6>"), num = Endpos - Startpos;
        str6 = buff.mid(Startpos + 5, num - 5);




        out << QString("  SPTP {A1 %1,A2 %2,A3 %3,A4 %4,A5 %5,A6 %6}\n").arg(str1).arg(str2).arg(str3).arg(str4).arg(str5).arg(str6);
        double time_interVal = optimal_time(t_index) - refer_time + d_interval;
        //if (count > 1) {
        //    time_interVal = d_interval;
        //}
        out << QString("  TIME_BLOCK PART = %1\n").arg(time_interVal, 0, 'f', 4);
        refer_time = optimal_time(t_index) + d_interval;
    }


    out << "  SPTP {A1 0.000000,A2 -90.000000,A3 90.000000,A4 0.000000,A5 0.000000,A6 0.000000}\n";
    out << "  TIME_BLOCK PART = 1\n";

    int NU = *Init_NU();

    out << QString("TIME_BLOCK END = %1\n").arg(optimal_time(NU - 1) + 1, 0, 'f', 4);
    //out << "TIME_BLOCK END = 12\n";
    out << "ENDSPLINE";
    file.close();
}


#define random(x) (rand()%x)
#define NUM_LOOP 10


void generate_trajectory() {
    srand((int)time(0));
    MatrixXd t1(1, 11);
    t1 << 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0, 0;

    for (int i = 1; i < NUM_LOOP; ++i) {
        double num = random(3000) / 1000.0 + 1; //随机产生0-9十个数
        t1(i) += num + t1(i - 1);
    }
    cout << t1 << endl;


    MatrixXd q1(1, 10);
    q1 << 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0;

    for (int i = 1; i < NUM_LOOP / 2; ++i) {
        double num = random(30000) / 1000.0; //随机产生0-9十个数
        q1(i) -= num;
    }
    for (int i = NUM_LOOP / 2; i < NUM_LOOP - 1; ++i) {
        double num = random(30000) / 1000.0; //随机产生0-9十个数
        q1(i) += num;
    }
    cout << q1 << endl;


    MatrixXd q2(1, 10);

    q2 << -90, -90, -90, -90, -90, -90, -90, -90, -90 , -90;

    for (int i = 1; i < NUM_LOOP - 1; ++i) {
        double num = random(30000) / 1000.0; //随机产生0-9十个数
        q2(i) += num;
    }

    cout << q2 << endl;


    MatrixXd q3(1, 10);
    q3 << 90, 90, 90, 90, 90, 90, 90, 90, 90 , 90;

    for (int i = 1; i < NUM_LOOP - 1; ++i) {
        double num = random(30000) / 1000.0; //随机产生0-9十个数
        q3(i) += num;
    }

    cout << q3 << endl;

    MatrixXd q4(1, 10);
    q4 << 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0;

    for (int i = 1; i < NUM_LOOP - 1; ++i) {
        double num = random(10000) / 1000.0 + 10; //随机产生0-9十个数
        q4(i) -= num * 10;
    }

    cout << q4 << endl;

    MatrixXd q5(1, 10);
    q5 << 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0;

    for (int i = 1; i < NUM_LOOP - 1; ++i) {
        double num = random(30000) / 1000.0 + 30; //随机产生0-9十个数
        q5(i) += num;
    }

    cout << q5 << endl;

    MatrixXd q6(1, 10);
    q6 << 0, 0, 0, 0, 0, 0, 0, 0, 0 , 0;

    for (int i = 1; i < NUM_LOOP - 1; ++i) {
        double num = random(10000) / 1000.0 + 10; //随机产生0-9十个数
        q6(i) += num * 10;
    }

    cout << q6 << endl;
#if 0

    //测试顺序，从q1开始，依次到q6
    MatrixXd E(1, 10);
    E << 70, 70, 70, 70, 70, 70, 70, 70, 70 , 70;
    for (int i = 0; i < NUM_LOOP - 1; ++i) {
        double num = random(20000) / 1000.0 + 10;
        E(i) += num * 10;
    }

    cout << E << endl;
    //double costMartrix[NU][NT][NV] = {{{0}}};
    double costMartrix[NU][NT][NV]{{{0}}};
    Path path_back[NU][NT][NV];
    MatrixXd totalEnergy = MatrixXd::Zero(NT, NV);



    VectorXd m_t = t1.row(0).transpose();
    VectorXd m_E = E.row(0).transpose();
    VectorXd m_q = q1.row(0).transpose();
    qDebug() << "MainAlogrithm ::>>" << endl;
    MainAlogrithm(costMartrix,path_back, m_q, m_t, m_E);

    for (int k = 0; k < NT; ++k) {
        for (int h = 0; h < NV; ++h) { // h用来求出m+1时的t以及u'
            int j = k, i = h;
            int m = NU - 1;
            double t_E = 0;
            while (m > 0) {
                 t_E += costMartrix[m][j][i];
                 j = path_back[m][j][i].t_index;
                 i = path_back[m][j][i].v_index;
                 --m;
            }
            totalEnergy(k, h) = t_E;
        }
    }

    std::ptrdiff_t i, j;
    double result = totalEnergy.minCoeff(&j, &i);
    qDebug() << "最小能量是 --- " << result << endl;
    int m = NU - 1;

    Path result_path[NU];
    while (m > 0) {
         j = path_back[m][j][i].t_index;
         i = path_back[m][j][i].v_index;
         result_path[m].t_index = j;
         result_path[m].v_index = i;
         --m;
    }

    //double testMatrix[NU][NT]{{0}};
    qDebug() << "路径为:" << endl;
    for (int i = 0; i < NU; ++i) {
        qDebug() << "m = " << i <<  " t_index = " << result_path[i].t_index <<
                    " v_index = " << result_path[i].v_index << endl;
    }

#endif

}


void Widget::ExitSafely_func()
{
    string sendStr = "<Sensor><Status><IsExit>TRUE</IsExit></Status><M_MODE>0</M_MODE><PX>1340</PX><PY>0</PY><PZ>1810</PZ><PA>180</PA><PB>90</PB><PC>180</PC></Sensor>";
    if (tcpsocket != NULL) {
        tcpsocket->write(sendStr.c_str(), sendStr.size());
    } else {
        QMessageBox::warning(this, "错误", "未与机器人连接！");
    }


}

