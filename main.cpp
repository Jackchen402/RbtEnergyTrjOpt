
#include "widget.h"
#include <QApplication>
//#include <QTextCodec>
#include <QtCore5Compat/QTextCodec>


#include <QChart>
#include <QLineSeries>
#include <QChartView>
#include <random>

//此句必备
//QT_CHARTS_USE_NAMESPACE



int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    sampling_data_path = QCoreApplication::applicationDirPath() + "/sampling_data.txt";
    kuku_ptpsrc_path = QCoreApplication::applicationDirPath() + "/kuka_ptpsrc.txt";
    Widget w;

    //generate_trajectory();

    w.show();

    return a.exec();
}



/*
int ret = modbus_read_input_registers(mb, 1, 2, voltage_l1);
if (ret == -1) {
    qDebug() << "read error \n";
}
float voltage = modbus_get_float_abcd(voltage_l1);
qDebug()  << voltage << endl;
*/

/*
    //cout << test(MatrixXd(2, 3)) << endl;

    VectorXd vec(6);
    vec << 1, 2, 3, 4, 5, 6;
    //cout << vec << endl;
    //cout << vec.array().square() * vec.array() << endl;
    //cout << 4 - vec.array() << endl;
    //cout << vec.array().sqrt() << endl;
    //cout << 1 / vec.array().square() << endl;

    MatrixXd matrix(6, 2);
    matrix << vec, vec;
    //matrix << vec, 1, 2, 3, 4, 5, 6;  //error
    //cout << matrix << endl;
    //Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(matrix);
    //cout << "the rank of matrix is " << lu_decomp.rank() << endl;
    //cout << matrix << endl;
    //VectorXd v = matrix.topRows(1); //error，只能是列向量
    VectorXd v = matrix.topRows(1).transpose();
    cout << v << endl;
*/
