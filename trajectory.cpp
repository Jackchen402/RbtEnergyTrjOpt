#if 1
#include "trajectory.h"
#include <vector>
#include <random>
int NU = 0;
int SEGMENTS = 3;
double DELTA = 0;
double dynm_factor = 0.2;


vector<double> VMatrix;
    // 代价矩阵
//double CostMatrix[NU][NT][NV]{{{0}}};
MatrixXd bMatrix;

// 三次样条插值函数
double spline(double x, const VectorXd& xi, const VectorXd& yi, const VectorXd& y2i)
{
    int n = xi.size();

    int klo = 0, khi = n - 1;

    while (khi - klo > 1) {
        int k = (khi + klo) / 2;
        if (xi(k) > x) {
            khi = k;
        } else {
            klo = k;
        }
    }

    double h = xi(khi) - xi(klo);

    if (h == 0.0) {
        return yi(klo);
    }

    double a = (xi(khi) - x) / h;
    double b = (x - xi(klo)) / h;

    double y = a * yi(klo) + b * yi(khi) + ((a * a * a - a) * y2i(klo) + (b * b * b - b) * y2i(khi)) * (h * h) / 6.0;

    return y;
}

// 计算三次样条插值的二阶导数
void spline(VectorXd x, VectorXd y, int n, double yp1, double ypn, VectorXd &y2)
{
    double *u = new double[n - 1];

    double p, qn, sig, un;

    y2(0) = -0.5;
    u[0] = (3.0 / (x(1) - x(0))) * ((y(1) - y(0)) / (x(1) - x(0)) - yp1);

    for (int i = 1; i < n - 1; i++) {
        sig = (x(i) - x(i - 1)) / (x(i + 1) - x(i - 1));
        p = sig * y2(i - 1) + 2.0;
        y2(i) = (sig - 1.0) / p;
        u[i] = (y(i + 1) - y(i)) / (x(i + 1) - x(i)) - (y(i) - y(i - 1)) / (x(i) - x(i - 1));
        u[i] = (6.0 * u[i] / (x(i + 1) - x(i - 1)) - sig * u[i - 1]) / p;
    }

    qn = 0.5;
    un = (3.0 / (x(n - 1) - x(n - 2))) * (ypn - (y(n - 1) - y(n - 2)) / (x(n - 1) - x(n - 2)));

    y2(n - 1) = (un - qn * u[n - 2]) / (qn * y2(n - 2) + 1.0);

    for (int k = n - 2; k >= 0; k--) {
        y2(k) = y2(k) * y2(k + 1) + u[k];
    }
}


vector<double> CalculateUprim(VectorXd time) {
    // 运动开始之前，机器人是静止的
    VMatrix.resize(NU + 1);
    qDebug() << NU << "\n";

    double NextV, V = V_INI;
    VMatrix[0] = V;
    // t0-->u0, t1-->u1  两者都是通过Nu来联系的
    for (int i = 1; i < NU; ++i) {
        NextV = -V + 2 * DELTA / (time(i) - time(i - 1));// 取对应下标的时间来计算hm
        VMatrix[i] = NextV;
        V = NextV;
    }
    return VMatrix;
}

double GetVGridPoint(int , int v_index,int u_index) {
    if (v_index == 0) {
        v_index = 1;
    }
    return VMatrix[u_index] * v_index * dynm_factor;
}

double GetTGridPoint(int t_index, int,int u_index, VectorXd time) {
    if (t_index == 0) {
        t_index = 1;
    }
    return time(u_index) * t_index * dynm_factor;
}


MatrixXd CalculateMatrixb(VectorXd Energy, VectorXd time) {
    int num_rows = 12;
    bMatrix.resize(num_rows, NU);
    //MatrixXd result(num_rows, NU);
    // 注意：time(m+1)可能越界，取决于NU是否大于time的长度
    for (int m = 0; m < NU - 1; ++m) {
        VectorXd E(SEGMENTS);
        for (int i = 0; i < SEGMENTS; ++i) {
            E(i) = Energy(m + i);
        }

        double Uprim = VMatrix[m];
        double nextUprim = VMatrix[m + 1];
        double gamma = (nextUprim - Uprim) / (time(m + 1) - time(m));  // (v1-v0)/(t1-t0)

        double alpha = (nextUprim - Uprim) / DELTA;  //(v1-v0)/delta

        //qDebug() << "m = " << m << "  CalculateCoefficientb(E, alpha, gamma, Uprim, nextUprim, SEGMENTS)>>:" << "\n ";
        VectorXd b = CalculateCoefficientb(E, alpha, gamma, Uprim, nextUprim, SEGMENTS);
        for (int j = 0; j < num_rows; ++j) {
            bMatrix(j, m) = b(j);
        }
    }
    //cout << result << "\n ";
    return bMatrix;
}


/*   打印三维矩阵
    for (int i = 0; i < NU; ++i) {
        for (int j = 0; j < NT; ++j) {
            for (int  k = 0; k < NV; ++k) {
                cout << CostMatrix[i][j][k] << " ";
            }
            cout << "\n ";
        }
        cout << "\n ";
    }

*/

int* Init_NU() {
    return &NU;
}

double* Init_dyn_factor() {
    return &dynm_factor;
}

double *Init_DELTA() {
    return &DELTA;
}

int *Init_SEGMENTS() {
    return &SEGMENTS;
}

MatrixXd &getParameterB() {
    return bMatrix;
}


void MainAlogrithm(double ***CostMatrix,Path ***path_back, vector<vector<vector<bool>>> &isScaled,MatrixXd joints, VectorXd time, VectorXd Energy) {
    qDebug() << "NU(控制段个数) = " << NU << "\n ";
    qDebug() << "DELTA = " << DELTA << "\n ";
    qDebug() << "CalculateUprim(time)>>:" << "\n ";
    // 计算v的所有值 -- u方向
    CalculateUprim(time);
    // 计算b的所有值 -- 单个关节
    qDebug() << "CalculateMatrixb(Energy, time)>>:" << "\n ";
    MatrixXd bMatrix = CalculateMatrixb(Energy, time);
    cout << bMatrix << "\n ";
    //cout << bMatrix;
    qDebug() << "MatrixXd A(NT, NV)>>:" << "\n ";
    // 单个网格能量矩阵


    //计算三次样条的插值
    double yp1 = 0.0, ypn = 0.0;
    VectorXd y2(NU);
    cout << "Energy.sum = " << Energy.sum() << "\n ";            ;
    qDebug() << "segments * U = " << SEGMENTS * NU << "\n ";
    int  Nu = NU, Nv = NV, Nt = NT;
    for (int num = 0; num < JOINT_NUM; ++num) {
        VectorXd joint = joints.col(num);
        spline(time, joint, NU, yp1, ypn, y2);

        for (int m = 0; m < Nu - 1; ++m) {
        //qDebug() <<"m = " << m << " for (int m = 0; m < Nu; ++m) >>" << "\n ";
        VectorXd b = bMatrix.col(m);
        double Em = 0;
        for (int s = 0; s < SEGMENTS; ++s) {
            Em += Energy(m + s);
        }
        for (int k = 0; k < Nt; ++k) {
            for (int h = 0; h < Nv; ++h) { // h用来求出m+1时的t以及u'

                double tm_next = GetTGridPoint(k, h, m + 1, time);

                double vm_next = GetVGridPoint(k, h, m + 1);

                MatrixXd A = MatrixXd::Constant(NT, NV, DBL_MAX);
                for (int j = 0; j < Nt; ++j) {
                    //读入当前关节qsV以及qsA的值

                    for (int i = 0; i < Nv; ++i) { // i用来求出m时的t以及u'
                        double tm = GetTGridPoint(j, i, m, time);
                        double vm = GetVGridPoint(j, i, m);
                        //if (k >=8 && h >= 9 && m == 29) {
                        //    qDebug() << "if (tm_next - tm <= 1e-8) { >>" << "\n ";
                        //}
                        //if (tm_next - tm <= 1e-6) {

                        //}

                        if ((vm >= -1e-6 && vm <= 1e-6)) {
                            double cost = Em;
                            A(j, i) = CostMatrix[m][j][i] + cost;
                            continue;
                        }

                        if (vm + vm_next < 0) { // 如果该项为0，那么hm求出来一定是负值
                            double cost = Em;
                            A(j, i) = CostMatrix[m][j][i] + cost;
                            continue;
                        }
                        else if (tm_next - tm <= 1e-6) {
                            double cost = Em;
                            A(j, i) = CostMatrix[m][j][i] + cost;
                            continue;
                        }

                        // 使用三点法进行二次求导
                        int midTime = (tm + tm_next) / 2;
                        double midJoint = spline(midTime, time, joint, y2);



                        double qsV = (joint(m + 1) - joint(m)) / (tm_next - tm);
                        double qsA = (joint(m + 1) + joint(m) - 2 * midJoint) / ((tm_next - tm) * (tm_next - tm));
                        //double qsA = 0;

                        double gamma = (vm_next - vm) / (tm_next - tm);
                        double alpha = (vm_next - vm) / DELTA;

                        double qrV = qsV / vm;
                        double qrA = (qsA - gamma * qrV) / (vm * vm);


                        double qrV_MAX = QSV_MAX / vm;
                        double qrA_MAX = (QSA_MAX - gamma * qrV_MAX) / (vm * vm); // 这个等式的含义不太明确，不是等比例缩放

                        if (qrV > qrV_MAX || qrV < -qrV_MAX || qrA > qrA_MAX || qrA < -qrA_MAX) { // 当该关节速度、加速度不满足要求时-- 需要保持原样
                            //if (m == 0)
                            //    qDebug() << "m = " << m <<  " 不满足要求 " << "\n ";
                            double cost = Em;
                            A(j, i) = CostMatrix[m][j][i] + cost;
                            break;
                        } else {

                            if ((b.sum()) < 1e-5) { //(b.sum()/12) < 1e-3
                                double cost = Em;
                                A(j, i) = CostMatrix[m][j][i] + cost;
                            } else {
                                double cost = CalculateCost(b, vm, vm_next, alpha, gamma, SEGMENTS);
                                if (cost <= 0) {
                                    A(j, i) = CostMatrix[m][j][i] + Em;
                                } else{
                                    // 机器人每0.05秒发送一条数据，而最小功率基本都是340，因此功率至少大于17才满足要求
                                    if (cost < Em && cost > 17 * SEGMENTS) {
                                        isScaled[m][j][i] = true;
                                        A(j, i) = CostMatrix[m][j][i] + cost;
                                    } else {
                                        A(j, i) = CostMatrix[m][j][i] + Em;
                                    }

                                }
                            }
                            //if (cost < 5) {
                            //    cout << "m = " << m << "b = " << b.transpose()  << " vm = " << vm  << " vm_next = " << vm_next << "\n ";
                            //}
                            //if (m == 0)
                            //qDebug() <<  "m = " << m << "cost = " << cost  << " CostMatrix[m][j][i] = " << CostMatrix[m][j][i] << "\n ";
                        }





                    }
                }

                std::ptrdiff_t i, j;
                CostMatrix[m + 1][k][h] = A.minCoeff(&j, &i);
                //cout << " A.minCoeff(&j, &i) : " << CostMatrix[m + 1][k][h] << " k = " << k << " h =" << h << " m =" << m << "\n ";
                path_back[m + 1][k][h].t_index = j;
                path_back[m + 1][k][h].v_index = i;

                //qDebug() << "--------------------------" << "\n ";
                //cout << A << "\n ";
                //qDebug() << "m = " << m << " j = " << j << "i = " << i << "\n ";
                //qDebug() << "m + 1 = " << m + 1 << " k = " << k << "h = " << h << "\n ";

            }
        }


    }
}



/*
    for (int i = 0; i < NU; ++i) {
        for (int j = 0; j < NT; ++j) {
            for (int  k = 0; k < NV; ++k) {
                cout << CostMatrix[i][j][k] << " ";
            }
            cout << "\n ";
        }
        cout << "\n ";
    }
    qDebug() << "MainAlogrithm end..........." << "\n ";
*/

}
double Calculatehm(double ***CostMatrix, Path ***path_back, VectorXd &hm, vector<vector<vector<bool>>> &isScaled) {
    MatrixXd totalEnergy = MatrixXd::Zero(NT, NV);
    double mincost = DBL_MAX;
    int i, j;
    for (int k = 0; k < NT; ++k) {
        for (int h = 0; h < NV; ++h) { // h用来求出m+1时的t以及u'
            double cost = CostMatrix[NU - 1][k][h];
            if (mincost > cost) {
                mincost = cost;
                j = k, i = h;
            }
        }
    }




    int m = NU - 1;

    Path *result_path = new Path[NU];
    while (m > 0) {
         j = path_back[m][j][i].t_index;
         i = path_back[m][j][i].v_index;
         if (i == 0) {
            result_path[m - 1].v_index = 1;
         } else {
             result_path[m - 1].v_index = i;
         }
         if (j == 0) {
             result_path[m - 1].t_index = 1;
         } else {
             result_path[m - 1].t_index = j;
         }
         --m;
    }

    for (int i = 0; i < NU - 1; ++i) {

        if (isScaled[i][result_path[i].t_index][result_path[i].v_index]) {
            double dividor = (VMatrix[i + 1] * result_path[i + 1].v_index + VMatrix[i] * result_path[i].v_index);
            if (dividor < 1e-6) {
                dividor = (VMatrix[i + 1] * dynm_factor + VMatrix[i] * dynm_factor);
                qDebug() << "出现一次都为零" << "\n ";
            }
            qDebug() << "m= " << i << " scaled" << "\n ";
            hm(i) = 2 * DELTA / dividor;
        } else {

            double dividor = (VMatrix[i + 1] + VMatrix[i]);
            hm(i) = 2 * DELTA / dividor;
        }



    }

    return mincost;
}


// 计算两个状态之间能量的转移增量
double CalculateCost(VectorXd b, double Uprim, double nextUprim, double alpha, double gamma, int k) {
    MatrixXd CoeMatrix(k, 12);
    double Coe_b1 = Uprim * Uprim * Uprim;
    double Coe_b3 = Uprim * Uprim;
    double Coe_b5 = Uprim;
    double Coe_b8 = alpha;
    double Coe_b10 = 1;
    double Coe_b11 = gamma;
    double Coe_b4b7 = 3 * (alpha * alpha) * Uprim;
    double Coe_b6b2 = gamma * Uprim;
    double Coe_b9b7 = alpha * gamma;

    double *Coe_nonlinear = new double[SEGMENTS * 3]{0};
    //qDebug() << "for (int i = 0; i < k; ++i) >>:" << "\n ";
    for (int i = 0; i < SEGMENTS; ++i) {
        double Coe_Vector = k / ((k - i) * Uprim + i * nextUprim);
        // b12, b13, b14, ... bk , 相当于公式中的 b12, b15, b18... b3i+12
        Coe_nonlinear[i] = 1 * Coe_Vector;
        // 相当于公式中的 b13, b16, b19, ... b3i+13
        Coe_nonlinear[i + k] = gamma * Coe_Vector;
        // 相当于公式中的 b14, b17, b20, ... b3i+14
        Coe_nonlinear[i + 2 * k] = gamma * gamma * Coe_Vector;
    }

    for (int i = 0; i < SEGMENTS; ++i) {
        CoeMatrix(i, 0) = Coe_b1;
        CoeMatrix(i, 1) = Coe_b3;
        CoeMatrix(i, 2) = Coe_b5;
        CoeMatrix(i, 3) = Coe_b8;
        CoeMatrix(i, 4) = Coe_b10;
        CoeMatrix(i, 5) = Coe_b11;
        CoeMatrix(i, 6) = Coe_b4b7;
        CoeMatrix(i, 7) = Coe_b6b2;
        CoeMatrix(i, 8) = Coe_b9b7;
        CoeMatrix(i, 9) = Coe_nonlinear[i];
        CoeMatrix(i, 10) = Coe_nonlinear[i + k];
        CoeMatrix(i, 11) = Coe_nonlinear[i + 2 * k];
    }
    delete []Coe_nonlinear;
    MatrixXd result = CoeMatrix * b;

    return result.sum();
}


// 根据广义逆矩阵论求出待定系数矩阵b
// 由于机器人有多个关节，因此Uprim 和 nextUprim 一定是向量的形式
VectorXd CalculateCoefficientb(VectorXd Energy, double alpha, double gamma, double Uprim, double nextUprim, int k) {
    VectorXd result;
    MatrixXd CoeMatrix(k, 12);
    double Coe_b1 = Uprim * Uprim * Uprim;
    double Coe_b3 = Uprim * Uprim;
    double Coe_b5 = Uprim;
    double Coe_b8 = alpha;
    double Coe_b10 = 1;
    double Coe_b11 = gamma;
    double Coe_b4b7 = 3 * (alpha * alpha) * Uprim;
    double Coe_b6b2 = gamma * Uprim;
    double Coe_b9b7 = alpha * gamma;

    double *Coe_nonlinear = new double[SEGMENTS * 3]{0};
    //qDebug() << "for (int i = 0; i < k; ++i) >>:" << "\n ";
    for (int i = 0; i < SEGMENTS; ++i) {
        double Coe_Vector = k / ((k - i) * Uprim + i * nextUprim);
        // b12, b13, b14, ... bk , 相当于公式中的 b12, b15, b18... b3i+12
        Coe_nonlinear[i] = 1 * Coe_Vector;
        // 相当于公式中的 b13, b16, b19, ... b3i+13
        Coe_nonlinear[i + k] = gamma * Coe_Vector;
        // 相当于公式中的 b14, b17, b20, ... b3i+14
        Coe_nonlinear[i + 2 * k] = gamma * gamma * Coe_Vector;
    }

    for (int i = 0; i < SEGMENTS; ++i) {
        CoeMatrix(i, 0) = Coe_b1;
        CoeMatrix(i, 1) = Coe_b3;
        CoeMatrix(i, 2) = Coe_b5;
        CoeMatrix(i, 3) = Coe_b8;
        CoeMatrix(i, 4) = Coe_b10;
        CoeMatrix(i, 5) = Coe_b11;
        CoeMatrix(i, 6) = Coe_b4b7;
        CoeMatrix(i, 7) = Coe_b6b2;
        CoeMatrix(i, 8) = Coe_b9b7;
        CoeMatrix(i, 9) = Coe_nonlinear[i];
        CoeMatrix(i, 10) = Coe_nonlinear[i + k];
        CoeMatrix(i, 11) = Coe_nonlinear[i + 2 * k];
    }
    //cout << CoeMatrix;
    // 先计算系数矩阵的加号逆
    delete []Coe_nonlinear;
    MatrixXd CoeMatrixInv = SolveSVDInverse(CoeMatrix);

    // 使用广义逆矩阵论求出向量b
    MatrixXd CoeMatrix2 = CoeMatrix;
    CoeMatrix2.conservativeResize(k, 13);
    for (int i = 0; i < SEGMENTS; ++i) {
        CoeMatrix2(i, 12) = Energy(i);
    }

    //cout << CoeMatrix2 << "\n ";

    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(CoeMatrix);
    Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp2(CoeMatrix2);


    int rA = lu_decomp.rank(), rAb = lu_decomp2.rank();
    if (rA == rAb && rA == CoeMatrix.cols()) { // 有唯一解
        result = CoeMatrixInv * Energy;
    } else if (rA == rAb && rA < CoeMatrix.cols()) { //有无穷解
        // 找到它的最小范数解
        result = CoeMatrixInv * Energy;
    } else if (rA < rAb) { //无解，需要求最小二乘解
        // 找到他的最小范数解
        result = CoeMatrixInv * Energy;
    }

    // 最后几个b的顺序影响了计算能量时使用Uprim的顺序
    return result;
}







// 计算系数矩阵的广义逆矩阵
MatrixXd SolveSVDInverse(MatrixXd CoeMatrix) {
    // M = USV^T

    BDCSVD<MatrixXd> svd(CoeMatrix, ComputeFullU | ComputeFullV);
    //JacobiSVD<MatrixXd> svd(CoeMatrix, ComputeFullU | ComputeFullV);
    double tolerance = 1.e-8;
    int rows = CoeMatrix.rows();
    int cols = CoeMatrix.cols();
    int k = min(rows, cols);
    MatrixXd X = MatrixXd::Zero(cols, rows);
    MatrixXd singularValues_inv = svd.singularValues(); //奇异值向量
    MatrixXd singularValues_inv_mat = MatrixXd::Zero(cols, rows);
    for (long i = 0; i < k; ++i) {
        if (singularValues_inv(i) > tolerance) {
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        } else {
            singularValues_inv(i) = 0;
        }
    }

    for (long i = 0; i < k; ++i) {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }

    X = (svd.matrixV()) * (singularValues_inv_mat) * (svd.matrixU().transpose()); // X = V*S^-1*U^T
    return X;

}
#endif
