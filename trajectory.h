#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <Eigen/Dense>
#include <vector>
#include <QtDebug>
#include <iostream>
using namespace Eigen;
using namespace std;
using Eigen::VectorXd;

#define JOINT_NUM 6
#define QSV_MAX 2000
#define QSA_MAX 1000
#define V_INI 10
#define NV 10
#define NT 10

/*
#define NU 10

*/

struct Path {
  int t_index;
  int v_index;
  Path() : t_index(0), v_index(0) {}
};

//extern MatrixXd VMatrix;

vector<double> CalculateUprim(VectorXd time);
MatrixXd CalculateMatrixb(VectorXd Energy, VectorXd time);

MatrixXd SolveSVDInverse(MatrixXd CoeMatrix);
double GetTGridPoint(int t_index, int,int u_index, VectorXd time);
double GetVGridPoint(int , int v_index,int u_index);
VectorXd CalculateCoefficientb(VectorXd Energy, double alpha, double gamma, double Uprim, double nextUprim, int k);
double CalculateCost(VectorXd b, double Uprim, double nextUprim, double alpha, double gamma, int k);

void MainAlogrithm(double ***CostMatrix,Path ***path_back, vector<vector<vector<bool>>> &isScaled,MatrixXd joints, VectorXd time, VectorXd Energy);
double Calculatehm(double ***CostMatrix, Path ***path_back, VectorXd &hm, vector<vector<vector<bool>>> &isScaled);
int* Init_NU();
double* Init_dyn_factor();
double *Init_DELTA();
int *Init_SEGMENTS();

double spline(double x, const VectorXd& xi, const VectorXd& yi, const VectorXd& y2i);
void spline(VectorXd x, VectorXd y, int n, double yp1, double ypn, VectorXd &y2);
MatrixXd &getParameterB();

#endif // TRAJECTORY_H
