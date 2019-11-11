//
// Triangulation
//
#ifndef _Triangulation_H_
#define _Triangulation_H_

#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<double,9,9> Matrix9d;
typedef Matrix<double,9,1> Vector9d;
typedef Matrix<double,3,4> Matrix34d;
typedef Matrix<double,4,3> Matrix43d;
typedef Matrix<double,2,3> Matrix23d;

// unnamed namespace for small function
namespace {

}

// Namespace Triangulation
namespace Triangulation {

// constants for defualt values
const double Default_f0 = 600.0;
const int Max_Iteration = 30;
const double Convergence_EPS = 1e-5;
const double Large_Number = 1e99;

// constant Vectors and Matrices
const Vector2d ZeroVec2 = Vector2d::Zero();
const Vector3d ZeroVec3 = Vector3d::Zero();
const Matrix9d ZeroMat9 = Matrix9d::Zero();

// common functions

// Triangulation using least squares from camera matices
Vector3d least_squares(const Vector2d& pos0,
                       const Vector2d& pos1,
                       const Matrix34d &P0,
                       const Matrix34d &P1,
                       double f0 = Default_f0);

// Triangulation using least squares from camera matices
bool least_squares(Vector2d pos0[],
                   Vector2d pos1[],
                   int Num,
                   const Matrix34d &P0,
                   const Matrix34d &P1,
                   Vector3d X[],
                   double f0 = Default_f0);

// Optimal correction for triangulation
double optimal_correction(const Vector2d& p0,
                          const Vector2d& p1,
                          const Vector9d &theta,
                          Vector2d& np0,
                          Vector2d& np1,
                          int Max_Iter = Max_Iteration,
                          double Conv_EPS = Convergence_EPS,
                          double f0 = Default_f0);

// Optimal correction for triangulation
bool optimal_correction(Vector2d p0[],
                        Vector2d p1[],
                        int      Num,
                        const Vector9d &theta,
                        Vector2d np0[],
                        Vector2d np1[],
                        int Max_Iter = Max_Iteration,
                        double Conv_EPS = Convergence_EPS,
                        double f0 = Default_f0);
}

#endif // _Triangulation_H_
