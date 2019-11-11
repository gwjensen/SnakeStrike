//
// Image position correction for multiple view triangualtion
//

#ifndef _MULTIPLE_VIEW_TRIANGULATION_H_
#define _MULTIPLE_VIEW_TRIANGULATION_H_

#include <Eigen/Dense>

using namespace Eigen;

typedef Matrix<double,3,4> Matrix34d;
typedef Matrix<double,3,9> Matrix39d;
typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,3> Matrix63d;
typedef Matrix<double,6,6> Matrix6d;
typedef Matrix<double,9,1> Vector9d;
typedef Matrix<double,9,9> Matrix9d;
typedef DiagonalMatrix<double,9> DiagMatrix9d;

// Namespace MultipleViewTriangulation
namespace MultipleViewTriangulation {
   
   // constants for defualt values
   const double Default_f0 = 1000.0;
   const int Max_Iteration = 100;
   const double Convergence_EPS = 1e-12;
   const double Large_Number = 1e99;

   // constant Vectors and Matrices
   const Vector2d ZeroVec2 = Vector2d::Zero();
   const Vector3d ZeroVec3 = Vector3d::Zero();
   const Vector6d ZeroVec6 = Vector6d::Zero();
   const Matrix6d ZeroMat6 = Matrix6d::Zero();

   // common functions
   void make_trifocal_tensor(const Matrix34d P[], Matrix3d *tfT);
   
   double calc_Txyz(const Matrix3d tfT[],
                    const Vector3d& x,
                    const Vector3d& y,
                    const Vector3d& z,
                    int p, int q);
   
   // generalized inverse with rank 3 using SVD
   Matrix9d generalized_inverse_rank3(const Matrix9d& C);

   // generalized inverse with rank N using SVD
   MatrixXd generalized_inverse_rank_N(const MatrixXd& M, int rank);

   // Correct Points
   bool correct_triplet_tensor(const Matrix3d tfT[],
                               const Vector3d& x0i,
                               const Vector3d& x1i,
                               const Vector3d& x2i,
                               Vector3d& x0c,
                               Vector3d& x1c,
                               Vector3d& x2c,
                               double *reproj,
                               double Max_Iter = Max_Iteration,
                               double Conv_EPS = Convergence_EPS);
   // Correct Points
   bool correct_triplet_proj(const Matrix34d Proj[3],
                             const Vector3d& x0i,
                             const Vector3d& x1i,
                             const Vector3d& x2i,
                             Vector3d& x0c,
                             Vector3d& x1c,
                             Vector3d& x2c,
                             double *reproj,
                             int Max_Iter = Max_Iteration,
                             double Conv_EPS = Convergence_EPS);

   // Correct Points
   bool correct_triplets(const Matrix34d *Proj,
                         Vector2d x0i[],
                         Vector2d x1i[],
                         Vector2d x2i[],
                         Vector2d x0c[],
                         Vector2d x1c[],
                         Vector2d x2c[],
                         int Num,
                         double reproj[],
                         int Mac_Iter = Max_Iteration,
                         double Conv_EPS = Convergence_EPS,
                         double f0 = Default_f0);

   // Optimal correction from N cameras
   bool optimal_correction(const Matrix3d tfT[][3],
                           int CNum,
                           Vector3d xk[],
                           Vector3d xc[],
                           double *reperr,
                           int Max_Iter = Max_Iteration,
                           double Conv_EPS = Convergence_EPS);

   //My attempt at modifying the OC alg to have a little memory.
   bool optimal_correction_Grady(  const Matrix3d tfT[][3],
                                   int CNum,
                                   Vector3d xk[],
                                   Vector3d xkal[],
                                   Vector3d xc[],
                                   double *reperr,
                                   int Max_Iter,
                                   double Conv_EPS );

   // correct all points for every camera.
   bool optimal_correction_all(Matrix34d Proj[],
                               int CamNumAll,
                               MatrixXd Xk0[],
                               MatrixXd Xkc0[],
                               MatrixXi& idx,
                               double reperr[],
                               int PtNum,
                               int Max_Iter = Max_Iteration,
                               double Conv_EPS = Convergence_EPS,
                               double f0 = Default_f0);

   bool optimal_correction_all( Matrix34d Proj[],
                               int CamNumAll,
                               MatrixXd Xk0[],
                               MatrixXd XkLastPoint0[],
                               MatrixXd Xkc0[],
                               MatrixXi& idx,
                               double reperr[],
                               int PtNum,
                               int Max_Iter = Max_Iteration,
                               double Conv_EPS = Convergence_EPS,
                               double f0 = Default_f0 );
   
   // triangulation from single triplet
   Vector3d triangulation(const Matrix34d& P0,
                          const Matrix34d& P1,
                          const Matrix34d& P2,
                          const Vector2d& pos0,
                          const Vector2d& pos1,
                          const Vector2d& pos2,
                          double f0 = Default_f0);

   // triangulation from triplets
   bool triangulation(const Matrix34d& P0,
                      const Matrix34d& P1,
                      const Matrix34d& P2,
                      Vector2d pos0[],
                      Vector2d pos1[],
                      Vector2d pos2[],
                      Vector3d r[],
                      int Num,
                      double f0 = Default_f0);

   // triangulation from multiple view
   Vector3d triangulation(const Matrix34d Prj[],
                          const Vector2d xy[],
                          int CamNum,
                          double f0 = Default_f0);

   // triangulation from all cameras and points
   bool triangulation_all(const Matrix34d Prj[],
                          int CamNum,
                          MatrixXd x[],
                          Vector3d Xr[],
                          int PtNum,
                          const MatrixXi& idx,
                          double f0 = Default_f0);
}
// end of the namespace MultipleViewTriangulation

#endif // _MULTIPLE_VIEW_TRIANGULATION_H_
