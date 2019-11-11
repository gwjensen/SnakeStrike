//
// Common functions for multiple view triangulation
//

#include <iostream>

#include "MultipleViewTriangulation.h"

using namespace Eigen;

namespace {
//Commented out because it isn't used
//double eddington(int i, int j, int k)
//{
//    double ret;

//    if (i == 0)
//    {
//        if (j == 1 && k == 2)
//            ret = 1.0;
//        else if (j == 2 && k == 1)
//            ret = -1.0;
//        else
//            ret = 0.0;
//    }
//    else if (i == 1)
//    {
//        if (j == 2 && k == 0)
//            ret = 1.0;
//        else if (j == 0 && k == 2)
//            ret = -1.0;
//        else
//            ret = 0.0;
//    }
//    else // i == 2
//    {
//        if (j == 0 && k == 1)
//            ret = 1.0;
//        else if (j == 1 && k == 0)
//            ret = -1.0;
//        else
//            ret = 0.0;
//    }

//    return ret;
//}
}

void
MultipleViewTriangulation::make_trifocal_tensor(
    const Matrix34d *P,
    Matrix3d *tfT)
{
    Matrix4d t1, t2, t3;

    // const. elements of T1(j,k), T2(j,k), and T3(j,k)
    t1.row(0) = t3.row(1) = P[0].row(1);
    t1.row(1) = t2.row(0) = P[0].row(2);
    t2.row(1) = t3.row(0) = P[0].row(0);

    for (int j = 0; j < 3; j++)
    {
        t1.row(2) = t2.row(2) = t3.row(2) = P[1].row(j);

        for (int k = 0; k < 3; k++)
        {
            t1.row(3) = t2.row(3) = t3.row(3) = P[2].row(k);

            tfT[0](j,k) = t1.determinant();
            tfT[1](j,k) = t2.determinant();
            tfT[2](j,k) = t3.determinant();
        }
    }
}

double
MultipleViewTriangulation::calc_Txyz(
    const Matrix3d tfT[],
    const Vector3d& x, const Vector3d& y, const Vector3d& z,
    int p, int q)
{
    double tmp;
    int p1, p2, q1, q2;

    p1 = (p + 1) % 3;
    p2 = (p + 2) % 3;
    q1 = (q + 1) % 3;
    q2 = (q + 2) % 3;

    tmp = 0.0;
    for (int i = 0; i < 3; i++)
    {
        tmp += x[i] *
               (tfT[i](p1,q1) * y(p2) * z(q2) - tfT[i](p2,q1) * y(p1) * z(q2)
                - tfT[i](p1,q2) * y(p2) * z(q1) + tfT[i](p2,q2) * y(p1) * z(q1));
    }

    return tmp;
}

// generalized inverse with rank 3 using SVD
Matrix9d
MultipleViewTriangulation::generalized_inverse_rank3(
    const Matrix9d& M
)
{
    JacobiSVD<Matrix9d> SVD(M, ComputeFullU | ComputeFullV);
    Vector9d sg3;

    sg3 <<
        1.0/SVD.singularValues()(0),
            1.0/SVD.singularValues()(1),
            1.0/SVD.singularValues()(2),
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0;

    return SVD.matrixV() * sg3.asDiagonal() * SVD.matrixU().transpose();
}

// generalized inverse with rank N using SVD
MatrixXd
MultipleViewTriangulation::generalized_inverse_rank_N(
    const MatrixXd& M, int rank
)
{
    JacobiSVD<MatrixXd> SVD(M, ComputeFullU | ComputeFullV);

    int Size = M.cols();
    VectorXd sg = VectorXd::Zero(Size);
    for (int r = 0; r < rank; r++)
        sg(r) = 1.0/SVD.singularValues()(r);

    return SVD.matrixV() * sg.asDiagonal() * SVD.matrixU().transpose();
}
