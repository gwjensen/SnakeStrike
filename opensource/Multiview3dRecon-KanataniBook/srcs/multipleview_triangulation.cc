//
// Triangulation from multiple view
//

#include <iostream>
#include "MultipleViewTriangulation.h"

using namespace Eigen;

// triangulation from multiple view
Vector3d
MultipleViewTriangulation::triangulation(
    const Matrix34d Prj[],
    const Vector2d xy[],
    int CamNum,
    double f0)
{
    MatrixXd T(2*CamNum,3);
    VectorXd p(2*CamNum);
    Matrix3d M;
    Vector3d b;
    Vector3d r;

    for (int cm = 0; cm < CamNum; cm++)
    {
        T(2*cm  ,0) = f0 * Prj[cm](0,0) - xy[cm](0) * Prj[cm](2,0);
        T(2*cm+1,0) = f0 * Prj[cm](1,0) - xy[cm](1) * Prj[cm](2,0);

        T(2*cm  ,1) = f0 * Prj[cm](0,1) - xy[cm](0) * Prj[cm](2,1);
        T(2*cm+1,1) = f0 * Prj[cm](1,1) - xy[cm](1) * Prj[cm](2,1);

        T(2*cm  ,2) = f0 * Prj[cm](0,2) - xy[cm](0) * Prj[cm](2,2);
        T(2*cm+1,2) = f0 * Prj[cm](1,2) - xy[cm](1) * Prj[cm](2,2);

        p(2*cm)   = f0 * Prj[cm](0,3) - xy[cm](0) * Prj[cm](2,3);
        p(2*cm+1) = f0 * Prj[cm](1,3) - xy[cm](1) * Prj[cm](2,3);
    }

    M = T.transpose() * T;
    b = - T.transpose() * p;

    // solve using LU decomp.
    FullPivLU<Matrix3d> LU(M);
    r = LU.solve(b);

    return r;
}

// triangulation from multiple view
bool
MultipleViewTriangulation::triangulation_all(
    const Matrix34d Prj[],
    int CamNumAll,
    MatrixXd x[],
    Vector3d Xr[],
    int PtNum,
    const MatrixXi& idx,
    double f0)
{
    int CamNum;
    FullPivLU<Matrix3d> LU;
    Matrix3d M;
    Vector3d b;

    for (int pt = 0; pt < PtNum; pt++)
    {
        CamNum = 0;
        for (int cm = 0; cm < CamNumAll; cm++)
            if (idx(pt,cm))  CamNum++;

        MatrixXd T(2*CamNum,3);
        VectorXd p(2*CamNum);
        int cc;

        cc = 0;
        for (int cm = 0; cm < CamNumAll; cm++)
        {
            if (idx(pt,cm))
            {
                T(cc,0) = f0 * Prj[cm](0,0) - x[pt].col(cm)(0) * Prj[cm](2,0);
                T(cc,1) = f0 * Prj[cm](0,1) - x[pt].col(cm)(0) * Prj[cm](2,1);
                T(cc,2) = f0 * Prj[cm](0,2) - x[pt].col(cm)(0) * Prj[cm](2,2);
                p(cc++) = f0 * Prj[cm](0,3) - x[pt].col(cm)(0) * Prj[cm](2,3);

                T(cc,0) = f0 * Prj[cm](1,0) - x[pt].col(cm)(1) * Prj[cm](2,0);
                T(cc,1) = f0 * Prj[cm](1,1) - x[pt].col(cm)(1) * Prj[cm](2,1);
                T(cc,2) = f0 * Prj[cm](1,2) - x[pt].col(cm)(1) * Prj[cm](2,2);
                p(cc++) = f0 * Prj[cm](1,3) - x[pt].col(cm)(1) * Prj[cm](2,3);
            }
        }

        M = T.transpose() * T;
        b = - T.transpose() * p;

        // solve using LU decomp.
        LU.compute(M);
        Xr[pt] = LU.solve(b);
    }

    return true;
}
