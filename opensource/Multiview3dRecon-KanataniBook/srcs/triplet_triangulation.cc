//
// Triangulation from multiple view
//

#include "MultipleViewTriangulation.h"

// triangulation from single triplet
Vector3d
MultipleViewTriangulation::triangulation(
    const Matrix34d& P0,
    const Matrix34d& P1,
    const Matrix34d& P2,
    const Vector2d& pos0,
    const Vector2d& pos1,
    const Vector2d& pos2,
    double f0)
{
    Matrix63d T;
    Vector6d p;
    Vector3d r;
    Matrix3d M;
    Vector3d b;
    double x0, y0, x1, y1, x2, y2;

    x0 = pos0(0);
    y0 = pos0(1);
    x1 = pos1(0);
    y1 = pos1(1);
    x2 = pos2(0);
    y2 = pos2(1);

    T(0,0) = f0 * P0(0,0) - x0 * P0(2,0);
    T(0,1) = f0 * P0(0,1) - x0 * P0(2,1);
    T(0,2) = f0 * P0(0,2) - x0 * P0(2,2);

    T(1,0) = f0 * P0(1,0) - y0 * P0(2,0);
    T(1,1) = f0 * P0(1,1) - y0 * P0(2,1);
    T(1,2) = f0 * P0(1,2) - y0 * P0(2,2);

    T(2,0) = f0 * P1(0,0) - x1 * P1(2,0);
    T(2,1) = f0 * P1(0,1) - x1 * P1(2,1);
    T(2,2) = f0 * P1(0,2) - x1 * P1(2,2);

    T(3,0) = f0 * P1(1,0) - y1 * P1(2,0);
    T(3,1) = f0 * P1(1,1) - y1 * P1(2,1);
    T(3,2) = f0 * P1(1,2) - y1 * P1(2,2);

    T(4,0) = f0 * P2(0,0) - x2 * P2(2,0);
    T(4,1) = f0 * P2(0,1) - x2 * P2(2,1);
    T(4,2) = f0 * P2(0,2) - x2 * P2(2,2);

    T(5,0) = f0 * P2(1,0) - y2 * P2(2,0);
    T(5,1) = f0 * P2(1,1) - y2 * P2(2,1);
    T(5,2) = f0 * P2(1,2) - y2 * P2(2,2);

    p <<
      f0 * P0(0,3) - x0 * P0(2,3),
      f0 * P0(1,3) - y0 * P0(2,3),
      f0 * P1(0,3) - x1 * P1(2,3),
      f0 * P1(1,3) - y1 * P1(2,3),
      f0 * P2(0,3) - x2 * P2(2,3),
      f0 * P2(1,3) - y2 * P2(2,3);

    M = T.transpose() * T;
    b = - T.transpose() * p;

    // solve using LU decomp.
    FullPivLU<Matrix3d> LU(M);
    r = LU.solve(b);

    return r;
}

// triangulation from triplets
bool
MultipleViewTriangulation::triangulation(
    const Matrix34d& P0,
    const Matrix34d& P1,
    const Matrix34d& P2,
    Vector2d pos0[],
    Vector2d pos1[],
    Vector2d pos2[],
    Vector3d r[],
    int Num,
    double f0)
{
    for (int i = 0; i < Num; i++)
        r[i] = triangulation(P0, P1, P2, pos0[i], pos1[i], pos2[i], f0);

    return true;
}
