//
// Correct three points for multiple view triangulation
//

#include "MultipleViewTriangulation.h"

// Correct Points
bool
MultipleViewTriangulation::correct_triplet_tensor(
    const Matrix3d tfT[3],
    const Vector3d& x0i,
    const Vector3d& x1i,
    const Vector3d& x2i,
    Vector3d& x0c,
    Vector3d& x1c,
    Vector3d& x2c,
    double *reproj,
    double Max_Iter,
    double Conv_EPS
)
{
    Vector3d x0, x1, x2, xh0, xh1, xh2, xt0, xt1, xt2;
    double E, E0;
    Matrix39d P, Q, R;
    Vector3d Pk[3], pt, qt, rt;
    Matrix9d C, C3i;
    Vector9d F;
    Vector9d lmd;
    int count;
    int pq, rs;

    // Set Pk
    Pk[0] << 1.0, 0.0, 0.0;
    Pk[1] << 0.0, 1.0, 0.0;
    Pk[2] << 0.0, 0.0, 0.0;

    // initialize
    xh0 = x0i;
    xh1 = x1i;
    xh2 = x2i;
    xt0 = xt1 = xt2 = ZeroVec3;

    E0 = Large_Number;
    count = 0;

    do
    {
        // Calc. Ppqs, Qpqs, and Rpqs
        for (int s = 0; s < 3; s++)
            for (int p = 0; p < 3; p++)
                for (int q = 0; q < 3; q++)
                {
                    pq = 3*p + q;
                    P(s,pq) = calc_Txyz(tfT, Pk[s], xh1, xh2, p, q);
                    Q(s,pq) = calc_Txyz(tfT, xh0, Pk[s], xh2, p, q);
                    R(s,pq) = calc_Txyz(tfT, xh0, xh1, Pk[s], p, q);
                }


        // Calc. Cpqrs and Fpq
        for (int p = 0; p < 3; p++)
            for (int q = 0; q < 3; q++)
            {
                pq = 3*p + q;
                for (int r = 0; r < 3; r++)
                    for (int s = 0; s < 3; s++)
                    {
                        rs = 3*r + s;
                        C(pq,rs) =
                            calc_Txyz(tfT, P.col(rs), xh1, xh2, p, q)
                            + calc_Txyz(tfT, xh0, Q.col(rs), xh2, p, q)
                            + calc_Txyz(tfT, xh0, xh1, R.col(rs), p, q);
                    }
                F(pq) =
                    calc_Txyz(tfT, xh0, xh1, xh2, p, q)
                    + calc_Txyz(tfT, xt0, xh1, xh2, p, q)
                    + calc_Txyz(tfT, xh0, xt1, xh2, p, q)
                    + calc_Txyz(tfT, xh0, xh1, xt2, p, q);
            }

        // generalized inverse with rank 3
        C3i = generalized_inverse_rank3(C);

        // solve linear equations
        lmd = C3i * F;

        // update xt and xt
        for (int i = 0; i < 3; i++)
        {
            xt0(i) = xt1(i) = xt2(i) = 0.0;
            for (int p = 0; p < 3; p++)
                for (int q = 0; q < 3; q++)
                {
                    pq = 3*p + q;
                    xt0(i) += P(i,pq) * lmd(pq);
                    xt1(i) += Q(i,pq) * lmd(pq);
                    xt2(i) += R(i,pq) * lmd(pq);
                }
        }
        xh0 = x0 - xt0;
        xh1 = x1 - xt1;
        xh2 = x2 - xt2;

        // Compute reprojection error
        E = xt0.squaredNorm() + xt1.squaredNorm() + xt2.squaredNorm();

        if (fabs (E - E0) < Conv_EPS)
        {
            *reproj= E;
            x0c = xh0;
            x1c = xh1;
            x2c = xh2;
            break;
        }
        E0 = E;
    }
    while (++count < Max_Iter);

    if (count == Max_Iter)
        return false;

    return true;
}

// Correct Points
bool
MultipleViewTriangulation::correct_triplet_proj(
    const Matrix34d Proj[3],
    const Vector3d& x0i,
    const Vector3d& x1i,
    const Vector3d& x2i,
    Vector3d& x0c,
    Vector3d& x1c,
    Vector3d& x2c,
    double *reproj,
    int Max_Iter,
    double Conv_EPS
)
{
    Matrix3d tfT[3];
    bool flag;

    make_trifocal_tensor(Proj, tfT);
    flag = correct_triplet_tensor(tfT, x0i, x1i, x2i, x0c, x1c, x2c,
                                  reproj, Max_Iter, Conv_EPS);

    return flag;
}


// Correct Points
bool
MultipleViewTriangulation::correct_triplets(
    const Matrix34d Proj[],
    Vector2d x0i2[],
    Vector2d x1i2[],
    Vector2d x2i2[],
    Vector2d x0c2[],
    Vector2d x1c2[],
    Vector2d x2c2[],
    int Num,
    double *reproj,
    int Max_Iter,
    double Conv_EPS,
    double f0
)
{
    Matrix3d tfT[3];
    Vector3d x0i, x1i, x2i, x0c, x1c, x2c;
    double rpj;
    bool flag;

    // Set 3-vectors

    // Set Trifocal Tensor
    make_trifocal_tensor(Proj, tfT);

    for (int i = 0; i < Num; i++)
    {
        x0i << x0i2[i](0)/f0, x0i2[i](1)/f0, 1.0;
        x1i << x1i2[i](0)/f0, x1i2[i](1)/f0, 1.0;
        x2i << x2i2[i](0)/f0, x2i2[i](1)/f0, 1.0;

        flag = correct_triplet_tensor(tfT, x0i, x1i, x2i,
                                      x0c, x1c, x2c,
                                      &rpj, Max_Iter, Conv_EPS);
        if (flag)
        {
            x0c2[i] << x0c(0) * f0, x0c(1) * f0;
            x1c2[i] << x1c(0) * f0, x1c(1) * f0;
            x2c2[i] << x2c(0) * f0, x2c(1) * f0;
            reproj[i] = rpj;
        }
        else
            reproj[i] = Large_Number;
    }

    return true;
}
