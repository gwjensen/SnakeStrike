//
// Optimal correction for multiple view triangulation
//

#include <iostream>
#include <cstdio>

#include <math.h> /*sqrt, pow*/

#include "Triangulation.h"
#include "MultipleViewTriangulation.h"

// small function
namespace {

double sqr(double x) {
    return x*x;
}
// convert two projection matices to F matrix
Vector9d make_Fmat(const Matrix34d& P0, const Matrix34d& P1)
{
    Vector9d theta;
    Matrix4d T;

    // F(0,0)
    T.row(0) = P0.row(1);
    T.row(1) = P0.row(2);
    T.row(2) = P1.row(1);
    T.row(3) = P1.row(2);
    theta(0) = T.determinant();

    // F(0,1): 1st and 2nd rows are equivalent to F(0,0)
    T.row(2) = P1.row(2);
    T.row(2) = P1.row(0);
    theta(1) = T.determinant();

    // F(0,2): 1st and 2nd rows are equivalent to F(0,0)
    T.row(2) = P1.row(0);
    T.row(2) = P1.row(1);
    theta(2) = T.determinant();

    // F(1,0)
    T.row(0) = P0.row(2);
    T.row(1) = P0.row(0);
    T.row(2) = P1.row(1);
    T.row(3) = P1.row(2);
    theta(3) = T.determinant();

    // F(1,1)
    T.row(0) = P0.row(0);
    T.row(1) = P0.row(2);
    T.row(2) = P1.row(0);
    T.row(3) = P1.row(2);
    theta(4) = T.determinant();

    // F(1,2)
    T.row(0) = P0.row(2);
    T.row(1) = P0.row(0);
    T.row(2) = P1.row(0);
    T.row(3) = P1.row(1);
    theta(5) = T.determinant();

    // F(2,0)
    T.row(0) = P0.row(0);
    T.row(1) = P0.row(1);
    T.row(2) = P1.row(1);
    T.row(3) = P1.row(2);
    theta(6) = T.determinant();

    // F(2,1)
    T.row(2) = P1.row(2);
    T.row(3) = P1.row(0);
    theta(7) = T.determinant();

    // F(2,2)
    T.row(2) = P1.row(0);
    T.row(3) = P1.row(1);
    theta(8) = T.determinant();

    return theta;
}
} // unnamed namespace

// Optimal correction
bool
MultipleViewTriangulation::optimal_correction(
    const Matrix3d tfT[][3],
    int CNum,
    Vector3d xk[],
    Vector3d xc[],
    double *reperr,
    int Max_Iter,
    double Conv_EPS
)
{
    Vector3d xkh[CNum], xkt[CNum];
    double EE, E0;
    Matrix39d P[CNum], Q[CNum], R[CNum];
    Vector3d Pk[3], pt, qt, rt;
    Matrix9d A[CNum], B[CNum], C[CNum], D[CNum], E[CNum];
    Vector9d F[CNum];
//   Vector9d lmd[CNum];

    // Matrix 9(M-2) * 9(M-2)
    int sz = 9*(CNum - 2);
    MatrixXd T;
    MatrixXd Ti(sz, sz);
    VectorXd ff(sz);
    VectorXd llmd(sz);

    int count;

    // Set Pk
    Pk[0] << 1.0, 0.0, 0.0;
    Pk[1] << 0.0, 1.0, 0.0;
    Pk[2] << 0.0, 0.0, 0.0;

    //std::fprintf( stderr,"Starting Point matrices before iterations.\n");
    for (int kp = 0; kp < CNum; kp++)
    {
        xkh[kp] = xk[kp];
        xkt[kp] = ZeroVec3;
        //std::cout << "xk[" << kp << "] :" << std::endl;
        //std::cout << xk[kp] << std::endl;
        //std::cout << "xkt[" << kp << "] :" << std::endl;
        //std::cout << xkt[kp] << std::endl;
        //std::cout << "xkh[" << kp << "] :" << std::endl;
        //std::cout << xkh[kp] << std::endl;
    }

	
    E0 = Large_Number;
    count = 0;

    do
    {
        int pq, rs;
        // set Pkpqs, Qkpqs, and Rkpqs
        // Pkpqs = P[kp](s,3*p+q)
        // Qkpqs = Q[kp](s,3*p+q)
        // Rkpqs = R[kp](s,3*p+q)
        for (int kp = 0; kp < CNum; kp++)
        {
            int kpm1, kpm2, kpp1, kpp2;
            kpm1 = kp - 1;
            kpm2 = kp - 2;
            kpp1 = kp + 1;
            kpp2 = kp + 2;

            for (int s = 0; s < 3; s++)
                for (int p = 0; p < 3; p++)
                    for (int q = 0; q < 3; q++)
                    {
                        pq = 3*p + q;
                        if (kp < CNum - 2)
                            P[kp](s,pq) = calc_Txyz(tfT[kp], Pk[s], xkh[kpp1], xkh[kpp2], p, q);
                        else
                            P[kp](s,pq) = 0.0;
                        if (kp >= 1 && kp < CNum -1)
                            Q[kp](s,pq) = calc_Txyz(tfT[kp-1], xkh[kpm1], Pk[s], xkh[kpp1], p, q);
                        else
                            Q[kp](s,pq) = 0.0;
                        if (kp >= 2)
                            R[kp](s,pq) = calc_Txyz(tfT[kp-2], xkh[kpm2], xkh[kpm1], Pk[s], p, q);
                        else
                            R[kp](s,pq) = 0.0;
                    }
        }

        // set Ak(pqrs), Bk(pqrs), Ck(pqrs), Dk(pqrs), Ek(pqrs), and Fk(pq)
        for (int kp = 0; kp < CNum - 2; kp++)
        {
            for (int p = 0; p < 3; p++)
            {
                for (int q = 0; q < 3; q++)
                {
                    pq = 3*p + q;
                    for (int r = 0; r < 3; r++)
                    {
                        for (int s = 0; s < 3; s++)
                        {
                            rs = 3*r + s;

                            if (kp >= 2)
                            {
                                A[kp](pq,rs) =
                                    calc_Txyz(tfT[kp], R[kp].col(rs), xkh[kp+1], xkh[kp+2], p, q);
                            }
                            if (kp >= 1)
                            {
                                B[kp](pq,rs) =
                                    calc_Txyz(tfT[kp], Q[kp].col(rs), xkh[kp+1], xkh[kp+2], p, q)
                                    + calc_Txyz(tfT[kp], xkh[kp], R[kp+1].col(rs), xkh[kp+2], p, q);
                            }
                            C[kp](pq,rs) =
                                calc_Txyz(tfT[kp], P[kp].col(rs), xkh[kp+1], xkh[kp+2], p, q)
                                + calc_Txyz(tfT[kp], xkh[kp], Q[kp+1].col(rs), xkh[kp+2], p, q)
                                + calc_Txyz(tfT[kp], xkh[kp], xkh[kp+1], R[kp+2].col(rs), p, q);
                            if (kp <= CNum - 4)
                            {
                                D[kp](pq,rs) =
                                    calc_Txyz(tfT[kp], xkh[kp], P[kp+1].col(rs), xkh[kp+2], p, q)
                                    + calc_Txyz(tfT[kp], xkh[kp], xkh[kp+1], Q[kp+2].col(rs), p, q);
                            }
                            if (kp <= CNum - 5)
                            {
                                E[kp](pq,rs) =
                                    calc_Txyz(tfT[kp], xkh[kp], xkh[kp+1], P[kp+2].col(rs), p, q);
                            }
                        }
                    }
                    F[kp](pq) =
                        calc_Txyz(tfT[kp], xkh[kp], xkh[kp+1], xkh[kp+2], p, q)
                        + calc_Txyz(tfT[kp], xkt[kp], xkh[kp+1], xkh[kp+2], p, q)
                        + calc_Txyz(tfT[kp], xkh[kp], xkt[kp+1], xkh[kp+2], p, q)
                        + calc_Txyz(tfT[kp], xkh[kp], xkh[kp+1], xkt[kp+2], p, q);
                }
            }
        }

        T = MatrixXd::Zero(sz, sz);
        for (int kp = 0; kp < CNum - 2; kp++)
        {
            T.block(9*kp,9*kp,9,9) = C[kp];
            if (kp <= CNum - 4)
            {
                T.block(9*kp, 9*(kp+1), 9, 9) = D[kp];
            }
            if (kp <= CNum - 5)
            {
                T.block(9*kp, 9*(kp+2), 9, 9) = E[kp];
            }
            if (kp >= 1)
            {
                T.block(9*kp, 9*(kp-1), 9, 9) = B[kp];
            }
            if (kp >= 2)
            {
                T.block(9*kp, 9*(kp-2), 9, 9) = A[kp];
            }
            ff.segment(9*kp,9) = F[kp];
        }

        // Generalized Inverse
        Ti = generalized_inverse_rank_N(T, 2*CNum - 3);

        // solve linear equations
        llmd = Ti * ff;

        // update xt and xt
        //std::cout << "Updating matrices ..." << std::endl;
        for (int kp = 0; kp < CNum; kp++)
        {
            xkt[kp] = ZeroVec3;
            for (int i = 0; i < 3; i++)
            {
				for (int p = 0; p < 3; p++)
				{
                    for (int q = 0; q < 3; q++)
                    {
                        double ttt1, ttt2, ttt3;

                        pq = 3*p + q;

                        if (9*kp + pq >= sz)
                            ttt1 = 0.0;
                        else
                            ttt1 = P[kp](i,pq) * llmd(9*kp+pq);

                        if (9*(kp-1) + pq >= sz || kp < 1)
                            ttt2 = 0.0;
                        else
                            ttt2 = Q[kp](i,pq) * llmd(9*(kp-1)+pq);

                        if (9*(kp-2) + pq >= sz || kp < 2)
                            ttt3 = 0.0;
                        else
                            ttt3 = R[kp](i,pq) * llmd(9*(kp-2)+pq);

                        xkt[kp](i) += ttt1 + ttt2 + ttt3;
                    }
				}
			}
            xkh[kp] = xk[kp] - xkt[kp];
            //std::cout << "xk[" << kp << "] :" << std::endl;
            //std::cout << xk[kp] << std::endl;
            //std::cout << "xkt[" << kp << "] :" << std::endl;
            //std::cout << xkt[kp] << std::endl;
            //std::cout << "xkh[" << kp << "] :" << std::endl;
            //std::cout << xkh[kp] << std::endl;

            //std::stringstream stream;
            //stream << "Error for cam " << kp << ", iteration " << count << " is " << (xkt[kp].norm() * Default_f0)  << "." << std::endl;
            //stream << "Grady::Error for cam " << kp << ", iteration " << count << " is " << (sqrt(pow(xkt[kp][0],2) + pow(xkt[kp][1],2) +  pow(xkt[kp][2],2) ) * Default_f0 ) << "." << std::endl;


            //stream << "xk[" << kp << "] :" << std::endl;
            //stream << xk[kp] << std::endl;
            //stream << "xkh[" << kp << "] :" << std::endl;
            //stream << xkh[kp] << std::endl;
            //stream << "xkt[" << kp << "] :" << std::endl;
            //stream << xkt[kp] << std::endl;
            //std::fprintf( stderr, "%s\n", stream.str().c_str());

        }

        // Compute reprojection error
        EE = 0.0;
        for (int kp = 0; kp < CNum; kp++){
            EE += (xkt[kp].norm() * Default_f0);
		}
        std::stringstream stream;
        //stream << "Total error for iteration " << count << " is " << EE << "." << std::endl;
        //std::fprintf( stderr, "%s\n", stream.str().c_str());
        if (fabs (EE - E0) < Conv_EPS)
        {
            *reperr = EE;
            for (int kp = 0; kp < CNum; kp++){
                xc[kp] = xkh[kp];
            }
            //std::stringstream stream;
            //stream << "\n\nThe OC required " << count << " steps to converge to <" << Conv_EPS << std::endl;
            //stream << "Total error for that sequence was " << EE << "." << std::endl;
            //std::fprintf(stderr, "%s\n", stream.str().c_str());
            return true;
        }
        E0 = EE;
    }
    while (++count < Max_Iter);
	
    return false;
}

// Optimal correction - Modified by Grady
bool
MultipleViewTriangulation::optimal_correction_Grady(
    const Matrix3d tfT[][3],
    int CNum,
    Vector3d xk[],
    Vector3d xkal[],
    Vector3d xc[],
    double *reperr,
    int Max_Iter,
    double Conv_EPS
)
{
    Vector3d xkh[CNum], xkt[CNum];
    double EE, E0;
    Matrix39d P[CNum], Q[CNum], R[CNum];
    Vector3d Pk[3], pt, qt, rt;
    Matrix9d A[CNum], B[CNum], C[CNum], D[CNum], E[CNum];
    Vector9d F[CNum];
//   Vector9d lmd[CNum];

    // Matrix 9(M-2) * 9(M-2)
    int sz = 9*(CNum - 2);
    MatrixXd T;
    MatrixXd Ti(sz, sz);
    VectorXd ff(sz);
    VectorXd llmd(sz);

    int count;

    // Set Pk
    Pk[0] << 1.0, 0.0, 0.0;
    Pk[1] << 0.0, 1.0, 0.0;
    Pk[2] << 0.0, 0.0, 0.0;

    //std::fprintf( stderr,"Starting Point matrices before iterations.\n");
    for (int kp = 0; kp < CNum; kp++)
    {
        xkh[kp] = xk[kp];
        xkt[kp] = ZeroVec3;
        //std::cout << "xk[" << kp << "] :" << std::endl;
        //std::cout << xk[kp] << std::endl;
        //std::cout << "xkt[" << kp << "] :" << std::endl;
        //std::cout << xkt[kp] << std::endl;
        //std::cout << "xkh[" << kp << "] :" << std::endl;
        //std::cout << xkh[kp] << std::endl;
    }


    E0 = Large_Number;
    count = 0;

    do
    {
        int pq, rs;
        // set Pkpqs, Qkpqs, and Rkpqs
        // Pkpqs = P[kp](s,3*p+q)
        // Qkpqs = Q[kp](s,3*p+q)
        // Rkpqs = R[kp](s,3*p+q)
        for (int kp = 0; kp < CNum; kp++)
        {
            int kpm1, kpm2, kpp1, kpp2;
            kpm1 = kp - 1;
            kpm2 = kp - 2;
            kpp1 = kp + 1;
            kpp2 = kp + 2;

            for (int s = 0; s < 3; s++)
                for (int p = 0; p < 3; p++)
                    for (int q = 0; q < 3; q++)
                    {
                        pq = 3*p + q;
                        if (kp < CNum - 2)
                            P[kp](s,pq) = calc_Txyz(tfT[kp], Pk[s], xkh[kpp1], xkh[kpp2], p, q);
                        else
                            P[kp](s,pq) = 0.0;
                        if (kp >= 1 && kp < CNum -1)
                            Q[kp](s,pq) = calc_Txyz(tfT[kp-1], xkh[kpm1], Pk[s], xkh[kpp1], p, q);
                        else
                            Q[kp](s,pq) = 0.0;
                        if (kp >= 2)
                            R[kp](s,pq) = calc_Txyz(tfT[kp-2], xkh[kpm2], xkh[kpm1], Pk[s], p, q);
                        else
                            R[kp](s,pq) = 0.0;
                    }
        }

        // set Ak(pqrs), Bk(pqrs), Ck(pqrs), Dk(pqrs), Ek(pqrs), and Fk(pq)
        for (int kp = 0; kp < CNum - 2; kp++)
        {
            for (int p = 0; p < 3; p++)
            {
                for (int q = 0; q < 3; q++)
                {
                    pq = 3*p + q;
                    for (int r = 0; r < 3; r++)
                    {
                        for (int s = 0; s < 3; s++)
                        {
                            rs = 3*r + s;

                            if (kp >= 2)
                            {
                                A[kp](pq,rs) =
                                    calc_Txyz(tfT[kp], R[kp].col(rs), xkh[kp+1], xkh[kp+2], p, q);
                            }
                            if (kp >= 1)
                            {
                                B[kp](pq,rs) =
                                    calc_Txyz(tfT[kp], Q[kp].col(rs), xkh[kp+1], xkh[kp+2], p, q)
                                    + calc_Txyz(tfT[kp], xkh[kp], R[kp+1].col(rs), xkh[kp+2], p, q);
                            }
                            C[kp](pq,rs) =
                                calc_Txyz(tfT[kp], P[kp].col(rs), xkh[kp+1], xkh[kp+2], p, q)
                                + calc_Txyz(tfT[kp], xkh[kp], Q[kp+1].col(rs), xkh[kp+2], p, q)
                                + calc_Txyz(tfT[kp], xkh[kp], xkh[kp+1], R[kp+2].col(rs), p, q);
                            if (kp <= CNum - 4)
                            {
                                D[kp](pq,rs) =
                                    calc_Txyz(tfT[kp], xkh[kp], P[kp+1].col(rs), xkh[kp+2], p, q)
                                    + calc_Txyz(tfT[kp], xkh[kp], xkh[kp+1], Q[kp+2].col(rs), p, q);
                            }
                            if (kp <= CNum - 5)
                            {
                                E[kp](pq,rs) =
                                    calc_Txyz(tfT[kp], xkh[kp], xkh[kp+1], P[kp+2].col(rs), p, q);
                            }
                        }
                    }
                    F[kp](pq) =
                        calc_Txyz(tfT[kp], xkh[kp], xkh[kp+1], xkh[kp+2], p, q)
                        + calc_Txyz(tfT[kp], xkt[kp], xkh[kp+1], xkh[kp+2], p, q)
                        + calc_Txyz(tfT[kp], xkh[kp], xkt[kp+1], xkh[kp+2], p, q)
                        + calc_Txyz(tfT[kp], xkh[kp], xkh[kp+1], xkt[kp+2], p, q);
                }
            }
        }

        T = MatrixXd::Zero(sz, sz);
        for (int kp = 0; kp < CNum - 2; kp++)
        {
            T.block(9*kp,9*kp,9,9) = C[kp];
            if (kp <= CNum - 4)
            {
                T.block(9*kp, 9*(kp+1), 9, 9) = D[kp];
            }
            if (kp <= CNum - 5)
            {
                T.block(9*kp, 9*(kp+2), 9, 9) = E[kp];
            }
            if (kp >= 1)
            {
                T.block(9*kp, 9*(kp-1), 9, 9) = B[kp];
            }
            if (kp >= 2)
            {
                T.block(9*kp, 9*(kp-2), 9, 9) = A[kp];
            }
            ff.segment(9*kp,9) = F[kp];
        }

        // Generalized Inverse
        Ti = generalized_inverse_rank_N(T, 2*CNum - 3);

        // solve linear equations
        llmd = Ti * ff;

        // update xt and xt
        //std::cout << "Updating matrices ..." << std::endl;
        for (int kp = 0; kp < CNum; kp++)
        {
            xkt[kp] = ZeroVec3;
            for (int i = 0; i < 3; i++)
            {
                for (int p = 0; p < 3; p++)
                {
                    for (int q = 0; q < 3; q++)
                    {
                        double ttt1, ttt2, ttt3;

                        pq = 3*p + q;

                        if (9*kp + pq >= sz)
                            ttt1 = 0.0;
                        else
                            ttt1 = P[kp](i,pq) * llmd(9*kp+pq);

                        if (9*(kp-1) + pq >= sz || kp < 1)
                            ttt2 = 0.0;
                        else
                            ttt2 = Q[kp](i,pq) * llmd(9*(kp-1)+pq);

                        if (9*(kp-2) + pq >= sz || kp < 2)
                            ttt3 = 0.0;
                        else
                            ttt3 = R[kp](i,pq) * llmd(9*(kp-2)+pq);

                        xkt[kp](i) += ttt1 + ttt2 + ttt3;
                    }
                }
            }
            xkh[kp] = xk[kp] - xkt[kp] - xkal[kp];
            //std::cout << "xk[" << kp << "] :" << std::endl;
            //std::cout << xk[kp] << std::endl;
            //std::cout << "xkt[" << kp << "] :" << std::endl;
            //std::cout << xkt[kp] << std::endl;
            //std::cout << "xkh[" << kp << "] :" << std::endl;
            //std::cout << xkh[kp] << std::endl;

            //std::stringstream stream;
            //stream << "Error for cam " << kp << ", iteration " << count << " is " << (xkt[kp].norm() * Default_f0)  << "." << std::endl;
            //stream << "Grady::Error for cam " << kp << ", iteration " << count << " is " << (sqrt(pow(xkt[kp][0],2) + pow(xkt[kp][1],2) +  pow(xkt[kp][2],2) ) * Default_f0 ) << "." << std::endl;


            //stream << "xk[" << kp << "] :" << std::endl;
            //stream << xk[kp] << std::endl;
            //stream << "xkh[" << kp << "] :" << std::endl;
            //stream << xkh[kp] << std::endl;
            //stream << "xkt[" << kp << "] :" << std::endl;
            //stream << xkt[kp] << std::endl;
            //std::fprintf( stderr, "%s\n", stream.str().c_str());

        }

        // Compute reprojection error
        EE = 0.0;
        for (int kp = 0; kp < CNum; kp++){
            EE += (xkt[kp].norm() * Default_f0);
        }
        std::stringstream stream;
        //stream << "Total error for iteration " << count << " is " << EE << "." << std::endl;
        //std::fprintf( stderr, "%s\n", stream.str().c_str());
        if (fabs (EE - E0) < Conv_EPS)
        {
            *reperr = EE;
            for (int kp = 0; kp < CNum; kp++){
                xc[kp] = xkh[kp];
            }
            //std::stringstream stream;
            //stream << "\n\nThe OC required " << count << " steps to converge to <" << Conv_EPS << std::endl;
            //stream << "Total error for that sequence was " << EE << "." << std::endl;
            //std::fprintf(stderr, "%s\n", stream.str().c_str());
            return true;
        }
        E0 = EE;
    }
    while (++count < Max_Iter);

    return false;
}

// correct all points for every camera.
bool
MultipleViewTriangulation::optimal_correction_all(
    Matrix34d Proj[],
    int CamNumAll,
    MatrixXd Xk0[],
    MatrixXd Xkc0[],
    MatrixXi& idx,
    double reperr[],
    int PtNum,
    int Max_Iter,
    double Conv_EPS,
    double f0
)
{
    int CamNum;
    int cc;
    bool flag;

    // size check
    if (Xk0[0].rows() != 2)
        return false;

    // correction loop for each point
    for (int pnum = 0; pnum < PtNum; pnum++)
    {
        // Count cameras
        CamNum = 0;
        for (int cnum = 0; cnum < CamNumAll; cnum++)
        {
            idx(pnum,cnum) = false;
            if (Xk0[pnum].col(cnum)(0) < 0.0) continue; // x < 0: a point cannot be observe
            idx(pnum,cnum) = true;
            CamNum++;
        }

        // three or more cameras
        if (CamNum >= 3)
        {
            Matrix34d Prj[CamNum];
            Matrix3d tfT[CamNum-2][3];
            Vector3d xk[CamNum], xkc[CamNum];

            cc = 0;
            for (int cnum = 0; cnum < CamNumAll; cnum++)
            {
                if (idx(pnum,cnum))
                {
                    xk[cc] << Xk0[pnum].col(cnum)/f0, 1.0;
                    Prj[cc] = Proj[cnum];
                    //std::stringstream stream;

                    //stream << "Prj[" << cc << "] :" << std::endl;
                    //stream << Prj[cc] << std::endl;
                    //stream << "Proj[" << cnum << "] :" << std::endl;
                   //stream << Proj[cnum] << std::endl;
                   //stream << "xk[" << cc << "] :" << std::endl;
                   // stream << xk[cc] << std::endl;
                   // stream << "Xk0[" << pnum << "] :" << std::endl;
                   // stream << Xk0[pnum].col(cnum) << std::endl;
                    //stream << "xkt[" << kp << "] :" << std::endl;
                    //stream << xkt[kp] << std::endl;
                   // std::fprintf( stderr, "%s\n", stream.str().c_str());
                    cc++;
                }
            }

            // compute trifocal tensors
            for (int cnum = 0; cnum < CamNum - 2; cnum++)
                make_trifocal_tensor(&(Prj[cnum]), &(tfT[cnum][0]));

            // correction
            flag = optimal_correction(tfT, CamNum, xk, xkc, reperr,
                                      Max_Iter, Conv_EPS);

            if (flag) // success
            {
                //std::fprintf( stderr, "\n\nIndividual call to optimal_correction was successful.\n\n" );
                cc = 0;
                for (int cnum = 0; cnum < CamNumAll; cnum++)
                {
                    if (idx(pnum,cnum))
                    {
                        Xkc0[pnum].col(cnum) << f0 * xkc[cc](0), f0 * xkc[cc](1);
                        cc++;
                    }
                    else
                        Xkc0[pnum].col(cnum) = ZeroVec2;
                }
            }
            else // failed
            {
				std::fprintf( stderr, "\n\nIndividual call to optimal_correction FAILED.\n\n" );
                for (int cnum = 0; cnum < CamNum; cnum++)
                    Xkc0[pnum].col(cnum) = ZeroVec2;
            }
        }
        else if (CamNum == 2)
        {
            Matrix34d Prj[CamNum];
            Vector2d pos[2], np[2];
            Vector9d theta;

            cc = 0;
            for (int cnum = 0; cnum < CamNumAll; cnum++)
            {
                if (idx(pnum,cnum))
                {
                    Prj[cc] = Proj[cnum];               // copy projection matrix
                    pos[cc] = Xk0[pnum].col(cnum);      // copy data
                    //std::stringstream stream;

                    //stream << "Prj[" << cc << "] :" << std::endl;
                    //stream << Prj[cc] << std::endl;
                    //stream << "Proj[" << cnum << "] :" << std::endl;
                    //stream << Proj[cnum] << std::endl;
                    //stream << "pos[" << cc << "] :" << std::endl;
                    //stream << pos[cc] << std::endl;
                    //stream << "Xk0[" << pnum << "] :" << std::endl;
                    //stream << Xk0[pnum] << std::endl;
                    //stream << "xkt[" << kp << "] :" << std::endl;
                    //stream << xkt[kp] << std::endl;
                    //std::fprintf( stderr, "%s\n", stream.str().c_str());

                    cc++;
                }
            }




            // Fundamental matrix
            theta = make_Fmat(Prj[0], Prj[1]);

            // correction using stereo triangulation
            reperr[pnum] = Triangulation::optimal_correction(pos[0], pos[1], theta,
                           np[0], np[1],
                           Max_Iter, Conv_EPS, f0);
            // copy correcetd data
            cc = 0;
            for (int cnum = 0; cnum < CamNumAll; cnum++)
            {
                if (idx(pnum,cnum))
                {
                    Xkc0[pnum].col(cnum) = np[cc];
                    cc++;
                }
                else
                {
                    Xkc0[pnum].col(cnum) = ZeroVec2;
                }
            }
        }
    }

    return true;
}

// correct all points for every camera.
bool
MultipleViewTriangulation::optimal_correction_all(
    Matrix34d Proj[],
    int CamNumAll,
    MatrixXd Xk0[],
    MatrixXd XkLastPoint0[],
    MatrixXd Xkc0[],
    MatrixXi& idx,
    double reperr[],
    int PtNum,
    int Max_Iter,
    double Conv_EPS,
    double f0
)
{
    int CamNum;
    int cc;
    bool flag;

    // size check
    if (Xk0[0].rows() != 2 || XkLastPoint0[0].rows() != 2)
        return false;

    // correction loop for each point
    for (int pnum = 0; pnum < PtNum; pnum++)
    {
        // Count cameras
        CamNum = 0;
        for (int cnum = 0; cnum < CamNumAll; cnum++)
        {
            idx(pnum,cnum) = false;
            if (Xk0[pnum].col(cnum)(0) < 0.0) continue; // x < 0: a point cannot be observe
            idx(pnum,cnum) = true;
            CamNum++;
        }

        // three or more cameras
        if (CamNum >= 3)
        {
            Matrix34d Prj[CamNum];
            Matrix3d tfT[CamNum-2][3];
            Vector3d xk[CamNum], xkLastPoint[CamNum], xkc[CamNum];

            cc = 0;
            for (int cnum = 0; cnum < CamNumAll; cnum++)
            {
                if (idx(pnum,cnum))
                {
                    xk[cc] << Xk0[pnum].col(cnum)/f0, 1.0;
                    xkLastPoint[cc] << XkLastPoint0[pnum].col(cnum)/f0, 1.0;
                    Prj[cc] = Proj[cnum];
                    cc++;
                }
            }

            // compute trifocal tensors
            for (int cnum = 0; cnum < CamNum - 2; cnum++)
                make_trifocal_tensor(&(Prj[cnum]), &(tfT[cnum][0]));

            // correction
            flag = optimal_correction_Grady(tfT, CamNum, xk, xkLastPoint, xkc, reperr,
                                      Max_Iter, Conv_EPS);

            if (flag) // success
            {
                //std::fprintf( stderr, "\n\nIndividual call to optimal_correction was successful.\n\n" );
                cc = 0;
                for (int cnum = 0; cnum < CamNumAll; cnum++)
                {
                    if (idx(pnum,cnum))
                    {
                        Xkc0[pnum].col(cnum) << f0 * xkc[cc](0), f0 * xkc[cc](1);
                        cc++;
                    }
                    else
                        Xkc0[pnum].col(cnum) = ZeroVec2;
                }
            }
            else // failed
            {
                std::fprintf( stderr, "\n\nIndividual call to optimal_correction FAILED.\n\n" );
                for (int cnum = 0; cnum < CamNum; cnum++)
                    Xkc0[pnum].col(cnum) = ZeroVec2;
            }
        }
        else if (CamNum == 2)
        {
            Matrix34d Prj[CamNum];
            Vector2d pos[2], np[2];
            Vector9d theta;

            cc = 0;
            for (int cnum = 0; cnum < CamNumAll; cnum++)
            {
                if (idx(pnum,cnum))
                {
                    Prj[cc] = Proj[cnum];               // copy projection matrix
                    pos[cc] = Xk0[pnum].col(cnum);      // copy data
                    cc++;
                }
            }

            // Fundamental matrix
            theta = make_Fmat(Prj[0], Prj[1]);

            // correction using stereo triangulation
            reperr[pnum] = Triangulation::optimal_correction(pos[0], pos[1], theta,
                           np[0], np[1],
                           Max_Iter, Conv_EPS, f0);
            // copy correcetd data
            cc = 0;
            for (int cnum = 0; cnum < CamNumAll; cnum++)
            {
                if (idx(pnum,cnum))
                {
                    Xkc0[pnum].col(cnum) = np[cc];
                    cc++;
                }
                else
                {
                    Xkc0[pnum].col(cnum) = ZeroVec2;
                }
            }
        }
    }

    return true;
}

namespace {

// Convert four 2-vectors into 9-vector
Vector9d
convert_vec9_st(const Vector2d &p0h, const Vector2d &p1h,
                const Vector2d &p0t, const Vector2d &p1t,
                double f0)
{
    Vector9d t;

    t <<
      p0h(0)*p1h(0) + p1h(0)*p0t(0) + p0h(0)*p1t(0),
          p0h(0)*p1h(1) + p1h(1)*p0t(0) + p0h(0)*p1t(1),
//      p0h(0)*p1h(0) + p1h(0)*p0t(0) + p1h(0)*p1t(0),
//      p0h(0)*p1h(1) + p1h(1)*p0t(0) + p1h(0)*p1t(1),
          f0*(p0h(0) + p0t(0)),
          p0h(1)*p1h(0) + p1h(0)*p0t(1) + p0h(1)*p1t(0),
          p0h(1)*p1h(1) + p1h(1)*p0t(1) + p0h(1)*p1t(1),
//      p0h(1)*p1h(0) + p1h(0)*p0t(1) + p1h(1)*p1t(0),
//      p0h(1)*p1h(1) + p1h(1)*p0t(1) + p1h(1)*p1t(1),
          f0*(p0h(1) + p0t(1)),
          f0*(p1h(0) + p1t(0)),
          f0*(p1h(1) + p1t(1)),
          f0*f0;

    return t;
}

// Make covariance matrix from two 2-vectors
Matrix9d
make_cov_mat(const Vector2d &p0, const Vector2d &p1, double f0)
{
    Matrix9d V0 = Triangulation::ZeroMat9;
    double f02 = sqr(f0);

    V0(0,0) = sqr(p0(0)) + sqr(p1(0));
    V0(1,1) = sqr(p0(0)) + sqr(p1(1));
    V0(2,2) = V0(5,5) = V0(6,6) = V0(7,7) = f02;
    V0(3,3) = sqr(p0(1)) + sqr(p1(0));
    V0(4,4) = sqr(p0(1)) + sqr(p1(1));

    V0(0,1) = V0(1,0) = V0(3,4) = V0(4,3) = p1(0) * p1(1);
    V0(0,2) = V0(2,0) = V0(3,5) = V0(5,3) = f0 * p1(0);
    V0(0,3) = V0(3,0) = V0(1,4) = V0(4,1) = p0(0) * p0(1);
    V0(0,6) = V0(6,0) = V0(1,7) = V0(7,1) = f0 * p0(0);
    V0(1,2) = V0(2,1) = V0(4,5) = V0(5,4) = f0 * p1(1);
    V0(3,6) = V0(6,3) = V0(4,7) = V0(7,4) = f0 * p0(1);

    return V0;
}

}
// end of unnamed namespace

// Optimal Correction for Triangulation
double
Triangulation::optimal_correction(
    const Vector2d& pos0,
    const Vector2d& pos1,
    const Vector9d &theta,
    Vector2d& npos0,
    Vector2d& npos1,
    int      Max_Iter,
    double   Conv_EPS,
    double   f0
)
{
    Vector9d     Xst;                 // Xi*
    Matrix9d     V0;
    Vector2d     p0h, p1h, p0t, p1t;
    double       S0, S;//, dS;
    double       c;
    int          count;
    Matrix23d    thM0, thM1;
    Vector3d     t0, t1;

    // set Matrix
    thM0 << theta(0), theta(1), theta(2), theta(3), theta(4), theta(5);
    thM1 << theta(0), theta(3), theta(6), theta(1), theta(4), theta(7);

    // Initialization
    p0h = pos0;
    p1h = pos1;
    p0t = p1t = ZeroVec2;

    // set large number to S0
    S0 = Large_Number;

    // Update by iteration
    count = 0;
    do
    {
        // Set Xi and V0
        V0 = make_cov_mat(p0h, p1h, f0);
        Xst = convert_vec9_st(p0h, p1h, p0t, p1t, f0);

        // compute ``tilde''s
        // t0 << p0h(0), p0h(1), f0;
        // t1 << p1h(0), p1h(1), f0;
        t0 << p0h, f0;
        t1 << p1h, f0;
        c = theta.dot(Xst) / (theta.dot(V0*theta));

        p0t = c * thM0 * t1;
        p1t = c * thM1 * t0;

        // update ``hat''s
        p0h = pos0 - p0t;
        p1h = pos1 - p1t;

        // compute residual
        S = p0t.norm() + p1t.norm();

        // check convergence
        if (fabs(S - S0) < Conv_EPS)  break;
        S0 = S;
    }
    while (++count < Max_Iter);

    // Maximum Iteration
    if (count >= Max_Iter) // failed
    {
        npos0 = npos1 = ZeroVec2;
    }
    else
    {
        // set corrected data
        npos0 = p0h;
        npos1 = p1h;
    }

    return S;
}

// Optimal Correction for Triangulation
bool
Triangulation::optimal_correction(
    Vector2d pos0[],     // Data:                    INPUT
    Vector2d pos1[],     // Data:                    INPUT
    int      Num,      // Number of Data:            INPUT
    const Vector9d &theta,   // Fundamental Matrix:  INPUT
    Vector2d npos0[],     // Data:                   OUTPUT
    Vector2d npos1[],     // Data:                   OUTPUT
    int      Max_Iter, // Max. Iteration:            INPUT w. default
    double   Conv_EPS, // Threshold for convergence: INPUT w. default
    double   f0        // Default focal length:      INPUT w. default
)
{
    //double S;

    for (int al = 0; al < Num; al++)
    {
        //S = optimal_correction(pos0[al], pos1[al], theta, npos0[al], npos1[al],
        //                       Max_Iter, Conv_EPS, f0);
        optimal_correction(pos0[al], pos1[al], theta, npos0[al], npos1[al],
                                      Max_Iter, Conv_EPS, f0);
    }

    return true;
}

namespace {
// make Matrix T
inline Matrix43d
make_matrix_T(const Matrix34d& P0, const Matrix34d& P1,
              const Vector2d& p0, const Vector2d& p1,
              double f0)
{
    Matrix43d    T;

    T <<
      f0*P0(0,0)-p0(0)*P0(2,0), f0*P0(0,1)-p0(0)*P0(2,1), f0*P0(0,2)-p0(0)*P0(2,2),
      f0*P0(1,0)-p0(1)*P0(2,0), f0*P0(1,1)-p0(1)*P0(2,1), f0*P0(1,2)-p0(1)*P0(2,2),
      f0*P1(0,0)-p1(0)*P1(2,0), f0*P1(0,1)-p1(0)*P1(2,1), f0*P1(0,2)-p1(0)*P1(2,2),
      f0*P1(1,0)-p1(1)*P1(2,0), f0*P1(1,1)-p1(1)*P1(2,1), f0*P1(1,2)-p1(1)*P1(2,2);

    return T;
}

// make vector p
inline Vector4d
make_vector_p(const Matrix34d& P0, const Matrix34d& P1,
              const Vector2d& p0, const Vector2d& p1,
              double f0)
{
    Vector4d     p;

    p <<
      f0*P0(0,3) - p0(0)*P0(2,3),
      f0*P0(1,3) - p0(1)*P0(2,3),
      f0*P1(0,3) - p1(0)*P1(2,3),
      f0*P1(1,3) - p1(1)*P1(2,3);

    return p;
}

}
// end of unnamed namespace

// Triangulation using least squares from two camera matrices
Vector3d
Triangulation::least_squares(
    const Vector2d& pos0,
    const Vector2d& pos1,
    const Matrix34d& P0,
    const Matrix34d& P1,
    double    f0)
{
    Matrix43d    T;
    Vector4d     p;
    Vector3d     mTtp;
    Matrix3d     TtT;
    Vector3d     X;

    // constructer of LU
    FullPivLU<Matrix3d> LU;

    T = make_matrix_T(P0, P1, pos0, pos1, f0);
    p = make_vector_p(P0, P1, pos0, pos1, f0);
    TtT = T.transpose() * T;
    mTtp = - T.transpose() * p;

    // compute least squares solution by LU decomposition
    LU.compute(TtT);
    X = LU.solve(mTtp);

    return X;
}

// Triangulation using least squares from two camera matrices
bool
Triangulation::least_squares(
    Vector2d pos0[],
    Vector2d pos1[],
    int      Num,
    const Matrix34d& P0,
    const Matrix34d& P1,
    Vector3d X[],
    double    f0)
{
    for (int al = 0; al < Num; al++)
        X[al] = least_squares(pos0[al], pos1[al], P0, P1, f0);

    return true;
}
