#include <math.h>
#include <string.h>

void m3_copy(double dist[3][3], const double src[3][3])
{
    memcpy(dist, src, sizeof(double[3][3]));
}


void m3_inverse(double m_inv[3][3], const double m[3][3])
{
    double _m[3][3];
    double buf;
    int i, j, k;

    m3_copy(_m, m);

    for (i=0; i<3; i++)
    {
        for (j=0; j<3; j++)
        {
            if (i == j)
            {
                m_inv[i][j] = 1.0;
            }
            else
            {
                m_inv[i][j] = 0.0;
            }
        }
    }

    for (i=0; i<3; i++)
    {
        buf = 1/_m[i][i];
        for (j=0; j<3; j++)
        {
            _m[i][j] *= buf;
            m_inv[i][j] *= buf;
        }

        for (j=0; j<3; j++)
        {
            if (i != j)
            {
                buf = _m[j][i];
                for (k=0; k<3; k++)
                {
                    _m[j][k] -= _m[i][k] * buf;
                    m_inv[j][k] -= m_inv[i][k] * buf;
                }
            }
        }
    }
}


void m4_translate(double t[4][4], double x, double y, double z)
{
    t[0][3] = t[0][0]*x + t[0][1]*y + t[0][2]*z + t[0][3];
    t[1][3] = t[1][0]*x + t[1][1]*y + t[1][2]*z + t[1][3];
    t[2][3] = t[2][0]*x + t[2][1]*y + t[2][2]*z + t[2][3];
}


void m4_rotate(double t[4][4], double angle, char axis)
{
    int i;
    double tmp1, tmp2;
    double c = cos(angle);
    double s = sin(angle);

    switch (axis)
    {
    case 'x':
        for (i=0; i<3; i++)
        {
            tmp1 = t[i][1];
            tmp2 = t[i][2];
            t[i][1] = tmp1*c + tmp2*s;
            t[i][2] = -tmp1*s + tmp2*c;
        }
        break;

    case 'y':
        for (i=0; i<3; i++)
        {
            tmp1 = t[i][2];
            tmp2 = t[i][0];
            t[i][2] = tmp1*c + tmp2*s;
            t[i][0] = -tmp1*s + tmp2*c;
        }
        break;

    case 'z':
        for (i=0; i<3; i++)
        {
            tmp1 = t[i][0];
            tmp2 = t[i][1];
            t[i][0] = tmp1*c + tmp2*s;
            t[i][1] = -tmp1*s + tmp2*c;
        }
        break;

    default:
        break;
    }
}


void m4_get_rotation(double r[3][3], const double t[4][4])
{
    int i, j;
    for (i=0; i<3; i++) for (j=0; j<3; j++) r[i][j] = t[i][j];
}


void m6_LU_decomp(double m[6][6], int p[6], int* signum)
{
    int i, j, k;

    *signum = 1;
    for (i=0; i<6; i++) p[i] = i;

    for (j=0; j<5; j++)
    {
        /* Find maximum in the j-th column */

        double mjj;
        double max = fabs(m[j][j]);
        int i_pivot = j;

        for (i=j+1; i<6; i++)
        {
            double mij = fabs(m[i][j]);

            if (mij > max)
            {
                max = mij;
                i_pivot = i;
            }
        }

        if (i_pivot != j)
        {
//            gsl_matrix_swap_rows (A, j, i_pivot);
//            gsl_permutation_swap (p, j, i_pivot);
            double tmp;
            for (k=0; k<6; k++)
            {
                tmp = m[j][k];
                m[j][k] = m[i_pivot][k];
                m[i_pivot][k] = tmp;
            }
            int tmpi = p[j];
            p[j] = p[i_pivot];
            p[i_pivot] = tmpi;
            *signum = -(*signum);
        }

        mjj = m[j][j];

        if (mjj != 0.0)
        {
            for (i=j+1; i<6; i++)
            {
                double mij = m[i][j] / mjj;
                m[i][j] = mij;

                for (k=j+1; k<6; k++)
                    m[i][k] -= mij * m[j][k];
            }
        }
    }
}


void v6_gaussian_elimination_LU(double x[6], const double LU[6][6], const double b[6], const int p[6])
{
#if CHECKARG == 1

#endif // CHECKARG
    int i, j;
    double y[6];
    double bp[6];
    double tmp;

    for (i=0; i<6; i++) bp[i] = b[p[i]];

    /* 前進代入 Ly = b */
    for (i=0; i<6; i++)
    {
        tmp = 0.0;
        for (j=0; j<i; j++)
        {
            tmp += LU[i][j] * y[j];
        }
        y[i] = bp[i] - tmp;
    }

    /* 後退代入 Ux = y */
    for (i=5; i>=0; i--)
    {
        tmp = 0.0;
        for (j=i+1; j<6; j++)
        {
            tmp += LU[i][j] * x[j];
        }
        x[i] = (y[i] - tmp) / LU[i][i];
    }
}


double m6_det_LU(const double LU[6][6], int signum)
{
    double det = (double)signum;
    int i;
    for (i=0; i<6; i++)
    {
        det *= LU[i][i];
    }

    return det;
}
