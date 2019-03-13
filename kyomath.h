#ifndef KYOMATH_INCLUDED_FA4ED750_4DA6_42A9_BEE6_1EB4EA2539E0
#define KYOMATH_INCLUDED_FA4ED750_4DA6_42A9_BEE6_1EB4EA2539E0

void m3_copy(double dist[3][3], const double src[3][3]);
void m3_inverse(double m_inv[3][3], const double m[3][3]);

void m4_translate(double t[4][4], double x, double y, double z);
void m4_rotate(double t[4][4], double angle, char axis);
void m4_get_rotation(double r[3][3], const double t[4][4]);
void m6_LU_decomp(double m[6][6], int p[6], int* signum);
void v6_gaussian_elimination_LU(double x[6], const double LU[6][6], const double b[6], const int p[6]);
double m6_det_LU(const double LU[6][6], int signum);

#endif  // KYOMATH_INCLUDED_FA4ED750_4DA6_42A9_BEE6_1EB4EA2539E0
