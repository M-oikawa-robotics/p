#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <math.h>
#include "robot.h"
#include "control.h"
#include "kyomath.h"

#define DT 0.001
#define DISP_INTERVAL 100
#define MOTION_LENGTH 2500
#define mutexLock()
#define mutexUnlock()
#define printk(arg)

/* @20180509 */
#define KPOSITION   80
#define KPOSTURE    700
#define KVPOSITION   100
#define KVPOSTURE   80

/* @20181205 */
#define KPOSITION   460
#define KPOSTURE    400
#define KVPOSITION   100
#define KVPOSTURE   80
//static double Stiff_position[3] = {150, 550,400} ;     //oikawa
static double Stiff_position[3] = {300, 300,1000} ;     //oikawa
static double Stiff_posture[3]  = {500, 500,500} ;     //oikawa
//static double Stiff_posture[3]  = {200, 200,500} ;     //oikawa
static double theta_e = -M_PI/6;  
static double theta_e = -M_PI/6;
static double theta_e = -M_PI/6;                     //剛性楕円の傾き
//static double Stiff_position[3] = {250, 250,500} ;     //oikawa
//static double Stiff_posture[3]  = {500, 500,500} ;     //oikawa



struct MotionData
{
    double x[MOTION_LENGTH];      /**< \brief 位置の$x$-軸方向成分 */
    double y[MOTION_LENGTH];      /**< \brief 位置の$z$-軸方向成分 */
    double theta[MOTION_LENGTH];  /**< \brief 位置の$\theta$-軸方向成分 */
    double vx[MOTION_LENGTH];     /**< \brief 速度の$x$-軸方向成分 */
    double vy[MOTION_LENGTH];     /**< \brief 速度の$z$-軸方向成分 */
    double omega[MOTION_LENGTH];  /**< \brief 速度の$\theta$-軸方向成分 */
    double ax[MOTION_LENGTH];     /**< \brief 加速度の$x$-軸方向成分 */
    double ay[MOTION_LENGTH];     /**< \brief 加速度の$z$-軸方向成分 */
    double domega[MOTION_LENGTH]; /**< \brief 加速度の$\theta$-軸方向成分 */
};


static void control_init(void);
static void cmd1(double time);
static void Force_abs(void);
static void direct_kinematics(void);
static void inverse_kinematics(void);
static void get_jacobian_matrix(double jacobi_matrix[6][6], const double joint_pos[6]);
static void dob(double dis[6], const double fc);
static void pos_diff(double x_dot[6], double x_dot_s[6], const double x[6], const double fc);
static void ang_diff(double x_dot[6], double x_dot_s[6], const double x[6], const double fc);
static void v3_diff(double x_dot[3], double x_dot_s[3], const double x[3], const double fc);
static void m6_diff(double m_dot[6][6], double m_dot_s[6][6], const double m[6][6], const double fc);
static void rpy_dev2omega(double omega[3], const double dev[3], const double start[3]);
static void load_path(void);
static void output_log(void);
static void calc_stiffness_ellpse_matrix(double theta_e_x,double theta_e_y,double theta_e_z,
                                         double kx,double ky,double kz);


static double spatula_pos[6];
static double spatula_vel[6];
static double spatula_vel_rpy[6];
static double spatula_vel_rpy_s[6];
static double spatula_acc[6];
static double spatula_pos_cmd[6];
static double spatula_vel_cmd[6];
static double jacobi[6][6];
static double jacobi_dot[6][6];
static double jacobi_dot_s[6][6];
static double joint_pos[6];
static double joint_vel[6];
static double joint_vel_s[6];
static double joint_acc[6];
static double joint_torque[6];
static double DOB_state[6];
static double DOB_activation;
static double Kp[6] = {KPOSITION, KPOSITION, KPOSITION, KPOSTURE, KPOSTURE, KPOSTURE};
static double Kv[6] = {KVPOSITION, KVPOSITION, KVPOSITION, KVPOSTURE, KVPOSTURE, KVPOSTURE};
static struct MotionData motion_cmd;
static const double spatula_length = 0.14;

static double Force[6] = {0.0};
static double Force_absolute[6] = {0.0};
static double Force_offset[6] = {0.0};
static double Force_cmd[6] = {0.0};
static double Force_pre[6] = {0.0};
static double Force_speed[6] = {0.0};
static int Force_frag = 0;

static int is_initialized = 0;
static int step;
static double spatula_pos0[6];     // ロボットの最初の状態
static double spatula_pos_cmd0[6]; // 理想初期状態
static std::ofstream logfile;

static int Force_control_frag = 0;
static int count_Force = 0;

static double Lowpass_back[6] = {0.0};
static double Lowpass_integral[6] = {0.0};

void control_turnover(void)
{
    int i;
    double dev[6];
    double dev_omega[6];
    double torque_dis[6];

    if (is_initialized == 0)
    {
        control_init();
    }

    // Update command states
    cmd1((double)step*DT);

    // Update joint angles
    robot_get_joint_angle(joint_pos);     //Pybulletのロボットから関節角度を取得
//    robot_get_joint_force();          //oikawa   End Effecterの力情報を取得
    ang_diff(joint_vel, joint_vel_s, joint_pos, 10.0);

    direct_kinematics();                //関節角度から手先位置を計算

    // 位置・姿勢の偏差を計算する
    for (i=0; i<6; i++)
    {
        dev[i] = spatula_pos_cmd[i] - spatula_pos[i];
    }

    // 姿勢角度の偏差の急変動を抑える
    for (i=3; i<6; i++)
    {
        if (dev[i] > M_PI)
        {
            dev[i] -= M_PI*2;
        }
        else if (dev[i] <= -M_PI)
        {
            dev[i] += M_PI*2;
        }
    }

    // 姿勢偏差の表現を角速度ベクトルの形式に変換する
    for (i=0; i<6; i++)
    {
        dev_omega[i] = dev[i];
    }
    rpy_dev2omega(&dev_omega[3], &dev[3], &spatula_pos[3]);

    // ゲイン倍して加速度指令を算出する
    for (i=0; i<6; i++)
    {
        spatula_acc[i] = Kp[i] * dev_omega[i];
        spatula_acc[i] += Kv[i] * (spatula_vel_cmd[i] - spatula_vel[i]);
        //spatula_acc[i] += position_acc_ff[i];
    }
        /*       力情報取得      */
    if((double)step*DT > 3.50)
     {
         double _g = 2.0 * M_PI * 10.0;

         robot_get_force(Force); //力情報取得
         printf("%f,%f,%f,%f,%f,%f\n",Force[0],Force[1],Force[2],Force[3],Force[4],Force[5]);
         /*       オフセット処理          */
         if (Force_frag == 0)
         {
             //for(i=0; i<6; i++) Force_offset[i] = Force[i];
             Force_frag = 1;
         }
         for(i=0; i<6; i++) Force[i] = Force[i] - Force_offset[i];
         /*                           */
         Force_abs(); // 座標変換
         /*   ローパスフィルタ   */
         for(i=0;i<6;i++)
         {
             Lowpass_integral[i] += _g*( Force[i] - Lowpass_back[i]) *DT;
             Lowpass_back[i] = Lowpass_integral[i];
             Force[i] = Lowpass_integral[i];
         }
         /*                     */
         //if (step % 100 == 0) printf("Fx = %lf, Fy = %lf, Fz = %lf\n",Force[0],Force[1],Force[2]);
         //if (step % 100 == 0) printf("Fx = %lf, Fy = %lf, Fz = %lf\n",Force_absolute[0],Force_absolute[1],Force_absolute[2]);
     }

     if(Force_control_frag == 1)
     {
        double K = 0.7, s;
        double F_acc, P_acc;

        s = 0.98;

        F_acc = s * K * -1 * ( Force_cmd[2] - Force[2] );
        P_acc = (1 - s) *  spatula_acc[2];

        spatula_acc[2] = F_acc + P_acc ;

        //if (step % 100 == 0) printf("%d:: acc = %lF, Fz = %lf, Fz_speed = %lf\n",count_Force, spatula_acc[2], Force_absolute[2], Force_speed[2]);
        if (step % 100 == 0) printf("%d:: acc = %lF, Fz = %lf\n",count_Force, spatula_acc[2], Force[2]);
        //if (step % 100 == 0) printf("Facc = %lF, Pacc = %lf\n", F_acc, P_acc);
        count_Force ++ ;
     }
     /*                      */

    inverse_kinematics();

    dob(torque_dis, 2.0);
    for (i=0; i<6; i++)
    {
        joint_torque[i] = link_inertia[i] * joint_acc[i];
        //joint_torque[i] += torque_dis[i] * DOB_activation;
    }

    // Output joint torque
    robot_set_joint_torque(joint_torque);

    output_log();

    step++;
}


static void control_init(void)
{
    int i, j;

    printk("control_turnover: initialization starts.\n");
    step = 0;

    // Initialize first states
    robot_get_joint_angle(joint_pos);
    direct_kinematics();
    get_jacobian_matrix(jacobi, joint_pos);
    for (i=0; i<6; i++)
    {
        spatula_pos0[i] = spatula_pos[i];
        spatula_vel[i] = 0.0;
        spatula_vel_rpy[i] = 0.0;
        spatula_vel_rpy_s[i] = spatula_pos[i];
        joint_vel[i] = 0.0;
        joint_vel_s[i] = joint_pos[i];
        for (j=0; j<6; j++)
        {
            jacobi_dot[i][j] = 0.0;
            jacobi_dot_s[i][j] = jacobi[i][j];
        }
        DOB_state[i] = 0.0;
    }

    DOB_activation = 0.0;

    // Initialize first commands
   // spatula_pos_cmd0[0] = 0.4;
    spatula_pos_cmd0[0] = 0.45;
    spatula_pos_cmd0[1] = 0.0;
    spatula_pos_cmd0[2] = 0.150;
    spatula_pos_cmd0[3] = 0.0;//M_PI_2 * 0.7;
    spatula_pos_cmd0[4] =  -M_PI;
    spatula_pos_cmd0[5] = 0.0;//M_PI_2;
    /* */

    //load_path();

    logfile.open("./CSV/log_turnover.csv");
    output_log();

    is_initialized = 1;
    printk("control_turnover: initialization finished.\n");
}


static void cmd1(double t)
{
    static const double t_start = 3.0;
    static const double t_pause = 3.0;
    int i;

    if (t < t_start)
    {   // 初期状態にもっていく
        double _t = t / t_start;
        for (i=0; i<6; i++)
        {
            spatula_pos_cmd[i] = spatula_pos0[i] + (spatula_pos_cmd0[i] - spatula_pos0[i])*_t;
            spatula_vel_cmd[i] = 0.0;
            DOB_activation = _t;
        }
    }
    else if (t < t_start+t_pause)
    {   // 一時停止する
        for (i=0; i<6; i++)
        {
            spatula_pos_cmd[i] = spatula_pos_cmd0[i] ;
            spatula_vel_cmd[i] = 0.0;
        }
    }
    else
    {
      /*普通の位置制御*/
      spatula_pos_cmd[0] = spatula_pos_cmd0[0] ;//+0.07*cos(2*DT*step);//+ 0.1*sin(DT*step);
      spatula_pos_cmd[1] = spatula_pos_cmd0[1] ;//+0.01*sin(4*DT*step);
      spatula_pos_cmd[2] = spatula_pos_cmd0[2] - 0.15 ;//*sin(DT*step);//0.18*sin(DT*step);
      spatula_pos_cmd[3] = spatula_pos_cmd0[3] ;
      spatula_pos_cmd[4] = spatula_pos_cmd0[4] ;
      spatula_pos_cmd[5] = spatula_pos_cmd0[5] ;

      /*普通の位置制御ここまで*/
/*
      /*アドミタンス制御による位置制御*/
/*
      spatula_pos_cmd[0] += Force[0]/Stiff_position[0];
      spatula_pos_cmd[1] += Force[1]/Stiff_position[1];
      spatula_pos_cmd[2] += Force[2]/Stiff_position[2];
      spatula_pos_cmd[3] += Force[3]/Stiff_posture[0];
      spatula_pos_cmd[4] += Force[4]/Stiff_posture[1];
      spatula_pos_cmd[5] += Force[5]/Stiff_posture[2];
*/
        /*剛性楕円による変位を計算*/
      //theta_e = (M_PI/6)*cos(DT*step);

      spatula_pos_cmd[0] += (((cos(theta_e)*cos(theta_e)/Stiff_position[0] + sin(theta_e)*sin(theta_e)/Stiff_position[2]))*Force[0]+
                              ((1/Stiff_position[2]) - (1/Stiff_position[0]))*cos(theta_e)*sin(theta_e)*Force[2]);
      spatula_pos_cmd[1] += Force[1]/Stiff_position[1];
      spatula_pos_cmd[2] += (((sin(theta_e)*sin(theta_e)/Stiff_position[0] + cos(theta_e)*cos(theta_e)/Stiff_position[2]))*Force[2]+
                              ((1/Stiff_position[2]) - (1/Stiff_position[0]))*cos(theta_e)*sin(theta_e)*Force[0]);
      //spatula_pos_cmd[3] += Force[3]/Stiff_posture[0];
      //spatula_pos_cmd[4] += Force[4]/Stiff_posture[1];
      //spatula_pos_cmd[5] += Force[5]/Stiff_posture[2];
      printf("%f\n",theta_e);

      /*アドミタンス制御による位置制御ここまで*/

      /*Imitation Learning*/
      /*
      spatula_pos_cmd[0] = LSTM_pos_cmd[0];
      spatula_pos_cmd[1] = LSTM_pos_cmd[1];
      spatula_pos_cmd[2] = LSTM_pos_cmd[2];
      spatula_pos_cmd[3] = LSTM_pos_cmd[3];
      spatula_pos_cmd[4] = LSTM_pos_cmd[4];
      spatula_pos_cmd[5] = LSTM_pos_cmd[5];

      spatula_pos_cmd[0] += Force_EE[0]/LSTM_Stiff_cmd[0];
      spatula_pos_cmd[1] += Force_EE[1]/LSTM_Stiff_cmd[1];
      spatula_pos_cmd[2] += Force_EE[2]/LSTM_Stiff_cmd[2];
      spatula_pos_cmd[3] += Force_EE[3]/LSTM_Stiff_cmd[3];
      spatula_pos_cmd[4] += Force_EE[4]/LSTM_Stiff_cmd[4];
      spatula_pos_cmd[5] += Force_EE[5]/LSTM_Stiff_cmd[5];
      /*Imitation Learning*/

        spatula_vel_cmd[0] = 0.0;
        spatula_vel_cmd[1] = 0.0;
        spatula_vel_cmd[2] = 0.0;
        spatula_vel_cmd[3] = 0.0;
        spatula_vel_cmd[4] = 0.0;
        spatula_vel_cmd[5] = 0.0;

    }
}

static void calc_stiffness_ellipse_matrix(double theta_e_x,double theta_e_y,double theta_e_z,
                                         double kx,double ky,double kz)
{
   int i;
   int j;
   int k;

   double sa;
   double ca;
   double sb, cb;
   double sg, cg;

   double   R_rpy[3][3];
   double   R_rpy_T[3][3];
   double   stiff_matrix_inverse[3][3];
   double   R_rpy_SMI[3][3];
   double   stiffness_ellipse_matrix[3][3];


   sa = sin(theta_e_x);
   ca = cos(theta_e_x);
   sb = sin(theta_e_y);
   cb = cos(theta_e_y);
   sg = sin(theta_e_z);
   cg = cos(theta_e_z);

   R_rpy[0][0] = cb*ca;
   R_rpy[0][1] = sg*sb*ca - cg*sa;
   R_rpy[0][2] = sg*sa - cg*sb*ca;
   R_rpy[1][0] = cb*sa;
   R_rpy[1][1] = sg*sb*sa + cg*ca;
   R_rpy[1][2] = -sg*ca + cg*sb*sa;
   R_rpy[2][0] = -sb;
   R_rpy[2][1] = sg*cb;
   R_rpy[2][2] = cg*cb;

   R_rpy_T[0][0] = cb*ca;
   R_rpy_T[0][1] = cb*sa;
   R_rpy_T[0][2] = -sb;
   R_rpy_T[1][0] = sg*sb*ca - cg*sa;
   R_rpy_T[1][1] = sg*sb*sa + cg*ca;
   R_rpy_T[1][2] = sg*cb;
   R_rpy_T[2][0] = sg*sa - cg*sb*ca;
   R_rpy_T[2][1] = -sg*ca + cg*sb*sa;
   R_rpy_T[2][2] = cg*cb;

   stiff_matrix_inverse[0][0] = 1/kx;
   stiff_matrix_inverse[0][1] = 0;
   stiff_matrix_inverse[0][2] = 0;
   stiff_matrix_inverse[1][0] = 0;
   stiff_matrix_inverse[1][1] = 1/ky;
   stiff_matrix_inverse[1][2] = 0;
   stiff_matrix_inverse[2][0] = 0;
   stiff_matrix_inverse[2][1] = 0;
   stiff_matrix_inverse[2][2] = 1/kz;

  for (i=0; i<3; i++)
      for (j=0; j<3; j++)
        for (k=0; k<3; k++)
      {
    {
    R_rpy_SMI[i][j] += R_rpy_T[i][k] * stiff_matrix_inverse[k][j];
    }
      }

    for (i=0; i<3; i++)
        for (j=0; j<3; j++)
          for (k=0; k<3; k++)
          {
        {
        stiffness_ellipse_matrix[i][j] += R_rpy_SMI[i][k] * R_rpy[k][j];
        }
          }


}

static void Force_abs(void)
{
    double _Force[6], _pos[3];
    double _Force1[6],_Force2[6];
    int i;
    double cr,sr;
    double cp,sp;
    double cy,sy;
    for(i=0;i<6;i++) _Force[i] = Force[i];
    for(i=0;i<3;i++) _pos[i] = spatula_pos[i+3];
    cr = cos(_pos[0]);
    sr = sin(_pos[0]);
    cp = cos(_pos[1]);
    sp = sin(_pos[1]);
    cy = cos(_pos[2]);
    sy = sin(_pos[2]);
    /*  ロール軸変換   */
    _Force1[0] = _Force[0] + 0 + 0;
    _Force1[1] = 0 + cr*_Force[1] + -sr*_Force[2];
    _Force1[2] = 0 + sr*_Force[1] + cr*_Force[2];
    /*              */
    /*  ピッチ軸変換   */
    _Force2[0] = cp*_Force1[0] + 0 + sp*_Force1[2];
    _Force2[1] = 0 + _Force1[1] + 0;
    _Force2[2] = -sp*_Force1[0] + 0 + cp*_Force1[2];
    /*              */
    /*  ヨー軸変換   */
    Force_absolute[0] = cy*_Force2[0] + -sy*_Force2[1] + 0;
    Force_absolute[1] = sy*_Force2[0] + cy*_Force2[1] + 0;
    Force_absolute[2] = 0 + 0 + _Force2[2];
    /*              */

}

static void direct_kinematics(void)
{
    double T[4][4];

    robot_get_transformation(T, joint_pos);     // transformation from origin to hand
    m4_translate(T, 0.0, 0.0, spatula_length);  // transformation from hand to spatula

    // extract position
    int i;
    for (i=0; i<3; i++)
    {
        spatula_pos[i] = T[i][3];
    }

    // extract attitude
    /* R = Rx Rz Ry */
    if (T[0][1] >= 1.0)
    {
        spatula_pos[3] = atan2(T[1][0], -T[2][0]);
        spatula_pos[4] = atan2(-T[1][2], T[2][2]);
        spatula_pos[5] = -M_PI;
    }
    else if (T[0][1] <= -1.0)
    {
        spatula_pos[3] = atan2(T[1][0], -T[2][0]);
        spatula_pos[4] = atan2(-T[1][2], T[2][2]);
        spatula_pos[5] = M_PI;
    }
    else
    {
        spatula_pos[3] = atan2(T[2][1], T[1][1]);
        spatula_pos[4] = atan2(T[0][2], T[0][0]);
        spatula_pos[5] = -asin(T[0][1]);
    }

    pos_diff(spatula_vel_rpy, spatula_vel_rpy_s, spatula_pos, 10.0);
    for (i=0; i<3; i++)
    {
        spatula_vel[i] = spatula_vel_rpy[i];
    }
    rpy_dev2omega(&spatula_vel[3], &spatula_vel_rpy[3], &spatula_pos[3]);
}


static void inverse_kinematics(void)
{
    int i, j;
    double _accel[6];
    double jth[6];
    int p[6];
    int signum;
    double det;

    get_jacobian_matrix(jacobi, joint_pos);
    m6_diff(jacobi_dot, jacobi_dot_s, jacobi, 10.0);
    for (i=0; i<6; i++)
    {
        jth[i] = 0.0;
        for (j=0; j<6; j++)
        {
            jth[i] += jacobi_dot[i][j] * joint_vel[j];
        }
        _accel[i] = spatula_acc[i] - jth[i];
    }

    m6_LU_decomp(jacobi, p, &signum);
    det = m6_det_LU(jacobi, signum);

    if (fabs(det) > 1e-10)
    {
        v6_gaussian_elimination_LU(joint_acc, jacobi, _accel, p);
    }
    else
    {
        //kyo_vector6d_set_zero(joint_accel_ref);
        //std::cout << "det(J) = 0!!!" << std::endl;
    }
}


static void get_jacobian_matrix(double jacobi_matrix[6][6], const double joint_pos[6])
{
    double T[4][4];
    double dp[6][3];
    int i, j;

    for (i=0; i<4; i++)
    {
        for (j=0; j<4; j++)
        {
            T[i][j] = (i==j) ? 1.0 : 0.0;
        }
    }

    for (i=0; i<3; i++)
    {
        jacobi_matrix[i+3][0] = T[i][2];
    }

    m4_rotate(T, joint_pos[0], 'z');
    for (i=0; i<3; i++)
    {
        dp[0][i] = T[i][2] * link_length[1];
        jacobi_matrix[i+3][1] = T[i][1];
    }

    m4_rotate(T, joint_pos[1], 'y');
    for (i=0; i<3; i++)
    {
        dp[1][i] = T[i][2] * link_length[2];
        jacobi_matrix[i+3][2] = T[i][1];
    }

    m4_rotate(T, joint_pos[2], 'y');
    for (i=0; i<3; i++)
    {
        dp[2][i] = T[i][2] * link_length[3] + T[i][1] * (-0.03);
        jacobi_matrix[i+3][3] = T[i][2];
    }

     m4_rotate(T, joint_pos[3], 'z');
     for (i=0; i<3; i++)
     {
         dp[3][i] = T[i][2] * link_length[4];
         jacobi_matrix[i+3][4] = T[i][1];
     }

    m4_rotate(T, joint_pos[4], 'y');
    for (i=0; i<3; i++)
    {
        dp[4][i] = T[i][2] * link_length[5];
        jacobi_matrix[i+3][5] = T[i][2];
    }

    m4_rotate(T, joint_pos[5], 'z');
    for (i=0; i<3; i++)
    {
        dp[5][i] = T[i][2] * (link_length[6] + spatula_length);
    }

    double p[3];
    for (i=0; i<3; i++)
    {
        p[i] = 0.0;
    }
    for (i=5; i>=0; i--)
    {
        for (j=0; j<3; j++)
        {
            p[j] += dp[i][j];
        }
        jacobi_matrix[0][i] = jacobi_matrix[4][i]*p[2] - jacobi_matrix[5][i]*p[1];
        jacobi_matrix[1][i] = jacobi_matrix[5][i]*p[0] - jacobi_matrix[3][i]*p[2];
        jacobi_matrix[2][i] = jacobi_matrix[3][i]*p[1] - jacobi_matrix[4][i]*p[0];
    }
}


static void dob(double dis[6], const double fc)
{
    double g = 2.0*M_PI * fc;
    double gp[6];
    double x[6];
    double z[6];
    int i;

    for (i=0; i<6; i++)
    {
        // Calculate gp = g * mv
        gp[i] = g * link_inertia[i] * joint_vel[i];

        x[i] = gp[i] + joint_torque[i];

        // LPF: x -> DOB_state
        z[i] = g * (x[i] - DOB_state[i]);
        DOB_state[i] += z[i] * DT;

        dis[i] = DOB_state[i] - gp[i];
    }
}


static void pos_diff(double x_dot[6], double x_dot_s[6], const double x[6], const double fc)
{
    int i;

    double _g = 2.0*M_PI * fc;
    double _x_normalized[6];

/* 姿勢角度の時間変化の急変動を抑える */
    for (i=0; i<6; i++)
    {
        _x_normalized[i] = x[i];
    }
    for (i=3; i<6; i++)
    {
        if (_x_normalized[i] - x_dot_s[i] > M_PI)
        {
            _x_normalized[i] -= M_PI*2;
        }
        else if (_x_normalized[i] - x_dot_s[i] <= -M_PI)
        {
            _x_normalized[i] += M_PI*2;
        }
    }
/* */

    for (i=0; i<6; i++)
    {
        x_dot_s[i] += x_dot[i] * DT;
        x_dot[i] = _g * (_x_normalized[i] - x_dot_s[i]);
    }
}


static void ang_diff(double x_dot[6], double x_dot_s[6], const double x[6], const double fc)
{
    int i;

    double _g = 2.0*M_PI * fc;
    double _x_normalized[6];

    /* 姿勢角度の時間変化の急変動を抑える */
    for (i=0; i<6; i++)
    {
        _x_normalized[i] = x[i];
    }
    for (i=0; i<6; i++)
    {
        if (_x_normalized[i] - x_dot_s[i] > M_PI)
        {
            _x_normalized[i] -= M_PI*2;
        }
        else if (_x_normalized[i] - x_dot_s[i] <= -M_PI)
        {
            _x_normalized[i] += M_PI*2;
        }
    }
    /* */

    for (i=0; i<6; i++)
    {
        x_dot_s[i] += x_dot[i] * DT;
        x_dot[i] = _g * (_x_normalized[i] - x_dot_s[i]);
    }
}


static void v3_diff(double x_dot[3], double x_dot_s[3], const double x[3], const double fc)
{
    int i;

    double _g = 2.0*M_PI * fc;

    for (i=0; i<3; i++)
    {
        x_dot_s[i] += x_dot[i] * DT;
        x_dot[i] = _g * (x[i] - x_dot_s[i]);
    }
}


static void m6_diff(double m_dot[6][6], double m_dot_s[6][6], const double m[6][6], const double fc)
{
    int i, j;

    double g = 2.0*M_PI * fc;

    for (i=0; i<6; i++)
    {
        for (j=0; j<6; j++)
        {
            m_dot_s[i][j] += m_dot[i][j] * DT;
            m_dot[i][j] = g * (m[i][j] - m_dot_s[i][j]);
        }
    }
}


static void rpy_dev2omega(double omega[3], const double dev[3], const double start[3])
{
    int i;

    /* R = Rx Rz Ry */
    double cx = cos(start[0]);
    double cz = cos(start[2]);
    double sx = sin(start[0]);
    double sz = sin(start[2]);
    omega[0] = dev[0] - sz*dev[1];
    omega[1] = cx*cz*dev[1] - sx*dev[2];
    omega[2] = sx*cz*dev[1] + cx*dev[2];
}


static void load_path(void)
{
    int i;
    float x, y, theta, vx, vy, omega, vx_pre, vy_pre, omega_pre;
    float _tmp;

    FILE *fp1;
    //char *fname = "/home/d-kiki/kyo_simulate/path_0902/a20_b005_n.csv";
    //char *fname = "/home/d-kiki/kyo_simulate/kyo_simulate/path_NN.csv";
    char *fname = "m05_a20_b01_n.csv";

    fp1 = fopen(fname, "r");
    if (fp1 == NULL)
    {
        printf("Could not open %s.¥n", fname);
    }

    motion_cmd.ax[0]     = 0;
    motion_cmd.ay[0]     = 0;
    motion_cmd.domega[0] = 0;
    vx_pre = vy_pre = omega_pre = 0;

    // |   ||           Okonomiyaki             ||              Spatula              |
    // | t || x | y |theta|x_dot|y_dot|theta_dot|| x | y |theta|x_dot|y_dot|theta_dot|
    // | 0 || 1 | 2 |  3  |  4  |  5  |    6    || 7 | 8 |  9  |  10 |  11 |    12   |
    motion_cmd.x[0]     = 0;
    motion_cmd.y[0]     = 0;
    motion_cmd.theta[0] = 0;
    motion_cmd.vx[0]    = 0;
    motion_cmd.vy[0]    = 0;
    motion_cmd.omega[0] = 0;
    i = 1;
    while ((fscanf(fp1, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &_tmp, &_tmp, &_tmp, &_tmp, &_tmp, &_tmp, &_tmp, &x, &y, &theta, &vx, &vy, &omega) != EOF)
           && (i < MOTION_LENGTH))
    {
        motion_cmd.x[i]     = x;
        motion_cmd.y[i]     = y - 0.2;
        motion_cmd.theta[i] = theta;
        motion_cmd.vx[i]    = vx;
        motion_cmd.vy[i]    = vy;
        motion_cmd.omega[i] = omega;

        motion_cmd.ax[i-1]     = (vx-vx_pre)/DT;
        motion_cmd.ay[i-1]     = (vy-vy_pre)/DT;
        motion_cmd.domega[i-1] = (omega-omega_pre)/DT;
        vx_pre = vx;
        vy_pre = vy;
        omega_pre = omega;
        i++;
    }

    for (; i<MOTION_LENGTH; i++)
    {
        motion_cmd.x[i]     = x;
        motion_cmd.y[i]     = y - 0.2;
        motion_cmd.theta[i] = theta;
        motion_cmd.vx[i]    = 0;
        motion_cmd.vy[i]    = 0;
        motion_cmd.omega[i] = 0;
        motion_cmd.ax[i-1]     = 0;
        motion_cmd.ay[i-1]     = 0;
        motion_cmd.domega[i-1] = 0;
    }
    motion_cmd.ax[i-1]     = 0;
    motion_cmd.ay[i-1]     = 0;
    motion_cmd.domega[i-1] = 0;

    /* *
    for (i=0; i<MOTION_LENGTH; i++)
    {
        std::cout << motion_cmd.x[i] << "," << motion_cmd.y[i] << "," << motion_cmd.theta[i] << ",";
        std::cout << motion_cmd.vx[i] << "," << motion_cmd.vy[i] << "," << motion_cmd.omega[i] << ",";
        std::cout << std::endl;
    }
    /* */

    fclose(fp1);
}


static void v6_output(const char* name, const double v[6])
{
#if ACTUALROBOT_ENVIRONMENT == 1
    int i;
    printk(name);
    printk(":");
    for (i=0; i<6; i++)
    {
        printk(" %6d,", (int)(v[i]*1e3));
    }
    printk("\n");
#else
    int i;
    std::cout << std::fixed << std::showpos << std::setprecision(6);
    std::cout << name << ":";
    for (i=0; i<6; i++)
    {
        std::cout << std::setw(10) << v[i] << ",";
    }
    std::cout << std::endl;
#endif
}


static void output_log(void)
{
#if ACTUALROBOT_ENVIRONMENT == 1
    if (is_initialized = 1)
    {
        mutexLock();
        mutexUnlock();
    }
#else
    int i;
    if (is_initialized == 0)
    {
        logfile << "t";
        logfile << "," << "p1" << "," << "p2" << "," << "p3" << "," << "p4" << "," << "p5" << "," << "p6";
        logfile << "," << "v1" << "," << "v2" << "," << "v3" << "," << "v4" << "," << "v5" << "," << "v6";
        logfile << "," << "pc1" << "," << "pc2" << "," << "pc3" << "," << "pc4" << "," << "pc5" << "," << "pc6";
        logfile << "," << "vc1" << "," << "vc2" << "," << "vc3" << "," << "vc4" << "," << "vc5" << "," << "vc6";
        logfile << "," << "j1" << "," << "j2" << "," << "j3" << "," << "j4" << "," << "j5" << "," << "j6";
        logfile << "," << "t1" << "," << "t2" << "," << "t3" << "," << "t4" << "," << "t5" << "," << "t6";
        logfile << "," << "Fx" << "," << "Fy" << "," << "Fz" << "," << "Mx" << "," << "My" << "," << "Mz";
        logfile << std::endl;
    }
    else
    {
        /* Display */
        if (step%DISP_INTERVAL == 0)
        {
            std::cout << std::endl;
            v6_output("p  ", spatula_pos);
            v6_output("pc ", spatula_pos_cmd);
            v6_output("ac ", spatula_acc);
            v6_output("trq", joint_torque);
            std::cout << std::endl;
        }
        /* */

        /* Record */
        logfile << step*DT;
        for (i=0; i<6; i++)
        {
            logfile << "," << spatula_pos[i];
        }
        for (i=0; i<6; i++)
        {
            logfile << "," << spatula_vel_rpy[i];
        }
        for (i=0; i<6; i++)
        {
            logfile << "," << spatula_pos_cmd[i];
        }
        for (i=0; i<6; i++)
        {
            logfile << "," << spatula_vel_cmd[i];
        }
        for (i=0; i<6; i++)
        {
            logfile << "," << joint_pos[i];
        }
        for (i=0; i<6; i++)
        {
            logfile << "," << joint_torque[i];
        }
        for (i=0; i<6; i++)
        {
            logfile << "," << Force[i];
        }
        logfile << std::endl;
        /* */
    }
#endif
}
