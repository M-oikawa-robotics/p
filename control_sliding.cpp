#include <iostream>
#include <iomanip>
#include <fstream>
#include "robot.h"
#include "control.h"
#include "kyomath.h"
#include "nn_interface.h"
#include "pancake.h"

#define DT 0.001
#define DISP_INTERVAL 100
#define MOTION_LENGTH 2500
#define mutexLock()
#define mutexUnlock()
#define printk(arg)

/* @0902T1647 *
#define KPOSITION   450
#define KPOSTURE    500
#define KVPOSITION   50
#define KVPOSTURE    60
/* */
#define KPOSITION   800
#define KPOSTURE   1000
#define KVPOSITION  100
#define KVPOSTURE   150
/* */


struct MotionData
{
    double x[MOTION_LENGTH];      /**< \brief 位置の$x$-軸方向成分 */
    double y[MOTION_LENGTH];      /**< \brief 位置の$z$-軸方向成分 */
    double vx[MOTION_LENGTH];     /**< \brief 速度の$x$-軸方向成分 */
    double vy[MOTION_LENGTH];     /**< \brief 速度の$z$-軸方向成分 */
    double ax[MOTION_LENGTH];     /**< \brief 加速度の$x$-軸方向成分 */
    double ay[MOTION_LENGTH];     /**< \brief 加速度の$z$-軸方向成分 */
};


static void control_init(void);
static void cmd1(double time);
static void cmd2(double time);
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


static double spatula_pos[6];
static double spatula_vel[6];
static double spatula_vel_rpy[6];
static double spatula_vel_rpy_s[6];
static double spatula_acc[6];
static double spatula_pos_cmd[6];
static double spatula_vel_cmd[6];
static double spatula_acc_ff[6];
static double transformation[4][4];
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
static const double spatula_length = 0.30;

static double actual_spatula_state[4];
static double actual_object_state[4];

static int is_initialized = 0;
static int is_use_NN = 0;
static int step;
static double spatula_pos0[6];     // ロボットの最初の状態
static double spatula_pos_cmd0[6]; // 理想初期状態
static std::ofstream logfile;


void control_sliding(void)
{
    int i;
    double dev[6];
    double dev_omega[6];
    double torque_dis[6];

    if (is_initialized == 0)
    {
        control_init();
    }

    // Update joint angles
    robot_get_joint_angle(joint_pos);
    ang_diff(joint_vel, joint_vel_s, joint_pos, 20.0);

    direct_kinematics();

    // Update command states
    cmd2((double)step*DT);

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
        //spatula_acc[i] += spatula_acc_ff[i];
    }
    /* *
    if (is_use_NN)
    {
        double x[8];
        double u[2];

        get_object_state(actual_spatula_state, actual_object_state);
        x[0] = actual_spatula_state[0];
        x[1] = actual_spatula_state[1];
        x[2] = actual_spatula_state[2];
        x[3] = actual_spatula_state[3];
        x[4] = actual_object_state[0];
        x[5] = actual_object_state[1];
        x[6] = actual_object_state[2];
        x[7] = actual_object_state[3];

        input_state(x);
        receive_action(u);

        spatula_acc[0] = u[0];
        spatula_acc[1] = u[1];
    }
    /* */

    inverse_kinematics();

    dob(torque_dis, 10.0);
    for (i=0; i<6; i++)
    {
        joint_torque[i] = link_inertia[i] * joint_acc[i];
        joint_torque[i] += torque_dis[i] * DOB_activation;
    }

    // Output joint torque
    robot_set_joint_torque(joint_torque);

    output_log();

    step++;
}


static void control_init(void)
{
    int i, j;

    printk("control_spatula: initialization starts.\n");
    step = 0;
    
    initialize_NN();
    for (i=0; i<4; i++)
    {
        actual_spatula_state[i] = 0.0;
        actual_object_state[i] = 0.0;
    }
    
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

        spatula_vel_cmd[i] = 0.0;
        spatula_acc_ff[i] = 0.0;
    }

    // Initialize first commands
    spatula_pos_cmd0[0] = 0.65;
    spatula_pos_cmd0[1] = 0.0;
    spatula_pos_cmd0[2] = 0.3;
    spatula_pos_cmd0[3] = 0.0;
    spatula_pos_cmd0[4] = M_PI_2;
    spatula_pos_cmd0[5] = 0.0;
    /* */

    load_path();

    logfile.open("log_sliding.csv");
    output_log();

    is_initialized = 1;
    printk("control_spatula: initialization finished.\n");
}


static void cmd1(double t)
{
    static const double t_start = 3.0;
    static const double t_pause = 1.0;
    int i;

    if (t < t_start)
    {   // 初期状態にもっていく
        double _t = t / t_start;
        for (i=0; i<6; i++)
        {
            spatula_pos_cmd[i] = spatula_pos0[i] + (spatula_pos_cmd0[i] - spatula_pos0[i])*_t;
            spatula_vel_cmd[i] = 0.0;
        }
    }
    else if (t < t_start+t_pause)
    {   // 一時停止する
        for (i=0; i<6; i++)
        {
            spatula_pos_cmd[i] = spatula_pos_cmd0[i];
            spatula_vel_cmd[i] = 0.0;
        }
    }
    else
    {   // ひっくり返す
        int tau = (int)((t-(t_start+t_pause))/DT);
        if (tau < MOTION_LENGTH)
        {
            spatula_pos_cmd[0] = spatula_pos_cmd0[0] + motion_cmd.x[tau];
            spatula_pos_cmd[1] = spatula_pos_cmd0[1] + motion_cmd.y[tau];
            spatula_vel_cmd[0] = motion_cmd.vx[tau];
            spatula_vel_cmd[1] = motion_cmd.vy[tau];
        }
    }
}


static void cmd2(double t)
{
    static const double t_start = 1.0;
    static const double t_pause = 5.0;
    int i;
    int eom;

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
            spatula_pos_cmd[i] = spatula_pos_cmd0[i];
            spatula_vel_cmd[i] = 0.0;
        }
    }
    else
    {
        is_end_of_motion(&eom);
        if (eom == 0)
        {
            double x[8];
            double u[2];

            get_object_state(actual_spatula_state, actual_object_state);
            x[0] = actual_spatula_state[0];
            x[1] = actual_spatula_state[1];
            x[2] = actual_spatula_state[2];
            x[3] = actual_spatula_state[3];
            x[4] = actual_object_state[0];
            x[5] = actual_object_state[1];
            x[6] = actual_object_state[2];
            x[7] = actual_object_state[3];

            input_state(x);
            receive_action(u);

            for (i=0; i<2; i++)
            {
                spatula_pos_cmd[i] += spatula_vel_cmd[i]*DT + u[i]*DT*DT/2;
                spatula_vel_cmd[i] += u[i]*DT;
                spatula_acc_ff[i] = u[i];
            }

            is_use_NN = 1;

            /* *
            for (i=0; i<6; i++)
            {
                spatula_pos_cmd[i] = spatula_pos[i];
            }
            /* */
        }
        else
        {
            is_use_NN = 0;

            for (i=0; i<6; i++)
            {
                spatula_vel_cmd[i] = 0.0;
            }
        }
    }
}


static void direct_kinematics(void)
{
    robot_get_transformation(transformation, joint_pos);     // transformation from origin to hand
    m4_translate(transformation, 0.0, 0.0, spatula_length);  // transformation from hand to spatula

    // extract position
    int i;
    for (i=0; i<3; i++)
    {
        spatula_pos[i] = transformation[i][3];
    }

    // extract attitude
    /* R = Rx Rz Ry */
    if (transformation[0][1] >= 1.0)
    {
        spatula_pos[3] = atan2(transformation[1][0], -transformation[2][0]);
        spatula_pos[4] = atan2(-transformation[1][2], transformation[2][2]);
        spatula_pos[5] = -M_PI;
    }
    else if (transformation[0][1] <= -1.0)
    {
        spatula_pos[3] = atan2(transformation[1][0], -transformation[2][0]);
        spatula_pos[4] = atan2(-transformation[1][2], transformation[2][2]);
        spatula_pos[5] = M_PI;
    }
    else
    {
        spatula_pos[3] = atan2(transformation[2][1], transformation[1][1]);
        spatula_pos[4] = atan2(transformation[0][2], transformation[0][0]);
        spatula_pos[5] = -asin(transformation[0][1]);
    }

    pos_diff(spatula_vel_rpy, spatula_vel_rpy_s, spatula_pos, 20.0);
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
    static double det_min = 10.0;

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

    //if ((fabs(det) < det_min) && (fabs(det) > 1e-10))
    if (fabs(det) < det_min)
    {
        std::cout << "det(J):" << det << std::endl;
        det_min = fabs(det);
    }

    if (fabs(det) > 0.001)
    {
        v6_gaussian_elimination_LU(joint_acc, jacobi, _accel, p);
    }
    else
    {
        //kyo_vector6d_set_zero(joint_accel_ref);
        std::cout << "det(J) = 0!!!" << std::endl;
        quit_generate();
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

    motion_cmd.ax[0] = 0;
    motion_cmd.ay[0] = 0;
    vx_pre = vy_pre = 0;

    // |   ||           Okonomiyaki             ||              Spatula              |
    // | t || x | y |theta|x_dot|y_dot|theta_dot|| x | y |theta|x_dot|y_dot|theta_dot|
    // | 0 || 1 | 2 |  3  |  4  |  5  |    6    || 7 | 8 |  9  |  10 |  11 |    12   |
    motion_cmd.x[0]  = 0;
    motion_cmd.y[0]  = 0;
    motion_cmd.vx[0] = 0;
    motion_cmd.vy[0] = 0;
    i = 1;
    while ((fscanf(fp1, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f",
                   &_tmp, &_tmp, &_tmp, &_tmp, &_tmp, &_tmp, &_tmp, &x, &y, &theta, &vx, &vy, &omega) != EOF)
           && (i < MOTION_LENGTH))
    {
        motion_cmd.x[i]  = x;
        motion_cmd.y[i]  = y - 0.2;
        motion_cmd.vx[i] = vx;
        motion_cmd.vy[i] = vy;
        motion_cmd.ax[i-1] = (vx-vx_pre)/DT;
        motion_cmd.ay[i-1] = (vy-vy_pre)/DT;
        vx_pre = vx;
        vy_pre = vy;
        i++;
    }

    for (; i<MOTION_LENGTH; i++)
    {
        motion_cmd.x[i]  = x;
        motion_cmd.y[i]  = y - 0.2;
        motion_cmd.vx[i] = 0;
        motion_cmd.vy[i] = 0;
        motion_cmd.ax[i-1] = 0;
        motion_cmd.ay[i-1] = 0;
    }
    motion_cmd.ax[i-1] = 0;
    motion_cmd.ay[i-1] = 0;

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


static void v4_output(const char* name, const double v[6])
{
#if ACTUALROBOT_ENVIRONMENT == 1
    int i;
    printk(name);
    printk(":");
    for (i=0; i<4; i++)
    {
        printk(" %6d,", (int)(v[i]*1e3));
    }
    printk("\n");
#else
    int i;
    std::cout << std::fixed << std::showpos << std::setprecision(6);
    std::cout << name << ":";
    for (i=0; i<4; i++)
    {
        std::cout << std::setw(10) << v[i] << ",";
    }
    std::cout << std::endl;
#endif
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
        logfile << "," << "a1" << "," << "a2" << "," << "a3" << "," << "a4" << "," << "a5" << "," << "a6";
        logfile << "," << "pc1" << "," << "pc2" << "," << "pc3" << "," << "pc4" << "," << "pc5" << "," << "pc6";
        logfile << "," << "vc1" << "," << "vc2" << "," << "vc3" << "," << "vc4" << "," << "vc5" << "," << "vc6";
        logfile << "," << "ac1" << "," << "ac2" << "," << "ac3" << "," << "ac4" << "," << "ac5" << "," << "ac6";
        logfile << "," << "xs1" << "," << "xs2" << "," << "xs3" << "," << "xs4";
        logfile << "," << "xd1" << "," << "xd2" << "," << "xd3" << "," << "xd4";
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
            v4_output("xs ", actual_spatula_state);
            v4_output("xd ", actual_object_state);
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
            logfile << "," << spatula_acc[i];
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
            logfile << "," << spatula_acc_ff[i];
        }
        for (i=0; i<4; i++)
        {
            logfile << "," << actual_spatula_state[i];
        }
        for (i=0; i<4; i++)
        {
            logfile << "," << actual_object_state[i];
        }
        logfile << std::endl;
        /* */
    }
#endif
}