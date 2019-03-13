#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include "robot.h"
#include "control.h"

#define DT 0.001
#define DISP_INTERVAL 100
#define mutexLock()
#define mutexUnlock()
#define printk(arg)


static void control_init(void);
static void cmd1(double time);
static void dob(double dis[6], const double fc);
static void ang_diff(double x_dot[6], double x_dot_s[6], const double x[6], const double fc);
static void output_log(void);


static double joint_angle[6];
static double joint_angle_cmd[6];
static double joint_torque[6];
static double joint_vel[6];
static double joint_vel_s[6];
static double DOB_state[6];
static double DOB_activation;
static const double K = 400;
//static const double K = 10000.0;
static int is_initialized = 0;
static int step;
static double joint_angle0[6];     // ロボットの最初の状態
static double joint_angle_cmd0[6]; // 理想初期状態
static std::ofstream logfile;


void control_joint(void)
{
    int i;
    double torque_dis[6];

    if (is_initialized == 0)
    {
        control_init();
    }

    // Update command states
    cmd1((double)step*DT);

    // Update joint angles
    robot_get_joint_angle(joint_angle);
    ang_diff(joint_vel, joint_vel_s, joint_angle, 10.0);

    // P controller
    for (i=0; i<6; i++)
    {
        joint_torque[i] = K * (joint_angle_cmd[i] - joint_angle[i]);
    }

    // Compensate disturbance
    dob(torque_dis, 2.0);
    for (i=0; i<6; i++)
    {
        joint_torque[i] += torque_dis[i] * DOB_activation;
    }

    // Output joint torque
    robot_set_joint_torque(joint_torque);

    output_log();

    step++;
}


static void control_init(void)
{
    printk("control_joint: initialization starts.\n");
    step = 0;

    // Initialize first states
    robot_get_joint_angle(joint_angle0);


    /* spatula */
    joint_angle_cmd0[0] = 0;
    joint_angle_cmd0[1] = M_PI/4;
    joint_angle_cmd0[2] = M_PI/3;
    joint_angle_cmd0[3] = M_PI;
    joint_angle_cmd0[4] = -M_PI/2;
    joint_angle_cmd0[5] = 0;


    /* */

    int i;
    for (i=0; i<6; i++)
    {
        joint_vel[i] = 0.0;
        joint_vel_s[i] = joint_angle0[i];
        DOB_state[i] = 0.0;
    }
    DOB_activation = 0.0;

    logfile.open("./CSV/log_joint.csv");
    output_log();

    is_initialized = 1;
    printk("control_joint: initialization finished.\n");
}


static void cmd1(double time)
{
    static const double t_start = 5.0;
    int i;

    if (time < t_start)
    {   // 初期状態にもっていく
        double _t = time / t_start;
        for (i=0; i<6; i++)
        {
            joint_angle_cmd[i] = joint_angle0[i] + (joint_angle_cmd0[i] - joint_angle0[i])*_t;
        }
        DOB_activation = _t;
    }
    else
    {
        DOB_activation = 1.0;
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
        logfile << "," << "j1" << "," << "j2" << "," << "j3" << "," << "j4" << "," << "j5" << "," << "j6";
        logfile << "," << "jc1" << "," << "jc2" << "," << "jc3" << "," << "jc4" << "," << "jc5" << "," << "jc6";
        logfile << "," << "t1" << "," << "t2" << "," << "t3" << "," << "t4" << "," << "t5" << "," << "t6";
        logfile << std::endl;
    }
    else
    {
        /* Display */
        if (step%DISP_INTERVAL == 0)
        {
            std::cout << std::endl;
            v6_output("ang", joint_angle);
            v6_output("cmd", joint_angle_cmd);
            v6_output("trq", joint_torque);
            std::cout << std::endl;
        }
        /* */

        /* Record */
        logfile << step*DT;
        for (i=0; i<6; i++)
        {
            logfile << "," << joint_angle[i];
        }
        for (i=0; i<6; i++)
        {
            logfile << "," << joint_angle_cmd[i];
        }
        for (i=0; i<6; i++)
        {
            logfile << "," << joint_torque[i];
        }
        logfile << std::endl;
        /* */
    }
#endif
}
