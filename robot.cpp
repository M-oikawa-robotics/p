#include <vector>
#include "robot.h"
#include "kyomath.h"
#include "math.h"
#include <iostream>
#include <iomanip>
#include <fstream>

double link_length[7]  = { 0.13, 0.16, 0.27, 0.04, 0.23, 0.01, 0.08};
double link_inertia[6] = {       0.50, 1.50, 0.80, 0.40, 0.50, 0.01};
double link_weight[7]  = {19.00, 2.00, 2.00, 2.00, 1.00, 1.00, 0.50};

const double torque_max[6] = {0.3178 * 100.0, 0.3178 * 224.0, 0.3178 * 120.0, 0.0952 * 120.0, 0.0952 * 100.0, 0.0952 * 81.482};

static double _joint_angle[6] = {0.0};
static double _LSTM_output[12] = {0.0};
//static double _force_value[6] = {0.0};
static double _joint_torque[6] = {0.0};
static double _force[6] = {0.0};


void robot_get_joint_angle(double joint_angle[6])
{
    int i;
    for (i=0; i<6; i++)
    {
        joint_angle[i] = _joint_angle[i];
    }
}



int robot_set_joint_torque(const double joint_torque[6])
{
    int i;
    for (i=0; i<6; i++)
    {
        _joint_torque[i] = joint_torque[i];
    }

    return 0;
}


void robot_get_transformation(double T[4][4], const double joint_angle[6])
{
    int i, j;

    for (i=0; i<4; i++)
    {
        for (j=0; j<4; j++)
        {
            if (i == j)
            {
                T[i][j] = 1.0;
            }
            else
            {
                T[i][j] = 0.0;
            }
        }
    }

    m4_translate(T,  0.0 , 0.0, link_length[0]); m4_rotate(T, joint_angle[0], 'z');
    m4_translate(T,  0.0 , 0.0, link_length[1]); m4_rotate(T, joint_angle[1], 'y');
    m4_translate(T,  0.0 , 0.0, link_length[2]); m4_rotate(T, joint_angle[2], 'y');
    m4_translate(T, -0.03, 0.0, link_length[3]); m4_rotate(T, joint_angle[3], 'z');
    m4_translate(T,  0.0 , 0.0, link_length[4]); m4_rotate(T, joint_angle[4], 'y');
    m4_translate(T,  0.0 , 0.0, link_length[5]); m4_rotate(T, joint_angle[5], 'z');
    m4_translate(T,  0.0 , 0.0, link_length[6]);
}


void robot_set_joint_angle(const std::vector<double> joint_angle)
{
    for (int i=0; i<6; i++)
    {
        _joint_angle[i] = joint_angle[i];
    }
}

void robot_set_force(const std::vector<double> force)
{
    for (int i=0; i<6; i++)
    {
        _force[i] = force[i] ;

        if(_force[i] > 200.0){
            _force[i] = 200.0;
        }
        else if(_force[i] < -200.0){
            _force[i] = -200.0;
        }
    }
}

void robot_get_force(double force[6])
{
    int i;
    for (i=0; i<6; i++)
    {
        force[i] = _force[i];
        //printf("%f\n",force[i]);
    }
}

void robot_set_LSTM_output(const std::vector<double> LSTM_output)
{
    for (int i=0; i<12; i++)
    {
        _LSTM_output[i] = LSTM_output[i] ;
        // printf("%f\n",_LSTM_output[i]);

    }
}

void robot_get_LSTM_output(double LSTM_output[12])
{
    for (int i=0; i<12; i++)
    {
        LSTM_output[i] = _LSTM_output[i];
      //  printf("%f\n",_LSTM_output[i]);

    }
}


std::vector<double> robot_get_joint_torque(void)
{
    std::vector<double> joint_torque(6);

    for (int i=0; i<6; i++)
    {
        joint_torque[i] = _joint_torque[i];
    }

    return joint_torque;
}


#if 0
void robot_get_transformation(double T[4][4], const double joint_angle[6])
{
    int i, j;

    for (i=0; i<4; i++)
    {
        for (j=0; j<4; j++)
        {
            if (i == j)
            {
                T[i][j] = 1.0;
            }
            else
            {
                T[i][j] = 0.0;
            }
        }
    }

    m4_translate(T,  0.0 , 0.0, link_length[0]); m4_rotate(T, joint_angle[0], 'z');
    m4_translate(T,  0.0 , 0.0, link_length[1]); m4_rotate(T, joint_angle[1], 'y');
    m4_translate(T,  0.0 , 0.0, link_length[2]); m4_rotate(T, joint_angle[2], 'y');
    m4_translate(T, -0.03, 0.0, link_length[3]); m4_rotate(T, joint_angle[3], 'z');
    m4_translate(T,  0.0 , 0.0, link_length[4]); m4_rotate(T, joint_angle[4], 'y');
    m4_translate(T,  0.0 , 0.0, link_length[5]); m4_rotate(T, joint_angle[5], 'z');
    m4_translate(T,  0.0 , 0.0, link_length[6]);
}
#endif
