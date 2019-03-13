#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "control.h"
#include "robot.h"

extern void robot_set_joint_angle(const std::vector<double>);
extern void robot_set_force(const std::vector<double> force);
extern std::vector<double> robot_get_joint_torque(void);
extern void set_object_state(const std::vector<double> xs, const std::vector<double> xo);
extern void robot_set_LSTM_output(const std::vector<double> LSTM_output);

PYBIND11_MODULE(mymod, m) {
    m.doc() = "Control MOTOMAN-MH3F";
    m.def("control_joint", &control_joint, "control joint.");
    m.def("control_turnover", &control_turnover, "control spatula for turning over motion.");
    m.def("control_sliding", &control_sliding, "control spatula for sliding motion.");
    m.def("set_force", &robot_set_force, "set force.");
    m.def("set_joint_angle", &robot_set_joint_angle, "set joint angle.");
    m.def("get_joint_torque", &robot_get_joint_torque, "get joint torque.");
    m.def("set_object_state", &set_object_state, "set state of spatula and pancake.");
    //m.def("set_LSTM_output", &robot_set_LSTM_output, "robot_set_LSTM_output");
    //m.def("get_LSTM_output", &robot_get_LSTM_output, "robot_get_LSTM_output");
}
