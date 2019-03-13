#include <Python.h>

int initialize_NN(void);
int finalize_NN(void);
int input_state(const double state[8]);
int receive_action(double action[2]);
int quit_generate(void);
int is_end_of_motion(int *eom);
