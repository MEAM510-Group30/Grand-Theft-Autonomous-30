// In this file, we define functions for PID control
// PID control class would be a template for both position and velocity control
// The control functions need to accept the reference signal, the actual signal, and also PID parameters as arguments
// PID global variables such as the error, integral, and derivative terms should be defined and stored in the PID class
// So in the main ino file, we need to first define and initialize the PID class, then call the control functions in the loop


#include "control.h"

