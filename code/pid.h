#ifndef _PID_H_
#define _PID_H_
#include "zf_common_headfile.h"

typedef struct
{
    float kp;    // P
    float ki;    // I
    float kd;    // D
    float i_max;
    float p_max;
    float d_max;
    float pid_max;

    float low_pass;

    float out_p;
    float out_i;
    float out_d;

    float error;
    float pre_error;
    float pre_pre_error;

    float output;
    float pre_output;
} pid_param_t;
extern pid_param_t servo_pid;
// extern pid_param_t speed_pid;

// typedef struct
// {
//     float total_encoder;
//     float target_encoder;
//     float encoder_raw;
//     float encoder_speed; // Measured speed
//     float target_speed;
//     int32 duty; // Motor PWM duty

//     enum
//     {
//         MODE_NORMAL,
//         MODE_BANGBANG,
//         MODE_SOFT,
//         MODE_POSLOOP,
//     } motor_mode;

//     pid_param_t pid;       // Motor PID param
//     pid_param_t brake_pid; // Motor PID param
// } motor_param_t;
// extern motor_param_t motor_l, motor_r;

float pid_solve(pid_param_t *pid, float error);

float increment_pid_solve(pid_param_t *pid, float error);

#endif /* _PID_H_ */
