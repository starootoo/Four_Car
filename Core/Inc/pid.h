#ifndef __PID_H__
#define __PID_H__





typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
    float target_value;
} pid_t;

void pid_init(pid_t *pid, float Kp, float Ki, float Kd, float target_value, float out_min, float out_max);
float pid_calc(pid_t *pid,  float measured);
void pid_con_speed(pid_t *pid_left,pid_t *pid_right);







#endif /* __PID_H__ */