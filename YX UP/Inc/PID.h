#ifndef __PID_H
#define __PID_H
#include "main.h"
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
							
float pid_calc(pid_struct_t *pid, float ref, float fdb);	
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);	
void set_motor_voltage_can_2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void set_motor_voltage_6020_can_1(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
float pid_yaw_calc(pid_struct_t *pid, float fdb,float limit_speed);
					
#endif
							