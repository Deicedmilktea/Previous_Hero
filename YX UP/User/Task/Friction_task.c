#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "Friction_task.h"
#include "Exchange_task.h"

//�������дĦ���ֺͲ���
//Ħ���ֵ�ID�ֱ���2��3
//���̵�ID��5

int16_t bopan = -10*19;

//PID��ʼ��
static void Friction_init();

//Ħ���ּ��ٸ�ֵ
static void Friction_calc();

//Ħ�����ٶ�����
static void Friction_limit();

//Ħ���ּ��ٸ�ֵ
static void Friction_down();

//Ħ���ֳ����ж�
static bool Friction_judeg();

//Ħ����Pid���ֵ����
static void Friction_send();

//����Pid���ֵ����ͷ���
static void Bopan_send(int16_t speed);

void Friction_task(void const * argument)
{
  /* USER CODE BEGIN StartTask06 */
  /* Infinite loop */
	
	Friction_init();
  for(;;)
  {
		Friction_calc();//Ħ����һֱת
		Friction_limit();
		if(rc_ctrl.rc.s[1] == 1 || press_left)
		{	
			Bopan_send(bopan);
		}
		else
		{			
			Bopan_send(0);
//			Friction_down();			
		}
		Friction_send();


		
//		if(Friction_judeg())
//		{
//			Bopan_send(bopan);
//		}
//		else
//		{
//			Bopan_send(0);
//		}
    osDelay(1);
  }
  /* USER CODE END StartTask06 */
}

static void Friction_init()
{
	pid_init(&motor_pid_can_2[2],40,0.8,1,16384,16384);
	pid_init(&motor_pid_can_2[1],40,0.8,1,16384,16384);
	pid_init(&motor_pid[4],30,0.1,5,16384,16384);
}


static void Friction_calc()
{
	target_speed_can_2[1]=19*240;//258
	target_speed_can_2[2]=-19*240;
	
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[1].rotor_speed);
	motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed);
}


static void Friction_down()
{
	target_speed_can_2[1]=0;
	target_speed_can_2[2]=0;
	motor_info_can_2[1].set_voltage = pid_calc(&motor_pid_can_2[1], target_speed_can_2[1], motor_info_can_2[1].rotor_speed);
	motor_info_can_2[2].set_voltage = pid_calc(&motor_pid_can_2[2], target_speed_can_2[2], motor_info_can_2[2].rotor_speed);
}

static bool Friction_judeg()
{
	if(	(motor_info_can_2[1].rotor_speed>=8500) && (motor_info_can_2[2].rotor_speed<=-8500) )
	{
		return true;
	}
	return false;
}

static void Friction_send()
{
		set_motor_voltage_can_2(0, 
                      motor_info_can_2[0].set_voltage, 
                      motor_info_can_2[1].set_voltage, 
                      motor_info_can_2[2].set_voltage, 
                      motor_info_can_2[3].set_voltage);
}

static void Bopan_send(int16_t speed)
{
		motor_info[4].set_voltage=pid_calc(&motor_pid[4],speed,motor_info[4].rotor_speed);
		set_motor_voltage(1, 
                      motor_info[4].set_voltage, 
                      motor_info[5].set_voltage, 
                      motor_info[6].set_voltage, 
                      0);
}

static void Friction_limit()
{

}