#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "PID.h"
#include "arm_math.h"
#include "can.h"
#include "INS_task.h"
#include "Pitch_task.h"
#include "Exchange_task.h"


//此任务用来对云台进行模式选择，控制，校准等
//封装一些函数用来进行控制调用
//Pitch采用上C板CAN_2，电机ID为5
//存在问题：需要将云台Pitch锁在最下方再启动，Pitch受高频噪点影响，收敛速度太慢

//定义一些变量
//限位参数（机械测量）
#define Up_inf 35
#define Down_inf 0
#define mouse_y_valve 10
#define mouse_y_weight 24.0f
#define Pitch_minipc_valve 1
#define Pitch_minipc_weight	150.0f

//imu数据
fp32 Err_pitch;
int16_t Up_pitch;
int16_t Down_pitch;
uint16_t Remember_pitch = 0;
uint8_t Remember_pitch_flag = 1;
extern ins_data_t ins_data;
extern int16_t mouse_y;

//初始化PID参数
static void gimbal_init();	

//校验连接成功
static bool gimbal_judge();	

//读取imu参数
static void gimbal_read_imu();

//模式选择
static void gimbal_choice();

//Mode_1下的控制算法
static void gimbal_mode_1();

//PID计算和发送
static void gimbal_can_send();

//限位（相对陀螺仪数据）
static void gimbal_imu_limit();

//反转限位
static void gimbal_imu_limit_2();

//鼠标控制Pitch(叠加)
static void gimbal_mouse();

//叠加自瞄
static void gimbal_minipc_control();
// pitch

void Pitch_task(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */

	gimbal_init();	
	
  for(;;)
  {
		if(rc_ctrl.rc.s[1]==3 || rc_ctrl.rc.s[1]==1	|| rc_ctrl.rc.s[1]==2)
		{
			gimbal_mode_1();
			gimbal_mouse();
			
			//右键启动自瞄
	  	if(press_right)
		 {
		  	gimbal_minipc_control();
		 }
		  gimbal_imu_limit();			
		}	
		gimbal_can_send();
		
    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

//初始化PID参数
static void gimbal_init()	
{
		pid_init(&motor_pid_can_2[4],30,0.1,5,10000,10000);
}


//校验连接成功
static bool gimbal_judge()
{

}


//读取imu参数
static void gimbal_read_imu()
{
	//获取Down_pitch(写在了can的接收函数里面)
	
	//上-下
	Up_pitch = (int)ins_data.angle[1];
	Err_pitch = Down_pitch - Up_pitch ;//下-上为正值
	Err_pitch += Remember_pitch;
}


//模式选择
static void gimbal_choice()
{

}


//Mode_1算法，最简单的云台控制（速度环）
static void gimbal_mode_1()
{
		if( rc_ctrl.rc.ch[1]>1074&&rc_ctrl.rc.ch[1]<=1684  )
		{
			target_speed_can_2[4]=3000;//1250
		}
		else if(rc_ctrl.rc.ch[1]>=324&&rc_ctrl.rc.ch[1]<974)
		{
			target_speed_can_2[4]=-3000;//1250
		}
		else
		{
			target_speed_can_2[4]=0;
		}
}	

//PID计算和发送
static void gimbal_can_send()
{
		
    motor_info_can_2[4].set_voltage = pid_calc(&motor_pid_can_2[4], target_speed_can_2[4], motor_info_can_2[4].rotor_speed);//????PID???
		
		set_motor_voltage_can_2(1, 
                      motor_info_can_2[4].set_voltage, 
                      motor_info_can_2[5].set_voltage, 
                      motor_info_can_2[6].set_voltage, 
                      0);
}


//陀螺仪限位（相对）,停止限位
static void gimbal_imu_limit()
{
	gimbal_read_imu();
	if( (Err_pitch > Up_inf) && ((rc_ctrl.rc.ch[1]>1074&&rc_ctrl.rc.ch[1]<=1684 ) || (ctrl_flag) || (mouse_y > mouse_y_valve)) )
	{
		target_speed_can_2[4]=0;
	}
	
	else if( (Err_pitch < Down_inf) && ( (rc_ctrl.rc.ch[1]>=324&&rc_ctrl.rc.ch[1]<974) || (shift_flag) || (mouse_y < -mouse_y_valve)) )
	{
		target_speed_can_2[4]=0;
	}
}

//反转限位法
static void gimbal_imu_limit_2()
{
	gimbal_read_imu();
	if( (Err_pitch > Up_inf) && ((rc_ctrl.rc.ch[1]>1074&&rc_ctrl.rc.ch[1]<=1684 ) || (ctrl_flag)))
	{
		target_speed_can_2[4]=-400;
	}
	
	else if( (Err_pitch < Down_inf) && ( (rc_ctrl.rc.ch[1]>=324&&rc_ctrl.rc.ch[1]<974) || (shift_flag)) )
	{
		target_speed_can_2[4]=400;
	}
}

//鼠标控制
static void gimbal_mouse()
{
	if(mouse_y > mouse_y_valve || mouse_y < -mouse_y_valve)
	{
		target_speed_can_2[4] += (fp32)mouse_y * mouse_y_weight;
	}
}

//自瞄
static void gimbal_minipc_control()
{
	if(Pitch_minipc > Pitch_minipc_valve || Pitch_minipc < -Pitch_minipc_valve)
	{
		target_speed_can_2[4] -= ((fp32)Pitch_minipc) * Pitch_minipc_weight;
	}
}


