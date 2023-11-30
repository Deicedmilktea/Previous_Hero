#include "PID.h"

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}

float pid_calc(pid_struct_t *pid, float ref, float fdb)//ref是目标值,fdb是电机解码的速度返回值
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];//err[1]是上一次计算出来的差值
  pid->err[0] = pid->ref - pid->fdb;//err[0]是这一次的预期速度和实际速度的差值,这两个值是可以是负数的
  
  pid->p_out  = pid->kp * pid->err[0];//40 3 0是标准值，把这个加到watch1里面
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//防止越界
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);//防止越界
  return pid->output;//电机返回的报文有转速和转矩电流，但是只能发电压值(-30000至30000)，有点抽象这个PID
}

void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x1ff);//如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = (v1>>8)&0xff;	//先发高八位		
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}


void set_motor_voltage_can_2(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x1ff);//如果id_range==0则等于0x1ff,id_range==1则等于0x2ff（ID号）
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

	tx_data[0] = (v1>>8)&0xff;	//先发高八位		
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}



float pid_yaw_calc(pid_struct_t *pid,float fdb,float limit_speed)//fdb是电机解码的速度返回值
{
	/*可以使用双环PID进行控制，外环是角度控制，内环是角速度控制*/
	//先写外环
	if(target_angle<=8191 && target_angle>0)//越界停止
	{
		if(motor_info[5].rotor_angle<=target_angle)
		{
		err_angle=target_angle-motor_info[5].rotor_angle;//计算出差值
		}
		else if(motor_info[5].rotor_angle>target_angle)
		{
		err_angle=-(motor_info[5].rotor_angle-target_angle);
		}
		else
		{
			target_angle=motor_info[5].rotor_angle;//赋予边界值
			err_angle=0;//越界清零停止，理论上不能大于一周圈，之后若是要取消这个限制，要想办法处理越界的情况
		}
	}
	small=err_angle/angle_limit;
	//再写内环，外环的输出值作为内环的输入值
	pid->ref = limit_speed * small;//根据角度的差值大小来确定速度大小
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];//err[1]是上一次计算出来的差值
  pid->err[0] = pid->ref - pid->fdb;//err[0]是这一次的预期速度和实际速度的差值,这两个值是可以是负数的
  
  pid->p_out  = pid->kp * pid->err[0];//40 3 0是标准值，把这个加到watch1里面
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//防止越界
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);//防止越界
  return pid->output;//电机返回的报文有转速和转矩电流，但是只能发电压值(-30000至30000)，有点抽象这个PID
	
}