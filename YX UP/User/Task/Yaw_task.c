#include "Yaw_task.h"
#include "Exchange_task.h"

//	˼·�������ٶȻ�������PID��������̨������̨�����ǵ�yawֵ�������ǶȻ����в���
//  ע�⣺��̨����ת�Ǹ���
//  ��һ�汾�����ң�������Ƶİ汾
//  YAW����6020�����������C��CAN_1�����IDΪ6

//	����һЩȫ�ֱ���
extern int16_t mouse_x;
extern ins_data_t ins_data;
int8_t yaw_choice_flag = 0;
int8_t yaw_mode = 1;
	
fp32 ins_yaw;
fp32 ins_yaw_update = 0;
fp32 Driftring_yaw = 0;
fp32 ins_pitch;
fp32 ins_row;
fp32 init_yaw;	//��סinit_yaw��ֵ
int yaw_model_flag = 1;
fp32 err_yaw;		//��ס�ĽǶ�
fp32 angle_weight = 1;	//�ǶȻ�->�ٶȻ���ӳ���Ȩ��
pid_struct_t angle_pid;
fp32 ins_gyro;
fp32 exchangeagle = 57.3f;

int Yaw_count = 0;

fp32 target_yaw_angle = 0;
fp32 target_angletemp = 0;
//ǰ�����Ʊ���
int16_t Rotate_w;
int16_t Rotate_W;

#define Rotate_gain 1.38f
#define Chassis_R	30.0f
#define Chassis_r 7.5f//7.5f

#define valve 50		//��ֵ
#define base 1024		//ң�����Ļ���ֵ
#define base_max 1684		
#define base_min 364
#define angle_valve 1		//�Ƕ���ֵ���������Χ�ھͲ�ȥ������
#define mouse_x_valve 10
#define mouse_x_weight 0.5f
#define mouse_x_angle_weight 0.05f
#define Yaw_minipc_valve 1
#define Yaw_minipc_weight 1.0f

//У��Ư�Ʊ�־λ
int8_t Update_yaw_flag = 0;

//����һЩ����

//��ʼ��PID����
static void Yaw_init();	

//ÿ��ѭ����ʼ��
static void Yaw_loop_init();

//��ȡimu����
static void Yaw_read_imu();

//ģʽѡ��
static void Yaw_choice();

//����������̨�����У�
static void Yaw_fix();
static void Yaw_angle_fix();

//Mode_1�µĿ����㷨
static void Yaw_mode_1();
static void Yaw_angle_mode_1();

//Mode_2�µĿ����㷨
static void Yaw_mode_2();
static void Yaw_angle_mode_2();
//ǰ������
static void Yaw_Rotate();

//�����Ƶ���
static void Yaw_mouse();

//PID����ͷ���
static void Yaw_can_send();

//�����Ӿ�����
static void Yaw_minipc_control();

//�Ӿ�����
static void Yaw_minipc_zero();
static void Yaw_control(float angle);

void Yaw_task(void const *pvParameters)
{
  //��ʼ������

	Yaw_init();
	
	//ѭ����������
  for(;;)
  {
		Yaw_loop_init();
		Yaw_read_imu();

		//ģʽ�ж�,���Ͻǿ��ؿ������·�
		if(rc_ctrl.rc.s[1] == 2 && ins_yaw)
		{
			Yaw_choice();
			if(yaw_mode == 1)
			{
				Yaw_Rotate();
				Yaw_mode_1();
			}
		else if(yaw_mode == 2)
			{
				Yaw_Rotate();
				Yaw_mode_2();
			}
		}
		
		else if(rc_ctrl.rc.s[1] == 1 || rc_ctrl.rc.s[1] == 3)
		{
			Yaw_Rotate();
			Yaw_mode_2();
		}
		motor_info[5].set_voltage = pid_calc(&motor_pid[5], target_speed[5], motor_info[5].rotor_speed);
		Yaw_can_send();
    osDelay(1);
  }

}
void Yaw_angle_task(void const *pvParameters)
{
  //��ʼ������
	
	Yaw_init();
	
	//ѭ����������
  for(;;)
  {
		Yaw_loop_init();
		Yaw_read_imu();

		//ģʽ�ж�,���Ͻǿ��ؿ������·�
		if(rc_ctrl.rc.s[1] == 2 && ins_yaw)
		{
			Yaw_choice();
			if(yaw_mode == 1)
			{
				Yaw_Rotate();
				Yaw_angle_mode_1();
			}
		else if(yaw_mode == 2)
			{
				Yaw_Rotate();
				Yaw_angle_mode_2();
			}
		}
		
		else if(rc_ctrl.rc.s[1] == 1 || rc_ctrl.rc.s[1] == 3)
		{
			Yaw_Rotate();
			Yaw_angle_mode_2();
		}
		Yaw_control(target_yaw_angle);
		Yaw_can_send();
    osDelay(1);
  }

}
static void Yaw_control(float angle)
{
	int controlflag = 0;
	for(controlflag = 0;controlflag < 3;controlflag++)
	{
		target_angletemp = pid_calc(&angle_pid, angle, ins_yaw);
	}
	motor_info[5].set_voltage = pid_calc(&motor_pid[5], target_angletemp, ins_gyro);
}

//��ʼ��PID����
static void Yaw_init()	
{
	//idΪcan1��5��
	pid_init(&motor_pid[5],85,7,5,15000,15000);//85,7,5//110,1,40,15000,15000
	pid_init(&angle_pid,1,0,0,15,15);//85,7,5//110,1,40,15000,15000
}


//ѭ����ʼ��
static void Yaw_loop_init()
{
	target_speed[5]=0;
}


//��ȡimu����
static void Yaw_read_imu()
{
		//�����Ƕ�ֵ��ȡ
		ins_yaw = ins_data.angle[0];
		ins_pitch = ins_data.angle[1];
		ins_row = ins_data.angle[2]*exchangeagle;
		ins_gyro = ins_data.gyro[0];
	
		//��ȡƯ��ֵ
		if(Update_yaw_flag)
		{
			Update_yaw_flag = 0;//�����־λ
			Driftring_yaw = ins_yaw - 0.0f;
		}
		
		//У��
		ins_yaw_update = ins_yaw - Driftring_yaw;
}

static void Yaw_can_send()
{
	CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
	
	tx_header.StdId = 0x2ff;
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (motor_info[5].set_voltage>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[1] = (motor_info[5].set_voltage)&0xff;
  tx_data[2] = 0x00;
  tx_data[3] = 0x00;
  tx_data[4] = 0x00;
  tx_data[5] = 0x00;
  tx_data[6] = 0x00;
  tx_data[7] = 0x00;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);

}

static void Yaw_Rotate()
{
	Rotate_W = (Rotate_gain * Rotate_w * Chassis_r) / Chassis_R;
	target_speed[5]-=Rotate_W;
}
//����������̨
static void Yaw_fix()
{
	//ң�л�������̨(һ�������)
					if(yaw_model_flag == 1)		//�ƶ���̨�����¼�ס��̨�ĳ�ʼλ�õ�ֵ
					{
						init_yaw = ins_yaw_update;
						yaw_model_flag = 0;
					}
						err_yaw = ins_yaw_update - init_yaw;		//��ʵʱ���ݼ���ʼ����
					
					//Խ�紦��,��֤ת�����򲻱�
					if(err_yaw < -180)	//	Խ��ʱ��180 -> -180
					{
						err_yaw += 360;
					}
					
					else if(err_yaw > 180)	//	Խ��ʱ��-180 -> 180
					{
						err_yaw -= 360;
					}
				
					
					//��ֵ�ж�
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_speed[5] += err_yaw * angle_weight;
					}
					
					else
					{
						target_speed[5] = 0;
					}
				
}
//����������̨
static void Yaw_angle_fix()
{
	//ң�л�������̨(һ�������)
					if(yaw_model_flag == 1)		//�ƶ���̨�����¼�ס��̨�ĳ�ʼλ�õ�ֵ
					{
						init_yaw = ins_yaw_update;
						yaw_model_flag = 0;
					}
						err_yaw = ins_yaw_update - init_yaw;		//��ʵʱ���ݼ���ʼ����
								
					

				
					
					//��ֵ�ж�
					if(err_yaw > angle_valve || err_yaw < -angle_valve)
					{
						target_yaw_angle = init_yaw;
					}
					
					else
					{
						target_speed[5] = 0;
					}
				
}
static void Yaw_mouse()
{
	if(mouse_x > mouse_x_valve || mouse_x < -mouse_x_valve)
	{
		yaw_model_flag = 1;
		target_speed[5] += (fp32)mouse_x * mouse_x_weight;
	}
}

static void Yaw_angle_mouse()
{
	if(mouse_x > mouse_x_valve || mouse_x < -mouse_x_valve)
	{
		yaw_model_flag = 1;
		target_yaw_angle += (fp32)mouse_x * mouse_x_angle_weight;
		
	}
}

static void Yaw_mode_1()
{
				
				if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
				{
					Yaw_fix();
				}
				//�����е�ʱ������ƶ���̨
				else if( (rc_ctrl.rc.ch[0] >= base+valve && rc_ctrl.rc.ch[0] <= base_max) || (e_flag))
				{
					target_speed[5] += 60;
					yaw_model_flag = 1;
				}
				else if( (rc_ctrl.rc.ch[0] >= base_min && rc_ctrl.rc.ch[0]<base - valve ) || (q_flag))
				{
					target_speed[5] -= 60;
					yaw_model_flag = 1;
				}

				
				Yaw_mouse();
				//�Ҽ���������
				if(press_right)
				{
					Yaw_minipc_control();
				}
											

}
static void Yaw_angle_mode_1()
{
				
				if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
				{
					Yaw_angle_fix();
				}
				//�����е�ʱ������ƶ���̨
				else if( (rc_ctrl.rc.ch[0] >= base+valve && rc_ctrl.rc.ch[0] <= base_max) || (e_flag))
				{
					target_yaw_angle += 5;
					yaw_model_flag = 1;
				}
				else if( (rc_ctrl.rc.ch[0] >= base_min && rc_ctrl.rc.ch[0]<base - valve ) || (q_flag))
				{
					target_yaw_angle -= 5;
					yaw_model_flag = 1;
				}

				
				Yaw_angle_mouse();
				//�Ҽ���������
				if(press_right)
				{
					Yaw_minipc_control();
				}
											

}
static void Yaw_mode_2()
{
	
		if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
	{
		Yaw_fix();
	}
	else if( (rc_ctrl.rc.ch[0] >= base+valve && rc_ctrl.rc.ch[0] <= base_max) || (e_flag) )
	{
		target_speed[5] += 60;
		yaw_model_flag = 1;
	}
	
	else if( (rc_ctrl.rc.ch[0] >= base_min && rc_ctrl.rc.ch[0]<base - valve ) || (q_flag) )
	{
		target_speed[5] -= 60;
		yaw_model_flag = 1;
	}
	Yaw_mouse();
					//�Ҽ���������
	if(press_right)
	{
		Yaw_minipc_control();
	}


}
static void Yaw_angle_mode_2()
{
	
		if(rc_ctrl.rc.ch[0] > base-valve && rc_ctrl.rc.ch[0] < base+valve && (!q_flag) && (!e_flag) && (mouse_x < mouse_x_valve) && (mouse_x > -mouse_x_valve) && (!press_right))
	{
		Yaw_angle_fix();
	}
	else if( (rc_ctrl.rc.ch[0] >= base+valve && rc_ctrl.rc.ch[0] <= base_max) || (e_flag) )
	{
		target_yaw_angle += 5;
		yaw_model_flag = 1;
	}
	
	else if( (rc_ctrl.rc.ch[0] >= base_min && rc_ctrl.rc.ch[0]<base - valve ) || (q_flag) )
	{
		target_yaw_angle += 5;
		yaw_model_flag = 1;
	}
	Yaw_angle_mouse();
					//�Ҽ���������
	if(press_right)
	{
		Yaw_minipc_control();
	}


}

//�½��Ӵ���
static void Yaw_choice()
{
	if(r_flag)
	{
		yaw_choice_flag = 1;
	}
	
	if( (!r_flag) && (yaw_choice_flag == 1) )	
	{
		yaw_choice_flag = 0;
		if(yaw_mode == 1)
		{
			yaw_mode = 2;
		}
		else if(yaw_mode == 2)
		{
			yaw_mode = 1;
		}
	}
}

static void Yaw_minipc_control()
{
	if(Yaw_minipc > Yaw_minipc_valve || Yaw_minipc < -Yaw_minipc_valve)
	{
		yaw_model_flag = 1;
		target_speed[5] += ((fp32)Yaw_minipc) * Yaw_minipc_weight;
	}
}

static void Yaw_minipc_zero()
{
	
}