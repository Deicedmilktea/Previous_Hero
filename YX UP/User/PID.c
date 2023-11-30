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

float pid_calc(pid_struct_t *pid, float ref, float fdb)//ref��Ŀ��ֵ,fdb�ǵ��������ٶȷ���ֵ
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];//err[1]����һ�μ�������Ĳ�ֵ
  pid->err[0] = pid->ref - pid->fdb;//err[0]����һ�ε�Ԥ���ٶȺ�ʵ���ٶȵĲ�ֵ,������ֵ�ǿ����Ǹ�����
  
  pid->p_out  = pid->kp * pid->err[0];//40 3 0�Ǳ�׼ֵ��������ӵ�watch1����
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//��ֹԽ��
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);//��ֹԽ��
  return pid->output;//������صı�����ת�ٺ�ת�ص���������ֻ�ܷ���ѹֵ(-30000��30000)���е�������PID
}

void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x1ff);//���id_range==0�����0x1ff,id_range==1�����0x2ff��ID�ţ�
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (v1>>8)&0xff;	//�ȷ��߰�λ		
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
    
  tx_header.StdId = (id_range == 0)?(0x200):(0x1ff);//���id_range==0�����0x1ff,id_range==1�����0x2ff��ID�ţ�
  tx_header.IDE   = CAN_ID_STD;//��׼֡
  tx_header.RTR   = CAN_RTR_DATA;//����֡
  tx_header.DLC   = 8;		//�������ݳ��ȣ��ֽڣ�

	tx_data[0] = (v1>>8)&0xff;	//�ȷ��߰�λ		
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0);
}



float pid_yaw_calc(pid_struct_t *pid,float fdb,float limit_speed)//fdb�ǵ��������ٶȷ���ֵ
{
	/*����ʹ��˫��PID���п��ƣ��⻷�ǽǶȿ��ƣ��ڻ��ǽ��ٶȿ���*/
	//��д�⻷
	if(target_angle<=8191 && target_angle>0)//Խ��ֹͣ
	{
		if(motor_info[5].rotor_angle<=target_angle)
		{
		err_angle=target_angle-motor_info[5].rotor_angle;//�������ֵ
		}
		else if(motor_info[5].rotor_angle>target_angle)
		{
		err_angle=-(motor_info[5].rotor_angle-target_angle);
		}
		else
		{
			target_angle=motor_info[5].rotor_angle;//����߽�ֵ
			err_angle=0;//Խ������ֹͣ�������ϲ��ܴ���һ��Ȧ��֮������Ҫȡ��������ƣ�Ҫ��취����Խ������
		}
	}
	small=err_angle/angle_limit;
	//��д�ڻ����⻷�����ֵ��Ϊ�ڻ�������ֵ
	pid->ref = limit_speed * small;//���ݽǶȵĲ�ֵ��С��ȷ���ٶȴ�С
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];//err[1]����һ�μ�������Ĳ�ֵ
  pid->err[0] = pid->ref - pid->fdb;//err[0]����һ�ε�Ԥ���ٶȺ�ʵ���ٶȵĲ�ֵ,������ֵ�ǿ����Ǹ�����
  
  pid->p_out  = pid->kp * pid->err[0];//40 3 0�Ǳ�׼ֵ��������ӵ�watch1����
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//��ֹԽ��
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(pid->output, -pid->out_max, pid->out_max);//��ֹԽ��
  return pid->output;//������صı�����ת�ٺ�ת�ص���������ֻ�ܷ���ѹֵ(-30000��30000)���е�������PID
	
}