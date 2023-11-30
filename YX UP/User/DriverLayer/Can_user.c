#include "Can_user.h"
#include "remote_control.h"

//	Can 的一些用户撰写的接收函数
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern int16_t Rotate_w;
extern int16_t Down_pitch;
extern int8_t Update_yaw_flag;
extern fp32 init_yaw;

void can_1_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 0;                       // filter 0
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;//标识符寄存器 
  can_filter.FilterIdLow  = 0;//标识符寄存器 
  can_filter.FilterMaskIdHigh = 0;//屏蔽寄存器
  can_filter.FilterMaskIdLow  = 0;       //屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
	HAL_CAN_Start(&hcan1);//启动can，封装在can_user_init()里了
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);		//使能can的FIFO0中断，也封装在can_user_init()里了
}

void can_2_user_init(CAN_HandleTypeDef* hcan )
{
  CAN_FilterTypeDef  can_filter;

  can_filter.FilterBank = 14;                       // filter 14
  can_filter.FilterMode =  CAN_FILTERMODE_IDMASK;  // 标识符屏蔽位模式
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;		//过滤器位宽为单个32位
  can_filter.FilterIdHigh = 0;//标识符寄存器 
  can_filter.FilterIdLow  = 0;//标识符寄存器 
  can_filter.FilterMaskIdHigh = 0;//屏蔽寄存器
  can_filter.FilterMaskIdLow  = 0;       //屏蔽寄存器   set mask 0 to receive all can id
  can_filter.FilterFIFOAssignment = CAN_RX_FIFO0; // assign to fifo0，接收器是FIFO0
  can_filter.FilterActivation = ENABLE;           // enable can filter
  can_filter.SlaveStartFilterBank  = 14;          // only meaningful in dual can mode
   
  HAL_CAN_ConfigFilter(hcan, &can_filter);        // init can filter
	HAL_CAN_Start(&hcan2);//启动can，封装在can_user_init()里了
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);		//使能can的FIFO0中断，也封装在can_user_init()里了
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//接受中断回调函数
{
  CAN_RxHeaderTypeDef rx_header;
  if(hcan->Instance == CAN1)
  {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can1 data
		if(rx_header.StdId==0x33)//双C板传递遥控器信号的接口标识符
		{
			can_cnt_2++;
			if (can_cnt_2 == 100)//闪烁蓝灯代表遥控接收正常通信
			{
				can_cnt_2 = 0;
				HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
			}
			rc_ctrl.rc.ch[0] = (rx_data[0] | (rx_data[1] << 8)) & 0x07ff;        //!< Channel 0  中值为1024，最大值1684，最小值364，波动范围：660
			rc_ctrl.rc.ch[1] = (((rx_data[1] >> 3)&0xff) | (rx_data[2] << 5)) & 0x07ff; //!< Channel 1
			rc_ctrl.rc.ch[2] = (((rx_data[2] >> 6)&0xff) | (rx_data[3] << 2) |          //!< Channel 2
                         (rx_data[4] << 10)) &0x07ff;
			rc_ctrl.rc.ch[3] = (((rx_data[4] >> 1)&0xff) | (rx_data[5] << 7)) & 0x07ff; //!< Channel 3
			rc_ctrl.rc.s[0] = ((rx_data[5] >> 4) & 0x0003);                  //!< Switch left！！！这尼玛是右
			rc_ctrl.rc.s[1] = ((rx_data[5] >> 4) & 0x000C) >> 2;    		//!< Switch right！！！这才是左
			rc_ctrl.mouse.x = rx_data[6] | (rx_data[7] << 8);                    //!< Mouse X axis
			}
		if(rx_header.StdId==0x34)//双C板传递遥控器信号的接口标识符
		{
			rc_ctrl.mouse.y = rx_data[0] | (rx_data[1] << 8);                    //!< Mouse Y axis
			rc_ctrl.mouse.z = rx_data[2] | (rx_data[3] << 8);                  //!< Mouse Z axis
			rc_ctrl.mouse.press_l = rx_data[4];                                  //!< Mouse Left Is Press ?
			rc_ctrl.mouse.press_r = rx_data[5];                                  //!< Mouse Right Is Press ?
			rc_ctrl.key.v = rx_data[6] | (rx_data[7] << 8); 
/*			//!< KeyBoard value	
    rc_ctrl.rc.ch[4] = rx_data[16] | (rx_data[17] << 8);                 //NULL
			*/	}
		if(rx_header.StdId==0x35)//双C板传递遥控器信号的接口标识符
		{
    rc_ctrl.rc.ch[4] = rx_data[0] | (rx_data[1] << 8);                 //NULL
			
		//接收底盘旋转量
			Rotate_w = (rx_data[3] << 8) | rx_data[4];
			
		//接收底盘imu的Pitch数据
			Down_pitch = (rx_data[5] << 8) | rx_data[6];
			
		}
		
		//YAW校正接收
		if(rx_header.StdId==0x66)
		{
			if(rx_data[0] == 0xff)
			{
				Update_yaw_flag = 1;
				HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);
				init_yaw = 0;//回正锁云台的值
			}
		}
		
		
		if ((rx_header.StdId >= 0x201)//201-207
   && (rx_header.StdId <  0x208))                  // 判断标识符，标识符为0x200+ID
  {
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
    motor_info[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[index].temp           =   rx_data[6];
		if(index==0)
		{can_cnt_1 ++;}
  }
	
	//YAW
	else if(rx_header.StdId == 0x209)
	{
		motor_info[5].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info[5].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info[5].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info[5].temp           =   rx_data[6];
	}

	
  }
	 if(hcan->Instance == CAN2)
  {		uint8_t             rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data); //receive can2 data
		if ((rx_header.StdId >= FEEDBACK_ID_BASE)//201-207
   && (rx_header.StdId <  FEEDBACK_ID_BASE + MOTOR_MAX_NUM))                  // 判断标识符，标识符为0x200+ID
  {
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE;                  // get motor index by can_id
    motor_info_can_2[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info_can_2[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info_can_2[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info_can_2[index].temp           =   rx_data[6];

  }
	  if ((rx_header.StdId >= FEEDBACK_ID_BASE_6020)//205-211,注意把ID调成大于3,不然就会和读取3508的函数产生冲突
   && (rx_header.StdId <  FEEDBACK_ID_BASE_6020 + MOTOR_MAX_NUM))                  // 判断标识符，标识符为0x204+ID
  {
    uint8_t index = rx_header.StdId - FEEDBACK_ID_BASE_6020;                  // get motor index by can_id
    motor_info_can_2[index].rotor_angle    = ((rx_data[0] << 8) | rx_data[1]);
    motor_info_can_2[index].rotor_speed    = ((rx_data[2] << 8) | rx_data[3]);
    motor_info_can_2[index].torque_current = ((rx_data[4] << 8) | rx_data[5]);
    motor_info_can_2[index].temp           =   rx_data[6];

  }
  }

  if (can_cnt_1 == 500)//闪烁红灯代表can正常通信
  {
    can_cnt_1 = 0;
		HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_12);

  }
}

void can_remote(uint8_t sbus_buf[],uint32_t id)//调用can来发送遥控器数据
{
  CAN_TxHeaderTypeDef tx_header;  
  tx_header.StdId = id;
  tx_header.IDE   = CAN_ID_STD;//标准帧
  tx_header.RTR   = CAN_RTR_DATA;//数据帧
  tx_header.DLC   = 8;		//发送数据长度（字节）

  HAL_CAN_AddTxMessage(&hcan1, &tx_header, sbus_buf,(uint32_t*)CAN_TX_MAILBOX0);
}
