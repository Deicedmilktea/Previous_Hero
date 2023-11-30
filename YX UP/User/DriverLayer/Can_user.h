#ifndef CAN_USER_H
#define CAN_USER_H

#include "main.h"

void can_1_user_init(CAN_HandleTypeDef* hcan );
void can_2_user_init(CAN_HandleTypeDef* hcan );
void can_remote(uint8_t sbus_buf[],uint32_t id);


#endif