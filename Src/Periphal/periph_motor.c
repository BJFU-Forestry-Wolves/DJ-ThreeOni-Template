/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_motor.c
 *  Description  : This file contains motor control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:43:48
 *  LastEditTime : 2023-05-05 17:59:23
 */


#include "periph_motor.h"
#include "sys_const.h"
#include "stdio.h"
#include "stdlib.h"

/********** VOLATILE USER CODE **********/

const uint32_t Const_Motor_MOTOR_OFFLINE_TIME = 200;
const uint32_t Const_Motor_MOTOR_TX_EXTID = 0x01;
const uint32_t Const_Motor_MOTOR_TX_DLC = 8;
const uint32_t Const_Motor_MOTOR_RX_DLC = 8;

Motor_MotorGroupTypeDef *Motor_groupHandle[MOTOR_GROUP_NUM];

Motor_MotorGroupTypeDef Motor_ChassisMotors;
Motor_MotorGroupTypeDef Motor_ChassisSteerMotors;
Motor_MotorGroupTypeDef Motor_PitchMotors;        
Motor_MotorGroupTypeDef Motor_ShootMotors;
Motor_MotorGroupTypeDef Motor_FeedMotors;
Motor_MotorGroupTypeDef Motor_Big_YawMotors;  
Motor_MotorGroupTypeDef Motor_Small_YawMotors;  
Motor_MotorGroupTypeDef Motor_DM_Motors;        


Motor_MotorTypeDef Motor_ChassisLeftFrontMotor;
Motor_MotorTypeDef Motor_ChassisRightFrontMotor;
Motor_MotorTypeDef Motor_ChassisRearMotor;
Motor_MotorTypeDef Motor_ChassisFontRightSteerMotor;
Motor_MotorTypeDef Motor_ChassisFontLeftSteerMotor;
Motor_MotorTypeDef Motor_ChassisBackLeftSteerMotor;
Motor_MotorTypeDef Motor_ChassisBackRightSteerMotor;


/**
  * @brief      Motor encoder decoding callback function
  * @param      canid: CAN Handle number
  * @param      stdid: CAN identifier
  * @param      rxdata: CAN rx data buff
  * @retval     NULL
  */
void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len) {

	for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
        for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
            if (Motor_groupHandle[i]->motor_handle[j] == NULL) continue;
            if ((phcan == Motor_groupHandle[i]->can_handle) 
                  && (stdid == Motor_groupHandle[i]->motor_handle[j]->id) && phcan != NULL) {
                Motor_groupHandle[i]->motor_handle[j]->callback(Motor_groupHandle[i]->motor_handle[j], rxdata, len);
            }
        }
    }
}


/********** VOLATILE USER CODE END **********/


/**
  * @brief      Initialize all motors
  * @param      NULL
  * @retval     NULL
  */
void Motor_InitAllMotors() {
	
	
	Motor_groupHandle[3] = &Motor_ChassisMotors;//底盘全向轮电机组(3个2006)
    Motor_InitMotorGroup(&Motor_ChassisMotors, Motor_TYPE_RM2006, 3, &hcan1, 0x200);
    Motor_InitMotor(&Motor_ChassisLeftFrontMotor,  Motor_TYPE_RM2006, 0x201, 0.1, rm2006_encoder_callback);  /* 左前 id1 */
    Motor_InitMotor(&Motor_ChassisRightFrontMotor, Motor_TYPE_RM2006, 0x202, 0.1, rm2006_encoder_callback);  /* 右前 id2 */
    Motor_InitMotor(&Motor_ChassisRearMotor,       Motor_TYPE_RM2006, 0x203, 0.1, rm2006_encoder_callback);  /* 正后 id3 */
    Motor_ChassisMotors.motor_handle[0] = &Motor_ChassisLeftFrontMotor;
    Motor_ChassisMotors.motor_handle[1] = &Motor_ChassisRightFrontMotor;
    Motor_ChassisMotors.motor_handle[2] = &Motor_ChassisRearMotor;

	
	
		
}


/**
  * @brief      Initialize the motor
  * @param      pmotor: Pointer to motor object
  * @param      type: Type of motor (pwm or can)
  * @param      callback: Motor callback function
  * @retval     NULL
  */
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint16_t id, float fil_param, 
                     Motor_EncoderCallbackFuncTypeDef callback) {
    if (pmotor == NULL) return;
    pmotor->last_update_time = 0;
    pmotor->type = type;
    pmotor->id = id;			
    pmotor->init = 0;			
    pmotor->is_online = 0;
    pmotor->output = 0;
    pmotor->callback = callback;
    Filter_LowPassInit(fil_param, &pmotor->fdb_fil_param);
}


/**
  * @brief      Initialization of motor group
  * @param      pgroup: Pointer to motor group
  * @param      type: Type of motor (pwm or can)
  * @param      motor_num: Number of motor group
  * @param      phcan: Pointer of can handle
  * @param      stdid: Motor id
  * @retval     NULL
  */
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, Motor_MotorTypeEnum type, uint8_t motor_num, CAN_HandleTypeDef* phcan, uint16_t stdid) {
    if (pgroup == NULL) return;
    pgroup->motor_num = motor_num;
    pgroup->type = type;

    if (phcan == NULL) return;
    pgroup->can_handle = phcan;
    Can_InitTxHeader(&(pgroup->can_header), stdid, Const_Motor_MOTOR_TX_EXTID, Const_Motor_MOTOR_TX_DLC);

    for (int i = 0; i < 4; ++i) {
        pgroup->motor_handle[i] = NULL;
    }
}


/**
  * @brief      Set output
  * @param      pmotor: Pointer to motor object
  * @param      pparam: Pointer to motor parameter object
  * @retval     NULL
  */
void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output) {
    pmotor->output = output;//?? PID ???????????????output???
}


/**
  * @brief      Get motor output value
  * @param      pmotor: Pointer to motor object
  * @retval     Output value
  */
uint16_t Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL) return 0;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED) return 0;

    int16_t ret = 0;
    if (pmotor->type == Motor_TYPE_RM3508 || pmotor->type == Motor_TYPE_RM2006 || pmotor->type ==Motor_TYPE_RM6020) {
        ret = (int16_t)(pmotor->output * 1000.0f);               // output / 10.0f * 10000.0f
        return (uint16_t)ret;
    }
    return 0;
}


/**
  * @brief      Transmitter output
  * @param      pgroup: Pointer to the motor group to send
  * @retval     NULL
  */
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef *pgroup) {
    if (pgroup == NULL) return;//
    uint8_t txdata[64];
	memset(txdata, 0, 64);
    uint16_t out0 = Motor_GetMotorOutput(pgroup->motor_handle[0]);
    uint16_t out1 = Motor_GetMotorOutput(pgroup->motor_handle[1]);
    uint16_t out2 = Motor_GetMotorOutput(pgroup->motor_handle[2]);
    uint16_t out3 = Motor_GetMotorOutput(pgroup->motor_handle[3]);
    txdata[0] = (uint8_t)(out0 >> 8);
    txdata[1] = (uint8_t)out0;
    txdata[2] = (uint8_t)(out1 >> 8);
    txdata[3] = (uint8_t)out1;
    txdata[4] = (uint8_t)(out2 >> 8);
    txdata[5] = (uint8_t)out2;
    txdata[6] = (uint8_t)(out3 >> 8);
    txdata[7] = (uint8_t)out3;
    Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txdata);
}


void Motor_SendMotorGroupsOutput() {
    for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
        Motor_SendMotorGroupOutput(Motor_groupHandle[i]);
    }
}


/**
  * @brief      Judge whether any motor is offline
  * @param      NULL
  * @retval     Offline or not (1 is yes, 0 is no)
  */
uint8_t Motor_IsAnyMotorOffline() {
    for (int i = 0; i < MOTOR_GROUP_NUM; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!Motor_IsMotorOffline(Motor_groupHandle[i]->motor_handle[j])) {
                return 0;
            }
        }
    }
    return 1;
}


/**
  * @brief      Judge whether the motor is offline
  * @param      pmotor: Pointer to motor object
  * @retval     Offline or not (1 is yes, 0 is no)
  */
uint8_t Motor_IsMotorOffline(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL) return 0;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED) return 0;
    uint32_t now = HAL_GetTick();
    return (now - pmotor->last_update_time) < Const_Motor_MOTOR_OFFLINE_TIME;
}


/********** RM2006,RM3508,GM6020电机回调函数 **********/

/**
  * @brief      rm2006 motor encoder callback
  * @param      pmotor: Pointer to motor object
  * @retval     NULL
  */
void rm2006_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL) return;
    if (len != 8) return;

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    pmotor->encoder.angle   = (float)((int16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]));
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3])) / 36.0f;
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
    pmotor->encoder.temp = 0; 
    pmotor->init = 1; 

    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
    pmotor->last_update_time = HAL_GetTick(); 
}


/**
  * @brief      Gimbal motor encoder callback
  * @param      pmotor: Pointer to motor object
  * @retval     NULL
  */
void gm6020_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    // Calculate angle difference and number of cycles
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      //The increase of mechanical angle is positive
    if (diff < -5500)           // Make a positive turn
        pmotor->encoder.round_count++;
    else if (diff > 5500)       // Turn around in the opposite direction
        pmotor->encoder.round_count--;

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    pmotor->encoder.angle   = (float)((int16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]));
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3]));
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
    pmotor->encoder.temp = 0; 
    pmotor->init = 1; 

    // Calculate continuous angle
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    pmotor->encoder.limited_angle = (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    pmotor->last_update_time = HAL_GetTick();
}


/**
  * @brief      Gimbal motor encoder callback
  * @param      pmotor: Pointer to motor object
  * @retval     NULL
  */
void rm3508_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL) return;
    

    // Assign a value to the previous angle and get the latest angle
    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    pmotor->encoder.angle   = (float)((int16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]));
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3])) / 19.0f;
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
    pmotor->encoder.temp    = (float)((int16_t)((uint16_t)rxbuff[6]));
    
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 19
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f / 19.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 360.0f / 19.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;	
	
    pmotor->last_update_time = HAL_GetTick(); 
}

