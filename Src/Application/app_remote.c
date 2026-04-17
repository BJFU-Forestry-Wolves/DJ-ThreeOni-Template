/*
 *  Project      : Polaris
 * 
 *  file         : app_remote.c
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-07 11:26:15
 */


#include "sys_const.h"
#include "app_remote.h"
#include "module_chassis.h"
#include "periph_servo.h"
#include "cmsis_os.h"
#include "util_can.h"
#include "alg_math.h"


#define REMOTE_TASK_PERIOD  1
#define ENCODER_LIMIT 500
Remote_RemoteControlTypeDef Remote_remoteControlData;
Math_SlopeParamTypeDef Remote_ChassisFBSlope;
PID_GimbalYawVisionTypeDef Gimbal_YawVisionPID;
float last_encoder_angle=0;
float encoder_angle=0;
float get_abs(float a){
    if(a>=0){
		return a;
		}
		else return(0-a);

}
float limit_siqu(float last_angle,float angle){
       if(get_abs((int16_t)(angle-last_angle))<=ENCODER_LIMIT){ 
			 return last_angle;}
	     else{
			 return angle;
			 }

}

/**
  * @brief          Remote task
  * @param          NULL
  * @retval         NULL
  */
void Remote_Task(void const * argument) {

    forever {
        Remote_ControlCom();
      osDelay(REMOTE_TASK_PERIOD);
    }
}


/**
  * @brief      Remote Control Init
  * @param      NULL
  * @retval     NULL
  */
void Remote_RemotrControlInit() {
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    
    Math_InitSlopeParam(&Remote_ChassisFBSlope, MOUSE_CHASSIS_ACCELERATE, MOUSE_CHASSIS_ACCELERATE);
	
	
}


/**
  * @brief      Gets the pointer to the remote control data object
  * @param      NULL
  * @retval     Pointer to remote control data object
  */
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr() {
    return &Remote_remoteControlData;
}


/**
* @brief      Remote control command
* @param      NULL
* @retval     NULL
*/
Remote_RemoteDataTypeDef *testdata;
void Remote_ControlCom() {
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		testdata = data;
    control_data->pending = 1;

    switch (data->remote.s[0]) {
    /*      right switch control mode   */
        case Remote_SWITCH_UP: {
            /* right switch up is remote normal mode */
            Remote_RemoteProcess();
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* right switch mid is keymouse mode    */
            Remote_RemoteShooterModeSet();
            break;
        }
        case Remote_SWITCH_DOWN: {
            /* right switch down is auto aim mode   */
            Remote_KeyMouseProcess();
            Remote_MouseShooterModeSet();
            break;
        }
        default:
            break;
    }

    control_data->pending = 0;
}


/**
* @brief      Mouse shoot mode set
* @param      NULL
* @retval     NULL
*/

/**
* @brief      Remote shoot mode set
* @param      NULL
* @retval     NULL
*/
float testvision;
float testvision2;
void Remote_RemoteShooterModeSet() {
	
}


/**
* @brief      Remote control process
* @param      NULL
* @retval     NULL

*/
float test_pitch_ref;
float test_pitch_limit;
float test_pitch_set;
void Remote_RemoteProcess() {
    
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		
    switch (data->remote.s[1]) {
    /*      left switch control mode   */
        case Remote_SWITCH_UP: {
                Chassis_SetChassisMode(Chassis_XTL);
			Chassis_SetChassisRef((float)data->remote.ch[1], (float)data->remote.ch[0], (float)data->remote.ch[2]);
           
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            Chassis_SetChassisMode(Chassis_XTL);
	       Chassis_SetChassisRef((float)data->remote.ch[1], (float)data->remote.ch[0], (float)data->remote.ch[2]);
					break;
        }
        case Remote_SWITCH_DOWN: {
            /* left switch down is slow shooting   */
            Chassis_SetChassisMode(Chassis_XTL);
	      Chassis_SetChassisRef((float)data->remote.ch[1], (float)data->remote.ch[0], (float)data->remote.ch[2]);
							
            break;
        }
        default:
            break;
    }
	
    
}


/******************************************�������ģʽ******************************************************/
/*************************************�˲��ֵ���д��������ʹ��************************************************/



float autoaim_pitchz;
void Remote_KeyMouseProcess() { 

}
int count_cqie = 0;
int test_count ;
void Remote_MouseShooterModeSet() {
  
}


