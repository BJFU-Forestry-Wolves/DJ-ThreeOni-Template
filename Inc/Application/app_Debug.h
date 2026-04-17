/*
 *  Project      : 
 * 
 *  file         : app_Debug.c
 *  Description  : ���ļ�������VOFA��ū��
 *  LastEditors  : Yuyuan
 *  Date         : 2025��12��22��19:31:49
 *  LastEditTime : 
 */

#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif



#include "alg_math.h"
#include "alg_pid.h"



void Debug_Task(void const * argument);
void PID_ChangePID(PID_PIDParamTypeDef* pparam);
void PID_GimbalYawVision_SetAllParam(float new_kp, float new_ki, float new_kd);
#endif

#ifdef __cplusplus
}
#endif
