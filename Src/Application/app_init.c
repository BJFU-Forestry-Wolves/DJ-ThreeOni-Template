/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_init.c
 *  Description  : All initialization threads
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:42:52
 *  LastEditTime : 2023-05-05 15:17:43
 */

#include "app_init.h"
#include "alg_pid.h"
#include "periph_motor.h"
#include "periph_remote.h"
#include "module_chassis.h"
#include "app_remote.h"
#include "util_debug.h"      //调试专用
#include "periph_bmi088.h"  // BMI088 IMU传感器驱动
#include "sys_dwt.h"         // 数据观察和跟踪单元（用于精确延时）
#include "sys_softTimer.h"   // 软件定时器系统
#include "app_ins.h"         // 惯性导航系统应用

void Init_InitAll() {

    DWT_Init(168); 
    BMI088_Init(0);                   // 初始化BMI088 IMU传感器（加速度计+陀螺仪）
    Debug_Init() ;
    Remote_InitRemote();

    INS_Init();                       // 初始化惯性导航系统，融合IMU数据
    // Motor Group init
	Can_InitFilterAndStart(&hcan1);
	Can_InitFilterAndStart(&hcan2);
    Motor_InitAllMotors();

	Chassis_InitChassis();
    Remote_RemotrControlInit();


	
}


void Init_Task(void const * argument) {
    forever {
        vTaskSuspend(Init_TaskHandleHandle);
      osDelay(25);
    }
}
