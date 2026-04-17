/**
 * @file app_Debug.c
 * @brief ??????????????
 * @details 
 * @author Yuyuan
 * @version V1.0
 * @date 2026-01-02
 * @attention ???CubeMX?????????????????
 */

#include "main.h"
#include "app_Debug.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "util_debug.h"
#include "sys_const.h"
#include "app_remote.h"
#include "app_ins.h"

#define VOFA_JUSTFLOAT_CHANNEL_NUM  1      
void Debug_Task(void const *argument)
{
 
    INS_INSTypeDef *ins = INS_GetINSPtr();
    for(;;)
    {


		
        //PID_GimbalYawVision_SetAllParam(YAKp, YAKi, YAKd);
        //PID_ChangePID(&gimbalyaw->angPIDParam);
		//??TIM3????????????10ms???g_uart6_print_flag????????1???????10ms??????????
       // if (g_uart6_print_flag == 1)
        //{

            float send_data[VOFA_JUSTFLOAT_CHANNEL_NUM] =
            {
              ins->Yaw
            };
			
            //		usart_printf("%d,%d,%d,%d,%d\r\n",param1,param2,param3,param4,param5);
            vofa_justfloat_send(send_data, VOFA_JUSTFLOAT_CHANNEL_NUM);
            g_uart6_print_flag = 0;
      // }
        osDelay(2);
    }



}




