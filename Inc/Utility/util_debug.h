/*
 *  Project      :  
 *  
 *  FilePath     : util_debug.h
 *  Description  : 
 *  LastEditors  : Mr.Lee
 *  Date         : 2024��10��3��23:18:23
 *  LastEditTime : 
 */


#ifndef UTIL_DEBUG_H
#define UTIL_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#define VOFA_DATAPACK_HEAD      '='     /* VOFA����֡ͷ */
#define VOFA_DATAPACK_END       '\n'    /* VOFA����֡β */
#define VOFA_DATAPACK_MAXLEN    200     /* VOFA������󳤶� */


#define VOFA_JUSTFLOAT_FLOAT_BYTES  4       // ÿ��floatռ4�ֽ�
#define VOFA_JUSTFLOAT_TAIL_BYTES   4       // ֡βռ4�ֽ�
#define VOFA_JUSTFLOAT_TOTAL_LEN    (VOFA_JUSTFLOAT_CHANNEL_NUM * VOFA_JUSTFLOAT_FLOAT_BYTES + VOFA_JUSTFLOAT_TAIL_BYTES)

// JustFloat֡β��С���ֽ��򣺶�Ӧ32λֵ0x7F800000��
static const uint8_t VOFA_JUSTFLOAT_TAIL[VOFA_JUSTFLOAT_TAIL_BYTES] = {0x00, 0x00, 0x80, 0x7F};


// ������ջ�������С��������λ�����͵����ݳ��ȵ�������256�ֽ��㹻�󲿷ֳ�����
#define UART6_RX_BUF_SIZE 256

// ȫ�ֽ��ջ��������洢ԭʼ���ݣ�
extern uint8_t Uart6_Rx_Buf[UART6_RX_BUF_SIZE];
// ��Ч���ݳ��ȣ������ж��л�ȡ����¼��λ��ʵ�ʷ��͵��ֽ�����
extern uint16_t Uart6_Rx_Len;
// ������ɱ�־λ��������ѭ��/���������ж��Ƿ��������ݣ�
extern uint8_t Uart6_Rx_Complete_Flag;
extern volatile uint8_t g_uart6_print_flag ; 

uint16_t DMACurrentDataCounter(DMA_Stream_TypeDef *dma_stream);



void usart_printf(const char *fmt,...);
void Debug_Init();
void Debug_RXCallback(UART_HandleTypeDef* huart);
void vofa_justfloat_send(float *data_buf, uint8_t channel_num);

#endif

#ifdef __cplusplus
}

#endif
