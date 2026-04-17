/*
 *  Project      : ���ļ���һ���������ܸ�
 *
 *  FilePath     : util_debug.c
 *  Description  :
 *  LastEditors  : Mr.Lee
 *  Date         : 2024��10��3��23:18:23
 *  LastEditTime :
 */


/**
* @file util_debug.c
* @brief �ڶ���������Yuyuan��
* @details ���ļ�����VOFA����
* @author Yuyuan
* @version V1.0
* @date 2026-01-02  15��55
* @attention  ������app_Debug.c������ã������޸�ֱ�ӿ���ʹ�ã�������FireWater��JustFloat����Э�飬�����������
*             Ҳ����˵���㲻�����Դ�ӡ����Ҫ��������VOFA�����γɲ���ͼ�����ӻ�PID���������㻹����VOFA������ʵʱ�ı�PID�����ݲ���ͼѸ�ٵ���
*              ���KP,KD,KI��������TMDллYUYUAN
*/
#include "util_debug.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "app_Debug.h"
#include "app_remote.h"


#define Const_Debug_RX_BUFF_LEN             	54
#define 			Debug_Uart				huart6
#define PRINT_TX_BUF_SIZE 256              // ���ͻ�������С
#define VOFA_JUSTFLOAT_TX_BUF_SIZE  64  // �㹻����8��ͨ��+֡β

UART_HandleTypeDef *Const_Debug_UART_HANDLER       = &huart6;
uint8_t Debug_RxData[54];


static uint8_t g_print_tx_buf[PRINT_TX_BUF_SIZE];
static volatile uint8_t g_dma_tx_busy = 0; // DMA����æ��ǣ�0-���У�1-æµ
volatile uint8_t g_uart6_print_flag = 0; // 10ms��ӡ��־λ��0-δ��ʱ����1-�ɴ�ӡ


uint8_t g_ucVofaBuf[VOFA_DATAPACK_MAXLEN];
static float vofa_get_data(uint8_t, uint8_t);
static void vofa_set_sram_data(uint8_t, float);
static uint8_t g_vofa_justfloat_tx_buf[VOFA_JUSTFLOAT_TX_BUF_SIZE];



/**
  * @brief ����VOFA��FireWaterЭ�飬���ǻ����ռ��CPU��Դ������freertos�ᱻ��ס
  *         ����YUYUAN�����᲻��ɾ�����ͱ������������
  * @param fmt
  * @return NULL
  * @retval NULL
  */
//ʹ�÷�����usart_printf("%d,%d,%d,%d,%d\r\n",param1,param2,param3,param4,param5);      \r\n������ɾ��
void usart_printf(const char *fmt, ...)
{
    va_list ap;
    uint16_t len;

    // ��DMA���ڷ��ͣ�ֱ�ӷ���
    if (g_dma_tx_busy == 1)
    {
        return;
    }

    // 1. ��ʽ�����ݵ����ͻ�������ʹ��vsnprintf���⻺���������
    va_start(ap, fmt);
    len = vsnprintf((char *)g_print_tx_buf, PRINT_TX_BUF_SIZE - 1, fmt, ap);
    va_end(ap);
    g_print_tx_buf[len] = '\0'; // ȷ���ַ�������

    // 2. ���DMA����æ������DMA����������
    g_dma_tx_busy = 1;
    HAL_UART_Transmit_DMA(&Debug_Uart, g_print_tx_buf, len);
}





/**
  * @brief ����VOFA��JustFloatЭ�飬�����ٶȿ죬����Ӱ��freertos
  * @param  
  * @return NULL
  * @retval NULL
  */
void vofa_justfloat_send(float *data_buf, uint8_t channel_num)
{
    // 1. �Ϸ���У��
    if (data_buf == NULL || channel_num == 0 || g_dma_tx_busy == 1)
    {
        return; // ��ָ��/��ͨ��/DMAæµʱֱ�ӷ���
    }

    // 2. �����ܳ��ȣ�ͨ������+֡β��
    uint16_t total_len = channel_num * 4 + 4;
    if (total_len > VOFA_JUSTFLOAT_TX_BUF_SIZE)
    {
        return; // ���������㣬�������
    }

    // 3. ��װfloat���ݣ������ƿ�����С����ֱ��ӳ�䣩
    uint8_t *tx_ptr = g_vofa_justfloat_tx_buf;
    for (uint8_t i = 0; i < channel_num; i++)
    {
        // ��floatǿ��ת��Ϊuint8_tָ�룬���ֽڿ���
        memcpy(tx_ptr, &data_buf[i], 4);
        tx_ptr += 4; // ָ��ƫ��4�ֽڣ�ָ����һ��ͨ��
    }

    // 4. ƴ��֡β
    memcpy(tx_ptr, VOFA_JUSTFLOAT_TAIL, 4);

    // 5. ����DMA����
    g_dma_tx_busy = 1;
    HAL_UART_Transmit_DMA(&Debug_Uart, g_vofa_justfloat_tx_buf, total_len);
}



 //vofa���͸���Ƭ�������� ����Щ�����㲻��Ҫ��
void vofa_set_data(uint8_t _rx_byte)     
{
    static uint8_t end_pos = 0;
    static uint8_t head_pos = 0;


    g_ucVofaBuf[end_pos] = _rx_byte;

    if (_rx_byte == VOFA_DATAPACK_HEAD)
    {
        head_pos = end_pos;
    }
    else if(_rx_byte == VOFA_DATAPACK_END && g_ucVofaBuf[head_pos] == VOFA_DATAPACK_HEAD)
    {

        float VofaData = vofa_get_data(head_pos, end_pos);


        vofa_set_sram_data(head_pos, VofaData);


        end_pos = 0;
        head_pos = 0;
        memset(g_ucVofaBuf, 0x00, VOFA_DATAPACK_MAXLEN);
    }


    if(++end_pos > VOFA_DATAPACK_MAXLEN)
    {
        end_pos = 0;
        memset(g_ucVofaBuf, 0x00, VOFA_DATAPACK_MAXLEN);
    }
}



static float vofa_get_data(uint8_t _head, uint8_t _end)
{
    char _VofaData[_end - (_head + 1)];
    for(uint8_t i = 0; i < _end; i++)
        _VofaData[i] = g_ucVofaBuf[i + _head + 1];

    return atof(_VofaData);
}



/**
  * @brief  ���մ˸�ʽ,YP=%.2f\n�������������YPʶ��\n�������м���VOFA���͵����ݣ����Զ��滻����Ҫ�ı������
  * @param 
  * @return NULL
  * @retval NULL
  */
static void vofa_set_sram_data(uint8_t _head, float _data)
{
    uint8_t _id_pos1 = _head - 2;   /* ���ݺŵĵ�1λ - P/I/... */
    uint8_t _id_pos2 = _head - 1;   /* ���ݺŵĵ�2λ - 1/2/3/4/...*/


    /* P1 - g_tAnglePID.kp */
    if (g_ucVofaBuf[_id_pos1] == 'Y' && g_ucVofaBuf[_id_pos2] == 'P')
    {
        
    }
    /* D1 - g_tAnglePID.kd */
    else if (g_ucVofaBuf[_id_pos1] == 'Y' && g_ucVofaBuf[_id_pos2] == 'D')
    {
     
    }
    else if (g_ucVofaBuf[_id_pos1] == 'Y' && g_ucVofaBuf[_id_pos2] == 'I')
    {
        
    }
    else if (g_ucVofaBuf[_id_pos1] == 'T' && g_ucVofaBuf[_id_pos2] == '1')
    {
       
    }
    else if (g_ucVofaBuf[_id_pos1] == 'S' && g_ucVofaBuf[_id_pos2] == 'P')
    {
       
    }
    else if (g_ucVofaBuf[_id_pos1] == 'S' && g_ucVofaBuf[_id_pos2] == 'D')
    {
       
    }
    else if (g_ucVofaBuf[_id_pos1] == 'M' && g_ucVofaBuf[_id_pos2] == 'T')
    {

    }


}

/**
  * @brief   ����UART6�Ŀ����ж�DMA�������������ʹ��DMA
  * @param : [����/��]
  * @return NULL
  * @retval NULL
  */
void Debug_Init()
{

    /*	��ʼ��DMAͨ��	*/    //��ʵ���ǿ����жϵ�����
    __HAL_UART_CLEAR_IDLEFLAG(Const_Debug_UART_HANDLER);
    __HAL_UART_ENABLE_IT(Const_Debug_UART_HANDLER, UART_IT_IDLE);



    uint32_t tmp1 = 0;


    tmp1 = Const_Debug_UART_HANDLER->RxState;
    if(tmp1 == HAL_UART_STATE_READY)
    {

        Const_Debug_UART_HANDLER->pRxBuffPtr = Debug_RxData;
        Const_Debug_UART_HANDLER->RxXferSize = 54;
        Const_Debug_UART_HANDLER->ErrorCode  = HAL_UART_ERROR_NONE;
        /* ��DMA����ͨ�� */
        HAL_DMA_Start(Const_Debug_UART_HANDLER->hdmarx, (uint32_t)&Const_Debug_UART_HANDLER->Instance->DR, (uint32_t)Debug_RxData, 54);

        SET_BIT(Const_Debug_UART_HANDLER->Instance->CR3, USART_CR3_DMAR);

    }
}

/**
  * @brief      Debug���ջص�����
  * @param      ������Լ�ctrl+F������������õ�
  * @retval     ��UART6���յ�VOFA���������ݣ��ͻ�ִ���������
  */
int test_lens;
void Debug_RXCallback(UART_HandleTypeDef *huart)
{
    /* �ر�DMA */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* ����DMA�д������������ */
    int rxdatalen = Const_Debug_RX_BUFF_LEN - DMACurrentDataCounter(huart->hdmarx->Instance);
    test_lens = rxdatalen;
    if (test_lens > 0)   // ���ⳤ��Ϊ0ʱ��Чѭ��
    {
        for (int i = 0; i < test_lens; i++)
        {
            vofa_set_data(Debug_RxData[i]); // ����ֽڴ���VOFA��������
        }
        //    }
        /* ���´�DMA */
        __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Debug_RX_BUFF_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}


/**
  * @brief      ��ȡ��ǰDMAy Streamx������ʣ������ݵ�Ԫ��
  * @param      dma_stream: ָ�� `DMA_Stream_TypeDef` �ṹ��ָ�룬
  *             ����y������1��2����ʾѡ���DMA��������
  *             x������0��7����ʾѡ���DMA Stream��
  * @retval     ��ǰDMAy Streamx������ʣ������ݵ�Ԫ����
  * @note       �ú���ͨ����ȡDMA���ṹ���NDTR�Ĵ���ֵ����ȡ��ǰDMA��������δ��������ݵ�Ԫ����
  *             NDTR�Ĵ�����Number of Data to Transfer Register���洢��DMA����ʣ������ݵ�Ԫ����
  *             �üĴ�����ֵ����ÿ�����ݴ�����Զ��ݼ���
  */
uint16_t DMACurrentDataCounter(DMA_Stream_TypeDef *dma_stream)
{
    /* ���ص�ǰDMAy Streamx������ʣ������ݵ�Ԫ�� */
    return ((uint16_t)(dma_stream->NDTR));
}





/**
  * @brief  ����UART6��TX�Ļص�������DMA״̬����DMA������������
  * @param huart: [����/��]
  * @return NULL
  * @retval NULL
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // ������USART6�ķ�����ɻص�
    if (huart->Instance == Debug_Uart.Instance)
    {
        g_dma_tx_busy = 0; // ����DMA����״̬��������һ�δ�ӡ
    }
}

