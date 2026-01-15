//
// Created by 17087 on 25-11-3.
//
#include "servo_motor_uart.h"

uint8_t usart1SendBuf[USART_SEND_BUF_SIZE + 1];
uint8_t usart1RecvBuf[USART_RECV_BUF_SIZE + 1];
RingBufferTypeDef usart1SendRingBuf;
RingBufferTypeDef usart1RecvRingBuf;
Usart_DataTypeDef FSUS_usart1;
uint8_t rc1;

/* Transmit chunk size for non-blocking interrupt-driven sending.
   Keep chunk moderate to avoid long interrupt transmission times. */
#define USR_UART_TX_CHUNK_SIZE  64

/* Temporary transmit buffer and busy flag (file-local) */
static uint8_t usart1_tx_chunk[USR_UART_TX_CHUNK_SIZE];
static volatile uint8_t usart1_tx_busy = 0;

/**
 * Initialize user UART structures and start RX interrupt.
 * This also ensures the ring buffers are created. TX is implemented
 * in a non-blocking manner using HAL_UART_Transmit_IT in chunked mode.
 */
void User_Uart_Init(UART_HandleTypeDef *huartx)
{
    // 创建缓冲组
    RingBuffer_Init(&usart1SendRingBuf, USART_SEND_BUF_SIZE, usart1SendBuf);
    RingBuffer_Init(&usart1RecvRingBuf, USART_RECV_BUF_SIZE, usart1RecvBuf);
    // 初始化自定义用户串口结构体
    FSUS_usart1.recvBuf = &usart1RecvRingBuf;
    FSUS_usart1.sendBuf = &usart1SendRingBuf;
    FSUS_usart1.huartX = huartx;
    // Ensure TX state is idle
    usart1_tx_busy = 0;
    // 开启接收中断
    HAL_UART_Receive_IT(FSUS_usart1.huartX, (uint8_t *)&rc1, 1);
}

/**
 * Start a non-blocking transmit from the ring buffer.
 * This function will pull up to USR_UART_TX_CHUNK_SIZE bytes from the ring buffer
 * into a static chunk and call HAL_UART_Transmit_IT. If a transmit is already
 * in progress this function returns immediately.
 */
static void StartUartSendFromRing(Usart_DataTypeDef *usart)
{
    // Only single instance supported in this file (FSUS_usart1).
    if (usart->huartX == NULL) return;
    if (usart1_tx_busy) return; // already sending

    uint16_t toSend = 0;
    uint16_t available = RingBuffer_GetByteUsed(usart->sendBuf);
    if (available == 0) return;

    // Limit to chunk size
    if (available > USR_UART_TX_CHUNK_SIZE) available = USR_UART_TX_CHUNK_SIZE;

    // Pop bytes into chunk
    while (toSend < available) {
        usart1_tx_chunk[toSend++] = RingBuffer_Pop(usart->sendBuf);
    }

    if (toSend > 0) {
        usart1_tx_busy = 1;
        // Start interrupt-driven transmit (non-blocking). Timeout not used here.
        HAL_StatusTypeDef st = HAL_UART_Transmit_IT(usart->huartX, usart1_tx_chunk, toSend);
        if (st != HAL_OK) {
            // If starting TX failed, clear busy and optionally push bytes back.
            usart1_tx_busy = 0;
            // Attempt to push bytes back to ring buffer (best-effort).
            for (uint16_t i = 0; i < toSend; ++i) {
                RingBuffer_Push(usart->sendBuf, usart1_tx_chunk[i]);
            }
            printf("[Usart_Send] HAL_UART_Transmit_IT failed: %d\r\n", (int)st);
        }
    }
}

/**
 * Public function to request sending of all bytes currently in the ring buffer.
 * Non-blocking: it triggers an interrupt-driven transmit if idle; otherwise
 * the ongoing Tx completion callback will pull the next chunk.
 */
void Usart_SendAll(Usart_DataTypeDef *usart)
{
    // Trigger chunked non-blocking send if not busy.
    StartUartSendFromRing(usart);
}

/* UART Tx complete callback: clear busy flag and attempt to send next chunk. */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return;

    if (huart->Instance == FSUS_usart1.huartX->Instance)
    {
        // mark as not busy
        usart1_tx_busy = 0;
        // If there is more data, start next chunk
        if (RingBuffer_GetByteUsed(FSUS_usart1.sendBuf) > 0) {
            StartUartSendFromRing(&FSUS_usart1);
        }
    }
}

/* UART Rx complete callback: push received byte and re-enable RX interrupt. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ucTemp;
    if (huart->Instance == FSUS_usart1.huartX->Instance)
    {
        ucTemp = rc1;
        RingBuffer_Push(FSUS_usart1.recvBuf, ucTemp); // 接收到数据放入缓冲区，不在中断具体处理数据
        HAL_UART_Receive_IT(FSUS_usart1.huartX, (uint8_t *)&rc1, 1);
    }
}
