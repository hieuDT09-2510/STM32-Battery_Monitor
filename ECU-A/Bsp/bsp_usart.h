/*
 * Tên file: bsp_usart.h
 *
 * Tên mô-đun: Giao diện công khai cho driver thu/phát cấp board USART1
 *
 * Trách nhiệm mô-đun:
 * 1. Khởi tạo GPIO, baudrate và ngắt nhận của USART1;
 * 2. Dùng bộ đệm vòng tĩnh để lưu tạm các byte nhận được trong ngắt;
 * 3. Cung cấp giao diện gửi chặn (blocking) và đọc không chặn (non-blocking);
 * 4. Cung cấp bộ đếm chẩn đoán số byte nhận, lỗi phần cứng và tràn bộ đệm.
 *
 * Phạm vi mô-đun:
 * Mô-đun này chỉ chịu trách nhiệm thu/phát byte tin cậy, không ghép lệnh, không phân tích CLI,
 * không thực hiện printf, truy cập Flash hoặc logic nghiệp vụ BMS trong ngắt.
 */
#ifndef __BSP_USART_H
#define __BSP_USART_H

#include "stm32f10x.h"
#include <stdio.h>

void USART1_Init(void);
void USART1_SendByte(uint8_t data);
void USART1_SendString(const char *str);
uint8_t USART1_ReadByteNonBlocking(uint8_t *data);
void USART1_WaitSendComplete(void);

/* Hàm xử lý nhận tối thiểu được gọi từ điểm vào ngắt phần cứng USART1. */
void USART1_RxIRQHandler(void);

/* Các bộ đếm chẩn đoán nhận USART1 dùng để theo dõi trên Keil Watch. */
extern volatile uint32_t g_usart1_rx_byte_count;
extern volatile uint32_t g_usart1_rx_hw_error_count;
extern volatile uint32_t g_usart1_rx_buffer_overflow_count;

#endif
