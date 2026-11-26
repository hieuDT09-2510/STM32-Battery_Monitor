#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "stm32f10x.h"

// Khai báo hàm khởi tạo LED
void LED_Init(void);

// Khai báo hàm khởi tạo GPIO nút nhấn.
void KEY_Init(void);

// Khai báo hàm đọc trạng thái nút nhấn.
uint8_t KEY_Read(uint8_t key_id);


#define LED_ON()    GPIOC->ODR &= ~(1U << 13)
#define LED_OFF()   GPIOC->ODR |=  (1U << 13)
#define LED_Toggle() GPIOC->ODR ^= (1U << 13)

// Định nghĩa mã logic cho nút nhấn khởi động.
#define KEY_START 0

// Định nghĩa mã logic cho nút nhấn khôi phục lỗi.
#define KEY_FAULT_RESET 1

#endif
