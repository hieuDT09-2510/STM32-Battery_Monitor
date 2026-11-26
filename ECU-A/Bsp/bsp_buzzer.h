// Ngăn chặn việc include lặp lại tệp tiêu đề.
#ifndef __BSP_BUZZER_H

// Định nghĩa macro bảo vệ tệp tiêu đề.
#define __BSP_BUZZER_H

// Include định nghĩa thanh ghi STM32F103.
#include "stm32f10x.h"

// Khai báo hàm khởi tạo còi báo.
void BUZZER_Init(void);

// Khai báo hàm bật còi báo.
void BUZZER_On(void);

// Khai báo hàm tắt còi báo.
void BUZZER_Off(void);

// Kết thúc bảo vệ tệp tiêu đề.
#endif
