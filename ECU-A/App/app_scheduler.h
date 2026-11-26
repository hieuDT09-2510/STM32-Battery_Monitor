/*
 * Mô-đun: Giao diện công khai của bộ lập lịch tương thích trong giai đoạn chuyển sang FreeRTOS.
 *
 * Hiện tại chỉ định nghĩa mã chức năng chu kỳ LED chưa chuyển đổi.
 *
 * Các chức năng Sensor, BMS, CAN, FaultLog và Health
 * đã được chuyển sang các tác vụ FreeRTOS độc lập tương ứng.
 *
 * Thứ tự mã tác vụ phải khớp hoàn toàn với thứ tự mảng
 * g_task_period trong app_scheduler.c.
 */
#ifndef __APP_SCHEDULER_H
#define __APP_SCHEDULER_H
#include "stm32f10x.h"

typedef enum
{
    TASK_LED_HEARTBEAT = 0,            // Nhịp tim LED vận hành
    TASK_COUNT
} TaskID_t;


void Scheduler_Init(void);
void Scheduler_Run(void);

#endif
