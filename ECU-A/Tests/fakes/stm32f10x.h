/*
 * Fake thanh ghi STM32 tối thiểu dùng cho unit test công tắc tải trên máy host.
 * Ở đây chỉ định nghĩa các trường GPIOB và RCC mà bsp_load_switch.c thực sự truy cập; bài test không truy cập phần cứng.
 */
#ifndef TEST_FAKE_STM32F10X_H
#define TEST_FAKE_STM32F10X_H

#include <stdint.h>

typedef struct
{
    /* Thứ tự trường giữ nguyên bố cục thanh ghi GPIO của STM32 để mã nguồn được kiểm thử có thể biên dịch nguyên trạng. */
    volatile uint32_t CRL;
    volatile uint32_t CRH;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t BRR;
} GPIO_TypeDef;

typedef struct
{
    /* Thứ tự trường bao phủ từ đầu RCC đến APB2ENR để bài test quan sát bit cho phép clock của GPIOB. */
    volatile uint32_t CR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t APB1RSTR;
    volatile uint32_t AHBENR;
    volatile uint32_t APB2ENR;
} RCC_TypeDef;

extern GPIO_TypeDef g_test_gpio_b; /* Thanh ghi GPIOB giả lập trong RAM của tiến trình test. */
extern RCC_TypeDef g_test_rcc;     /* Thanh ghi RCC giả lập trong RAM của tiến trình test. */

/* Chuyển hướng tên ngoại vi trong mã nguồn thực tế sang đối tượng RAM. */
#define GPIOB (&g_test_gpio_b)
#define RCC   (&g_test_rcc)

#endif
