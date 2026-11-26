/*
 * Unit test trên máy host cho driver cấp board TPS22919.
 * Kiểm chứng chế độ PB9 và việc ghi mức cao/thấp thông qua thanh ghi STM32 giả lập, không cần kết nối board thực tế.
 */
#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "stm32f10x.h"
#include "bsp_load_switch.h"

GPIO_TypeDef g_test_gpio_b; /* Thực thể thanh ghi GPIOB giả lập. */
RCC_TypeDef g_test_rcc;     /* Thực thể thanh ghi RCC giả lập. */

static void ResetFakeRegisters(void)
{
    /* Mỗi test case bắt đầu từ trạng thái reset để tránh sót bit thanh ghi từ case trước. */
    (void)memset(&g_test_gpio_b, 0, sizeof(g_test_gpio_b));
    (void)memset(&g_test_rcc, 0, sizeof(g_test_rcc));
}

static void TestInitConfiguresPb9AndKeepsLoadOff(void)
{
    ResetFakeRegisters();
    /* Đặt toàn bộ CRH lên 1 trước để đảm bảo Init thực sự ghi lại cấu hình 4-bit của PB9. */
    g_test_gpio_b.CRH = 0xFFFFFFFFUL;
    LoadSwitch_Init();

    /* Kiểm chứng clock GPIOB, chế độ ngõ ra push-pull PB9 và mức tắt mặc định. */
    assert((g_test_rcc.APB2ENR & (1UL << 3)) != 0UL);
    assert(((g_test_gpio_b.CRH >> 4) & 0xFUL) == 0x2UL);
    assert((g_test_gpio_b.BRR & (1UL << 9)) != 0UL);
}

static void TestEnableAndDisableDrivePb9(void)
{
    ResetFakeRegisters();
    LoadSwitch_Init();

    /* Enable phải ghi BSRR để set bit PB9, Disable phải ghi BRR để xóa PB9 về 0. */
    LoadSwitch_Enable();
    assert((g_test_gpio_b.BSRR & (1UL << 9)) != 0UL);

    LoadSwitch_Disable();
    assert((g_test_gpio_b.BRR & (1UL << 9)) != 0UL);
}

int main(void)
{
    TestInitConfiguresPb9AndKeepsLoadOff();
    TestEnableAndDisableDrivePb9();
    return 0;
}
