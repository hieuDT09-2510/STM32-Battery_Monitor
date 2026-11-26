/*
 * Tên file: bsp_load_switch.c
 *
 * File này chỉ thao tác với kết nối phần cứng cố định PB9. Phần cứng dự án đã cố định,
 * nên không tạo thêm macro hay bộ nhớ đệm trạng thái chạy cho một chân không thay đổi,
 * giữ cho driver cấp board đơn giản và trực tiếp.
 */
#include "bsp_load_switch.h"
#include "stm32f10x.h"

void LoadSwitch_Init(void)
{
    /* PB9 thuộc GPIOB, cần bật clock cổng trước. */
    RCC->APB2ENR |= (1U << 3);

    /*
     * Trước khi đổi chế độ GPIO, đặt thanh ghi chốt ngõ ra xuống mức thấp trước.
     * Như vậy khi PB9 chuyển từ trạng thái ngõ vào lúc reset sang ngõ ra sẽ không làm bật tải thoáng qua.
     */
    GPIOB->BRR = (1U << 9);

    /* Cấu hình PB9 làm ngõ ra đẩy-kéo thông thường 2MHz: MODE=10, CNF=00. */
    GPIOB->CRH &= ~(0xFU << 4);
    GPIOB->CRH |=  (0x2U << 4);
}

void LoadSwitch_Enable(void)
{
    /* Chân ON của TPS22919 tích cực mức cao. */
    GPIOB->BSRR = (1U << 9);
}

void LoadSwitch_Disable(void)
{
    /* Ghi 1 vào BRR chỉ xóa bit ngõ ra tương ứng, không ảnh hưởng tới các chân khác của GPIOB. */
    GPIOB->BRR = (1U << 9);
}
