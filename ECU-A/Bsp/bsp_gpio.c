#include "stm32f10x.h"
#include "bsp_gpio.h"


void LED_Init(void)
{
	// 1. Bật clock cho GPIOC
	RCC->APB2ENR |= (1U << 4);
	
    // 2. Đặt trước thanh ghi chốt ngõ ra PC13 lên mức cao để LED mặc định tắt.
    GPIOC->BSRR = (1U << 13);

	// 3. Cấu hình PC13
	GPIOC->CRH &= ~(0xfU<<20);
	GPIOC->CRH |= (0x2U<<20);
	
}

void KEY_Init(void)
{
     // Bật clock ngoại vi GPIOB dùng cho nút nhấn khởi động PB6.
    RCC->APB2ENR |= (1U << 3);

    // Bật clock ngoại vi GPIOA dùng cho nút nhấn khôi phục lỗi PA2.
    RCC->APB2ENR |= (1U << 2);

    // Xóa cấu hình GPIO cũ của PB6 (nút nhấn khởi động).
    GPIOB->CRL &= ~(0xFU << 24);

    // Cấu hình PB6 ở chế độ ngõ vào kéo lên/kéo xuống (pull-up/pull-down).
    GPIOB->CRL |= (0x8U << 24);

    // Đặt bit ODR tương ứng của PB6 bằng 1 để chọn điện trở kéo lên nội bộ.
    GPIOB->ODR |= (1U << 6);

    // Xóa cấu hình GPIO cũ của PA2 (nút nhấn khôi phục lỗi).
    GPIOA->CRL &= ~(0xFU << 8);

    // Cấu hình PA2 ở chế độ ngõ vào kéo lên/kéo xuống (pull-up/pull-down).
    GPIOA->CRL |= (0x8U << 8);

    // Đặt bit ODR tương ứng của PA2 bằng 1 để chọn điện trở kéo lên nội bộ.
    GPIOA->ODR |= (1U << 2);
}

// Đọc xem nút nhấn chỉ định có đang được nhấn hay không.
uint8_t KEY_Read(uint8_t key_id)
{
    // Kiểm tra xem nút cần đọc hiện tại có phải nút khởi động hay không.
    if(key_id == KEY_START)
    {
        // Kiểm tra xem PB6 có bị nút nhấn kéo xuống mức thấp hay không.
        if((GPIOB->IDR & (1U << 6)) == 0)
        {
            // Trả về 1 cho biết nút khởi động đã được nhấn.
            return 1;
        }

        // Trả về 0 cho biết nút khởi động chưa được nhấn.
        return 0;
    }

    // Kiểm tra xem nút cần đọc hiện tại có phải nút khôi phục lỗi hay không.
    if(key_id == KEY_FAULT_RESET)
    {
        // Kiểm tra xem PA2 có bị nút nhấn kéo xuống mức thấp hay không.
        if((GPIOA->IDR & (1U << 2)) == 0)
        {
            // Trả về 1 cho biết nút khôi phục lỗi đã được nhấn.
            return 1;
        }

        // Trả về 0 cho biết nút khôi phục lỗi chưa được nhấn.
        return 0;
    }

    // Mã nút nhấn không hợp lệ thì thống nhất trả về chưa nhấn.
    return 0;
}
