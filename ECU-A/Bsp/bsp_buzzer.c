// Bao gồm định nghĩa thanh ghi STM32F103.
#include "stm32f10x.h"

// Bao gồm giao diện trình điều khiển còi báo.
#include "bsp_buzzer.h"

// Định nghĩa giá trị tự động nạp lại TIM1, tương ứng với PWM 2kHz.
#define BUZZER_PWM_ARR 499U

// Định nghĩa giá trị so sánh tỷ lệ chu kỳ xung 50% khi bật còi báo.
#define BUZZER_PWM_DUTY 125U

// Định nghĩa giá trị so sánh mức cao không đổi khi tắt còi báo.
#define BUZZER_PWM_OFF 500U

// Khởi tạo PA8 và PWM TIM1_CH1.
void BUZZER_Init(void)
{
    // Bật clock ngoại vi GPIOA.
    RCC->APB2ENR |= (1U << 2);

    // Bật clock ngoại vi TIM1.
    RCC->APB2ENR |= (1U << 11);

    // Xóa cấu hình GPIO ban đầu của PA8.
    GPIOA->CRH &= ~(0xFU << 0);

    // Cấu hình PA8 làm ngõ ra đẩy-kéo chức năng thay thế 2MHz.
    GPIOA->CRH |= (0xAU << 0);

    // Dừng bộ đếm TIM1 để tránh tạo dạng sóng bất thường trong quá trình cấu hình.
    TIM1->CR1 &= ~(1U << 0);

    // Thiết lập bộ chia tần trước của TIM1 thành 71, chia clock bộ định thời 72MHz xuống 1MHz.
    TIM1->PSC = 71U;

    // Thiết lập giá trị tự động nạp lại thành 499 để chu kỳ PWM là 500us.
    TIM1->ARR = BUZZER_PWM_ARR;

    // Thiết lập giá trị so sánh kênh 1 của TIM1 thành 500 để PA8 giữ mức cao không đổi sau khi khởi tạo.
    TIM1->CCR1 = BUZZER_PWM_OFF;

    // Xóa cấu hình chế độ so sánh ngõ ra kênh 1 của TIM1.
    TIM1->CCMR1 &= ~(0xFFU << 0);

    // Cấu hình kênh 1 của TIM1 sang chế độ PWM 1.
    TIM1->CCMR1 |= (0x6U << 4);

    // Bật chức năng nạp trước giá trị so sánh cho kênh 1 của TIM1.
    TIM1->CCMR1 |= (1U << 3);

    // Kích hoạt ngõ ra kênh 1 của TIM1.
    TIM1->CCER |= (1U << 0);

    // Bật chức năng nạp trước giá trị tự động nạp lại của TIM1.
    TIM1->CR1 |= (1U << 7);

    // Kích hoạt ngõ ra chính của bộ định thời nâng cao TIM1.
    TIM1->BDTR |= (1U << 15);

    // Tạo một sự kiện cập nhật để cấu hình PSC, ARR và CCR có hiệu lực ngay lập tức.
    TIM1->EGR |= (1U << 0);

    // Khởi động bộ đếm TIM1.
    TIM1->CR1 |= (1U << 0);
}

// Bật còi báo.
void BUZZER_On(void)
{
    // Thiết lập tỷ lệ chu kỳ xung TIM1_CH1 thành 50% để tạo sóng vuông 2kHz.
    TIM1->CCR1 = BUZZER_PWM_DUTY;
}

// Tắt còi báo.
void BUZZER_Off(void)
{
    // Thiết lập TIM1_CH1 ở mức cao không đổi để còi báo ngừng rung.
    TIM1->CCR1 = BUZZER_PWM_OFF;
}
