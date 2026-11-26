// Driver ADC+DMA bộ đệm kép (kích hoạt bằng TIM3, 2 kênh: điện áp + nhiệt độ)
#include "stm32f10x.h"
#include "bsp_adc_dma.h"
#include "bsp_tim.h"


#define ADC_BUF_SIZE    4       // 2 kênh x 2 nhóm
__IO uint16_t adc_buf[ADC_BUF_SIZE];

volatile uint8_t g_adc_half_ready = 0;
volatile uint8_t g_adc_full_ready = 0;

// Chẩn đoán: Ghi nhận số lần vào ngắt truyền một nửa (half-transfer) DMA.
volatile uint32_t g_dma_half_irq_count = 0U;

// Chẩn đoán: Ghi nhận số lần vào ngắt truyền hoàn tất (full-transfer) DMA.
volatile uint32_t g_dma_full_irq_count = 0U;

void ADC_DMA_Init(void)
{
	// 1. Bật clock ADC1/DMA1/GPIOA
	RCC->APB2ENR |= (1<<9);
	RCC->AHBENR |= (1<<0);
	RCC->APB2ENR |= (1<<2);
	
	// 2. Cấu hình PA0/PA1 analog input
	GPIOA->CRL &= ~(0XF<<0);
	GPIOA->CRL &= ~(0XF<<4);
	
	// 3. Chia tần số clock ADC
	RCC->CFGR &= ~(0X3<<14);
	RCC->CFGR |= (0X2<<14);
	
	// 4. Thời gian lấy mẫu 239.5 chu kỳ
    ADC1->SMPR2 |= (0x7 << 0);   // CH0
    ADC1->SMPR2 |= (0x7 << 3);   // CH1
	
	// 5. Quét đa kênh -- SCAN
	ADC1->CR1 |= (1<<8);
	
	// 6. 2 chuyển đổi
	ADC1->SQR1 |= (1<<20);
	
	// 7. Thứ tự chuyển đổi
	ADC1->SQR3 |= (0<<0);
	ADC1->SQR3 |= (1<<5);
	
	// 8. Kích hoạt bằng timer TIM3_TRGO
	ADC1->CR2 &= ~(0X7<<17);
	ADC1->CR2 |= (0X4<<17); // Sự kiện TRGO của TIM3 100
	ADC1->CR2 |= (1<<20); // Cho phép kích hoạt ngoài
	
	// 9. Cấu hình DMA
	DMA1_Channel1->CPAR = (uint32_t) & ADC1->DR;
	DMA1_Channel1->CMAR = (uint32_t)adc_buf;
	DMA1_Channel1->CNDTR = ADC_BUF_SIZE;
	
	DMA1_Channel1->CCR |= (1 << 7);     // MINC -- Tăng địa chỉ bộ nhớ
   DMA1_Channel1->CCR |= (1 << 5);     // CIRC -- Chế độ vòng tròn (circular)
   DMA1_Channel1->CCR |= (1 << 8);     // PSIZE=16 -- Độ rộng dữ liệu ngoại vi 16-bit
   DMA1_Channel1->CCR |= (1 << 10);    // MSIZE=16 -- Độ rộng dữ liệu bộ nhớ 16-bit

	// 10. Ngắt hoàn tất truyền DMA / ngắt truyền một nửa
	DMA1_Channel1->CCR |= (1<<2);
	DMA1_Channel1->CCR |= (1<<1);
	
	/*
	 * Ngắt DMA hiện tại chỉ set cờ hoàn thành lấy mẫu, không gọi API FreeRTOS,
	 * do đó tạm thời dùng mức ưu tiên chiếm trước cao là 1.
	 */
	NVIC_SetPriority(
		DMA1_Channel1_IRQn,
		NVIC_EncodePriority(NVIC_PriorityGroup_4, 1, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
	// 11. Bật DMA
	DMA1_Channel1->CCR |= (1<<0);
	
	// 12. ADC cho phép DMA
	ADC1->CR2 |= (1<<8);
	
	// 13. Bật ADC
	ADC1->CR2 |= (1<<0);
	
	// 14. Hiệu chuẩn (calibration)
	ADC1->CR2 |= (1 << 3);
  while(ADC1->CR2 & (1 << 3));
  ADC1->CR2 |= (1 << 2);
  while(ADC1->CR2 & (1 << 2));
  // Lần chuyển đổi đầu tiên tự động kích hoạt sau khi TIM3 khởi động
}

uint8_t ADC_ReadSafe(uint16_t *ch0, uint16_t *ch1)
{
     if(g_adc_half_ready)
    {
        g_adc_half_ready = 0;
        *ch0 = adc_buf[0];    // Điện áp Cell (nửa vùng trước)
        *ch1 = adc_buf[1];    // Nhiệt độ (nửa vùng trước)
        return 1;
    }
    if(g_adc_full_ready)
    {
        g_adc_full_ready = 0;
        *ch0 = adc_buf[2];    // Điện áp Cell (nửa vùng sau)
        *ch1 = adc_buf[3];    // Nhiệt độ (nửa vùng sau)
        return 1;
    }
    return 0;
}

void DMA1_Channel1_IRQHandler(void)
{
    // DMA hoàn tất truyền một nửa.
    if ((DMA1->ISR & (1U << 2)) != 0U)
    {
        // IFCR là thanh ghi ghi 1 xóa 0, bắt buộc phải gán trực tiếp, không dùng |=.
        DMA1->IFCR = (1U << 2);

        // Thông báo cho vòng lặp chính nửa vùng trước có một nhóm dữ liệu hoàn chỉnh.
        g_adc_half_ready = 1U;

        // Đếm chẩn đoán.
        g_dma_half_irq_count++;
    }

    // DMA hoàn tất toàn bộ truyền.
    if ((DMA1->ISR & (1U << 1)) != 0U)
    {
        // Ghi trực tiếp 1 để xóa cờ hoàn tất toàn bộ truyền.
        DMA1->IFCR = (1U << 1);

        // Thông báo cho vòng lặp chính nửa vùng sau có một nhóm dữ liệu hoàn chỉnh.
        g_adc_full_ready = 1U;

        // Đếm chẩn đoán.
        g_dma_full_irq_count++;
    }
}
