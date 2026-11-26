#include "stm32f10x.h"
#include "bsp_tim.h"

static volatile uint32_t g_sys_tick = 0;
	
void TIM3_Init(void)
{
	// 1. Bật clock cho TIM3
	RCC->APB1ENR |= (1<<1);
	
	// 2. Cấu hình chu kỳ 1ms
	TIM3->PSC = 71;
	TIM3->ARR = 999;
	
	// 3. Tạo sự kiện cập nhật (Update Event)
	TIM3->CR2 &= ~(0X7<<4);
	TIM3->CR2 |= (0X2<<4); // MMS -- sự kiện cập nhật (TRGO)
	
	// Cho phép ngắt với mức ưu tiên ngắt cao nhất
	TIM3->DIER |= (1<<0);
	
	/*
	 * Ngắt TIM3 chỉ duy trì bộ đếm mili-giây, không gọi FreeRTOS API,
	 * do đó có thể giữ mức ưu tiên chiếm quyền cao nhất là 0.
	 */
	NVIC_SetPriority(
		TIM3_IRQn,
		NVIC_EncodePriority(NVIC_PriorityGroup_4, 0, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

	// Bật bộ đếm
	TIM3->CR1 |= (1<<0);

}



uint32_t GetTick(void)
{
	return g_sys_tick;
}


void TIM3_IRQHandler(void)
{
	// 1. Chờ UIF=1 (cờ ngắt cập nhật)
	if(TIM3->SR & (1<<0))
	{
		// Xóa cờ ngắt cập nhật
		TIM3->SR &= ~(1<<0);
		
		// Tăng g_sys_tick
		g_sys_tick++;
	}
}
