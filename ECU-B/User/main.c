/*
 * Tên tệp: main.c
 *
 * Chức năng mô-đun:
 * Hoàn tất khởi tạo ngoại vi của nút màn hình hiển thị ECU-B, đồng thời làm mới OLED và xuất thông tin chẩn đoán trong vòng lặp chính.
 */
#include "stm32f10x.h"
#include "bsp_gpio.h"
#include "bsp_can.h"
#include "bsp_i2c.h"
#include "bsp_oled.h"
#include "bsp_usart.h"
#include "app_dash.h"
#include "bsp_tim.h"

int main(void)
{
	
	uint32_t last_refresh = 0;
	uint32_t last_print   = 0;


  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  LED_Init(); 
	USART1_Init(); 
	printf("ECU-B STARTED\r\n");
	I2C1_Init(); 
	OLED_Init(); 
	CAN1_Init();
	TIM3_Init();
	
  printf("[DASH] System Ready\r\n");
	
    
	OLED_Clear(); 
	OLED_ShowString(0,0,"BMS Dash v1.0"); 
	OLED_Refresh();

		while(1)
		{
            				/* OLED chỉ làm mới mỗi 100ms trong vòng lặp chính, ngắt CAN không thực hiện thao tác hiển thị tốn thời gian. */
				if(GetTick() - last_refresh >= 100U)
				{
						last_refresh = GetTick();

						Dash_RefreshDisplay();
				}

				if(GetTick() - last_print >= 1000)
				{
						last_print = GetTick();

						printf("[DASH] Cell=%dmV SOC=%d%% T=%dC Fault=0x%04X Alive=%u SeqErr=%u ErrCnt=%lu\r\n",
									 g_dash_data.cell_mv,
									 g_dash_data.soc,
									 g_dash_data.temp,
									 (unsigned int)g_dash_data.fault,
									 (unsigned int)g_dash_data.last_301_alive,
									 (unsigned int)g_dash_data.alive_301_error,
									 (unsigned long)g_dash_data.alive_301_error_count);
				}
		}
}
