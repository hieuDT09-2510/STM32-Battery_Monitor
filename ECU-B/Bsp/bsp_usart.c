#include "bsp_usart.h"
#include <stdio.h>
#include "stm32f10x.h"


void USART1_Init(void)
{
	//1. Bật xung nhịp GPIOA/USART1
	RCC->APB2ENR |= (1<<2);
	RCC->APB2ENR |= (1<<14);
	
	//2. Cấu hình PA9--TX PA10--RX
	GPIOA->CRH &= ~(0XF<<4);
	GPIOA->CRH |= (0XB<<4);
	
	GPIOA->CRH &=  ~(0XF<<8);
	GPIOA->CRH |= (0X4<<8);
	
	//3. Cấu hình tốc độ baud 115200
	USART1->BRR = 0X0270;
	
	//4. Bật cho phép USART/nhận/truyền
	USART1->CR1 |= (1<<2);
	USART1->CR1 |= (1<<3);
	USART1->CR1 |= (1<<13);
}


void USART1_SendByte(uint8_t data)
{
	// Chờ thanh ghi truyền trống
	while(!(USART1->SR & (1<<7)));
	USART1->DR = data;
}


//const -- chỉ có thể đọc, không thể sửa đổi
void USART1_SendString(const char *str)
{
	while(*str)
	{
		USART1_SendByte((uint8_t)*str++);
	}
}


int fputc(int ch, FILE *f)
{
    USART1_SendByte((uint8_t)ch);
    return ch;
}
