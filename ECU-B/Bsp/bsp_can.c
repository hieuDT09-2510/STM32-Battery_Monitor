#include "bsp_can.h"
#include "can_protocol.h"
#include "stm32f10x.h"
#include "app_dash.h"
#include "bsp_usart.h"


// Khởi tạo CAN
void CAN1_Init(void)
{
	//1. Bật xung nhịp GPIOA/chức năng thay thế AFIO/CAN1
	RCC->APB2ENR |= (1<<2);
	RCC->APB2ENR |= (1<<0);
	RCC->APB1ENR |= (1<<25);
	
	//2. Cấu hình PA11-CAN_RX đầu vào kéo lên / PA12--CAN_TX đẩy-kéo chức năng thay thế
	GPIOA->CRH &= ~(0xF << 12);
    GPIOA->CRH |=  (0x8 << 12);
    GPIOA->ODR |= (1 << 11);

    GPIOA->CRH &= ~(0xF << 16);
    GPIOA->CRH |=  (0xB << 16);
	
	//3. Vào chế độ khởi tạo
	CAN1->MCR |= (1<<0);// Yêu cầu khởi tạo INRQ
	while((CAN1->MSR & (1<<0)) == 0);// Xác nhận khởi tạo INAK
	
	// Tự động khôi phục khi Bus-Off
	CAN1->MCR |= (1 << 6);
	
	CAN1->IER |= CAN_IER_FMPIE0;
	
	//4. Cấu hình định thời bit, tốc độ baud 500kbps
	CAN1->BTR = 0;
	
	CAN1->BTR |= (0<<24);//SJW=1TQ->0
	CAN1->BTR |= (3<<20);//BS2=4TQ->3
	CAN1->BTR |= (12<<16);//BS1=13TQ->12
	CAN1->BTR |= (3<<0); //BRP=4TQ->3
	
	//5. Cấu hình bộ lọc
//	CAN1->FMR |= 1;  // Vào chế độ cấu hình
//	
////CAN1->FA1R &= ~((1 << 0) | (1 << 1));// Tắt bộ lọc

//	// Filter0: 0x301 + 0x302
//	CAN1->FM1R &= ~(1 << 0);
//	CAN1->FM1R |= (1 << 0);   // Chế độ danh sách
//	CAN1->FS1R &= ~(1 << 0);
//	CAN1->FS1R |= (1 << 0);   // 32-bit
//	CAN1->sFilterRegister[0].FR1 = (0x301 << 21);
//	CAN1->sFilterRegister[0].FR2 = (0x302 << 21);
//	CAN1->FA1R |= (1 << 0);   // Kích hoạt Filter0

//	// Filter1: 0x303 + 0x100
//	CAN1->FM1R |= (1 << 1);   // Chế độ danh sách
//	CAN1->FS1R |= (1 << 1);   // 32-bit
//	CAN1->sFilterRegister[1].FR1 = (0x303 << 21);
//	CAN1->sFilterRegister[1].FR2 = (0x100 << 21);
//	CAN1->FA1R |= (1 << 1);   // Kích hoạt Filter1

//	CAN1->FFA1R &= ~((1 << 0) | (1 << 1));  // Đều đưa vào FIFO0
//	
//	CAN1->FMR &= ~1;  // Thoát chế độ cấu hình

		// Tạm thời thay thế cấu hình bộ lọc
CAN1->FMR |= 1;
CAN1->FM1R &= ~(1 << 0);       // Chế độ mặt nạ
CAN1->FS1R |= (1 << 0);        // 32-bit
CAN1->sFilterRegister[0].FR1 = 0;   // ID=0
CAN1->sFilterRegister[0].FR2 = 0;   // MASK=0 -> nhận tất cả
CAN1->FA1R |= (1 << 0);
CAN1->FFA1R &= ~(1 << 0);
CAN1->FMR &= ~1;
	
	// 6. Thoát khởi tạo
  CAN1->MCR &= ~(1 << 1);
  CAN1->MCR &= ~(1 << 0);
  while(CAN1->MSR & (1 << 0));
	
	
	NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn,NVIC_EncodePriority
	(NVIC_PriorityGroup_2,1,0));

		NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}



uint8_t CAN1_Send(CAN_Frame_t *frame) { return 0; } // Nút màn hình hiển thị chỉ nhận không gửi


uint8_t CAN1_Receive(CAN_Frame_t *frame)
{
    uint32_t temp;

    /* FIFO0 không có dữ liệu */
    if((CAN1->RF0R & 0x03) == 0)
    {
        return 0;
    }

    /* Tràn FIFO */
    if(CAN1->RF0R & (1 << 4))
    {
        CAN1->RF0R |= (1 << 4);
    }

    /* Chỉ đọc khung dữ liệu chuẩn */
    if(CAN1->sFIFOMailBox[0].RIR & (1 << 2))
    {
        CAN1->RF0R |= (1 << 5);
        return 0;
    }
		
		/* Khung từ xa (Remote frame) */
		if(CAN1->sFIFOMailBox[0].RIR & (1 << 1))
		{
				CAN1->RF0R |= (1 << 5);
				return 0;
		}

    frame->id =
        (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;

    frame->dlc =
        CAN1->sFIFOMailBox[0].RDTR & 0x0F;

    if(frame->dlc > 8)
    {
        frame->dlc = 8;
    }

    temp = CAN1->sFIFOMailBox[0].RDLR;

    frame->data[0] = temp & 0xFF;
    frame->data[1] = (temp >> 8) & 0xFF;
    frame->data[2] = (temp >> 16) & 0xFF;
    frame->data[3] = (temp >> 24) & 0xFF;

    temp = CAN1->sFIFOMailBox[0].RDHR;

    frame->data[4] = temp & 0xFF;
    frame->data[5] = (temp >> 8) & 0xFF;
    frame->data[6] = (temp >> 16) & 0xFF;
    frame->data[7] = (temp >> 24) & 0xFF;

    /* Giải phóng FIFO */
    CAN1->RF0R |= (1 << 5);

    return 1;
}



void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CAN_Frame_t frame;
    if(CAN1_Receive(&frame))
    {
        /* Trong ngắt chỉ nhận và cập nhật RAM, nghiêm cấm gọi printf hoặc làm mới OLED 1024 byte tại đây. */
        Dash_UpdateData(&frame);
    }
    CAN1->RF0R &= ~(1 << 3);
    CAN1->RF0R &= ~(1 << 4);
}
