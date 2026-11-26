/*
 * Tên file: bsp_can.c
 *
 * Tác dụng mô-đun:
 * Cấu hình CAN1 của STM32F103, cung cấp truyền và nhận đồng bộ khung dữ liệu tiêu chuẩn.
 *
 * Cấu hình hiện tại:
 * Baudrate 500kbps, sử dụng PA11 làm RX, PA12 làm TX.
 *
 * Nguyên tắc xử lý Bus-Off:
 * Bật khôi phục phần cứng tự động ABOM khi khởi tạo;
 * Hàm gửi chỉ phát hiện Bus-Off và trả về kết quả, không bắt buộc khởi động lại CAN trong quá trình gửi.
 *
 * Hiện tại việc gửi vẫn áp dụng chờ đồng bộ số lần hữu hạn.
 * Khi di chuyển lên FreeRTOS sau này, hàng đợi bản tin sẽ do task gửi CAN độc lập quản lý.
 */

#include "bsp_can.h"
#include "can_protocol.h"
#include "stm32f10x.h"
#include "bsp_usart.h"



/*
 * Dữ liệu chẩn đoán CAN.
 * Biến toàn cục được startup code xóa về 0 sau khi bật nguồn, sau đó do driver CAN cập nhật.
 */
volatile CAN_SendResult_t g_can_last_send_result = CAN_SEND_OK;
volatile uint32_t g_can_tx_ok_count = 0U;
volatile uint32_t g_can_tx_fail_count = 0U;
volatile uint32_t g_can_bus_off_count = 0U;
volatile uint32_t g_can_recovery_count = 0U;
volatile uint32_t g_can_last_esr = 0U;
volatile uint8_t g_can_bus_off_active = 0U;


/*
 * Ghi nhận tập trung kết quả một lần gửi CAN, và trả kết quả gốc về cho caller.
 *
 * Như vậy mỗi lối ra trong CAN1_Send đều được thống kê qua cùng một hàm,
 * tránh trường hợp thất bại nào đó quên cập nhật bộ đếm.
 */
static CAN_SendResult_t CAN1_RecordSendResult(CAN_SendResult_t result)
{
    g_can_last_send_result = result;
    g_can_last_esr = CAN1->ESR;

    if (result == CAN_SEND_OK)
    {
        g_can_tx_ok_count++;
    }
    else
    {
        g_can_tx_fail_count++;
    }

    return result;
}


// Khởi tạo CAN
void CAN1_Init(void)
{ 
	// 1. Bật clock GPIOA/AFIO alternate function/CAN1
	RCC->APB2ENR |= (1<<2);
	RCC->APB2ENR |= (1<<0);
	RCC->APB1ENR |= (1<<25);
	
	// 2. Cấu hình PA11-CAN_RX pull-up input / PA12-CAN_TX push-pull alternate function
	GPIOA->CRH &= ~(0xF << 12);
    GPIOA->CRH |=  (0x8 << 12);
    GPIOA->ODR |= (1 << 11);

    GPIOA->CRH &= ~(0xF << 16);
    GPIOA->CRH |=  (0xB << 16);
	
	// 3. Đi vào chế độ khởi tạo
	CAN1->MCR |= (1<<0); // Yêu cầu khởi tạo INRQ
	while((CAN1->MSR & (1<<0)) == 0); // Xác nhận khởi tạo INAK
	
	// 4. Cấu hình bit timing, baudrate 500kbps
	CAN1->BTR = 0;
	
	CAN1->BTR |= (0<<24);//SJW=1TQ->0
	CAN1->BTR |= (3<<20);//BS2=4TQ->3
	CAN1->BTR |= (12<<16);//BS1=13TQ->12
	CAN1->BTR |= (3<<0); //BRP=4TQ->3
	
	// 5. Cấu hình bộ lọc
	CAN1->FMR |= 1;  // Đi vào chế độ cấu hình
	

	// Filter0: 0x301 + 0x302
	CAN1->FM1R &= ~(1 << 0);
	CAN1->FM1R |= (1 << 0);   // Chế độ danh sách (identifier list)
	CAN1->FS1R &= ~(1 << 0);
	CAN1->FS1R |= (1 << 0);   // 32-bit
	CAN1->sFilterRegister[0].FR1 = (0x301 << 21);
	CAN1->sFilterRegister[0].FR2 = (0x302 << 21);
	CAN1->FA1R |= (1 << 0);   // Kích hoạt Filter0

	// Filter1: 0x303 + 0x100
	CAN1->FM1R |= (1 << 1);   // Chế độ danh sách
	CAN1->FS1R |= (1 << 1);   // 32-bit
	CAN1->sFilterRegister[1].FR1 = (0x303 << 21);
	CAN1->sFilterRegister[1].FR2 = (0x100 << 21);
	CAN1->FA1R |= (1 << 1);   // Kích hoạt Filter1

	CAN1->FFA1R &= ~((1 << 0) | (1 << 1));  // Đều vào FIFO0
	
	CAN1->FMR &= ~1;  // Thoát chế độ cấu hình
	
	CAN1->MCR |= (1 << 6);  // ABOM=1 Tự động phục hồi Bus-Off

	// 6. Thoát khỏi khởi tạo
    CAN1->MCR &= ~(1 << 1);
    CAN1->MCR &= ~(1 << 0);
	CAN1->MCR &= ~(1 << 4);
    while(CAN1->MSR & (1 << 0));
	
	printf("[CAN] Init done. MCR=0x%08X MSR=0x%08X BTR=0x%08X\r\n",
       CAN1->MCR, CAN1->MSR, CAN1->BTR);

}






// ECU-A gửi
CAN_SendResult_t CAN1_Send(CAN_Frame_t *frame)
{
	
	uint32_t esr;
    uint8_t mailbox = 0;
    uint32_t dl, dh;

    uint32_t timeout;

    uint32_t rqcp_mask;
    uint32_t txok_mask;
    uint32_t alst_mask;
    uint32_t terr_mask;

    /*
    * Mỗi lần trước khi gửi chỉ đọc thanh ghi trạng thái lỗi một lần.
    * Nếu controller vẫn ở trạng thái Bus-Off, lần này không thao tác mailbox truyền.
    *
    * ABOM sẽ tự động xóa Bus-Off sau khi bus thỏa mãn điều kiện phục hồi,
    * các lần gọi CAN1_Send chu kỳ sau sẽ thử gửi lại.
    */
    esr = CAN1->ESR;
    g_can_last_esr = esr;

    /*
    * Bit 2 BOFF của ESR bằng 1 biểu thị controller CAN đã rút khỏi truyền thông bus.
    * Chỉ tích lũy một lần khi trạng thái chuyển từ "bình thường" sang "Bus-Off".
    */
    if ((esr & (1U << 2)) != 0U)
    {
        if (g_can_bus_off_active == 0U)
        {
            g_can_bus_off_active = 1U;
            g_can_bus_off_count++;
        }

        return CAN1_RecordSendResult(CAN_SEND_BUS_OFF);
    }

    /*
    * Lần kiểm tra trước vẫn ở Bus-Off mà lần này BOFF đã xóa về 0,
    * chứng tỏ ABOM đã hoàn thành một lần tự động phục hồi.
    */
    if (g_can_bus_off_active != 0U)
    {
        g_can_bus_off_active = 0U;
        g_can_recovery_count++;
    }
		
    /* DLC tối đa 8 byte */
    if(frame->dlc > 8)
    {
        frame->dlc = 8;
    }

    /* Chỉ cho phép standard frame ID */
    if(frame->id > 0x7FF)
    {
        return CAN1_RecordSendResult(CAN_SEND_INVALID);
    }

    /*--------------------------------------------------
      Tìm mailbox truyền rảnh
      TME0 -> bit26
      TME1 -> bit27
      TME2 -> bit28
    --------------------------------------------------*/
          
    if(CAN1->TSR & (1 << 26))
    {
        mailbox = 0;
    }
    else if(CAN1->TSR & (1 << 27))
    {
        mailbox = 1;
    }
    else if(CAN1->TSR & (1 << 28))
    {
        mailbox = 2;
    }
    else
    {
        return  CAN1_RecordSendResult(CAN_SEND_NO_MAILBOX);
    }

    /*--------------------------------------------------
      Xóa thanh ghi điều khiển mailbox
    --------------------------------------------------*/
    CAN1->sTxMailBox[mailbox].TIR = 0;

    /*--------------------------------------------------
      Cài đặt standard frame ID
      Standard ID nằm ở [31:21]
    --------------------------------------------------*/
    CAN1->sTxMailBox[mailbox].TIR |= (frame->id << 21);

    /*--------------------------------------------------
      Cài đặt độ dài dữ liệu
    --------------------------------------------------*/
    CAN1->sTxMailBox[mailbox].TDTR = frame->dlc;
		
		
		

    /*--------------------------------------------------
      Đóng gói 4 byte đầu
    --------------------------------------------------*/
    dl =
        ((uint32_t)frame->data[0]) |
        ((uint32_t)frame->data[1] << 8) |
        ((uint32_t)frame->data[2] << 16) |
        ((uint32_t)frame->data[3] << 24);

    /*--------------------------------------------------
      Đóng gói 4 byte sau
    --------------------------------------------------*/
    dh =
        ((uint32_t)frame->data[4]) |
        ((uint32_t)frame->data[5] << 8) |
        ((uint32_t)frame->data[6] << 16) |
        ((uint32_t)frame->data[7] << 24);

    CAN1->sTxMailBox[mailbox].TDLR = dl;
    CAN1->sTxMailBox[mailbox].TDHR = dh;

    /*--------------------------------------------------
      Chọn bit trạng thái theo mailbox
    --------------------------------------------------*/
    if(mailbox == 0)
    {
        rqcp_mask = (1 << 0);
        txok_mask = (1 << 1);
        alst_mask = (1 << 2);
        terr_mask = (1 << 3);
    }
    else if(mailbox == 1)
    {
        rqcp_mask = (1 << 8);
        txok_mask = (1 << 9);
        alst_mask = (1 << 10);
        terr_mask = (1 << 11);
    }
    else
    {
        rqcp_mask = (1 << 16);
        txok_mask = (1 << 17);
        alst_mask = (1 << 18);
        terr_mask = (1 << 19);
    }

    /*--------------------------------------------------
      Yêu cầu gửi
      TXRQ = 1
    --------------------------------------------------*/

    CAN1->sTxMailBox[mailbox].TIR |= 1;

    /*--------------------------------------------------
      Chờ gửi hoàn tất
    --------------------------------------------------*/
    timeout = 500000;

    while(!(CAN1->TSR & rqcp_mask))
    {
       /*
        * Trong thời gian chờ gửi cũng kiểm tra Bus-Off.
        * Khi không có ACK, controller CAN có thể đi vào Bus-Off ngay trong quá trình gửi lần này.
        */
        esr = CAN1->ESR;
        g_can_last_esr = esr;

        if ((esr & (1U << 2)) != 0U)
        {
            /*
            * Chỉ tích lũy một lần tại khoảnh khắc chuyển từ bình thường sang Bus-Off,
            * kiểm tra lặp lại sau đó sẽ không tăng trùng lặp.
            */
            if (g_can_bus_off_active == 0U)
            {
                g_can_bus_off_active = 1U;
                g_can_bus_off_count++;
            }

            /*
            * Hủy bỏ bản tin chưa hoàn thành trong mailbox hiện tại,
            * tránh việc sau khi bus phục hồi lại tự động gửi bản tin cũ này.
            */
            if (mailbox == 0U)
            {
                CAN1->TSR = (1U << 7);   // ABRQ0
            }
            else if (mailbox == 1U)
            {
                CAN1->TSR = (1U << 15);  // ABRQ1
            }
            else
            {
                CAN1->TSR = (1U << 23);  // ABRQ2
            }

            return CAN1_RecordSendResult(CAN_SEND_BUS_OFF);
        }
        
        if(--timeout == 0)
					{
						
						printf("TIMEOUT TSR=0x%08X ESR=0x%08X\r\n",
           CAN1->TSR,
           CAN1->ESR);
						
						/* Timeout: Dùng ABRQ hủy gửi, giải phóng mailbox */
						if(mailbox == 0) CAN1->TSR |= (1 << 7);   // ABRQ0
						if(mailbox == 1) CAN1->TSR |= (1 << 15);  // ABRQ1
						if(mailbox == 2) CAN1->TSR |= (1 << 23);  // ABRQ2
						
						/* Chờ hủy hoàn tất (phần cứng tự động khôi phục bit TME) */
						for(volatile int i = 0; i < 5000; i++);
						
						/* Xóa bit trạng thái */
						CAN1->TSR |= rqcp_mask | txok_mask;
						
						return  CAN1_RecordSendResult(CAN_SEND_TIMEOUT);
					}
			
    }

    /*--------------------------------------------------
      Phán đoán kết quả gửi
    --------------------------------------------------*/
    if(CAN1->TSR & txok_mask)
    {
        /* Xóa bit trạng thái */
        CAN1->TSR |=
            rqcp_mask |
            txok_mask |
            alst_mask |
            terr_mask;

        return  CAN1_RecordSendResult(CAN_SEND_OK);
    }

    if(CAN1->TSR & alst_mask)
    {
        /* Mất phân xử (arbitration lost) */
        CAN1->TSR |=
            rqcp_mask |
            txok_mask |
            alst_mask |
            terr_mask;

        return CAN1_RecordSendResult(CAN_SEND_ARB_LOST);
    }

    if(CAN1->TSR & terr_mask)
    {
        /* Lỗi truyền */
        CAN1->TSR |=
            rqcp_mask |
            txok_mask |
            alst_mask |
            terr_mask;

        return CAN1_RecordSendResult(CAN_SEND_TX_ERROR);
    }

    /* Lỗi không xác định */
    CAN1->TSR |=
        rqcp_mask |
        txok_mask |
        alst_mask |
        terr_mask;

    return CAN1_RecordSendResult(CAN_SEND_INVALID);
}






uint8_t CAN1_Receive(CAN_Frame_t *frame)
{
    uint32_t temp;

    /* FIFO0 không có dữ liệu */
    if((CAN1->RF0R & 0x03) == 0)
    {
        return 0;
    }

    /* FIFO tràn (overflow) */
    if(CAN1->RF0R & (1 << 4))
    {
        CAN1->RF0R |= (1 << 4);
    }

    /* Chỉ đọc standard data frame */
    if(CAN1->sFIFOMailBox[0].RIR & (1 << 2))
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
