/*
 * Tên file: bsp_can.h
 *
 * Chức năng mô-đun:
 * Cung cấp giao diện khởi tạo, truyền đồng bộ và nhận cho CAN1.
 *
 * CAN1_Send thông báo cho tầng trên thông qua giá trị trả về rõ ràng:
 * lần truyền này thành công hay thất bại ở giai đoạn nào.
 */

#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "stm32f10x.h"
#include "can_protocol.h"


/*
 * Kết quả truyền CAN.
 * Dùng tên định danh thay cho các số 0, 1, 2... để bên gọi không phải nhớ "magic number".
 */
typedef enum
{
    CAN_SEND_NO_MAILBOX = 0, // Cả ba mailbox truyền đều bận, lần này không gửi đi
    CAN_SEND_OK         = 1, // Bản tin truyền thành công
    CAN_SEND_TIMEOUT    = 2, // Quá thời gian chờ truyền hoàn tất
    CAN_SEND_ARB_LOST   = 3, // Mất phân xử (arbitration lost) trong quá trình truyền
    CAN_SEND_TX_ERROR   = 4, // Bộ điều khiển CAN báo lỗi truyền
    CAN_SEND_INVALID    = 5, // Tham số, ID hoặc kết quả truyền bất thường
    CAN_SEND_BUS_OFF    = 6  // Bộ điều khiển hiện đang ở trạng thái Bus-Off
    
} CAN_SendResult_t;

/*
 * Các biến chẩn đoán vận hành CAN.
 * Các biến này chỉ dùng để quan sát và phán đoán lỗi về sau, không tham gia nội dung bản tin.
 */
extern volatile CAN_SendResult_t g_can_last_send_result; // Kết quả truyền gần nhất
extern volatile uint32_t g_can_tx_ok_count;              // Số lần truyền thành công tích lũy
extern volatile uint32_t g_can_tx_fail_count;            // Số lần truyền thất bại tích lũy
extern volatile uint32_t g_can_bus_off_count;             // Số lần vào trạng thái Bus-Off tích lũy
extern volatile uint32_t g_can_recovery_count;            // Số lần khôi phục từ Bus-Off tích lũy
extern volatile uint32_t g_can_last_esr;                  // Thanh ghi trạng thái lỗi CAN đọc được gần nhất
extern volatile uint8_t g_can_bus_off_active;             // Hiện tại có đang ở trạng thái Bus-Off hay không

void CAN1_Init(void);

CAN_SendResult_t CAN1_Send(CAN_Frame_t *frame);

uint8_t CAN1_Receive(CAN_Frame_t *frame);


#endif
