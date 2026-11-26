#ifndef __APP_BMS_STATE_H
#define __APP_BMS_STATE_H

#include "stm32f10x.h"
#include "app_event.h"     // Đưa vào định nghĩa enum Event_t


/*
 * Mô-đun: Định nghĩa bit lỗi BMS.
 * Chức năng: Thống nhất mã lỗi dùng cho máy trạng thái, tránh viết trực tiếp số thập lục phân.
 * Mã hóa: Mỗi loại lỗi chiếm một bit nhị phân độc lập, có thể kết hợp bằng phép toán OR theo bit.
 * Tương thích: Giữ nguyên giá trị của lỗi quá áp, thấp áp, quá nhiệt và tự kiểm tra thất bại trước đây.
 * Lỗi hở mạch/ngắn mạch NTC đã được Fault Manager quản lý xác nhận và phục hồi dựa trên bản chụp Sensor.
 */

// Không có lỗi.
#define BMS_FAULT_NONE          0x00U

// Bit0: Quá áp pin.
#define BMS_FAULT_OV            0x01U

// Bit1: Thấp áp pin.
#define BMS_FAULT_UV            0x02U

// Bit2: Quá nhiệt pin.
#define BMS_FAULT_OT            0x04U

// Bit3: Lỗi hở mạch NTC.
#define BMS_FAULT_NTC_OPEN      0x08U

// Bit4: Lỗi ngắn mạch NTC.
#define BMS_FAULT_NTC_SHORT     0x10U

// Bit7: Tự kiểm tra thất bại.
#define BMS_FAULT_SELF_CHECK    0x80U

extern uint16_t g_fault_code;  // Mặt nạ lỗi đang hoạt động 16-bit hiện tại, khớp với CAN 0x303

/*
 * Mô-đun: Định nghĩa trạng thái hệ thống giám sát pin.
 * Chức năng: Mô tả các giai đoạn khởi tạo, tự kiểm tra, chờ, giám sát và cảnh báo lỗi.
 * Hiện tại chỉ có công tắc tải trên mạch TPS22919, không có cặp MOS sạc/xả đấu lưng (back-to-back),
 * do đó không định nghĩa đầy đủ các trạng thái thực thi bảo vệ sạc và xả.
 * Giữ nguyên mã số của các trạng thái hợp lệ trước đây
 */
typedef enum
{
    // Khởi tạo khi cấp nguồn, không có nghĩa là ngắt nguồn phần cứng.
    BMS_INIT = 0,

    // Kiểm tra xem hệ thống có đáp ứng điều kiện vào chế độ chờ hay không.
    BMS_SELF_CHECK = 1,

    // Chờ yêu cầu bắt đầu giám sát.
    BMS_STANDBY = 2,

    // Thực hiện giám sát điện áp, nhiệt độ và gửi báo cáo dữ liệu.
    BMS_MONITOR = 3,

    // Thực hiện cảnh báo lỗi và ngắt tải trên mạch, không có nghĩa là cắt toàn bộ dòng sạc/xả bên ngoài.
    BMS_FAULT = 4
} BMS_State_t;



void BMS_SetState(BMS_State_t state);           // Thiết lập trạng thái hiện tại
BMS_State_t BMS_GetState(void);                 // Lấy trạng thái hiện tại
void BMS_StateMachine(void);                    // Điểm vào máy trạng thái (được gọi bởi bộ lập lịch)
void BMS_EventHandle(Event_t event);            // Xử lý sự kiện (được gọi bởi Event_Process)



#endif
