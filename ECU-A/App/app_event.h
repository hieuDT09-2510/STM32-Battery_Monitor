/*
 * Tên tệp: app_event.h
 *
 * Tên mô-đun: Giao diện công khai hàng đợi sự kiện trạng thái BMS
 *
 * Chức năng mô-đun:
 * Lưu trữ các lệnh ngắn gọn như "bắt đầu giám sát, dừng giám sát, vào trạng thái lỗi, yêu cầu khôi phục",
 * sau đó Event_Process() chuyển lệnh cho BMS_EventHandle() để xử lý chuyển đổi trạng thái.
 *
 * Phạm vi mô-đun:
 * Mô-đun này không đánh giá điện áp hay nhiệt độ có bị lỗi hay không, cũng không lưu dữ liệu hiện trường khi xảy ra lỗi.
 * Việc ghi nhận lỗi chi tiết do app_fault_event đảm nhiệm, việc đánh giá điều kiện lỗi do app_fault_manager đảm nhiệm.
 *
 * Hạn chế đồng thời hiện tại:
 * Phiên bản bare-metal chỉ được thiết kế để gọi đơn luồng trong vòng lặp chính, không nên thao tác hàng đợi đồng thời từ ngắt và vòng lặp chính.
 * Sau khi chuyển sang FreeRTOS sẽ sử dụng FreeRTOS Queue để đảm bảo an toàn khi truyền giữa các tác vụ.
 */

#ifndef __APP_EVENT_H
#define __APP_EVENT_H
#include "stm32f10x.h"

/*
 * Mỗi giá trị enum đại diện cho một yêu cầu xử lý trạng thái.
 * Mã số 5 trước đây là sự kiện mô phỏng sạc đầy đã bị xóa; để tránh làm thay đổi các mã số hiện có, mã số 5 tiếp tục được để trống.
 */
typedef enum
{
    // Không có sự kiện.
    EVENT_NONE = 0,

    // Yêu cầu chuyển từ chế độ chờ sang giám sát.
    EVENT_START = 1,

    // Yêu cầu quay lại chế độ chờ từ giám sát.
    EVENT_STOP = 2,

    // Thông báo cho máy trạng thái về việc xảy ra lỗi.
    EVENT_FAULT = 3,

    // Yêu cầu kiểm tra điều kiện khôi phục lỗi.
    EVENT_RECOVER = 4,

    // Dự phòng cho sự kiện mở rộng lệnh CAN.
    EVENT_CAN_CMD = 6,

    // Dự phòng cho sự kiện mở rộng quá thời gian (timeout).
    EVENT_TIMEOUT = 7
} Event_t;


void Event_Init(void);						//Khởi tạo
void Event_Push(Event_t event);		        //Đẩy sự kiện vào cuối hàng đợi
Event_t Event_Pop(void);					//Lấy sự kiện từ đầu hàng đợi
void Event_Process(void);					//Xử lý tất cả các sự kiện đang chờ trong hàng đợi
uint8_t Event_Available(void);		//Kiểm tra hàng đợi có rỗng không

#endif
