/*
 * Tên tệp: app_dash.h
 * Chức năng mô-đun: Lưu trữ dữ liệu hiển thị nhận từ CAN và trạng thái truyền thông của từng bản tin trên ECU-B.
 * 
 * 0x301 và 0x302 lưu thời gian nhận riêng biệt và phán đoán quá thời gian truyền thông (timeout giao tiếp) độc lập,
 * việc nhận được bản tin khác không được che khuất tình trạng mất bản tin trạng thái quan trọng hoặc bản tin nhiệt độ.
 *
 * 0x301 và 0x302 kiểm tra riêng CRC8 và Alive Counter:
 * CRC dùng để đánh giá nội dung bản tin hiện tại có đáng tin cậy hay không;
 * Alive Counter dùng để kiểm tra bản tin có bị lặp, nhảy số thứ tự hay bị mất hay không.
 * 
 * Trạng thái truyền thông và trạng thái thông báo trên OLED được lưu riêng để thông báo vẫn tiếp tục hiển thị thêm một khoảng thời gian sau khi truyền thông khôi phục.
 * Đồng thời kiểm tra Alive Counter 4-bit của 0x301, ghi nhận trạng thái nhảy số hiện tại và số lần bất thường tích lũy.
 * 
 */
#ifndef __APP_DASH_H
#define __APP_DASH_H
#include "stm32f10x.h"
#include "can_protocol.h"


/*
 * Quy trình xử lý nhiệt độ:
 Nhận được bản tin nhiệt độ
    ↓
Kiểm tra độ dài có đúng 8 byte hay không
    ↓
Tính lại CRC8 của 7 byte đầu tiên
    ↓
CRC có khớp không?
 ┌──Không: Từ chối toàn bộ khung, không cập nhật nhiệt độ và thời gian
 ↓Có
Tách Alive Counter riêng của bản tin nhiệt độ
    ↓
So sánh với số thứ tự khung trước + 1
    ↓
Liên tục: Xóa cờ bất thường Alive hiện tại
Nhảy số: Ghi nhận số lần bất thường, nhưng vẫn sử dụng giá trị nhiệt độ hiện tại
    ↓
Lưu nhiệt độ, mốc so sánh Alive và thời gian nhận
*/



/*
 Nhóm thời gian:
last_30x_rx_tick

Nhóm Alive Counter:
received_30x
last_30x_alive
alive_30x_error
alive_30x_error_count

Nhóm CRC:
crc_30x_error
crc_30x_error_count
last_30x_received_crc
last_30x_calculated_crc
*/
typedef struct
{
    uint16_t cell_mv;          // Điện áp (mV)

    uint8_t  chg_state;        // Trạng thái sạc/xả

    uint8_t  temp;             // Nhiệt độ

    uint8_t  soc;              // SOC mang theo bởi 0x301 (ước lượng tuyến tính từ điện áp), 0xFF biểu thị không hợp lệ

    uint16_t fault;            // Mặt nạ lỗi đang kích hoạt 16-bit mang theo bởi 0x303

    volatile uint8_t valid;    // Đã từng nhận được dữ liệu hay chưa

    uint32_t last_rx_tick;     // Thời điểm cuối cùng nhận được bất kỳ bản tin BMS hợp lệ đã nhận diện

    uint32_t last_301_rx_tick; // Thời điểm gần nhất nhận được 0x301, dùng để phán đoán timeout độc lập
    uint32_t last_302_rx_tick;  // Thời điểm gần nhất nhận được bản tin nhiệt độ có CRC đúng
    uint32_t last_303_rx_tick;  // Thời điểm gần nhất nhận được bản tin lỗi có CRC đúng

    uint32_t comm_301_notice_since_ms; // Thời điểm bắt đầu hiển thị thông báo mất 0x301 lần này

    uint32_t alive_301_error_count; // Số lần tích lũy 0x301 không liên tục số thứ tự, chỉ tăng không xóa sau khi cấp nguồn
    uint32_t crc_301_error_count; // Số lần tích lũy kiểm tra CRC thất bại. Không xóa về 0 sau khi hiện tại khôi phục bình thường, dùng để chẩn đoán lịch sử.

    uint32_t alive_302_error_count; // Số lần tích lũy bản tin nhiệt độ không liên tục số thứ tự.
    uint32_t crc_302_error_count;   // Số lần tích lũy bản tin nhiệt độ kiểm tra CRC thất bại.

    uint32_t alive_303_error_count; // Số lần tích lũy bản tin lỗi không liên tục số thứ tự
    uint32_t crc_303_error_count;   // Số lần tích lũy bản tin lỗi kiểm tra CRC thất bại


    uint8_t received_301;      // 0=sau khi cấp nguồn chưa từng nhận 0x301, 1=đã nhận thành công ít nhất một lần
    uint8_t received_302;      // 0=sau khi cấp nguồn chưa từng nhận 0x302, 1=đã nhận thành công ít nhất một lần
    uint8_t received_303;     // 0=chưa từng nhận bản tin lỗi hợp lệ, 1=đã nhận ít nhất một lần

    uint8_t comm_301_timeout;  // 1=0x301 đã không đến trong 500ms liên tiếp
    uint8_t comm_302_timeout;  //  1=liên tiếp 1500ms không nhận được bản tin nhiệt độ có CRC đúng
    uint8_t comm_303_timeout;  //  1=liên tiếp 500ms không nhận được bản tin lỗi có CRC đúng

    uint8_t comm_301_notice_active; // 1=OLED vẫn cần hiển thị thông báo mất 0x301

    uint8_t last_301_alive;     // Alive Counter mang theo trong khung 0x301 gần nhất (0~15)
    uint8_t last_302_alive;     // Alive Counter mang theo trong khung 0x302 gần nhất (0~15)
    uint8_t last_303_alive;    // Alive Counter trong khung bản tin lỗi hợp lệ gần nhất

    uint8_t alive_301_error;    // 1=lần này 0x301 không liên tục số thứ tự, tự động xóa về 0 khi khung tiếp theo liên tục
    uint8_t alive_302_error;    // 1=lần này 0x302 không liên tục số thứ tự, tự động xóa về 0 khi khung tiếp theo liên tục
    uint8_t alive_303_error;    // 1=lần này bản tin lỗi không liên tục số thứ tự, xóa về 0 khi khung tiếp theo liên tục
   /*
    * 1 biểu thị bản tin trạng thái quan trọng nhận được gần nhất bị lỗi CRC;
    * Tự động khôi phục về 0 khi khung tiếp theo có CRC đúng.
    */
    uint8_t crc_301_error;  // Khung gần nhất có lỗi CRC hay không
    uint8_t crc_302_error;  // Khung nhiệt độ gần nhất có lỗi CRC hay không
    uint8_t crc_303_error;  // 1=khung bản tin lỗi gần nhất bị lỗi CRC

    /*
    * CRC mang theo trong khung bản tin gần nhất.
    * Chủ yếu dùng để quan sát trong Keil Watch.
    */
    uint8_t last_301_received_crc; // CRC do bản tin tự mang theo
    uint8_t last_302_received_crc;// CRC mang theo trong khung nhiệt độ gần nhất.
    uint8_t last_303_received_crc; // CRC mang theo trong khung lỗi gần nhất

    /*
    * CRC do ECU-B tính lại dựa trên 7 byte đầu tiên.
    * Trong trường hợp bình thường phải bằng với received_crc.
    */
    uint8_t last_301_calculated_crc; // CRC điện áp do ECU-B tính lại
    uint8_t last_302_calculated_crc; // CRC do ECU-B tính lại dựa trên 7 byte đầu của bản tin nhiệt độ.
    uint8_t last_303_calculated_crc; // CRC bản tin lỗi do ECU-B tính lại
} BMS_DisplayData_t;

// Ngắt CAN ghi vào, vòng lặp chính đọc ra; volatile yêu cầu mỗi lần đều lấy giá trị từ bộ nhớ thực tế.
extern volatile BMS_DisplayData_t g_dash_data;

void Dash_Init(void);
void Dash_UpdateData(CAN_Frame_t *frame);
void Dash_UpdateCommStatus(uint32_t now_ms);
void Dash_RefreshDisplay(void);

#endif





 
 
