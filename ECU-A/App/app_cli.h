/*
 * Tên tệp: app_cli.h
 *
 * Tên mô-đun: Giao diện chẩn đoán dòng lệnh USART
 *
 * Trách nhiệm của mô-đun:
 * 1. Đọc byte không chặn từ bộ đệm vòng USART RX và ghép thành lệnh hoàn chỉnh;
 * 2. Truy vấn trạng thái Sensor, BMS, FaultManager, CAN và nhật ký Flash;
 * 3. Thực hiện các thao tác chẩn đoán như tiêm lỗi, xóa nhật ký, kiểm tra watchdog và đặt lại phần mềm;
 * 4. Xuất kết quả chẩn đoán ra cổng nối tiếp để dễ đọc và khoanh vùng sự cố.
 *
 * Tác vụ FreeRTOS hiện tại:
 * CLI_Init() và CLI_Process() được gọi bởi DiagnosticTask. Ngắt USART chỉ ghi các byte
 * nhận được vào bộ đệm vòng BSP, không phân tích lệnh hay thực hiện tác vụ tốn thời gian trong ngắt.
 *
 * Ranh giới với trình quản lý lỗi:
 * Tiêm lỗi qua CLI chỉ thiết lập điều kiện kiểm thử, không sửa dữ liệu ADC và không sửa trực tiếp mã lỗi đang hoạt động;
 * các bước xác nhận, chốt giữ, khôi phục, thông báo CAN và ghi nhật ký sau đó vẫn do luồng xử lý bình thường thực hiện.
 */
#ifndef APP_CLI_H
#define APP_CLI_H

#include <stdint.h>

/* Tổng số lệnh đã gửi thành công tới bộ điều phối lệnh, tính cả lệnh đã biết và chưa biết. */
extern volatile uint32_t g_cli_command_count;

/* Số lần tích lũy gặp lệnh không nhận dạng được hoặc tên tiêm lỗi không được hỗ trợ. */
extern volatile uint32_t g_cli_unknown_count;

/* Xóa bộ đệm dòng và các bộ đếm chẩn đoán, đồng thời xuất thông báo CLI sẵn sàng và dấu nhắc đầu tiên. */
void CLI_Init(void);

/* Xử lý không chặn một số lượng byte giới hạn trong bộ đệm vòng RX, được gọi định kỳ bởi DiagnosticTask. */
void CLI_Process(void);

#endif
