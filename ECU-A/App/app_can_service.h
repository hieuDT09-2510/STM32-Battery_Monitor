/*
 * Tên tệp: app_can_service.h
 *
 * Tên mô-đun: Giao diện công khai dịch vụ ứng dụng CAN
 *
 * Trách nhiệm mô-đun:
 * 1. Cung cấp ra bên ngoài các giao diện gửi bản tin trạng thái điện áp, nhiệt độ, lỗi và dòng điện;
 * 2. Công khai Alive Counter riêng của bốn loại bản tin để chẩn đoán trên Keil Watch;
 * 3. Tách biệt mô-đun nghiệp vụ BMS khỏi việc đóng gói giao thức CAN và thao tác truyền phần cứng CAN.
 *
 * Giới hạn mô-đun:
 * Mô-đun này không thu thập ADC, không quy đổi NTC, không phán đoán lỗi pin,
 * không chuyển đổi trạng thái BMS và cũng không xử lý trực tiếp hiển thị ECU-B.
 *
 * Giai đoạn chuyển đổi hiện tại:
 * Bốn loại bản tin đã được gọi định kỳ bởi CanTask độc lập;
 * các tác vụ khác không được gọi trực tiếp giao diện gửi.
 * 
 * Ràng buộc đồng thời hiện tại:
 * Mọi thao tác gửi CAN định kỳ đều do CanTask thực hiện;
 * các tác vụ khác không được gọi trực tiếp AppCAN_Send...() hoặc CAN1_Send().
 */
#ifndef APP_CAN_SERVICE_H
#define APP_CAN_SERVICE_H

#include <stdint.h>

/*
 * Gửi điện áp, SOC và trạng thái giám sát bằng bản chụp cảm biến hợp lệ mới nhất.
 */
void AppCAN_SendVoltageStatus(void);

/*
 * Gửi trạng thái nhiệt độ bằng bản chụp cảm biến hợp lệ mới nhất.
 */
void AppCAN_SendTemperatureStatus(void);

/*
 * Gửi đầy đủ trạng thái lỗi đang hoạt động hiện tại.
 */
void AppCAN_SendFaultStatus(void);

/*
 * Ba loại bản tin có Alive Counter độc lập riêng.
 *
 * Các biến này chỉ được công khai phục vụ chẩn đoán trên Keil Watch,
 * mô-đun nghiệp vụ không được sửa đổi trực tiếp.
 */
extern volatile uint8_t g_can_301_alive_counter;
extern volatile uint8_t g_can_302_alive_counter;
extern volatile uint8_t g_can_303_alive_counter;

#endif
