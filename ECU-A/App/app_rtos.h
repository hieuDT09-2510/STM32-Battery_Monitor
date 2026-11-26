/*
 * Tên tệp: app_rtos.h
 *
 * Tên mô-đun: Giao diện công khai tổ chức các tác vụ FreeRTOS
 *
 * Trách nhiệm mô-đun:
 * 1. Cung cấp điểm vào để khởi tạo các tác vụ ứng dụng ra bên ngoài;
 * 2. Định nghĩa các biến chẩn đoán hoạt động RTOS để theo dõi trên Keil Watch;
 * 3. Tách biệt hàm main khỏi phần triển khai chi tiết của từng tác vụ.
 *
 * Giai đoạn chuyển đổi hiện tại:
 * 1. HealthTask độc lập kiểm tra nhịp tim (heartbeat) của các tác vụ trọng yếu và quản lý làm mới IWDG;
 * 2. SensorTask độc lập xử lý dữ liệu ADC/DMA và phát hành snapshot cảm biến;
 * 3. BmsTask độc lập chạy đánh giá lỗi, máy trạng thái và xử lý sự kiện trạng thái;
 * 4. CanTask độc lập gửi bản tin trạng thái điện áp (gồm SOC), nhiệt độ và lỗi;
 * 5. FaultLogTask độc lập chờ sự kiện lỗi và ghi vào Flash;
 * 6. DiagnosticTask xử lý CLI và chẩn đoán tài nguyên.
 */

#ifndef APP_RTOS_H
#define APP_RTOS_H

#include <stdint.h>


/*
 * Số lần cộng dồn mà tác vụ chẩn đoán đã chạy hoàn chỉnh.
 * Biến này tăng liên tục chứng tỏ các tác vụ FreeRTOS đang được lập lịch định kỳ.
 */
extern volatile uint32_t g_rtos_diagnostic_run_count;
/*
 * Số lần kiểm tra sức khỏe hệ thống mà HealthTask đã hoàn thành.
 * Trong điều kiện bình thường tăng khoảng mỗi 100ms một lần.
 */
extern volatile uint32_t g_rtos_health_run_count;
/*
 * Số lần cộng dồn mà SensorTask đã chạy hoàn chỉnh.
 *
 * Biến này ghi lại số lần tác vụ được lập lịch thực thi;
 * Trường sequence trong SensorSample ghi lại số lần lấy thành công dữ liệu ADC mới.
 * Hai giá trị này mang ý nghĩa khác nhau.
 */
extern volatile uint32_t g_rtos_sensor_run_count;

/*
 * Số lần cộng dồn mà BmsTask đã chạy hoàn chỉnh.
 *
 * Trong điều kiện bình thường tăng mỗi 10ms một lần;
 * Nếu máy trạng thái, xử lý sự kiện hoặc báo cáo heartbeat bị treo, bộ đếm này sẽ dừng lại.
 */
extern volatile uint32_t g_rtos_bms_run_count;

/*
 * Số chu kỳ gửi 100ms mà CanTask đã hoàn thành.
 * Trong điều kiện bình thường tăng khoảng 10 lần mỗi giây.
 */
extern volatile uint32_t g_rtos_can_run_count;
/*
 * Số lần CanTask thực hiện gửi khẩn cấp do trạng thái lỗi thay đổi.
 */
extern volatile uint32_t g_rtos_can_urgent_send_count;

/*
 * Số lượng sự kiện lỗi mà FaultLogTask đã xử lý.
 */
extern volatile uint32_t g_rtos_fault_log_processed_count;

/*
 * Số byte FreeRTOS Heap còn lại sau khi tạo xong toàn bộ tác vụ ứng dụng.
 */
extern volatile uint32_t g_rtos_free_heap_after_create;

/*
 * Không gian ngăn xếp (stack) còn lại tối thiểu trong lịch sử của từng tác vụ, đơn vị là phần tử stack 32-bit.
 */
extern volatile uint32_t g_rtos_health_stack_free_words;
extern volatile uint32_t g_rtos_sensor_stack_free_words;
extern volatile uint32_t g_rtos_bms_stack_free_words;
extern volatile uint32_t g_rtos_can_stack_free_words;
extern volatile uint32_t g_rtos_fault_log_stack_free_words;
extern volatile uint32_t g_rtos_diagnostic_stack_free_words;

/*
 * Số byte FreeRTOS Heap còn lại hiện tại trong quá trình vận hành.
 */
extern volatile uint32_t g_rtos_free_heap_current;


/*
 * Tạo toàn bộ các tác vụ FreeRTOS cần thiết cho giai đoạn hiện tại.
 *
 * Giá trị trả về:
 * 1: Tạo thành công toàn bộ tác vụ;
 * 0: Ít nhất một tác vụ tạo thất bại.
 *
 * Hàm này chỉ tạo tác vụ, không khởi động bộ lập lịch (scheduler).
 */
uint8_t AppRTOS_CreateTasks(void);


#endif
