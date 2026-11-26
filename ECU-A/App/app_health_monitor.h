/*
 * Tên tệp: app_health_monitor.h
 *
 * Tên mô-đun: Giao diện công khai giám sát sức khỏe hệ thống và Watchdog độc lập
 *
 * Nhiệm vụ của mô-đun:
 * 1. Định nghĩa các bit nhịp tim sức khỏe của các tác vụ trọng yếu;
 * 2. Tiếp nhận nhịp tim vận hành của tác vụ Sensor, BMS và CAN;
 * 3. Cung cấp điểm vào kiểm tra sức khỏe chu kỳ 100ms;
 * 4. Chỉ cho phép làm mới IWDG khi toàn bộ tác vụ trọng yếu đều bình thường.
 *
 * Phạm vi mô-đun:
 * Mô-đun này không xử lý dữ liệu cảm biến, không đánh giá lỗi pin,
 * không truyền bản tin CAN, cũng không thực thi các tác vụ nghiệp vụ cụ thể.
 *
 * Quy tắc đồng thời hiện tại:
 * Nhiều tác vụ FreeRTOS có thể gọi HealthMonitor_Report();
 * việc sửa đổi đồng thời các bit nhịp tim được bảo vệ bởi vùng găng bên trong mô-đun.
 */
#ifndef APP_HEALTH_MONITOR_H
#define APP_HEALTH_MONITOR_H

#include <stdint.h>


/* Báo cáo khi Sensor Service tạo thành công snapshot mới. */
#define HEALTH_SOURCE_SENSOR       ((uint8_t)0x01U)

/* Báo cáo sau khi tác vụ cốt lõi BMS 10ms thực thi hoàn tất và trả về. */
#define HEALTH_SOURCE_BMS          ((uint8_t)0x02U)

/*
 * Báo cáo sau khi CanTask hoàn thành một lần truyền chu kỳ 100ms.
 */
#define HEALTH_SOURCE_CAN          ((uint8_t)0x04U)


/*
 * Toàn bộ nhịp tim của các tác vụ trọng yếu bắt buộc phải xuất hiện trong mỗi cửa sổ kiểm tra 100ms.
 * Sensor = 0x01  Nhị phân: 0000 0001
 * BMS    = 0x02  Nhị phân: 0000 0010
 * CAN    = 0x04  Nhị phân: 0000 0100
 * --------------------------------
 * Tổng   = 0x07  Nhị phân: 0000 0111
 */
#define HEALTH_REQUIRED_MASK       \
    ((uint8_t)(HEALTH_SOURCE_SENSOR | \
               HEALTH_SOURCE_BMS | \
               HEALTH_SOURCE_CAN))


/* Nhịp tim đã nhận được trong cửa sổ kiểm tra hiện tại. */
extern volatile uint8_t g_health_seen_mask;

/* Nhịp tim thực tế đã nhận được trong cửa sổ kiểm tra trước. */
extern volatile uint8_t g_health_last_window_mask;

/* Số lần tích lũy làm mới IWDG thành công. */
extern volatile uint32_t g_health_feed_count;

/* Số lần tích lũy nhịp tim không đầy đủ, không làm mới IWDG. */
extern volatile uint32_t g_health_missed_count;

/*
 * Công tắc kiểm thử watchdog.
 * Khi đặt bằng 1, Health Monitor ngừng làm mới IWDG để xác minh reset phần cứng.
 */
extern volatile uint8_t g_health_test_stop_feed;


/* Khởi tạo trạng thái sức khỏe và Watchdog độc lập của STM32. */
void HealthMonitor_Init(void);

/*
 * Báo cáo bit nhịp tim tương ứng sau khi tác vụ trọng yếu thực thi thành công trọn vẹn.
 *
 * Giao diện hiện tại dùng vùng găng tác vụ FreeRTOS, chỉ được phép gọi trong ngữ cảnh tác vụ,
 * không được phép gọi trực tiếp trong ngắt.
 */
void HealthMonitor_Report(uint8_t source_mask);

/* Kiểm tra nhịp tim mỗi 100ms một lần và quyết định có làm mới IWDG hay không. */
void HealthMonitor_Run(void);


#endif
