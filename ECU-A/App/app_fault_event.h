/*
 * Tên file: app_fault_event.h
 *
 * Tên mô-đun: Public API hàng đợi sự kiện sườn lỗi
 *
 * Trách nhiệm mô-đun:
 * 1. Định nghĩa định dạng dữ liệu cho 3 loại sự kiện: "xác nhận mới", "xóa mới" và "khởi động hệ thống".
 *
 * 2. Lưu ngữ cảnh độ rộng cố định tại thời điểm trạng thái lỗi biến đổi,
 *    bao gồm:
 *    - Thời gian xảy ra sự kiện
 *    - Bit lỗi biến đổi lần này
 *    - Bit lỗi vẫn còn tồn tại sau biến đổi
 *    - Snapshot điện áp pin
 *    - Snapshot nhiệt độ
 *    - Trạng thái máy trạng thái BMS
 *    - Tính hiệu lực của dữ liệu cảm biến
 *
 * 3. Cung cấp nguồn dữ liệu sự kiện lịch sử lưu theo thứ tự cho log Flash.
 *
 * Bổ sung giải thích:
 * FaultEvent_t cũng được log khởi động tái sử dụng, nhưng SYSTEM_BOOT không đi vào hàng đợi RAM của mô-đun này;
 * bản ghi khởi động do FlashLog trực tiếp cấu tạo và ghi vào Flash trong giai đoạn khởi tạo.
 *
 * Ranh giới mô-đun:
 * Mô-đun này chỉ chịu trách nhiệm "lưu và truyền đạt sự kiện lỗi đã phát sinh", không chịu trách nhiệm:
 *
 * - Phán đoán điện áp nào đó có thuộc quá áp không;
 * - Phán đoán nhiệt độ nào đó có thuộc quá nhiệt không;
 * - Thực hiện trễ xác nhận lỗi hoặc chống rung (debounce);
 * - Sửa đổi máy trạng thái BMS;
 * - Điều khiển cơ cấu chấp hành như relay, MOS, còi báo;
 * - Trực tiếp truy cập SPI Flash;
 * - Trực tiếp gửi bản tin CAN;
 * - Quyết định chiến lược phục hồi lỗi.
 *
 * Nói cách khác:
 *
 *     Mô-đun phát hiện lỗi
 *          ↓
 *     Phát hiện sườn lỗi
 *          ↓
 *     FaultEvent_Push()
 *          ↓
 *     Hàng đợi sự kiện RAM
 *          ↓
 *     Task FaultLog / Chẩn đoán kết quả ghi qua USART
 *
 * Mô hình đồng thời:
 * Phiên bản bare-metal trước đây thiết kế theo kịch bản single-producer, single-consumer.
 *
 * Mô hình đồng thời:
 * Bên trong sử dụng Queue tĩnh FreeRTOS.
 *
 * Producer hiện tại là luồng phán đoán lỗi BMS,
 * Consumer hiện tại là luồng xử lý log lỗi.
 *
 * En-queue và de-queue có thể được gọi từ các task khác nhau;
 * Phía gọi không cần trực tiếp thao tác chỉ số hàng đợi, cũng không cần thêm critical section phụ.
 */

#ifndef APP_FAULT_EVENT_H 
#define APP_FAULT_EVENT_H 
 
#include <stdint.h> 
#include "FreeRTOS.h"
#include "app_sensor.h" 
 
// Hàng đợi cố định lưu 8 sự kiện, bên trong dùng bộ nhớ tĩnh, không cấp phát động từ Heap 
#define FAULT_EVENT_QUEUE_CAPACITY       8U 
 
// Bit0 của valid_flags: Điện áp pin trong sự kiện hợp lệ. 
#define FAULT_EVENT_VALID_VOLTAGE        ((uint8_t)0x01U) 
 
// Bit1 của valid_flags: Nhiệt độ trong sự kiện hợp lệ. 
#define FAULT_EVENT_VALID_TEMPERATURE    ((uint8_t)0x02U) 

// Bit2 của valid_flags: Trong sự kiện có lưu SOC tích phân Coulomb tại thời điểm xảy ra lỗi.
#define FAULT_EVENT_VALID_SOC            ((uint8_t)0x04U)

// Bit7 của valid_flags: Sự kiện xác nhận mới lần này đến từ test inject của CLI. 
#define FAULT_EVENT_FLAG_INJECTED         ((uint8_t)0x80U) 
 
// Phân biệt 3 loại sự kiện log: xác nhận lỗi, xóa lỗi và khởi động hệ thống. 
typedef enum 
{ 
    FAULT_EVENT_CONFIRMED = 1, // Xác nhận lỗi
    FAULT_EVENT_CLEARED = 2, // Xóa lỗi
    FAULT_EVENT_SYSTEM_BOOT = 3 // Khởi động hệ thống
} FaultEventType_t; 
 
/* 
 * Snapshot độ rộng cố định của một sự kiện hệ thống.
 * Sự kiện lỗi dùng changed_mask mô tả biến đổi lần này, dùng active_mask mô tả lỗi còn lại;
 * sự kiện khởi động không có biến đổi lỗi, do đó cả 2 mặt nạ đều là 0.
 */ 
typedef struct 
{ 
    uint32_t timestamp_ms; // Thời gian xảy ra sự kiện, đơn vị ms
    uint16_t changed_mask; // Lỗi mới phát sinh hoặc mới xóa lần này
    uint16_t active_mask;  // Toàn bộ các lỗi vẫn còn tồn tại sau biến đổi
    uint16_t cell_voltage_mv; // Snapshot điện áp pin, đơn vị mV
    int16_t temperature_dC; // Snapshot nhiệt độ, đơn vị 0.1°C
    /* Sử dụng lưu trữ 1 byte rõ ràng, tránh các trình biên dịch khác nhau làm đổi layout bản ghi Flash. */ 
    uint8_t event_type;  // 1=Xác nhận mới, 2=Xóa mới, 3=Khởi động hệ thống
    uint8_t bms_state;  // Trạng thái máy trạng thái BMS, phạm vi giá trị 0~255
    uint8_t valid_flags; // Bit0=Điện áp, Bit1=Nhiệt độ, Bit2=SOC, Bit7=Test inject
    uint8_t soc_percent; // % SOC số nguyên tại thời điểm sườn lỗi phát sinh; khi không hợp lệ là 0xFF
} FaultEvent_t; 
 
/* 
 * Thông tin chẩn đoán hàng đợi dùng quan sát trên Keil Watch. 
 * Mã nghiệp vụ không được sửa đổi hàng đợi qua biến này, chỉ được gọi public API bên dưới. 
 */ 
typedef struct 
{ 
    FaultEvent_t last_pushed; // Snapshot sự kiện ghi vào hàng đợi thành công gần nhất
    FaultEvent_t last_processed; // Snapshot sự kiện lấy ra khỏi hàng đợi thành công gần nhất
    uint32_t pushed_count;   // Tổng số sự kiện ghi vào hàng đợi thành công kể từ khi bật nguồn
    uint32_t processed_count; // Tổng số sự kiện lấy ra khỏi hàng đợi thành công kể từ khi bật nguồn
    uint32_t dropped_count;   // Tổng số sự kiện bị loại bỏ do hàng đợi đầy kể từ khi bật nguồn
    uint8_t depth;            // Độ sâu hàng đợi hiện tại
    uint8_t last_pushed_valid; // Sự kiện ghi gần nhất có hợp lệ không
    uint8_t last_processed_valid; // Sự kiện lấy ra gần nhất có hợp lệ không
} FaultEventDebug_t; 
 
extern volatile FaultEventDebug_t g_fault_event_debug; 
 
// Tạo hoặc reset queue tĩnh FreeRTOS, và xóa toàn bộ thông tin chẩn đoán Watch 
void FaultEvent_Init(void); 
 
/* 
 * Ghi không chặn (non-blocking) một sự kiện lỗi. 
 * Thành công trả về 1; tham số bất hợp pháp hoặc hàng đợi đầy trả về 0. Hàng đợi đầy không ảnh hưởng hành động bảo vệ. 
 */ 
uint8_t FaultEvent_Push(FaultEventType_t event_type, 
                        uint16_t changed_mask, 
                        uint16_t active_mask, 
                        const SensorSample_t *sample, 
                        uint8_t bms_state, 
                        uint32_t timestamp_ms); 
 
/*
 * Lấy ra sự kiện lỗi sớm nhất từ hàng đợi.
 *
 * wait_ticks = 0：
 * Hàng đợi rỗng thì trả về ngay lập tức.
 *
 * wait_ticks = portMAX_DELAY：
 * Khi hàng đợi rỗng thì chặn task hiện tại, cho đến khi có sự kiện mới vào hàng đợi.
 */
uint8_t FaultEvent_Pop(
    FaultEvent_t *event,
    TickType_t wait_ticks);
 
/*
 * Chờ và xử lý một sự kiện lỗi.
 *
 * Thời gian chờ do caller quyết định:
 * Bộ lập lịch cũ truyền 0, không chặn;
 * FaultLogTask về sau truyền portMAX_DELAY, chặn chờ sự kiện.
 */
uint8_t FaultEvent_ProcessOne(
    TickType_t wait_ticks);
 
#endif 
