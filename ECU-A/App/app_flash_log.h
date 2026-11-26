/*
 * Tên file: app_flash_log.h
 *
 * Tên mô-đun: Nhật ký hộp đen lỗi Flash V2
 *
 * Trách nhiệm mô-đun:
 * 1. Ghi các sự kiện xác nhận lỗi, xóa lỗi và khởi động hệ thống vào SPI Flash ngoài;
 * 2. Lưu thời gian lỗi, mặt nạ lỗi, điện áp, nhiệt độ, SOC, trạng thái BMS,
 *    trạng thái CAN và nguyên nhân reset của lần bật nguồn này;
 * 3. Sử dụng bản ghi cố định 32 byte và CRC8 kiểm tra tính toàn vẹn bản ghi;
 * 4. Cung cấp API khởi tạo, ghi, in qua UART và xóa log.
 *
 * Ranh giới mô-đun:
 * Mô-đun này không phán đoán điều kiện lỗi, không sửa đổi trạng thái BMS, cũng không điều khiển còi báo và CAN.
 * Nó chỉ chịu trách nhiệm lưu trữ bền vững các sự kiện lỗi đã xác nhận và thông tin khởi động hệ thống lần này.
 *
 * Vùng lưu trữ:
 * Log cũ V1 giữ lại tại 0x000000～0x000FFF của Flash ngoài;
 * Log mới V2 sử dụng 0x001000～0x001FFF.
 */
#ifndef APP_FLASH_LOG_H
#define APP_FLASH_LOG_H

#include <stdint.h>

#include "app_fault_event.h"


/* Một bản ghi V2 cố định 32 byte. */
#define FLASH_LOG_RECORD_SIZE       32U

/* Một sector 4KB có thể lưu 128 bản ghi 32 byte. */
#define FLASH_LOG_MAX_RECORDS       128U


/*
 * Trạng thái CAN ghi vào Flash.
 * Ghi lại trạng thái truyền thông được hệ thống xác nhận khi sự kiện lỗi xảy ra.
 */
typedef enum
{
    FLASH_LOG_CAN_OK = 0,
    FLASH_LOG_CAN_DEGRADED = 1,
    FLASH_LOG_CAN_BUS_OFF = 2
} FlashLogCanState_t;


/*
 * Nguyên nhân reset của lần khởi động chương trình này.
 * Tất cả các bản ghi lỗi sinh ra trong một chu kỳ bật nguồn đều dùng chung một nguyên nhân reset.
 */
typedef enum
{
    FLASH_LOG_RESET_UNKNOWN = 0,
    FLASH_LOG_RESET_POWER_ON = 1,
    FLASH_LOG_RESET_PIN = 2,
    FLASH_LOG_RESET_SOFTWARE = 3,
    FLASH_LOG_RESET_IWDG = 4,
    FLASH_LOG_RESET_WWDG = 5,
    FLASH_LOG_RESET_LOW_POWER = 6
} FlashLogResetReason_t;


/*
 * Biến chẩn đoán log dùng để quan sát trên Keil Watch.
 * Mã nghiệp vụ chỉ được đọc, không được sửa đổi trực tiếp.
 */
extern volatile uint16_t g_flash_log_next_index;
extern volatile uint32_t g_flash_log_write_ok_count;
extern volatile uint32_t g_flash_log_write_fail_count;
extern volatile uint32_t g_flash_log_crc_error_count;
extern volatile uint32_t g_flash_log_erase_fail_count;
extern volatile FlashLogResetReason_t g_flash_log_reset_reason;


/*
 * Khởi tạo log Flash V2.
 *
 * Nhiệm vụ chính:
 * 1. Đọc và lưu nguyên nhân reset của lần khởi động này;
 * 2. Quét sector log V2;
 * 3. Tìm vị trí bản ghi có thể ghi tiếp theo;
 * 4. Thống kê lỗi CRC phát hiện trong quá trình quét.
 */
void FlashLog_Init(void);


/*
 * Ghi một bản ghi khởi động hệ thống.
 *
 * Bản ghi này không đi qua hàng đợi sự kiện lỗi, vì khi vừa khởi động hệ thống chưa có sườn sự kiện lỗi;
 * Trường Reset trong bản ghi giải thích lần khởi động này do bật nguồn, nút bấm, phần mềm hay watchdog gây ra.
 * Ghi thành công trả về 1, thất bại trả về 0.
 */
uint8_t FlashLog_WriteBootEvent(void);


/*
 * Ghi một sự kiện lỗi hoàn chỉnh.
 *
 * Ghi thành công và đọc lại xác thực hợp lệ trả về 1;
 * Tham số không hợp lệ hoặc xác thực ghi thất bại trả về 0.
 */
uint8_t FlashLog_WriteEvent(const FaultEvent_t *event);


/*
 * Đọc từng bản ghi từ Flash và in qua USART.
 *
 * Mỗi lần chỉ lưu một bản ghi 32 byte trên stack,
 * không còn cấp phát mảng cục bộ 4096 byte như bản cũ.
 */
void FlashLog_PrintAll(void);


/*
 * Xóa sector log V2 và xóa về 0 thông tin chẩn đoán log.
 *
 * Không xóa log cũ V1 trong sector 0.
 */
/* Xóa và đọc lại xác thực thành công trả về 1; xóa chưa có hiệu lực trả về 0. */
uint8_t FlashLog_Clear(void);


#endif
