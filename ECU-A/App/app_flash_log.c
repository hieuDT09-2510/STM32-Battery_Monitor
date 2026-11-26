/*
 * Tên file: app_flash_log.c
 *
 * Tên mô-đun: Nhật ký hộp đen lỗi Flash V2
 *
 * Trách nhiệm mô-đun:
 * 1. Sử dụng 0x001000～0x001FFF của SPI Flash ngoài để lưu bản ghi hệ thống V2;
 * 2. Mỗi bản ghi cố định 32 byte, một sector 4KB lưu 128 bản ghi;
 * 3. Quét bản ghi khi bật nguồn, tìm vị trí ghi an toàn tiếp theo;
 * 4. Sử dụng cờ đánh dấu bản ghi, số phiên bản và CRC8 để nhận diện bản ghi không hợp lệ hoặc bị hỏng;
 * 5. Lưu mỗi sự kiện khởi động hệ thống và nguyên nhân reset của nó.
 *
 * Nguyên tắc thiết kế:
 * Flash sau khi xóa là 0xFF, ghi chỉ có thể đổi 1 thành 0;
 * Vị trí đã ghi hoặc bị ghi hỏng một phần không thể ghi đè trực tiếp;
 * Chỉ sau khi log đầy mới xóa toàn bộ sector V2 để bắt đầu lại.
 *
 * Chức năng hiện tại:
 * Đã triển khai quét khởi động, ghi sự kiện khởi động/lỗi, đọc lại xác thực sau ghi,
 * kiểm tra tính toàn vẹn CRC, in log lịch sử và xóa sector log.
 * 
 * Quy tắc đồng thời (concurrency):
 * 1. Trước khi scheduler khởi động không có đồng thời giữa các task, quét khởi động và ghi log khởi động thực thi trực tiếp;
 * 2. Sau khi scheduler vận hành, các thao tác ghi, in và làm rỗng dùng chung FlashAccess Mutex;
 * 3. Mutex chỉ bảo vệ tài nguyên Flash, không tham gia phán đoán lỗi và hành động bảo vệ;
 * 4. Mọi caller bên ngoài chỉ được sử dụng các public API do file này cung cấp.
 */

#include "app_flash_log.h"

#include <stdio.h>

#include "app_flash_access.h"
#include "app_soc.h"
#include "bsp_spi_flash.h"
#include "can_protocol.h"
#include "app_fault_manager.h"


/* V2 sử dụng sector 4KB thứ hai của Flash ngoài, giữ lại log cũ V1 trong sector 0. */
#define FLASH_LOG_V2_SECTOR_ADDR     ((uint32_t)0x001000U)

/* Phần mở đầu cố định của mỗi bản ghi V2 hợp lệ. */
#define FLASH_LOG_V2_MAGIC           ((uint8_t)0xA5U)
#define FLASH_LOG_V2_VERSION         ((uint8_t)0x02U)

/* CRC được lưu tại byte cuối cùng của bản ghi 32 byte. */
#define FLASH_LOG_V2_CRC_OFFSET      31U

/*
 * Vị trí cố định của các trường trong bản ghi V2 32 byte.
 * Sử dụng offset rõ ràng, tránh ghi trực tiếp struct C vào Flash gây sai lệch căn chỉnh.
 */
#define FLASH_LOG_OFFSET_MAGIC          0U
#define FLASH_LOG_OFFSET_VERSION        1U
#define FLASH_LOG_OFFSET_EVENT_TYPE     2U
#define FLASH_LOG_OFFSET_VALID_FLAGS    3U
#define FLASH_LOG_OFFSET_SEQUENCE       4U
#define FLASH_LOG_OFFSET_TIMESTAMP      6U
#define FLASH_LOG_OFFSET_CHANGED_MASK  10U
#define FLASH_LOG_OFFSET_ACTIVE_MASK   12U
#define FLASH_LOG_OFFSET_VOLTAGE_MV    14U
#define FLASH_LOG_OFFSET_TEMPERATURE   16U
#define FLASH_LOG_OFFSET_SOC           18U
#define FLASH_LOG_OFFSET_BMS_STATE     19U
#define FLASH_LOG_OFFSET_CAN_STATE     20U
#define FLASH_LOG_OFFSET_RESET_REASON  21U

/*
 * Dữ liệu chẩn đoán log Flash.
 * Không dùng static để Keil Watch có thể trực tiếp quan sát.
 */
volatile uint16_t g_flash_log_next_index = 0U;
volatile uint32_t g_flash_log_write_ok_count = 0U;
volatile uint32_t g_flash_log_write_fail_count = 0U;
volatile uint32_t g_flash_log_crc_error_count = 0U;
volatile uint32_t g_flash_log_erase_fail_count = 0U;
volatile FlashLogResetReason_t g_flash_log_reset_reason =
    FLASH_LOG_RESET_UNKNOWN;

/* Mutex do app_flash_access quản lý tập trung, SOC NVM và nhật ký lỗi dùng chung một khóa. */
/*
 * Đánh giá xem toàn bộ một vị trí 32 byte có còn ở trạng thái đã xóa không.
 *
 * Không thể chỉ kiểm tra Byte0:
 * Nếu lần ghi Flash trước đột ngột mất nguồn, Byte0 có thể vẫn là 0xFF,
 * nhưng một số byte phía sau có thể đã bị ghi thành 0.
 * Vị trí như vậy không thể ghi đè lần nữa.
 */
static uint8_t FlashLog_RecordIsErased(const uint8_t *record)
{
    uint8_t i;

    if (record == 0)
    {
        return 0U;
    }

    for (i = 0U; i < FLASH_LOG_RECORD_SIZE; i++)
    {
        if (record[i] != 0xFFU)
        {
            return 0U;
        }
    }

    return 1U;
}


/*
 * Đọc lại toàn bộ sector V2, xác nhận mỗi byte đều đã phục hồi về 0xFF.
 * Mỗi lần chỉ sử dụng mảng cục bộ 32 byte, tránh chiếm dụng không gian stack 4KB.
 */
static uint8_t FlashLog_SectorIsErased(void)
{
    uint8_t record[FLASH_LOG_RECORD_SIZE];
    uint16_t index;
    uint32_t address;

    for (index = 0U; index < FLASH_LOG_MAX_RECORDS; index++)
    {
        address =
            FLASH_LOG_V2_SECTOR_ADDR +
            ((uint32_t)index * FLASH_LOG_RECORD_SIZE);

        XM25QH32_ReadData(address,
                         record,
                         FLASH_LOG_RECORD_SIZE);

        if (FlashLog_RecordIsErased(record) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}


/*
 * Kiểm tra xem một bản ghi đã bị chiếm dụng có hợp lệ hay không.
 *
 * Thứ tự kiểm tra:
 * 1. Cờ đánh dấu cố định có phải 0xA5 không;
 * 2. Phiên bản có phải V2 không;
 * 3. CRC tính từ Byte0～Byte30 có bằng Byte31 không.
 */
static uint8_t FlashLog_RecordIsValid(const uint8_t *record)
{
    uint8_t calculated_crc;

    if (record == 0)
    {
        return 0U;
    }

    if (record[0] != FLASH_LOG_V2_MAGIC)
    {
        return 0U;
    }

    if (record[1] != FLASH_LOG_V2_VERSION)
    {
        return 0U;
    }

    calculated_crc =
        BMS_CalculateCRC8(record, FLASH_LOG_V2_CRC_OFFSET);

    if (calculated_crc != record[FLASH_LOG_V2_CRC_OFFSET])
    {
        return 0U;
    }

    return 1U;
}


/*
 * Đọc cờ reset RCC, phán đoán tại sao chương trình khởi động lần này.
 *
 * Nhiều cờ có thể đồng thời bằng 1, do đó xác định độ ưu tiên theo mức độ nghiêm trọng:
 * Watchdog > Reset phần mềm > Reset tiết kiệm năng lượng > Reset bật nguồn > Chân Reset.
 */
static FlashLogResetReason_t FlashLog_DetectResetReason(void)
{
    uint32_t csr;
    FlashLogResetReason_t reason;

    csr = RCC->CSR;
    reason = FLASH_LOG_RESET_UNKNOWN;

    if ((csr & RCC_CSR_IWDGRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_IWDG;
    }
    else if ((csr & RCC_CSR_WWDGRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_WWDG;
    }
    else if ((csr & RCC_CSR_SFTRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_SOFTWARE;
    }
    else if ((csr & RCC_CSR_LPWRRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_LOW_POWER;
    }
    else if ((csr & RCC_CSR_PORRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_POWER_ON;
    }
    else if ((csr & RCC_CSR_PINRSTF) != 0U)
    {
        reason = FLASH_LOG_RESET_PIN;
    }

    /*
     * Sau khi lưu nguyên nhân thì xóa các cờ reset phần cứng.
     * Nếu không thì lần reset phần mềm tiếp theo có thể nhìn thấy cờ cũ của lần trước.
     */
    RCC->CSR |= RCC_CSR_RMVF;

    return reason;
}


/*
 * Ghi một số 16-bit theo thứ tự byte cao trước.
 *
 * Ví dụ 0x0201:
 * record[offset]     = 0x02
 * record[offset + 1] = 0x01
 */
static void FlashLog_WriteU16(uint8_t *record,
                              uint8_t offset,
                              uint16_t value)
{
    record[offset] = (uint8_t)(value >> 8);
    record[offset + 1U] = (uint8_t)value;
}


/*
 * Ghi một số 32-bit theo thứ tự byte cao trước.
 *
 * Ví dụ timestamp 0x0001E816:
 * Lần lượt ghi 00, 01, E8, 16.
 */
static void FlashLog_WriteU32(uint8_t *record,
                              uint8_t offset,
                              uint32_t value)
{
    record[offset] = (uint8_t)(value >> 24);
    record[offset + 1U] = (uint8_t)(value >> 16);
    record[offset + 2U] = (uint8_t)(value >> 8);
    record[offset + 3U] = (uint8_t)value;
}


/*
 * Đọc một số 16-bit byte cao trước từ bản ghi.
 *
 * Ví dụ 2 byte là 0x12, 0x34:
 * Kết quả 16-bit trả về chính là 0x1234.
 */
static uint16_t FlashLog_ReadU16(const uint8_t *record,
                                 uint8_t offset)
{
    uint16_t value;

    value = (uint16_t)record[offset] << 8;
    value |= (uint16_t)record[offset + 1U];

    return value;
}


/*
 * Đọc một số 32-bit byte cao trước từ bản ghi.
 *
 * Ví dụ 4 byte là 00, 00, 4A, 94:
 * Kết quả trả về là 0x00004A94, tức hệ thập phân 19092.
 */
static uint32_t FlashLog_ReadU32(const uint8_t *record,
                                 uint8_t offset)
{
    uint32_t value;

    value = (uint32_t)record[offset] << 24;
    value |= (uint32_t)record[offset + 1U] << 16;
    value |= (uint32_t)record[offset + 2U] << 8;
    value |= (uint32_t)record[offset + 3U];

    return value;
}


/*
 * Ghi lại trạng thái CAN dựa trên các lỗi hoạt động vẫn còn tồn tại sau khi sự kiện xảy ra.
 *
 * Bus-Off có độ ưu tiên cao nhất;
 * Không có Bus-Off nhưng tồn tại suy giảm truyền thông thì ghi DEGRADED;
 * Khi cả hai đều không tồn tại thì ghi OK.
 */
static FlashLogCanState_t FlashLog_GetCanState(
    const FaultEvent_t *event)
{
    if ((event->active_mask & FAULT_MASK_CAN_BUS_OFF) != 0U)
    {
        return FLASH_LOG_CAN_BUS_OFF;
    }

    if ((event->active_mask & FAULT_MASK_CAN_DEGRADED) != 0U)
    {
        return FLASH_LOG_CAN_DEGRADED;
    }

    return FLASH_LOG_CAN_OK;
}


/* So sánh dữ liệu ghi và dữ liệu đọc lại từ Flash có hoàn toàn giống nhau không. */
static uint8_t FlashLog_RecordEquals(const uint8_t *expected,
                                     const uint8_t *actual)
{
    uint8_t i;

    for (i = 0U; i < FLASH_LOG_RECORD_SIZE; i++)
    {
        if (expected[i] != actual[i])
        {
            return 0U;
        }
    }

    return 1U;
}


/* Bộ đếm chẩn đoán giữ nguyên sau khi đạt giá trị lớn nhất, tránh quay vòng về 0 sau thời gian dài vận hành. */
static void FlashLog_IncrementSaturated(
    volatile uint32_t *value)
{
    if (*value < 0xFFFFFFFFU)
    {
        (*value)++;
    }
}


/* Chuyển đổi mã sự kiện trong bản ghi thành tên dễ hiểu trên UART. */
static const char *FlashLog_GetEventName(uint8_t event_type)
{
    if (event_type == FAULT_EVENT_CONFIRMED)
    {
        return "CONFIRMED";
    }

    if (event_type == FAULT_EVENT_CLEARED)
    {
        return "CLEARED";
    }

    if (event_type == FAULT_EVENT_SYSTEM_BOOT)
    {
        return "BOOT";
    }

    return "UNKNOWN";
}


/*
 * Giải thích nguồn gốc bản ghi dựa trên loại sự kiện và cờ inject:
 * Khởi động đến từ hệ thống, xóa lỗi đến từ quy trình phục hồi, sự kiện xác nhận đến từ phát hiện thực tế hoặc inject CLI.
 */
static const char *FlashLog_GetSourceName(uint8_t event_type,
                                          uint8_t valid_flags)
{
    if (event_type == FAULT_EVENT_SYSTEM_BOOT)
    {
        return "SYSTEM";
    }

    if (event_type == FAULT_EVENT_CLEARED)
    {
        return "RECOVERY";
    }

    if ((valid_flags & FAULT_EVENT_FLAG_INJECTED) != 0U)
    {
        return "INJECTED";
    }

    return "REAL";
}

/*
 * Khởi tạo Flash V2.
 *
 * Mỗi vị trí được chia làm 3 trường hợp:
 *
 * Toàn bộ là 0xFF:
 *     Đây là vị trí thực sự trống đầu tiên, có thể dùng làm vị trí ghi tiếp theo.
 *
 * Bản ghi hợp lệ:
 *     Chứng tỏ ở đây đã có log lịch sử, tiếp tục quét bản ghi tiếp theo.
 *
 * Đã bị chiếm dụng nhưng không hợp lệ:
 *     Có thể do hỏng CRC hoặc mất nguồn tạo thành bản ghi dở dang.
 *     Thống kê lỗi và bỏ qua, không thể ghi đè lên nó.
 */
void FlashLog_Init(void)
{
    uint8_t record[FLASH_LOG_RECORD_SIZE];
    uint16_t index;
    uint32_t address;

    /*
    * Tạo Mutex tĩnh trong lần khởi tạo đầu tiên.
    * Không tạo lại khi gọi hàm khởi tạo các lần tiếp theo.
    */
    g_flash_log_next_index = FLASH_LOG_MAX_RECORDS;
    g_flash_log_write_ok_count = 0U;
    g_flash_log_write_fail_count = 0U;
    g_flash_log_crc_error_count = 0U;
    g_flash_log_erase_fail_count = 0U;

    g_flash_log_reset_reason = FlashLog_DetectResetReason();

    for (index = 0U; index < FLASH_LOG_MAX_RECORDS; index++)
    {
        address =
            FLASH_LOG_V2_SECTOR_ADDR +
            ((uint32_t)index * FLASH_LOG_RECORD_SIZE);

        XM25QH32_ReadData(
            address,
            record,
            FLASH_LOG_RECORD_SIZE);

        if (FlashLog_RecordIsErased(record) != 0U)
        {
            g_flash_log_next_index = index;
            break;
        }

        if (FlashLog_RecordIsValid(record) == 0U)
        {
            g_flash_log_crc_error_count++;
        }
    }

    printf("[LOG V2] Ready Index=%u/%u Reset=%u Invalid=%lu\r\n",
           (unsigned int)g_flash_log_next_index,
           (unsigned int)FLASH_LOG_MAX_RECORDS,
           (unsigned int)g_flash_log_reset_reason,
           (unsigned long)g_flash_log_crc_error_count);
}

/*
 * Mã hóa một FaultEvent thành 32 byte cố định và ghi vào Flash ngoài.
 *
 * Quy trình thực thi:
 * Kiểm tra sự kiện
 *   ↓
 * Xóa sector đã đầy khi cần thiết
 *   ↓
 * Lắp ráp 32 byte theo từng trường
 *   ↓
 * Tính CRC8 của Byte0～Byte30
 *   ↓
 * Ghi vào Flash bằng Page Program
 *   ↓
 * Đọc lại ngay 32 byte
 *   ↓
 * So sánh từng byte và cập nhật bộ đếm chẩn đoán
 * 
 * Logic ghi thực tế được thực hiện khi đã nắm giữ Flash Mutex.
 * Caller không được gọi trực tiếp hàm nội bộ này.
 */
static uint8_t FlashLog_WriteEventUnlocked(
    const FaultEvent_t *event)
{
    uint8_t record[FLASH_LOG_RECORD_SIZE];
    uint8_t verify_record[FLASH_LOG_RECORD_SIZE];
    uint8_t i;
    uint16_t record_index;
    uint32_t address;

    /*
     * Con trỏ NULL hoặc loại sự kiện không hợp lệ thì không ghi.
     * Xác nhận/xóa lỗi bắt buộc phải chứa bit biến đổi; khởi động hệ thống không có biến đổi lỗi, cho phép bằng 0.
     */
    if ((event == 0) ||
        ((event->event_type != FAULT_EVENT_CONFIRMED) &&
         (event->event_type != FAULT_EVENT_CLEARED) &&
         (event->event_type != FAULT_EVENT_SYSTEM_BOOT)) ||
        (((event->event_type == FAULT_EVENT_CONFIRMED) ||
          (event->event_type == FAULT_EVENT_CLEARED)) &&
         (event->changed_mask == 0U)))
    {
        FlashLog_IncrementSaturated(
            &g_flash_log_write_fail_count);

        return 0U;
    }

    /*
     * Sau khi toàn bộ 128 bản ghi bị chiếm dụng, xóa cả sector V2.
     * Sector 0 chứa log cũ V1 sẽ không bị ảnh hưởng.
     */
    if (g_flash_log_next_index >= FLASH_LOG_MAX_RECORDS)
    {
        XM25QH32_SectorErase(FLASH_LOG_V2_SECTOR_ADDR);

        /*
         * Khi xóa chưa vượt qua đọc lại xác thực thì không được reset con trỏ ghi về 0,
         * nếu không sẽ tiếp tục lập trình trên vị trí vẫn còn dữ liệu cũ và phá hủy bản ghi.
         */
        if (FlashLog_SectorIsErased() == 0U)
        {
            FlashLog_IncrementSaturated(
                &g_flash_log_erase_fail_count);
            FlashLog_IncrementSaturated(
                &g_flash_log_write_fail_count);

            printf("[LOG V2] Full-sector erase verify failed\r\n");
            return 0U;
        }

        g_flash_log_next_index = 0U;
        g_flash_log_crc_error_count = 0U;
    }

    record_index = g_flash_log_next_index;

    /*
     * Trước tiên xóa toàn bộ bản ghi về 0.
     * Byte22～Byte30 không gán giá trị riêng, do đó tự nhiên đóng vai trò trường dành riêng lưu 0.
     */
    for (i = 0U; i < FLASH_LOG_RECORD_SIZE; i++)
    {
        record[i] = 0U;
    }

    record[FLASH_LOG_OFFSET_MAGIC] =
        FLASH_LOG_V2_MAGIC;

    record[FLASH_LOG_OFFSET_VERSION] =
        FLASH_LOG_V2_VERSION;

    record[FLASH_LOG_OFFSET_EVENT_TYPE] =
        event->event_type;

    record[FLASH_LOG_OFFSET_VALID_FLAGS] =
        event->valid_flags;

    //Ghi một số 16-bit theo thứ tự byte cao trước.
    FlashLog_WriteU16(
        record,
        FLASH_LOG_OFFSET_SEQUENCE,
        record_index);
    //Ghi một số 32-bit theo thứ tự byte cao trước.
    FlashLog_WriteU32(
        record,
        FLASH_LOG_OFFSET_TIMESTAMP,
        event->timestamp_ms);

    FlashLog_WriteU16(
        record,
        FLASH_LOG_OFFSET_CHANGED_MASK,
        event->changed_mask);

    FlashLog_WriteU16(
        record,
        FLASH_LOG_OFFSET_ACTIVE_MASK,
        event->active_mask);

    FlashLog_WriteU16(
        record,
        FLASH_LOG_OFFSET_VOLTAGE_MV,
        event->cell_voltage_mv);

    /*
     * Nhiệt độ int16_t sau khi chuyển thành uint16_t thì các bit nhị phân bên dưới giữ nguyên.
     * Ví dụ -5.0°C lưu thành -50, tương ứng 0xFFCE.
     */
    FlashLog_WriteU16(
        record,
        FLASH_LOG_OFFSET_TEMPERATURE,
        (uint16_t)event->temperature_dC);

    /* SOC do FaultEvent chụp lại khi sườn sự kiện lỗi xảy ra, không được tính lại khi ghi Flash tốc độ chậm. */
    record[FLASH_LOG_OFFSET_SOC] = event->soc_percent;

    record[FLASH_LOG_OFFSET_BMS_STATE] =
        event->bms_state;

    record[FLASH_LOG_OFFSET_CAN_STATE] =
        (uint8_t)FlashLog_GetCanState(event);

    record[FLASH_LOG_OFFSET_RESET_REASON] =
        (uint8_t)g_flash_log_reset_reason;

    /*
     * Cuối cùng tính CRC.
     * CRC bao phủ Byte0～Byte30, bản thân CRC lưu tại Byte31.
     */
    record[FLASH_LOG_V2_CRC_OFFSET] =
        BMS_CalculateCRC8(
            record,
            FLASH_LOG_V2_CRC_OFFSET);

    address =
        FLASH_LOG_V2_SECTOR_ADDR +
        ((uint32_t)record_index *
         FLASH_LOG_RECORD_SIZE);

    XM25QH32_PageProgram(
        address,
        record,
        FLASH_LOG_RECORD_SIZE);

    /*
     * Ghi xong đọc lại từ Flash ngay lập tức.
     * API không cung cấp giá trị trả về lỗi phần cứng, nên đọc lại so sánh là biện pháp xác nhận thực tế.
     */
    XM25QH32_ReadData(
        address,
        verify_record,
        FLASH_LOG_RECORD_SIZE);

    if ((FlashLog_RecordEquals(record, verify_record) == 0U) ||
        (FlashLog_RecordIsValid(verify_record) == 0U))
    {
        FlashLog_IncrementSaturated(
            &g_flash_log_write_fail_count);

        /*
         * Vị trí thất bại có thể đã bị ghi một số bit thành 0, không thể ghi đè lại.
         * Lần tiếp theo chuyển sang dùng vị trí 32 byte phía sau.
         */
        g_flash_log_next_index++;

        printf("[LOG V2] Write verify failed at #%u\r\n",
               (unsigned int)record_index);

        return 0U;
    }

    g_flash_log_next_index++;

    FlashLog_IncrementSaturated(
        &g_flash_log_write_ok_count);

    printf("[LOG V2] Write #%u Event=%u Changed=0x%04X Active=0x%04X\r\n",
           (unsigned int)record_index,
           (unsigned int)event->event_type,
           (unsigned int)event->changed_mask,
           (unsigned int)event->active_mask);

    return 1U;
}


/*
 * Public API ghi nhật ký lỗi an toàn luồng (thread-safe).
 *
 * Tất cả caller trong giai đoạn vận hành bắt buộc phải đi qua đây,
 * không được gọi trực tiếp hàm nội bộ Unlocked.
 * 
 * Public function lấy Mutex
        ↓
 * Gọi hàm ghi thực tế nội bộ
        ↓
 * Hàm nội bộ trả về cho public function dù thành công hay thất bại
        ↓
 * Public function giải phóng Mutex tập trung
 */
uint8_t FlashLog_WriteEvent(
    const FaultEvent_t *event)
{
    uint8_t result;

    if (FlashAccess_Lock() == 0U)
    {
        FlashLog_IncrementSaturated(
            &g_flash_log_write_fail_count);

        return 0U;
    }

    result =
        FlashLog_WriteEventUnlocked(event);

    FlashAccess_Unlock();

    return result;
}

/*
 * Tạo một bản ghi khởi động trong giai đoạn khởi tạo hệ thống.
 * Lúc này TIM3 chưa chạy, cũng chưa có snapshot cảm biến, do đó thời gian, điện áp và nhiệt độ không hợp lệ;
 * nếu vừa phục hồi thành công từ SOC NVM, có thể đồng thời lưu SOC đã phục hồi.
 * FlashLog_WriteEvent sẽ ghi nguyên nhân reset do FlashLog_Init phát hiện vào bản ghi.
 */
uint8_t FlashLog_WriteBootEvent(void)
{
    FaultEvent_t event;         /* Tái sử dụng định dạng sự kiện thống nhất để cấu tạo bản ghi SYSTEM_BOOT. */
    SocSnapshot_t soc_snapshot; /* Snapshot SOC có thể đã được phục hồi từ NVM. */

    event.timestamp_ms = 0U;
    event.changed_mask = 0U;
    event.active_mask = 0U;
    event.cell_voltage_mv = 0U;
    event.temperature_dC = 0;
    event.event_type = (uint8_t)FAULT_EVENT_SYSTEM_BOOT;
    event.bms_state = 0U;
    event.valid_flags = 0U;
    event.soc_percent = 0xFFU;

    if (Soc_GetSnapshot(&soc_snapshot) != 0U)
    {
        event.soc_percent = (uint8_t)(
            (soc_snapshot.soc_centi_percent + 50U) / 100U);
        event.valid_flags |= FAULT_EVENT_VALID_SOC;
    }

    return FlashLog_WriteEvent(&event);
}


/*
 * Đọc từng bản ghi và in lịch sử hệ thống và lỗi trong Flash V2.
 *
 * Mỗi lần chỉ sử dụng một mảng 32 byte, tránh tạo bộ đệm cả sector 4KB
 * trong vùng stack dung lượng nhỏ của STM32.
 */
static void FlashLog_PrintAllUnlocked(void)
{
    uint8_t record[FLASH_LOG_RECORD_SIZE]; /* 32 byte thô của slot hiện tại. */
    uint8_t valid_flags;       /* Các bit hiệu lực của điện áp, nhiệt độ, SOC và nguồn inject. */
    uint8_t event_type;        /* Xác nhận, xóa lỗi hoặc khởi động. */
    uint8_t soc;               /* Phần trăm SOC số nguyên trong bản ghi. */
    uint8_t soc_valid;         /* Kết luận hiệu lực của SOC sau khi tương thích các bản ghi V2 mới và cũ. */
    uint16_t index;            /* Slot quét hiện tại 0~127. */
    uint16_t sequence;         /* Số thứ tự log lưu trong bản ghi. */
    uint16_t changed_mask;     /* Bit lỗi mới xác nhận hoặc mới xóa của sự kiện này. */
    uint16_t active_mask;      /* Toàn bộ các bit lỗi vẫn đang hoạt động sau khi sự kiện xảy ra. */
    uint16_t voltage_mv;       /* Điện áp pin tại hiện trường sự kiện, đơn vị mV. */
    int16_t temperature_dC;    /* Nhiệt độ tại hiện trường sự kiện, đơn vị 0.1°C. */
    int32_t temperature_value; /* Dùng 32-bit xử lý giá trị tuyệt đối nhiệt độ âm, tránh tràn số. */
    uint32_t timestamp_ms;     /* Thời gian xảy ra sự kiện, đơn vị ms. */
    uint32_t address;          /* Địa chỉ Flash ngoài của slot hiện tại. */

    printf("\r\n[LOG V2] Dump begin\r\n");

    for (index = 0U; index < FLASH_LOG_MAX_RECORDS; index++)
    {
        address =
            FLASH_LOG_V2_SECTOR_ADDR +
            ((uint32_t)index * FLASH_LOG_RECORD_SIZE);

        XM25QH32_ReadData(address,
                         record,
                         FLASH_LOG_RECORD_SIZE);

        /*
         * Gặp bản ghi đầu tiên hoàn toàn để trống, chứng tỏ phía sau chưa ghi,
         * do đó kết thúc lần đọc này.
         */
        if (FlashLog_RecordIsErased(record) != 0U)
        {
            break;
        }

        /*
         * Bản ghi đã bị chiếm dụng, nhưng cờ đánh dấu, phiên bản hoặc CRC bị lỗi.
         * In lỗi và bỏ qua, tiếp tục kiểm tra các bản ghi phía sau.
         */
        if (FlashLog_RecordIsValid(record) == 0U)
        {
            printf("[LOG V2] Slot=%u INVALID\r\n",
                   (unsigned int)index);
            continue;
        }

        event_type =
            record[FLASH_LOG_OFFSET_EVENT_TYPE];

        valid_flags =
            record[FLASH_LOG_OFFSET_VALID_FLAGS];

        sequence =
            FlashLog_ReadU16(record,
                             FLASH_LOG_OFFSET_SEQUENCE);

        timestamp_ms =
            FlashLog_ReadU32(record,
                             FLASH_LOG_OFFSET_TIMESTAMP);

        changed_mask =
            FlashLog_ReadU16(record,
                             FLASH_LOG_OFFSET_CHANGED_MASK);

        active_mask =
            FlashLog_ReadU16(record,
                             FLASH_LOG_OFFSET_ACTIVE_MASK);

        voltage_mv =
            FlashLog_ReadU16(record,
                             FLASH_LOG_OFFSET_VOLTAGE_MV);

        temperature_dC =
            (int16_t)FlashLog_ReadU16(
                record,
                FLASH_LOG_OFFSET_TEMPERATURE);

        soc = record[FLASH_LOG_OFFSET_SOC];

        /*
         * Bản ghi mới dùng Bit2 đánh dấu rõ ràng SOC; bản ghi V2 cũ không có bit này, nhưng miễn điện áp hợp lệ và
         * SOC đã lưu nằm trong khoảng 0~100, vẫn hiển thị tương thích theo định dạng cũ.
         */
        soc_valid =
            (((valid_flags & FAULT_EVENT_VALID_SOC) != 0U) ||
             (((valid_flags & FAULT_EVENT_VALID_VOLTAGE) != 0U) &&
              (soc <= 100U))) ? 1U : 0U;

        printf("[LOG V2] #%u %s Source=%s Time=%lums Changed=0x%04X Active=0x%04X\r\n",
               (unsigned int)sequence,
               FlashLog_GetEventName(event_type),
               FlashLog_GetSourceName(event_type, valid_flags),
               (unsigned long)timestamp_ms,
               (unsigned int)changed_mask,
               (unsigned int)active_mask);

        if (((valid_flags & FAULT_EVENT_VALID_VOLTAGE) != 0U) &&
            (soc_valid != 0U))
        {
            printf("         Voltage=%umV SOC=%u%%\r\n",
                   (unsigned int)voltage_mv,
                   (unsigned int)soc);
        }
        else if ((valid_flags & FAULT_EVENT_VALID_VOLTAGE) != 0U)
        {
            printf("         Voltage=%umV SOC=INVALID\r\n",
                   (unsigned int)voltage_mv);
        }
        else if (soc_valid != 0U)
        {
            printf("         Voltage=INVALID SOC=%u%%\r\n",
                   (unsigned int)soc);
        }
        else
        {
            printf("         Voltage=INVALID SOC=INVALID\r\n");
        }

        if ((valid_flags & FAULT_EVENT_VALID_TEMPERATURE) != 0U)
        {
            /*
             * Nâng lên 32-bit trước rồi mới xử lý số âm.
             * Như vậy ngay cả khi dữ liệu nhiệt độ tiệm cận cận dưới int16_t cũng không bị tràn số.
             */
            temperature_value = (int32_t)temperature_dC;

            if (temperature_value < 0)
            {
                temperature_value = -temperature_value;

                printf("         Temperature=-%ld.%ldC",
                       (long)(temperature_value / 10),
                       (long)(temperature_value % 10));
            }
            else
            {
                printf("         Temperature=%ld.%ldC",
                       (long)(temperature_value / 10),
                       (long)(temperature_value % 10));
            }
        }
        else
        {
            printf("         Temperature=INVALID");
        }

        printf(" BMS=%u CAN=%u Reset=%u\r\n",
               (unsigned int)record[FLASH_LOG_OFFSET_BMS_STATE],
               (unsigned int)record[FLASH_LOG_OFFSET_CAN_STATE],
               (unsigned int)record[FLASH_LOG_OFFSET_RESET_REASON]);
    }

    printf("[LOG V2] Dump end. Occupied=%u\r\n\r\n",
           (unsigned int)index);
}


/*
 * Public API in nhật ký lịch sử Flash an toàn luồng.
 *
 * Giữ Mutex trong suốt thời gian in để output lần này thấy nội dung log nhất quán,
 * các lượt ghi mới do FaultLogTask sinh ra sẽ tạm thời chờ trong hàng đợi sự kiện RAM.
 */
void FlashLog_PrintAll(void)
{
    if (FlashAccess_Lock() == 0U)
    {
        printf("[LOG V2] Flash mutex unavailable\r\n");
        return;
    }

    FlashLog_PrintAllUnlocked();

    FlashAccess_Unlock();
}


/*
 * Xóa nhật ký lỗi Flash V2.
 *
 * Chỉ xóa sector 4KB thứ hai do V2 sử dụng,
 * không ảnh hưởng đến log cũ V1 được giữ lại trong sector đầu tiên.
 *
 * Hàm này chỉ được gọi bởi thao tác rõ ràng của người dùng,
 * không được đưa vào task chu kỳ hoặc quy trình khởi động hệ thống.
 */
static uint8_t FlashLog_ClearUnlocked(void)
{
    /*
     * Xóa toàn bộ sector 4KB tại vị trí 0x001000.
     * Driver tầng dưới sẽ chờ Flash kết thúc trạng thái bận mới trả về.
     */
    XM25QH32_SectorErase(FLASH_LOG_V2_SECTOR_ADDR);

    /*
     * Bắt buộc phải đọc lại cả sector để xác nhận kết quả xóa.
     * Khi thất bại thì giữ nguyên next_index ban đầu, tránh việc ghi đè dữ liệu cũ từ vị trí số 0.
     */
    if (FlashLog_SectorIsErased() == 0U)
    {
        FlashLog_IncrementSaturated(
            &g_flash_log_erase_fail_count);

        printf("[LOG V2] Clear verify failed. Index remains %u\r\n",
               (unsigned int)g_flash_log_next_index);
        return 0U;
    }

    /*
     * Sau khi xóa sector, vị trí số 0 lại trở thành vị trí ghi tiếp theo.
     */
    g_flash_log_next_index = 0U;

    /*
     * Xóa các bộ đếm chẩn đoán của đợt thao tác log này.
     * Nguyên nhân reset không xóa về 0, vì nguyên nhân khởi động lần này vẫn còn hiệu lực.
     */
    g_flash_log_write_ok_count = 0U;
    g_flash_log_write_fail_count = 0U;
    g_flash_log_crc_error_count = 0U;
    g_flash_log_erase_fail_count = 0U;

    printf("[LOG V2] Cleared. Next Index=0\r\n");
    return 1U;
}

/*
 * Public API làm rỗng nhật ký an toàn luồng.
 *
 * Giữ Mutex trong thời gian xóa và đọc lại xác thực,
 * ngăn FaultLogTask đồng thời ghi bản ghi vào sector đang bị xóa.
 */
uint8_t FlashLog_Clear(void)
{
    uint8_t result;

    if (FlashAccess_Lock() == 0U)
    {
        return 0U;
    }

    result =
        FlashLog_ClearUnlocked();

    FlashAccess_Unlock();

    return result;
}
