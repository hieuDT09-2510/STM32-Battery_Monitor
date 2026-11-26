/*
 * Tên file: app_soc_storage.h
 * Tên mô-đun: Lưu trữ bền vững SOC khi mất nguồn
 *
 * Trách nhiệm mô-đun:
 * 1. Mã hóa SocPersistentState_t thành bản ghi cố định 32 byte độc lập với layout bộ nhớ trình biên dịch;
 * 2. Bật nguồn quét 2 sector 4KB, tìm ra bản ghi SOC có số thứ tự mới nhất và checksum hợp lệ;
 * 3. Áp dụng ghi nối tiếp (append), sau khi sector hiện tại đầy thì xóa sector kia rồi chuyển đổi;
 * 4. Mỗi lần ghi xong đều đọc lại, so sánh từng byte và giải mã xác thực lại;
 * 5. Cung cấp bộ đếm chẩn đoán kết quả quét, xóa và ghi.
 *
 * Luồng dữ liệu:
 * app_soc tạo yêu cầu lưu -> mô-đun này lấy Flash Mutex -> mã hóa và ghi
 * -> đọc lại xác thực -> thông báo app_soc cập nhật mốc "đã lưu gần nhất".
 *
 * Ranh giới mô-đun:
 * Mô-đun này không tính SOC, không lấy mẫu dòng điện, không quyết định lỗi, cũng không trực tiếp in text CLI.
 */
#ifndef APP_SOC_STORAGE_H
#define APP_SOC_STORAGE_H

#include <stdint.h>

#include "app_soc.h"

/* Mỗi bản ghi cố định 32 byte, thuận tiện tính địa chỉ và kiểm tra CRC độc lập. */
#define SOC_STORAGE_RECORD_SIZE        32U

/* Flash ngoài mỗi sector 4KB, do đó 4096 / 32 = 128 bản ghi. */
#define SOC_STORAGE_RECORDS_PER_SECTOR 128U

/* Trạng thái vận hành lưu trữ SOC cho CLI và debugger theo dõi. */
typedef struct
{
    uint32_t scan_valid_count; /* Số bản ghi vượt qua kiểm tra định dạng, CRC và phạm vi khi quét lúc bật nguồn. */
    uint32_t crc_error_count;  /* Số bản ghi quét thấy không rỗng nhưng không giải mã được. */
    uint32_t write_ok_count;   /* Số lần tích lũy ghi, đọc lại và xác thực thành công. */
    uint32_t write_fail_count; /* Số lần tích lũy lấy khóa, ghi hoặc xác thực sau ghi thất bại. */
    uint32_t erase_count;      /* Số lần tích lũy thực hiện xóa do chuyển đổi sector. */
    uint32_t erase_fail_count; /* Số lần kiểm tra sau khi xóa vẫn còn dữ liệu khác 0xFF. */
    uint32_t last_sequence;    /* Số thứ tự tăng đơn điệu của bản ghi hợp lệ mới nhất hiện tại. */
    uint16_t next_index;       /* Chỉ số slot bản ghi tiếp theo của sector hoạt động (0~128). */
    uint8_t active_sector;     /* Sector ghi nối tiếp hiện tại: 0=A, 1=B. */
    uint8_t restored_valid;    /* 1=Quét lúc bật nguồn tìm thấy bản ghi hợp lệ có thể phục hồi. */
} SocStorageDiagnostics_t;

/* Mã hóa trạng thái theo big-endian cố định thành 32 byte, ghi CRC8 vào byte cuối cùng. */
void SocStorage_EncodeRecord(uint8_t record[SOC_STORAGE_RECORD_SIZE],
                             uint32_t sequence,
                             const SocPersistentState_t *state);

/* Kiểm tra và giải mã bản ghi; trả về 0 nếu định dạng, CRC hoặc phạm vi dữ liệu bị lỗi. */
uint8_t SocStorage_DecodeRecord(const uint8_t record[SOC_STORAGE_RECORD_SIZE],
                                uint32_t *sequence,
                                SocPersistentState_t *state);

/* Kiểm tra 32 byte có toàn 0xFF không; toàn 0xFF nghĩa là slot Flash này chưa từng được ghi. */
uint8_t SocStorage_RecordIsErased(
    const uint8_t record[SOC_STORAGE_RECORD_SIZE]);

/* Dùng so sánh an toàn quay vòng 32-bit để phán đoán candidate có mới hơn reference không. */
uint8_t SocStorage_IsSequenceNewer(uint32_t candidate,
                                   uint32_t reference);

/* Bật nguồn quét 2 sector, thiết lập trạng thái phục hồi, sector hoạt động và vị trí ghi tiếp theo. */
void SocStorage_Init(void);

/* Sao chép trạng thái mới nhất nhận được từ quét lúc bật nguồn; trả về 0 khi không có bản ghi hợp lệ. */
uint8_t SocStorage_Load(SocPersistentState_t *state);

/* Lưu đồng bộ một bản trạng thái và hoàn thành đọc lại xác thực; trả về 1 khi thành công. */
uint8_t SocStorage_Save(const SocPersistentState_t *state);

/* Nếu app_soc có yêu cầu lưu thì thực hiện lưu; trả về 1 sau khi ghi và xác nhận thành công. */
uint8_t SocStorage_ProcessPending(uint32_t now_ms);

/* Sao chép thông tin chẩn đoán lưu trữ hiện tại. */
void SocStorage_GetDiagnostics(SocStorageDiagnostics_t *diagnostics);

#endif
