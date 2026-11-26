/*
 * Tên file: app_soc_storage.c
 *
 * Layout bản ghi cố định 32 byte:
 *   [0..1]   Magic "SO"
 *   [2]      Phiên bản định dạng
 *   [3]      Cờ hiệu lực
 *   [4..7]   Số thứ tự bản ghi
 *   [8..15]  Điện tích còn lại uA*s
 *   [16..17] SOC, đơn vị 0.01%
 *   [18..21] Dung lượng định mức uAh
 *   [22..25] Timestamp mili-giây khi tạo trạng thái
 *   [26..29] Dung lượng đo thực tế SOH gần nhất uAh
 *   [30]     Cờ hiệu lực SOH
 *   [31]     CRC8 của 31 byte trước
 *
 * Tất cả các trường nhiều byte đều được mã hóa tường minh theo định dạng big-endian, không được ghi trực tiếp struct C vào Flash.
 * Như vậy có thể tránh việc padding struct, căn chỉnh thành viên và khác biệt giữa các trình biên dịch làm thay đổi định dạng dữ liệu lưu trữ.
 */
#include "app_soc_storage.h"

#include <string.h>

/* Magic là ASCII 'S', 'O', dùng để loại trừ nhanh dữ liệu ngẫu nhiên hoặc dữ liệu của mô-đun khác. */
#define SOC_RECORD_MAGIC_0          ((uint8_t)0x53U)
#define SOC_RECORD_MAGIC_1          ((uint8_t)0x4FU)
/* Khi định dạng thay đổi không tương thích cần tăng phiên bản, tránh firmware cũ giải mã nhầm bản ghi mới. */
#define SOC_RECORD_VERSION          ((uint8_t)0x01U)
/* CRC lưu tại byte cuối cùng, phạm vi tính toán từ record[0] đến record[30]. */
#define SOC_RECORD_CRC_OFFSET       31U

/* Các offset dưới đây định nghĩa giao thức lưu trữ bền vững, không được ngầm thay đổi theo thứ tự thành viên struct C. */
#define SOC_OFFSET_SEQUENCE         4U
#define SOC_OFFSET_REMAINING        8U
#define SOC_OFFSET_PERCENT          16U
#define SOC_OFFSET_CAPACITY         18U
#define SOC_OFFSET_TIMESTAMP        22U
#define SOC_OFFSET_SOH_CAPACITY     26U
#define SOC_OFFSET_SOH_VALID        30U

/* CRC-8/SAE-J1850: Giá trị ban đầu 0xFF, đa thức 0x1D, kết quả XOR với 0xFF. */
static uint8_t SocStorage_CalculateCrc(const uint8_t *data,
                                       uint8_t length)
{
    uint8_t crc = 0xFFU;  /* Giá trị khởi tạo theo quy định của thuật toán CRC. */
    uint8_t byte_index;   /* Chỉ số byte dữ liệu đang xử lý. */
    uint8_t bit_index;    /* Chỉ số bit đang xử lý trong byte hiện tại. */

    /* Mỗi byte đầu vào trước hết XOR với CRC, sau đó dịch từng bit 8 lần theo bit cao nhất. */
    for (byte_index = 0U; byte_index < length; byte_index++)
    {
        crc ^= data[byte_index];
        for (bit_index = 0U; bit_index < 8U; bit_index++)
        {
            /* Khi bit cao nhất là 1, sau khi dịch trái thì XOR với đa thức sinh 0x1D. */
            crc = ((crc & 0x80U) != 0U) ?
                (uint8_t)((crc << 1U) ^ 0x1DU) :
                (uint8_t)(crc << 1U);
        }
    }
    /* SAE-J1850 quy định trước khi xuất ra phải XOR tiếp với 0xFF. */
    return (uint8_t)(crc ^ 0xFFU);
}

/* Ghi số nguyên 16-bit vào bản ghi theo thứ tự byte cao trước, tránh phụ thuộc endian của MCU. */
static void SocStorage_WriteU16(uint8_t *data,
                                uint8_t offset,
                                uint16_t value)
{
    data[offset] = (uint8_t)(value >> 8U);
    data[offset + 1U] = (uint8_t)value;
}

/* Ghi số nguyên 32-bit vào bản ghi theo thứ tự byte cao trước. */
static void SocStorage_WriteU32(uint8_t *data,
                                uint8_t offset,
                                uint32_t value)
{
    data[offset] = (uint8_t)(value >> 24U);
    data[offset + 1U] = (uint8_t)(value >> 16U);
    data[offset + 2U] = (uint8_t)(value >> 8U);
    data[offset + 3U] = (uint8_t)value;
}

/* Ghi số nguyên 64-bit từng byte vào bản ghi theo thứ tự byte cao trước. */
static void SocStorage_WriteU64(uint8_t *data,
                                uint8_t offset,
                                uint64_t value)
{
    uint8_t index; /* Lần lượt ghi 8 byte từ bit63..56 đến bit7..0. */
    for (index = 0U; index < 8U; index++)
    {
        data[offset + index] =
            (uint8_t)(value >> (uint8_t)(56U - index * 8U));
    }
}

/* Khôi phục số nguyên 16-bit từ thứ tự byte big-endian của bản ghi. */
static uint16_t SocStorage_ReadU16(const uint8_t *data,
                                   uint8_t offset)
{
    return (uint16_t)(((uint16_t)data[offset] << 8U) |
                      (uint16_t)data[offset + 1U]);
}

/* Khôi phục số nguyên 32-bit từ thứ tự byte big-endian của bản ghi. */
static uint32_t SocStorage_ReadU32(const uint8_t *data,
                                   uint8_t offset)
{
    return ((uint32_t)data[offset] << 24U) |
           ((uint32_t)data[offset + 1U] << 16U) |
           ((uint32_t)data[offset + 2U] << 8U) |
           (uint32_t)data[offset + 3U];
}

/* Khôi phục số nguyên 64-bit từ thứ tự byte big-endian của bản ghi. */
static uint64_t SocStorage_ReadU64(const uint8_t *data,
                                   uint8_t offset)
{
    uint8_t index;        /* Chỉ số thành viên 8 byte đang đọc. */
    uint64_t value = 0ULL; /* Mỗi khi đọc 1 byte, giá trị cũ dịch trái 8 bit rồi ghép byte mới vào. */
    for (index = 0U; index < 8U; index++)
    {
        value = (value << 8U) | (uint64_t)data[offset + index];
    }
    return value;
}

void SocStorage_EncodeRecord(uint8_t record[SOC_STORAGE_RECORD_SIZE],
                             uint32_t sequence,
                             const SocPersistentState_t *state)
{
    /*
     * Trước tiên điền toàn bộ 0, để các byte dành riêng [26..30] chưa dùng hiện tại có giá trị xác định,
     * khi mở rộng định dạng trong tương lai vẫn duy trì tính nhất quán khi tính CRC của bản ghi cũ.
     */
    (void)memset(record, 0, SOC_STORAGE_RECORD_SIZE);
    /* Ghi định danh bản ghi, phiên bản định dạng và cờ hiệu lực. */
    record[0] = SOC_RECORD_MAGIC_0;
    record[1] = SOC_RECORD_MAGIC_1;
    record[2] = SOC_RECORD_VERSION;
    record[3] = 1U;
    /* Số thứ tự dùng để phán đoán bản ghi nào mới nhất giữa 2 sector. */
    SocStorage_WriteU32(record, SOC_OFFSET_SEQUENCE, sequence);
    /* Lưu trạng thái cốt lõi của tích phân Coulomb, sau khi khởi động lại có thể tiếp tục tích lũy từ đây. */
    SocStorage_WriteU64(record,
                        SOC_OFFSET_REMAINING,
                        (uint64_t)state->remaining_uAs);
    SocStorage_WriteU16(record,
                        SOC_OFFSET_PERCENT,
                        state->soc_centi_percent);
    SocStorage_WriteU32(record,
                        SOC_OFFSET_CAPACITY,
                        state->rated_capacity_uAh);
    SocStorage_WriteU32(record,
                        SOC_OFFSET_TIMESTAMP,
                        state->timestamp_ms);
    SocStorage_WriteU32(record,
                        SOC_OFFSET_SOH_CAPACITY,
                        state->learned_capacity_uAh);
    record[SOC_OFFSET_SOH_VALID] =
        (state->soh_valid != 0U) ? 1U : 0U;
    /* CRC bắt buộc phải tính cuối cùng, để nó bao phủ toàn bộ 31 byte đã ghi trước đó. */
    record[SOC_RECORD_CRC_OFFSET] =
        SocStorage_CalculateCrc(record, SOC_RECORD_CRC_OFFSET);
}

uint8_t SocStorage_DecodeRecord(const uint8_t record[SOC_STORAGE_RECORD_SIZE],
                                uint32_t *sequence,
                                SocPersistentState_t *state)
{
    uint64_t remaining; /* Trước tiên đọc theo kiểu không dấu, thuận tiện kiểm tra phạm vi từ 0 đến đầy dung lượng. */

    /* Phòng thủ con trỏ NULL, tránh gọi hàm bất thường phá hủy bộ nhớ. */
    if ((record == 0) || (sequence == 0) || (state == 0))
    {
        return 0U;
    }
    /* Khi định danh, phiên bản hoặc cờ hiệu lực không khớp, slot này không thuộc về định dạng hiện tại. */
    if ((record[0] != SOC_RECORD_MAGIC_0) ||
        (record[1] != SOC_RECORD_MAGIC_1) ||
        (record[2] != SOC_RECORD_VERSION) ||
        (record[3] != 1U))
    {
        return 0U;
    }
    /* CRC không nhất quán chứng tỏ mất nguồn, ngắt quãng khi ghi hoặc hỏng dữ liệu, từ chối sử dụng toàn bộ bản ghi. */
    if (record[SOC_RECORD_CRC_OFFSET] !=
        SocStorage_CalculateCrc(record, SOC_RECORD_CRC_OFFSET))
    {
        return 0U;
    }

    /* Chỉ giải mã các trường nghiệp vụ sau khi CRC vượt qua kiểm tra. */
    remaining = SocStorage_ReadU64(record, SOC_OFFSET_REMAINING);
    state->soc_centi_percent =
        SocStorage_ReadU16(record, SOC_OFFSET_PERCENT);
    state->rated_capacity_uAh =
        SocStorage_ReadU32(record, SOC_OFFSET_CAPACITY);
    state->timestamp_ms =
        SocStorage_ReadU32(record, SOC_OFFSET_TIMESTAMP);
    state->learned_capacity_uAh =
        SocStorage_ReadU32(record, SOC_OFFSET_SOH_CAPACITY);
    state->soh_valid = record[SOC_OFFSET_SOH_VALID];

    /*
     * CRC chỉ có thể phát hiện thay đổi dữ liệu, không chứng minh giá trị nghiệp vụ hợp lý, vì vậy kiểm tra ngữ nghĩa thêm.
     * Dung lượng định mức không khớp chứng tỏ bản ghi đến từ cấu hình khác, firmware hiện tại không trực tiếp áp dụng.
     */
    if ((remaining > (uint64_t)SOC_FULL_CHARGE_UAS) ||
        (state->soc_centi_percent > 10000U) ||
        (state->rated_capacity_uAh != SOC_RATED_CAPACITY_UAH) ||
        (state->soh_valid > 1U))
    {
        return 0U;
    }

    if (state->soh_valid == 0U)
    {
        state->learned_capacity_uAh = 0U;
    }

    /* Sau khi tất cả xác thực đều vượt qua mới bàn giao trạng thái và số thứ tự cho phía gọi. */
    state->remaining_uAs = (int64_t)remaining;
    *sequence = SocStorage_ReadU32(record, SOC_OFFSET_SEQUENCE);
    return 1U;
}

uint8_t SocStorage_RecordIsErased(
    const uint8_t record[SOC_STORAGE_RECORD_SIZE])
{
    uint8_t index; /* Trạng thái sau khi xóa Flash mỗi bit đều là 1, tức mỗi byte đều là 0xFF. */
    for (index = 0U; index < SOC_STORAGE_RECORD_SIZE; index++)
    {
        if (record[index] != 0xFFU)
        {
            return 0U;
        }
    }
    return 1U;
}

uint8_t SocStorage_IsSequenceNewer(uint32_t candidate,
                                   uint32_t reference)
{
    /*
     * Chuyển sang sai số có dấu để xử lý chính xác trường hợp số thứ tự 32-bit quay vòng từ 0xFFFFFFFF về 0,
     * với điều kiện khoảng cách giữa 2 bản ghi cần so sánh bất kỳ không vượt quá 2^31.
     */
    return (((int32_t)(candidate - reference)) > 0) ? 1U : 0U;
}

#ifndef SOC_STORAGE_CODEC_TEST

#include "app_flash_access.h"
#include "bsp_spi_flash.h"

/* Hai sector 4KB độc lập; chuyển đổi 2 sector cho phép giữ lại bản SOC hợp lệ trước đó khi xóa sector mới. */
#define SOC_SECTOR_A_ADDR ((uint32_t)0x002000UL)
#define SOC_SECTOR_B_ADDR ((uint32_t)0x003000UL)
/* Giá trị sentinel khi quét lúc bật nguồn chưa tìm thấy bản ghi hợp lệ. */
#define SOC_NO_SECTOR     ((uint8_t)0xFFU)

/* Vị trí ghi nối tiếp và số thứ tự mới nhất nhận được từ việc quét một sector đơn lẻ. */
typedef struct
{
    uint16_t next_index;       /* Vị trí sau slot không rỗng cuối cùng. */
    uint32_t newest_sequence;  /* Số thứ tự bản ghi hợp lệ mới nhất trong sector này. */
    uint8_t has_valid;         /* 1=Sector này tồn tại ít nhất một bản ghi hợp lệ. */
} SocSectorScan_t;

static SocStorageDiagnostics_t g_soc_storage_diagnostics; /* Chẩn đoán vận hành lưu trữ. */
static SocPersistentState_t g_soc_restored_state;          /* Bản sao trạng thái mới nhất được chọn ra từ việc quét lúc bật nguồn. */

/* Ánh xạ mã số sector logic 0/1 thành địa chỉ bắt đầu thực tế của Flash ngoài. */
static uint32_t SocStorage_GetSectorAddress(uint8_t sector)
{
    return (sector == 0U) ? SOC_SECTOR_A_ADDR : SOC_SECTOR_B_ADDR;
}

/* So sánh từng byte dùng cho xác thực sau ghi; chỉ trả về 1 khi hoàn toàn giống nhau. */
static uint8_t SocStorage_BufferEquals(const uint8_t *left,
                                       const uint8_t *right,
                                       uint16_t length)
{
    uint16_t index; /* Chỉ số byte đang so sánh hiện tại. */
    for (index = 0U; index < length; index++)
    {
        if (left[index] != right[index])
        {
            return 0U;
        }
    }
    return 1U;
}

/* Sau khi xóa đọc từng slot, chỉ khi toàn bộ 128 bản ghi của cả 4KB đều là 0xFF mới tính là thành công. */
static uint8_t SocStorage_SectorIsErased(uint32_t address)
{
    uint8_t record[SOC_STORAGE_RECORD_SIZE]; /* Bộ đệm đọc slot đơn. */
    uint16_t index;                          /* Mã số slot được quét. */

    for (index = 0U; index < SOC_STORAGE_RECORDS_PER_SECTOR; index++)
    {
        XM25QH32_ReadData(
            address + (uint32_t)index * SOC_STORAGE_RECORD_SIZE,
            record,
            SOC_STORAGE_RECORD_SIZE);
        if (SocStorage_RecordIsErased(record) == 0U)
        {
            return 0U;
        }
    }
    return 1U;
}

static void SocStorage_ScanSector(uint8_t sector,
                                  SocSectorScan_t *scan,
                                  uint8_t *global_valid,
                                  uint32_t *global_sequence,
                                  SocPersistentState_t *global_state,
                                  uint8_t *global_sector)
{
    uint8_t record[SOC_STORAGE_RECORD_SIZE]; /* 32 byte thô của slot hiện tại. */
    SocPersistentState_t state;              /* Kết quả giải mã bản ghi hợp lệ hiện tại. */
    uint32_t sequence;                       /* Số thứ tự bản ghi hiện tại. */
    uint32_t base = SocStorage_GetSectorAddress(sector); /* Địa chỉ đầu của sector. */
    uint16_t index;                          /* Slot hiện tại 0~127. */

    /* Mỗi lần quét đều bắt đầu từ kết quả rỗng. */
    scan->next_index = 0U;
    scan->newest_sequence = 0U;
    scan->has_valid = 0U;

    for (index = 0U; index < SOC_STORAGE_RECORDS_PER_SECTOR; index++)
    {
        XM25QH32_ReadData(
            base + (uint32_t)index * SOC_STORAGE_RECORD_SIZE,
            record,
            SOC_STORAGE_RECORD_SIZE);

        /* Slot đã xóa không có bản ghi; tiếp tục quét nhằm tương thích với trường hợp ở giữa xuất hiện lỗ hổng do ghi thất bại. */
        if (SocStorage_RecordIsErased(record) != 0U)
        {
            continue;
        }

        /* Slot không rỗng không thể ghi NoCheck lần nữa, vị trí tiếp theo ít nhất phải vượt qua nó. */
        scan->next_index = (uint16_t)(index + 1U);
        /* Bản ghi không rỗng nhưng giải mã thất bại được tính vào lỗi và bỏ qua, không cản trở việc quét các bản ghi tiếp theo. */
        if (SocStorage_DecodeRecord(record, &sequence, &state) == 0U)
        {
            g_soc_storage_diagnostics.crc_error_count++;
            continue;
        }

        g_soc_storage_diagnostics.scan_valid_count++;
        /* Cập nhật số thứ tự hợp lệ mới nhất của chính sector này. */
        if ((scan->has_valid == 0U) ||
            (SocStorage_IsSequenceNewer(sequence,
                                        scan->newest_sequence) != 0U))
        {
            scan->has_valid = 1U;
            scan->newest_sequence = sequence;
        }

        /* Cùng chọn ra trạng thái mới nhất toàn cục giữa 2 sector A và B. */
        if ((*global_valid == 0U) ||
            (SocStorage_IsSequenceNewer(sequence,
                                        *global_sequence) != 0U))
        {
            *global_valid = 1U;
            *global_sequence = sequence;
            *global_state = state;
            *global_sector = sector;
        }
    }
}

void SocStorage_Init(void)
{
    SocSectorScan_t scans[2];             /* Kết quả quét riêng biệt của từng sector A và B. */
    SocPersistentState_t newest_state;    /* SOC hợp lệ mới nhất toàn cục. */
    uint32_t newest_sequence = 0U;        /* Số thứ tự mới nhất toàn cục. */
    uint8_t newest_sector = SOC_NO_SECTOR; /* Sector chứa bản ghi mới nhất. */
    uint8_t valid = 0U;                   /* Đã từng phát hiện bản ghi hợp lệ hay chưa. */

    /* Bật nguồn tái thiết lập toàn bộ chẩn đoán vận hành và bộ đệm phục hồi. */
    (void)memset(&g_soc_storage_diagnostics,
                 0,
                 sizeof(g_soc_storage_diagnostics));
    (void)memset(&g_soc_restored_state, 0, sizeof(g_soc_restored_state));

    /* Quét là một transaction Flash hoàn chỉnh, ngăn các truy cập khác xen vào giữa quá trình quét. */
    if (FlashAccess_Lock() == 0U)
    {
        g_soc_storage_diagnostics.write_fail_count++;
        return;
    }

    /* Chỉ giải phóng Mutex sau khi đã quét xong 2 sector. */
    SocStorage_ScanSector(0U, &scans[0], &valid,
                          &newest_sequence, &newest_state, &newest_sector);
    SocStorage_ScanSector(1U, &scans[1], &valid,
                          &newest_sequence, &newest_state, &newest_sector);
    FlashAccess_Unlock();

    /* Khi có bản ghi hợp lệ, tiếp tục ghi nối tiếp từ đuôi của sector chứa nó. */
    if (valid != 0U)
    {
        g_soc_restored_state = newest_state;
        g_soc_storage_diagnostics.restored_valid = 1U;
        g_soc_storage_diagnostics.active_sector = newest_sector;
        g_soc_storage_diagnostics.next_index = scans[newest_sector].next_index;
        g_soc_storage_diagnostics.last_sequence = newest_sequence;
    }
    else
    {
        /* Flash hoàn toàn mới mặc định bắt đầu từ slot có thể ghi đầu tiên của sector A. */
        g_soc_storage_diagnostics.active_sector = 0U;
        g_soc_storage_diagnostics.next_index = scans[0].next_index;
    }
}

uint8_t SocStorage_Load(SocPersistentState_t *state)
{
    /* Chỉ cung cấp dữ liệu cho mô-đun SOC khi quét lúc bật nguồn thu được bản ghi phục hồi hợp lệ. */
    if ((state == 0) ||
        (g_soc_storage_diagnostics.restored_valid == 0U))
    {
        return 0U;
    }
    /* Sao chép struct, phía gọi sẽ không nắm giữ địa chỉ bộ đệm nội bộ của mô-đun này. */
    *state = g_soc_restored_state;
    return 1U;
}

uint8_t SocStorage_Save(const SocPersistentState_t *state)
{
    uint8_t record[SOC_STORAGE_RECORD_SIZE]; /* Bản ghi đã mã hóa chuẩn bị ghi. */
    uint8_t verify[SOC_STORAGE_RECORD_SIZE]; /* Nội dung đọc lại từ Flash sau khi ghi. */
    uint8_t target_sector;                   /* Sector A/B thực tế được ghi lần này. */
    uint16_t target_index;                   /* Slot thực tế được ghi lần này. */
    uint32_t target_address;                 /* Địa chỉ Flash được tính từ sector và slot. */
    uint32_t next_sequence;                  /* Số thứ tự mới nên dùng cho bản ghi lần này. */
    uint32_t decoded_sequence;               /* Số thứ tự giải mã được từ bản ghi đọc lại. */
    SocPersistentState_t decoded_state;      /* Các trường nghiệp vụ giải mã được từ bản ghi đọc lại. */

    /* Trạng thái rỗng không thể lưu. */
    if (state == 0)
    {
        return 0U;
    }
    /* Khóa bao phủ transaction hoàn chỉnh: "xóa / ghi / đọc lại / xác thực / cập nhật metadata". */
    if (FlashAccess_Lock() == 0U)
    {
        g_soc_storage_diagnostics.write_fail_count++;
        return 0U;
    }

    /* Mặc định ghi nối tiếp vào slot trống tiếp theo của sector hoạt động hiện tại, không ghi đè lên bản ghi đã có. */
    target_sector = g_soc_storage_diagnostics.active_sector;
    target_index = g_soc_storage_diagnostics.next_index;
    next_sequence = (g_soc_storage_diagnostics.restored_valid != 0U) ?
                    (g_soc_storage_diagnostics.last_sequence + 1U) : 1U;

    /* Khi sector hiện tại đầy thì chuyển sang sector khác, và xóa sector đích trước. */
    if (target_index >= SOC_STORAGE_RECORDS_PER_SECTOR)
    {
        target_sector = (target_sector == 0U) ? 1U : 0U;
        target_index = 0U;
        target_address = SocStorage_GetSectorAddress(target_sector);
        XM25QH32_SectorErase(target_address);
        g_soc_storage_diagnostics.erase_count++;
        /* Sau khi xóa xác nhận từng slot toàn 0xFF, nếu thất bại thì giữ lại sector cũ và bản ghi hợp lệ cũ. */
        if (SocStorage_SectorIsErased(target_address) == 0U)
        {
            g_soc_storage_diagnostics.erase_fail_count++;
            g_soc_storage_diagnostics.write_fail_count++;
            FlashAccess_Unlock();
            return 0U;
        }
    }

    /* Sau khi mã hóa tính địa chỉ ghi theo "địa chỉ cơ sở sector + slot*32". */
    SocStorage_EncodeRecord(record, next_sequence, state);
    target_address =
        SocStorage_GetSectorAddress(target_sector) +
        (uint32_t)target_index * SOC_STORAGE_RECORD_SIZE;
    /* Slot đích đã được xác nhận có thể ghi qua quét/xóa, do đó sử dụng API ghi trang NoCheck. */
    XM25QH32_Write_NoCheck(record,
                           target_address,
                           SOC_STORAGE_RECORD_SIZE);
    /* Ghi xong lập tức đọc lại ngay, không thể chỉ tin tưởng hàm SPI đã trả về. */
    XM25QH32_ReadData(target_address,
                      verify,
                      SOC_STORAGE_RECORD_SIZE);

    /*
     * Xác thực 3 lớp: byte thô nhất quán, bản ghi giải mã được, số thứ tự khớp kỳ vọng lần này.
     * Bất kỳ điều nào thất bại đều không thể công bố bản ghi này là trạng thái hợp lệ mới nhất.
     */
    if ((SocStorage_BufferEquals(record,
                                 verify,
                                 SOC_STORAGE_RECORD_SIZE) == 0U) ||
        (SocStorage_DecodeRecord(verify,
                                 &decoded_sequence,
                                 &decoded_state) == 0U) ||
        (decoded_sequence != next_sequence))
    {
        g_soc_storage_diagnostics.write_fail_count++;

        /*
         * Slot ghi thất bại trong sector hiện tại có thể đã bị nạp một phần, không thể giả định nó vẫn là
         * toàn 0xFF. Bỏ qua slot này, tránh việc các lần ghi NoCheck sau mãi mãi kẹt ở cùng một vị trí.
         * Nếu thất bại xảy ra trong lúc chuyển đổi sector, giữ lại sector cũ làm sector hoạt động; lần lưu
         * tiếp theo sẽ xóa lại sector đích, bản SOC hợp lệ trước đó vẫn còn tồn tại.
         */
        if (target_sector == g_soc_storage_diagnostics.active_sector)
        {
            g_soc_storage_diagnostics.next_index =
                (uint16_t)(target_index + 1U);
        }
        FlashAccess_Unlock();
        return 0U;
    }

    /* Sau khi tất cả xác thực vượt qua, cuối cùng mới commit sector hoạt động, index và bộ đệm phục hồi. */
    g_soc_storage_diagnostics.active_sector = target_sector;
    g_soc_storage_diagnostics.next_index = (uint16_t)(target_index + 1U);
    g_soc_storage_diagnostics.last_sequence = next_sequence;
    g_soc_storage_diagnostics.restored_valid = 1U;
    g_soc_storage_diagnostics.write_ok_count++;
    g_soc_restored_state = *state;
    FlashAccess_Unlock();
    return 1U;
}

uint8_t SocStorage_ProcessPending(uint32_t now_ms)
{
    SocPersistentState_t state; /* Bản sao lưu bất biến được copy từ app_soc. */

    /* Trả về ngay khi không có yêu cầu lưu, không truy cập Flash. */
    if (Soc_GetSaveRequest(&state) == 0U)
    {
        return 0U;
    }
    /* Khi lưu thất bại không xóa pending trong app_soc, cho phép thử lại lần sau. */
    if (SocStorage_Save(&state) == 0U)
    {
        return 0U;
    }
    /* Chỉ cập nhật mốc đã lưu của mô-đun SOC khi ghi và đọc lại xác thực thành công. */
    Soc_MarkPersisted(&state, now_ms);
    return 1U;
}

void SocStorage_GetDiagnostics(SocStorageDiagnostics_t *diagnostics)
{
    /* CLI chỉ nhận được bản sao, không thể trực tiếp sửa đổi bộ đếm nội bộ của mô-đun này. */
    if (diagnostics != 0)
    {
        *diagnostics = g_soc_storage_diagnostics;
    }
}

#endif
