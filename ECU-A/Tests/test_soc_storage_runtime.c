/* Unit test trên máy host cho logic vận hành 2 sector của SOC: dùng RAM giả lập Flash ngoài. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_soc_storage.h"

#define FAKE_FLASH_SIZE 0x4000U /* Bao phủ hai sector 4KB tại 0x2000 và 0x3000 mà SOC sử dụng. */

static uint8_t g_fake_flash[FAKE_FLASH_SIZE]; /* NOR Flash giả lập bằng RAM. */
static uint8_t g_partial_write_once;          /* 1 = lần ghi tiếp theo chỉ hoàn thành nửa đầu. */

uint8_t FlashAccess_Lock(void)
{
    /* Unit test trên host chạy đơn luồng, khóa luôn thành công. */
    return 1U;
}

void FlashAccess_Unlock(void)
{
}

void XM25QH32_ReadData(uint32_t addr, uint8_t *buffer, uint16_t len)
{
    /* Dùng memcpy giả lập đọc liên tiếp từ Flash và chặn truy cập vượt giới hạn trong bài test. */
    assert((addr + len) <= FAKE_FLASH_SIZE);
    (void)memcpy(buffer, &g_fake_flash[addr], len);
}

void XM25QH32_SectorErase(uint32_t addr)
{
    /* Sau khi xóa NOR Flash, toàn bộ các bit trở về 1. */
    assert((addr + 4096U) <= FAKE_FLASH_SIZE);
    (void)memset(&g_fake_flash[addr], 0xFF, 4096U);
}

void XM25QH32_Write_NoCheck(uint8_t *buffer,
                            uint32_t addr,
                            uint16_t len)
{
    uint16_t index;              /* Byte hiện đang được lập trình. */
    uint16_t write_length = len; /* Có thể bị tiêm lỗi làm ngắn xuống còn nửa bản ghi. */

    assert((addr + len) <= FAKE_FLASH_SIZE);
    if (g_partial_write_once != 0U)
    {
        g_partial_write_once = 0U;
        write_length = (uint16_t)(len / 2U);
    }

    /* Lập trình NOR Flash chỉ có thể ghi bit 1 thành bit 0. */
    for (index = 0U; index < write_length; index++)
    {
        g_fake_flash[addr + index] &= buffer[index];
    }
}

uint8_t Soc_GetSaveRequest(SocPersistentState_t *state)
{
    (void)state;
    return 0U;
}

void Soc_MarkPersisted(const SocPersistentState_t *saved, uint32_t now_ms)
{
    (void)saved;
    (void)now_ms;
}

static SocPersistentState_t MakeState(uint16_t soc_centi, uint32_t time_ms)
{
    SocPersistentState_t state; /* Tạo trạng thái lưu trữ nhất quán từ tỷ lệ SOC kiểm thử. */
    (void)memset(&state, 0, sizeof(state));
    state.remaining_uAs =
        (SOC_FULL_CHARGE_UAS * (int64_t)soc_centi) / 10000LL;
    state.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;
    state.timestamp_ms = time_ms;
    state.soc_centi_percent = soc_centi;
    return state;
}

static void ResetFlash(void)
{
    /* Mỗi kịch bản bắt đầu từ trạng thái xóa trắng hoàn toàn và xóa cờ tiêm lỗi dùng một lần. */
    (void)memset(g_fake_flash, 0xFF, sizeof(g_fake_flash));
    g_partial_write_once = 0U;
}

static void TestNewestValidRecordAndCorruptionFallback(void)
{
    SocPersistentState_t first = MakeState(7000U, 100U);
    SocPersistentState_t second = MakeState(6900U, 200U);
    SocPersistentState_t restored;

    ResetFlash();
    SocStorage_Init();
    assert(SocStorage_Save(&first) == 1U);
    assert(SocStorage_Save(&second) == 1U);

    SocStorage_Init();
    assert(SocStorage_Load(&restored) == 1U);
    assert(restored.soc_centi_percent == 6900U);

    /* Sau khi làm hỏng CRC của bản ghi mới nhất, quá trình quét khi khởi động lại phải lùi về bản ghi hợp lệ trước đó. */
    g_fake_flash[0x002000U + 32U + 31U] ^= 0x01U;
    SocStorage_Init();
    assert(SocStorage_Load(&restored) == 1U);
    assert(restored.soc_centi_percent == 7000U);
}

static void TestRolloverPreservesLatestState(void)
{
    SocPersistentState_t state;
    SocPersistentState_t restored;
    uint16_t index;

    ResetFlash();
    SocStorage_Init();

    /* Ghi đầy 128 bản ghi vào sector A, sau đó ghi thêm 1 bản ghi để kích hoạt xóa sector B và chuyển đổi. */
    for (index = 0U; index < SOC_STORAGE_RECORDS_PER_SECTOR; index++)
    {
        state = MakeState((uint16_t)(8000U - index), index);
        assert(SocStorage_Save(&state) == 1U);
    }

    state = MakeState(7000U, 1000U);
    assert(SocStorage_Save(&state) == 1U);

    SocStorage_Init();
    assert(SocStorage_Load(&restored) == 1U);
    assert(restored.soc_centi_percent == 7000U);
}

static void TestPartialWriteSkipsPoisonedSlot(void)
{
    SocPersistentState_t state;
    SocPersistentState_t restored;
    SocStorageDiagnostics_t diagnostics;

    ResetFlash();
    SocStorage_Init();
    state = MakeState(6000U, 1U);
    assert(SocStorage_Save(&state) == 1U);

    /* Giả lập mất nguồn tạo ra nửa bản ghi; sau khi thất bại next_index bắt buộc phải bỏ qua slot bị hỏng. */
    g_partial_write_once = 1U;
    state = MakeState(5900U, 2U);
    assert(SocStorage_Save(&state) == 0U);
    SocStorage_GetDiagnostics(&diagnostics);
    assert(diagnostics.next_index == 2U);

    state = MakeState(5800U, 3U);
    assert(SocStorage_Save(&state) == 1U);
    SocStorage_Init();
    assert(SocStorage_Load(&restored) == 1U);
    assert(restored.soc_centi_percent == 5800U);
}

int main(void)
{
    TestNewestValidRecordAndCorruptionFallback();
    TestRolloverPreservesLatestState();
    TestPartialWriteSkipsPoisonedSlot();
    puts("SOC storage runtime tests passed");
    return 0;
}
