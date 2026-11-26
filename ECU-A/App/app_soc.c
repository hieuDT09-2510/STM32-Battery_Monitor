/*
 * Tên tệp: app_soc.c
 *
 * Luồng dữ liệu:
 *   SensorTask phát hành snapshot điện áp -> Soc_UpdateFromVoltage()
 *        -> Nội suy tuyến tính ra SOC -> CAN/CLI/nhật ký lỗi đọc snapshot.
 *
 * Quy ước đồng thời:
 * SensorTask là tác vụ ghi duy nhất; các tác vụ khác sao chép snapshot qua Soc_GetSnapshot().
 * Vùng găng chỉ bảo vệ việc cập nhật trạng thái RAM ngắn hoặc sao chép struct.
 */
#include "app_soc.h"

#include <string.h>

#ifndef SOC_HOST_TEST
#include "FreeRTOS.h"
#include "task.h"
/* Trên board dùng vùng găng FreeRTOS để đảm bảo snapshot không bị đọc dở nửa mới, nửa cũ. */
#define SOC_ENTER_CRITICAL() taskENTER_CRITICAL()
#define SOC_EXIT_CRITICAL()  taskEXIT_CRITICAL()
#else
/* Unit test trên host chạy đơn luồng, không phụ thuộc FreeRTOS. */
#define SOC_ENTER_CRITICAL() ((void)0)
#define SOC_EXIT_CRITICAL()  ((void)0)
#endif

static SocSnapshot_t g_soc_snapshot; /* Snapshot SOC đang được phát hành ra ngoài. */

void Soc_Init(void)
{
    SOC_ENTER_CRITICAL();
    (void)memset(&g_soc_snapshot, 0, sizeof(g_soc_snapshot));
    SOC_EXIT_CRITICAL();
}

uint16_t Soc_CalculateCentiPercent(uint16_t cell_mv)
{
    uint32_t span_mv = (uint32_t)SOC_FULL_VOLTAGE_MV -
                       (uint32_t)SOC_EMPTY_VOLTAGE_MV;

    if (cell_mv <= SOC_EMPTY_VOLTAGE_MV)
    {
        return 0U;
    }
    if (cell_mv >= SOC_FULL_VOLTAGE_MV)
    {
        return 10000U;
    }

    /* Nhân 10000 để đổi sang đơn vị 0.01%; cộng nửa khoảng chia để làm tròn số nguyên. */
    return (uint16_t)(
        (((uint32_t)cell_mv - SOC_EMPTY_VOLTAGE_MV) * 10000UL +
         span_mv / 2UL) / span_mv);
}

void Soc_UpdateFromVoltage(uint16_t cell_mv, uint32_t timestamp_ms)
{
    uint16_t soc_centi = Soc_CalculateCentiPercent(cell_mv);

    /* Bão hòa số thứ tự không để tràn vòng, tránh quay về 0 bị hiểu nhầm là "chưa có SOC hợp lệ". */
    SOC_ENTER_CRITICAL();
    if (g_soc_snapshot.sequence < 0xFFFFFFFFUL)
    {
        g_soc_snapshot.sequence++;
    }
    g_soc_snapshot.timestamp_ms = timestamp_ms;
    g_soc_snapshot.cell_mv = cell_mv;
    g_soc_snapshot.soc_centi_percent = soc_centi;
    g_soc_snapshot.valid = 1U;
    SOC_EXIT_CRITICAL();
}

uint8_t Soc_GetSnapshot(SocSnapshot_t *snapshot)
{
    if (snapshot == 0)
    {
        return 0U;
    }

    SOC_ENTER_CRITICAL();
    if (g_soc_snapshot.valid == 0U)
    {
        SOC_EXIT_CRITICAL();
        return 0U;
    }
    *snapshot = g_soc_snapshot;
    SOC_EXIT_CRITICAL();
    return 1U;
}
