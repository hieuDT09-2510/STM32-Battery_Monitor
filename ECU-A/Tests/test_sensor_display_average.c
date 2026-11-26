#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "app_sensor.h"

#define TEST_ADC_QUEUE_SIZE 128U

typedef struct
{
    uint16_t adc0;
    uint16_t adc1;
} TestAdcSample_t;

static TestAdcSample_t g_adc_queue[TEST_ADC_QUEUE_SIZE];
static uint8_t g_adc_read_index;
static uint8_t g_adc_write_index;

static void PushAdc(uint16_t adc0, uint16_t adc1)
{
    assert(g_adc_write_index < TEST_ADC_QUEUE_SIZE);
    g_adc_queue[g_adc_write_index].adc0 = adc0;
    g_adc_queue[g_adc_write_index].adc1 = adc1;
    g_adc_write_index++;
}

uint8_t ADC_ReadSafe(uint16_t *ch0, uint16_t *ch1)
{
    if ((ch0 == 0) || (ch1 == 0) ||
        (g_adc_read_index >= g_adc_write_index))
    {
        return 0U;
    }

    *ch0 = g_adc_queue[g_adc_read_index].adc0;
    *ch1 = g_adc_queue[g_adc_read_index].adc1;
    g_adc_read_index++;
    return 1U;
}

static uint16_t ExpectedCellMilli(uint16_t adc0)
{
    float voltage_v = (float)adc0 * 3.27f * 1.5f / 4095.0f;
    return (uint16_t)(voltage_v * 1000.0f + 0.5f);
}

static uint16_t SnapshotCellMilli(const SensorSample_t *sample)
{
    return (uint16_t)(sample->cell_voltage_v * 1000.0f + 0.5f);
}

static void TestPartialAndFullWindow(void)
{
    SensorSample_t raw;
    SensorSample_t display;
    uint16_t adc;

    g_adc_read_index = 0U;
    g_adc_write_index = 0U;
    Sensor_ServiceInit();

    PushAdc(1000U, 2048U);
    assert(Sensor_ServiceUpdate(1U) == 1U);
    assert(Sensor_GetLatest(&raw) == 1U);
    assert(Sensor_GetLatestDisplay(&display) == 1U);
    assert(SnapshotCellMilli(&raw) == ExpectedCellMilli(1000U));
    assert(SnapshotCellMilli(&display) ==
           (uint16_t)(((ExpectedCellMilli(1000U) + 5U) / 10U) * 10U));
    assert(display.sequence == raw.sequence);
    assert(display.timestamp_ms == raw.timestamp_ms);

    for (adc = 1001U; adc <= 1015U; adc++)
    {
        PushAdc(adc, 2048U);
        assert(Sensor_ServiceUpdate((uint32_t)(adc - 999U)) == 1U);
    }

    assert(Sensor_GetLatest(&raw) == 1U);
    assert(Sensor_GetLatestDisplay(&display) == 1U);
    assert(SnapshotCellMilli(&raw) == ExpectedCellMilli(1015U));
    assert(SnapshotCellMilli(&display) ==
           (uint16_t)(((ExpectedCellMilli(1000U) + 5U) / 10U) * 10U));

    PushAdc(1016U, 2048U);
    assert(Sensor_ServiceUpdate(17U) == 1U);
    assert(Sensor_GetLatest(&raw) == 1U);
    assert(Sensor_GetLatestDisplay(&display) == 1U);
    assert(SnapshotCellMilli(&raw) == ExpectedCellMilli(1016U));
    assert(SnapshotCellMilli(&display) ==
           (uint16_t)(((ExpectedCellMilli(1000U) + 5U) / 10U) * 10U));
    assert(display.sequence == raw.sequence);
    assert(display.timestamp_ms == 17U);
}

static void PublishRepeated(uint16_t adc0,
                            uint16_t count,
                            uint32_t *timestamp_ms)
{
    uint16_t index;

    for (index = 0U; index < count; index++)
    {
        PushAdc(adc0, 2048U);
        (*timestamp_ms)++;
        assert(Sensor_ServiceUpdate(*timestamp_ms) == 1U);
    }
}

static void TestDisplayVoltageUsesTenMillivoltDeadband(void)
{
    SensorSample_t display;
    uint16_t initial_display_mv;
    uint32_t timestamp_ms = 0U;

    g_adc_read_index = 0U;
    g_adc_write_index = 0U;
    Sensor_ServiceInit();

    PublishRepeated(3000U, 16U, &timestamp_ms);
    assert(Sensor_GetLatestDisplay(&display) == 1U);
    initial_display_mv =
        (uint16_t)(((ExpectedCellMilli(3000U) + 5U) / 10U) * 10U);
    assert(SnapshotCellMilli(&display) == initial_display_mv);

    /* Giá trị trung bình mới chênh lệch so với giá trị hiển thị hiện tại chưa tới 10mV, hiển thị bắt buộc phải giữ nguyên. */
    PublishRepeated(3004U, 16U, &timestamp_ms);
    assert(Sensor_GetLatestDisplay(&display) == 1U);
    assert(SnapshotCellMilli(&display) == initial_display_mv);

    /* Sau khi vượt qua vùng chết (deadband) 10mV, cập nhật sang nấc hiển thị 10mV mới. */
    PublishRepeated(3010U, 16U, &timestamp_ms);
    assert(Sensor_GetLatestDisplay(&display) == 1U);
    assert(SnapshotCellMilli(&display) == 3600U);
}

int main(void)
{
    SensorSample_t sample = {0};

    TestPartialAndFullWindow();
    TestDisplayVoltageUsesTenMillivoltDeadband();
    sample.sequence = 1U;
    sample.timestamp_ms = 1006U;
    assert(Sensor_CalculateAgeMs(&sample, 1000U) == 0U);
    assert(Sensor_CalculateAgeMs(&sample, 1016U) == 10U);
    puts("sensor display average tests passed");
    return 0;
}
