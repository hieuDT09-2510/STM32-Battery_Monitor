#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "app_current.h"

static void TestPartialWindowAndSignedCurrent(void)
{
    CurrentSample_t raw;
    CurrentSample_t display;

    Current_ServiceInit();
    Current_ServicePublish(100000L,
                           3700000UL,
                           10000000L,
                           370000L,
                           10U);

    assert(Current_GetLatest(&raw) == 1U);
    assert(Current_GetLatestDisplay(&display) == 1U);
    assert(raw.current_uA == 100000L);
    assert(display.current_uA == 100000L);
    assert(display.bus_voltage_uV == 3700000UL);
    assert(display.power_uW == 370000L);

    Current_ServicePublish(-100000L,
                           3800000UL,
                           -10000000L,
                           -380000L,
                           20U);

    assert(Current_GetLatest(&raw) == 1U);
    assert(Current_GetLatestDisplay(&display) == 1U);
    assert(raw.current_uA == -100000L);
    assert(display.current_uA == 0L);
    assert(display.bus_voltage_uV == 3750000UL);
    assert(display.shunt_voltage_nV == 0L);
    assert(display.power_uW == 0L);
    assert(display.sequence == raw.sequence);
    assert(display.timestamp_ms == raw.timestamp_ms);
}

static void TestFullWindowRollsForward(void)
{
    CurrentSample_t raw;
    CurrentSample_t display;
    uint32_t index;

    Current_ServiceInit();
    for (index = 1U; index <= 8U; index++)
    {
        Current_ServicePublish((int32_t)(index * 10000L),
                               3600000UL + index * 1000UL,
                               (int32_t)(index * 1000000L),
                               0L,
                               index * 10U);
    }

    assert(Current_GetLatest(&raw) == 1U);
    assert(Current_GetLatestDisplay(&display) == 1U);
    assert(raw.current_uA == 80000L);
    assert(display.current_uA == 40000L);
    assert(display.bus_voltage_uV == 3600000UL);
    assert(display.shunt_voltage_nV == 4500000L);
    assert(display.power_uW == 144000L);

    Current_ServicePublish(90000L,
                           3609000UL,
                           9000000L,
                           0L,
                           90U);

    assert(Current_GetLatest(&raw) == 1U);
    assert(Current_GetLatestDisplay(&display) == 1U);
    assert(raw.current_uA == 90000L);
    assert(display.current_uA == 60000L);
    assert(display.bus_voltage_uV == 3600000UL);
    assert(display.shunt_voltage_nV == 5500000L);
    assert(display.power_uW == 216000L);
    assert(display.sequence == raw.sequence);
    assert(display.timestamp_ms == 90U);
}

static void PublishCurrentRepeated(int32_t current_uA,
                                   uint32_t bus_voltage_uV,
                                   uint32_t count,
                                   uint32_t *timestamp_ms)
{
    uint32_t index;

    for (index = 0U; index < count; index++)
    {
        *timestamp_ms += 10U;
        Current_ServicePublish(current_uA,
                               bus_voltage_uV,
                               current_uA * 100L,
                               0L,
                               *timestamp_ms);
    }
}

static void TestDisplayCurrentAndBusVoltageUseDeadband(void)
{
    CurrentSample_t display;
    uint32_t timestamp_ms = 0U;

    Current_ServiceInit();
    PublishCurrentRepeated(335000L, 3675000UL, 8U, &timestamp_ms);
    assert(Current_GetLatestDisplay(&display) == 1U);
    assert(display.current_uA == 340000L);
    assert(display.bus_voltage_uV == 3680000UL);

    /* Mức thay đổi vẫn nằm trong vùng chết (deadband) 10mA/10mV, giá trị hiển thị giữ nguyên. */
    PublishCurrentRepeated(344000L, 3684000UL, 8U, &timestamp_ms);
    assert(Current_GetLatestDisplay(&display) == 1U);
    assert(display.current_uA == 340000L);
    assert(display.bus_voltage_uV == 3680000UL);

    /* Sau khi vượt qua vùng chết (deadband), cập nhật sang nấc 10mA/10mV mới. */
    PublishCurrentRepeated(351000L, 3691000UL, 8U, &timestamp_ms);
    assert(Current_GetLatestDisplay(&display) == 1U);
    assert(display.current_uA == 350000L);
    assert(display.bus_voltage_uV == 3690000UL);
    assert(display.power_uW == 1291500L);
}

int main(void)
{
    TestPartialWindowAndSignedCurrent();
    TestFullWindowRollsForward();
    TestDisplayCurrentAndBusVoltageUseDeadband();
    puts("current display average tests passed");
    return 0;
}
