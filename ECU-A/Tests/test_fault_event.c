/*
 * Host-side unit tests for the hardware-independent fault event queue.
 * These tests run with MinGW GCC and do not access STM32 registers.
 */
#include <assert.h>
#include <stdio.h>

#include "app_fault_event.h"

static void TestEventPayloadHasStableSize(void)
{
    /* Định dạng bản ghi Flash cần đồng nhất giữa các trình biên dịch, payload bắt buộc cố định 16 byte. */
    assert(sizeof(FaultEvent_t) == 16U);
}

static SensorSample_t MakeSample(void)
{
    SensorSample_t sample = {0};

    sample.sequence = 1U;
    sample.timestamp_ms = 1234U;
    sample.cell_voltage_v = 3.876f;
    sample.temperature_c = 25.34f;
    sample.voltage_valid = 1U;
    sample.temperature_valid = 1U;
    sample.ntc_input = NTC_INPUT_NORMAL;
    return sample;
}

static void TestPushAndPopPreserveContext(void)
{
    SensorSample_t sample = MakeSample();
    FaultEvent_t event;

    FaultEvent_Init();
    assert(FaultEvent_Push(FAULT_EVENT_CONFIRMED,
                           0x0003U,
                           0x0007U,
                           &sample,
                           3U,
                           1234U) == 1U);
    assert(g_fault_event_debug.depth == 1U);
    assert(g_fault_event_debug.pushed_count == 1U);

    assert(FaultEvent_Pop(&event, 0U) == 1U);
    assert(event.timestamp_ms == 1234U);
    assert(event.event_type == FAULT_EVENT_CONFIRMED);
    assert(event.changed_mask == 0x0003U);
    assert(event.active_mask == 0x0007U);
    assert(event.cell_voltage_mv == 3876U);
    assert(event.temperature_dC == 253);
    assert(event.bms_state == 3U);
    assert(event.valid_flags ==
           (FAULT_EVENT_VALID_VOLTAGE | FAULT_EVENT_VALID_TEMPERATURE));
    assert(g_fault_event_debug.depth == 0U);
    assert(g_fault_event_debug.processed_count == 1U);
}

static void TestInvalidTemperatureKeepsValidVoltage(void)
{
    SensorSample_t sample = MakeSample();
    FaultEvent_t event;

    sample.temperature_valid = 0U;
    sample.temperature_c = 0.0f;

    FaultEvent_Init();
    assert(FaultEvent_Push(FAULT_EVENT_CONFIRMED,
                           0x0008U,
                           0x0008U,
                           &sample,
                           4U,
                           2000U) == 1U);
    assert(FaultEvent_Pop(&event, 0U) == 1U);
    assert(event.valid_flags == FAULT_EVENT_VALID_VOLTAGE);
    assert(event.cell_voltage_mv == 3876U);
    assert(event.temperature_dC == 0);
}

static void TestFullQueueDropsNewestWithoutBlocking(void)
{
    SensorSample_t sample = MakeSample();
    uint8_t index;

    FaultEvent_Init();

    for (index = 0U; index < FAULT_EVENT_QUEUE_CAPACITY; index++)
    {
        assert(FaultEvent_Push(FAULT_EVENT_CONFIRMED,
                               0x0001U,
                               0x0001U,
                               &sample,
                               3U,
                               index) == 1U);
    }

    assert(g_fault_event_debug.depth == FAULT_EVENT_QUEUE_CAPACITY);
    assert(FaultEvent_Push(FAULT_EVENT_CLEARED,
                           0x0001U,
                           0x0000U,
                           &sample,
                           4U,
                           99U) == 0U);
    assert(g_fault_event_debug.depth == FAULT_EVENT_QUEUE_CAPACITY);
    assert(g_fault_event_debug.pushed_count == FAULT_EVENT_QUEUE_CAPACITY);
    assert(g_fault_event_debug.dropped_count == 1U);
}

static void TestRejectsInvalidInput(void)
{
    SensorSample_t sample = MakeSample();
    FaultEvent_t event;

    FaultEvent_Init();
    assert(FaultEvent_Push(FAULT_EVENT_CONFIRMED,
                           0U,
                           0U,
                           &sample,
                           3U,
                           0U) == 0U);
    assert(FaultEvent_Push((FaultEventType_t)0,
                           0x0001U,
                           0x0001U,
                           &sample,
                           3U,
                           0U) == 0U);
    assert(FaultEvent_Pop(0, 0U) == 0U);
    assert(FaultEvent_Pop(&event, 0U) == 0U);
}

int main(void)
{
    TestEventPayloadHasStableSize();
    TestPushAndPopPreserveContext();
    TestInvalidTemperatureKeepsValidVoltage();
    TestFullQueueDropsNewestWithoutBlocking();
    TestRejectsInvalidInput();
    puts("fault event queue tests passed");
    return 0;
}
