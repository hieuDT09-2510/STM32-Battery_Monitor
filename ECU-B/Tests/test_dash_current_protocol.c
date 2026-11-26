/* Test host ECU-B: Kiem tra viec phan tich, kiem tra tinh toan ven va hien thi ban tin dong dien/cong suat 0x304. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_dash.h"

static uint32_t g_test_tick;
static char g_oled_lines[8][32];

uint32_t GetTick(void)
{
    return g_test_tick;
}

void OLED_Clear(void)
{
    (void)memset(g_oled_lines, 0, sizeof(g_oled_lines));
}

void OLED_Refresh(void)
{
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *text)
{
    (void)x;
    if (y < 8U)
    {
        (void)snprintf(g_oled_lines[y], sizeof(g_oled_lines[y]),
                       "%s", text);
    }
}

static void Receive304(uint32_t tick,
                       int16_t current_ma,
                       int16_t power_mw,
                       uint8_t soc,
                       uint8_t alive)
{
    CAN_Frame_t frame;

    BMS_PackCurrent(&frame, current_ma, power_mw, soc, alive);
    g_test_tick = tick;
    Dash_UpdateData(&frame);
}

int main(void)
{
    CAN_Frame_t frame;

    Dash_Init();

    /* Khi chua nhan duoc khung dau tien phai hien thi ro rang la dang cho, khong duoc nham RAM sau khi xoa ve 0 la 0mA. */
    g_dash_data.valid = 1U;
    g_test_tick = 100U;
    Dash_RefreshDisplay();
    assert(strcmp(g_oled_lines[5], "I:WAIT") == 0);

    Receive304(100U, 125, 487, 65U, 14U);
    assert(g_dash_data.current_ma == 125);
    assert(g_dash_data.power_mw == 487);
    assert(g_dash_data.soc == 65U);
    assert(g_dash_data.received_304 == 1U);
    assert(g_dash_data.last_304_alive == 14U);
    assert(g_dash_data.crc_304_error == 0U);

    /* Alive doc lap cho phep quay vong tu 15 ve 0. */
    Receive304(200U, -125, -487, 64U, 15U);
    Receive304(300U, -250, -975, 63U, 0U);
    assert(g_dash_data.current_ma == -250);
    assert(g_dash_data.power_mw == -975);
    assert(g_dash_data.alive_304_error_count == 0U);

    /* Nhay so chi ghi nhan chan doan, gia tri hien tai co CRC dung van co the su dung. */
    Receive304(400U, 300, 1170, 62U, 3U);
    assert(g_dash_data.current_ma == 300);
    assert(g_dash_data.alive_304_error == 1U);
    assert(g_dash_data.alive_304_error_count == 1U);

    /* Loi CRC khong duoc ghi de len du lieu tin cay cuoi cung va thoi gian nhan. */
    BMS_PackCurrent(&frame, 700, 2730, 61U, 4U);
    frame.data[0] ^= 0x01U;
    g_test_tick = 450U;
    Dash_UpdateData(&frame);
    assert(g_dash_data.current_ma == 300);
    assert(g_dash_data.last_304_rx_tick == 400U);
    assert(g_dash_data.crc_304_error == 1U);
    assert(g_dash_data.crc_304_error_count == 1U);

    /* Khi ban tin dong dien khong cap nhat sau 500ms, chi danh dau dong dong dien la khong hop le. */
    Dash_UpdateCommStatus(900U);
    assert(g_dash_data.comm_304_timeout == 1U);

    /* Tao du lieu dong ho day du, kiem tra hien thi dong dien am/duong va vi tri dong loi. */
    Dash_Init();
    BMS_PackVoltage(&frame, 3800U, 88U, CAN_STATUS_MONITOR, 0U);
    g_test_tick = 100U;
    Dash_UpdateData(&frame);
    BMS_PackTemp(&frame, 25U, 0U);
    Dash_UpdateData(&frame);
    BMS_PackFault(&frame, 0U, 0U);
    Dash_UpdateData(&frame);
    Receive304(100U, -125, -475, 65U, 0U);

    g_test_tick = 100U;
    Dash_RefreshDisplay();
    assert(strcmp(g_oled_lines[1], "MONITOR") == 0);
    assert(strcmp(g_oled_lines[4], "SOC:65% SOH:88%") == 0);
    assert(strcmp(g_oled_lines[5], "I:-125mA") == 0);
    assert(strcmp(g_oled_lines[6], "Fault:NONE") == 0);

    puts("dashboard 0x304 current protocol test passed");
    return 0;
}
