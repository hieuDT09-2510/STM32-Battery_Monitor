/*
 * Ten file: test_dash_comm_timeout.c
 *
 * Tac dung mo-dun:
 * Kiem tra timeout truyen thong doc lap va hien thi ha cap OLED cua ECU-B doi voi 0x301, 0x302, 0x303 va 0x304.
 *
 * Trong tam kiem thu:
 * 1. 3 loai ban tin su dung thoi diem nhan hop le cuoi cung rieng biet;
 * 2. Mot loai ban tin den khong duoc che lap viec mat loai ban tin khac;
 * 3. Mat don le mot loai ban tin chi lam ha cap hien thi du lieu tuong ung;
 * 4. Khi tat ca ban tin BMS deu bi mat thi hien thi ngat toan bo CAN.
 */

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
    memset(g_oled_lines, 0, sizeof(g_oled_lines));
}


void OLED_Refresh(void)
{
}


void OLED_ShowString(uint8_t x, uint8_t y, const char *text)
{
    (void)x;

    if (y < 8U)
    {
        (void)snprintf(
            g_oled_lines[y],
            sizeof(g_oled_lines[y]),
            "%s",
            text);
    }
}


static void Receive301(uint32_t tick, uint8_t alive)
{
    CAN_Frame_t frame;

    BMS_PackVoltage(
        &frame,
        3850U,
        75U,
        CHG_STATE_IDLE,
        alive);

    g_test_tick = tick;
    Dash_UpdateData(&frame);
}


static void Receive302(uint32_t tick, uint8_t alive)
{
    CAN_Frame_t frame;

    BMS_PackTemp(&frame, 26U, alive);

    g_test_tick = tick;
    Dash_UpdateData(&frame);
}


static void Receive303(uint32_t tick, uint8_t alive, uint16_t fault)
{
    CAN_Frame_t frame;

    BMS_PackFault(&frame, fault, alive);

    g_test_tick = tick;
    Dash_UpdateData(&frame);
}

static void Receive304(uint32_t tick, uint8_t alive, int16_t current_ma)
{
    CAN_Frame_t frame;

    BMS_PackCurrent(&frame, current_ma, 0, 75U, alive);
    g_test_tick = tick;
    Dash_UpdateData(&frame);
}


int main(void)
{
    /* Ranh gioi cua cua so cho sau khi cap nguon. */
    Dash_Init();

    Dash_UpdateCommStatus(499U);
    assert(g_dash_data.comm_301_timeout == 0U);
    assert(g_dash_data.comm_302_timeout == 0U);
    assert(g_dash_data.comm_303_timeout == 0U);
    assert(g_dash_data.comm_304_timeout == 0U);

    Dash_UpdateCommStatus(500U);
    assert(g_dash_data.comm_301_timeout == 1U);
    assert(g_dash_data.comm_302_timeout == 0U);
    assert(g_dash_data.comm_303_timeout == 1U);
    assert(g_dash_data.comm_304_timeout == 1U);

    Dash_UpdateCommStatus(1500U);
    assert(g_dash_data.comm_302_timeout == 1U);

    /* Sau khi nhan 4 loai ban tin hop le, trang thai timeout cua tung loai deu duoc phuc hoi hoan toan. */
    Receive301(2000U, 0U);
    Receive302(2000U, 0U);
    Receive303(2000U, 0U, 0U);
    Receive304(2000U, 0U, 120);

    assert(g_dash_data.comm_301_timeout == 0U);
    assert(g_dash_data.comm_302_timeout == 0U);
    assert(g_dash_data.comm_303_timeout == 0U);
    assert(g_dash_data.comm_304_timeout == 0U);

    /* Tai 499ms, ban tin chu ky 100ms chua dat toi ranh gioi timeout 500ms. */
    Dash_UpdateCommStatus(2499U);
    assert(g_dash_data.comm_301_timeout == 0U);
    assert(g_dash_data.comm_303_timeout == 0U);
    assert(g_dash_data.comm_304_timeout == 0U);

    /* Tai 500ms, 0x301 va 0x303 bi timeout, 0x302 van o trong cua so 1500ms. */
    Dash_UpdateCommStatus(2500U);
    assert(g_dash_data.comm_301_timeout == 1U);
    assert(g_dash_data.comm_302_timeout == 0U);
    assert(g_dash_data.comm_303_timeout == 1U);
    assert(g_dash_data.comm_304_timeout == 1U);

    Dash_UpdateCommStatus(3499U);
    assert(g_dash_data.comm_302_timeout == 0U);

    Dash_UpdateCommStatus(3500U);
    assert(g_dash_data.comm_302_timeout == 1U);

    /* Chi phuc hoi 0x303, khong duoc tien the xoa timeout cua 0x301 va 0x302. */
    Receive303(3600U, 1U, CAN_FAULT_OV);
    assert(g_dash_data.comm_301_timeout == 1U);
    assert(g_dash_data.comm_302_timeout == 1U);
    assert(g_dash_data.comm_303_timeout == 0U);

    /* Chi mat 0x301: Cac du lieu hop le khac tiep tuc hien thi. */
    Dash_Init();
    Receive301(100U, 0U);
    Receive302(100U, 0U);
    Receive303(100U, 0U, CAN_FAULT_NTC_OPEN);
    Receive304(100U, 0U, 120);
    Receive302(700U, 1U);
    Receive303(700U, 1U, CAN_FAULT_NTC_OPEN);
    Receive304(700U, 1U, 121);

    g_test_tick = 700U;
    Dash_RefreshDisplay();

    assert(strcmp(g_oled_lines[1], "STATUS:CAN LOST") == 0);
    assert(strcmp(g_oled_lines[2], "Cell:CAN LOST") == 0);
    assert(strcmp(g_oled_lines[3], "Temp:26C") == 0);
    assert(strcmp(g_oled_lines[4], "SOC:75% SOH:--") == 0);
    assert(strcmp(g_oled_lines[5], "I:+121mA") == 0);
    assert(strcmp(g_oled_lines[6], "F:0008 NTCO") == 0);

    /* Chi mat 0x302: Chi danh dau dong nhiet do la khong kha dung. */
    Dash_Init();
    Receive301(100U, 0U);
    Receive302(100U, 0U);
    Receive303(100U, 0U, 0U);
    Receive304(100U, 0U, -80);
    Receive301(1700U, 1U);
    Receive303(1700U, 1U, 0U);
    Receive304(1700U, 1U, -81);

    g_test_tick = 1700U;
    Dash_RefreshDisplay();

    assert(strcmp(g_oled_lines[1], "STANDBY") == 0);
    assert(strcmp(g_oled_lines[2], "Cell:3.850V") == 0);
    assert(strcmp(g_oled_lines[3], "Temp:CAN LOST") == 0);
    assert(strcmp(g_oled_lines[4], "SOC:75% SOH:75%") == 0);
    assert(strcmp(g_oled_lines[5], "I:-81mA") == 0);
    assert(strcmp(g_oled_lines[6], "Fault:NONE") == 0);

    /* Chi mat 0x303: Giu lai mat na tin cay cuoi cung va danh dau mat CAN. */
    Dash_Init();
    Receive301(100U, 0U);
    Receive302(100U, 0U);
    Receive303(100U, 0U, CAN_FAULT_OV);
    Receive304(100U, 0U, 50);
    Receive301(700U, 1U);
    Receive302(700U, 1U);
    Receive304(700U, 1U, 51);

    g_test_tick = 700U;
    Dash_RefreshDisplay();

    assert(strcmp(g_oled_lines[6], "F:0001 CAN LOST") == 0);

    /* Tat ca ban tin BMS deu dung qua 1000ms, su dung canh bao truyen thong toan man hinh. */
    Dash_Init();
    Receive301(100U, 0U);
    Receive302(100U, 0U);
    Receive303(100U, 0U, 0U);
    Receive304(100U, 0U, 10);

    g_test_tick = 1201U;
    Dash_RefreshDisplay();

    assert(strcmp(g_oled_lines[0], "BMS DASH") == 0);
    assert(strcmp(g_oled_lines[2], "CAN LOST!") == 0);

    puts("dashboard independent CAN timeout test passed");
    return 0;
}
