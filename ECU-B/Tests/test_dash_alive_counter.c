/*
 * Test phia host: Kiem tra dong bo lan dau, tinh lien tuc, quay vong (wrap-around) va phat hien nhay so Alive Counter 0x301 cua ECU-B.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "app_dash.h"

static uint32_t g_test_tick;

uint32_t GetTick(void)
{
    return g_test_tick;
}

void OLED_Clear(void) {}
void OLED_Refresh(void) {}
void OLED_ShowString(uint8_t x, uint8_t y, const char *text)
{
    (void)x;
    (void)y;
    (void)text;
}

static void Receive301(CAN_Frame_t *frame, uint8_t alive)
{
    /* Su dung ham giao thuc thuc te de tao dong thoi du lieu nghiep vu, Alive va CRC. */
    BMS_PackVoltage(
        frame,
        3482U,
        40U,
        CHG_STATE_IDLE,
        alive);

    g_test_tick += 100U;
    Dash_UpdateData(frame);
}

int main(void)
{
    CAN_Frame_t frame = {0};

    Dash_Init();

    /* Khung dau tien chi thiet lap moc so sanh, bat ky gia tri khoi tao nao cung khong duoc bao loi. */
    Receive301(&frame, 14U);
    assert(g_dash_data.last_301_alive == 14U);
    assert(g_dash_data.alive_301_error == 0U);
    assert(g_dash_data.alive_301_error_count == 0U);

    /* 14->15->0 thuoc ve tang tien binh thuong va quay vong bo dem 4-bit. */
    Receive301(&frame, 15U);
    Receive301(&frame, 0U);
    assert(g_dash_data.last_301_alive == 0U);
    assert(g_dash_data.alive_301_error == 0U);
    assert(g_dash_data.alive_301_error_count == 0U);

    /* Cho doi 1 nhung lai nhan 3, cho thay so thu tu khung khong lien tuc. */
    Receive301(&frame, 3U);
    assert(g_dash_data.last_301_alive == 3U);
    assert(g_dash_data.alive_301_error == 1U);
    assert(g_dash_data.alive_301_error_count == 1U);

    /* Sau do 3->4 phuc hoi tinh lien tuc, loi hien tai duoc xoa, so lan tich luy trong lich su duoc giu lai. */
    Receive301(&frame, 4U);
    assert(g_dash_data.last_301_alive == 4U);
    assert(g_dash_data.alive_301_error == 0U);
    assert(g_dash_data.alive_301_error_count == 1U);

    puts("dashboard 0x301 alive counter test passed");
    return 0;
}
