/*
 * Test phia host: Kiem tra toan bo du lieu man hinh OLED duoc gui trong mot transaction I2C lien tuc.
 * Lenh van co the dung transaction WriteBuffer ngan; 1024 byte bo nho dem hien thi khong duoc lap lai START/STOP tung byte.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "bsp_oled.h"

static uint32_t g_write_buffer_calls;
static uint32_t g_start_calls;
static uint32_t g_stop_calls;
static uint32_t g_send_addr_calls;
static uint32_t g_send_data_calls;
static uint32_t g_wait_ack_calls;

void I2C1_WriteBuffer(uint8_t dev_addr, uint8_t *buffer, uint16_t len)
{
    (void)dev_addr;
    (void)buffer;
    (void)len;
    g_write_buffer_calls++;
}

void I2C1_Start(void) { g_start_calls++; }
void I2C1_Stop(void) { g_stop_calls++; }

void I2C1_SendAddr(uint8_t addr_7bit, uint8_t rw)
{
    (void)addr_7bit;
    (void)rw;
    g_send_addr_calls++;
}

void I2C1_SendData(uint8_t data)
{
    (void)data;
    g_send_data_calls++;
}

uint8_t I2C1_WaitAck(void)
{
    g_wait_ack_calls++;
    return 0U;
}

int main(void)
{
    OLED_Refresh();

    /* Cai dat pham vi cot va trang tong cong 6 lenh, moi lenh dung mot transaction ngan. */
    assert(g_write_buffer_calls == 6U);

    /* Byte dieu khien 0x40 va 1024 byte bo nho dem hien thi dung chung mot transaction du lieu lien tuc. */
    assert(g_start_calls == 1U);
    assert(g_send_addr_calls == 1U);
    assert(g_send_data_calls == 1025U);
    assert(g_wait_ack_calls == 1025U);
    assert(g_stop_calls == 1U);

    puts("OLED continuous refresh transport test passed");
    return 0;
}
