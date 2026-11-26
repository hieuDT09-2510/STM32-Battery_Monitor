/* Test phia host: 3482mV bat buoc phai hien thi thanh 3.482V. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_dash.h"

static char g_voltage_line[32];

uint32_t GetTick(void)
{
    return 0U;
}

void OLED_Clear(void) {}
void OLED_Refresh(void) {}

void OLED_ShowString(uint8_t x, uint8_t y, const char *text)
{
    (void)x;

    if (y == 2U)
    {
        (void)snprintf(g_voltage_line,
                       sizeof(g_voltage_line),
                       "%s",
                       text);
    }
}

int main(void)
{
    Dash_Init();
    g_dash_data.valid = 1U;
    g_dash_data.cell_mv = 3482U;

    Dash_RefreshDisplay();

    assert(strcmp(g_voltage_line, "Cell:3.482V") == 0);
    puts("dashboard voltage format test passed");
    return 0;
}
