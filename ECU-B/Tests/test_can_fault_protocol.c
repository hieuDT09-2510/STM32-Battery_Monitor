/*
 * Ten file: test_can_fault_protocol.c
 * Tac dung mo-dun: Kiem tra ECU-B co the phan tich Alive va mat na loi cua 3 loai ban tin theo giao thuc hien tai.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "can_protocol.h"


int main(void)
{
    CAN_Frame_t frame = {0};

    /* 4 bit cao thuoc ve bit danh rieng, ca 3 ham phan tich deu chi duoc tra ve 4 bit thap. */
    frame.data[4] = 0xAEU;
    assert(BMS_Unpack301Alive(&frame) == 0x0EU);
    assert(BMS_Unpack302Alive(&frame) == 0x0EU);
    assert(BMS_Unpack303Alive(&frame) == 0x0EU);

    /* Mat na loi duoc khoi phuc theo thu tu byte cao truoc. */
    frame.data[0] = 0x12U;
    frame.data[1] = 0x34U;
    assert(BMS_UnpackFault(&frame) == 0x1234U);

    /* Gia tri kiem tra tieu chuan cua CRC-8/SAE-J1850. */
    {
        static const uint8_t crc_test_data[] =
        {
            '1', '2', '3', '4', '5',
            '6', '7', '8', '9'
        };

        assert(
            BMS_CalculateCRC8(
                crc_test_data,
                (uint8_t)sizeof(crc_test_data)) ==
            0x4BU);
    }

    puts("ECU-B CAN protocol unpacking test passed");
    return 0;
}
