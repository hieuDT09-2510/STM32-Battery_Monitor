/*
 * Tên tệp: test_can_fault_protocol.c
 *
 * Chức năng mô-đun:
 * Kiểm chứng kết quả đóng gói giao thức tầng ứng dụng CAN của ECU-A trên máy tính.
 *
 * Nội dung kiểm chứng:
 * 1. Thuật toán CRC-8/SAE-J1850 có chính xác hay không;
 * 2. Bố cục byte, Alive Counter và CRC của bản tin điện áp/trạng thái 0x301;
 * 3. Bố cục byte, Alive Counter và CRC của bản tin nhiệt độ 0x302;
 * 4. Bố cục byte, Alive Counter và CRC của bản tin lỗi 0x303;
 * 5. Mã hóa có dấu, Alive Counter và CRC của bản tin dòng điện/công suất 0x304.
 */

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "can_protocol.h"


int main(void)
{
    CAN_Frame_t frame = {0};

    /* Dữ liệu kiểm tra chuẩn CRC-8/SAE-J1850: "123456789" bắt buộc phải ra 0x4B. */
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

    /*
     * Kiểm chứng 0x301: 3482mV = 0x0D9A, SOH = 88, trạng thái = 3.
     * Khi đầu vào Alive là 0x1E, chỉ cho phép 4 bit thấp 0x0E đi vào bản tin.
     */
    BMS_PackVoltage(
        &frame,
        3482U,
        88U,
        CAN_STATUS_MONITOR,
        0x1EU);

    assert(frame.id == CAN_ID_BMS_VOLTAGE);
    assert(frame.dlc == 8U);
    assert(frame.data[0] == 0x0DU);
    assert(frame.data[1] == 0x9AU);
    assert(frame.data[2] == 0x58U);
    assert(frame.data[3] == CAN_STATUS_MONITOR);
    assert(frame.data[4] == 0x0EU);
    assert(frame.data[5] == 0x00U);
    assert(frame.data[6] == 0x00U);
    assert(frame.data[7] ==
           BMS_CalculateCRC8(frame.data, 7U));
    assert(BMS_UnpackVoltageSoh(&frame) == 88U);
    assert(BMS_Unpack301Alive(&frame) == 0x0EU);

    /*
     * Kiểm chứng 0x302: 25°C = 0x19.
     * Khi đầu vào Alive là 0x13, chỉ cho phép 4 bit thấp 0x03 đi vào bản tin.
     */
    BMS_PackTemp(
        &frame,
        25U,
        0x13U);

    assert(frame.id == CAN_ID_BMS_TEMP);
    assert(frame.dlc == 8U);
    assert(frame.data[0] == 0x19U);
    assert(frame.data[1] == 0x00U);
    assert(frame.data[2] == 0x00U);
    assert(frame.data[3] == 0x00U);
    assert(frame.data[4] == 0x03U);
    assert(frame.data[5] == 0x00U);
    assert(frame.data[6] == 0x00U);
    assert(frame.data[7] == 0xB0U);
    assert(BMS_Unpack302Alive(&frame) == 0x03U);

    /*
     * Kiểm chứng 0x303: mặt nạ lỗi 0x1234 được gửi theo thứ tự byte cao trước.
     * Khi đầu vào Alive là 0x15, chỉ cho phép 4 bit thấp 0x05 đi vào bản tin.
     */
    BMS_PackFault(
        &frame,
        0x1234U,
        0x15U);

    assert(frame.id == CAN_ID_BMS_FAULT);
    assert(frame.dlc == 8U);
    assert(frame.data[0] == 0x12U);
    assert(frame.data[1] == 0x34U);
    assert(frame.data[2] == 0x00U);
    assert(frame.data[3] == 0x00U);
    assert(frame.data[4] == 0x05U);
    assert(frame.data[5] == 0x00U);
    assert(frame.data[6] == 0x00U);
    assert(frame.data[7] == 0xB9U);
    assert(BMS_UnpackFault(&frame) == 0x1234U);
    assert(BMS_Unpack303Alive(&frame) == 0x05U);

    /*
     * Kiểm chứng 0x304: -125mA và -500mW đều sử dụng mã bù 2 16-bit, SOC = 65,
     * khi đầu vào Alive là 0x1B chỉ giữ lại 4 bit thấp 0x0B.
     */
    BMS_PackCurrent(
        &frame,
        -125,
        -500,
        65U,
        0x1BU);

    assert(frame.id == CAN_ID_BMS_CURRENT);
    assert(frame.dlc == 8U);
    assert(frame.data[0] == 0xFFU);
    assert(frame.data[1] == 0x83U);
    assert(frame.data[2] == 0xFEU);
    assert(frame.data[3] == 0x0CU);
    assert(frame.data[4] == 0x0BU);
    assert(frame.data[5] == 65U);
    assert(frame.data[6] == 0x00U);
    assert(frame.data[7] ==
           BMS_CalculateCRC8(frame.data, 7U));
    assert(BMS_UnpackCurrentMa(&frame) == -125);
    assert(BMS_UnpackPowerMw(&frame) == -500);
    assert(BMS_UnpackCurrentSoc(&frame) == 65U);
    assert(BMS_Unpack304Alive(&frame) == 0x0BU);

    /* Khi vượt quá phạm vi int16 bắt buộc phải bão hòa (saturate), không được cắt cụt làm đổi dấu. */
    BMS_PackCurrent(&frame, 50000L, -50000L, 0xFFU, 0U);
    assert(BMS_UnpackCurrentMa(&frame) == 32767);
    assert(BMS_UnpackPowerMw(&frame) == -32768);

    puts("ECU-A CAN protocol packing test passed");
    return 0;
}
