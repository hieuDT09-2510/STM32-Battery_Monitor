/*
 * Tên tệp: can_protocol.h
 * Chức năng mô-đun: Định nghĩa định dạng bản tin CAN và các phương thức đóng gói/giải mã chung giữa ECU-A và ECU-B.
 *
 * Bố cục bản tin trạng thái đo lường 0x301:
 * data[0]~data[1] = Điện áp pin, đơn vị mV, byte cao đứng trước;
 * data[2] = Phần trăm số nguyên SOC (ước lượng tuyến tính từ điện áp pin), 0xFF biểu thị chưa có kết quả hợp lệ;
 * data[3] = Trạng thái giám sát;
 * 4 bit thấp của data[4] = Alive Counter (vòng lặp 0~15); 4 bit cao dự phòng;
 * data[5]~data[6] = Dự phòng, cố định xóa về 0 khi gửi;
 * data[7] = Giá trị kiểm tra CRC8 của Byte0~Byte6.
 *
 * Bố cục bản tin trạng thái nhiệt độ 0x302:
 * data[0] = Nhiệt độ;
 * data[1]~data[3] = Dự phòng, cố định xóa về 0 khi gửi;
 * 4 bit thấp của data[4] = Alive Counter (vòng lặp 0~15);
 * data[5]~data[6] = Dự phòng, cố định xóa về 0 khi gửi;
 * data[7] = Giá trị kiểm tra CRC8 của Byte0~Byte6.
 * 
 * 
 * Bố cục bản tin trạng thái lỗi 0x303:
 * data[0] = 8 bit cao của mặt nạ lỗi đang kích hoạt 16-bit;
 * data[1] = 8 bit thấp của mặt nạ lỗi đang kích hoạt 16-bit;
 * data[2]~data[3] = Dự phòng, cố định xóa về 0 khi gửi;
 * 4 bit thấp của data[4] = Alive Counter (vòng lặp 0~15);
 * data[5]~data[6] = Dự phòng, cố định xóa về 0 khi gửi;
 * data[7] = Giá trị kiểm tra CRC8 của Byte0~Byte6.
 */
#ifndef __CAN_PROTOCOL_H
#define __CAN_PROTOCOL_H

#include "stm32f10x.h"

#define CAN_ID_BMS_VOLTAGE    0x301
#define CAN_ID_BMS_TEMP       0x302
#define CAN_ID_BMS_FAULT      0x303
#define CAN_ID_CTRL_CMD       0x100

// Định nghĩa mặt nạ lỗi đang kích hoạt 0x303, ECU-B dùng các bit này để giải thích tên lỗi.
#define CAN_FAULT_OV            ((uint16_t)0x0001U)
#define CAN_FAULT_UV            ((uint16_t)0x0002U)
#define CAN_FAULT_OT            ((uint16_t)0x0004U)
#define CAN_FAULT_NTC_OPEN      ((uint16_t)0x0008U)
#define CAN_FAULT_NTC_SHORT     ((uint16_t)0x0010U)
#define CAN_FAULT_SENSOR_DATA   ((uint16_t)0x0020U)
#define CAN_FAULT_ADC_TIMEOUT   ((uint16_t)0x0040U)
#define CAN_FAULT_SELF_CHECK    ((uint16_t)0x0080U)

typedef struct
{
    uint16_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CAN_Frame_t;

#define CHG_STATE_IDLE       0
#define CHG_STATE_DISCHARGE  1
#define CHG_STATE_CHARGE     2

/* ECU-A đang ở trạng thái giám sát, không biểu thị đã có khả năng thực thi sạc hoặc xả. */
#define CAN_STATUS_MONITOR   3U

#define CAN_301_ALIVE_MASK   0x0FU

// Alive Counter của bản tin nhiệt độ cũng dùng 4 bit thấp, phạm vi giá trị từ 0~15.
#define CAN_302_ALIVE_MASK 0x0FU

// Bản tin lỗi có Alive Counter 4-bit độc lập, phạm vi giá trị từ 0~15.
#define CAN_303_ALIVE_MASK   0x0FU

/*
 * BMS_CalculateCRC8 — Tính CRC8 tầng ứng dụng.
 *
 * Tham số data:
 *   Trỏ tới địa chỉ đầu của dữ liệu cần tính CRC.
 *
 * Tham số length:
 *   Số byte dữ liệu tham gia tính toán.
 *
 * Giá trị trả về:
 *   CRC 8-bit đã tính xong.
 *
 * Hiện sử dụng tham số CRC-8/SAE-J1850:
 *   Polynomial = 0x1D
 *   Init       = 0xFF
 *   XorOut     = 0xFF
 */
static inline uint8_t BMS_CalculateCRC8(
    const uint8_t *data,
    uint8_t length)
{
    uint8_t crc = 0xFFU;
    uint8_t byte_index;
    uint8_t bit_index;

    for(byte_index = 0U;
        byte_index < length;
        byte_index++)
    {
        /*
         * Trước tiên XOR byte dữ liệu hiện tại với CRC,
         * để từng bit của byte này tham gia vào phép tính tiếp theo.
         */
        crc ^= data[byte_index];

        /*
         * Một byte có 8 bit, vì vậy mỗi byte dữ liệu xử lý 8 lần.
         */
        for(bit_index = 0U;
            bit_index < 8U;
            bit_index++)
        {
            /*
             * Nếu bit cao nhất của CRC bằng 1:
             * Dịch trái rồi XOR với đa thức 0x1D.
             */
            if((crc & 0x80U) != 0U)
            {
                crc = (uint8_t)(
                    (crc << 1U) ^ 0x1DU);
            }
            else
            {
                /*
                 * Khi bit cao nhất bằng 0 thì chỉ cần dịch trái.
                 */
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    /*
     * SAE-J1850 cuối cùng còn phải XOR với 0xFF.
     */
    return (uint8_t)(crc ^ 0xFFU);
}

// Đóng gói bản tin trạng thái điện áp 0x301
static inline void BMS_PackVoltage(CAN_Frame_t *f,
                                   uint16_t mv,
                                   uint8_t soc,
                                   uint8_t chg_state,
                                   uint8_t alive_counter)
{
    f->id = CAN_ID_BMS_VOLTAGE;
    f->dlc = 8;

    f->data[0] = (mv >> 8) & 0xFF; 
    f->data[1] = mv & 0xFF;

    f->data[2] = soc;
    f->data[3] = chg_state;

    f->data[4] = alive_counter & CAN_301_ALIVE_MASK;
    
   /*
    * Byte5 và Byte6 tạm thời chưa có dữ liệu nghiệp vụ,
    * trước khi tham gia tính CRC phải cố định xóa về 0.
    */
    f->data[5] = 0U;
    f->data[6] = 0U;

   /*
    * CRC bao phủ từ Byte0~Byte6, tổng cộng 7 byte.
    * Kết quả tính toán đặt vào byte cuối cùng Byte7.
    */
    f->data[7] =
        BMS_CalculateCRC8(f->data, 7U);
}

static inline uint8_t BMS_UnpackVoltageSoc(const CAN_Frame_t *f)
{
    return f->data[2];
}

// Đóng gói bản tin trạng thái nhiệt độ 0x302
/*
 * BMS_PackTemp — Lắp ghép bản tin trạng thái nhiệt độ.
 *
 * Byte0: Nhiệt độ;
 * Byte1~3: Dự phòng;
 * 4 bit thấp của Byte4: Alive Counter bản tin nhiệt độ;
 * Byte5~6: Dự phòng;
 * Byte7: CRC8 của Byte0~Byte6.
 */
static inline void BMS_PackTemp(
    CAN_Frame_t *f,
    uint8_t temp,
    uint8_t alive_counter)
{
    f->id = CAN_ID_BMS_TEMP;
    f->dlc = 8U;

    f->data[0] = temp;

    /*
     * Hiện không có dữ liệu nghiệp vụ nhiệt độ khác,
     * nên Byte1~Byte3 cố định xóa về 0.
     */
    f->data[1] = 0U;
    f->data[2] = 0U;
    f->data[3] = 0U;

    /*
     * Byte4 chỉ giữ lại 4 bit thấp của bộ đếm.
     */
    f->data[4] =
        (uint8_t)(alive_counter &
                  CAN_302_ALIVE_MASK);

    f->data[5] = 0U;
    f->data[6] = 0U;

    /*
     * Sau khi điền xong toàn bộ 7 byte đầu tiên,
     * mới có thể tính CRC cuối cùng.
     */
    f->data[7] =
        BMS_CalculateCRC8(f->data, 7U);
}



// Chỉ lấy 4 bit thấp theo quy định giao thức, tránh ảnh hưởng đến Alive Counter khi dùng 4 bit cao trong tương lai.
static inline uint8_t BMS_Unpack301Alive(const CAN_Frame_t *f)
{
    return (uint8_t)(f->data[4] & CAN_301_ALIVE_MASK);
}

/*
 * Trích xuất Alive Counter riêng của bản tin trạng thái nhiệt độ
 * từ 4 bit thấp của Byte4.
 */
static inline uint8_t BMS_Unpack302Alive(
    const CAN_Frame_t *f)
{
    return (uint8_t)(
        f->data[4] &
        CAN_302_ALIVE_MASK);
}

/*
 * Trích xuất Alive Counter riêng của bản tin trạng thái lỗi
 * từ 4 bit thấp của Byte4.
 */
static inline uint8_t BMS_Unpack303Alive(
    const CAN_Frame_t *f)
{
    return (uint8_t)(
        f->data[4] &
        CAN_303_ALIVE_MASK);
}


/*
 * BMS_PackFault — Lắp ghép bản tin trạng thái lỗi.
 *
 * fault_mask lưu tất cả lỗi đang kích hoạt hiện tại;
 * alive_counter biểu thị thứ tự gửi của bản tin này;
 * Byte7 dùng để kiểm tra 7 byte đầu có bị thay đổi ngoài ý muốn hay không.
 */
static inline void BMS_PackFault(
    CAN_Frame_t *f,
    uint16_t fault_mask,
    uint8_t alive_counter)
{
    f->id = CAN_ID_BMS_FAULT;
    f->dlc = 8U;

    // Mặt nạ lỗi 16-bit dùng byte cao đứng trước.
    f->data[0] =
        (uint8_t)((fault_mask >> 8) & 0x00FFU);

    f->data[1] =
        (uint8_t)(fault_mask & 0x00FFU);

    // Hiện không có dữ liệu nghiệp vụ khác, các byte dự phòng phải cố định xóa về 0.
    f->data[2] = 0U;
    f->data[3] = 0U;

    // Byte4 chỉ dùng 4 bit thấp để lưu số thứ tự riêng của bản tin lỗi.
    f->data[4] =
        (uint8_t)(alive_counter &
                  CAN_303_ALIVE_MASK);

    f->data[5] = 0U;
    f->data[6] = 0U;

    // Phải điền Byte0~Byte6 trước, cuối cùng mới tính CRC.
    f->data[7] =
        BMS_CalculateCRC8(f->data, 7U);
}


/* Khôi phục hai byte trong 0x303 theo đúng thứ tự đó thành mặt nạ lỗi 16-bit. */
static inline uint16_t BMS_UnpackFault(const CAN_Frame_t *f)
{
    return (uint16_t)(((uint16_t)f->data[0] << 8) |
                      (uint16_t)f->data[1]);
}

#endif
