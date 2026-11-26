/*
 * Tên tệp: app_can_service.c
 *
 * Tên mô-đun: Dịch vụ gửi ứng dụng CAN
 *
 * Trách nhiệm mô-đun:
 * 1. Đọc bản chụp điện áp và nhiệt độ mới nhất do Sensor Service phát hành;
 * 2. Đọc trạng thái giám sát hiện tại và trạng thái lỗi đang kích hoạt của BMS;
 * 3. Chuyển dữ liệu nghiệp vụ cho CAN Protocol để đóng gói thành khung tin chuẩn;
 * 4. Quản lý Alive Counter riêng cho từng loại trong ba loại khung tin;
 * 5. Gọi CAN BSP để gửi khung tin đã đóng gói xuống phần cứng.
 *
 * Luồng dữ liệu hiện tại:
 * Bản chụp Sensor/SOC / Trạng thái BMS
 *          ↓
 * AppCAN_Send...()
 *          ↓
 * CAN Protocol đóng gói và tính CRC8
 *          ↓
 * CAN1_Send()
 *          ↓
 * Bộ điều khiển CAN phát đi
 *
 * Ràng buộc đồng thời hiện tại:
 * Chỉ CanTask được phép gọi giao diện gửi CAN của mô-đun này.
 *
 * Khi BmsTask phát hiện trạng thái lỗi thay đổi, chỉ gửi thông báo tác vụ tới CanTask,
 * không gọi trực tiếp trình điều khiển CAN, qua đó đảm bảo phần cứng CAN chỉ có một thực thể thao tác duy nhất.
 * 
 * Ranh giới mô-đun:
 * Mô-đun này không phán đoán quá áp, thấp áp hay quá nhiệt, không sửa đổi trạng thái BMS,
 * không ghi Flash, và cũng không phụ trách việc nhận và hiển thị của ECU-B.
 */
#include "app_can_service.h"

#include "app_sensor.h"
#include "app_soc.h"
#include "app_bms_state.h"

#include "bsp_can.h"
#include "bsp_tim.h"
#include "can_protocol.h"


/*
 * Alive Counter dùng cho khung tiếp theo của bản tin trạng thái điện áp.
 * Xoay vòng trong khoảng 0~15 sau mỗi lần đóng gói một khung.
 */
volatile uint8_t g_can_301_alive_counter = 0U;

/*
 * Bản tin trạng thái nhiệt độ sử dụng Alive Counter riêng,
 * không bị ảnh hưởng bởi chu kỳ gửi của bản tin điện áp.
 */
volatile uint8_t g_can_302_alive_counter = 0U;

/*
 * Bản tin trạng thái lỗi sử dụng Alive Counter riêng,
 * cả việc gửi định kỳ và gửi nhanh khi có lỗi sau này đều sẽ dùng chung bộ đếm này.
 */
volatile uint8_t g_can_303_alive_counter = 0U;


/*
 * Lấy một khung bản chụp Sensor đang tồn tại và có tuổi nhỏ hơn 100ms.
 *
 * Hàm này chỉ kiểm tra bản chụp có tồn tại và còn đủ mới hay không;
 * tính hợp lệ của các trường điện áp và nhiệt độ do từng hàm gửi tự kiểm tra riêng.
 */
static uint8_t AppCAN_GetFreshSensorSample(
    SensorSample_t *sample)
{
    if ((sample == 0) ||
        (Sensor_GetLatestDisplay(sample) == 0U))
    {
        return 0U;
    }

    // Tính toán theo dấu thời gian xem bản chụp còn nằm trong giới hạn tuổi cho phép hay không
    return Sensor_IsFresh(
        sample,
        GetTick(),
        100U);
}

/*
 * Gửi 0x301 bằng bản chụp hợp lệ mới nhất.
 * Dù đang ở trạng thái chờ, giám sát hay lỗi, mẫu điện áp vẫn tiếp tục được cập nhật qua CAN.
 * 4 bit thấp của Byte4 mang Alive Counter, tăng tuần hoàn từ 0~15 sau mỗi lần đóng gói một khung.
 */
void AppCAN_SendVoltageStatus(void)
{
    CAN_Frame_t tx;             /* Khung 0x301 gồm 8 byte sắp được chuyển cho CAN BSP để gửi. */
    SensorSample_t sample;      /* Bản sao bản chụp điện áp mới nhất do SensorTask phát hành. */
    SocSnapshot_t soc_snapshot; /* Dùng để lấy kết quả SOC gần nhất; khi không hợp lệ sẽ không chặn việc gửi điện áp. */
    uint16_t cell_mv;           /* Giá trị milivôn số nguyên mà giao thức CAN sử dụng. */
    uint8_t soc;                /* CAN sử dụng giá trị SOC số nguyên; 0xFF biểu thị chưa có giá trị hợp lệ. */
    uint8_t monitor_status;     /* Trạng thái giám sát/rảnh dùng để hiển thị trên ECU-B. */

    //AppCAN_GetFreshSensorSample--Lấy một khung bản chụp Sensor đang tồn tại và có tuổi nhỏ hơn 100ms
    if ((AppCAN_GetFreshSensorSample(&sample) == 0U) ||
        (sample.voltage_valid == 0U))
    {
        return;
    }

    /* Chuyển đổi V sang mV và làm tròn; dùng giá trị không hợp lệ theo giao thức là 0xFF khi SOC không hợp lệ. */
    cell_mv = (uint16_t)(sample.cell_voltage_v * 1000.0f + 0.5f);
    soc = 0xFFU;
    if (Soc_GetSnapshot(&soc_snapshot) != 0U)
    {
        soc = (uint8_t)(
            (soc_snapshot.soc_centi_percent + 50U) / 100U);
    }
    monitor_status = ((BMS_GetState() == BMS_MONITOR) ||
                      (BMS_GetState() == BMS_FAULT)) ?
                     CAN_STATUS_MONITOR : CHG_STATE_IDLE;

    BMS_PackVoltage(&tx,
                    cell_mv,
                    soc,
                    monitor_status,
                    g_can_301_alive_counter);

    // Chỉ giữ lại 4 bit thấp: 15 cộng 1 sẽ tự nhiên quay về 0 thông qua mặt nạ bit.
    g_can_301_alive_counter =
        (uint8_t)((g_can_301_alive_counter + 1U) & CAN_301_ALIVE_MASK);

    (void)CAN1_Send(&tx);
}

/*
 * AppCAN_SendTemperatureStatus — Gửi trạng thái nhiệt độ hợp lệ hiện tại.
 *
 * Không gửi giá trị giả khi bản chụp nhiệt độ không hợp lệ;
 * lỗi NTC vẫn được báo cáo thông qua bản tin trạng thái lỗi.
 */
void AppCAN_SendTemperatureStatus(void)
{
    CAN_Frame_t tx;
    SensorSample_t sample;
    uint8_t temperature;

    if ((AppCAN_GetFreshSensorSample(&sample) == 0U) ||
        (sample.temperature_valid == 0U))
    {
        return;
    }

    temperature =
    (uint8_t)(sample.temperature_c + 0.5f);

    /*
    * Đóng gói bản tin bằng bộ đếm riêng của bản tin nhiệt độ.
    * Bên trong BMS_PackTemp sẽ đồng thời tính toán CRC8.
    */
    BMS_PackTemp(
        &tx,
        temperature,
        g_can_302_alive_counter);

    /*
    * Tăng bộ đếm thêm 1 sau mỗi lần đóng gói một khung bản tin nhiệt độ.
    * 15 cộng 1 sẽ quay về 0 thông qua mặt nạ 4 bit thấp.
    */
    g_can_302_alive_counter =
        (uint8_t)(
            (g_can_302_alive_counter + 1U) &
            CAN_302_ALIVE_MASK);

    (void)CAN1_Send(&tx);
}


/*
 * AppCAN_SendFaultStatus — Gửi trạng thái lỗi đang kích hoạt hiện tại.
 *
 * Hiện tại chỉ được gọi bởi CanTask:
 * 1. Trong trường hợp bình thường, gửi định kỳ mỗi 100ms một lần;
 * 2. Khi mặt nạ lỗi thay đổi, BmsTask thông báo cho CanTask gửi ngay lập tức.
 *
 * Việc thống nhất do CanTask gửi giúp tránh nhiều tác vụ cùng lúc truy cập trình điều khiển CAN,
 * đồng thời tránh việc Alive Counter của bản tin lỗi bị sửa đổi đồng thời.
 */
void AppCAN_SendFaultStatus(void)
{
    CAN_Frame_t tx;

    /*
     * Đóng gói mặt nạ lỗi đang kích hoạt, Alive Counter và CRC8
     * thành một bản tin lỗi hoàn chỉnh.
     */
    BMS_PackFault(
        &tx,
        g_fault_code,
        g_can_303_alive_counter);

    /*
     * Tăng số thứ tự thêm 1 sau mỗi lần đóng gói một khung.
     * 15 cộng 1 sẽ quay về 0 thông qua mặt nạ 4 bit thấp.
     */
    g_can_303_alive_counter =
        (uint8_t)(
            (g_can_303_alive_counter + 1U) &
            CAN_303_ALIVE_MASK);

    (void)CAN1_Send(&tx);
}
