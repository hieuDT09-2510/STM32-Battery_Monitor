/*
 * Mô-đun: Dịch vụ chuyển đổi cảm biến chung và snapshot mới nhất.
 * PA0: Giá trị thô → Điện áp chân → Khôi phục điện áp pin theo tỉ lệ 1.5 lần.
 * PA1: Phân loại đầu vào trước, sau đó chuyển đổi nhiệt độ theo pull-up 10k, NTC 10k/B3950.
 * Sensor Service độc quyền sử dụng ADC_ReadSafe, đính kèm số thứ tự và timestamp cho mỗi nhóm dữ liệu mới.
 * Các mô-đun khác đọc snapshot, không còn tiêu thụ trực tiếp cờ DMA ready.
 * Việc phát hành và sao chép snapshot mới nhất được bảo vệ bằng critical section ngắn FreeRTOS,
 * ngăn SensorTask xảy ra xung đột đồng thời với các task BMS, CAN, CLI dẫn đến đọc dữ liệu không hoàn chỉnh.
 * Mô-đun này chỉ cung cấp sự thật đo lường, không thực hiện xác nhận lỗi, chốt lỗi, chuyển trạng thái hay hành động bảo vệ.
 */

// Include kiểu dữ liệu công cộng và khai báo hàm.
#include "app_sensor.h"
/*
 * FreeRTOS.h cung cấp cấu hình kernel và kiểu dữ liệu chung;
 * task.h cung cấp API critical section cho task.
 */
#include "FreeRTOS.h"
#include "task.h"
// Include API đọc an toàn ADC/DMA, chỉ cho phép dịch vụ này sử dụng
#include "bsp_adc_dma.h"

// Include hàm logarit tự nhiên dùng trong công thức NTC.
#include <math.h>

// Tiếp tục dùng điện áp tham chiếu hiện tại, không bù cho IC ổn áp bị hỏng.
#define SENSOR_ADC_VREF_V       3.27f

// Phân áp pin: trở trên 10k, trở dưới 20k, tỉ lệ khôi phục là 1.5.
#define SENSOR_CELL_GAIN        1.5f

/* Cửa sổ trung bình trượt điện áp chỉ dùng cho hiển thị CLI, CAN và OLED. */
#define SENSOR_DISPLAY_VOLTAGE_WINDOW 16U

// Tiếp tục dùng ngưỡng test cận dưới NTC hiện tại.
#define SENSOR_NTC_ADC_LOW      16U

// Tiếp tục dùng ngưỡng test cận trên NTC hiện tại.
#define SENSOR_NTC_ADC_HIGH     4079U

// Lưu snapshot hoàn chỉnh gần nhất; chỉ do Sensor_ServiceUpdate ghi.
static SensorSample_t g_latest_sample;

/* Snapshot hiển thị gần nhất; điện áp là trung bình trượt, các trường khác đến từ snapshot thô mới nhất. */
static SensorSample_t g_latest_display_sample;

/*
 * Cửa sổ điện áp dùng mili-volt số nguyên, tránh sai số tích lũy dấu phẩy động lâu dài.
 * Trạng thái này chỉ do SensorTask ghi, không cần khóa riêng.
 */
static uint16_t g_voltage_mv_window[SENSOR_DISPLAY_VOLTAGE_WINDOW];
static uint32_t g_voltage_mv_sum = 0U;
static uint8_t g_voltage_window_count = 0U;
static uint8_t g_voltage_window_index = 0U;

/* Điện áp thực sự bàn giao cho phía hiển thị sau khi đã qua lượng tử hóa 10mV và phán đoán vùng chết. */
static uint16_t g_display_voltage_mv = 0U;
static uint8_t g_display_voltage_initialized = 0U;

// Lưu số thứ tự snapshot được gán gần nhất; 0 dành riêng cho "chưa có snapshot".
static uint32_t g_sample_sequence = 0U;

// Đánh dấu dịch vụ đã nhận thành công ít nhất một nhóm dữ liệu hay chưa.
static uint8_t g_has_sample = 0U;

/*
 * Mô-đun: Chuyển đổi một kết quả lấy mẫu.
 * Điện áp và nhiệt độ xử lý độc lập, một kênh không hợp lệ không ngăn cản kênh kia cập nhật.
 * Giá trị không hợp lệ điền 0, chỉ dùng khởi tạo trường, không thể coi là giá trị đo thực tế.
 */
void Sensor_Convert(uint16_t adc0, uint16_t adc1, SensorSample_t *sample)
{
    // Lưu điện trở NTC, đơn vị Ohm.
    float resistance;

    // Lưu nghịch đảo của nhiệt độ tuyệt đối.
    float inverse_kelvin;

    // Trả về ngay khi không có đối tượng đầu ra.
    if (sample == 0)
    {
        return;
    }

     // Kết quả chuyển đổi thuần túy không có ngữ cảnh thu thập, metadata do Sensor Service điền sau khi đọc thành công.
    sample->sequence = 0U;
    sample->timestamp_ms = 0U;


    // Lưu giá trị đọc thô, thuận tiện cho chẩn đoán tiếp theo.
    sample->adc_pa0 = adc0;
    sample->adc_pa1 = adc1;

    // Mỗi lần đều tái khởi tạo kết quả, tránh dùng lại cờ hợp lệ của khung trước.
    sample->pa0_voltage_v = 0.0f;
    sample->cell_voltage_v = 0.0f;
    sample->temperature_c = 0.0f;
    sample->voltage_valid = 0U;
    sample->temperature_valid = 0U;
    sample->ntc_input = NTC_INPUT_UNKNOWN;

    // Miễn là giá trị thô điện áp nằm trong phạm vi 12-bit thì tiến hành chuyển đổi số trị.
    if (adc0 <= 4095U)
    {
        // Tính điện áp chân PA0.
        sample->pa0_voltage_v = (float)adc0 * SENSOR_ADC_VREF_V / 4095.0f;

        // Khôi phục điện áp pin, không coi 0V hoặc kịch thang đo là trạng thái an toàn.
        sample->cell_voltage_v = sample->pa0_voltage_v * SENSOR_CELL_GAIN;

        // Đánh dấu chuyển đổi điện áp thành công, quá áp và thấp áp do mô-đun lỗi phán đoán.
        sample->voltage_valid = 1U;
    }

    // Loại trừ vượt ngưỡng trước, tránh phân loại nhầm dữ liệu bị hỏng thành hở mạch.
    if (adc1 > 4095U)
    {
        sample->ntc_input = NTC_INPUT_ADC_INVALID;
    }
    // Báo cáo nghi ngờ ngắn mạch khi gần điện thế đất (GND), tạm thời chưa xác nhận lỗi.
    else if (adc1 <= SENSOR_NTC_ADC_LOW)
    {
        sample->ntc_input = NTC_INPUT_SHORT_SUSPECT;
    }
    // Báo cáo nghi ngờ hở mạch khi gần kịch thang đo, tạm thời chưa xác nhận lỗi.
    else if (adc1 >= SENSOR_NTC_ADC_HIGH)
    {
        sample->ntc_input = NTC_INPUT_OPEN_SUSPECT;
    }
    // Chỉ tính toán nhiệt độ cho đầu vào được phép chuyển đổi.
    else
    {
        // Tính điện trở theo quan hệ phân áp pull-up 10k, NTC nối đất.
        resistance = 10000.0f * (float)adc1 / (4095.0f - (float)adc1);

        // Dùng R25=10k, B=3950 tính nghịch đảo nhiệt độ tuyệt đối.
        inverse_kelvin = 1.0f / 298.15f
                      + logf(resistance / 10000.0f) / 3950.0f;

        // Chuyển sang độ C, giữ lại nhiệt độ âm.
        sample->temperature_c = 1.0f / inverse_kelvin - 273.15f;

        // Đánh dấu cho phép dùng kết quả nhiệt độ, việc có quá nhiệt hay không do bên khác phán đoán.
        sample->temperature_valid = 1U;
        sample->ntc_input = NTC_INPUT_NORMAL;
    }
}

/* Khởi tạo dịch vụ snapshot cảm biến. */
void Sensor_ServiceInit(void)
{
    SensorSample_t empty_sample = {0};
    uint8_t index;

    g_latest_sample = empty_sample;
    g_latest_display_sample = empty_sample;
    g_sample_sequence = 0U;
    g_has_sample = 0U;
    g_voltage_mv_sum = 0U;
    g_voltage_window_count = 0U;
    g_voltage_window_index = 0U;
    g_display_voltage_mv = 0U;
    g_display_voltage_initialized = 0U;

    for (index = 0U; index < SENSOR_DISPLAY_VOLTAGE_WINDOW; index++)
    {
        g_voltage_mv_window[index] = 0U;
    }
}

/* Thử nhận và phát hành một nhóm snapshot 2 kênh mới. */
uint8_t Sensor_ServiceUpdate(uint32_t now_ms)
{
    uint16_t adc0;
    uint16_t adc1;
    uint16_t voltage_mv;
    uint32_t average_mv;
    uint32_t next_sequence;// Số thứ tự snapshot khung tiếp theo
    SensorSample_t new_sample;// Đối tượng cục bộ, tránh để lộ snapshot chưa hoàn chỉnh ra ngoài
    SensorSample_t display_sample; /* Snapshot điện áp trung bình cho CLI/CAN đọc. */

    // Khi không có nửa bộ đệm hoặc toàn bộ đệm DMA mới, không thay đổi snapshot hiện có.
    if (ADC_ReadSafe(&adc0, &adc1) == 0U)
    {
        return 0U;
    }

    // Hoàn thành chuyển đổi trong đối tượng cục bộ trước, tránh để lộ ra ngoài kết quả mới cập nhật một nửa.
    Sensor_Convert(adc0, adc1, &new_sample);

    // Tính số thứ tự tiếp theo; 0 vĩnh viễn dành riêng cho "chưa từng phát hành snapshot".
    next_sequence = g_sample_sequence + 1U;
    if (next_sequence == 0U)
    {
        next_sequence = 1U;
    }

    // Metadata chỉ đại diện cho khung mới thực sự lấy từ ADC/DMA lần này.
    new_sample.sequence = next_sequence;
    new_sample.timestamp_ms = now_ms;

    /*
     * Snapshot gốc giữ nguyên không đổi; snapshot hiển thị chỉ làm mượt điện áp hợp lệ.
     * Khi cửa sổ chưa đầy thì tính trung bình theo số mẫu hiện có, do đó khung đầu tiên có thể hiển thị ngay.
     */
    display_sample = new_sample;
    if (new_sample.voltage_valid != 0U)
    {
        voltage_mv = (uint16_t)(new_sample.cell_voltage_v * 1000.0f + 0.5f);

        if (g_voltage_window_count < SENSOR_DISPLAY_VOLTAGE_WINDOW)
        {
            g_voltage_window_count++;
        }
        else
        {
            g_voltage_mv_sum -= g_voltage_mv_window[g_voltage_window_index];
        }

        g_voltage_mv_window[g_voltage_window_index] = voltage_mv;
        g_voltage_mv_sum += voltage_mv;
        g_voltage_window_index++;
        if (g_voltage_window_index >= SENSOR_DISPLAY_VOLTAGE_WINDOW)
        {
            g_voltage_window_index = 0U;
        }

        average_mv = (g_voltage_mv_sum +
                      (uint32_t)(g_voltage_window_count / 2U)) /
                     (uint32_t)g_voltage_window_count;

        /*
         * Khung đầu tiên làm tròn ngay đến nấc 10mV. Về sau chỉ khi giá trị trung bình chênh lệch ít nhất 10mV
         * so với giá trị hiển thị hiện tại mới đổi nấc, tránh lật qua lật lại giữa các số cuối liền kề.
         */
        if (g_display_voltage_initialized == 0U)
        {
            g_display_voltage_mv =
                (uint16_t)(((average_mv + 5U) / 10U) * 10U);
            g_display_voltage_initialized = 1U;
        }
        else if (((average_mv >= g_display_voltage_mv) &&
                  ((average_mv - g_display_voltage_mv) >= 10U)) ||
                 ((average_mv < g_display_voltage_mv) &&
                  ((uint32_t)g_display_voltage_mv - average_mv >= 10U)))
        {
            g_display_voltage_mv =
                (uint16_t)(((average_mv + 5U) / 10U) * 10U);
        }

        display_sample.cell_voltage_v =
            (float)g_display_voltage_mv / 1000.0f;
        display_sample.pa0_voltage_v =
            display_sample.cell_voltage_v / SENSOR_CELL_GAIN;
    }

    // Phát hành một lần sau khi cấu tạo hoàn chỉnh, không lo xung đột sao chép đồng thời.
    /*
    * Phép chuyển đổi dấu phẩy động của new_sample đã hoàn thành bên ngoài critical section.
    *
    * Trong critical section chỉ sao chép kết quả cuối cùng và cập nhật trạng thái phát hành,
    * ngăn chặn việc các task khác đang sao chép dở thì bị SensorTask ghi đè.
    */    
    taskENTER_CRITICAL();    //

    g_latest_sample = new_sample;
    g_latest_display_sample = display_sample;
    g_sample_sequence = next_sequence;
    g_has_sample = 1U;

    taskEXIT_CRITICAL();    

    return 1U;
}

/*
 * Sao chép snapshot hoàn chỉnh gần nhất.
 *
 * Trả về một bản sao độc lập. Sau khi caller thoát khỏi hàm này,
 * SensorTask tiếp tục cập nhật snapshot nội bộ cũng không làm đổi dữ liệu caller đã nhận.
 */
uint8_t Sensor_GetLatest(SensorSample_t *sample)
{
    uint8_t sample_available;
    // Con trỏ NULL không đi vào critical section, cũng không làm mục tiêu sao chép

    if (sample == 0)
    {
        return 0U;
    }

    taskENTER_CRITICAL();    // Đi vào critical section, bảo vệ việc truy cập snapshot toàn cục, ngăn bị SensorTask sửa đổi đồng thời

    sample_available = g_has_sample;
    if(sample_available != 0U)
    {
        *sample = g_latest_sample;
    }
    taskEXIT_CRITICAL();    

    return sample_available;
}

/* Sao chép snapshot điện áp trung bình chỉ dùng cho hiển thị và truyền thông. */
uint8_t Sensor_GetLatestDisplay(SensorSample_t *sample)
{
    uint8_t sample_available;

    if (sample == 0)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    sample_available = g_has_sample;
    if (sample_available != 0U)
    {
        *sample = g_latest_display_sample;
    }
    taskEXIT_CRITICAL();

    return sample_available;
}

uint32_t Sensor_CalculateAgeMs(const SensorSample_t *sample,
                               uint32_t now_ms)
{
    int32_t signed_age_ms;

    if ((sample == 0) || (sample->sequence == 0U))
    {
        return 0U;
    }

    /*
     * Giải thích số có dấu có thể nhận diện vài mili-giây "timestamp tương lai" do switch task gây ra;
     * vòng lặp quay vòng bình thường của Tick 32-bit vẫn thu được tuổi là số dương nhỏ chính xác.
     */
    signed_age_ms = (int32_t)(now_ms - sample->timestamp_ms);
    if (signed_age_ms < 0)
    {
        return 0U;
    }

    return (uint32_t)signed_age_ms;
}

/* Tính toán xem snapshot có còn trong độ tuổi cho phép dựa trên timestamp. */
uint8_t Sensor_IsFresh(const SensorSample_t *sample,
                       uint32_t now_ms,
                       uint32_t max_age_ms)
{
    if ((sample == 0) || (sample->sequence == 0U))
    {
        return 0U;
    }

    if ((uint32_t)(now_ms - sample->timestamp_ms) > max_age_ms)
    {
        return 0U;
    }

    return 1U;
}
