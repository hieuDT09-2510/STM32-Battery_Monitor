/*
 * Tên tệp: app_current.c
 *
 * Luồng dữ liệu: Đo cục bộ CurrentTask → new_sample cục bộ → Phát hành toàn bộ trong vùng găng ngắn
 *                                                                         ↓
 *                                                CAN, SOC và CLI sao chép bản chụp mới nhất
 *
 * Việc phát hành bản chụp chỉ sao chép struct đã quy đổi hoàn chỉnh bên trong vùng găng ngắn.
 * Không được phép đưa thao tác truy cập I2C, phép tính nhân chia và xuất cổng nối tiếp vào vùng găng.
 * Như vậy vùng găng chỉ bảo vệ "tính nhất quán dữ liệu tổng hợp", không bao bọc thao tác ngoại vi chậm bằng việc tắt ngắt.
 */
#include "app_current.h"

#include "FreeRTOS.h"
#include "task.h"

/* Cửa sổ đo dòng điện chỉ dùng cho hiển thị CLI, CAN và OLED. */
#define CURRENT_DISPLAY_WINDOW 8U

/* Bản chụp phát hành thành công trọn vẹn gần nhất; đọc thất bại sẽ không sửa đổi nó. */
static CurrentSample_t g_latest_current_sample;

/* Bản chụp hiển thị của dòng điện, điện áp bus và điện áp shunt đã qua trung bình trượt. */
static CurrentSample_t g_latest_current_display_sample;

/* CurrentTask là tác nhân ghi duy nhất đối với các trạng thái cửa sổ này. */
static int32_t g_current_window[CURRENT_DISPLAY_WINDOW];
static uint32_t g_bus_voltage_window[CURRENT_DISPLAY_WINDOW];
static int32_t g_shunt_voltage_window[CURRENT_DISPLAY_WINDOW];
static int64_t g_current_sum = 0LL;
static uint64_t g_bus_voltage_sum = 0ULL;
static int64_t g_shunt_voltage_sum = 0LL;
static uint8_t g_current_window_count = 0U;
static uint8_t g_current_window_index = 0U;

/* Giá trị giữ hiển thị đã qua lượng tử hóa 10mA/10mV và kiểm tra vùng chết. */
static int32_t g_display_current_uA = 0L;
static uint32_t g_display_bus_voltage_uV = 0UL;
static uint8_t g_current_display_initialized = 0U;

/* Làm tròn giá trị microampe có dấu về mức 10mA gần nhất. */
static int32_t Current_QuantizeTenMilliamp(int32_t value_uA)
{
    int64_t magnitude;

    if (value_uA >= 0L)
    {
        return (int32_t)((((int64_t)value_uA + 5000LL) / 10000LL) *
                         10000LL);
    }

    magnitude = -(int64_t)value_uA;
    return (int32_t)(-(((magnitude + 5000LL) / 10000LL) * 10000LL));
}

/* Trả về hiệu tuyệt đối giữa hai giá trị dòng điện có dấu, dùng 64-bit để tránh tràn số khi trừ. */
static uint64_t Current_AbsoluteDifference(int32_t left,
                                           int32_t right)
{
    int64_t difference = (int64_t)left - (int64_t)right;

    return (difference < 0LL) ?
        (uint64_t)(-difference) : (uint64_t)difference;
}

/* Trạng thái chẩn đoán tích lũy của Current Service, chỉ đọc ghi thông qua giao diện của mô-đun này. */
static CurrentDiagnostics_t g_current_diagnostics;

/* Mốc số thứ tự làm căn cứ cho lần phát hành thành công tiếp theo; giá trị 0 được dành riêng để biểu thị "chưa từng phát hành". */
static uint32_t g_current_sequence = 0U;

/* 0=chưa có bản chụp hợp lệ; 1=g_latest_current_sample có thể sao chép cho bên tiêu thụ. */
static uint8_t g_current_sample_available = 0U;

/* Tăng bão hòa bộ đếm chẩn đoán, tránh quay vòng từ 0xFFFFFFFF về 0 sau thời gian dài vận hành. */
static void Current_IncrementSaturated(uint32_t *value)
{
    if (*value < 0xFFFFFFFFUL)
    {
        (*value)++;
    }
}

void Current_ServiceInit(void)
{
    /* Đối tượng cục bộ toàn 0 đảm bảo các trường mới thêm vào struct trong tương lai cũng được khởi tạo. */
    CurrentSample_t empty_sample = {0};
    CurrentDiagnostics_t empty_diagnostics = {0};
    uint8_t index;

    /* Hàm này được gọi trước khi bộ lập lịch khởi chạy nên không cần vùng găng. */
    g_latest_current_sample = empty_sample;
    g_latest_current_display_sample = empty_sample;
    g_current_diagnostics = empty_diagnostics;
    g_current_sequence = 0U;
    g_current_sample_available = 0U;
    g_current_sum = 0LL;
    g_bus_voltage_sum = 0ULL;
    g_shunt_voltage_sum = 0LL;
    g_current_window_count = 0U;
    g_current_window_index = 0U;
    g_display_current_uA = 0L;
    g_display_bus_voltage_uV = 0UL;
    g_current_display_initialized = 0U;

    for (index = 0U; index < CURRENT_DISPLAY_WINDOW; index++)
    {
        g_current_window[index] = 0L;
        g_bus_voltage_window[index] = 0UL;
        g_shunt_voltage_window[index] = 0L;
    }
}

void Current_ServicePublish(int32_t current_uA, //Dòng điện
                            uint32_t bus_voltage_uV, //Điện áp bus phía IN− (dùng để tính công suất)
                            int32_t shunt_voltage_nV, //Độ sụt áp hai đầu điện trở - điện áp shunt (tính dòng điện)
                            int32_t power_uW, //Công suất
                            uint32_t timestamp_ms)
{
    CurrentSample_t new_sample; /* Bản chụp ứng viên hoàn chỉnh được lắp ghép ngoài vùng găng. */
    CurrentSample_t display_sample; /* Bản chụp hiển thị đã tính toán xong trong chu kỳ này. */
    uint32_t next_sequence;     /* Số thứ tự mới dùng cho lần phát hành này. */
    int32_t average_current_uA;
    uint32_t average_bus_voltage_uV;
    int64_t display_power_wide; /* Ngăn tràn số 32-bit khi nhân dòng điện trung bình với điện áp. */

    /* Bình thường số thứ tự tăng dần; sau khi uint32_t tràn sẽ tạm thời nhận giá trị 0. */
    next_sequence = g_current_sequence + 1U;

    /* Bỏ qua 0, luôn để sequence=0 chỉ biểu thị "chưa từng phát hành bản chụp hợp lệ". */
    if (next_sequence == 0U)
    {
        next_sequence = 1U;
    }

    /* Tất cả các trường được ghi vào struct cục bộ trước, trong lúc đó các tác vụ khác vẫn có thể đọc an toàn khung dữ liệu trước đó. */
    new_sample.sequence = next_sequence;
    new_sample.timestamp_ms = timestamp_ms;
    new_sample.current_uA = current_uA;
    new_sample.bus_voltage_uV = bus_voltage_uV;
    new_sample.shunt_voltage_nV = shunt_voltage_nV;
    new_sample.power_uW = power_uW;
    new_sample.valid = 1U;

    /* Sau khi cửa sổ đầy, loại bỏ mẫu cũ nhất trước rồi mới ghi mẫu mới của chu kỳ này. */
    if (g_current_window_count < CURRENT_DISPLAY_WINDOW)
    {
        g_current_window_count++;
    }
    else
    {
        g_current_sum -= g_current_window[g_current_window_index];
        g_bus_voltage_sum -=
            g_bus_voltage_window[g_current_window_index];
        g_shunt_voltage_sum -=
            g_shunt_voltage_window[g_current_window_index];
    }

    g_current_window[g_current_window_index] = current_uA;
    g_bus_voltage_window[g_current_window_index] = bus_voltage_uV;
    g_shunt_voltage_window[g_current_window_index] = shunt_voltage_nV;
    g_current_sum += current_uA;
    g_bus_voltage_sum += bus_voltage_uV;
    g_shunt_voltage_sum += shunt_voltage_nV;

    g_current_window_index++;
    if (g_current_window_index >= CURRENT_DISPLAY_WINDOW)
    {
        g_current_window_index = 0U;
    }

    display_sample = new_sample;
    average_current_uA =
        (int32_t)(g_current_sum / (int64_t)g_current_window_count);
    average_bus_voltage_uV =
        (uint32_t)(g_bus_voltage_sum /
                   (uint64_t)g_current_window_count);
    display_sample.shunt_voltage_nV =
        (int32_t)(g_shunt_voltage_sum /
                  (int64_t)g_current_window_count);

    /*
     * Khung đầu tiên được lượng tử hóa ngay lập tức. Về sau chỉ cập nhật khi giá trị trung bình vượt quá vùng chết 10mA hoặc 10mV,
     * nhờ đó nhiễu nhỏ sẽ không làm chữ số cuối trên CLI, CAN và OLED dao động qua lại.
     */
    if (g_current_display_initialized == 0U)
    {
        g_display_current_uA =
            Current_QuantizeTenMilliamp(average_current_uA);
        g_display_bus_voltage_uV =
            ((average_bus_voltage_uV + 5000UL) / 10000UL) * 10000UL;
        g_current_display_initialized = 1U;
    }
    else
    {
        if (Current_AbsoluteDifference(average_current_uA,
                                       g_display_current_uA) >= 10000ULL)
        {
            g_display_current_uA =
                Current_QuantizeTenMilliamp(average_current_uA);
        }

        if (((average_bus_voltage_uV >= g_display_bus_voltage_uV) &&
             ((average_bus_voltage_uV - g_display_bus_voltage_uV) >=
              10000UL)) ||
            ((average_bus_voltage_uV < g_display_bus_voltage_uV) &&
             ((g_display_bus_voltage_uV - average_bus_voltage_uV) >=
              10000UL)))
        {
            g_display_bus_voltage_uV =
                ((average_bus_voltage_uV + 5000UL) / 10000UL) * 10000UL;
        }
    }

    display_sample.current_uA = g_display_current_uA;
    display_sample.bus_voltage_uV = g_display_bus_voltage_uV;

    /* Nhân uA với uV rồi chia cho 10^6 để ra uW. */
    display_power_wide =
        ((int64_t)display_sample.current_uA *
         (int64_t)display_sample.bus_voltage_uV) / 1000000LL;
    if (display_power_wide > 2147483647LL)
    {
        display_sample.power_uW = 2147483647L;
    }
    else if (display_power_wide < (-2147483647LL - 1LL))
    {
        display_sample.power_uW = (-2147483647L - 1L);
    }
    else
    {
        display_sample.power_uW = (int32_t)display_power_wide;
    }

    /*
     * Từ đây trở đi chỉ thực hiện gán trong RAM; tạm thời cấm chuyển ngữ cảnh tác vụ và các ngắt được kiểm soát,
     * nhằm ngăn bên tiêu thụ đọc phải kết quả lẫn lộn giữa trường cũ và mới khi struct đang sao chép dở dang.
     */
    taskENTER_CRITICAL();
    g_latest_current_sample = new_sample;
    g_latest_current_display_sample = display_sample;
    g_current_sequence = next_sequence;
    g_current_sample_available = 1U;

    /* Phát hành thành công đồng nghĩa liên kết trong chu kỳ này đã phục hồi, xóa số lần lỗi liên tiếp và mã lỗi gần nhất về 0. */
    Current_IncrementSaturated(&g_current_diagnostics.publish_count);
    g_current_diagnostics.consecutive_fail_count = 0U;
    g_current_diagnostics.last_ina_status = 0U;
    g_current_diagnostics.last_i2c_status = 0U;
    taskEXIT_CRITICAL();
}

/* Ghi nhận một lần lỗi khởi tạo INA226 cùng nguyên nhân ở tầng thiết bị và tầng bus. */
void Current_ServiceRecordInitFailure(uint8_t ina_status,
                                      uint8_t i2c_status)
{
    /* Cập nhật nhiều trường chẩn đoán như một thao tác nhất quán. */
    taskENTER_CRITICAL();
    Current_IncrementSaturated(&g_current_diagnostics.init_fail_count);
    Current_IncrementSaturated(&g_current_diagnostics.consecutive_fail_count);
    g_current_diagnostics.last_ina_status = ina_status;
    g_current_diagnostics.last_i2c_status = i2c_status;
    taskEXIT_CRITICAL();
}

/* Ghi nhận một lần lỗi đo chu kỳ; không ghi đè bản chụp thành công cuối cùng. */
void Current_ServiceRecordReadFailure(uint8_t ina_status,
                                      uint8_t i2c_status)
{
    /*
     * Ở đây chỉ cập nhật thông tin chẩn đoán, không xóa g_latest_current_sample.
     * Bên tiêu thụ vẫn có thể đọc giá trị hợp lệ cuối cùng, nhưng phải dùng dấu thời gian để kiểm tra xem dữ liệu đã quá hạn chưa.
     */
    taskENTER_CRITICAL();
    Current_IncrementSaturated(&g_current_diagnostics.read_fail_count);
    Current_IncrementSaturated(&g_current_diagnostics.consecutive_fail_count);
    g_current_diagnostics.last_ina_status = ina_status;
    g_current_diagnostics.last_i2c_status = i2c_status;
    taskEXIT_CRITICAL();
}


/* Ghi nhận vượt quá chu kỳ của CurrentTask để tác vụ chẩn đoán theo dõi tính thời gian thực. */
void Current_ServiceRecordOverrun(void)
{
    /* overrun là chỉ số tính thời gian thực của bộ lập lịch, được thống kê tách biệt với lỗi giao tiếp INA226/I2C. */
    taskENTER_CRITICAL();
    Current_IncrementSaturated(&g_current_diagnostics.overrun_count);
    taskEXIT_CRITICAL();
}

/* Sao chép bản chụp hoàn chỉnh mới nhất vào sample; trả về 1 nếu có bản chụp, 0 nếu không có dữ liệu hoặc tham số không hợp lệ. */
uint8_t Current_GetLatest(CurrentSample_t *sample)
{
    uint8_t available; /* Lưu trạng thái đã có bản chụp hợp lệ khi đi vào vùng găng hay chưa. */

    /* Trả về thất bại khi bên gọi không cung cấp địa chỉ đầu ra. */
    if (sample == 0)
    {
        return 0U;
    }

    /* Sử dụng cùng cơ chế bảo vệ với phía phát hành để đảm bảo sao chép trọn vẹn một bản chụp nhất quán. */
    taskENTER_CRITICAL();
    available = g_current_sample_available;

    /* Giữ nguyên đối tượng ban đầu của bên gọi khi chưa có dữ liệu hợp lệ nào được phát hành. */
    if (available != 0U)
    {
        *sample = g_latest_current_sample;
    }
    taskEXIT_CRITICAL();

    return available;
}

/* Sao chép bản chụp giá trị trung bình hiển thị mới nhất cho CLI hoặc CAN. */
uint8_t Current_GetLatestDisplay(CurrentSample_t *sample)
{
    uint8_t available;

    if (sample == 0)
    {
        return 0U;
    }

    taskENTER_CRITICAL();
    available = g_current_sample_available;
    if (available != 0U)
    {
        *sample = g_latest_current_display_sample;
    }
    taskEXIT_CRITICAL();

    return available;
}

/* Kiểm tra bản chụp có hợp lệ và độ tuổi nhỏ hơn nghiêm ngặt max_age_ms hay không. */
uint8_t Current_IsFresh(const CurrentSample_t *sample,
                        uint32_t now_ms,
                        uint32_t max_age_ms)
{
    /* Con trỏ rỗng, cờ không hợp lệ hoặc số thứ tự dành riêng 0 đều không được coi là dữ liệu khả dụng. */
    if ((sample == 0) ||
        (sample->valid == 0U) ||
        (sample->sequence == 0U))
    {
        return 0U;
    }

    /* Khi age bằng ngưỡng là đã quá hạn, vì vậy phạm vi dữ liệu còn mới là từ 0 đến max_age_ms-1. */
    /*
     * Hiệu thời gian không dấu cho phép bộ đếm mili-giây 32-bit quay vòng tự nhiên.
     * Bên gọi phải sử dụng nguồn thời gian giống với timestamp_ms và không sớm hơn thời điểm lấy mẫu.
     */
    if ((uint32_t)(now_ms - sample->timestamp_ms) >= max_age_ms)
    {
        return 0U;
    }

    return 1U;
}

/* Lấy một bản chụp chẩn đoán Current. */
void Current_GetDiagnostics(CurrentDiagnostics_t *diagnostics)
{
    /* Bỏ qua ngay nếu địa chỉ đầu ra rỗng để tránh ghi bộ nhớ bất hợp pháp. */
    if (diagnostics == 0)
    {
        return;
    }

    /* Sao chép toàn bộ cấu trúc cần một vùng găng ngắn để tránh các bộ đếm đến từ những thời điểm khác nhau. */
    taskENTER_CRITICAL();
    *diagnostics = g_current_diagnostics;
    taskEXIT_CRITICAL();
}
