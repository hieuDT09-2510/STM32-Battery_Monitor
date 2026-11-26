/*
 * Mô-đun: Dịch vụ chuyển đổi cảm biến chung và snapshot mới nhất.
 * Đầu vào: Giá trị thô của kênh điện áp PA0 và kênh nhiệt độ PA1 do ADC/DMA cung cấp.
 * Đầu ra: Snapshot hoàn chỉnh kèm số thứ tự, timestamp, đại lượng vật lý, cờ hiệu lực và phân loại NTC.
 * Sensor Service là consumer duy nhất của ADC_ReadSafe; các mô-đun khác chỉ đọc snapshot.
 * Mô-đun này không xác nhận lỗi, không thay đổi trạng thái BMS, không gửi CAN, không ghi Flash.
 */
#ifndef APP_SENSOR_H
#define APP_SENSOR_H

// Include kiểu số nguyên độ rộng cố định, tránh phụ thuộc thanh ghi vi điều khiển cụ thể.
#include <stdint.h>

// Định nghĩa phân loại đầu vào NTC, giữ nguyên tên và mã số dùng trong test program gốc.
typedef enum
{
    // Chưa phân loại.
    NTC_INPUT_UNKNOWN = 0,
    // Giá trị thô cho phép thực hiện chuyển đổi nhiệt độ, không có nghĩa là nhiệt độ chưa vượt giới hạn.
    NTC_INPUT_NORMAL = 1,
    // Đầu vào gần cực đại thang đo, nghi ngờ NTC hoặc đường dây bị hở mạch.
    NTC_INPUT_OPEN_SUSPECT = 2,
    // Đầu vào gần điện thế đất (GND), nghi ngờ NTC hoặc đường dây bị ngắn mạch.
    NTC_INPUT_SHORT_SUSPECT = 3,
    // Giá trị thô vượt quá phạm vi ADC 12-bit.
    NTC_INPUT_ADC_INVALID = 4
} NTC_InputState_t;

// Định nghĩa kết quả hoàn chỉnh của một lần chuyển đổi, do caller lưu giữ.
typedef struct
{
    // Tăng 1 mỗi khi nhận một nhóm dữ liệu 2 kênh mới; 0 biểu thị chưa có snapshot hợp lệ.
    uint32_t sequence;
    // Thời gian mili-giây hệ thống khi Sensor Service nhận nhóm dữ liệu này.
    uint32_t timestamp_ms;
    // Lưu giá trị đọc thô của PA0.
    uint16_t adc_pa0;
    // Lưu giá trị đọc thô của PA1.
    uint16_t adc_pa1;
    // Lưu điện áp chân PA0, đơn vị V.
    float pa0_voltage_v;
    // Lưu điện áp pin sau khi khôi phục tỉ lệ, đơn vị V.
    float cell_voltage_v;
    // Lưu nhiệt độ NTC, đơn vị độ C.
    float temperature_c;
    // 1 biểu thị điện áp đã chuyển đổi, 0 biểu thị điện áp lần này không khả dụng.
    uint8_t voltage_valid;
    // 1 biểu thị nhiệt độ đã chuyển đổi, 0 biểu thị nhiệt độ lần này không khả dụng.
    uint8_t temperature_valid;
    // Lưu phân loại đầu vào NTC lần này, không tương đương mã lỗi đã xác nhận.
    NTC_InputState_t ntc_input;
} SensorSample_t;


/*
 * Chuyển đổi một nhóm giá trị thô; mỗi lần ghi đè kết quả, từng kênh phán đoán độc lập.
 * Trường giá trị của kênh không hợp lệ điền 0, caller bắt buộc phải kiểm tra cờ valid tương ứng trước.
 * sample rỗng thì trả về ngay, không truy cập bất kỳ đầu ra nào.
 * API này không ghi nhận thời điểm lấy mẫu, không chứng minh dữ liệu còn tươi mới hoặc 2 kênh đồng bộ.
 */
void Sensor_Convert(uint16_t adc0, uint16_t adc1, SensorSample_t *sample);

/* Khởi tạo snapshot mới nhất;
 *nên gọi một lần trước khi ADC bắt đầu tạo dữ liệu. */
void Sensor_ServiceInit(void);

/*
 * Thử lấy một nhóm dữ liệu mới từ ADC/DMA.
 * Tạo thành công snapshot mới trả về 1; hiện tại không có dữ liệu mới trả về 0.
 */
uint8_t Sensor_ServiceUpdate(uint32_t now_ms);

/* Khi có snapshot sao chép cho caller và trả về 1; chưa có snapshot hoặc tham số rỗng trả về 0. */
uint8_t Sensor_GetLatest(SensorSample_t *sample);

/*
 * Sao chép snapshot dùng cho hiển thị CLI và CAN.
 * Trong đó điện áp pin là trung bình trượt của 16 mẫu hợp lệ gần nhất; timestamp, số thứ tự,
 * nhiệt độ và cờ hiệu lực vẫn đến từ khung mới nhất. Phán đoán bảo vệ không được dùng API này; ước tính SOC dùng nó để giảm rung ADC.
 */
uint8_t Sensor_GetLatestDisplay(SensorSample_t *sample);

/*
 * Tính tuổi của snapshot chỉ phục vụ hiển thị chẩn đoán.
 * Khi timestamp của snapshot hơi đi trước do chuyển ngữ cảnh task thì trả về 0, tránh tràn dưới số không dấu.
 */
uint32_t Sensor_CalculateAgeMs(const SensorSample_t *sample,
                               uint32_t now_ms);

/* Đánh giá tuổi snapshot có vượt quá giá trị cho phép không; trả về 0 khi không có snapshot hoặc tham số rỗng. */
uint8_t Sensor_IsFresh(const SensorSample_t *sample,
                       uint32_t now_ms,
                       uint32_t max_age_ms);


#endif
