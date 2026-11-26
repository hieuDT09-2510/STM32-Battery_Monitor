/*
 * Tên tệp: app_current.h
 *
 * Tên module: Dịch vụ snapshot đo dòng điện
 *
 * Nhiệm vụ module:
 * 1. Nhận một bộ kết quả đo dòng điện đã được CurrentTask quy đổi xong;
 * 2. Bổ sung số thứ tự, dấu thời gian và cờ hợp lệ cho phép đo để tạo thành snapshot hoàn chỉnh;
 * 3. Công bố một lần trong vùng găng ngắn để tránh task khác đọc phải dữ liệu ghép nối từ các trường không đồng bộ;
 * 4. Cung cấp snapshot mới nhất, kiểm tra độ mới dữ liệu và chẩn đoán lỗi ở mức task.
 *
 * Đầu vào: Kết quả đo uA, uV, nV, uW mà CurrentTask nhận được từ bsp_ina226.
 * Đầu ra: CurrentSample_t có thể được sao chép an toàn bởi CAN, SOC và CLI.
 *
 * Ranh giới module:
 * - Không truy cập trực tiếp I2C2 hoặc thanh ghi INA226;
 * - Không quyết định chu kỳ lấy mẫu, không khởi tạo task FreeRTOS;
 * - Không thực hiện tích phân SOC, mã hóa CAN, phán đoán lỗi hoặc lưu vào Flash.
 */
#ifndef APP_CURRENT_H
#define APP_CURRENT_H

#include <stdint.h>

/* Một khung snapshot đo dòng điện đã được công bố hoàn chỉnh. */
typedef struct
{
    uint32_t sequence;         /* Số thứ tự công bố; bắt đầu từ 1, tăng thêm 1 sau mỗi lần công bố thành công, 0 nghĩa là chưa công bố. */
    uint32_t timestamp_ms;     /* Dấu thời gian hệ thống (ms) tại thời điểm INA226 đo xong và công bố. */
    int32_t current_uA;        /* Dòng điện, đơn vị uA; số dương là xả, số âm là sạc. */
    uint32_t bus_voltage_uV;   /* Điện áp bus phía IN- của INA226, đơn vị uV. */
    int32_t shunt_voltage_nV;  /* Điện áp hai đầu điện trở shunt, đơn vị nV, dấu biểu thị chiều dòng điện. */
    int32_t power_uW;          /* Công suất có dấu thu được từ dòng điện × điện áp bus, đơn vị uW. */
    uint8_t valid;             /* 1=cấu trúc này xuất phát từ một lần đo hoàn chỉnh thành công; 0=chưa có dữ liệu hợp lệ. */
} CurrentSample_t;

/* Chẩn đoán mức task của Current Service, không trùng lặp hay thay thế chẩn đoán tầng thấp của I2C và INA226. */
typedef struct
{
    uint32_t publish_count;          /* Tổng số lần công bố thành công snapshot hoàn chỉnh. */
    uint32_t init_fail_count;        /* Tổng số lần CurrentTask khởi tạo INA226 thất bại. */
    uint32_t read_fail_count;        /* Tổng số lần đọc định kỳ thất bại sau khi khởi tạo thành công. */
    uint32_t consecutive_fail_count; /* Số lần thất bại liên tiếp kể từ lần công bố thành công gần nhất. */
    uint32_t overrun_count;          /* Số lần CurrentTask phát hiện bản thân bị bỏ lỡ một hoặc nhiều chu kỳ 10ms. */
    uint8_t last_ina_status;         /* Giá trị INA226_Status_t của lần thất bại gần nhất; được xóa về 0 sau khi công bố thành công. */
    uint8_t last_i2c_status;         /* Giá trị I2C2_Status_t của lần thất bại gần nhất; được xóa về 0 sau khi công bố thành công. */
} CurrentDiagnostics_t;

/* Xóa trắng snapshot, số thứ tự và thông tin chẩn đoán khi cấp nguồn; gọi một lần trước khi tạo task. */
void Current_ServiceInit(void);

/*
 * Công bố một khung snapshot dòng điện hoàn chỉnh.
 * Tất cả các đại lượng vật lý phải đến từ cùng một lần gọi INA226_ReadMeasurement;
 * Hàm ghép dữ liệu trong cấu trúc cục bộ trước, sau đó thay thế toàn bộ snapshot toàn cục trong vùng găng ngắn.
 */
void Current_ServicePublish(int32_t current_uA,
                            uint32_t bus_voltage_uV,
                            int32_t shunt_voltage_nV,
                            int32_t power_uW,
                            uint32_t timestamp_ms);

/* Ghi nhận một lần khởi tạo INA226 thất bại cùng nguyên nhân ở tầng linh kiện và tầng bus. */
void Current_ServiceRecordInitFailure(uint8_t ina_status,
                                      uint8_t i2c_status);

/* Ghi nhận một lần đo định kỳ thất bại; không ghi đè khung snapshot thành công cuối cùng. */
void Current_ServiceRecordReadFailure(uint8_t ina_status,
                                      uint8_t i2c_status);

/* Ghi nhận trường hợp CurrentTask quá hạn chu kỳ để task chẩn đoán theo dõi tính thời gian thực. */
void Current_ServiceRecordOverrun(void);

/* Sao chép snapshot hoàn chỉnh mới nhất vào sample; trả về 1 nếu có snapshot, 0 nếu không có dữ liệu hoặc tham số không hợp lệ. */
uint8_t Current_GetLatest(CurrentSample_t *sample);

/*
 * Sao chép snapshot trung bình trượt 8 điểm dùng cho hiển thị CLI và CAN.
 * SOC, SOH và các logic điều khiển khác phải tiếp tục sử dụng Current_GetLatest hoặc kết quả đo thô của chu kỳ hiện tại.
 */
uint8_t Current_GetLatestDisplay(CurrentSample_t *sample);

/* Kiểm tra xem snapshot có hợp lệ và độ tuổi có nhỏ hơn nghiêm ngặt max_age_ms hay không. */
uint8_t Current_IsFresh(const CurrentSample_t *sample,
                        uint32_t now_ms,
                        uint32_t max_age_ms);

/* Sao chép một bản snapshot chẩn đoán Current hoàn chỉnh trong vùng găng ngắn. */
void Current_GetDiagnostics(CurrentDiagnostics_t *diagnostics);

#endif /* APP_CURRENT_H */
