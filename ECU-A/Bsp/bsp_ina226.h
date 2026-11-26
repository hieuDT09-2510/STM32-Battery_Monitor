/*
 * Tên file: bsp_ina226.h
 *
 * Tên mô-đun: Driver thiết bị INA226
 *
 * Trách nhiệm mô-đun:
 * 1. Kiểm tra Manufacturer ID và Die ID, tránh nhận nhầm thiết bị khác thành INA226;
 * 2. Ghi và đọc lại thanh ghi cấu hình (Configuration) và thanh ghi hiệu chuẩn (Calibration);
 * 3. Đọc 3 thanh ghi thô: điện áp shunt, điện áp bus và dòng điện;
 * 4. Chuyển đổi thành nV, uV, uA và uW theo thông số phần cứng hiện tại;
 * 5. Lưu thông tin chẩn đoán như kết quả khởi tạo, đọc và trạng thái I2C tầng dưới.
 *
 * Luồng dữ liệu:
 * bsp_i2c2 trả về giá trị thanh ghi thô 16-bit
 *              ↓
 * Mô-đun này giải thích dấu, LSB và thông số hiệu chuẩn theo datasheet INA226
 *              ↓
 * CurrentTask nhận được một nhóm INA226_Measurement_t có đơn vị rõ ràng
 *
 * Ranh giới mô-đun:
 * - Mô-đun này không hiện thực timing PB10/PB11, mọi truy cập bus giao cho bsp_i2c2;
 * - Mô-đun này không quyết định chu kỳ lấy mẫu 10ms, không phát hành snapshot liên task;
 * - Mô-đun này không làm tích phân SOC, phán đoán lỗi, gửi CAN hay điều khiển tải.
 */
#ifndef BSP_INA226_H
#define BSP_INA226_H

/* Định nghĩa kiểu số nguyên độ dài cố định, đảm bảo độ rộng thanh ghi và đại lượng vật lý rõ ràng. */
#include <stdint.h>

/* Sử dụng I2C2_Status_t và API đọc/ghi thanh ghi I2C2. */
#include "bsp_i2c2.h"

/* Trạng thái trả về của một thao tác cấp thiết bị INA226. */
typedef enum
{
    INA226_STATUS_OK = 0,                    /* Khởi tạo hoặc đo lường hoàn tất thành công. */
    INA226_STATUS_I2C_ERROR,                 /* Transaction I2C tầng dưới thất bại, chi tiết xem last_i2c_status. */
    INA226_STATUS_BAD_MANUFACTURER_ID,       /* Thanh ghi 0xFE không phải định danh TI 0x5449. */
    INA226_STATUS_BAD_DIE_ID,                /* Các bit model chip của thanh ghi 0xFF không phải 0x226. */
    INA226_STATUS_CONFIG_VERIFY_FAILED,      /* Đọc lại các bit hiệu lực của thanh ghi cấu hình không khớp giá trị đã ghi. */
    INA226_STATUS_CALIBRATION_VERIFY_FAILED, /* Giá trị đọc lại thanh ghi hiệu chuẩn không phải 0x0800. */
    INA226_STATUS_INVALID_ARGUMENT           /* Phía gọi truyền địa chỉ struct đo lường rỗng (NULL). */
} INA226_Status_t;

/*
 * Kết quả một lần đo INA226 hoàn chỉnh.
 *
 * Các trường raw giữ nguyên giá trị thanh ghi thô của chip, thuận tiện cho hiệu chuẩn và khắc phục sự cố;
 * các trường có đơn vị dùng trực tiếp cho Current Service, SOC, CAN và CLI.
 */
typedef struct
{
    int16_t raw_shunt;       /* Bù 2 thô của thanh ghi điện áp shunt, 1bit=2.5uV, có thể âm hoặc dương. */
    uint16_t raw_bus;        /* Giá trị thô không dấu của thanh ghi điện áp bus, 1bit=1.25mV. */
    int16_t raw_current;     /* Bù 2 thô của thanh ghi dòng điện, 1bit do giá trị hiệu chuẩn xác định là 25uA. */
    int32_t shunt_voltage_nV;/* Điện áp shunt, đơn vị nV; dùng nV biểu diễn trọn vẹn độ chính xác 0.5uV của 2.5uV. */
    uint32_t bus_voltage_uV; /* Điện áp bus phía IN-, đơn vị uV, giá trị luôn không âm. */
    int32_t current_uA;      /* Dòng điện, đơn vị uA; dự án này quy ước số dương là xả, số âm là sạc. */
    int32_t power_uW;        /* Công suất chuyển đổi từ current_uA x bus_voltage_uV, đơn vị uW, giữ dấu hướng. */
} INA226_Measurement_t;

/* Snapshot chẩn đoán cấp thiết bị INA226, cho CLI hiển thị trạng thái khởi tạo và sức khỏe. */
typedef struct
{
    uint32_t init_attempt_count;   /* Tổng số lần gọi INA226_Init. */
    uint32_t init_ok_count;        /* Số lần khởi tạo mà ID, cấu hình và hiệu chuẩn đều xác thực thành công. */
    uint32_t read_ok_count;        /* Số lần cả 3 thanh ghi đo lường đều đọc và chuyển đổi hoàn tất. */
    uint32_t read_fail_count;      /* Số lần bất kỳ thanh ghi nào bị lỗi khi đọc trong quá trình đo. */
    uint16_t manufacturer_id;      /* Giá trị thô của Manufacturer ID 0xFE đọc gần nhất. */
    uint16_t die_id;               /* Giá trị thô của Die ID 0xFF và số revision đọc gần nhất. */
    uint16_t config_readback;      /* Giá trị thô đọc lại từ 0x00 sau khi ghi cấu hình. */
    uint16_t calibration_readback; /* Giá trị thô đọc lại từ 0x05 sau khi ghi giá trị hiệu chuẩn. */
    uint8_t initialized;           /* 1=Khởi tạo gần nhất thành công; 0=Chưa thành công hoặc khởi tạo gần nhất thất bại. */
    uint8_t last_status;           /* Kết quả thao tác INA226 gần nhất, tương ứng INA226_Status_t. */
    uint8_t last_i2c_status;       /* Kết quả thao tác I2C tầng dưới gần nhất, tương ứng I2C2_Status_t. */
} INA226_Diagnostics_t;

/* Xóa về 0 toàn bộ dữ liệu chẩn đoán INA226; không truy cập thanh ghi phần cứng. */
void INA226_ResetDiagnostics(void);

/* Kiểm tra định danh thiết bị, ghi giá trị cấu hình/hiệu chuẩn và đọc lại xác thực. */
INA226_Status_t INA226_Init(void);

/*
 * Đọc liên tục điện áp shunt, điện áp bus và dòng điện, hoàn thành chuyển đổi số nguyên (fixed-point).
 * Chỉ khi trả về INA226_STATUS_OK, toàn bộ các trường trong measurement mới là kết quả mới của cùng một lần gọi.
 */
INA226_Status_t INA226_ReadMeasurement(INA226_Measurement_t *measurement);

/* Sao chép nguyên tử snapshot chẩn đoán cấp thiết bị; con trỏ NULL được bỏ qua an toàn. */
void INA226_GetDiagnostics(INA226_Diagnostics_t *diagnostics);

#endif /* BSP_INA226_H */
