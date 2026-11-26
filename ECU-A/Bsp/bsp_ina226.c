/*
 * Tên file: bsp_ina226.c
 *
 * Tác dụng mô-đun: Chuyển đổi giao thức thanh ghi INA226 thành kết quả dòng điện, điện áp và công suất có đơn vị.
 *
 * Thông số phần cứng: Rshunt=0.1Ω, Current_LSB=25µA/bit.
 * CAL=0.00512/(25µA×0.1Ω)=2048=0x0800。
 * Cấu hình chuyển đổi liên tục shunt + bus, mỗi cái 588µs, lấy trung bình 4 lần,
 * một nhóm kết quả trung bình cập nhật khoảng 4.704ms một lần.
 *
 * Tại sao không dùng số thực dấu phẩy động (float):
 * STM32F103 không có FPU phần cứng. Ở đây sử dụng thống nhất số nguyên nV, uV, uA và uW,
 * vừa giữ được độ phân giải cần thiết,vừa tránh thực hiện phép tính dấu phẩy động trong CurrentTask 10ms.
 */
#include "bsp_ina226.h"

#include "stm32f10x.h"

/* Địa chỉ I2C 7-bit do cách đấu nối A0/A1 của INA226 trên PCB hiện tại xác định. */
#define INA226_ADDRESS_7BIT            0x44U

/* Địa chỉ thanh ghi nội bộ INA226. */
#define INA226_REG_CONFIG              0x00U  // Cài đặt
#define INA226_REG_SHUNT_VOLTAGE       0x01U  // Điện áp shunt
#define INA226_REG_BUS_VOLTAGE         0x02U  // Điện áp bus
#define INA226_REG_CURRENT             0x04U  // Dòng điện
#define INA226_REG_CALIBRATION         0x05U  // Thanh ghi hiệu chuẩn
#define INA226_REG_MANUFACTURER_ID     0xFEU  // Mã nhà sản xuất
#define INA226_REG_DIE_ID              0xFFU  // Mã thiết bị

/* ID nhà sản xuất cố định của TI, và giá trị kiểm tra trường ID chip INA226. */
#define INA226_EXPECTED_MANUFACTURER   0x5449U
/* 4 bit thấp của DIE_ID là số revision, thay đổi theo phiên bản chip nên không tham gia so sánh model. */
#define INA226_EXPECTED_DIE_MASK       0xFFF0U
#define INA226_EXPECTED_DIE            0x2260U

/*
 * 0x02DF: AVG=4 lần, VBUSCT=588us，VSHCT=588us，MODE=đo liên tục shunt + bus.
 */
#define INA226_CONFIG_VALUE            0x02DFU
/* D14:D12 là bit dành riêng, phần cứng có thể đọc về cố định là 100b; chỉ kiểm tra cấu hình D11:D0. */
#define INA226_CONFIG_VERIFY_MASK       0x0FFFU
/* Giá trị thanh ghi hiệu chuẩn tính theo điện trở shunt 0.1Ω và 25uA/bit. */
#define INA226_CALIBRATION_VALUE       0x0800U
/* Thanh ghi dòng điện mỗi 1bit đại diện cho 25uA; hậu tố L đảm bảo phép nhân thực thi ít nhất theo số có dấu 32-bit. */
#define INA226_CURRENT_LSB_UA          25L

/* Trạng thái chẩn đoán tích lũy cấp thiết bị INA226, chỉ do file này ghi. */
static INA226_Diagnostics_t g_ina226_diagnostics;

/* Bộ đếm tăng bão hòa, tránh quay vòng về 0 sau khi đạt giá trị cực đại 32-bit. */
static void INA226_IncrementSaturated(uint32_t *value)
{
    if (*value < 0xFFFFFFFFUL)
    {
        (*value)++;
    }
}


/// @brief Đọc giá trị 16-bit của thanh ghi chỉ định trên INA226.
/// @param register_address Địa chỉ thanh ghi.
/// @param value Con trỏ tới biến 16-bit lưu kết quả đọc.
/// @return Trả về mã trạng thái INA226, chỉ thị thao tác có thành công không.
static INA226_Status_t INA226_ReadRegister(uint8_t register_address,
                                            uint16_t *value)
{
    I2C2_Status_t i2c_status; /* Lưu kết quả chi tiết của transaction đọc I2C tầng dưới. */

    /* Địa chỉ thiết bị cố định là 0x44, caller chỉ cần cung cấp địa chỉ thanh ghi nội bộ. */
    i2c_status = BSP_I2C2_ReadRegister16(INA226_ADDRESS_7BIT,
                                         register_address,
                                         value);
    /* Dù cấp thiết bị chỉ trả về I2C_ERROR, vẫn giữ lại nguyên nhân cụ thể NACK/timeout tầng dưới. */
    g_ina226_diagnostics.last_i2c_status = (uint8_t)i2c_status;

    /* Ánh xạ thống nhất mọi lỗi I2C tầng dưới thành I2C_ERROR cấp thiết bị. */
    if (i2c_status != I2C2_STATUS_OK)
    {
        return INA226_STATUS_I2C_ERROR;
    }

    return INA226_STATUS_OK;
}

/* Dùng để ghi giá trị 16-bit vào thanh ghi chỉ định của thiết bị INA226 qua giao tiếp I2C.
  * Hàm này trước tiên gọi BSP_I2C2_WriteRegister16 để thực hiện thao tác ghi,
  * và trả về mã trạng thái tương ứng dựa trên trạng thái I2C,
  * trong quá trình xử lý sẽ lưu trạng thái I2C cuối cùng để chẩn đoán.
 */
static INA226_Status_t INA226_WriteRegister(uint8_t register_address,
                                             uint16_t value)
{
    I2C2_Status_t i2c_status; /* Lưu kết quả chi tiết của transaction ghi I2C tầng dưới. */

    /* Tất cả các thanh ghi cấu hình INA226 đều gửi theo byte cao trước. */
    i2c_status = BSP_I2C2_WriteRegister16(INA226_ADDRESS_7BIT,
                                          register_address,
                                          value);
    g_ina226_diagnostics.last_i2c_status = (uint8_t)i2c_status;

    if (i2c_status != I2C2_STATUS_OK)
    {
        return INA226_STATUS_I2C_ERROR;
    }

    return INA226_STATUS_OK;
}

/* Xóa về 0 toàn bộ dữ liệu chẩn đoán INA226; không truy cập thanh ghi phần cứng. */
void INA226_ResetDiagnostics(void)
{
    /* Khởi tạo tập hợp đảm bảo khi struct thêm trường mới trong tương lai cũng tự động xóa về 0. */
    INA226_Diagnostics_t empty = {0};
    g_ina226_diagnostics = empty;
}

/* Kiểm tra định danh thiết bị, ghi cấu hình/hiệu chuẩn và đọc lại xác thực. */
INA226_Status_t INA226_Init(void)
{
    INA226_Status_t status; /* Lưu kết quả bước khởi tạo hiện tại; khi thất bại giữ nguyên đưa ra lối ra chung. */
    uint16_t value;         /* Biến tạm đọc lại thanh ghi 16-bit được tái sử dụng. */

    /* Mỗi lần gọi đều thống kê số lần thử, và thu hồi cờ "khởi tạo thành công" trước. */
    INA226_IncrementSaturated(&g_ina226_diagnostics.init_attempt_count);
    g_ina226_diagnostics.initialized = 0U;

    /* Xóa giá trị đọc lại cấu hình chưa được xác thực lại của vòng này, tránh CLI hiển thị hiện trường thành công cũ. */
    g_ina226_diagnostics.config_readback = 0U;
    g_ina226_diagnostics.calibration_readback = 0U;

    /* Bước 1: Đọc TI Manufacturer ID, xác nhận địa chỉ 0x44 thực sự có thiết bị của hãng kỳ vọng. */
    status = INA226_ReadRegister(INA226_REG_MANUFACTURER_ID, &value);
    if (status != INA226_STATUS_OK) { goto init_failed; }
    g_ina226_diagnostics.manufacturer_id = value;

    /* 0x5449 là định danh cố định TI theo nghĩa ASCII; dừng cấu hình tiếp theo khi không khớp. */
    if (value != INA226_EXPECTED_MANUFACTURER)
    {
        status = INA226_STATUS_BAD_MANUFACTURER_ID;
        goto init_failed;
    }

    /* Bước 2: Kiểm tra model chip; 4 bit thấp số revision được loại trừ bằng mặt nạ. */
    status = INA226_ReadRegister(INA226_REG_DIE_ID, &value);
    if (status != INA226_STATUS_OK) { goto init_failed; }
    g_ina226_diagnostics.die_id = value;
    if ((value & INA226_EXPECTED_DIE_MASK) != INA226_EXPECTED_DIE)
    {
        status = INA226_STATUS_BAD_DIE_ID;
        goto init_failed;
    }

    /* Bước 3: Cấu hình số lần lấy trung bình, thời gian chuyển đổi và chế độ chuyển đổi liên tục. */
    status = INA226_WriteRegister(INA226_REG_CONFIG,
                                  INA226_CONFIG_VALUE);
    if (status != INA226_STATUS_OK) { goto init_failed; }

    /* Bước 4: Ghi hệ số hiệu chuẩn, làm cho 1bit của thanh ghi dòng điện bằng 25uA. */
    status = INA226_WriteRegister(INA226_REG_CALIBRATION,
                                  INA226_CALIBRATION_VALUE);
    if (status != INA226_STATUS_OK) { goto init_failed; }

    /* Bước 5: Đọc lại cấu hình, xác thực việc ghi I2C thực sự đã nạp vào thanh ghi chip. */
    status = INA226_ReadRegister(INA226_REG_CONFIG, &value);
    if (status != INA226_STATUS_OK) { goto init_failed; }
    g_ina226_diagnostics.config_readback = value;
    /*
     * Thanh ghi cấu hình INA226 D14:D12 là bit dành riêng. Thực đo D14 cố định đọc về 1,
     * do đó sau khi ghi 0x02DF thì giá trị đọc lại hợp lệ là 0x42DF, không thể so sánh nghiêm ngặt bit dành riêng.
     */
    if ((value & INA226_CONFIG_VERIFY_MASK) !=
        (INA226_CONFIG_VALUE & INA226_CONFIG_VERIFY_MASK))
    {
        status = INA226_STATUS_CONFIG_VERIFY_FAILED;
        goto init_failed;
    }

    /* Bước 6: Đọc lại giá trị hiệu chuẩn; thanh ghi này không chứa bit đọc lại dành riêng cần bỏ qua. */
    status = INA226_ReadRegister(INA226_REG_CALIBRATION, &value);
    if (status != INA226_STATUS_OK) { goto init_failed; }
    g_ina226_diagnostics.calibration_readback = value;
    if (value != INA226_CALIBRATION_VALUE)
    {
        status = INA226_STATUS_CALIBRATION_VERIFY_FAILED;
        goto init_failed;
    }

    /* Sau khi tất cả kiểm tra định danh, ghi và đọc lại vượt qua, mới cho phép set cờ khởi tạo thành công. */
    g_ina226_diagnostics.initialized = 1U;
    g_ina226_diagnostics.last_status = (uint8_t)INA226_STATUS_OK;
    INA226_IncrementSaturated(&g_ina226_diagnostics.init_ok_count);
    return INA226_STATUS_OK;

init_failed:
    /*
     * Mọi luồng thất bại đều hội tụ về đây, giữ lại trạng thái cấp thiết bị cụ thể.
     * initialized đã được xóa về 0 ở đầu hàm, khi thất bại sẽ không còn sót lại trạng thái thành công cũ.
     */
    g_ina226_diagnostics.last_status = (uint8_t)status;
    return status;
}

/*
 * Đọc liên tục điện áp shunt, điện áp bus và dòng điện, hoàn thành chuyển đổi số nguyên fixed-point.
 * Chỉ khi trả về INA226_STATUS_OK, toàn bộ các trường trong measurement mới là kết quả mới của cùng một lần gọi.
 */
INA226_Status_t INA226_ReadMeasurement(INA226_Measurement_t *measurement)
{
    INA226_Status_t status; /* Lưu kết quả lần đọc hiện tại trong 3 thanh ghi. */
    uint16_t raw_shunt;     /* Dạng bit thô 16-bit của thanh ghi điện áp shunt. */
    uint16_t raw_bus;       /* Giá trị thô không dấu 16-bit của thanh ghi điện áp bus. */
    uint16_t raw_current;   /* Dạng bit bù 2 thô 16-bit của thanh ghi dòng điện. */
    int64_t power_pW;       /* uA x uV thu được pW; dùng 64-bit ngăn tràn số phép nhân trung gian. */

    /* Caller bắt buộc phải cung cấp địa chỉ lưu kết quả; con trỏ NULL không thể ghi. */
    if (measurement == 0)
    {
        return INA226_STATUS_INVALID_ARGUMENT;
    }

    /* Lần lượt đọc 3 đại lượng vật lý; bất kỳ bước nào thất bại đều không phát hành struct đo lường "cập nhật một phần". */
    status = INA226_ReadRegister(INA226_REG_SHUNT_VOLTAGE, &raw_shunt);
    if (status != INA226_STATUS_OK) { goto read_failed; }

    status = INA226_ReadRegister(INA226_REG_BUS_VOLTAGE, &raw_bus);
    if (status != INA226_STATUS_OK) { goto read_failed; }

    status = INA226_ReadRegister(INA226_REG_CURRENT, &raw_current);
    if (status != INA226_STATUS_OK) { goto read_failed; }

    /* Thanh ghi điện áp shunt và dòng điện là bù 2 nhị phân, ép kiểu int16_t khôi phục dấu âm dương. */
    measurement->raw_shunt = (int16_t)raw_shunt;
    measurement->raw_bus = raw_bus;
    measurement->raw_current = (int16_t)raw_current;

    /* Shunt LSB=2.5µV, nên dùng nV lưu trữ tránh làm mất 0.5µV. */
    measurement->shunt_voltage_nV =
        (int32_t)measurement->raw_shunt * 2500L;

    /* Bus LSB=1.25mV=1250uV, thanh ghi điện áp bus là số không dấu. */
    measurement->bus_voltage_uV =
        (uint32_t)measurement->raw_bus * 1250UL;

    /* Thanh ghi hiệu chuẩn đã định nghĩa LSB dòng điện là 25uA/bit. */
    measurement->current_uA =
        (int32_t)measurement->raw_current * INA226_CURRENT_LSB_UA;

    /* µA x µV = pW, chia cho 1,000,000 thu được µW; dấu giữ nguyên hướng sạc/xả. */
    power_pW = (int64_t)measurement->current_uA *
               (int64_t)measurement->bus_voltage_uV;
    measurement->power_uW = (int32_t)(power_pW / 1000000LL);

    /* Chỉ sau khi cả 3 thanh ghi và mọi chuyển đổi thành công, mới tích lũy một lần đo hoàn chỉnh thành công. */
    g_ina226_diagnostics.last_status = (uint8_t)INA226_STATUS_OK;
    INA226_IncrementSaturated(&g_ina226_diagnostics.read_ok_count);
    return INA226_STATUS_OK;

read_failed:
    /*
     * Khi thất bại không xóa struct gốc của caller, cũng không tăng số lần thành công;
     * CurrentTask thấy giá trị trả về khác OK sẽ không phát hành kết quả chưa hoàn chỉnh này.
     */
    g_ina226_diagnostics.last_status = (uint8_t)status;
    INA226_IncrementSaturated(&g_ina226_diagnostics.read_fail_count);
    return status;
}

/* Sao chép nguyên tử snapshot chẩn đoán cấp thiết bị; con trỏ NULL được bỏ qua an toàn. */
void INA226_GetDiagnostics(INA226_Diagnostics_t *diagnostics)
{
    uint32_t primask; /* Lưu PRIMASK trước khi sao chép, khôi phục trạng thái ngắt ban đầu khi kết thúc. */

    /* Trả về ngay khi không có địa chỉ đầu ra. */
    if (diagnostics == 0)
    {
        return;
    }

    /* Tắt ngắt trong chốc lát trong lúc sao chép nhiều trường của struct, thu được snapshot chẩn đoán nhất quán. */
    primask = __get_PRIMASK();
    __disable_irq();
    *diagnostics = g_ina226_diagnostics;
    /* Chỉ khôi phục khi ngắt đã bật trước khi vào, không tự ý mở ngắt mà caller đã chủ động tắt. */
    if (primask == 0U)
    {
        __enable_irq();
    }
}
