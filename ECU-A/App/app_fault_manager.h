/*
 * Mô-đun: Giao diện công khai của trình quản lý lỗi thống nhất.
 * Chức năng: Định nghĩa mã lỗi 16-bit, vòng đời của từng lỗi, trạng thái tổng hợp và giao diện xóa thủ công.
 * Phạm vi: Mô-đun này không đọc thanh ghi ADC, không điều khiển còi báo, không gửi CAN, không ghi Flash,
 *          và cũng không chuyển trực tiếp trạng thái BMS; các nguồn lỗi cụ thể chỉ gửi điều kiện boolean và thời gian hệ thống tới nó.
 * Đồng thời (Concurrency):
 * 1. BmsTask là tác nhân duy nhất được phép sửa vòng đời lỗi và trạng thái tổng hợp;
 * 2. Các tác vụ khác chỉ có thể đọc bản sao thông qua FaultManager_GetSnapshot();
 * 3. Việc sao chép snapshot dùng vùng găng (critical section) ngắn để đảm bảo dữ liệu trong struct cùng thuộc một thời điểm;
 * 4. CLI chỉ có thể sửa mặt nạ chèn lỗi (injection) thử nghiệm độc lập, không thể sửa trực tiếp trạng thái lỗi.
 */
#ifndef APP_FAULT_MANAGER_H
#define APP_FAULT_MANAGER_H

#include <stdint.h>

// Nạp SensorSample_t, trình quản lý lỗi lấy dữ liệu tin cậy thông qua snapshot cảm biến.
#include "app_sensor.h"

/*
 * Mặt nạ bit lỗi -- dùng để biểu diễn vị trí bit lỗi.
 * Mỗi loại lỗi chiếm một bit nhị phân, nên nhiều lỗi có thể được lưu đồng thời bằng phép OR theo bit.
 * Ví dụ: Khi UV và OT cùng tồn tại, mặt nạ lỗi là 0x0002 | 0x0004 = 0x0006.
 * Hiện tại sử dụng bit0~bit9; các bit cao còn lại dành cho lỗi sức khỏe tác vụ, Watchdog và Flash.
 * Mỗi loại lỗi chiếm một bit nhị phân.
 */

#define FAULT_MASK_NONE          ((uint16_t)0x0000U)
#define FAULT_MASK_OV            ((uint16_t)0x0001U)
#define FAULT_MASK_UV            ((uint16_t)0x0002U)
#define FAULT_MASK_OT            ((uint16_t)0x0004U)
#define FAULT_MASK_NTC_OPEN      ((uint16_t)0x0008U)
#define FAULT_MASK_NTC_SHORT     ((uint16_t)0x0010U)
#define FAULT_MASK_SENSOR_DATA   ((uint16_t)0x0020U)
#define FAULT_MASK_ADC_TIMEOUT   ((uint16_t)0x0040U)
#define FAULT_MASK_SELF_CHECK    ((uint16_t)0x0080U)
// Bộ đếm lỗi CAN ở mức cao, chất lượng truyền thông đã suy giảm nhưng bộ điều khiển chưa ngoại tuyến.
#define FAULT_MASK_CAN_DEGRADED  ((uint16_t)0x0100U)

// Bộ đếm lỗi phát CAN vượt quá giới hạn, bộ điều khiển đã vào trạng thái Bus-Off.
#define FAULT_MASK_CAN_BUS_OFF   ((uint16_t)0x0200U)

/*
 * Phạm vi lỗi cho phép CLI chèn (inject).
 * Không bao gồm SELF_CHECK và lỗi phần cứng CAN, tránh thử nghiệm phần mềm giả mạo trạng thái phần cứng thật.
 */
#define FAULT_INJECT_ALLOWED_MASK  \
    ((uint16_t)(FAULT_MASK_OV | FAULT_MASK_UV | FAULT_MASK_OT | \
                FAULT_MASK_NTC_OPEN | FAULT_MASK_NTC_SHORT | \
                FAULT_MASK_SENSOR_DATA | FAULT_MASK_ADC_TIMEOUT))

/* Mặt nạ chèn lỗi thử nghiệm hiện tại, công khai để chẩn đoán trên Keil Watch, mã nghiệp vụ sửa đổi thông qua giao diện hàm. */
extern volatile uint16_t g_fault_inject_mask;
/*
 * Chỉ số mảng lỗi -- dùng để tìm vị trí trong mảng.
 * FaultId_t chỉ dùng để truy cập items[] và bảng cấu hình; khi truyền ra ngoài sử dụng mặt nạ lỗi 16-bit ở trên.
 */
typedef enum
{
    // Quá áp pin.
    FAULT_ID_OV = 0,
    // Thấp áp pin.
    FAULT_ID_UV,
    // Quá nhiệt pin.
    FAULT_ID_OT,
    // NTC hoặc đường dây NTC nghi ngờ hở mạch.
    FAULT_ID_NTC_OPEN,
    // NTC hoặc đường dây NTC nghi ngờ ngắn mạch.
    FAULT_ID_NTC_SHORT,
    // Giá trị thô hoặc quan hệ các trường của snapshot Sensor không đáng tin cậy (lỗi dữ liệu).
    FAULT_ID_SENSOR_DATA,
    // Quá thời gian cho phép mà không nhận được snapshot ADC mới.
    FAULT_ID_ADC_TIMEOUT,
    // Tự kiểm tra khi cấp nguồn thất bại; do kết quả tự kiểm tra BMS gửi lên, giữ đến khi reset hệ thống.
    FAULT_ID_SELF_CHECK,
    // Bộ đếm lỗi CAN cao, truyền thông đang ở trạng thái suy giảm.
    FAULT_ID_CAN_DEGRADED,
    // Bộ điều khiển CAN đã vào trạng thái Bus-Off.
    FAULT_ID_CAN_BUS_OFF,

    // Số lượng mục lỗi đang được kích hoạt hiện tại, đồng thời làm độ dài mảng.
    FAULT_ID_COUNT
} FaultId_t;

/*
 * Trạng thái vòng đời của một lỗi đơn lẻ.
 * CLEAR -> PENDING -> CONFIRMED -> RECOVERY_PENDING -> xóa thủ công/tự động.
 * CLEAR - Hiện không có lỗi được xác nhận
 * PENDING - Điều kiện đã bất thường, nhưng thời gian duy trì chưa đủ
 * CONFIRMED - Thời gian duy trì đạt yêu cầu, lỗi được xác nhận chính thức
 * RECOVERY_PENDING - Điều kiện lỗi đã biến mất, đang xác nhận quá trình khôi phục có ổn định hay không
 */
typedef enum
{
    // Hiện không có lỗi được xác nhận, cũng không có bộ đếm xác nhận nào đang chạy.
    FAULT_STATE_CLEAR = 0,
    // Điều kiện kích hoạt đã xuất hiện, đang tích lũy thời gian xác nhận lỗi.
    FAULT_STATE_PENDING,
    // Điều kiện kích hoạt đã duy trì đủ thời gian quy định, lỗi được xác nhận chính thức và giữ hiệu lực.
    FAULT_STATE_CONFIRMED,
    // Đã thỏa mãn ngưỡng khôi phục, đang tích lũy thời gian ổn định khôi phục.
    FAULT_STATE_RECOVERY_PENDING
} FaultState_t;

// Mức độ lỗi quyết định máy trạng thái có bắt buộc phải chuyển vào BMS_FAULT hay không.
typedef enum
{
    // Lỗi cảnh báo: ghi nhận và báo cáo, nhưng thông thường vẫn cho phép tiếp tục giám sát cục bộ.
    FAULT_LEVEL_WARNING = 0,
    // Lỗi nghiêm trọng: yêu cầu máy trạng thái chuyển vào BMS_FAULT và thực hiện báo động cục bộ.
    FAULT_LEVEL_CRITICAL
} FaultLevel_t;

/* Lưu thông tin vận hành của một loại lỗi trong lần cấp nguồn này.
 * Lưu riêng giai đoạn hiện tại mà một loại lỗi đã đi tới, cùng thông tin định thời và thống kê liên quan.
 * Lưu trạng thái vòng đời, thời điểm vào trạng thái, thời gian xác nhận, thời gian khôi phục, số lần xảy ra và cờ sẵn sàng khôi phục.
 */
typedef struct
{
    // Trạng thái vòng đời hiện tại.
    FaultState_t state;
    // Thời gian hệ thống (ms) lúc bước vào trạng thái hiện tại, dùng để tính hiệu thời gian không dấu.
    uint32_t state_since_ms;
    // Thời gian điều kiện kích hoạt đã tồn tại liên tục, chủ yếu dùng để theo dõi trên Watch và chẩn đoán.
    uint32_t pending_ms;
    // Thời gian điều kiện khôi phục đã thỏa mãn liên tục, chủ yếu dùng để theo dõi trên Watch và chẩn đoán.
    uint32_t recovery_ms;
    // Số lần lỗi này được xác nhận chính thức từ sau khi cấp nguồn lần này, giữ nguyên khi đạt giá trị tối đa.
    uint16_t occurrence_count;
    // 1 biểu thị điều kiện khôi phục đã ổn định, nhưng lỗi được chốt thủ công vẫn cần nhận yêu cầu xóa.
    uint8_t recovery_ready;
} FaultItem_t; // Mục lỗi

/*
 * Snapshot hoàn chỉnh của trình quản lý lỗi.
 * items[] lưu trạng thái chi tiết của từng loại lỗi;
 * các mask cung cấp cho máy trạng thái, CAN và nhật ký sử dụng nhanh.
 */
typedef struct
{
    // Mỗi loại lỗi có một mục vận hành độc lập, bộ đếm thời gian không ghi đè lẫn nhau.
    FaultItem_t items[FAULT_ID_COUNT];

    // Toàn bộ các lỗi đã xác nhận và chưa được xóa, không bao gồm điều kiện mới ở trạng thái PENDING. (1: lỗi hiện vẫn tồn tại)
    uint16_t active_mask;

    // Các lỗi đã xác nhận và bắt buộc phải qua quy trình khôi phục thủ công để xóa. (1: lỗi đã bị chốt, bắt buộc nhấn phím để xóa)
    uint16_t latched_mask;

    // Các lỗi trong active_mask có mức độ là CRITICAL, dùng để quyết định có vào BMS_FAULT hay không. (1: đây là lỗi nghiêm trọng, hệ thống phải giữ FAULT)
    uint16_t critical_mask;

    // Các lỗi đã thỏa mãn liên tục điều kiện khôi phục và hiện được phép xóa. (1: cho phép xóa; 0: không cho phép xóa)
    uint16_t recovery_ready_mask;

    // Chỉ được đặt bit ở chu kỳ lỗi được xác nhận lần đầu tiên, dùng để chống trùng lặp cho sự kiện CAN và nhật ký Flash.
    uint16_t newly_confirmed_mask;

    // Chỉ được đặt bit ở chu kỳ lỗi thực sự bị xóa, dùng để ghi nhận sự kiện khôi phục.
    uint16_t newly_cleared_mask;
} FaultManagerSnapshot_t;

// Gọi một lần khi cấp nguồn: xóa sạch toàn bộ trạng thái lỗi, định thời, bộ đếm và mặt nạ tổng hợp.
void FaultManager_Init(void);

// Bắt đầu một vòng đánh giá hoàn chỉnh: xóa các mặt nạ sự kiện "mới xác nhận / mới xóa" chỉ duy trì trong một chu kỳ.
void FaultManager_BeginEvaluation(void);

/*
 * Cập nhật vòng đời chung của một loại lỗi.
 * trip_condition bằng 1 biểu thị hiện đang thỏa mãn điều kiện kích hoạt; recovery_condition bằng 1 biểu thị thỏa mãn điều kiện khôi phục.
 * Hai điều kiện do nguồn dữ liệu cụ thể tạo ra, hàm này chỉ phụ trách xác nhận theo thời gian và chuyển đổi trạng thái.
 */
void FaultManager_UpdateCondition(FaultId_t id,
                                  uint8_t trip_condition,
                                  uint8_t recovery_condition,
                                  uint32_t now_ms);

// Kết thúc một vòng đánh giá hoàn chỉnh: tạo lại các mặt nạ đang kích hoạt, đã chốt, nghiêm trọng và sẵn sàng khôi phục dựa trên toàn bộ items[].
void FaultManager_EndEvaluation(void);

/*
 * Đánh giá một khung snapshot Sensor.
 * sample_available biểu thị bên gọi có lấy được snapshot hay không; sample có thể rỗng (NULL); now_ms là thời gian hệ thống hiện tại.
 * Hàm này thực hiện tạo điều kiện cho điện áp, nhiệt độ, NTC, tính nhất quán dữ liệu và quá thời gian ADC.
 */
void FaultManager_UpdateSensor(const SensorSample_t *sample,
                               uint8_t sample_available,
                               uint32_t now_ms);

/*
 * Yêu cầu xóa thủ công các bit lỗi được chỉ định.
 * Chỉ những bit đồng thời nằm trong latched_mask và recovery_ready_mask mới thực sự được xóa; trả về mặt nạ đã xóa thực tế.
 */
uint16_t FaultManager_RequestManualClear(uint16_t requested_mask);

// Sao chép trình quản lý nội bộ cho bên gọi; thành công trả về 1, con trỏ rỗng trả về 0.
uint8_t FaultManager_GetSnapshot(FaultManagerSnapshot_t *snapshot);

/*
 * Thiết lập hoặc đọc mặt nạ chèn lỗi thử nghiệm.
 * Đặt bằng 0 biểu thị hủy chèn lỗi; các bit không được phép trong đầu vào sẽ tự động bị lọc bỏ.
 */
void FaultManager_SetInjectionMask(uint16_t mask);
uint16_t FaultManager_GetInjectionMask(void);

#endif
