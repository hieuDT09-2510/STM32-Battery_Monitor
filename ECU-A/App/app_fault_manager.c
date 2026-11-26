/*
 * Tên tệp: app_fault_manager.c
 * Tên mô-đun: Trình quản lý vòng đời lỗi thống nhất
 *
 * I. Mô-đun này giải quyết vấn đề gì
 * ------------------------------------------------------------
 * ADC chỉ có thể cho biết "tại khoảnh khắc này đo được bao nhiêu Vôn, bao nhiêu độ C". Nếu dùng trực tiếp
 * kết quả lấy mẫu một lần để quyết định lỗi, nhiễu điện hoặc dao động nhẹ quanh ngưỡng sẽ làm lỗi lặp đi lặp lại việc xuất hiện rồi biến mất.
 *
 * Mô-đun này chuyển đổi một kết quả đo thành trạng thái lỗi có quy tắc xác nhận theo thời gian, chốt (latch) và khôi phục:
 *
 *   Bình thường (CLEAR)
 *       ↓ Điều kiện bất thường
 *   Chờ xác nhận (PENDING)
 *       ↓ Bất thường liên tục đạt confirm_ms
 *   Đã xác nhận (CONFIRMED)
 *       ↓ Giá trị đi vào vùng khôi phục
 *   Chờ khôi phục ổn định (RECOVERY_PENDING)
 *       ↓ Ổn định đạt recovery_confirm_ms
 *   Tự động xóa, hoặc chờ nhấn phím thủ công để xóa
 *
 * II. Cấu trúc ba lớp bên trong tệp
 * ------------------------------------------------------------
 * 1. g_fault_config[]: Lưu quy tắc cố định của từng loại lỗi, ví dụ thời gian xác nhận và mức độ lỗi;
 * 2. FaultManager_UpdateCondition(): Một máy trạng thái vòng đời dùng chung cho mọi lỗi;
 * 3. FaultManager_UpdateSensor(): Chuyển đổi các đại lượng vật lý như điện áp, nhiệt độ thành điều kiện 0/1,
 *    rồi giao cho máy trạng thái vòng đời dùng chung xử lý.
 *
 * III. Mô-đun này xuất ra những gì
 * ------------------------------------------------------------
 * active_mask: Toàn bộ các lỗi hiện đã xác nhận và chưa được xóa;
 * latched_mask: Các lỗi cần xóa thủ công;
 * critical_mask: Các lỗi nghiêm trọng sẽ yêu cầu BMS chuyển vào trạng thái FAULT;
 * recovery_ready_mask: Các lỗi có điều kiện khôi phục đã ổn định, hiện được phép xóa;
 * newly_confirmed_mask: Chỉ xuất hiện ở chu kỳ lỗi vừa được xác nhận, dùng để tạo sự kiện;
 * newly_cleared_mask: Chỉ xuất hiện ở chu kỳ lỗi vừa được xóa, dùng để tạo sự kiện.
 *
 * IV. Mô-đun này không phụ trách những gì
 * ------------------------------------------------------------
 * Mô-đun này không đọc thanh ghi ADC, không điều khiển còi báo hay LED, không gửi CAN và cũng không ghi Flash.
 * Sau khi BmsTask đọc kết quả của mô-đun này mới quyết định chuyển trạng thái, báo động, thông báo CAN và ghi nhật ký.
 *
 * V. Luồng dữ liệu trong toàn bộ dự án
 * ------------------------------------------------------------
 * TIM3 kích hoạt ADC và DMA mỗi 1ms
 *        ↓
 * SensorTask quy đổi và phát hành snapshot SensorSample_t
 *        ↓
 * BmsTask mỗi 10ms gọi FaultManager_UpdateSensor()
 *        ↓
 * Tạo điều kiện kích hoạt và điều kiện khôi phục cho các lỗi OV, UV, OT, v.v.
 *        ↓
 * FaultManager_UpdateCondition() cập nhật vòng đời của từng loại lỗi
 *        ↓
 * FaultManager_EndEvaluation() tạo các mặt nạ tổng hợp
 *        ↓
 * BmsTask dựa theo snapshot để vào FAULT, thông báo CanTask hoặc tạo sự kiện nhật ký Flash
 *
 * VI. Chèn lỗi thử nghiệm (Test Injection)
 * ------------------------------------------------------------
 * CLI có thể ép chỉ định "điều kiện kích hoạt" của các lỗi thuộc nhóm cảm biến thành 1, nhưng không sửa giá trị ADC thật và
 * SensorSample_t. Nhờ vậy có thể kiểm chứng đầy đủ quy trình xử lý lỗi mà vẫn giữ nguyên hiện trường đo thực tế.
 */

#include "app_fault_manager.h"

#include "FreeRTOS.h"
#include "task.h"

/*
 * FaultConfig_t: Cấu hình cố định của một loại lỗi.
 *
 * Các trường này mô tả "quy tắc", không thay đổi trong lúc chạy; còn một lỗi cụ thể hiện đang ở trạng thái nào
 * được lưu trong g_fault_manager.items[].
 *
 * Lấy ví dụ quá áp:
 * mask=FAULT_MASK_OV          biểu thị quá áp chiếm bit0 của mã lỗi;
 * confirm_ms=500              biểu thị quá áp liên tục 500ms mới xác nhận chính thức;
 * recovery_confirm_ms=1000    biểu thị điều kiện khôi phục liên tục 1000ms mới cho phép xóa;
 * level=FAULT_LEVEL_CRITICAL  biểu thị nó sẽ yêu cầu BMS vào FAULT;
 * manual_clear=1              biểu thị sau khi khôi phục ổn định vẫn cần nhấn phím khôi phục.
 */
typedef struct
{
    // Vị trí bit mà lỗi này chiếm trong tổng mã lỗi 16-bit.
    uint16_t mask;
    // Điều kiện kích hoạt phải thỏa mãn liên tục bao lâu mới xác nhận chính thức, đơn vị ms; 0 biểu thị xác nhận ngay.
    uint16_t confirm_ms;
    // Điều kiện khôi phục phải thỏa mãn liên tục bao lâu mới cho phép xóa, đơn vị ms.
    uint16_t recovery_confirm_ms;
    // WARNING chỉ báo cáo, CRITICAL yêu cầu máy trạng thái chuyển vào trạng thái lỗi.
    FaultLevel_t level;
    // 1 biểu thị sau khi khôi phục ổn định vẫn cần yêu cầu xóa thủ công; 0 biểu thị cho phép tự động xóa.
    uint8_t manual_clear;
} FaultConfig_t;

/*
 * Bảng cấu hình của mười loại lỗi.
 *
 * Chỉ số mảng phải khớp nghiêm ngặt với thứ tự enum FaultId_t: items[id], cấu hình[id] và tên lỗi
 * đều ánh xạ qua cùng một id. Nếu thay đổi thứ tự FaultId_t thì bắt buộc phải điều chỉnh đồng thời bảng này.
 *
 * Các lỗi có confirm_ms bằng 0 sẽ xác nhận ngay khi điều kiện xuất hiện, ví dụ quá thời gian ADC và CAN Bus-Off;
 * Lỗi WARNING chỉ ghi nhận và báo cáo, lỗi CRITICAL sẽ tham gia đánh giá trạng thái lỗi BMS.
 * Các ngưỡng và thời gian hiện tại dùng để kiểm chứng nguyên mẫu đồ án tốt nghiệp, không đại diện cho hiệu chuẩn an toàn pin sản xuất hàng loạt.
 */
static const FaultConfig_t g_fault_config[FAULT_ID_COUNT] =
{
    // Bit lỗi              T/g xác nhận T/g khôi phục Mức độ              Xóa thủ công
    // Pin quá áp: bất thường liên tục 500ms để xác nhận, sau khi khôi phục ổn định cần xóa thủ công.
    {FAULT_MASK_OV,          500U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    // Pin thấp áp: bất thường liên tục 500ms để xác nhận, sau khi khôi phục ổn định cần xóa thủ công.
    {FAULT_MASK_UV,          500U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    // Pin quá nhiệt: bất thường liên tục 500ms để xác nhận, sau khi khôi phục ổn định cần xóa thủ công.
    {FAULT_MASK_OT,          500U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    // NTC hở mạch: kết quả phân loại của Sensor duy trì 500ms rồi xác nhận.
    {FAULT_MASK_NTC_OPEN,    500U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    // NTC ngắn mạch: kết quả phân loại của Sensor duy trì 500ms rồi xác nhận.
    {FAULT_MASK_NTC_SHORT,   500U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    // Các trường trong snapshot mâu thuẫn: xác nhận nhanh hơn lỗi ngưỡng vật lý, chỉ ra bản thân dữ liệu không đáng tin.
    {FAULT_MASK_SENSOR_DATA, 100U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    // Trong 100ms không có snapshot mới: xác nhận ngay lập tức, tránh tiếp tục dùng giá trị đo đã quá hạn.
    {FAULT_MASK_ADC_TIMEOUT,   0U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    // Tự kiểm tra khi cấp nguồn thất bại: xác nhận ngay lập tức, điều kiện do quy trình tự kiểm tra BMS cung cấp.
    {FAULT_MASK_SELF_CHECK,    0U, 1000U, FAULT_LEVEL_CRITICAL, 1U},
    // Tỷ lệ lỗi CAN tăng cao nhưng chưa ngoại tuyến: tạm coi là cảnh báo, tự động xóa sau khi khôi phục.
    {FAULT_MASK_CAN_DEGRADED,500U, 1000U, FAULT_LEVEL_WARNING,  0U},
    // Bộ điều khiển CAN vào Bus-Off: xác nhận ngay lập tức, sau khi khôi phục ổn định vẫn cần xóa thủ công.
    {FAULT_MASK_CAN_BUS_OFF,   0U, 1000U, FAULT_LEVEL_CRITICAL, 1U}
};

/*
 * g_fault_manager lưu trạng thái vận hành của mười loại lỗi và kết quả tổng hợp của vòng này.
 *
 * BmsTask là tác nhân ghi duy nhất. static hạn chế các tệp .c khác truy cập trực tiếp; các tác vụ khác phải gọi
 * FaultManager_GetSnapshot() để lấy bản sao, tránh mã bên ngoài đi vòng qua quy tắc vòng đời để sửa trạng thái.
 */
static FaultManagerSnapshot_t g_fault_manager;

/*
 * Mặt nạ chèn lỗi của CLI.
 *
 * Khi một bit bằng 1, chỉ ép trip_condition của lỗi đó thành 1 tại đầu vào UpdateCondition();
 * thời gian xác nhận, chốt và quy trình khôi phục vẫn thực hiện như bình thường. Biến được công khai để theo dõi trên Keil Watch,
 * mã nghiệp vụ nên truy cập thông qua giao diện Set/Get.
 */
volatile uint16_t g_fault_inject_mask = FAULT_MASK_NONE;

/*
 * Đưa một FaultItem_t trở về trạng thái ban đầu "không có lỗi".
 *
 * Xóa trạng thái và bộ đếm thời gian của vòng này, nhưng cố ý giữ lại occurrence_count, vì nó thống kê
 * tổng số lần lỗi đó đã được xác nhận trong lần cấp nguồn này, chứ không phải hiện tại có đang lỗi hay không.
 */
static void FaultItem_SetClear(FaultItem_t *item)
{
    item->state = FAULT_STATE_CLEAR;
    item->state_since_ms = 0U;
    item->pending_ms = 0U;
    item->recovery_ms = 0U;
    item->recovery_ready = 0U;
}

/*
 * Xác nhận chính thức một loại lỗi.
 *
 * Có hai thời điểm gọi: gọi ngay khi confirm_ms bằng 0, hoặc gọi sau khi thời gian duy trì PENDING đạt yêu cầu.
 * Ngoài việc chuyển trạng thái thành CONFIRMED, hàm còn tăng số lần xảy ra và đặt bit trong
 * newly_confirmed_mask. Việc này giúp BmsTask chỉ tạo một bản ghi nhật ký duy nhất, không ghi lặp lại mỗi 10ms trong suốt thời gian lỗi tồn tại.
 */
static void FaultItem_Confirm(FaultId_t id,
                              FaultItem_t *item,
                              uint32_t now_ms)
{
    item->state = FAULT_STATE_CONFIRMED;
    item->state_since_ms = now_ms;
    item->pending_ms = g_fault_config[id].confirm_ms;
    item->recovery_ms = 0U;
    item->recovery_ready = 0U;

    // Dùng bộ đếm bão hòa để tránh tràn vòng từ 0xFFFF về 0 sau thời gian chạy dài.
    if (item->occurrence_count < 0xFFFFU)
    {
        item->occurrence_count++;
    }

    g_fault_manager.newly_confirmed_mask |= g_fault_config[id].mask;
}

/*
 * Xóa chính thức một lỗi đã được xác nhận.
 *
 * Trước khi xóa, bên gọi phải đã kiểm tra thời gian khôi phục và điều kiện xóa thủ công. Hàm này đặt bit trong
 * newly_cleared_mask để BmsTask có thể lưu một bản ghi nhật ký "khôi phục lỗi", sau đó
 * khôi phục mục lỗi đó về CLEAR.
 */
static void FaultItem_ClearConfirmed(FaultId_t id,
                                     FaultItem_t *item)
{
    g_fault_manager.newly_cleared_mask |= g_fault_config[id].mask;
    FaultItem_SetClear(item);
}


/*
 * FaultManager_Init(): Khởi tạo trình quản lý lỗi khi cấp nguồn.
 *
 * Vị trí gọi: Trong main() trước khi tạo các tác vụ FreeRTOS.
 * Chức năng: Đưa mọi lỗi bắt đầu từ CLEAR, xóa sạch bộ đếm thời gian, số lần xảy ra, mặt nạ tổng hợp và chèn lỗi thử nghiệm.
 * Hàm này chỉ gọi ở giai đoạn khởi tạo; vòng đánh giá 10ms bình thường không được gọi lại, nếu không sẽ làm mất lỗi.
 */
void FaultManager_Init(void)
{
    uint8_t i;

    // Xóa từng mục lỗi độc lập, tránh các lỗi khác nhau dùng chung trạng thái định thời.
    for (i = 0U; i < (uint8_t)FAULT_ID_COUNT; i++)
    {
        g_fault_manager.items[i].occurrence_count = 0U;
        FaultItem_SetClear(&g_fault_manager.items[i]);
    }

    g_fault_manager.active_mask = FAULT_MASK_NONE;
    g_fault_manager.latched_mask = FAULT_MASK_NONE;
    g_fault_manager.critical_mask = FAULT_MASK_NONE;
    g_fault_manager.recovery_ready_mask = FAULT_MASK_NONE;
    g_fault_manager.newly_confirmed_mask = FAULT_MASK_NONE;
    g_fault_manager.newly_cleared_mask = FAULT_MASK_NONE;
    g_fault_inject_mask = FAULT_MASK_NONE;
}

/*
 * Bắt đầu một vòng đánh giá lỗi.
 *
 * newly_confirmed_mask và newly_cleared_mask biểu thị "sườn biến đổi mới xảy ra trong vòng này", chỉ được
 * giữ lại trong một chu kỳ BmsTask. Vì vậy trước mỗi vòng kiểm tra toàn bộ lỗi, phải xóa hai giá trị này về 0.
 * Các trạng thái duy trì như active_mask không bị xóa ở đây, chúng được tổng hợp lại trong EndEvaluation().
 */
void FaultManager_BeginEvaluation(void)
{
    /*
     * Hai mặt nạ này chỉ biểu thị "sự kiện sườn mới phát sinh trong vòng này",
     * không thể giữ lại xuyên chu kỳ như active_mask.
     */
    g_fault_manager.newly_confirmed_mask = FAULT_MASK_NONE;
    g_fault_manager.newly_cleared_mask = FAULT_MASK_NONE;
}


/*
 * FaultManager_UpdateCondition(): Máy trạng thái vòng đời dùng chung cho một lỗi đơn lẻ.
 *
 * Đầu vào:
 * id                 Loại lỗi cần cập nhật;
 * trip_condition     Hiện tại có thỏa mãn điều kiện kích hoạt lỗi hay không, 1=bất thường;
 * recovery_condition Hiện tại đã vào vùng khôi phục an toàn hay chưa, 1=thỏa mãn điều kiện khôi phục;
 * now_ms             Thời gian mili-giây hiện tại, dùng để tính thời gian duy trì điều kiện.
 *
 * trip_condition và recovery_condition không phải phép phủ định logic đơn thuần. Ví dụ với quá áp:
 * Khi điện áp > 4.20V thì trip=1; khi điện áp < 4.10V thì recovery=1; trong khoảng 4.10~4.20V cả hai đều bằng 0.
 * Vùng trễ (hysteresis) ở giữa giúp tránh việc xác nhận và khôi phục lặp đi lặp lại khi điện áp rung quanh ngưỡng.
 *
 * Ví dụ thực tế về quá áp:
 * 0ms đo được 4.25V      CLEAR→PENDING, ghi lại thời điểm bắt đầu;
 * 300ms vẫn là 4.25V     Chỉ cập nhật pending_ms, chưa phải lỗi chính thức;
 * 500ms vẫn là 4.25V     PENDING→CONFIRMED, và tạo ra một bit mới xác nhận;
 * Sau đó giảm xuống 4.05V CONFIRMED→RECOVERY_PENDING;
 * 4.05V giữ trong 1000ms recovery_ready=1, nhưng lỗi thủ công vẫn duy trì hoạt động;
 * Người dùng nhấn phím khôi phục  RequestManualClear() mới xóa nó.
 *
 * Đầu ra không dùng giá trị trả về, mà cập nhật g_fault_manager.items[id] cùng bit mới xác nhận/mới xóa.
 */
void FaultManager_UpdateCondition(FaultId_t id,
                                  uint8_t trip_condition,   // Biểu thị hiện có thỏa mãn điều kiện kích hoạt không: 1: hiện bất thường; 0: hiện không thỏa mãn điều kiện kích hoạt
                                  uint8_t recovery_condition, // Biểu thị hiện có thỏa mãn điều kiện khôi phục không
                                  uint32_t now_ms)
{
    FaultItem_t *item;
    const FaultConfig_t *config;
    uint32_t elapsed_ms;

    // Chống ID không hợp lệ gây vượt biên mảng items[] và bảng cấu hình.
    if ((uint32_t)id >= (uint32_t)FAULT_ID_COUNT)
    {
        return;
    }

    item = &g_fault_manager.items[id];
    config = &g_fault_config[id];

    /*
     * Khi chèn lỗi trúng lỗi hiện tại, ép điều kiện kích hoạt thành 1, điều kiện khôi phục thành 0.
     * Vòng đời phía sau vẫn thực hiện thời gian xác nhận, chốt và quy trình khôi phục như bình thường.
     */
    if ((g_fault_inject_mask & config->mask) != 0U)
    {
        trip_condition = 1U;
        recovery_condition = 0U;
    }

    // Chuẩn hóa mọi giá trị khác 0 do bên gọi truyền vào thành 1, phía sau chỉ xử lý rõ ràng 0/1.
    trip_condition = (trip_condition != 0U) ? 1U : 0U;
    recovery_condition = (recovery_condition != 0U) ? 1U : 0U;

    switch (item->state)
    {
        /* Hiện không có lỗi đã xác nhận, chờ bất thường xuất hiện lần đầu. */
        case FAULT_STATE_CLEAR:
            item->pending_ms = 0U;
            item->recovery_ms = 0U;
            item->recovery_ready = 0U;

            // Lần đầu phát hiện bất thường: nếu cần thời gian xác nhận thì vào PENDING, ngược lại xác nhận ngay.
            if (trip_condition != 0U)
            {
                if (config->confirm_ms == 0U)
                {
                    FaultItem_Confirm(id, item, now_ms);
                }
                else
                {
                    item->state = FAULT_STATE_PENDING;
                    item->state_since_ms = now_ms;
                }
            }
            break;

        /* Bất thường đã xuất hiện nhưng vẫn đang chờ hết thời gian xác nhận, dùng để lọc nhiễu thoáng qua. */
        case FAULT_STATE_PENDING:
            if (trip_condition == 0U)
            {
                /*
                 * Trong thời gian xác nhận mà điều kiện biến mất chứng tỏ chỉ là thoáng qua,
                 * không tính là một lỗi đã xác nhận.
                 */
                FaultItem_SetClear(item);
            }
            else
            {
                /*
                 * Dùng phép trừ không dấu để tính thời gian duy trì.
                 * Ngay cả khi Tick 32-bit bị tràn vòng, khoảng thời gian ngắn vẫn ra hiệu số chính xác.
                 */
                elapsed_ms = (uint32_t)(now_ms - item->state_since_ms);
                item->pending_ms = elapsed_ms;

                if (elapsed_ms >= config->confirm_ms)
                {
                    FaultItem_Confirm(id, item, now_ms);
                }
            }
            break;

        /* Lỗi đã có hiệu lực; chỉ sau khi vào vùng khôi phục mới bắt đầu tính thời gian khôi phục. */
        case FAULT_STATE_CONFIRMED:
            item->pending_ms = config->confirm_ms;
            item->recovery_ms = 0U;
            item->recovery_ready = 0U;

            // Chỉ khi đạt ngưỡng khôi phục độc lập mới bắt đầu tích lũy thời gian ổn định khôi phục.
            if (recovery_condition != 0U)
            {
                item->state = FAULT_STATE_RECOVERY_PENDING;
                item->state_since_ms = now_ms;

                if (config->recovery_confirm_ms == 0U)
                {
                    item->recovery_ready = 1U;
                }
            }
            break;

        /* Giá trị đã vào vùng khôi phục, đang xác nhận trạng thái khôi phục này có duy trì ổn định được không. */
        case FAULT_STATE_RECOVERY_PENDING:
            /*
             * Lỗi cũ tái xuất hiện hoặc mất điều kiện khôi phục,
             * đều phải quay về CONFIRMED để tích lũy lại thời gian khôi phục từ đầu.
             */
            if ((trip_condition != 0U) ||
                (recovery_condition == 0U))
            {
                item->state = FAULT_STATE_CONFIRMED;
                item->state_since_ms = now_ms;
                item->recovery_ms = 0U;
                item->recovery_ready = 0U;
            }
            else
            {
                elapsed_ms = (uint32_t)(now_ms - item->state_since_ms);
                item->recovery_ms = elapsed_ms;

                // Sau khi thời gian khôi phục đạt yêu cầu, lỗi xóa thủ công chỉ đánh dấu ready, không tự ý xóa.
                if (elapsed_ms >= config->recovery_confirm_ms)
                {
                    item->recovery_ready = 1U;

                    if (config->manual_clear == 0U)
                    {
                        FaultItem_ClearConfirmed(id, item);
                    }
                }
            }
            break;

        default:
            /*
             * Khi giá trị trạng thái bị hỏng thì quay về CLEAR.
             * Hệ thống hoàn chỉnh sau này sẽ đồng thời báo cáo FAULT_INTERNAL.
             */
            FaultItem_SetClear(item);
            break;
    }
}

/*
 * FaultManager_EndEvaluation(): Tổng hợp mười mục lỗi độc lập thành mã lỗi xuất ra ngoài.
 *
 * items[] phù hợp để xem trạng thái chi tiết của từng loại lỗi, nhưng máy trạng thái, CAN và CLI cần biết nhanh
 * "hiện tại đang có những lỗi nào". Do đó hàm này duyệt toàn bộ lỗi, ghép các bit tương ứng thành bốn mặt nạ.
 *
 * Ví dụ quá áp và quá nhiệt có thể tồn tại đồng thời, khi đó:
 * active_mask = FAULT_MASK_OV | FAULT_MASK_OT = 0x0001 | 0x0004 = 0x0005.
 *
 * Mỗi vòng đều xóa về 0 trước rồi tạo lại, đảm bảo bit lỗi đã xóa sẽ không còn sót lại. PENDING chưa phải
 * lỗi chính thức nên không đi vào bất kỳ mặt nạ lỗi đang kích hoạt nào xuất ra ngoài.
 */

void FaultManager_EndEvaluation(void)
{
    uint8_t i;  // Chỉ số lỗi đang kiểm tra hiện tại
    uint16_t mask;  // Mặt nạ tương ứng với lỗi hiện tại
    FaultItem_t *item; // Trạng thái vận hành của lỗi hiện tại
    const FaultConfig_t *config; // Cấu hình cố định của lỗi hiện tại

    /*
     * Mặt nạ tổng hợp mỗi vòng đều được tạo lại từ items[], thay vì chỉ đặt bit mà không xóa bit.
     * Như vậy sau khi xóa lỗi sẽ không bị sót bit cũ trong mã lỗi tổng.
     */
    g_fault_manager.active_mask = FAULT_MASK_NONE;
    g_fault_manager.latched_mask = FAULT_MASK_NONE;
    g_fault_manager.critical_mask = FAULT_MASK_NONE;
    g_fault_manager.recovery_ready_mask = FAULT_MASK_NONE;

    for (i = 0U; i < (uint8_t)FAULT_ID_COUNT; i++)
    {
        item = &g_fault_manager.items[i];
        config = &g_fault_config[i];
        mask = config->mask;

        // PENDING chưa được xác nhận, không báo ra ngoài là lỗi đang kích hoạt.
        if ((item->state == FAULT_STATE_CONFIRMED) ||
            (item->state == FAULT_STATE_RECOVERY_PENDING))
        {
            g_fault_manager.active_mask |= mask;

            if (config->manual_clear != 0U)
            {
                g_fault_manager.latched_mask |= mask;
            }

            if (config->level == FAULT_LEVEL_CRITICAL)
            {
                g_fault_manager.critical_mask |= mask;
            }

            if (item->recovery_ready != 0U)
            {
                g_fault_manager.recovery_ready_mask |= mask;
            }
        }
    }
}

/*
 * FaultManager_IsSampleConsistent(): Kiểm tra nội bộ một khung snapshot có tự nhất quán (nhất quán logic) không.
 *
 * Hàm kiểm tra "cấu trúc dữ liệu có đáng tin không", không kiểm tra điện áp hay nhiệt độ có vượt ngưỡng an toàn không.
 * Ví dụ điện áp 4.30V tuy sẽ kích hoạt OV, nhưng chỉ cần ADC và cờ hợp lệ đúng thì nó vẫn là một khung
 * dữ liệu nhất quán, không nên đồng thời báo SENSOR_DATA.
 *
 * Khi NTC được phân loại là hở mạch hoặc ngắn mạch, temperature_valid=0 là quan hệ đúng kỳ vọng; chỉ khi phân loại NTC
 * bình thường mà không có nhiệt độ hợp lệ, hoặc giá trị ADC thô vượt giới hạn cùng các trường hợp mâu thuẫn khác mới kết luận là không nhất quán.
 */
static uint8_t FaultManager_IsSampleConsistent(const SensorSample_t *sample)
{
    // Không có số thứ tự hợp lệ, ADC vượt phạm vi 12-bit hoặc quy đổi điện áp không hợp lệ đều thuộc snapshot không đáng tin.
    if ((sample == 0) ||
        (sample->sequence == 0U) ||
        (sample->adc_pa0 > 4095U) ||
        (sample->adc_pa1 > 4095U) ||
        (sample->voltage_valid == 0U))
    {
        return 0U;
    }

    // Khi NTC được phân loại bình thường thì bắt buộc phải đồng thời có kết quả nhiệt độ hợp lệ.
    if (sample->ntc_input == NTC_INPUT_NORMAL)
    {
        return (sample->temperature_valid != 0U) ? 1U : 0U;
    }

    // Khi NTC đã được phân loại là hở/ngắn mạch thì nhiệt độ không hợp lệ là kết quả hợp lý, không báo trùng SENSOR_DATA.
    if ((sample->ntc_input == NTC_INPUT_OPEN_SUSPECT) ||
        (sample->ntc_input == NTC_INPUT_SHORT_SUSPECT))
    {
        return (sample->temperature_valid == 0U) ? 1U : 0U;
    }

    return 0U;
}

/*
 * FaultManager_UpdateSensor(): Chuyển đổi một khung snapshot cảm biến thành các điều kiện lỗi.
 *
 * Hàm này là lớp thích ứng giữa mô-đun Sensor và máy trạng thái lỗi dùng chung. Thứ tự thực thi như sau:
 *
 * Snapshot SensorSample_t
 *      ↓ Có tồn tại và sequence hợp lệ không
 * present
 *      ↓ Dấu thời gian cách hiện tại có nhỏ hơn 100ms không
 * fresh
 *      ↓ ADC, cờ hợp lệ và phân loại NTC có nhất quán với nhau không
 * consistent
 *      ↓ Tạo điều kiện boolean trip/recovery cho từng loại lỗi
 * UpdateCondition() lần lượt cập nhật OV, UV, OT, NTC, bất thường dữ liệu và quá thời gian ADC
 *      ↓
 * EndEvaluation() tạo mặt nạ tổng hợp
 *
 * Vì sao phải đánh giá tính khả dụng của dữ liệu trước: Nếu ADC đã dừng, khung cuối cùng đúng lúc là 4.30V, không thể
 * dùng mãi khung dữ liệu cũ này để tiếp tục đánh giá quá áp; lúc này phải báo ADC_TIMEOUT. Khi đại lượng vật lý không khả dụng,
 * OV, UV, OT không tiếp tục xác nhận, cũng không dựa vào giá trị không hợp lệ để thực hiện khôi phục.
 *
 * Hàm này chỉ cập nhật bảy loại lỗi liên quan đến cảm biến. Điều kiện của SELF_CHECK, CAN_DEGRADED và CAN_BUS_OFF
 * do mô-đun BMS lấy từ trạng thái tự kiểm tra và driver CAN rồi mới gọi UpdateCondition() để cập nhật.
 * Cuối cùng mô-đun BMS sẽ gọi lại EndEvaluation() để tổng hợp thống nhất cả mười loại lỗi.
 */
void FaultManager_UpdateSensor(const SensorSample_t *sample,    // Địa chỉ snapshot cảm biến
                               uint8_t sample_available,    // Đã nhận thành công một khung snapshot hay chưa
                               uint32_t now_ms)
{
    /* Các biến trung gian tách biệt 4 tầng ý nghĩa: "tồn tại, mới, nhất quán, dùng được cho đánh giá vật lý". */
    uint8_t present;    // Có tồn tại một khung dữ liệu đã phát hành hay không?
    uint8_t fresh;  // Khung dữ liệu này có còn mới không? (tuổi < 100ms)
    uint8_t consistent; // Các trường trong snapshot có khớp logic với nhau không?
    uint8_t voltage_usable; // Điện áp có dùng được cho đánh giá OV/UV không? (mới và đã quy đổi)
    uint8_t temperature_usable; // Nhiệt độ có dùng được cho đánh giá OT không? (mới, đã quy đổi và phân loại NTC bình thường)

    // present chỉ chứng minh tồn tại một khung snapshot mang số thứ tự hợp lệ.
    present = ((sample_available != 0U) &&
               (sample != 0) &&
               (sample->sequence != 0U)) ? 1U : 0U;

    // Tuổi snapshot đạt 100ms thì coi như quá thời gian; cách viết hiệu thời gian tương thích tràn vòng Tick.
    fresh = ((present != 0U) &&
             ((uint32_t)(now_ms - sample->timestamp_ms) < 100U)) ? 1U : 0U;

    // Chỉ snapshot còn mới mới kiểm tra tính nhất quán các trường; dữ liệu quá hạn giao thống nhất cho ADC_TIMEOUT xử lý.
    consistent = (fresh != 0U) ?
                 FaultManager_IsSampleConsistent(sample) : 0U;

    // OV và UV bắt buộc dùng điện áp mới và đã quy đổi thành công.
    voltage_usable = ((fresh != 0U) &&
                      (sample->voltage_valid != 0U)) ? 1U : 0U;

    // OT bắt buộc dùng nhiệt độ mới, phân loại bình thường và đã quy đổi thành công.
    temperature_usable = ((fresh != 0U) &&
                          (sample->temperature_valid != 0U) &&
                          (sample->ntc_input == NTC_INPUT_NORMAL)) ? 1U : 0U;

    // Một khung snapshot tương ứng với một chu kỳ đánh giá hoàn chỉnh.
    FaultManager_BeginEvaluation();

    FaultManager_UpdateCondition(
        FAULT_ID_OV,
        // Ngưỡng kích hoạt 4.20V, ngưỡng khôi phục 4.10V, khoảng 0.10V ở giữa tạo vùng trễ.
        (uint8_t)((voltage_usable != 0U) &&
                  (sample->cell_voltage_v > 4.20f)),
        (uint8_t)((voltage_usable != 0U) &&
                  (sample->cell_voltage_v < 4.10f)),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_UV,
        // Ngưỡng kích hoạt 3.00V, ngưỡng khôi phục 3.10V, khoảng 0.10V ở giữa tạo vùng trễ.
        (uint8_t)((voltage_usable != 0U) &&
                  (sample->cell_voltage_v < 3.00f)),
        (uint8_t)((voltage_usable != 0U) &&
                  (sample->cell_voltage_v > 3.10f)),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_OT,
        // Ngưỡng kích hoạt 60℃, ngưỡng khôi phục 55℃, khoảng 5℃ ở giữa tạo vùng trễ.
        (uint8_t)((temperature_usable != 0U) &&
                  (sample->temperature_c > 60.0f)),
        (uint8_t)((temperature_usable != 0U) &&
                  (sample->temperature_c < 55.0f)),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_NTC_OPEN,
        // NTC hở mạch kích hoạt theo kết quả phân loại của Sensor; khôi phục yêu cầu NTC bình thường và nhiệt độ khả dụng.
        (uint8_t)((fresh != 0U) &&
                  (sample->ntc_input == NTC_INPUT_OPEN_SUSPECT)),
        (uint8_t)(temperature_usable != 0U),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_NTC_SHORT,
        // NTC ngắn mạch kích hoạt theo kết quả phân loại của Sensor; khôi phục yêu cầu NTC bình thường và nhiệt độ khả dụng.
        (uint8_t)((fresh != 0U) &&
                  (sample->ntc_input == NTC_INPUT_SHORT_SUSPECT)),
        (uint8_t)(temperature_usable != 0U),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_SENSOR_DATA,
        // Chỉ đánh giá mâu thuẫn trường khi snapshot còn mới, tránh báo trùng với quá thời gian ADC.
        (uint8_t)((fresh != 0U) && (consistent == 0U)),
        (uint8_t)((fresh != 0U) && (consistent != 0U)),
        now_ms);

    FaultManager_UpdateCondition(
        FAULT_ID_ADC_TIMEOUT,
        // Khi không có snapshot hoặc tuổi snapshot đạt 100ms thì xác nhận ngay quá thời gian ADC.
        (uint8_t)(fresh == 0U), // fresh = 0: không có snapshot, hoặc tuổi snapshot lớn hơn hoặc bằng 100ms
        (uint8_t)(fresh != 0U), // fresh = 1: có snapshot, và tuổi snapshot nhỏ hơn 100ms
        now_ms);

    // Sau khi cập nhật xong mọi lỗi cảm biến, thống nhất dựng lại các mặt nạ xuất ra ngoài.
    FaultManager_EndEvaluation();
}



/*
 * FaultManager_RequestManualClear(): Xử lý một yêu cầu khôi phục thủ công.
 *
 * requested_mask biểu thị người dùng muốn xóa những lỗi nào, không đồng nghĩa các lỗi đó chắc chắn sẽ bị xóa.
 * Chỉ các bit thỏa mãn đồng thời cả ba điều kiện sau mới vào eligible_mask:
 * 1. Lần này thực sự có yêu cầu xóa;
 * 2. Lỗi đó được cấu hình là chốt thủ công;
 * 3. Điều kiện khôi phục đã ổn định đủ thời gian quy định.
 *
 * Ví dụ: Yêu cầu xóa OV và UV, tức requested_mask=0x0003; nếu chỉ có OV sẵn sàng khôi phục,
 * eligible_mask cuối cùng chỉ có 0x0001, giá trị trả về cũng là 0x0001, UV vẫn tiếp tục hoạt động.
 */
uint16_t FaultManager_RequestManualClear(uint16_t requested_mask)
{
    uint8_t i;
    uint16_t eligible_mask;
    uint16_t cleared_mask = FAULT_MASK_NONE;

    /*
     * Ba điều kiện bắt buộc phải thỏa mãn đồng thời: người dùng yêu cầu, lỗi đã chốt, khôi phục đã ổn định.
     * Do đó nhấn phím không thể xóa lỗi đang ở trong điều kiện bất thường.
     */
    eligible_mask = requested_mask &
                    g_fault_manager.latched_mask &
                    g_fault_manager.recovery_ready_mask;

    for (i = 0U; i < (uint8_t)FAULT_ID_COUNT; i++)
    {
        if ((eligible_mask & g_fault_config[i].mask) != 0U)
        {
            FaultItem_ClearConfirmed((FaultId_t)i,
                                     &g_fault_manager.items[i]);
            cleared_mask |= g_fault_config[i].mask;
        }
    }

    FaultManager_EndEvaluation();
    return cleared_mask;
}


/*
 * FaultManager_GetSnapshot — Lấy một bản snapshot hoàn chỉnh, nhất quán của trình quản lý lỗi.
 *
 * BmsTask là tác nhân ghi duy nhất của trình quản lý lỗi;
 * Các chức năng CLI, CAN và chẩn đoán chỉ có thể đọc bản sao thông qua giao diện này.
 *
 * Trong lúc sao chép sử dụng vùng găng ngắn để ngăn BmsTask chiếm quyền thực thi khi đang sao chép dở struct,
 * nhờ đó tránh việc bên gọi đọc phải snapshot bị ghép từ dữ liệu cũ và mới của hai vòng khác nhau.
 */
uint8_t FaultManager_GetSnapshot(
    FaultManagerSnapshot_t *snapshot)
{
    /*
     * Bảo vệ con trỏ rỗng:
     * Khi bên gọi chưa cung cấp địa chỉ nhận hợp lệ,
     * không được phép ghi struct vào địa chỉ đó.
     */
    if (snapshot == 0)
    {
        return 0U;
    }

    /*
     * Vùng găng chỉ bao gồm một lần sao chép struct trong RAM.
     *
     * Bên trong vùng găng không được đưa vào printf, Flash, gửi CAN,
     * vTaskDelay hoặc bất kỳ thao tác nào có thể thực thi kéo dài.
     */
    taskENTER_CRITICAL();

    *snapshot = g_fault_manager;

    taskEXIT_CRITICAL();

    return 1U;
}


/*
 * Thiết lập mặt nạ chèn lỗi thử nghiệm CLI.
 *
 * FAULT_INJECT_ALLOWED_MASK sẽ lọc bỏ SELF_CHECK và các lỗi phần cứng CAN, tránh lệnh thử nghiệm
 * giả mạo trạng thái phần cứng thật. Truyền vào 0 biểu thị dừng mọi chèn lỗi; chèn lỗi chỉ thay đổi điều kiện lỗi, không thay đổi
 * điện áp và nhiệt độ thật do SensorTask phát hành.
 */
void FaultManager_SetInjectionMask(uint16_t mask)
{
    /* Lọc bỏ các bit không được phép chèn, tránh giả mạo SELF_CHECK hoặc trạng thái phần cứng CAN thật. */
    g_fault_inject_mask = mask & FAULT_INJECT_ALLOWED_MASK;
}


/* Trả về mặt nạ chèn lỗi thử nghiệm đang có hiệu lực hiện tại, dùng cho CLI hiển thị và nhật ký lỗi đánh dấu nguồn gốc. */
uint16_t FaultManager_GetInjectionMask(void)
{
    return g_fault_inject_mask;
}
