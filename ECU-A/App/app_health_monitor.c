/*
 * Tên tệp: app_health_monitor.c
 *
 * Tên mô-đun: Giám sát sức khỏe hệ thống và Watchdog độc lập
 *
 * Luồng hoạt động:
 * Chức năng trọng yếu thực thi thành công
 *       ↓
 * HealthMonitor_Report() đặt bit nhịp tim tương ứng
 *       ↓
 * Gọi HealthMonitor_Run() mỗi 100ms
 *       ↓
 * Kiểm tra cửa sổ này đã nhận đủ nhịp tim Sensor, BMS và CAN chưa
 *       ↓
 * Đủ: Làm mới IWDG
 * Thiếu: Không làm mới IWDG
 *
 * IWDG sử dụng xung nhịp LSI độc lập.
 * Ngay cả khi xung nhịp chính, TIM3 hoặc vòng lặp chính bị dừng, IWDG vẫn có thể reset hệ thống.
 *
 * Thời gian timeout điển hình của watchdog hiện tại khoảng 4 giây.
 * Tần số LSI vốn có sai số nên 4 giây là giá trị xấp xỉ, không phải định thời chính xác.
 * 
 *
 * Quy tắc đồng thời:
 * HealthMonitor_Report() được gọi bởi nhiều tác vụ FreeRTOS;
 * việc đặt bit nhịp tim cũng như thao tác "đọc xong xóa về 0" dùng vùng găng ngắn để bảo vệ,
 * tránh mất nhịp tim khi các tác vụ khác nhau sửa đổi đồng thời.
 */
#include "app_health_monitor.h"

/*
 * Bit nhịp tim Health được sửa đổi đồng thời bởi nhiều tác vụ,
 * dùng vùng găng tác vụ FreeRTOS để bảo vệ thao tác đọc-ghi kết hợp.
 */
#include "FreeRTOS.h"
#include "task.h"

#include "stm32f10x.h"


/*
 * IWDG sử dụng xung nhịp LSI khoảng 40kHz:
 *
 * 40kHz / 64 = 625Hz
 * (2499 + 1) / 625 = 4 giây
 */
#define HEALTH_IWDG_PRESCALER_DIV64    ((uint32_t)0x04U)
#define HEALTH_IWDG_RELOAD_VALUE       ((uint32_t)2499U)

/* Từ khóa IWDG. */
#define HEALTH_IWDG_KEY_WRITE_ENABLE   ((uint32_t)0x5555U)// Cho phép ghi vào thanh ghi IWDG
#define HEALTH_IWDG_KEY_RELOAD         ((uint32_t)0xAAAAU)// Nạp lại giá trị đếm IWDG (feed watchdog)
#define HEALTH_IWDG_KEY_START          ((uint32_t)0xCCCCU)// Khởi động watchdog độc lập IWDG


volatile uint8_t g_health_seen_mask = 0U; // Ghi lại các bit nhịp tim đã thấy
volatile uint8_t g_health_last_window_mask = 0U;// Ghi lại các bit nhịp tim của cửa sổ kiểm tra trước
volatile uint32_t g_health_feed_count = 0U;// Ghi lại số lần tích lũy làm mới IWDG thành công
volatile uint32_t g_health_missed_count = 0U;// Ghi lại số lần tích lũy nhịp tim không đầy đủ, không làm mới IWDG
volatile uint8_t g_health_test_stop_feed = 0U;// Công tắc kiểm thử watchdog, đặt bằng 1 để Health Monitor ngừng làm mới IWDG nhằm xác minh reset phần cứng


/* Giữ nguyên khi bộ đếm chẩn đoán đạt giá trị cực đại, tránh tràn vòng về 0 sau thời gian dài chạy. */
static void HealthMonitor_IncrementSaturated(
    volatile uint32_t *value)
{
    if (*value < 0xFFFFFFFFU)
    {
        (*value)++;
    }
}


void HealthMonitor_Init(void)
{
    g_health_seen_mask = 0U;
    g_health_last_window_mask = 0U;
    g_health_feed_count = 0U;
    g_health_missed_count = 0U;
    g_health_test_stop_feed = 0U;

    /*
     * Tạm dừng đồng thời IWDG khi trình gỡ lỗi tạm dừng CPU.
     * Tránh việc chương trình bị watchdog reset khi dừng tại breakpoint.
     */
    DBGMCU->CR |= DBGMCU_CR_DBG_IWDG_STOP;

    /*
     * Ghi 0x5555 để cho phép sửa bộ chia trước và giá trị nạp lại của IWDG.
     */
    IWDG->KR = HEALTH_IWDG_KEY_WRITE_ENABLE;

    /*
     * LSI khoảng 40kHz, sau khi chia 64 còn khoảng 625Hz.
     */
    IWDG->PR = HEALTH_IWDG_PRESCALER_DIV64;

    /*
     * Đếm 2500 lần, thời gian timeout điển hình khoảng 4 giây.
     */
    IWDG->RLR = HEALTH_IWDG_RELOAD_VALUE;

    /*
     * Nạp giá trị đếm lần đầu tiên.
     */
    IWDG->KR = HEALTH_IWDG_KEY_RELOAD;

    /*
     * Khởi động IWDG, đồng thời khởi động xung nhịp LSI cần thiết cho nó.
     *
     * Không chờ IWDG->SR xóa về 0 trước khi khởi động:
     * Việc cập nhật PR và RLR phụ thuộc xung nhịp LSI, chờ trước khi khởi động có thể bị treo vĩnh viễn.
     */
    IWDG->KR = HEALTH_IWDG_KEY_START;
}

// Báo cáo bit nhịp tim tương ứng sau khi chức năng trọng yếu thực thi thành công.
void HealthMonitor_Report(uint8_t source_mask)
{
    uint8_t valid_source_mask;
    /*
     * Chỉ chấp nhận các bit sức khỏe do hệ thống định nghĩa.
     * Ngay cả khi bên gọi truyền nhầm các bit khác cũng không làm nhiễu kết quả kiểm tra.
     */
    valid_source_mask =
        (uint8_t)(source_mask & HEALTH_REQUIRED_MASK);

    /*
     * Thao tác "đọc giá trị cũ, OR theo bit, ghi lại" không phải một lệnh bất khả phân.
     * Dùng vùng găng ngắn để tránh hai tác vụ báo cáo đồng thời ghi đè lẫn nhau.
     */
    taskENTER_CRITICAL();
    g_health_seen_mask |= valid_source_mask;
    taskEXIT_CRITICAL();

}

// Kiểm tra nhịp tim mỗi 100ms một lần và quyết định có làm mới IWDG hay không.
void HealthMonitor_Run(void)
{
    uint8_t window_mask;

    /*
    * Thao tác "lấy kết quả cửa sổ trước" và "bắt đầu cửa sổ tiếp theo" phải là một khối thống nhất.
    *
    * Nếu bị tác vụ khác báo cáo nhịp tim vào khoảng giữa lúc đọc xong và trước khi xóa về 0,
    * lệnh xóa ngay sau đó sẽ làm mất luôn nhịp tim mới vừa tới.
    */
    taskENTER_CRITICAL();

    window_mask = g_health_seen_mask;
    g_health_seen_mask = 0U;

    taskEXIT_CRITICAL();

    /*
    * last_window_mask chỉ do Health Monitor ghi,
    * dùng cho Watch quan sát nên không cần nằm trong vùng găng.
    */
    g_health_last_window_mask = window_mask;

    /*
     * Cố tình không làm mới IWDG khi công tắc kiểm thử đang bật.
     * Nó chỉ dùng để xác minh watchdog, khi vận hành bình thường phải giữ bằng 0.
     */
    if (g_health_test_stop_feed != 0U)
    {
        HealthMonitor_IncrementSaturated(
            &g_health_missed_count);
        return;
    }

    /*
    * Sau khi AND theo bit vẫn bằng đúng mặt nạ bắt buộc đầy đủ,
    * nghĩa là nhịp tim của cả ba tác vụ trọng yếu Sensor, BMS và CAN đều đã xuất hiện.
    */
    if ((window_mask & HEALTH_REQUIRED_MASK) ==
        HEALTH_REQUIRED_MASK)
    {
        IWDG->KR = HEALTH_IWDG_KEY_RELOAD;

        HealthMonitor_IncrementSaturated(
            &g_health_feed_count);
    }
    else
    {
        /*
         * Nhịp tim trong cửa sổ này không đầy đủ, không làm mới watchdog.
         * Khi cửa sổ tiếp theo khôi phục bình thường vẫn còn cơ hội làm mới lại.
         */
        HealthMonitor_IncrementSaturated(
            &g_health_missed_count);
    }
}
