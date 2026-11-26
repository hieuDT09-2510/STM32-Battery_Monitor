/*
 * Tên tệp: app_rtos.c
 *
 * Tên mô-đun: Tổ chức tác vụ FreeRTOS và điểm vào chuyển đổi
 *
 * Trách nhiệm mô-đun:
 * 1. Tập trung định nghĩa và tạo các tác vụ FreeRTOS của dự án này;
 * 2. Tập trung quản lý kích thước ngăn xếp (stack), độ ưu tiên và handle của từng tác vụ;
 * 3. Định nghĩa phương thức chạy định kỳ và phương thức block của từng tác vụ;
 * 4. Cung cấp dữ liệu chẩn đoán như số lần chạy tác vụ, mức stack còn lại tối thiểu và Heap còn lại;
 * 5. Tổ chức DiagnosticTask xử lý lệnh CLI và chẩn đoán tài nguyên hệ thống.
 *
 * Cấu trúc tác vụ hiện tại:
 *
 * HealthTask, độ ưu tiên 5:
 * Sau khi khởi động chờ trước 150ms, sau đó mỗi 100ms kiểm tra một lần
 * heartbeat của Sensor, BMS và CAN, đồng thời quyết định có làm mới IWDG hay không.
 *
 * SensorTask, độ ưu tiên 4:
 * Mỗi 1ms kiểm tra dữ liệu ADC/DMA, hoàn tất quy đổi cảm biến,
 * và phát hành snapshot cảm biến mới nhất.
 *
 * BmsTask, độ ưu tiên 3:
 * Mỗi 10ms đọc snapshot cảm biến mới nhất, chạy trình quản lý lỗi,
 * máy trạng thái BMS và xử lý sự kiện trạng thái.
 *
 * CanTask, độ ưu tiên 2:
 * Mỗi 100ms gửi trạng thái điện áp (gồm SOC) và trạng thái lỗi, mỗi 500ms gửi trạng thái nhiệt độ;
 * Khi trạng thái lỗi thay đổi, có thể gửi ngay bản tin lỗi thông qua thông báo tác vụ (task notification).
 *
 * FaultLogTask, độ ưu tiên 1:
 * Chờ block tối đa 1 giây trên FaultEvent Queue, sau khi nhận sự kiện sẽ ghi vào SPI Flash ngoài.
 *
 * DiagnosticTask, độ ưu tiên 1:
 * Mỗi 1ms quét vòng (polling) CLI, mỗi 1 giây cập nhật mức stack còn lại tối thiểu của các tác vụ và Heap còn lại.
 *
 * Luồng dữ liệu chính hiện tại:
 *
 * TIM3 kích hoạt ADC và DMA
 *      ↓
 * SensorTask quy đổi và phát hành snapshot cảm biến
 *      ↓
 * BmsTask đọc snapshot, đánh giá lỗi và chạy máy trạng thái
 *      ↓
 * ┌──────────────────┬──────────────────┐
 * ↓                  ↓                  ↓
 * Thông báo CanTask  FaultEvent Queue   Báo cáo heartbeat BMS
 * ↓                  ↓
 * Gửi bản tin CAN    FaultLogTask ghi Flash
 *
 * Sau khi SensorTask, BmsTask và CanTask hoàn thành công việc trọng yếu của mình,
 * từng tác vụ sẽ báo cáo heartbeat lên mô-đun Health.
 * HealthTask kiểm tra heartbeat định kỳ để quyết định có làm mới IWDG hay không.
 *
 * Phạm vi mô-đun:
 * Tệp này chỉ phụ trách "tác vụ do ai thực thi, khi nào thực thi, block như thế nào".
 * Các nghiệp vụ cụ thể như quy đổi cảm biến, đánh giá lỗi, đóng gói bản tin CAN và lưu trữ Flash
 * vẫn do từng mô-đun ứng dụng tương ứng hoàn thành.
 */

#include "app_rtos.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_can_service.h"
#include "app_cli.h"
#include "app_bms_state.h"
#include "app_event.h"
#include "app_fault_event.h"
/*
 * SensorTask dùng Sensor Service xử lý dữ liệu ADC,
 * dùng GetTick() để giữ cho snapshot Sensor và bộ định thời lỗi của BmsTask dùng chung một nguồn thời gian,
 * sau khi phát hành snapshot thành công sẽ báo cáo heartbeat lên Health Monitor.
 */
#include "app_sensor.h"
#include "app_soc.h"
#include "app_health_monitor.h"
#include "bsp_tim.h"

/*
 * DiagnosticTask phụ trách phân tích lệnh CLI, in qua cổng nối tiếp (UART) và chẩn đoán tài nguyên.
 *
 * Hiện cấp phát 256 phần tử stack 32-bit:
 * 256 × 4 byte = 1024 byte.
 *
 * Kiểm chứng tải (stress test) đã bao phủ các lệnh như status, fault, can stats, log và log clear,
 * mức stack còn lại tối thiểu (high watermark) vẫn còn đủ dự phòng.
 */
#define APP_RTOS_DIAGNOSTIC_STACK_WORDS       256U

/*
 * SensorTask phụ trách kiểm tra dữ liệu ADC/DMA,
 * và thực hiện quy đổi điện áp, nhiệt độ NTC cùng phép tính dấu phẩy động.
 *
 * Hiện cấp phát 256 phần tử stack 32-bit:
 * 256 × 4 byte = 1024 byte.
 *
 * Kích thước stack đã được kiểm chứng bằng dữ liệu mức stack còn lại tối thiểu
 * sau khi chạy lấy mẫu bình thường và kiểm thử lỗi.
 */
#define APP_RTOS_SENSOR_STACK_WORDS       256U

/*
 * BmsTask phụ trách chạy trình quản lý lỗi, máy trạng thái BMS và sự kiện trạng thái.
 *
 * Đường thực thi này có các struct cục bộ như FaultManagerSnapshot_t,
 * khi trạng thái thay đổi còn có thể thực hiện in chẩn đoán, nên thực tế dùng nhiều stack hơn
 * SensorTask và CanTask.
 *
 * Hiện cấp phát 256 phần tử stack 32-bit:
 * 256 × 4 byte = 1024 byte.
 */
#define APP_RTOS_BMS_STACK_WORDS          256U

/*
 * CanTask phụ trách đọc snapshot nghiệp vụ, lắp ghép bản tin CAN,
 * tính Alive Counter và CRC8, rồi gọi driver CAN để gửi.
 *
 * Hiện cấp phát 192 phần tử stack 32-bit:
 * 192 × 4 byte = 768 byte.
 */
#define APP_RTOS_CAN_STACK_WORDS          192U

/*
 * FaultLogTask phụ trách lấy sự kiện từ hàng đợi sự kiện lỗi,
 * chiếm giữ Flash Mutex, ghi vào SPI Flash ngoài rồi đọc lại để kiểm tra xác thực.
 *
 * Hiện cấp phát 192 phần tử stack 32-bit:
 * 192 × 4 byte = 768 byte.
 *
 * Các bài kiểm thử chèn lỗi, khôi phục lỗi và đọc nhật ký đã bao phủ các nhánh chính của nó.
 */
#define APP_RTOS_FAULT_LOG_STACK_WORDS    192U

/*
 * HealthTask chỉ phụ trách kiểm tra định kỳ heartbeat của các tác vụ trọng yếu và quyết định có làm mới IWDG hay không.
 *
 * Hiện cấp phát 192 phần tử stack 32-bit:
 * 192 × 4 byte = 768 byte.
 *
 * Trong quá trình chạy bình thường, lỗi và kiểm thử tải CLI,
 * mức sử dụng tối đa đo được thực tế trong lịch sử khoảng 28 words.
 * Tạm thời giữ nguyên cấu hình hiện tại, sau này khi tối ưu đồng loạt stack các tác vụ sẽ quyết định có thu nhỏ hay không.
 */
#define APP_RTOS_HEALTH_STACK_WORDS       192U

/*
 * HealthTask là tác vụ giám sát hệ thống, độ ưu tiên đặt là 5.
 *
 * Mỗi 100ms nó chỉ thực hiện một lần kiểm tra ngắn,
 * không in nhật ký, không truy cập Flash, cũng không xử lý nghiệp vụ phức tạp.
 *
 * Độ ưu tiên cao đảm bảo ngay cả khi CLI hoặc Flash bận rộn,
 * kiểm tra sức khỏe hệ thống vẫn có thể thực thi đúng giờ.
 */
#define APP_RTOS_HEALTH_PRIORITY          \
    (tskIDLE_PRIORITY + 5U)
/*
 * SensorTask có độ ưu tiên là 4, chỉ đứng sau HealthTask.
 *
 * Dữ liệu ADC là nguồn dữ liệu cho việc đánh giá lỗi BMS và gửi trạng thái CAN,
 * do đó cần kịp thời hoàn tất quy đổi và phát hành snapshot.
 * Khi tác vụ CLI hoặc Flash đang chạy, SensorTask có thể chiếm quyền (preempt) chúng.
 */
#define APP_RTOS_SENSOR_PRIORITY          \
    (tskIDLE_PRIORITY + 4U)

/*
 * BmsTask có độ ưu tiên là 3.
 *
 * SensorTask độ ưu tiên 4:
 * Tạo ra snapshot cảm biến tin cậy mới nhất trước.
 *
 * BmsTask độ ưu tiên 3:
 * Sau đó dùng snapshot để đánh giá lỗi và chuyển đổi trạng thái.
 *
 * CanTask độ ưu tiên 2:
 * Cuối cùng gửi trạng thái nghiệp vụ sau khi BMS đã xử lý.
 */
#define APP_RTOS_BMS_PRIORITY             \
    (tskIDLE_PRIORITY + 3U)

/*
 * CanTask có độ ưu tiên là 2.
 *
 * Nó thấp hơn SensorTask và BmsTask để tránh việc gửi truyền thông làm block tính toán lõi;
 * Nó cao hơn FaultLogTask và DiagnosticTask,
 * nhằm đảm bảo bản tin định kỳ và bản tin lỗi khẩn cấp có thể được gửi đi kịp thời.
 */
#define APP_RTOS_CAN_PRIORITY             \
    (tskIDLE_PRIORITY + 2U)

/*
 * DiagnosticTask có độ ưu tiên là 1.
 *
 * Nó xử lý CLI và chẩn đoán tài nguyên,
 * các chức năng này không trực tiếp quyết định bảo vệ BMS và gửi trạng thái CAN,
 * do đó được xếp chạy sau các tác vụ nghiệp vụ lõi.
 */
#define APP_RTOS_DIAGNOSTIC_PRIORITY          \
    (tskIDLE_PRIORITY + 1U)

/*
 * FaultLogTask có độ ưu tiên là 1.
 *
 * Việc xóa/ghi Flash và xuất nhật ký qua cổng nối tiếp mất nhiều thời gian,
 * do đó nó thấp hơn các tác vụ Sensor, BMS và CAN.
 * Khi hàng đợi rỗng tác vụ sẽ giữ trạng thái block, không chiếm dụng CPU.
 */
#define APP_RTOS_FAULT_LOG_PRIORITY       \
    (tskIDLE_PRIORITY + 1U)

/*
 * Handle của từng tác vụ ứng dụng.
 *
 * Handle của CanTask còn dùng để nhận thông báo thay đổi lỗi từ BmsTask;
 * các handle còn lại hiện chỉ dùng để truy vấn mức stack còn lại tối thiểu.
 */
static TaskHandle_t g_health_task_handle = 0;
static TaskHandle_t g_sensor_task_handle = 0;
static TaskHandle_t g_bms_task_handle = 0;
static TaskHandle_t g_can_task_handle = 0;
static TaskHandle_t g_fault_log_task_handle = 0;
static TaskHandle_t g_diagnostic_task_handle = 0;


/*
 * Phục vụ quan sát trên Keil Watch.
 * Mỗi lần chạy hoàn chỉnh một vòng lặp của tác vụ chẩn đoán thì cộng thêm 1.
 */
volatile uint32_t g_rtos_diagnostic_run_count = 0U;
/*
 * Số lần kiểm tra sức khỏe hệ thống mà HealthTask đã hoàn thành.
 *
 * Lần đầu tiên tăng sau khi hệ thống khởi động 150ms,
 * về sau trong trường hợp bình thường tăng khoảng mỗi 100ms một lần.
 */
volatile uint32_t g_rtos_health_run_count = 0U;
/*
 * SensorTask tăng thêm 1 mỗi khi hoàn thành một vòng lặp.
 * Khai báo không phải static để tiện quan sát trực tiếp trên Keil Watch.
 */
volatile uint32_t g_rtos_sensor_run_count = 0U;
/*
 * BmsTask cộng thêm 1 sau mỗi lần thực thi hoàn chỉnh một vòng máy trạng thái, xử lý sự kiện và báo cáo heartbeat.
 * Dùng cho Keil Watch xác nhận tác vụ BMS có chạy theo chu kỳ 10ms hay không.
 */
volatile uint32_t g_rtos_bms_run_count = 0U;
/*
 * CanTask tăng thêm 1 sau mỗi lần hoàn thành một chu kỳ cơ bản 100ms.
 * Trong trường hợp bình thường tăng khoảng 10 lần mỗi giây.
 */
volatile uint32_t g_rtos_can_run_count = 0U;
/*
 * Số lần CanTask thực hiện gửi khẩn cấp do trạng thái lỗi thay đổi.
 * Chỉ dùng cho Keil Watch kiểm chứng thông báo tác vụ có phát huy tác dụng hay không.
 */
volatile uint32_t g_rtos_can_urgent_send_count = 0U;
/*
 * Số lượng sự kiện lỗi mà FaultLogTask đã chờ được và xử lý thành công.
 *
 * Nhật ký BOOT khi khởi động không đi qua hàng đợi sự kiện lỗi,
 * do đó khi hệ thống vừa khởi động giá trị này vẫn bằng 0.
 */
volatile uint32_t g_rtos_fault_log_processed_count = 0U;

/*
 * Số byte FreeRTOS Heap còn lại sau khi tạo xong toàn bộ tác vụ ứng dụng.
 * Dùng để đánh giá có còn dư bộ nhớ khi tiếp tục tạo thêm tác vụ hay không.
 */
volatile uint32_t g_rtos_free_heap_after_create = 0U;


/*
 * Không gian stack còn lại ít nhất trong lịch sử của từng tác vụ:
 * Kể từ lúc tạo tác vụ đến nay, tại thời điểm nguy hiểm nhất vẫn còn dư bao nhiêu stack.
 *
 * Đơn vị là phần tử stack StackType_t, không phải byte.
 * Trong STM32F103, một phần tử stack là 4 byte.
 */
volatile uint32_t g_rtos_health_stack_free_words = 0U;
volatile uint32_t g_rtos_sensor_stack_free_words = 0U;
volatile uint32_t g_rtos_bms_stack_free_words = 0U;
volatile uint32_t g_rtos_can_stack_free_words = 0U;
volatile uint32_t g_rtos_fault_log_stack_free_words = 0U;
volatile uint32_t g_rtos_diagnostic_stack_free_words = 0U;

/*
 * Số byte FreeRTOS Heap còn lại hiện tại trong quá trình hệ thống vận hành.
 */
volatile uint32_t g_rtos_free_heap_current = 0U;

/*
 * Cập nhật dữ liệu chẩn đoán stack tác vụ và Heap của FreeRTOS.
 *
 * Hàm này được DiagnosticTask có độ ưu tiên thấp gọi mỗi 1 giây một lần,
 * tránh việc quét lặp đi lặp lại không gian stack trong các tác vụ tần suất cao như SensorTask.
 *
 * UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask ):
 * Dấu mức nước cao của stack tác vụ, dùng để xem không gian còn lại tối thiểu của stack trong quá trình tác vụ chạy, đánh giá stack có đủ dùng không.
 */
static void AppRTOS_UpdateResourceDiagnostics(void)
{
    /*
     * Sau khi tạo tác vụ thành công, handle mới trở thành giá trị hợp lệ.
     * Truy vấn không gian stack còn lại ít nhất trong lịch sử của HealthTask thông qua handle.
     */
    if (g_health_task_handle != 0)
    {
        g_rtos_health_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_health_task_handle);
    }

    if (g_sensor_task_handle != 0)
    {
        g_rtos_sensor_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_sensor_task_handle);
    }

    if (g_bms_task_handle != 0)
    {
        g_rtos_bms_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_bms_task_handle);
    }

    if (g_can_task_handle != 0)
    {
        g_rtos_can_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_can_task_handle);
    }

    if (g_fault_log_task_handle != 0)
    {
        g_rtos_fault_log_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_fault_log_task_handle);
    }

    if (g_diagnostic_task_handle != 0)
    {
        g_rtos_diagnostic_stack_free_words =
            (uint32_t)uxTaskGetStackHighWaterMark(
                g_diagnostic_task_handle);
    }

    g_rtos_free_heap_current =
        (uint32_t)xPortGetFreeHeapSize();
}







/*
 * Tác vụ cảm biến độc lập.
 * Phụ trách đọc, quy đổi và phát hành.
 * Hiện áp dụng chu kỳ 1ms kiểm tra cờ sẵn sàng DMA,
 * chưa đưa cơ chế thông báo trực tiếp từ DMA tới tác vụ ở giai đoạn này.
 */
static void AppRTOS_SensorTask(void *argument)
{
    TickType_t last_wake_time;    /* Mốc chu kỳ tuyệt đối 1ms do vTaskDelayUntil duy trì. */
    uint32_t now_ms;              /* Thời gian mili-giây TIM3 truyền cho Sensor Service trong vòng này. */
    uint8_t sample_updated;       /* 1=vòng này thực sự tiêu thụ dữ liệu DMA và đã phát hành snapshot mới. */
    SensorSample_t sensor_sample; /* Snapshot điện áp sau lọc trung bình trượt, dùng để ước lượng SOC. */

    (void)argument;

    /*
     * Ghi lại mốc chu kỳ ban đầu của tác vụ FreeRTOS.
     */
    last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        /*
         * Tạm thời sử dụng thời gian TIM3 làm dấu thời gian của snapshot cảm biến,
         * đảm bảo bộ định thời lỗi của SensorTask và BmsTask dùng chung một nguồn thời gian.
         */
        now_ms = GetTick();

        /*
         * Khi có dữ liệu DMA mới thì hoàn tất quy đổi và phát hành snapshot.
         * Khi không có dữ liệu mới thì không ghi đè khung trước đó.
         */
        sample_updated =
            Sensor_ServiceUpdate(now_ms);

        /*
         * Chỉ báo cáo heartbeat Sensor sau khi đã thực sự phát hành snapshot mới.
         *
         * Nếu tác vụ vẫn đang chạy nhưng ADC/DMA bị dừng,
         * sample_updated sẽ liên tục bằng 0, Health Monitor sẽ không nhận được heartbeat Sensor,
         * và cuối cùng hệ thống sẽ bị IWDG reset.
         */
        if (sample_updated != 0U)
        {
            /*
             * SOC được ước lượng tuyến tính trực tiếp từ điện áp pin, cập nhật một lần cho mỗi snapshot mới.
             * Dùng điện áp hiển thị sau khi lọc trung bình trượt 16 mẫu để tránh nhiễu ADC làm SOC nhảy qua lại;
             * việc đánh giá bảo vệ vẫn dùng snapshot gốc chưa lọc, không bị ảnh hưởng bởi chỗ này.
             */
            if ((Sensor_GetLatestDisplay(&sensor_sample) != 0U) &&
                (sensor_sample.voltage_valid != 0U))
            {
                Soc_UpdateFromVoltage(
                    (uint16_t)(sensor_sample.cell_voltage_v *
                               1000.0f + 0.5f),
                    sensor_sample.timestamp_ms);
            }

            HealthMonitor_Report(
                HEALTH_SOURCE_SENSOR);
        }

        /*
         * Bộ đếm này chứng minh bản thân SensorTask đã hoàn thành một vòng lặp.
         * Ngay cả khi DMA không có dữ liệu mới, nó vẫn tiếp tục tăng.
         */
        g_rtos_sensor_run_count++;

        /*
         * Duy trì chu kỳ tác vụ cố định 1ms.
         */
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(1U));
    }
}


/*
 * Tác vụ lõi BMS độc lập.
 *
 * Trách nhiệm mô-đun:
 * 1. Mỗi 10ms thực thi máy trạng thái BMS một lần;
 * 2. Dùng snapshot mới nhất do SensorTask phát hành để chạy đánh giá lỗi;
 * 3. Xử lý các sự kiện trạng thái khởi động, lỗi và khôi phục phát sinh trong vòng này;
 * 4. Sau khi thực thi hoàn chỉnh thành công sẽ báo cáo heartbeat BMS lên Health Monitor.
 *
 * Tác vụ này không phụ trách:
 * 1. Thu thập ADC và quy đổi giá trị cảm biến;
 * 2. Gửi CAN định kỳ;
 * 3. Ghi nhật ký Flash;
 * 4. Phân tích lệnh CLI;
 * 5. Quyết định cuối cùng có làm mới IWDG hay không.
 */
static void AppRTOS_BmsTask(void *argument)
{
    TickType_t last_wake_time;
    uint16_t previous_fault_mask;

    /*
     * Hiện tại không truyền tham số khởi tạo cho BmsTask.
     */
    (void)argument;

    /*
     * Lưu mốc chu kỳ ban đầu của tác vụ.
     *
     * Về sau vTaskDelayUntil() sẽ cộng thêm 10ms dựa trên giá trị này mỗi lần,
     * qua đó giữ được chu kỳ ổn định.
     */
    last_wake_time = xTaskGetTickCount();
    /*
    * Lưu trạng thái lỗi lúc mới vào tác vụ.
    * Về sau chỉ khi mặt nạ lỗi thực sự thay đổi mới thông báo CanTask.
    */
    previous_fault_mask = g_fault_code;

    for (;;)
    {
        /*
         * Bước thứ nhất: Thực thi máy trạng thái BMS.
         *
         * Nó sẽ đọc snapshot Sensor mới nhất, cập nhật FaultManager,
         * kiểm tra phím khởi động hoặc phím khôi phục theo trạng thái hiện tại, và sinh ra sự kiện trạng thái.
         */
        BMS_StateMachine();

        /*
         * Bước thứ hai: Xử lý các sự kiện trạng thái của vòng này.
         *
         * Ví dụ sau khi máy trạng thái xác nhận lỗi nghiêm trọng và đẩy vào EVENT_FAULT,
         * tại đây sẽ xử lý sự kiện đó ngay trong cùng chu kỳ tác vụ 10ms,
         * đưa trạng thái chuyển vào BMS_FAULT.
         */
        Event_Process();
        /*
        * Kiểm tra trước và sau khi BMS xử lý vòng này, mặt nạ lỗi đang kích hoạt có thay đổi hay không.
        *
        * Ở đây chỉ gửi thông báo, không gửi CAN trực tiếp,
        * qua đó giữ cho CanTask là tác nhân duy nhất sử dụng phần cứng CAN.
        *
        * Nếu kết quả lỗi vòng này khác với vòng trước thì thông báo tác vụ CAN gửi ngay trạng thái lỗi mới.
        */
        if (g_fault_code != previous_fault_mask)
        {
            previous_fault_mask = g_fault_code;

            if (g_can_task_handle != 0)
            {
                xTaskNotifyGive(g_can_task_handle);
            }
        }

        /*
         * Bước thứ ba: Báo cáo heartbeat của tác vụ BMS.
         *
         * Bắt buộc phải đặt sau khi máy trạng thái và xử lý sự kiện đều đã trả về.
         * Nếu logic lõi phía trước bị treo, chương trình sẽ không chạy tới đây,
         * Health Monitor sẽ không hiểu nhầm rằng BMS vẫn khỏe mạnh.
         */
        HealthMonitor_Report(
            HEALTH_SOURCE_BMS);

        /*
         * Chỉ khi logic lõi của cả vòng thực thi hoàn tất, bộ đếm số lần chạy mới tăng.
         */
        g_rtos_bms_run_count++;

        /*
         * Chạy với chu kỳ cố định 10ms.
         *
         * Sau khi BmsTask vào trạng thái block, CPU có thể chạy các tác vụ Ready khác;
         * nếu không có tác vụ ứng dụng nào sẵn sàng thì chạy IdleTask.
         * Trong thời gian tác vụ bị block sẽ không chiếm dụng CPU liên tục như vòng lặp rỗng.
         */
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(10U));
    }
}

/*
 * Tác vụ gửi CAN độc lập.
 *
 * Hai nguyên nhân đánh thức:
 * 1. Chờ đạt chu kỳ 100ms: gửi bản tin trạng thái định kỳ;
 * 2. Nhận được thông báo từ BmsTask: gửi ngay trạng thái lỗi mới nhất.
 * ulTaskNotifyTake(): phụ trách chờ "thay đổi lỗi" hoặc "đến hạn chu kỳ".
 * next_periodic_time: phụ trách đảm bảo chu kỳ bản tin 100ms không bị đặt lại bộ đếm thời gian do thông báo lỗi.
 *
 * Dù trong trường hợp nào, phần cứng CAN luôn chỉ do tác vụ này truy cập.
 */
static void AppRTOS_CanTask(void *argument)
{
    TickType_t next_periodic_time; // Thời điểm gửi định kỳ tiếp theo
    TickType_t now; // Thời gian hiện tại
    TickType_t wait_ticks;  // Chờ tối đa bao lâu
    uint32_t notify_count;  // Có nhận được thông báo thay đổi lỗi hay không
    uint8_t temperature_cycle = 0U; // Đếm khi nào gửi bản tin nhiệt độ

    (void)argument;

    /*
     * Vòng gửi định kỳ đầu tiên được sắp xếp sau khi tác vụ khởi động 100ms.
     */
    next_periodic_time =
        xTaskGetTickCount() +
        pdMS_TO_TICKS(100U);

    for (;;)
    {
        now = xTaskGetTickCount();

        /*
         * Tính thời gian còn lại cho tới chu kỳ 100ms tiếp theo.
         *
         * Nếu đã đến hạn, thời gian chờ đặt bằng 0,
         * tác vụ thực hiện gửi định kỳ ngay.
         */
        if ((int32_t)(next_periodic_time - now) > 0)
        {
            wait_ticks = next_periodic_time - now;
        }
        else
        {
            wait_ticks = 0U;
        }

        /*
         * Chờ một trong hai sự kiện:
         *
         * 1. BmsTask gửi thông báo tác vụ;
         * 2. wait_ticks hết giờ (timeout), nghĩa là đã tới thời gian gửi định kỳ.
         *
         * pdTRUE biểu thị sau khi lấy thông báo sẽ xóa bộ đếm thông báo về 0.
         */
        notify_count =
            ulTaskNotifyTake(
                pdTRUE, // Sau khi lấy thông báo thành công, xóa Notification Value về 0.
                wait_ticks);

        /*
         * Giá trị trả về khác 0 nghĩa là mặt nạ lỗi đã thay đổi.
         * Gửi ngay trạng thái lỗi hoàn chỉnh mới nhất.
         */
        if (notify_count != 0U)
        {
            AppCAN_SendFaultStatus();
            g_rtos_can_urgent_send_count++; // Số lần CanTask thực hiện gửi khẩn cấp do trạng thái lỗi thay đổi
        }

        now = xTaskGetTickCount();

        /*
         * Bất kể giữa chừng có nhận được thông báo lỗi hay không,
         * lịch gửi định kỳ 100ms ban đầu vẫn được duy trì độc lập.
         */
        if ((int32_t)(now - next_periodic_time) >= 0)
        {
            AppCAN_SendVoltageStatus(); // Quy đổi dữ liệu điện áp rồi gửi
            AppCAN_SendFaultStatus(); // Ghép mặt nạ lỗi đang kích hoạt, Alive Counter và CRC8 thành bản tin lỗi hoàn chỉnh rồi gửi

            temperature_cycle++; // Đếm khi nào gửi bản tin nhiệt độ

            if (temperature_cycle >= 5U)  // Nhiệt độ gửi mỗi 500ms một lần
            {
                temperature_cycle = 0U;
                AppCAN_SendTemperatureStatus(); // Gửi trạng thái nhiệt độ hợp lệ hiện tại
            }
            /*
            * Chỉ sau khi xử lý CAN định kỳ của vòng này trả về toàn bộ mới báo cáo heartbeat CanTask.
            *
            * Nếu đường gửi CAN bị treo, chương trình không thể chạy tới đây,
            * Health Monitor sẽ không nhận được heartbeat CAN và dừng nạp lại watchdog.
            */
            HealthMonitor_Report(
                HEALTH_SOURCE_CAN);

            g_rtos_can_run_count++;

            /*
             * Cộng thêm 100ms trên mốc chu kỳ gốc,
             * tránh thời gian chạy của tác vụ làm trôi dần chu kỳ.
             */
            next_periodic_time +=
                pdMS_TO_TICKS(100U);
        }
    }
}

/*
 * Tác vụ nhật ký lỗi độc lập.
 *
 * Nguồn dữ liệu:
 * FaultEvent Queue do BmsTask tạo ra.
 *
 * Phương thức thực thi:
 * Khi hàng đợi rỗng thì block tối đa 1 giây, không chiếm dụng CPU;
 * Khi hàng đợi xuất hiện sự kiện sẽ được FreeRTOS tự động đánh thức;
 * Mỗi lần xử lý một sự kiện theo thứ tự vào trước ra trước (FIFO), sau đó kiểm tra một lần yêu cầu lưu SOC.
 *
 * Tác vụ này không đánh giá lỗi, không sửa trạng thái BMS,
 * cũng không điều khiển trực tiếp còi báo và CAN.
 *
 * Logic vận hành:
 * Hàng đợi rỗng
 *    ↓
 * FaultLogTask bị block
 *    ↓
 * BmsTask xác nhận lỗi và đưa vào hàng đợi
 *    ↓
 * FreeRTOS đánh thức FaultLogTask
 *    ↓
 * Lấy sự kiện ra
 *    ↓
 * Chiếm giữ Flash Mutex
 *    ↓
 * Ghi, đọc lại kiểm tra xác thực, giải phóng Mutex
 *    ↓
 * Tiếp tục chờ sự kiện tiếp theo
 */
static void AppRTOS_FaultLogTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        /*
         * Chờ sự kiện lỗi tối đa 1 giây.
         */
        if (FaultEvent_ProcessOne(
                pdMS_TO_TICKS(1000U)) != 0U)
        {
            g_rtos_fault_log_processed_count++;
        }
    }
}

/*
 * Tác vụ giám sát sức khỏe hệ thống độc lập.
 *
 * Trách nhiệm mô-đun:
 * 1. Mỗi 100ms kiểm tra heartbeat của các tác vụ Sensor, BMS và CAN;
 * 2. Khi heartbeat đầy đủ thì làm mới watchdog độc lập IWDG;
 * 3. Khi heartbeat thiếu liên tục thì dừng nạp lại watchdog, để IWDG reset hệ thống.
 *
 * Tác vụ này không phụ trách:
 * 1. Tạo heartbeat của từng tác vụ nghiệp vụ;
 * 2. Thực hiện thu thập ADC, máy trạng thái BMS hoặc gửi CAN;
 * 3. In nhật ký hoặc xử lý lệnh CLI.
 *
 * Lần kiểm tra đầu tiên được sắp xếp vào thời điểm 150ms sau khi khởi động,
 * tránh biên chu kỳ thực thi của CanTask tại 100ms, 200ms, 300ms.
 */
static void AppRTOS_HealthTask(void *argument)
{
    TickType_t last_wake_time;

    /*
     * Hiện tại khi tạo tác vụ sẽ không truyền tham số vào.
     */
    (void)argument;

    /*
     * Ghi lại nhịp tick FreeRTOS khi HealthTask chạy lần đầu.
     */
    last_wake_time = xTaskGetTickCount();

    /*
     * Trì hoãn lần kiểm tra sức khỏe đầu tiên đến 150ms sau khi khởi động.
     *
     * Lần gửi định kỳ đầu tiên của CanTask xảy ra tại 100ms,
     * do đó nó có đủ thời gian hoàn tất việc gửi CAN và báo cáo heartbeat,
     * tránh việc HealthTask kiểm tra trước ngay trên cùng biên 100ms.
     */
    vTaskDelayUntil(
        &last_wake_time,
        pdMS_TO_TICKS(150U));

    for (;;)
    {
        /*
         * Đọc và xóa các bit heartbeat của cửa sổ này,
         * sau đó dựa vào việc heartbeat có đầy đủ hay không để quyết định có làm mới IWDG không.
         */
        HealthMonitor_Run();

        /*
         * Chỉ tăng sau khi HealthMonitor_Run() trả về hoàn chỉnh.
         * Dùng để chứng minh bản thân HealthTask vẫn luôn chạy định kỳ.
         */
        g_rtos_health_run_count++;

        /*
         * Lần kiểm tra đầu tiên xảy ra tại 150ms,
         * các lần kiểm tra tiếp theo xảy ra tại 250ms, 350ms, 450ms...
         */
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(100U));
    }
}


/*
 * Tác vụ chẩn đoán và bảo trì độc lập.
 *
 * Trách nhiệm mô-đun:
 * 1. Mỗi 1ms xử lý một lần bộ đệm thu USART CLI;
 * 2. Mỗi vòng xử lý tối đa 16 ký tự, giới hạn thời gian thực thi mỗi lần;
 * 3. Mỗi 1 giây cập nhật mức stack còn lại tối thiểu của các tác vụ và Heap còn lại;
 * 4. Cung cấp điểm vào truy vấn trạng thái, chèn lỗi, nhật ký và kiểm thử watchdog.
 *
 * Tác vụ này không thực hiện thu thập cảm biến, đánh giá lỗi BMS,
 * gửi CAN định kỳ, ghi nhật ký lỗi hoặc quyết định sức khỏe IWDG.
 *
 * Hiện sử dụng độ ưu tiên 1, các tác vụ nghiệp vụ lõi có thể chiếm quyền nó bất cứ lúc nào.
 */
static void AppRTOS_DiagnosticTask(void *argument)
{
    TickType_t last_wake_time;
    TickType_t resource_check_time;
    TickType_t now;

    (void)argument;

    /*
     * Lưu nhịp tick FreeRTOS khi tác vụ chạy lần đầu.
     * Về sau vTaskDelayUntil sẽ lấy nó làm mốc chu kỳ.
     */
    last_wake_time = xTaskGetTickCount();
    resource_check_time = last_wake_time;

    /*
    * Ghi nhận ngay một lần khi mới vào DiagnosticTask lần đầu,
    * quá trình chạy tiếp theo và kiểm thử tải sẽ tiếp tục làm mới mức dư tối thiểu trong lịch sử.
    */
    AppRTOS_UpdateResourceDiagnostics();

    for (;;)
    {
        /*
         * CLI trước đây do vòng lặp chính main quét vòng (polling),
         * nay do DiagnosticTask quét vòng định kỳ.
         */
        CLI_Process();
        /*
        * Mỗi 1 giây cập nhật một lần mức stack còn lại tối thiểu của tác vụ và Heap còn lại.
        */
        now = xTaskGetTickCount();

        if ((now - resource_check_time) >=
            pdMS_TO_TICKS(1000U))
        {
            /*
            * Sử dụng trực tiếp thời gian hiện tại để thiết lập lại mốc.
            * Ngay cả khi CLI từng in kéo dài, cũng sẽ không quét bù liên tiếp nhiều lần.
            */
            resource_check_time = now;

            AppRTOS_UpdateResourceDiagnostics();
        }
        /*
         * Bắt buộc đặt sau khi cả hai hàm chính đều đã trả về.
         * Nếu một trong hai hàm bị treo, bộ đếm sẽ ngừng tăng.
         */
        g_rtos_diagnostic_run_count++;

        /*
         * Cho tác vụ chạy theo chu kỳ cố định 1ms.
         * Trong thời gian tác vụ bị block, CPU có thể thực thi các tác vụ sẵn sàng khác.
         */
        vTaskDelayUntil(
            &last_wake_time,
            pdMS_TO_TICKS(1U));
    }
}


/*
 * Tạo toàn bộ các tác vụ ứng dụng của giai đoạn chuyển đổi hiện tại.
 *
 * Thứ tự tạo:
 * 1. HealthTask: kiểm tra heartbeat của các tác vụ trọng yếu và quyết định có làm mới IWDG hay không;
 * 2. SensorTask: xử lý dữ liệu ADC/DMA và phát hành snapshot cảm biến;
 * 3. BmsTask: chạy đánh giá lỗi, máy trạng thái và sự kiện trạng thái;
 * 4. CanTask: gửi tập trung bản tin định kỳ và bản tin lỗi khẩn cấp;
 * 5. FaultLogTask: chờ block sự kiện lỗi và ghi vào Flash;
 * 6. DiagnosticTask: xử lý CLI và chẩn đoán tài nguyên.
 *
 * Thứ tự tạo không đồng nghĩa với thứ tự chạy của tác vụ.
 * Sau khi bộ lập lịch khởi động, FreeRTOS chọn tác vụ chạy dựa theo độ ưu tiên của tác vụ.
 *
 * Hàm này chỉ tạo tác vụ, không khởi động bộ lập lịch.
 */
uint8_t AppRTOS_CreateTasks(void)
{
    BaseType_t create_result;

    g_health_task_handle = 0;
    g_sensor_task_handle = 0;
    g_bms_task_handle = 0;
    g_can_task_handle = 0;
    g_fault_log_task_handle = 0;
    g_diagnostic_task_handle = 0;

    // Số lần kiểm tra mà HealthTask đã hoàn thành
    g_rtos_health_run_count = 0U;

    // Không gian stack còn lại ít nhất trong lịch sử của HealthTask
    g_rtos_health_stack_free_words = 0U;
    g_rtos_sensor_stack_free_words = 0U;
    g_rtos_bms_stack_free_words = 0U;
    g_rtos_can_stack_free_words = 0U;
    g_rtos_fault_log_stack_free_words = 0U;
    g_rtos_diagnostic_stack_free_words = 0U;

    g_rtos_free_heap_current = 0U;

    /*
     * Tạo HealthTask độc lập.
     *
     * Tác vụ này sau khi khởi động chờ trước 150ms,
     * sau đó mỗi 100ms kiểm tra một lần heartbeat của các tác vụ trọng yếu.
     */
    create_result = xTaskCreate(
        AppRTOS_HealthTask,          // Hàm tác vụ cần chạy
        "Health",                    // Tên tác vụ hiển thị trong trình gỡ lỗi
        APP_RTOS_HEALTH_STACK_WORDS, // Cấp phát 192 words stack tác vụ
        0,                           // Không truyền tham số vào tác vụ
        APP_RTOS_HEALTH_PRIORITY,    // Độ ưu tiên 5
        &g_health_task_handle);      // Lưu handle tác vụ sau khi tạo

    if (create_result != pdPASS)
    {
        return 0U;
    }

    /*
     * Tạo SensorTask độc lập.
     */
    create_result = xTaskCreate(
        AppRTOS_SensorTask,
        "Sensor",
        APP_RTOS_SENSOR_STACK_WORDS,
        0,
        APP_RTOS_SENSOR_PRIORITY,
        &g_sensor_task_handle);

    if (create_result != pdPASS)
    {
        return 0U;
    }

    /*
     * Tạo BmsTask độc lập.
     */
    create_result = xTaskCreate(
        AppRTOS_BmsTask,
        "BMS",
        APP_RTOS_BMS_STACK_WORDS,
        0,
        APP_RTOS_BMS_PRIORITY,
        &g_bms_task_handle);

    if(create_result != pdPASS)
    {
        return 0U;
    }

    /*
    * Tạo CanTask độc lập.
    * Từ đây việc gửi CAN định kỳ do tác vụ này thực hiện tập trung.
    */
    create_result = xTaskCreate(
        AppRTOS_CanTask,
        "CAN",
        APP_RTOS_CAN_STACK_WORDS,
        0,
        APP_RTOS_CAN_PRIORITY,
        &g_can_task_handle);

    if (create_result != pdPASS)
    {
        return 0U;
    }

    /*
    * Tạo FaultLogTask độc lập.
    *
    * Sau khi tác vụ khởi động sẽ block ngay trên hàng đợi sự kiện lỗi,
    * cho đến khi BmsTask tạo ra sự kiện xác nhận hoặc xóa lỗi đầu tiên.
    */
    create_result = xTaskCreate(
        AppRTOS_FaultLogTask,
        "FaultLog",
        APP_RTOS_FAULT_LOG_STACK_WORDS,
        0,
        APP_RTOS_FAULT_LOG_PRIORITY,
        &g_fault_log_task_handle);

    if (create_result != pdPASS)
    {
        return 0U;
    }

    /*
    * Tạo tác vụ chẩn đoán độc lập.
    *
    * Tác vụ phụ trách quét vòng CLI và chẩn đoán tài nguyên,
    * không tham gia bảo vệ thời gian thực BMS và gửi CAN định kỳ.
    */
    create_result = xTaskCreate(
        AppRTOS_DiagnosticTask,
        "Diag",
        APP_RTOS_DIAGNOSTIC_STACK_WORDS,
        0,
        APP_RTOS_DIAGNOSTIC_PRIORITY,
        &g_diagnostic_task_handle);

    if (create_result != pdPASS)
    {
        return 0U;
    }

    /*
    * Ghi lại Heap còn lại hiện tại, để kiểm tra trên Keil Watch.
    *
    * Giá trị này không bao gồm không gian stack và TCB đã cấp phát cho từng tác vụ.
    */
    g_rtos_free_heap_after_create =
        (uint32_t)xPortGetFreeHeapSize();

    return 1U;
}
