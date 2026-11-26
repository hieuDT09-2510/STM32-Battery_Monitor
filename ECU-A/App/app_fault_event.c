/*
 * Tên file: app_fault_event.c
 *
 * Tên mô-đun: Hàng đợi RAM sự kiện sườn lỗi
 *
 * Trách nhiệm mô-đun:
 * 1. Lưu một bản snapshot hiện trường cố định khi xác nhận hoặc xóa lỗi;
 * 2. Sử dụng FreeRTOS Queue truyền sự kiện lỗi từ luồng thực thi BMS sang luồng xử lý log;
 * 3. Đảm bảo chỉ số hàng đợi và nội dung sự kiện không bị phá hủy khi các task đọc/ghi đồng thời;
 * 4. Mỗi lần lấy một sự kiện ra khỏi hàng đợi, ghi vào Flash V2 và xuất chẩn đoán qua UART.
 *
 * Luồng dữ liệu hiện tại:
 * FaultManager phát hiện biến đổi lỗi
 *          ↓
 * FaultEvent_Push() ghi vào hàng đợi không chặn
 *          ↓
 * FreeRTOS Queue lưu bản sao sự kiện
 *          ↓
 * FaultEvent_ProcessOne() lấy ra khỏi hàng đợi
 *          ↓
 * Log Flash và chẩn đoán USART
 *
 * Chiến lược độ tin cậy:
 * Khi hàng đợi đầy loại bỏ sự kiện mới và tăng dropped_count;
 * Luồng bảo vệ lỗi không chờ đợi Flash, cũng không bị block vì hàng đợi log bị đầy.
 *
 * Ranh giới mô-đun:
 * Mô-đun này không phán đoán điều kiện lỗi, không chuyển trạng thái BMS, không điều khiển còi báo,
 * cũng không chịu trách nhiệm thu thập cảm biến và gửi CAN.
 */ 
#include "app_fault_event.h" 
 
#include "FreeRTOS.h"
#include "queue.h"

#include <stdio.h> 
 
#include "app_flash_log.h"
#include "app_fault_manager.h"
#include "app_soc.h"

/*
 * Hàng đợi tĩnh FreeRTOS cần 3 phần:
 *
 * 1. g_fault_event_queue_control：
 *    FreeRTOS lưu vị trí đọc/ghi, số lượng hiện tại và các thông tin quản lý hàng đợi khác.
 *
 * 2. g_fault_event_queue_storage：
 *    Vùng nhớ RAM thực sự lưu 8 sự kiện FaultEvent_t.
 *
 * 3. g_fault_event_queue_handle：
 *    Handle được các hàm khác sử dụng khi thao tác trên hàng đợi này.
 *
 * Hàng đợi dùng bộ nhớ tĩnh, không cấp phát động không gian từ Heap FreeRTOS.
 */
static StaticQueue_t g_fault_event_queue_control;

static uint8_t g_fault_event_queue_storage[
    FAULT_EVENT_QUEUE_CAPACITY * sizeof(FaultEvent_t)];

static QueueHandle_t g_fault_event_queue_handle = 0;
 
// Không dùng static để Keil Watch có thể trực tiếp quan sát, mô-đun nghiệp vụ chỉ được đọc. 
volatile FaultEventDebug_t g_fault_event_debug; 
 
// Bộ đếm chẩn đoán giữ nguyên sau khi đạt giá trị lớn nhất, tránh quay vòng về 0 sau thời gian dài vận hành. 
static void FaultEvent_IncrementSaturated(volatile uint32_t *value) 
{ 
    if (*value < 0xFFFFFFFFU) 
    { 
        (*value)++; 
    } 
} 
 
// Chuyển đổi điện áp từ V sang mV, và giới hạn trong phạm vi uint16_t. 
static uint16_t FaultEvent_ToMillivolts(float voltage_v) 
{ 
    float value = voltage_v * 1000.0f; 
 
    if (value <= 0.0f) 
    { 
        return 0U; 
    } 
 
    if (value >= 65535.0f) 
    { 
        return 0xFFFFU; 
    } 
 
    return (uint16_t)(value + 0.5f); 
} 
 
// Chuyển đổi nhiệt độ từ °C sang 0.1°C, làm tròn số cả âm lẫn dương và giới hạn phạm vi. 
static int16_t FaultEvent_ToDeciCelsius(float temperature_c) 
{ 
    float value = temperature_c * 10.0f; 
 
    if (value <= -32768.0f) 
    { 
        return (int16_t)-32768; 
    } 
 
    if (value >= 32767.0f) 
    { 
        return (int16_t)32767; 
    } 
 
    return (value >= 0.0f) ? 
           (int16_t)(value + 0.5f) : 
           (int16_t)(value - 0.5f); 
} 
 
// Xóa sự kiện theo từng trường, tránh phụ thuộc memset và các hiện thực runtime khác nhau. 
static void FaultEvent_ClearEvent(volatile FaultEvent_t *event) 
{ 
    event->timestamp_ms = 0U; 
    event->changed_mask = 0U; 
    event->active_mask = 0U; 
    event->cell_voltage_mv = 0U; 
    event->temperature_dC = 0; 
    event->event_type = 0U; 
    event->bms_state = 0U; 
    event->valid_flags = 0U; 
    event->soc_percent = 0xFFU;
} 
 
void FaultEvent_Init(void)
{
    /*
     * Lần đầu khởi tạo, liên kết cấu trúc điều khiển và không gian lưu trữ cố định cho hàng đợi.
     *
     * Độ dài hàng đợi:
     * FAULT_EVENT_QUEUE_CAPACITY, tức 8 sự kiện.
     *
     * Kích thước mỗi phần tử:
     * sizeof(FaultEvent_t)。
     */
    if (g_fault_event_queue_handle == 0)
    {
        g_fault_event_queue_handle =
            xQueueCreateStatic(
                FAULT_EVENT_QUEUE_CAPACITY, // Hàng đợi lưu tối đa bao nhiêu phần tử
                sizeof(FaultEvent_t),   // Mỗi phần tử chiếm bao nhiêu byte
                g_fault_event_queue_storage, // Không gian lưu trữ dữ liệu
                &g_fault_event_queue_control); // Cấu trúc quản lý hàng đợi
    }
    else
    {
        /*
         * Nếu sau này gọi lại hàm khởi tạo,
         * làm sạch các sự kiện cũ chưa xử lý trong hàng đợi, nhưng không cấp phát lại bộ nhớ.
         */
        (void)xQueueReset(g_fault_event_queue_handle);
    }

    FaultEvent_ClearEvent(
        &g_fault_event_debug.last_pushed);

    FaultEvent_ClearEvent(
        &g_fault_event_debug.last_processed);

    // Biến thuận tiện cho debug trên Keil
    g_fault_event_debug.pushed_count = 0U;
    g_fault_event_debug.processed_count = 0U;
    g_fault_event_debug.dropped_count = 0U;
    g_fault_event_debug.depth = 0U;
    g_fault_event_debug.last_pushed_valid = 0U;
    g_fault_event_debug.last_processed_valid = 0U;
}

/*
Khi lỗi vừa xác nhận hoặc vừa xóa, sao chép lỗi, điện áp, nhiệt độ, trạng thái BMS và thời gian lúc đó vào hàng đợi RAM
Kiểm tra tham số
   ↓
Kiểm tra hàng đợi có đầy không
   ↓
Tìm vị trí trống do tail trỏ tới
   ↓
Điền thông tin cơ bản của sự kiện
   ↓
Điền điện áp và nhiệt độ hợp lệ
   ↓
Cập nhật thông tin debug
   ↓
Di chuyển tail
   ↓
Tăng count
   ↓
Trả về thành công
*/
uint8_t FaultEvent_Push(FaultEventType_t event_type, // Xác nhận lỗi hay xóa lỗi
                        uint16_t changed_mask, // Lỗi mới phát sinh hoặc mới xóa lần này
                        uint16_t active_mask, // Toàn bộ lỗi vẫn tồn tại sau biến đổi
                        const SensorSample_t *sample, // Snapshot đo truyền vào, có thể là NULL
                        uint8_t bms_state, // Trạng thái máy trạng thái BMS, phạm vi 0~255
                        uint32_t timestamp_ms) // Thời gian xảy ra sự kiện, đơn vị ms
{ 
    FaultEvent_t event;          /* Lắp ráp hoàn chỉnh trên stack rồi copy theo giá trị vào Queue. */
    SocSnapshot_t soc_snapshot;  /* Snapshot SOC tại thời điểm sườn lỗi phát sinh. */
 
    /*
     * Khi không có biến đổi lỗi hoặc loại sự kiện bất hợp pháp,
     * không tạo bản ghi lỗi không có ý nghĩa thực tế.
     */
    if ((changed_mask == 0U) ||
        ((event_type != FAULT_EVENT_CONFIRMED) &&
         (event_type != FAULT_EVENT_CLEARED)))
    {
        return 0U;
    }

    /*
     * Không thể tiếp tục thao tác khi hàng đợi chưa được khởi tạo thành công.
     */
    if (g_fault_event_queue_handle == 0)
    {
        return 0U;
    }

    /*
     * Lắp ráp sự kiện hoàn chỉnh trong biến cục bộ của hàm hiện tại trước.
     * Sau khi tất cả các trường được điền hoàn tất mới copy toàn bộ sự kiện vào hàng đợi.
     */
    event.timestamp_ms = timestamp_ms;
    event.changed_mask = changed_mask;
    event.active_mask = active_mask;
    event.cell_voltage_mv = 0U;
    event.temperature_dC = 0;
    event.event_type = (uint8_t)event_type;
    event.bms_state = bms_state;
    event.valid_flags = 0U;
    event.soc_percent = 0xFFU;

    /*
     * Điện áp và nhiệt độ phán đoán tính hiệu lực riêng biệt.
     * Ví dụ khi lỗi NTC khiến nhiệt độ không hợp lệ, vẫn có thể lưu điện áp pin đáng tin cậy.
     */
    if (sample != 0)
    {
        if (sample->voltage_valid != 0U)
        {
            event.cell_voltage_mv =
                FaultEvent_ToMillivolts(    // Chuyển đổi điện áp
                    sample->cell_voltage_v);

            event.valid_flags |=
                FAULT_EVENT_VALID_VOLTAGE; // Xác nhận dữ liệu hợp lệ
        }

        if (sample->temperature_valid != 0U)
        {
            event.temperature_dC =
                FaultEvent_ToDeciCelsius(
                    sample->temperature_c);

            event.valid_flags |=
                FAULT_EVENT_VALID_TEMPERATURE;
        }
    }

    /*
     * SOC cũng bắt buộc phải sao chép khi sườn lỗi phát sinh. Nếu chờ đến khi task log ưu tiên thấp ghi Flash
     * mới đọc thì có thể ghi nhận SOC của hàng trăm mili-giây sau đó.
     */
    if (Soc_GetSnapshot(&soc_snapshot) != 0U)
    {
        event.soc_percent = (uint8_t)(
            (soc_snapshot.soc_centi_percent + 50U) / 100U);
        event.valid_flags |= FAULT_EVENT_VALID_SOC;
    }

    /*
     * Khi sự kiện xác nhận lỗi khớp với mặt nạ test inject hiện tại,
     * đánh dấu trong log nguồn gốc là từ test inject.
     */
    if ((event_type == FAULT_EVENT_CONFIRMED) &&
        ((changed_mask &
          FaultManager_GetInjectionMask()) != 0U))
    {
        event.valid_flags |=
            FAULT_EVENT_FLAG_INJECTED;
    }

    /*
     * Thời gian chờ dùng 0 biểu thị thử vào hàng đợi ngay lập tức.
     *
     * Hàng đợi có chỗ trống: copy event và trả về pdPASS.
     * Hàng đợi đã đầy: thất bại ngay lập tức, không block quy trình bảo vệ BMS.
     */
    if (xQueueSend(
            g_fault_event_queue_handle,
            &event,
            0U) != pdPASS)
    {
        // Bộ đếm chẩn đoán giữ nguyên sau khi đạt giá trị lớn nhất, tránh quay vòng về 0 sau thời gian dài vận hành
        FaultEvent_IncrementSaturated(
            &g_fault_event_debug.dropped_count);

        return 0U;
    }

    /*
     * Chỉ sau khi thực sự vào hàng đợi thành công mới cập nhật thông tin chẩn đoán thành công.
     */
    g_fault_event_debug.last_pushed = event;
    g_fault_event_debug.last_pushed_valid = 1U;

    FaultEvent_IncrementSaturated(
        &g_fault_event_debug.pushed_count);

    g_fault_event_debug.depth =
        (uint8_t)uxQueueMessagesWaiting(
            g_fault_event_queue_handle);

    return 1U;
}
 
/*
Nó chịu trách nhiệm lấy ra sự kiện lỗi sớm nhất từ hàng đợi.
Kiểm tra tham số gọi
    ↓
Kiểm tra hàng đợi có rỗng không
    ↓
Sao chép sự kiện từ vị trí head
    ↓
head di chuyển đến vị trí tiếp theo
    ↓
Số lượng sự kiện trong hàng đợi giảm 1
    ↓
Cập nhật biến chẩn đoán Watch
    ↓
Trả về thành công
*/
uint8_t FaultEvent_Pop(
    FaultEvent_t *event,
    TickType_t wait_ticks)
{
    /*
     * Caller không cung cấp địa chỉ nhận,
     * hoặc hàng đợi chưa khởi tạo thì trực tiếp trả về thất bại.
     */
    if ((event == 0) ||
        (g_fault_event_queue_handle == 0))
    {
        return 0U;
    }

    /*
    * wait_ticks quyết định hành vi khi hàng đợi rỗng:
    *
    * 0：
    * Trả về ngay lập tức, không block task hiện tại.
    *
    * Số Tick hữu hạn:
    * Chờ tối đa thời gian chỉ định.
    *
    * portMAX_DELAY：
    * Chặn liên tục cho đến khi producer ghi sự kiện mới.
    *
    * Task không chiếm CPU trong thời gian bị block.
    */
    if (xQueueReceive(
            g_fault_event_queue_handle, // Handle hàng đợi
            event,                  // Địa chỉ dữ liệu nhận
            wait_ticks) != pdPASS) // Thời gian chờ tối đa
    {
        return 0U;
    }

    /*
     * Chỉ cập nhật dữ liệu chẩn đoán sau khi de-queue thành công.
     */
    g_fault_event_debug.last_processed = *event;
    g_fault_event_debug.last_processed_valid = 1U;

    FaultEvent_IncrementSaturated(
        &g_fault_event_debug.processed_count);

    // Sao chép số lượng message hiện tại bên trong FreeRTOS ra để Watch quan sát
    g_fault_event_debug.depth =
        (uint8_t)uxQueueMessagesWaiting(
            g_fault_event_queue_handle);

    return 1U;
}
 

/*
 * Mỗi lần tiêu thụ tối đa một sự kiện sườn lỗi.
 *
 * Thứ tự thực hiện:
 * Lấy sự kiện sớm nhất ra khỏi hàng đợi RAM
 *        ↓
 * Ghi vào Flash V2 và đọc lại xác thực
 *        ↓
 * Xuất thông tin chẩn đoán sự kiện
 *
 * Hàng đợi rỗng thì trả về ngay lập tức, không chiếm dụng thời gian vòng lặp lập lịch.
 */
uint8_t FaultEvent_ProcessOne(TickType_t wait_ticks)
{
    FaultEvent_t event;
    uint8_t flash_write_ok;

    /*
     * Trả về 1: Lấy thành công một sự kiện.
     * Trả về 0: Hàng đợi rỗng, không cần xử lý gì.
     */
    if (FaultEvent_Pop(
            &event,
            wait_ticks) == 0U)
    {
        /*
        * Khi hàng đợi rỗng hoặc nhận thất bại, trong event không có dữ liệu hợp lệ,
        * bắt buộc phải trả về ngay, không được tiếp tục truy cập nó.
        */
        return 0U;
    }

    /*
     * Sự kiện đã được copy từ hàng đợi RAM sang biến cục bộ event,
     * do đó trong thời gian ghi Flash tiếp theo vị trí hàng đợi ban đầu có thể tái sử dụng an toàn.
     */
    flash_write_ok = FlashLog_WriteEvent(&event);

    /*
     * Flash thất bại không được làm máy trạng thái bị kẹt.
     * Số lần thất bại do g_flash_log_write_fail_count tích lũy,
     * ở đây in thêm thông tin sự kiện lỗi nào bị lưu thất bại.
     */
    if (flash_write_ok == 0U)
    {
        printf("[FAULT_EVT] Flash write failed: type=%u changed=0x%04X\r\n",
               (unsigned int)event.event_type,
               (unsigned int)event.changed_mask);
    }

    printf("[FAULT_EVT] type=%u changed=0x%04X active=0x%04X time=%lu flash=%u\r\n",
           (unsigned int)event.event_type,
           (unsigned int)event.changed_mask,
           (unsigned int)event.active_mask,
           (unsigned long)event.timestamp_ms,
           (unsigned int)flash_write_ok);

    return 1U;
}
