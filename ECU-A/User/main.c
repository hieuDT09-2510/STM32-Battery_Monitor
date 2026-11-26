/*
 * Tên tệp: main.c
 *
 * Tên mô-đun: Điểm khởi động hệ thống ECU-A
 *
 * Trách nhiệm của mô-đun:
 * 1. Khởi tạo phần cứng cấp board và các mô-đun ứng dụng theo thứ tự phụ thuộc;
 * 2. Khởi tạo Sensor, trình quản lý lỗi, hàng đợi sự kiện và hệ thống ghi log;
 * 3. Tạo các task ứng dụng FreeRTOS;
 * 4. Khởi chạy bộ lập lịch (scheduler) FreeRTOS;
 * 5. Lưu giữ ngữ cảnh lỗi khi tạo task hoặc khởi chạy scheduler thất bại.
 *
 * Giai đoạn chuyển đổi hiện tại:
 * FreeRTOS đã tiếp quản việc lập lịch CPU.
 * Sensor, BMS, CAN, FaultLog và Health đã được chuyển thành các task độc lập;
 * Việc hiển thị trạng thái LED do BmsTask xử lý thống nhất, Scheduler bare-metal đã ngừng sử dụng;
 * DiagnosticTask đảm nhiệm chức năng CLI và chẩn đoán tài nguyên.
 *
 * Nguồn thời gian hiện tại:
 * SysTick cung cấp nhịp hệ thống (system tick) 1ms cho FreeRTOS;
 * TIM3 tạm thời tiếp tục cung cấp kích hoạt phần cứng ADC và thời gian GetTick() của bare-metal cũ.
 *
 * Định hướng tiếp theo:
 * Sau này sẽ hoàn thiện cơ chế bảo vệ đồng thời khi xuất dữ liệu USART đa task.
 */

// Nạp giao diện ngoại vi và mức ưu tiên ngắt của STM32F103.
#include "stm32f10x.h"
/*
 * FreeRTOS.h phải được đặt trước các tệp header chức năng khác của FreeRTOS.
 * task.h cung cấp các giao diện tạo task và khởi chạy scheduler.
 */
#include "FreeRTOS.h"
#include "task.h"
// Nạp khai báo in qua cổng nối tiếp.
#include <stdio.h>

// Nạp driver GPIO cấp board, còi báo và giao tiếp.
#include "bsp_gpio.h"
#include "bsp_buzzer.h"
#include "bsp_load_switch.h"
#include "bsp_usart.h"
#include "bsp_can.h"

// Nạp driver ADC/DMA, TIM3 và SPI Flash.
#include "bsp_adc_dma.h"
#include "bsp_tim.h"
#include "bsp_spi_flash.h"

// Nạp giao diện cảm biến, log và sự kiện ở tầng ứng dụng.
#include "app_sensor.h"
#include "app_soc.h"
#include "app_flash_access.h"
#include "app_flash_log.h"
#include "app_event.h"

#include "app_fault_manager.h"
#include "app_rtos.h"

// Nạp hàng đợi RAM lưu cạnh lỗi; sau này dùng giao diện này để tách biệt bảo vệ trạng thái và Flash log.
#include "app_fault_event.h"

// Nạp giao diện chẩn đoán dòng lệnh USART.
#include "app_cli.h"

// Nạp giao diện giám sát sức khỏe hệ thống và watchdog độc lập (IWDG).
#include "app_health_monitor.h"

// Lưu ID thiết bị SPI Flash để kiểm tra giao tiếp phần cứng qua Keil Watch.
volatile uint16_t g_spi_flash_id = 0U;



// Điểm vào chương trình.
int main(void)
{
    /*
     * FreeRTOS sử dụng BASEPRI để quản lý các ngắt được phép gọi API của kernel.
     * Toàn bộ 4 bit ưu tiên của STM32F103 được dùng làm mức ưu tiên chiếm quyền (preemption priority), không chia sub-priority.
     * Hiện tại FreeRTOS đã tiếp quản lập lịch, mọi ngắt có thể gọi API kernel đều phải tuân thủ phân nhóm này.
     */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // Khởi tạo giao diện tương tác cục bộ trước để có thể quan sát trạng thái và lỗi về sau.
    LED_Init();
    KEY_Init();
    BUZZER_Init();
    /* TPS22919 mặc định tắt; chỉ khi BMS vào MONITOR mới cho phép cấp nguồn cho tải. */
    LoadSwitch_Init();
    USART1_Init();

    /*
    * Khởi tạo toàn bộ trạng thái phần mềm trước, sau đó mới khởi động nguồn dữ liệu phần cứng tương ứng.
    * Sensor Service và Fault Manager phải được xóa về 0 trước khi ADC sinh dữ liệu.
    */
    Sensor_ServiceInit();

    /* SOC được SensorTask ước lượng liên tục dựa trên điện áp hợp lệ, ở đây xóa trạng thái trước. */
    Soc_Init();
	
    FaultManager_Init();

    // Phải xóa hàng đợi sự kiện lỗi và bộ đếm chẩn đoán trước lần đánh giá lỗi đầu tiên.
    FaultEvent_Init();
	
    ADC_DMA_Init();

    /*
    * Chỉ sau khi khởi tạo SPI1 mới có thể truy cập Flash ngoài.
    * Nếu đọc ID khi chưa khởi tạo SPI sẽ bị treo khi chờ cờ truyền nhận SPI.
    */
    SPI1_Init();
    g_spi_flash_id = XM25QH32_ReadID();
    /*
     * Tạo Flash Mutex dùng chung cho log lỗi và lưu trữ SOC trước.
     * Scheduler chưa khởi chạy nên quá trình quét lúc này không xảy ra tranh chấp đa task, nhưng thứ tự khởi tạo vẫn giữ nhất quán.
     */
    FlashAccess_Init();

    /* Quét sector dành riêng cho log lỗi để khôi phục vị trí ghi an toàn tiếp theo. */
    FlashLog_Init();

    /*
     * Sau khi quét Flash hoàn tất, lưu ngay lần khởi động này và nguyên nhân reset.
     * Lúc này TIM3 chưa khởi động nên mốc thời gian của bản ghi khởi động cố định là 0ms.
     */
    (void)FlashLog_WriteBootEvent();

    // Khởi tạo phần cứng CAN; hiện vẫn tiếp tục dùng driver 500kbps phiên bản cũ.
    CAN1_Init();

    // Xóa hàng đợi sự kiện ứng dụng; các task FreeRTOS sẽ thiết lập mốc chu kỳ riêng sau khi khởi chạy.
    Event_Init();

    /*
     * Cuối cùng khởi động TIM3.
     * Kể từ thời điểm này, bộ đếm mili-giây hệ thống và kích hoạt chu kỳ 1ms của ADC đồng thời có hiệu lực.
     */
    TIM3_Init();

   /*
    * Chỉ sau khi mọi phần cứng, trạng thái phần mềm và bộ lập lịch chu kỳ đã sẵn sàng,
    * mới khởi động Health Monitor và watchdog độc lập (IWDG).
    *
    * Nếu không làm mới (refresh) bình thường trong khoảng 4 giây sau khi khởi động, STM32 sẽ tự động reset.
    */
    HealthMonitor_Init();

    /*
    * Tạo toàn bộ các task ứng dụng FreeRTOS ở giai đoạn hiện tại:
    * Health, Sensor, BMS, CAN, FaultLog và Diagnostic.
    *
    * Tạo thành công chỉ có nghĩa là các task đã được thêm vào danh sách task của FreeRTOS,
    * lúc này scheduler chưa khởi chạy nên hàm của các task chưa thực sự bắt đầu chạy.
    */
    if (AppRTOS_CreateTasks() == 0U)
    {
        /*
        * Tạo thất bại thường do thiếu bộ nhớ FreeRTOS Heap,
        * hoặc tham số task truyền vào không hợp lệ.
        *
        * Không quay lại đường dẫn lập lịch bare-metal cũ, nếu không sẽ che lấp lỗi tạo task RTOS.
        */
        printf("[RTOS] ERROR: task creation failed\r\n");

        for (;;)
        {
            /*
            * Giữ nguyên hiện trường lỗi.
            * IWDG đã được khởi động nên ở trạng thái không debug cuối cùng hệ thống sẽ tự động reset.
            */
        }
    }

    /*
    * Đến đây nghĩa là toàn bộ các task ứng dụng đã được tạo thành công.
    */
    printf("[SYSTEM] ECU-A FreeRTOS scheduler start\r\n");

    /*
    * CLI chỉ khởi tạo một lần.
    * Sau đó CLI_Process() được DiagnosticTask gọi theo chu kỳ.
    */
    CLI_Init();

    /*
    * Khởi chạy bộ lập lịch FreeRTOS.
    *
    * Từ đây trở đi:
    * 1. SysTick tạo nhịp (tick) FreeRTOS mỗi 1ms;
    * 2. Kernel chọn task ở trạng thái Ready để chạy dựa trên trạng thái và độ ưu tiên;
    * 3. Các task ứng dụng bắt đầu chạy theo chu kỳ hoặc điều kiện sự kiện riêng;
    * 4. Hàm main không còn tham gia vòng lặp nghiệp vụ bình thường nữa.
    */
    vTaskStartScheduler();

    /*
    * Trong điều kiện bình thường vTaskStartScheduler() không bao giờ trả về.
    *
    * Nếu thực thi đến đây, thường là do tạo Idle task thất bại,
    * nguyên nhân phổ biến nhất là thiếu bộ nhớ FreeRTOS Heap.
    */
    printf("[RTOS] ERROR: scheduler returned\r\n");

    for (;;)
    {
        /*
        * Giữ nguyên hiện trường lỗi và chờ IWDG reset.
        */
    }
}
