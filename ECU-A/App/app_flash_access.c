/*
 * Tên file: app_flash_access.c
 *
 * Tất cả các task truy cập SPI Flash ngoài bắt buộc phải thông qua file này để lấy cùng một Mutex.
 * Khóa bảo vệ toàn bộ một transaction logic, ví dụ "ghi xong đọc lại xác thực", chứ không chỉ riêng một lần
 * truyền byte SPI, nếu không task khác có thể thay đổi nội dung Flash giữa lúc ghi và xác thực.
 */
#include "app_flash_access.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Control block của Mutex tĩnh, được RAM ứng dụng cung cấp, không cấp phát động từ Heap FreeRTOS. */
static StaticSemaphore_t g_flash_mutex_control;

/* Handle Mutex; 0 biểu thị FlashAccess_Init chưa tạo thành công mutex. */
static SemaphoreHandle_t g_flash_mutex_handle = 0;

void FlashAccess_Init(void)
{
    /* Ngăn chặn khởi tạo lặp lại ghi đè lên handle đã có hoặc phá hủy trạng thái Mutex đang dùng. */
    if (g_flash_mutex_handle == 0)
    {
        /* Sử dụng API tạo tĩnh, dung lượng bộ nhớ cố định, thuận tiện cho hệ thống nhúng đánh giá Heap. */
        g_flash_mutex_handle =
            xSemaphoreCreateMutexStatic(&g_flash_mutex_control);
    }
}

uint8_t FlashAccess_Lock(void)
{
    /* Truy vấn trạng thái scheduler để tương thích với luồng quét Flash trong giai đoạn khởi động. */
    BaseType_t scheduler_state = xTaskGetSchedulerState();

    /*
     * Trước khi scheduler khởi động chỉ có context của main, không xảy ra đồng thời với task khác, nên không cần Take.
     * Điều này cũng cho phép hoàn thành quét log/SOC trong giai đoạn bật nguồn trước khi tạo task.
     */
    if (scheduler_state == taskSCHEDULER_NOT_STARTED)
    {
        return 1U;
    }

    if ((scheduler_state != taskSCHEDULER_RUNNING) ||
        (g_flash_mutex_handle == 0))
    {
        return 0U;
    }

    /*
     * Chờ vô hạn được thực hiện bằng cách đưa task gọi vào trạng thái Blocked, không chiếm CPU chờ bận.
     * Mutex còn cung cấp cơ chế thừa kế ưu tiên, giảm nguy cơ đảo ngược quyền ưu tiên do task ưu tiên thấp giữ khóa.
     */
    return (xSemaphoreTake(g_flash_mutex_handle, portMAX_DELAY) == pdTRUE) ?
           1U : 0U;
}

void FlashAccess_Unlock(void)
{
    /* Trước khi scheduler khởi động Lock không thực sự lấy Mutex, nên ở đây cũng không thực hiện Give. */
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    {
        return;
    }

    if (g_flash_mutex_handle != 0)
    {
        (void)xSemaphoreGive(g_flash_mutex_handle);
    }
}

    /* Khi scheduler bị suspend hoặc Mutex chưa được tạo thì không thể chặn chờ an toàn. */
    /* Trả lại Mutex khi handle hợp lệ, đánh thức task có độ ưu tiên cao nhất đang chờ Flash. */
