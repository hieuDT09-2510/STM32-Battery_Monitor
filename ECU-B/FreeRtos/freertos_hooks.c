#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"

/**
 * @brief Hàm callback khi tràn ngăn xếp (stack overflow) của task FreeRTOS
 *
 * @param xTask
 *        Handle của task bị tràn ngăn xếp.
 *
 * @param pcTaskName
 *        Tên của task bị tràn ngăn xếp.
 *
 * @note
 * Khi configCHECK_FOR_STACK_OVERFLOW được đặt là 1 hoặc 2,
 * ứng dụng bắt buộc phải triển khai hàm này.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   char *pcTaskName)
{
    /*
     * Tránh trình biên dịch cảnh báo tham số không được sử dụng.
     * Khi debug có thể quan sát trực tiếp hai tham số này.
     */
    (void)xTask;
    (void)pcTaskName;

    /*
     * Vào hàm này cho thấy ngăn xếp của một task nào đó đã gặp rủi ro nghiêm trọng.
     * Tiếp tục chạy lúc này có thể gây ra:
     *
     * 1. TCB bị phá hủy;
     * 2. Ngăn xếp của các task khác bị phá hủy;
     * 3. Địa chỉ trả về bị ghi đè;
     * 4. Chương trình rơi vào HardFault;
     * 5. Hệ thống phát sinh hành vi không thể dự đoán.
     *
     * Do đó tại đây tắt ngắt trước và dừng máy,
     * giữ nguyên hiện trường để trình debug Keil kiểm tra.
     */
    taskDISABLE_INTERRUPTS();

    for (;;)
    {
        /*
         * Đặt breakpoint tại đây.
         *
         * Tập trung quan sát:
         * pcTaskName: task nào bị tràn;
         * xTask: handle của task tương ứng.
         */
    }
}
/**
 * @brief Cung cấp giá trị Canary bảo vệ con trỏ heap cho FreeRTOS heap_4
 *
 * @param pxHeapCanary
 *        Con trỏ dùng để trả về giá trị Canary.
 *
 * @note
 * Khi configENABLE_HEAP_PROTECTOR được đặt là 1,
 * ứng dụng bắt buộc phải triển khai hàm này.
 */
void vApplicationGetRandomHeapCanary(
    portPOINTER_SIZE_TYPE *pxHeapCanary)
{
    /*
     * Trước tiên kiểm tra con trỏ đầu ra do FreeRTOS truyền vào có hợp lệ hay không.
     */
    configASSERT(pxHeapCanary != NULL);

    /*
     * Project cơ sở STM32F103 hiện tại chưa có bộ tạo số ngẫu nhiên phần cứng.
     *
     * Bản thân STM32F103C8 không có ngoại vi RNG, vì vậy tại đây tạm sử dụng:
     * Giá trị cố định khác không
     * XOR
     * SystemCoreClock
     * XOR
     * Địa chỉ biến đầu ra hiện tại
     *
     * Mục đích chủ yếu là tránh Canary bằng 0, đồng thời giúp các bố cục firmware khác nhau
     * nhận được giá trị không hoàn toàn giống nhau.
     *
     * Lưu ý:
     * Đây không phải số ngẫu nhiên an toàn mật mã.
     * Mục tiêu của heap protector là tăng khả năng phát hiện hỏng con trỏ heap,
     * không dùng cho mục đích mã hóa.
     */
    *pxHeapCanary =
        (portPOINTER_SIZE_TYPE)0xA5F03C96UL
        ^ (portPOINTER_SIZE_TYPE)SystemCoreClock
        ^ (portPOINTER_SIZE_TYPE)pxHeapCanary;

    /*
     * Phòng trường hợp cực đoan kết quả tính toán vừa đúng bằng 0.
     */
    if (*pxHeapCanary == (portPOINTER_SIZE_TYPE)0U)
    {
        *pxHeapCanary =
            (portPOINTER_SIZE_TYPE)0x5A3CC3A5UL;
    }
}
