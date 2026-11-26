/*
 * Tên file: app_flash_access.h
 * Tên mô-đun: Bảo vệ truy cập chia sẻ SPI Flash ngoài
 *
 * Trách nhiệm mô-đun:
 * Cung cấp cùng một khóa mutex FreeRTOS cho task nhật ký lỗi,
 * luồng lưu trữ SOC và các lệnh log/log clear của CLI, đảm bảo một quy trình
 * đọc, ghi, xóa hoặc xác thực Flash hoàn chỉnh không bị chen ngang bởi task khác.
 *
 * Tại sao sử dụng Mutex:
 * Thao tác Flash có thể kéo dài tương đối lâu, không thể dùng critical section dài chặn ngắt; Mutex sẽ làm
 * task chờ đi vào trạng thái Blocked, đồng thời giữ lại cơ chế thừa kế ưu tiên của FreeRTOS.
 *
 * Ranh giới mô-đun:
 * Mô-đun này không thực thi bất kỳ lệnh SPI Flash nào, cũng không đánh giá dữ liệu log hay SOC có hợp lệ không;
 * nó chỉ quản lý "ai hiện tại có quyền truy cập Flash".
 */
#ifndef APP_FLASH_ACCESS_H
#define APP_FLASH_ACCESS_H

#include <stdint.h>

/* Tạo Mutex tĩnh trước khi scheduler khởi động; gọi lặp lại sẽ không tạo lại. */
void FlashAccess_Init(void);

/*
 * Lấy quyền truy cập Flash.
 * Khi scheduler chưa chạy trả về 1 vì lúc này không có đồng thời giữa các task; sau khi scheduler chạy sẽ
 * chặn chờ Mutex. Trả về 0 khi khởi tạo bất thường hoặc trạng thái scheduler bất thường.
 */
uint8_t FlashAccess_Lock(void);

/* Giải phóng quyền truy cập Flash do task hiện tại nắm giữ; bắt buộc phải đi cặp với Lock thành công. */
void FlashAccess_Unlock(void);

#endif
