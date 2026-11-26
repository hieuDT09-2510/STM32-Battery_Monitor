/*
 * Tên tệp: app_soc.h
 * Tên mô-đun: Dịch vụ ước tính SOC theo điện áp
 *
 * Nhiệm vụ của mô-đun:
 * 1. Ước tính SOC từ điện áp pin bằng nội suy tuyến tính;
 * 2. Phát hành một snapshot SOC nhất quán để CAN, CLI và nhật ký lỗi cùng đọc.
 *
 * Mô hình ước tính:
 * cell_mv <= 3050mV được xem là 0%, cell_mv >= 4150mV được xem là 100%,
 * ở giữa thực hiện nội suy tuyến tính giữa hai điểm. Mô hình này không có thông tin dòng điện,
 * đường cong OCV-SOC thực tế không hoàn toàn tuyến tính, sụt áp tải và nhiệt độ cũng gây sai số,
 * nên kết quả chỉ mang tính tham khảo sơ bộ.
 *
 * Phạm vi mô-đun:
 * Mô-đun này không đọc ADC, không truy cập Flash, cũng không phụ trách truyền CAN hay hiển thị OLED.
 * Mỗi lần SOC đều được tính lại từ điện áp hiện tại, không có trạng thái tích phân, nên không cần lưu khi mất nguồn.
 */
#ifndef APP_SOC_H
#define APP_SOC_H

#include <stdint.h>

/* Điểm đầu mút nội suy tuyến tính: <= điện áp này là 0%. */
#define SOC_EMPTY_VOLTAGE_MV         ((uint16_t)3050U)
/* Điểm đầu mút nội suy tuyến tính: >= điện áp này là 100%. */
#define SOC_FULL_VOLTAGE_MV          ((uint16_t)4150U)

/* Snapshot SOC vận hành phục vụ CAN, CLI và chẩn đoán. */
typedef struct
{
    uint32_t sequence;           /* Số thứ tự cập nhật; 0 nghĩa là chưa phát hành SOC hợp lệ. */
    uint32_t timestamp_ms;       /* Dấu thời gian của mẫu điện áp dùng cho lần cập nhật gần nhất. */
    uint16_t cell_mv;            /* Điện áp pin dùng để ước tính, đơn vị mV. */
    uint16_t soc_centi_percent;  /* 0~10000, tương ứng 0.00%~100.00%. */
    uint8_t valid;               /* 1 = đã ước tính được SOC từ điện áp hợp lệ. */
} SocSnapshot_t;

/* Xóa trạng thái SOC, chờ điện áp hợp lệ đầu tiên. */
void Soc_Init(void);

/* Hàm thuần túy: điện áp mV -> SOC (đơn vị 0.01%, 0~10000), thuận tiện cho unit test trên host. */
uint16_t Soc_CalculateCentiPercent(uint16_t cell_mv);

/* Cập nhật snapshot SOC bằng một giá trị điện áp hợp lệ; khi điện áp không hợp lệ, bên gọi không gọi hàm này. */
void Soc_UpdateFromVoltage(uint16_t cell_mv, uint32_t timestamp_ms);

/* Sao chép snapshot hợp lệ hiện tại; thành công trả về 1, tham số rỗng hoặc chưa có SOC trả về 0. */
uint8_t Soc_GetSnapshot(SocSnapshot_t *snapshot);

#endif
