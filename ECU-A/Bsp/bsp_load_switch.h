#ifndef __BSP_LOAD_SWITCH_H
#define __BSP_LOAD_SWITCH_H

/*
 * Tên file: bsp_load_switch.h
 * Tên mô-đun: Driver cấp board (BSP) cho công tắc tải TPS22919
 *
 * Kết nối phần cứng: PB9 -> R5 -> TPS22919 ON.
 * Chân ON tích cực mức cao; khi khởi tạo và ở trạng thái ngắt đều xuất mức thấp.
 *
 * Trách nhiệm mô-đun: Chỉ chuyển đổi trạng thái "đóng/ngắt tải" thành mức logic trên chân PB9,
 * không phán đoán thời điểm được phép đóng tải. Việc có cho phép cấp nguồn hay không do máy trạng thái BMS quyết định.
 *
 * Giới hạn phần cứng hiện tại: TPS22919 là công tắc tải một chiều; mô-đun này chỉ thực hiện
 * minh họa đóng/ngắt tải trên mạch, không tương đương với mạch bảo vệ sạc/xả hoàn chỉnh dùng MOS đấu lưng.
 */
/* Khởi tạo PB9 làm ngõ ra đẩy-kéo (push-pull) và mặc định ở trạng thái ngắt để tránh đóng tải nhầm khi cấp nguồn. */
void LoadSwitch_Init(void);
/* Xuất mức cao trên PB9 để TPS22919 nối thông ngõ vào tới ngõ ra. */
void LoadSwitch_Enable(void);
/* Xuất mức thấp trên PB9 để TPS22919 ngắt tải. */
void LoadSwitch_Disable(void);

#endif /* __BSP_LOAD_SWITCH_H */
