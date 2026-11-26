/*
 * Tên tệp: app_scheduler.c
 *
 * Nhiệm vụ của mô-đun:
 * 1. Tạm thời chạy chức năng chu kỳ LED chưa chuyển đổi;
 * 2. Mỗi khi chuyển đổi xong một tác vụ độc lập, xóa đầu vào tương ứng trong mô-đun này;
 * 3. Xóa bỏ mô-đun này sau khi chuyển đổi toàn bộ nghiệp vụ.
 *
 * Các chức năng đã chuyển đổi:
 * 1. SensorTask phụ trách dữ liệu cảm biến;
 * 2. BmsTask phụ trách đánh giá lỗi và máy trạng thái;
 * 3. CanTask phụ trách truyền CAN;
 * 4. FaultLogTask phụ trách lưu trữ bền vững sự kiện lỗi.
 * 5. HealthTask phụ trách kiểm tra nhịp tim tác vụ trọng yếu và làm mới IWDG.
 *
 * Ràng buộc quan trọng:
 * 1. Mô-đun này không được gọi lại Sensor_ServiceUpdate();
 * 2. Mô-đun này không được gọi lại BMS_StateMachine();
 * 3. Mô-đun này không được gọi lại Event_Process();
 * 4. Tránh để cùng một mô-đun nghiệp vụ có hai nơi thực thi.
 */

#include "app_scheduler.h"
#include "app_bms_state.h"
#include "bsp_tim.h"
#include "bsp_gpio.h"




static const uint16_t g_task_period[TASK_COUNT] = {
    500U,    // TASK_LED_HEARTBEAT — Nháy LED nhịp tim
};



// g_task_last_run — Dấu thời gian lần thực thi gần nhất của mỗi tác vụ (ms)
// So sánh với giá trị trả về của GetTick() để xác định đã đến thời điểm thực thi chưa
static uint32_t g_task_last_run[TASK_COUNT] = {0};


// Scheduler_Init — Khởi tạo bộ lập lịch
// Đặt lại thời gian thực thi gần nhất của mọi tác vụ về 0
// Mỗi tác vụ sẽ chạy lần đầu khi thời gian hệ thống đạt tới chu kỳ đầu tiên của nó
void Scheduler_Init(void)
{
	for(uint8_t i = 0; i< TASK_COUNT; i++)
	{
		g_task_last_run[i] = 0;
	}
}




/*
 * Scheduler_Run — Điểm vào lập lịch tương thích trong giai đoạn chuyển sang FreeRTOS.
 *
 * CompatibilityTask gọi hàm này mỗi 1ms một lần,
 * hàm này dựa vào GetTick() để kiểm tra xem nghiệp vụ chu kỳ còn lại đã đến hạn chưa.
 *
 * Chiến lược thực thi hiện tại:
 * 500ms: Đảo trạng thái LED nhịp tim khi không ở trạng thái lỗi.
 *
 * Sensor, BMS, CAN, FaultLog và Health đã chuyển sang các tác vụ độc lập,
 * hàm này không được gọi lại các điểm vào thực thi định kỳ của các mô-đun đó.
 */
void Scheduler_Run(void)
{
	// Lấy thời gian hiện tại
	uint32_t now = GetTick();

	for(uint8_t i = 0; i < TASK_COUNT; i++)
	{
		// Kiểm tra: thời gian hiện tại - lần thực thi trước >= chu kỳ tác vụ
		if(now - g_task_last_run[i] >= g_task_period[i])
		{
			// Cập nhật dấu thời gian
			g_task_last_run[i] = now;
			
			// Thực hiện thao tác tương ứng theo ID tác vụ
			switch (i)
				{
					/*
					* Xử lý LED vận hành thông thường mỗi 500ms một lần.
					* Khi BMS đang ở trạng thái lỗi, LED do logic chỉ báo lỗi điều khiển,
					* tại đây không đảo trạng thái LED nữa.
					*/
					case TASK_LED_HEARTBEAT:
					{
						if (BMS_GetState() != BMS_FAULT)
						{
							LED_Toggle();
						}

						break;
					}

					default:
						break;
				}
		}
	}
}
