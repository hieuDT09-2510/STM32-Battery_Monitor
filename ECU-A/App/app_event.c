/*
 * Tên tệp: app_event.c
 *
 * Vai trò mô-đun: Triển khai hàng đợi vòng RAM cho các lệnh trạng thái BMS.
 * Event_Push() phụ trách ghi lệnh, Event_Process() phụ trách lần lượt lấy lệnh và giao cho máy trạng thái.
 *
 * Quy tắc hàng đợi:
 * - head trỏ tới vị trí chờ đọc tiếp theo, tail trỏ tới vị trí chờ ghi tiếp theo;
 * - head == tail biểu thị hàng đợi rỗng;
 * - Để phân biệt hàng đợi đầy và rỗng, mảng 8 vị trí lưu tối đa 7 sự kiện;
 * - Khi hàng đợi đầy thì giữ sự kiện cũ, bỏ qua sự kiện mới;
 * - Được BmsTask gọi mỗi 10ms.
 *
 * Luồng ví dụ:
 * Event_Push(EVENT_FAULT) -> Event_Process()
 * -> BMS_EventHandle(EVENT_FAULT) -> Trạng thái BMS chuyển sang BMS_FAULT.
 */
#include "stm32f10x.h"
#include "app_event.h"
#include "app_bms_state.h"

// Dung lượng hàng đợi
#define EVENT_QUEUE_SIZE    8
// Bộ đệm sự kiện
static Event_t g_event_queue[EVENT_QUEUE_SIZE];
// Con trỏ đọc/ghi
static uint8_t g_event_head = 0;
static uint8_t g_event_tail = 0;

// Khởi tạo dữ liệu hàng đợi, đặt con trỏ đọc/ghi về 0
void Event_Init()
{
	 g_event_head = 0;
   g_event_tail = 0;
}




// Event_Push — Đẩy một sự kiện vào cuối hàng đợi
//
// Tham số: event -> sự kiện cần đẩy vào
//
// Quy trình hoạt động:
//   1. Nếu là EVENT_NONE thì bỏ qua trực tiếp (không xử lý sự kiện rỗng)
//   2. Tính vị trí con trỏ ghi tiếp theo: (tail + 1) % SIZE
//   3. Nếu vị trí tiếp theo khác head, chứng tỏ hàng đợi chưa đầy:
//        -> Ghi sự kiện vào vị trí tail hiện tại
//        -> Di chuyển tail sang vị trí tiếp theo
//   4. Nếu hàng đợi đã đầy, bỏ qua sự kiện này (không ghi đè dữ liệu cũ)
//
// Hiện tại chỉ cho phép gọi trong ngữ cảnh vòng lặp chính bare-metal; truy cập đồng thời cần vùng găng hoặc hàng đợi RTOS bảo vệ.

void Event_Push(Event_t event)
{
	// Bỏ qua sự kiện
	if(event == EVENT_NONE)
        return;

	// Tính vị trí tiếp theo
	uint8_t next = (g_event_tail + 1) % EVENT_QUEUE_SIZE;
	
	// Kiểm tra hàng đợi có đầy không
	if(next != g_event_head)
	{
		// Chưa đầy, lưu sự kiện
		g_event_queue[g_event_tail] = event;
		// Tiến con trỏ ghi
		g_event_tail = next;
		
	}
		
	// Hàng đợi đầy thì bỏ qua sự kiện
}


// Event_Pop — Lấy một sự kiện từ đầu hàng đợi
//
// Giá trị trả về: sự kiện lấy ra được; trả về EVENT_NONE khi hàng đợi rỗng
//
// Quy trình hoạt động:
//   1. Nếu head == tail, hàng đợi rỗng, trả về EVENT_NONE
//   2. Lấy sự kiện tại vị trí head
//   3. Di chuyển head sang vị trí tiếp theo
//   4. Trả về sự kiện đã lấy
//
// Gọi trong vòng lặp main, không gọi trong ngắt

Event_t Event_Pop(void)
{
	// Kiểm tra hàng đợi có rỗng không
	if(g_event_head == g_event_tail)
		return EVENT_NONE;
	
	// Lấy sự kiện tại vị trí HEAD hiện tại
	Event_t event = g_event_queue[g_event_head];
	
	// Tiến con trỏ đọc lên trước
	g_event_head = (g_event_head + 1) % EVENT_QUEUE_SIZE;
	
	return event;
}



// Event_Available — Kiểm tra xem trong hàng đợi có sự kiện không
// Giá trị trả về: 0 = hàng đợi rỗng, khác 0 = có sự kiện

uint8_t Event_Available(void)
{
	// head != tail nghĩa là hàng đợi không rỗng
	return (g_event_head != g_event_tail);
	
}



// Event_Process — Xử lý tất cả các sự kiện đang chờ trong hàng đợi
//
// Lặp lấy từng sự kiện trong hàng đợi và giao cho BMS_EventHandle xử lý
// Được bộ lập lịch gọi mỗi 20ms một lần
//
// Quy trình hoạt động:
//   while(hàng đợi không rỗng)
//   {
//       Lấy ra một sự kiện
//       Giao cho BMS_EventHandle() xử lý chuyển trạng thái
//   }

void Event_Process(void)
{
	// Lặp qua toàn bộ sự kiện trong hàng đợi
	while(Event_Available())
	{
		// Lấy ra một sự kiện
		Event_t evt = Event_Pop();
		// Giao cho mô-đun máy trạng thái xử lý
    // BMS_EventHandle sẽ căn cứ vào loại sự kiện và trạng thái hiện tại
    // để quyết định có chuyển trạng thái hay không
    BMS_EventHandle(evt);
	}
}
