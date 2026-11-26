/*
 * Mô-đun: Máy trạng thái hệ thống giám sát pin.
 * Trạng thái: INIT, SELF_CHECK, STANDBY, MONITOR, FAULT.
 * Chức năng: Tổ chức khởi tạo, tự kiểm tra, giám sát, cảnh báo lỗi, yêu cầu khôi phục và hiển thị trạng thái cục bộ.
 * Phạm vi cơ cấu chấp hành: Chỉ điều khiển công tắc tải tích hợp TPS22919 và bộ báo động, không có MOS đấu lưng (back-to-back),
 * do đó không thể độc lập ngắt sạc ngoài và không tương đương với cơ chế bảo vệ sạc/xả BMS hoàn chỉnh.
 * Phạm vi dữ liệu: Chỉ đọc snapshot do Sensor Service phát hành, không sử dụng trực tiếp các cờ ADC/DMA.
 *
 * Phạm vi giao tiếp:
 * Mô-đun này chỉ tạo ra trạng thái BMS hiện tại và trạng thái lỗi đang kích hoạt,
 * không đóng gói bản tin ứng dụng CAN, không quản lý Alive Counter và không gọi driver truyền CAN.
 */
// Nguyên tắc thiết kế:
//   1. Task chỉ kiểm tra điều kiện, không thay đổi trạng thái trực tiếp
//   2. Việc chuyển trạng thái được thực hiện tập trung trong BMS_EventHandle
//   3. Tách biệt việc kiểm tra và chuyển trạng thái thông qua Event_Push/Event_Process

#include "app_bms_state.h"
#include "app_event.h"
#include "bsp_gpio.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "bsp_usart.h"
#include "app_flash_log.h"
#include "app_sensor.h"
#include "bsp_buzzer.h"
#include "bsp_load_switch.h"
#include "app_fault_manager.h"
#include "app_fault_event.h"


// ─── Biến toàn cục của máy trạng thái ───
static BMS_State_t g_bms_state = BMS_INIT;  // Trạng thái BMS hiện tại
uint16_t g_fault_code = 0U;       // Mặt nạ lỗi đang kích hoạt 16-bit hiện tại, đồng nhất với active_mask của trình quản lý lỗi
// Ghi lại tự kiểm tra khi cấp nguồn lần này có thất bại hay không; nếu tự kiểm tra thất bại thì yêu cầu reset hệ thống để tự kiểm tra lại.
static uint8_t g_self_check_failed = 0U;









//====================================Giao diện trạng thái=====================================

// BMS_SetState — Thiết lập trạng thái BMS (sửa trực tiếp biến trạng thái)
// Được gọi bởi BMS_EventHandle, Task không gọi trực tiếp (ngoại trừ INIT/SELF_CHECK)
void BMS_SetState(BMS_State_t state)
 { 
    // Sửa trực tiếp biến trạng thái toàn cục
	 g_bms_state = state; 

     /*
      * Tải chỉ được phép cấp nguồn ở trạng thái giám sát bình thường.
      * INIT, SELF_CHECK, STANDBY và FAULT đều giữ trạng thái tắt.
      * Mọi thay đổi trạng thái đều đi qua hàm này, do đó chiến lược cơ cấu chấp hành không bị phân tán ở các hàm xử lý từng trạng thái.
      * TPS22919 chỉ là cơ cấu chấp hành đóng/ngắt tải trên mạch, không tương đương với bảo vệ MOS sạc/xả hoàn chỉnh.
      */
     if (state == BMS_MONITOR)
     {
         LoadSwitch_Enable();
     }
     else
     {
         LoadSwitch_Disable();
     }

     //Kiểm tra trạng thái mới có phải là trạng thái lỗi không
     if(state == BMS_FAULT)
     {
         // Khi vào trạng thái lỗi, tắt LED, bật còi báo
         LED_OFF();
         BUZZER_On();
     }
     else
     {
         // Không phải trạng thái lỗi, tắt còi báo
         BUZZER_Off();
     }
 }


// BMS_GetState — Lấy trạng thái BMS hiện tại
// Dùng cho Event_Process kiểm tra trạng thái hiện tại có cho phép chuyển đổi không
BMS_State_t BMS_GetState(void) 
 {
		return g_bms_state;
 }


// Xử lý sự kiện — Mọi chuyển đổi trạng thái đều tập trung tại đây
// Được thực thi gián tiếp sau khi BmsTask gọi Event_Process mỗi 10ms
// Task chỉ Push sự kiện, không gọi trực tiếp BMS_SetState
void BMS_EventHandle(Event_t event)
{
    switch(event)
    {

        /*
         * Mô-đun: Xử lý sự kiện khởi động, dừng và lỗi.
         * Khởi động và dừng chỉ chuyển đổi trạng thái giám sát phần mềm, không điều khiển dòng điện pin.
         * Khi nhận được sự kiện lỗi trong lúc chờ hoặc đang giám sát, chuyển vào trạng thái cảnh báo lỗi.
         */
         // Sau khi nhận yêu cầu khởi động ở trạng thái chờ, chuyển vào trạng thái giám sát.
        case EVENT_START:
            if (BMS_GetState() == BMS_STANDBY)
            {
                BMS_SetState(BMS_MONITOR);
            }
            break;

        // Sau khi nhận yêu cầu dừng ở trạng thái giám sát, quay về trạng thái chờ.
        case EVENT_STOP:
            if (BMS_GetState() == BMS_MONITOR)
            {
                BMS_SetState(BMS_STANDBY);
            }
            break;

        // Sau khi nhận sự kiện lỗi ở trạng thái chờ hoặc giám sát, chuyển vào trạng thái lỗi.
        case EVENT_FAULT:
            if ((BMS_GetState() == BMS_STANDBY) ||
                (BMS_GetState() == BMS_MONITOR))
            {
                BMS_SetState(BMS_FAULT);
            }
            break;
        // ── Khôi phục lỗi: FAULT → STANDBY ──
       /*
        * Mô-đun: Xử lý khôi phục thủ công của trình quản lý lỗi thống nhất.
        * Chức năng: PA2 chỉ đưa ra yêu cầu xóa; trình quản lý dựa trên trạng thái chốt và trạng thái ổn định khôi phục
        *            để quyết định lỗi nào có thể xóa. Nếu vẫn còn lỗi nghiêm trọng thì tiếp tục giữ BMS_FAULT.
        */
        case EVENT_RECOVER:
        {
            FaultManagerSnapshot_t fault_snapshot;//Lưu kết quả của trình quản lý lỗi trước hoặc sau khi xóa
            SensorSample_t event_sample;    //Lưu snapshot điện áp, nhiệt độ tại thời điểm xóa lỗi
            uint16_t cleared_mask;  //Những lỗi nào thực tế đã được xóa bởi lần nhấn phím này
            uint8_t event_sample_available; //Đã lấy được snapshot cảm biến thành công hay chưa

            // Không ở trạng thái lỗi thì không chấp nhận yêu cầu khôi phục thủ công.
            if (BMS_GetState() != BMS_FAULT)
            {
                break;
            }

            // Đọc snapshot của trình quản lý lỗi trước khi xóa.
            if (FaultManager_GetSnapshot(&fault_snapshot) == 0U)
            {
                break;
            }

            /*
            * Yêu cầu xóa toàn bộ các lỗi đã chốt (latched).
            * Bên trong trình quản lý sẽ chỉ xóa những lỗi đã vào recovery_ready_mask,
            * các lỗi chưa hồi phục sẽ không bị xóa cưỡng bức bằng phím nhấn.
            */
            cleared_mask =
                FaultManager_RequestManualClear(fault_snapshot.latched_mask);

            // Đọc lại trạng thái lỗi thực tế sau khi xóa.
            if (FaultManager_GetSnapshot(&fault_snapshot) == 0U)
            {
                break;
            }

            /*
             * Sự kiện xóa lỗi cần lưu lại các giá trị đo "tại thời điểm xảy ra xóa lỗi".
             * Ở đây sao chép ngay snapshot gần nhất; khi UART hoặc Flash tiêu thụ dữ liệu sau đó sẽ không đọc cảm biến thời gian thực nữa.
             */
            event_sample_available = Sensor_GetLatest(&event_sample);

            if (fault_snapshot.newly_cleared_mask != FAULT_MASK_NONE)
            {
                (void)FaultEvent_Push(
                    FAULT_EVENT_CLEARED,
                    fault_snapshot.newly_cleared_mask,
                    fault_snapshot.active_mask,
                    (event_sample_available != 0U) ? &event_sample : 0,
                    (uint8_t)BMS_GetState(),
                    GetTick());
            }

            // Sau khi xóa, đồng bộ ngay mặt nạ lỗi đang hoạt động 16-bit đầy đủ để hiển thị trạng thái và gửi khung 0x303.
            g_fault_code = fault_snapshot.active_mask;

            /*
            * Hàm này chỉ cập nhật g_fault_code, không gửi CAN trực tiếp.
            *
            * Sau khi BmsTask trở về từ máy trạng thái sẽ so sánh xem mặt nạ lỗi có thay đổi hay không.
            * Nếu có thay đổi, BmsTask đánh thức CanTask thông qua task notification,
            * để CanTask gửi ngay trạng thái lỗi đầy đủ mới nhất.
            *
            * Cách này đảm bảo phần cứng CAN luôn chỉ có một tác vụ thực thi duy nhất là CanTask.
            */
            // Xuất thông tin chẩn đoán khi có lỗi thực sự được xóa.
            if (cleared_mask != FAULT_MASK_NONE)
            {
                printf("[BMS] Fault cleared: 0x%04X\r\n",
                    (unsigned int)cleared_mask);
            }

            /*
            * Chỉ khi toàn bộ các lỗi nghiêm trọng đã được xóa, hệ thống mới được phép rời khỏi trạng thái FAULT.
            * Nếu chỉ xóa được một phần, còi báo và trạng thái lỗi tiếp tục được duy trì.
            */
            if (fault_snapshot.critical_mask == FAULT_MASK_NONE)
            {
                BMS_SetState(BMS_STANDBY);
            }

            break;
        }

    
        default:
            break;
    }
}

/*
 * Mô-đun: Bộ thích ứng giữa máy trạng thái và trình quản lý lỗi hợp nhất.
 * Tác dụng: Gửi snapshot cảm biến mới nhất và kết quả tự kiểm tra, đọc mặt nạ lỗi tổng hợp,
 * và gửi sự kiện FAULT vào hàng đợi sự kiện của máy trạng thái khi tồn tại lỗi nghiêm trọng.
 * Phạm vi giới hạn: Hàm này không bật còi báo trực tiếp, cũng không chuyển đổi trạng thái BMS trực tiếp.
 *
 * Lấy dữ liệu từ Sensor Service
            ↓
 * Giao cho Fault Manager phán đoán lỗi
            ↓
 *   Nhận kết quả phán đoán
        ┌───┴─────────┐
        ↓             ↓
 * Lưu bản ghi lỗi    Yêu cầu máy trạng thái vào FAULT
 * FaultEvent_Push     Event_Push
 * (một bản ghi lỗi đầy đủ) (một lệnh chuyển trạng thái)
 *
 * Trách nhiệm của các hàm:
 * FaultManager_UpdateCondition()--Xử lý xác nhận/chốt/khôi phục lỗi
 * FaultManager_UpdateSensor()--Kiểm tra quá áp/thấp áp/quá nhiệt/snapshot quá hạn
 * BMS_StateMachine()--Lấy snapshot mới nhất
 * FaultManager_EndEvaluation()--Tổng hợp nhiều lỗi
 */
static void BMS_UpdateSensorFaultManager(void)
{
    SensorSample_t sample;//Lưu dữ liệu cảm biến của chu kỳ này
    FaultManagerSnapshot_t fault_snapshot;//Lưu kết quả phán đoán của trình quản lý lỗi trong chu kỳ này
    uint8_t sample_available;//Cho biết đã lấy được snapshot cảm biến hay chưa
    uint32_t now_ms;//Lưu thời gian hệ thống hiện tại (mili-giây)
    /*
    * Lưu số lần Bus-Off đã giao cho trình quản lý lỗi xử lý ở lần trước.
    * Từ khóa static giúp biến giữ nguyên giá trị sau khi thoát hàm.
    */
    static uint32_t last_seen_bus_off_count = 0U;

    uint32_t can_esr;             // Snapshot thanh ghi trạng thái lỗi CAN của chu kỳ này
    uint8_t can_degraded;         // CAN hiện tại có đang ở trạng thái suy giảm truyền thông hay không
    uint8_t can_bus_off_event;    // Trong chu kỳ này có mới xảy ra Bus-Off hay không
    uint8_t can_healthy;          // CAN hiện tại đã khôi phục trạng thái bình thường hay chưa

    now_ms = GetTick();

    // Lấy một khung snapshot do Sensor Service phát hành gần nhất.
    sample_available = Sensor_GetLatest(&sample);

    /*
    * Chỉ đọc thanh ghi ESR một lần, đảm bảo mọi phán đoán CAN trong chu kỳ này sử dụng trạng thái tại cùng một thời điểm.
    */
    can_esr = CAN1->ESR;

    /*
    * EWGF: Bộ đếm lỗi đã vào ngưỡng cảnh báo.
    * EPVF: Bộ điều khiển đã vào trạng thái Error Passive.
    *
    * Bus-Off được xử lý bởi một mục lỗi độc lập, do đó khi có BOFF sẽ không báo trùng CAN_DEGRADED.
    */
    can_degraded =
        (uint8_t)(
            ((can_esr & (CAN_ESR_EWGF | CAN_ESR_EPVF)) != 0U) &&
            ((can_esr & CAN_ESR_BOFF) == 0U)
        );

    /*
    * g_can_bus_off_count là giá trị tích lũy, không thể lấy trực tiếp điều kiện "khác 0" làm điều kiện kích hoạt.
    * Nếu không, sau khi xảy ra một lần Bus-Off, điều kiện kích hoạt sẽ vĩnh viễn bằng 1.
    *
    * Ở đây chỉ tạo một xung kích hoạt khi số lần tích lũy thay đổi.
    */
    if (g_can_bus_off_count != last_seen_bus_off_count)
    {
        can_bus_off_event = 1U;
        last_seen_bus_off_count = g_can_bus_off_count;
    }
    else
    {
        can_bus_off_event = 0U;
    }

    /*
    * CAN khôi phục trạng thái bình thường phải thỏa mãn đồng thời:
    * 1. Không ở trạng thái cảnh báo, Error Passive hoặc Bus-Off;
    * 2. Driver hiện tại không ở trạng thái Bus-Off;
    * 3. Bản tin gần nhất đã được gửi thành công.
    */
    can_healthy =
        (uint8_t)(
            ((can_esr &
            (CAN_ESR_EWGF | CAN_ESR_EPVF | CAN_ESR_BOFF)) == 0U) &&
            (g_can_bus_off_active == 0U) &&
            (g_can_last_send_result == CAN_SEND_OK)
        );


    /*
     * Cập nhật các lỗi điện áp, nhiệt độ, NTC, tính nhất quán dữ liệu và quá thời gian chờ (timeout) ADC.
     * Khi không có snapshot thì truyền con trỏ NULL để trình quản lý tự phán đoán timeout ADC.
     */
    FaultManager_UpdateSensor(
        (sample_available != 0U) ? &sample : 0,
        sample_available,
        now_ms);

    /*
    * Suy giảm truyền thông CAN:
    * Điều kiện duy trì liên tục 500ms mới xác nhận, sau khi khôi phục bình thường 1000ms sẽ tự động xóa.
    */
    FaultManager_UpdateCondition(
        FAULT_ID_CAN_DEGRADED,
        can_degraded,
        can_healthy,
        now_ms);

    /*
    * CAN Bus-Off:
    * Xác nhận ngay lập tức khi số lần tích lũy của driver thay đổi;
    * Sau khi ABOM khôi phục và duy trì bình thường liên tục 1000ms, cho phép xác nhận thủ công để xóa.
    */
    FaultManager_UpdateCondition(
        FAULT_ID_CAN_BUS_OFF,
        can_bus_off_event,
        can_healthy,
        now_ms);

    /*
     * Đưa cả kết quả tự kiểm tra khi cấp nguồn (POST) vào cùng một trình quản lý.
     * Sau khi tự kiểm tra thất bại, trạng thái trip được giữ nguyên, chỉ có thể khôi phục bằng cách cấp nguồn lại và tự kiểm tra lại.
     */
    FaultManager_UpdateCondition(
        FAULT_ID_SELF_CHECK,
        g_self_check_failed,
        (uint8_t)(g_self_check_failed == 0U),
        now_ms);

    /*
     * Bên trong UpdateSensor đã hoàn thành một lần tổng hợp.
     * Sau khi bổ sung điều kiện CAN và SELF_CHECK, mới thống nhất tạo mặt nạ lỗi cuối cùng.
     */
    FaultManager_EndEvaluation();

    // Lấy kết quả lỗi đầy đủ của chu kỳ này.
    if (FaultManager_GetSnapshot(&fault_snapshot) == 0U)
    {
        return;
    }

    /*
     * Chỉ lưu sự kiện tại sườn xác nhận lỗi lần đầu tiên.
     * active_mask đang tồn tại liên tục sẽ không bị ghi lặp lại, tránh cùng một lỗi cứ mỗi 10ms lại tạo một bản ghi log.
     */
    if (fault_snapshot.newly_confirmed_mask != FAULT_MASK_NONE)
    {
        (void)FaultEvent_Push(
            FAULT_EVENT_CONFIRMED,
            fault_snapshot.newly_confirmed_mask,
            fault_snapshot.active_mask,
            (sample_available != 0U) ? &sample : 0,
            (uint8_t)BMS_GetState(),
            now_ms);
    }

    // Mỗi chu kỳ đồng bộ đầy đủ mặt nạ lỗi đang hoạt động 16-bit, không còn cắt bỏ 8 bit cao.
    g_fault_code = fault_snapshot.active_mask;

    /*
    * Hàm này chỉ cập nhật trạng thái lỗi, không gửi CAN trực tiếp.
    *
    * Bản tin trạng thái lỗi do CanTask thống nhất gửi đi:
    * 1. Trong trường hợp bình thường, gửi định kỳ mỗi 100ms một lần;
    * 2. Khi mặt nạ lỗi thay đổi, BmsTask thông báo cho CanTask gửi ngay lập tức.
    *
    * Việc thống nhất một tác vụ thực thi giúp tránh nhiều tác vụ cùng lúc truy cập driver CAN,
    * đồng thời tránh việc nhiều tác vụ cùng lúc sửa đổi Alive Counter của bản tin lỗi.
    */

    /*
     * Chỉ cần lỗi nghiêm trọng vẫn còn tồn tại và chưa chuyển vào FAULT, tiếp tục phát sự kiện FAULT lặp lại.
     * Nhờ vậy ngay cả khi hàng đợi sự kiện tạm thời bị đầy ở một chu kỳ nào đó, chu kỳ tiếp theo vẫn sẽ thử lại.
     * EVENT_FAULT là sự kiện lũy đẳng, nhận thừa một lần cũng không gây chuyển trạng thái sai.
     */
    if ((fault_snapshot.critical_mask != FAULT_MASK_NONE) &&
        (BMS_GetState() != BMS_FAULT))
    {
        Event_Push(EVENT_FAULT);
    }
}


/*
 * Task_Init — Thực hiện khởi tạo phần mềm một lần cho máy trạng thái BMS.
 *
 * Hàm này chỉ xuất thông tin khởi động và chuyển vào SELF_CHECK,
 * không dùng vòng lặp trễ rỗng để tránh tác vụ ưu tiên cao BmsTask chiếm dụng CPU trong thời gian dài.
 *
 * Trạng thái hoạt động của hệ thống được biểu thị qua LED nhịp tim và thông tin khởi động qua cổng nối tiếp sau đó.
 */
static void Task_Init(void)
{
    printf("[BMS] INIT - Initializing...\r\n");

    /*
     * INIT không có nguồn sự kiện bên ngoài,
     * sau khi hoàn tất khởi tạo giai đoạn này sẽ chuyển trực tiếp vào SELF_CHECK.
     */
    BMS_SetState(BMS_SELF_CHECK);
}



// Task_SelfCheck — Tác vụ tự kiểm tra
// Ghi lại dấu thời gian khi vào lần đầu, trễ 100ms chờ ADC/DMA ổn định
// Kiểm tra sau 100ms:
//   1. ADC+DMA có hoạt động bình thường không (kiểm tra cờ ngắt nửa/toàn phần truyền)
//   2. Bus CAN có bình thường không (kiểm tra thanh ghi trạng thái lỗi ESR)
// Đạt→STANDBY, thất bại→FAULT
static void Task_SelfCheck(void)
{
    uint32_t now_ms = GetTick();
    SensorSample_t sample;
    uint32_t can_msr;   // Chế độ hoạt động hiện tại của CAN
    uint32_t can_esr;   // Trạng thái lỗi hiện tại của CAN
    uint8_t fail = 0U;  // Kết quả tổng thể tự kiểm tra của chu kỳ này

    // Dấu thời gian bắt đầu tự kiểm tra (static, chỉ khởi tạo một lần)
    static uint32_t selfcheck_start = 0;

    // ── Giai đoạn 1: Ghi lại thời gian bắt đầu, chờ ADC/DMA ổn định ──
    if(selfcheck_start == 0)
    {
        selfcheck_start = GetTick();  // Ghi lại thời gian hiện tại
        return;                       // Lần đầu không kiểm tra, chờ lần vào tiếp theo
    }

    // ── Giai đoạn 2: Chờ 100ms, để DMA hoàn thành ít nhất một lần truyền nửa bộ đệm ──
    if(GetTick() - selfcheck_start < 100)
    {
        return;  // Chưa đủ 100ms, tiếp tục chờ
    }

    // ── Giai đoạn 3: Đã đủ 100ms, bắt đầu kiểm tra ──
    // Kiểm tra ADC+DMA: bất kỳ cờ ngắt nào từng được bật lên chứng tỏ hoạt động bình thường
    // Phải đã phát hành bản chụp (snapshot) trước đó, và khung cuối cùng không được quá 100ms.
    if ((Sensor_GetLatest(&sample) == 0U) ||
        (Sensor_IsFresh(&sample, now_ms, 100U) == 0U))
    {
        printf("[BMS] SENSOR SNAPSHOT FAIL\r\n");
        fail = 1U;
    }


    /*
    * Tự kiểm tra CAN khi cấp nguồn chỉ kiểm tra xem bộ điều khiển có đang ở trạng thái sẵn sàng hoạt động hay không:
    *
    * INAK=1: Bộ điều khiển vẫn đang dừng ở chế độ khởi tạo;
    * SLAK=1: Bộ điều khiển vẫn đang ở chế độ ngủ;
    * ABOM=0: Chưa bật tự động khôi phục Bus-Off;
    * BOFF=1: Bộ điều khiển hiện tại đã rơi vào trạng thái Bus-Off.
    *
    * LEC chỉ là loại lỗi gần nhất, không đại diện cho việc CAN hiện tại có hoạt động được hay không,
    * do đó ở đây không còn sử dụng LEC làm điều kiện thất bại của tự kiểm tra khi cấp nguồn.
    */
    can_msr = CAN1->MSR;
    can_esr = CAN1->ESR;

    if (((can_msr & (CAN_MSR_INAK | CAN_MSR_SLAK)) != 0U) ||
        ((CAN1->MCR & CAN_MCR_ABOM) == 0U) ||
        ((can_esr & CAN_ESR_BOFF) != 0U))
    {
        printf("[BMS] CAN SELF-CHECK FAIL MCR=0x%08X MSR=0x%08X ESR=0x%08X\r\n",
            CAN1->MCR,
            can_msr,
            can_esr);

        fail = 1U;
    }
    // Đặt lại dấu thời gian bắt đầu, chuẩn bị cho lần vào SELF_CHECK tiếp theo
    selfcheck_start = 0;

    // ── Đánh giá kết quả ──
    // Quyết định trạng thái tiếp theo dựa trên kết quả tự kiểm tra của chu kỳ này.
    if (fail != 0U)
    {
        /*
        * Tự kiểm tra thất bại là lỗi toàn vẹn khi cấp nguồn.
        * Giữ cờ cho đến khi reset hệ thống, tránh việc nhấn nút khôi phục để bỏ qua lỗi tự kiểm tra khi cấp nguồn.
        */
        g_self_check_failed = 1U;
        g_fault_code = BMS_FAULT_SELF_CHECK;
        BMS_SetState(BMS_FAULT);
    }
    else
    {
        // Tự kiểm tra đạt, xóa rõ ràng điều kiện lỗi tự kiểm tra.
        g_self_check_failed = 0U;

        printf("[BMS] SELF_CHECK PASS\r\n");
        BMS_SetState(BMS_STANDBY);
    }
}


/*
 * Mô-đun: Xử lý trạng thái chờ.
 * Chức năng hiện tại: Phát hiện sườn nhấn nút khởi động PB6, gửi EVENT_START.
 * Sau khi khởi động sẽ chuyển vào MONITOR, không kích hoạt sạc theo SOC và không điều khiển đường dẫn pin.
 * SensorTask vẫn tiếp tục lấy mẫu; đánh giá lỗi chung đã được thực hiện trước khi vào hàm trạng thái này.
 */
static void Task_Standby(void)
{
// Lưu trạng thái nút khởi động lần trước để phát hiện sườn nhấn.
    static uint8_t last_start_pressed = 0U;

    // Đọc trạng thái nút khởi động PB6 hiện tại.
    uint8_t start_pressed = KEY_Read(KEY_START);

    // Kiểm tra xem nút khởi động có vừa chuyển từ nhả sang nhấn hay không.
    if((start_pressed == 1U) && (last_start_pressed == 0U))
    {
        // Xuất sự kiện nút khởi động qua cổng nối tiếp.
        printf("[BMS] START key pressed\r\n");

        // Gửi sự kiện khởi động BMS vào hàng đợi sự kiện.
        Event_Push(EVENT_START);
    }

    // Lưu trạng thái nút nhấn lần này để phát hiện sườn nhấn ở chu kỳ tiếp theo.
    last_start_pressed = start_pressed;
				
}


/*
 * Mô-đun: Nghiệp vụ trạng thái giám sát.
 * Dữ liệu snapshot cảm biến và đánh giá lỗi đã được xử lý bởi luồng chung BMS_StateMachine;
 * 0x301, 0x302 và 0x303 do CanTask gửi, nên tại đây không gửi lại CAN.
 */
static void Task_Monitor(void)
{
    /* Trạng thái giám sát hiện tại không có tác vụ cơ cấu chấp hành bổ sung nào. */
}


/*
 * Cập nhật đồng bộ đèn báo trạng thái PC13 theo trạng thái BMS.
 *
 * Trạng thái không lỗi: Đảo trạng thái mỗi 500ms, biểu thị hệ thống đang hoạt động;
 * Trạng thái lỗi: Đảo trạng thái mỗi 200ms, biểu thị hệ thống đang ở trạng thái cảnh báo.
 *
 * Chỉ phân biệt hai chế độ hiển thị “Lỗi” và “Không lỗi”.
 * Khi chế độ thay đổi, tắt LED trước và đặt lại mốc thời gian,
 * tránh việc chuyển trạng thái kế thừa thời gian còn lại của chế độ cũ.
 *
 * Hàm này chỉ do BmsTask thực thi, đảm bảo PC13 chỉ do một tác vụ điều khiển.
 */
static void BMS_UpdateLed(void)
{
    static uint32_t last_toggle_ms = 0U;
    static uint8_t last_fault_mode = 0U;

    uint32_t now_ms = GetTick();
    uint8_t fault_mode;
    uint32_t toggle_period_ms;

    fault_mode =
        (g_bms_state == BMS_FAULT) ? 1U : 0U;

    toggle_period_ms =
        (fault_mode != 0U) ? 200U : 500U;

    if (fault_mode != last_fault_mode)
    {
        last_fault_mode = fault_mode;
        last_toggle_ms = now_ms;
        LED_OFF();
        return;
    }

    if ((now_ms - last_toggle_ms) >= toggle_period_ms)
    {
        last_toggle_ms = now_ms;
        LED_Toggle();
    }
}







// Task_Fault — Phát hiện nhấn PA2 và gửi yêu cầu khôi phục lỗi.
// LED báo lỗi và còi báo được xử lý bởi logic chỉ thị trạng thái thống nhất.
static void Task_Fault(void)
{
   /*
    * Phát hiện sườn nhấn của nút khôi phục PA2.
    * Dùng phát hiện sườn thay vì mức liên tục để tránh việc đẩy lặp lại sự kiện khôi phục mỗi 10ms khi giữ nút.
    */
    {
        static uint8_t last_reset_pressed = 0U;
        uint8_t reset_pressed = KEY_Read(KEY_FAULT_RESET);

        if ((reset_pressed != 0U) &&
            (last_reset_pressed == 0U))
        {
            printf("[BMS] Manual recovery requested\r\n");
            Event_Push(EVENT_RECOVER);
        }

        last_reset_pressed = reset_pressed;
    }
}

/*
 * Mô-đun: Điểm vào điều phối chu kỳ của máy trạng thái.
 * Chức năng: Thực thi hàm xử lý tương ứng dựa theo trạng thái hiện tại.
 * Được BmsTask gọi mỗi 10ms một lần; hàm này không tạo tác vụ FreeRTOS.
 * Giai đoạn này chưa bao gồm tác vụ điều khiển sạc hoặc xả.
 */
void BMS_StateMachine(void)
{
    /*
    * Sau khi tự kiểm tra hoàn tất, tiếp tục đánh giá lỗi liên tục.
    * Trạng thái FAULT cũng bắt buộc phải tiếp tục đánh giá, nếu không sẽ không thể tích lũy thời gian ổn định để khôi phục.
    */
    if ((g_bms_state == BMS_STANDBY) ||
        (g_bms_state == BMS_MONITOR) ||
        (g_bms_state == BMS_FAULT))
    {
        BMS_UpdateSensorFaultManager();
    }
    // Chọn hàm xử lý cho chu kỳ này theo trạng thái hiện tại.
    switch (g_bms_state)
    {
        // Thực hiện khởi tạo khi cấp nguồn.
        case BMS_INIT:
            Task_Init();
            break;

        // Thực hiện tự kiểm tra hệ thống.
        case BMS_SELF_CHECK:
            Task_SelfCheck();
            break;

        // Chờ yêu cầu bắt đầu giám sát.
        case BMS_STANDBY:
            Task_Standby();
            break;

        // Thực hiện nghiệp vụ giám sát pin.
        case BMS_MONITOR:
            Task_Monitor();
            break;

        // Thực hiện cảnh báo lỗi và xử lý yêu cầu khôi phục.
        case BMS_FAULT:
            Task_Fault();
            break;

        // Tạm thời giữ nguyên cách xử lý trạng thái bất thường cũ: quay về trạng thái khởi tạo.
        default:
            BMS_SetState(BMS_INIT);
            break;
    }

    /*
     * Sau khi kết thúc xử lý nghiệp vụ trạng thái, cập nhật đồng bộ trạng thái LED cục bộ.
     * Thao tác GPIO tốn rất ít thời gian, sẽ không gây nghẽn tác vụ BMS.
     */
    BMS_UpdateLed();
}
