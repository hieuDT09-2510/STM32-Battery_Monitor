/*
 * Tên tệp: app_cli.c
 *
 * Tên mô-đun: Giao diện chẩn đoán dòng lệnh USART
 *
 * Luồng dữ liệu hiện tại:
 * Ngắt nhận USART1 đọc một byte
 *        ↓
 * Bộ đệm vòng RX 64 byte của BSP lưu tạm byte đó
 *        ↓
 * DiagnosticTask gọi CLI_Process(), lấy byte chuyển cho CLI_HandleByte()
 *        ↓
 * g_cli_line ghép thành một dòng; chỉ thực thi CLI_ExecuteCommand() khi nhận ký tự về đầu dòng hoặc xuống dòng
 *        ↓
 * Truy vấn các mô-đun Sensor, BMS, FaultManager, CAN, Flash, v.v. theo lệnh, hoặc thực hiện thao tác kiểm thử
 *
 * Ranh giới thiết kế:
 * 1. Ngắt nhận chỉ lưu byte, không phân tích lệnh, gọi printf hay truy cập Flash trong ngắt;
 * 2. CLI_Process() không chặn (non-blocking); mỗi lượt lấy tối đa 16 byte để tránh chiếm giữ tác vụ chẩn đoán;
 * 3. Tiêm lỗi chỉ thay đổi điều kiện kích hoạt kiểm thử của FaultManager, không sửa dữ liệu ADC thực,
 *    cũng không đặt trực tiếp mã lỗi hoạt động đã xác nhận;
 * 4. Mô-đun này không phải bộ phân xử truyền UART: nhiều tác vụ cùng gọi printf vẫn có thể in xen kẽ nhau.
 */

#include "app_cli.h"

#include <stdio.h>
#include <string.h>

#include "stm32f10x.h"
#include "app_bms_state.h"
#include "app_fault_manager.h"
#include "app_flash_log.h"
#include "app_health_monitor.h"
#include "app_sensor.h"
#include "app_soc.h"
#include "bsp_can.h"
#include "bsp_tim.h"
#include "bsp_usart.h"


/* Bao gồm ký tự kết thúc \0, thực tế cho phép nhập tối đa 31 ký tự hiển thị được. */
#define CLI_LINE_BUFFER_SIZE       32U

/* Số ký tự tối đa lấy ra trong một lần gọi CLI_Process, nhằm giới hạn thời gian thực thi mỗi lần gọi. */
#define CLI_MAX_CHARS_PER_PROCESS  16U


/* Bộ đệm dòng CLI: lưu các ký tự đã lấy từ bộ đệm vòng RX nhưng chưa được gửi đi bằng phím Enter. */
static char g_cli_line[CLI_LINE_BUFFER_SIZE];
/* Số ký tự hiện tại không bao gồm '\0' ở cuối chuỗi, giá trị tối đa là 31. */
static uint8_t g_cli_line_length = 0U;

/* Số lệnh đã gửi (bao gồm lệnh không xác định) và số lệnh không xác định/không hỗ trợ, dùng để quan sát trong Keil Watch. */
volatile uint32_t g_cli_command_count = 0U;
volatile uint32_t g_cli_unknown_count = 0U;


/* Sau khi đếm tới 0xFFFFFFFF thì giữ nguyên giá trị tối đa, tránh bị tràn quay vòng về 0 khi chạy dài hạn. */
static void CLI_IncrementSaturated(volatile uint32_t *value)
{
    if (*value < 0xFFFFFFFFU)
    {
        (*value)++;
    }
}


/* Chuyển đổi kiểu liệt kê trạng thái BMS thành văn bản dễ đọc; ở đây chỉ truy vấn, không sửa đổi máy trạng thái. */
static const char *CLI_GetBmsStateName(BMS_State_t state)
{
    switch (state)
    {
        case BMS_INIT:
            return "INIT";
        case BMS_SELF_CHECK:
            return "SELF_CHECK";
        case BMS_STANDBY:
            return "STANDBY";
        case BMS_MONITOR:
            return "MONITOR";
        case BMS_FAULT:
            return "FAULT";
        default:
            return "UNKNOWN";
    }
}


/* Dấu nhắc "> " cho biết CLI đã sẵn sàng nhận lệnh tiếp theo. */
static void CLI_PrintPrompt(void)
{
    printf("> ");
}


/* Liệt kê các lệnh mà firmware hiện hỗ trợ; cần cập nhật đồng bộ tại đây khi thêm lệnh mới. */
static void CLI_PrintHelp(void)
{
    printf("\r\nCommands:\r\n");
    printf("  help       - show command list\r\n");
    printf("  status     - show sensor and BMS state\r\n");
    printf("  fault      - show fault manager summary\r\n");
    printf("  fault inject <name> - inject a sensor fault\r\n");
    printf("  fault inject none   - stop fault injection\r\n");
    printf("  log        - print Flash V2 fault history\r\n");
    printf("  log clear  - erase Flash V2 fault history\r\n");
    printf("  can stats  - show CAN diagnostic counters\r\n");
    printf("  soc        - show voltage-based SOC estimate\r\n");
    printf("  watchdog test - stop feeding IWDG and verify reset\r\n");
    printf("  reboot     - software reset ECU-A\r\n\r\n");
}


/*
 * Dịch từng bit của mặt nạ bit 16-bit thành tên. Một mặt nạ có thể đồng thời chứa nhiều lỗi,
 * ví dụ 0x0005 = OV(0x0001) | OT(0x0004), do đó kiểm tra từng bit bằng if thay vì switch.
 * 0 chỉ biểu thị mặt nạ truyền vào hiện không có bit nào được bật, chuỗi xuất ra "NONE" không phải byte bản tin CAN.
 */
static void CLI_PrintFaultNames(uint16_t mask)
{
    if (mask == FAULT_MASK_NONE)
    {
        printf("NONE");
        return;
    }

    if ((mask & FAULT_MASK_OV) != 0U)           { printf("OV "); }
    if ((mask & FAULT_MASK_UV) != 0U)           { printf("UV "); }
    if ((mask & FAULT_MASK_OT) != 0U)           { printf("OT "); }
    if ((mask & FAULT_MASK_NTC_OPEN) != 0U)     { printf("NTC_OPEN "); }
    if ((mask & FAULT_MASK_NTC_SHORT) != 0U)    { printf("NTC_SHORT "); }
    if ((mask & FAULT_MASK_SENSOR_DATA) != 0U)  { printf("SENSOR_DATA "); }
    if ((mask & FAULT_MASK_ADC_TIMEOUT) != 0U)  { printf("ADC_TIMEOUT "); }
    if ((mask & FAULT_MASK_SELF_CHECK) != 0U)   { printf("SELF_CHECK "); }
    if ((mask & FAULT_MASK_CAN_DEGRADED) != 0U) { printf("CAN_DEGRADED "); }
    if ((mask & FAULT_MASK_CAN_BUS_OFF) != 0U)  { printf("CAN_BUS_OFF "); }
}


/*
 * Lệnh status: đọc tạm thời trạng thái hiện tại của BMS và snapshot mới nhất do Sensor phát hành.
 * sample là bản sao riêng của hàm này; việc SensorTask phát hành snapshot mới sau đó sẽ không làm thay đổi nó.
 * Lệnh này chỉ xem dữ liệu, không lấy mẫu lại và cũng không đánh giá lại lỗi.
 */
static void CLI_PrintStatus(void)
{
    SensorSample_t sample;  /* Snapshot đo lường hoàn chỉnh được sao chép từ Sensor. */
    uint32_t now_ms;        /* Giá trị Tick hệ thống theo mili-giây tại thời điểm truy vấn. */
    uint32_t age_ms;        /* Tuổi của snapshot, dùng để quan sát xem có hiện tượng ngừng trệ thu thập dữ liệu hay không. */
    uint32_t voltage_mv;    /* Quy đổi điện áp pin sang mV để tránh vấn đề định dạng số thực dấu phẩy động qua cổng nối tiếp. */
    int32_t temperature_dC; /* Quy đổi nhiệt độ sang đơn vị 0.1℃, giữ nguyên dấu của nhiệt độ âm. */
    uint8_t sample_available;

    /* Sao chép snapshot trước rồi mới đọc thời gian, tránh dấu thời gian của snapshot mới hơn giá trị now_ms đã đọc. */
    sample_available = Sensor_GetLatestDisplay(&sample);
    now_ms = GetTick();

    printf("\r\n[STATUS] State=%s Fault=0x%04X Tick=%lums\r\n",
           CLI_GetBmsStateName(BMS_GetState()),
           (unsigned int)g_fault_code,
           (unsigned long)now_ms);

    /* Khi chưa có snapshot nào sau khi cấp nguồn, không được sử dụng các trường chưa khởi tạo trong sample. */
    if (sample_available == 0U)
    {
        printf("[STATUS] Sensor=NO_DATA\r\n");
        return;
    }

    age_ms = Sensor_CalculateAgeMs(&sample, now_ms);

    printf("[STATUS] Sequence=%lu Age=%lums ADC0=%u ADC1=%u\r\n",
           (unsigned long)sample.sequence,
           (unsigned long)age_ms,
           (unsigned int)sample.adc_pa0,
           (unsigned int)sample.adc_pa1);

    /* Mỗi trường kiểm tra cờ hợp lệ độc lập; nhiệt độ không hợp lệ không cản trở việc hiển thị điện áp hợp lệ. */
    if (sample.voltage_valid != 0U)
    {
        voltage_mv = (uint32_t)(sample.cell_voltage_v * 1000.0f + 0.5f);
        printf("[STATUS] Cell=%lu.%02luV\r\n",
               (unsigned long)(voltage_mv / 1000UL),
               (unsigned long)((voltage_mv % 1000UL) / 10UL));
    }
    else
    {
        printf("[STATUS] Cell=INVALID\r\n");
    }

    if (sample.temperature_valid != 0U)
    {
        /* Số dương cộng 0.5, số âm trừ 0.5 rồi ép sang số nguyên để làm tròn đến 0.1℃. */
        temperature_dC = (sample.temperature_c >= 0.0f) ?
            (int32_t)(sample.temperature_c * 10.0f + 0.5f) :
            (int32_t)(sample.temperature_c * 10.0f - 0.5f);

        /* In dấu âm trước, sau đó lấy giá trị tuyệt đối để tách phần nguyên và phần thập phân. */
        if (temperature_dC < 0)
        {
            temperature_dC = -temperature_dC;
            printf("[STATUS] Temp=-%ld.%ldC\r\n",
                   (long)(temperature_dC / 10),
                   (long)(temperature_dC % 10));
        }
        else
        {
            printf("[STATUS] Temp=%ld.%ldC\r\n",
                   (long)(temperature_dC / 10),
                   (long)(temperature_dC % 10));
        }
    }
    else
    {
        printf("[STATUS] Temp=INVALID NTC=%u\r\n",
               (unsigned int)sample.ntc_input);
    }
}


/*
 * Lệnh fault: xem trạng thái tổng hợp của trình quản lý lỗi.
 * Active=đã xác nhận và chưa được xóa; Latched=đã chốt bằng phần mềm;
 * Critical=lỗi nghiêm trọng cần chuyển sang trạng thái FAULT; RecoveryReady=đạt điều kiện phục hồi và có thể xóa;
 * Injected=điều kiện kiểm thử do CLI thiết lập hiện tại, có thể chưa qua xác nhận, không thể đánh đồng với Active.
 */
static void CLI_PrintFault(void)
{
    FaultManagerSnapshot_t snapshot;

    if (FaultManager_GetSnapshot(&snapshot) == 0U)
    {
        printf("\r\n[FAULT] Snapshot unavailable\r\n");
        return;
    }

    printf("\r\n[FAULT] Active=0x%04X Latched=0x%04X\r\n",
           (unsigned int)snapshot.active_mask,
           (unsigned int)snapshot.latched_mask);
    printf("[FAULT] Critical=0x%04X RecoveryReady=0x%04X\r\n",
           (unsigned int)snapshot.critical_mask,
           (unsigned int)snapshot.recovery_ready_mask);
    printf("[FAULT] Names=");
    CLI_PrintFaultNames(snapshot.active_mask);
    printf("\r\n");
    printf("[FAULT] Injected=0x%04X ",
           (unsigned int)FaultManager_GetInjectionMask());
    CLI_PrintFaultNames(FaultManager_GetInjectionMask());
    printf("\r\n");
}


/*
 * Ánh xạ tên do người dùng nhập thành một bit lỗi kiểm thử.
 * name đầu vào như "ov" hoặc "none"; mask là địa chỉ đầu ra, khi thành công sẽ ghi giá trị bit tương ứng.
 * Hàm này chỉ phân tích chuỗi, không khởi động bộ đếm thời gian xác nhận và không làm thay đổi mẫu đo thực.
 * Không cho phép giả lập các lỗi SELF_CHECK và lỗi phần cứng CAN thông qua lệnh này.
 * Trả về 1 nếu tên hợp lệ, trả về 0 nếu con trỏ rỗng hoặc tên không được hỗ trợ.
 */
static uint8_t CLI_ParseInjectionName(const char *name,
                                      uint16_t *mask)
{
    if ((name == 0) || (mask == 0))
    {
        return 0U;
    }

    if (strcmp(name, "none") == 0)             { *mask = FAULT_MASK_NONE; }
    else if (strcmp(name, "ov") == 0)          { *mask = FAULT_MASK_OV; }
    else if (strcmp(name, "uv") == 0)          { *mask = FAULT_MASK_UV; }
    else if (strcmp(name, "ot") == 0)          { *mask = FAULT_MASK_OT; }
    else if (strcmp(name, "ntc_open") == 0)    { *mask = FAULT_MASK_NTC_OPEN; }
    else if (strcmp(name, "ntc_short") == 0)   { *mask = FAULT_MASK_NTC_SHORT; }
    else if (strcmp(name, "sensor_data") == 0) { *mask = FAULT_MASK_SENSOR_DATA; }
    else if (strcmp(name, "adc_timeout") == 0) { *mask = FAULT_MASK_ADC_TIMEOUT; }
    else
    {
        return 0U;
    }

    return 1U;
}


/*
 * Lệnh fault inject: thiết lập hoặc hủy bỏ mặt nạ tiêm lỗi.
 * Ví dụ "fault inject ov" ép điều kiện kích hoạt (trip) quá áp tại đầu vào FaultManager thành 1,
 * các bước tiếp theo gồm xác nhận 500ms, chốt lỗi, cảnh báo, thông báo CAN và đưa nhật ký vào hàng đợi vẫn đi theo luồng xử lý bình thường.
 * "fault inject none" chỉ hủy bỏ việc tiêm lỗi; các lỗi đã xác nhận vẫn được xử lý theo quy tắc phục hồi.
 */
static void CLI_SetFaultInjection(const char *name)
{
    uint16_t mask;

    if (CLI_ParseInjectionName(name, &mask) == 0U)
    {
        CLI_IncrementSaturated(&g_cli_unknown_count);
        printf("\r\n[CLI] Unsupported injection: %s\r\n", name);
        printf("[CLI] Use ov, uv, ot, ntc_open, ntc_short, "
               "sensor_data, adc_timeout or none\r\n");
        return;
    }

    /* Ghi cấu hình tiêm lỗi thông qua giao diện công khai, không ghi trực tiếp vào giá trị ADC hoặc g_fault_code. */
    FaultManager_SetInjectionMask(mask);

    printf("\r\n[CLI] Fault injection=0x%04X ",
           (unsigned int)mask);
    CLI_PrintFaultNames(mask);
    printf("\r\n");

    if (mask == FAULT_MASK_NONE)
    {
        printf("[CLI] Injection stopped; normal recovery rules now apply\r\n");
    }
    else
    {
        printf("[CLI] Real sensor values are unchanged\r\n");
    }
}


/*
 * Lệnh can stats: đọc trạng thái hiện tại của trình điều khiển CAN và các bộ đếm tích lũy từ khi cấp nguồn.
 * LastResult là kết quả của lần gọi truyền gần nhất, không đại diện cho toàn bộ lịch sử truyền đều thành công;
 * BusOffActive là trạng thái hiện tại, TxOK/TxFail/BusOff/Recovery là số lần tích lũy.
 */
static void CLI_PrintCanStats(void)
{
    printf("\r\n[CAN] LastResult=%u BusOffActive=%u ESR=0x%08lX\r\n",
           (unsigned int)g_can_last_send_result,
           (unsigned int)g_can_bus_off_active,
           (unsigned long)g_can_last_esr);
    printf("[CAN] TxOK=%lu TxFail=%lu BusOff=%lu Recovery=%lu\r\n",
           (unsigned long)g_can_tx_ok_count,
           (unsigned long)g_can_tx_fail_count,
           (unsigned long)g_can_bus_off_count,
           (unsigned long)g_can_recovery_count);
}


/* Lệnh soc chỉ đọc snapshot trong RAM; SOC được tính bằng nội suy tuyến tính từ điện áp pin. */
static void CLI_PrintSoc(void)
{
    SocSnapshot_t snapshot; /* Snapshot SOC trong RAM. */

    if (Soc_GetSnapshot(&snapshot) == 0U)
    {
        printf("\r\n[SOC] Valid=0\r\n");
        return;
    }

    printf("\r\n[SOC] Valid=1 Cell=%umV SOC=%u.%02u%%\r\n",
           (unsigned int)snapshot.cell_mv,
           (unsigned int)(snapshot.soc_centi_percent / 100U),
           (unsigned int)(snapshot.soc_centi_percent % 100U));
    printf("[SOC] Model=LINEAR %u-%umV Seq=%lu Time=%lums\r\n",
           (unsigned int)SOC_EMPTY_VOLTAGE_MV,
           (unsigned int)SOC_FULL_VOLTAGE_MV,
           (unsigned long)snapshot.sequence,
           (unsigned long)snapshot.timestamp_ms);
}


/*
 * Bộ điều phối lệnh đầy đủ: đầu vào phải là một dòng văn bản hoàn chỉnh kết thúc bằng '\0'.
 * strcmp yêu cầu khớp hoàn toàn cả dòng; "fault inject " dùng strncmp để khớp tiền tố cố định,
 * phần tên phía sau giao cho CLI_SetFaultInjection(). Hàm này chạy trong ngữ cảnh tác vụ.
 */
static void CLI_ExecuteCommand(const char *command)
{
    CLI_IncrementSaturated(&g_cli_command_count);

    if (strcmp(command, "help") == 0)
    {
        CLI_PrintHelp();
    }
    else if (strcmp(command, "status") == 0)
    {
        CLI_PrintStatus();
    }
    else if (strcmp(command, "fault") == 0)
    {
        CLI_PrintFault();
    }
    else if (strncmp(command, "fault inject ", 13U) == 0)
    {
        /* Tiền tố dài đúng 13 ký tự, &command[13] trỏ tới ký tự đầu của ov, uv hoặc none. */
        CLI_SetFaultInjection(&command[13]);
    }
    else if (strcmp(command, "log") == 0)
    {
        /* Module FlashLog chịu trách nhiệm đọc lịch sử và xuất từng bản ghi, CLI không giải mã trực tiếp Flash. */
        FlashLog_PrintAll();
    }
    else if (strcmp(command, "log clear") == 0)
    {
        /* Việc xóa, kiểm tra và quản lý chỉ mục nằm trong module FlashLog; xuất thông báo nếu thất bại. */
        if (FlashLog_Clear() == 0U)
        {
            printf("[CLI] Log clear failed; old write index preserved\r\n");
        }
    }
    else if (strcmp(command, "can stats") == 0)
    {
        CLI_PrintCanStats();
    }
    else if (strcmp(command, "soc") == 0)
    {
        CLI_PrintSoc();
    }
    else if (strcmp(command, "watchdog test") == 0)
    {
        /*
         * Chỉ yêu cầu Health Monitor ngừng làm mới IWDG, không chủ động gọi reset phần mềm.
         * Nhờ vậy, việc reset có thực sự xảy ra hay không sẽ chứng minh đúng watchdog độc lập đang hoạt động.
         */
        g_health_test_stop_feed = 1U;
        printf("\r\n[HEALTH] IWDG feed stopped for test\r\n");
        printf("[HEALTH] Hardware reset expected in about 4 seconds\r\n");
    }
    else if (strcmp(command, "reboot") == 0)
    {
        printf("\r\n[CLI] Software reset requested\r\n");
        /* Chờ gửi xong byte thông báo cuối cùng để tránh reset làm cắt ngang câu này. */
        USART1_WaitSendComplete();
        NVIC_SystemReset();
    }
    else
    {
        CLI_IncrementSaturated(&g_cli_unknown_count);
        printf("\r\n[CLI] Unknown command: %s\r\n", command);
        printf("[CLI] Type help for command list\r\n");
    }
}


/*
 * Xử lý một byte vừa lấy ra từ bộ đệm vòng RX của BSP: kết thúc dòng, xóa lùi, lọc hoặc nối thêm.
 * Biến g_cli_line ở đây là bộ đệm để "ghép thành lệnh hoàn chỉnh", không phải bộ đệm vòng RX do ngắt ghi vào.
 */
static void CLI_HandleByte(uint8_t data)
{
    /* Ký tự về đầu dòng hoặc xuống dòng biểu thị đã nhập xong một lệnh. */
    if ((data == '\r') || (data == '\n'))
    {
        /* Ký tự kết thúc thứ hai trong chuỗi CR+LF sẽ không tạo ra một lệnh rỗng. */
        if (g_cli_line_length == 0U)
        {
            return;
        }

        /* Sau khi ghi ký tự kết thúc chuỗi C, bộ điều phối lệnh mới có thể dùng strcmp một cách an toàn. */
        g_cli_line[g_cli_line_length] = '\0';
        printf("\r\n");
        CLI_ExecuteCommand(g_cli_line);
        /* Xóa bộ đệm dòng sau khi thực thi; ký tự xuống dòng thứ hai trong CRLF sẽ bị bỏ qua vì độ dài bằng 0. */
        g_cli_line_length = 0U;
        g_cli_line[0] = '\0';
        CLI_PrintPrompt();
        return;
    }

    /* Hỗ trợ phím Backspace và phím Delete để sửa lệnh chưa gửi thực thi. */
    if ((data == 0x08U) || (data == 0x7FU))
    {
        if (g_cli_line_length > 0U)
        {
            g_cli_line_length--;
            g_cli_line[g_cli_line_length] = '\0';
            /* Lùi con trỏ, ghi đè khoảng trắng để xóa ký tự cũ, rồi lùi con trỏ lại lần nữa. */
            USART1_SendString("\b \b");
        }
        return;
    }

    /* Chỉ giữ lại các ký tự hiển thị được trong khoảng ASCII 0x20~0x7E; bỏ qua các ký tự điều khiển khác. */
    if ((data < 0x20U) || (data > 0x7EU))
    {
        return;
    }

    /* Cuối bộ đệm phải chừa lại một byte cho '\0'; nếu quá dài thì xóa cả dòng và nhắc nhập lại. */
    if (g_cli_line_length >= (CLI_LINE_BUFFER_SIZE - 1U))
    {
        g_cli_line_length = 0U;
        g_cli_line[0] = '\0';
        printf("\r\n[CLI] Command too long, input cleared\r\n");
        CLI_PrintPrompt();
        return;
    }

    /* Nối thêm một ký tự và luôn duy trì chuỗi C hợp lệ kết thúc bằng '\0'. */
    g_cli_line[g_cli_line_length] = (char)data;
    g_cli_line_length++;
    g_cli_line[g_cli_line_length] = '\0';

    /* Thiết bị tự echo lại ký tự; nếu phần mềm serial đang bật local echo thì nên tắt đi. */
    USART1_SendByte(data);
}


/* Hàm main gọi một lần trước khi khởi chạy bộ lập lịch: xóa trạng thái CLI và xuất dấu nhắc lệnh đầu tiên. */
void CLI_Init(void)
{
    g_cli_line_length = 0U;
    g_cli_line[0] = '\0';
    g_cli_command_count = 0U;
    g_cli_unknown_count = 0U;

    printf("[CLI] Diagnostic terminal ready. Type help.\r\n");
    CLI_PrintPrompt();
}



/*
 * Điểm vào non-blocking được DiagnosticTask gọi định kỳ.
 * USART1_ReadByteNonBlocking() đọc từ bộ đệm vòng RX phần mềm, không đọc trực tiếp thanh ghi DR;
 * Thoát ngay nếu không có byte nào, mỗi lượt xử lý tối đa 16 byte, phần còn lại để lần gọi sau.
 */
void CLI_Process(void)
{
    uint8_t data;            /* Một byte hiện tại vừa lấy ra từ bộ đệm vòng RX. */
    uint8_t processed_count; /* Số byte đã xử lý trong lượt này, dùng để giới hạn khối lượng công việc mỗi lượt. */

    processed_count = 0U;

    while ((processed_count < CLI_MAX_CHARS_PER_PROCESS) &&
           (USART1_ReadByteNonBlocking(&data) != 0U))
    {
        CLI_HandleByte(data);
        processed_count++;
    }
}
