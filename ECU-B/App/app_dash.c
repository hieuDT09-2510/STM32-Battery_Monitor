// ============================================================
// app_dash.c
//
// ECU-B Mô-đun hiển thị dashboard
//
// Chức năng:
// 1. Nhận dữ liệu CAN
// 2. Phân tích trạng thái BMS
// 3. Hiển thị OLED
// 4. Kiểm tra timeout CAN
// 5. Kiểm tra tính liên tục của Alive Counter 0x301, tích lũy số lần số thứ tự bất thường
// 6. Kiểm tra CRC8 bản tin trạng thái then chốt, bản tin lỗi không cập nhật dữ liệu nghiệp vụ
// 7. Kiểm tra CRC8 và Alive Counter của bản tin nhiệt độ, bản tin lỗi không cập nhật nhiệt độ
// 8. Khi một loại bản tin bị timeout chỉ đánh dấu dữ liệu tương ứng mất hiệu lực, các dữ liệu hợp lệ khác tiếp tục hiển thị
// 8. Khi bản tin nhiệt độ bị timeout thì dừng hiển thị nhiệt độ cũ, nhưng giữ lại các dữ liệu mới khác
// 9. Kiểm tra CRC8 và Alive Counter của bản tin lỗi, bản tin lỗi không được phép sửa đổi trạng thái lỗi hiện tại
// 10. Khi bản tin lỗi bị timeout, giữ lại mặt nạ lỗi tin cậy cuối cùng và hiển thị mất truyền thông

// Bản tin CAN:
// 0x301
//   Byte0~1 Cell Voltage(mV)
//   Byte2    SOC(%), ước tính tuyến tính từ điện áp, 0xFF biểu thị không có SOC hợp lệ
//   Byte3    Charge State
//   Byte4 4-bit thấp Alive Counter (vòng lặp 0~15)
//   Byte5~6  Reserved
//   Byte7    CRC8 of Byte0~Byte6
//
// 0x302
//   Byte0    Temperature
//   Byte1~3  Reserved
//   Byte4 4-bit thấp Alive Counter (vòng lặp 0~15)
//   Byte5~6  Reserved
//   Byte7    CRC8 of Byte0~Byte6
//
// 0x303
//   Byte0~1 Active Fault Mask, high byte first
//   Byte2~3 Reserved
//   Byte4 4-bit thấp Alive Counter (vòng lặp 0~15)
//   Byte5~6 Reserved
//   Byte7    CRC8 of Byte0~Byte6

// ============================================================

#include "app_dash.h"
#include "bsp_oled.h"
#include "bsp_tim.h"

#include <stdio.h>
#include <string.h>

// 0x301 gửi mỗi 100ms; không nhận được trong 500ms liên tục thì coi là timeout truyền thông trạng thái điện áp.
#define CAN_301_TIMEOUT_MS        500U

// 0x302 gửi mỗi 500ms; không nhận được trong 1500ms liên tục thì coi là timeout truyền thông trạng thái nhiệt độ.
#define CAN_302_TIMEOUT_MS        1500U

// 0x303 gửi mỗi 100ms; không nhận được khung hợp lệ trong 500ms liên tục thì coi là timeout truyền thông trạng thái lỗi.
#define CAN_303_TIMEOUT_MS        500U

// Sau khi truyền thông phục hồi vẫn để thông báo mất 0x301 hiển thị ít nhất 2 giây, không block vòng lặp chính.
#define CAN_301_NOTICE_HOLD_MS   2000U


// ============================================================
// Dữ liệu hiển thị toàn cục
// ============================================================

volatile BMS_DisplayData_t g_dash_data = {0};


// ============================================================
// Dash_Init
//
// Khởi tạo dashboard
// ============================================================

void Dash_Init(void)
{
    memset((void *)&g_dash_data, 0, sizeof(g_dash_data));
    g_dash_data.soc = 0xFFU;
}


// ============================================================
// Dash_UpdateData
//
// Cập nhật dữ liệu hiển thị sau khi nhận CAN
//
// Được gọi bởi ngắt nhận CAN
// ============================================================

void Dash_UpdateData(CAN_Frame_t *frame)
{
    uint32_t rx_tick;
    uint8_t recognized = 1U;
    uint8_t alive_counter;
    uint8_t expected_alive;

    if(frame == NULL)
        return;

    // Ngắt lần này chỉ đọc thời gian một lần, đảm bảo các trường thời gian của cùng một khung hoàn toàn nhất quán.
    rx_tick = GetTick();

    switch(frame->id)
    {
        //------------------------------------------------------
        // Điện áp + SOC + Trạng thái
        //------------------------------------------------------
        case CAN_ID_BMS_VOLTAGE:
        {
            /*
            * Giao thức hiện tại quy định chiều dài bản tin trạng thái then chốt phải là 8 byte。
            * Khi chiều dài không đúng không thể đọc đủ CRC, toàn bộ khung xử lý là không hợp lệ。
            */
            if(frame->dlc != 8U)
            {
                g_dash_data.crc_301_error = 1U;
                g_dash_data.crc_301_error_count++;

                /*
                * Sau khi recognized xóa về 0, khung này sẽ không cập nhật
                * thời gian "bản tin BMS hợp lệ cuối cùng".
                */
                recognized = 0U;
                break;
            }

            /*
            * Lưu CRC do bên gửi mang theo, thuận tiện quan sát qua Watch.
            */
            g_dash_data.last_301_received_crc =
                frame->data[7];

            /*
            * ECU-B tính lại CRC cho Byte0~Byte6.
            */
            g_dash_data.last_301_calculated_crc =
                BMS_CalculateCRC8(frame->data, 7U);

            /*
            * Hai CRC không bằng nhau, chứng tỏ dữ liệu khung này không tin cậy.
            */
            if(g_dash_data.last_301_calculated_crc !=
            g_dash_data.last_301_received_crc)
            {
                g_dash_data.crc_301_error = 1U;
                g_dash_data.crc_301_error_count++;

                /*
                * Trực tiếp thoát khỏi case này:
                * Không cập nhật điện áp, SOC, Alive Counter và thời gian nhận。
                */
                recognized = 0U;
                break;
            }

            /*
            * Chạy được đến đây chứng tỏ chiều dài bản tin và CRC đều đúng.
            */
            g_dash_data.crc_301_error = 0U;
                        g_dash_data.cell_mv =
                            ((uint16_t)frame->data[0] << 8) |
                            frame->data[1];

                        g_dash_data.soc =
                            BMS_UnpackVoltageSoc(frame);

                        g_dash_data.chg_state =
                            frame->data[3];

                        alive_counter = BMS_Unpack301Alive(frame);

            /*
             * Khung đầu tiên không có "khung trước" để so sánh, chỉ lưu mốc cơ sở.
             * Từ khung thứ hai trở đi, giá trị kỳ vọng luôn là "khung trước + 1 rồi giữ lại 4-bit thấp".
             */
            if (g_dash_data.received_301 != 0U)
            {
                expected_alive =
                    (uint8_t)((g_dash_data.last_301_alive + 1U) &
                              CAN_301_ALIVE_MASK);

                if (alive_counter != expected_alive)
                {
                    g_dash_data.alive_301_error = 1U;
                    g_dash_data.alive_301_error_count++;
                }
                else
                {
                    g_dash_data.alive_301_error = 0U;
                }
            }
            else
            {
                g_dash_data.alive_301_error = 0U;
            }

            // Cho dù khung này có bị nhảy số hay không, đều lấy khung này làm mốc cơ sở mới cho lần so sánh tiếp theo.
            g_dash_data.last_301_alive = alive_counter;

            g_dash_data.valid = 1U;
            g_dash_data.received_301 = 1U;
            g_dash_data.last_301_rx_tick = rx_tick;
            g_dash_data.comm_301_timeout = 0U;

            break;
        }

        //------------------------------------------------------
        // Nhiệt độ
        //------------------------------------------------------
       case CAN_ID_BMS_TEMP:
        {
            /*
            * Bản tin nhiệt độ bắt buộc phải đầy đủ 8 byte。
            * Khi lỗi chiều dài không thể lấy được CRC hoàn chỉnh。
            */
            if(frame->dlc != 8U)
            {
                g_dash_data.crc_302_error = 1U;
                g_dash_data.crc_302_error_count++;
                recognized = 0U;
                break;
            }

            /*
            * Lưu CRC do bản tin mang theo，
            * và tính lại theo Byte0~Byte6。
            */
            g_dash_data.last_302_received_crc =
                frame->data[7];

            g_dash_data.last_302_calculated_crc =
                BMS_CalculateCRC8(frame->data, 7U);

            /*
            * Khi CRC không khớp thì từ chối toàn bộ khung。
            * Không được cập nhật nhiệt độ, số thứ tự và thời gian nhận。
            */
            if(g_dash_data.last_302_calculated_crc !=
            g_dash_data.last_302_received_crc)
            {
                g_dash_data.crc_302_error = 1U;
                g_dash_data.crc_302_error_count++;
                recognized = 0U;
                break;
            }

            /*
            * CRC đúng, cho phép sử dụng dữ liệu khung này.
            */
            g_dash_data.crc_302_error = 0U;

            /*
            * Trước hết phân tích Alive Counter của bản tin nhiệt độ khung này.
            */
            alive_counter =
                BMS_Unpack302Alive(frame);

            /*
            * Khung đầu tiên chỉ thiết lập mốc cơ sở。
            * Từ khung thứ hai bắt đầu kiểm tra số thứ tự có liên tục hay không。
            */
            if(g_dash_data.received_302 != 0U)
            {
                expected_alive =
                    (uint8_t)(
                        (g_dash_data.last_302_alive + 1U) &
                        CAN_302_ALIVE_MASK);

                if(alive_counter != expected_alive)
                {
                    g_dash_data.alive_302_error = 1U;
                    g_dash_data.alive_302_error_count++;
                }
                else
                {
                    g_dash_data.alive_302_error = 0U;
                }
            }
            else
            {
                g_dash_data.alive_302_error = 0U;
            }

            /*
            * Sử dụng khung hợp lệ hiện tại làm mốc cơ sở cho lần so sánh tiếp theo.
            */
            g_dash_data.last_302_alive =
                alive_counter;

            /*
            * Sau khi xử lý CRC và số thứ tự xong mới cập nhật nhiệt độ。
            * Số thứ tự bị nhảy không có nghĩa là nội dung nhiệt độ nhất định bị sai，
            * vì vậy khi CRC đúng vẫn áp dụng nhiệt độ hiện tại。
            */
            g_dash_data.temp =
                frame->data[0];

            g_dash_data.valid = 1U;
            g_dash_data.received_302 = 1U;
            g_dash_data.last_302_rx_tick = rx_tick;

            // Nhận được bản tin nhiệt độ có CRC đúng, lập tức phục hồi trạng thái truyền thông nhiệt độ.
            g_dash_data.comm_302_timeout = 0U;

            break;
        }

        //------------------------------------------------------
        // Mã lỗi
        //------------------------------------------------------
            case CAN_ID_BMS_FAULT:
            {
                /*
                * Bản tin lỗi bắt buộc phải chứa đầy đủ 8 byte.
                * Khi chiều dài không đúng, không thể đọc được CRC hoàn chỉnh.
                */
                if (frame->dlc != 8U)
                {
                    g_dash_data.crc_303_error = 1U;
                    g_dash_data.crc_303_error_count++;
                    recognized = 0U;
                    break;
                }

                /*
                * Lưu CRC do bên gửi mang theo,

                * rồi tính lại theo Byte0~Byte6.
                */
                g_dash_data.last_303_received_crc =
                    frame->data[7];

                g_dash_data.last_303_calculated_crc =
                    BMS_CalculateCRC8(frame->data, 7U);

                /*
                * CRC không khớp, chứng tỏ nội dung lỗi hiện tại không tin cậy.
                * Không cho phép bản tin lỗi thiết lập hoặc xóa hiển thị lỗi.
                */
                if (g_dash_data.last_303_calculated_crc !=
                    g_dash_data.last_303_received_crc)
                {
                    g_dash_data.crc_303_error = 1U;
                    g_dash_data.crc_303_error_count++;
                    recognized = 0U;
                    break;
                }

                // Chạy được đến đây chứng tỏ chiều dài bản tin và CRC đều đúng.
                g_dash_data.crc_303_error = 0U;

                // Lấy ra Alive Counter của chính bản tin lỗi hiện tại.
                alive_counter =
                    BMS_Unpack303Alive(frame);

                /*
                * Khung đầu tiên chỉ thiết lập mốc số thứ tự;
                * Từ khung thứ hai bắt đầu kiểm tra có bằng khung trước cộng 1 hay không.
                */
                if (g_dash_data.received_303 != 0U)
                {
                    expected_alive =
                        (uint8_t)(
                            (g_dash_data.last_303_alive + 1U) &
                            CAN_303_ALIVE_MASK);

                    if (alive_counter != expected_alive)
                    {
                        g_dash_data.alive_303_error = 1U;
                        g_dash_data.alive_303_error_count++;
                    }
                    else
                    {
                        g_dash_data.alive_303_error = 0U;
                    }
                }
                else
                {
                    g_dash_data.alive_303_error = 0U;
                }

                /*
                * Sau khi CRC đúng, khung hiện tại có thể dùng làm mốc số thứ tự mới.
                * Ngay cả khi bị nhảy số, nội dung lỗi hiện tại vẫn có thể sử dụng.
                */
                g_dash_data.last_303_alive =
                    alive_counter;

                /*
                * Sau khi tất cả các bước kiểm tra tính toàn vẹn kết thúc,
                * mới cho phép cập nhật mặt nạ lỗi hoạt động dùng cho OLED.
                */
                g_dash_data.fault =
                    BMS_UnpackFault(frame);

                g_dash_data.valid = 1U;
                g_dash_data.received_303 = 1U;
                g_dash_data.last_303_rx_tick = rx_tick;
                // Nhận được bản tin lỗi có CRC đúng, lập tức phục hồi trạng thái truyền thông lỗi.
                g_dash_data.comm_303_timeout = 0U;

                break;
            }

        default:
            recognized = 0U;
            break;
    }

    // Bản tin chưa nhận diện không được làm mới thời gian truyền thông tổng BMS, tránh trường hợp lưu lượng không liên quan che lấp mất truyền thông.
    if (recognized != 0U)
    {
        g_dash_data.last_rx_tick = rx_tick;
    }
}


/*
 * Dash_UpdateCommStatus — Cập nhật trạng thái timeout độc lập của từng bản tin chu kỳ.
 *
 * 0x301 gửi mỗi 100ms, liên tục 500ms không có khung hợp lệ thì timeout;
 * 0x302 gửi mỗi 500ms, liên tục 1500ms không có khung hợp lệ thì timeout;
 * 0x303 gửi mỗi 100ms, liên tục 500ms không có khung hợp lệ thì timeout.
 *
 * 3 loại bản tin sử dụng riêng biệt thời gian nhận của mình,
 * một loại bản tin đến không được che lấp việc mất loại bản tin khác.
 *
 * Tất cả phép trừ thời gian đều dùng phép trừ không dấu,
 * do đó có thể xử lý quay vòng bộ đếm mili-giây 32-bit.
 */
void Dash_UpdateCommStatus(uint32_t now_ms)
{
    uint8_t previous_timeout;
    uint8_t current_timeout;

    previous_timeout = g_dash_data.comm_301_timeout;

    if (g_dash_data.received_301 == 0U)
    {
        current_timeout =
            (now_ms >= CAN_301_TIMEOUT_MS) ? 1U : 0U;
    }
    else
    {
        current_timeout =
            ((uint32_t)(now_ms - g_dash_data.last_301_rx_tick) >=
             CAN_301_TIMEOUT_MS) ? 1U : 0U;
    }

    g_dash_data.comm_301_timeout = current_timeout;

    // Chỉ bắt đầu bấm giờ khi chuyển từ bình thường sang timeout, không được bấm giờ lặp đi lặp lại mỗi lần refresh.
    if ((current_timeout != 0U) && (previous_timeout == 0U))
    {
        g_dash_data.comm_301_notice_active = 1U;
        g_dash_data.comm_301_notice_since_ms = now_ms;
    }

    // Khi truyền thông vẫn timeout thì không gỡ thông báo; chỉ gỡ bỏ sau khi truyền thông phục hồi và hiển thị đủ 2 giây.
    if ((current_timeout == 0U) &&
        (g_dash_data.comm_301_notice_active != 0U) &&
        ((uint32_t)(now_ms - g_dash_data.comm_301_notice_since_ms) >=
         CAN_301_NOTICE_HOLD_MS))
    {
        g_dash_data.comm_301_notice_active = 0U;
    }

   /*
    * Đánh giá độc lập bản tin nhiệt độ có timeout hay không.
    * 0x301 hoặc 0x303 tiếp tục đến cũng không thể làm mới thời gian này.
    */
    if (g_dash_data.received_302 == 0U)
    {
        /*
        * Sau khi cấp nguồn có thể chưa nhận được khung bản tin nhiệt độ đầu tiên,
        * vì vậy tạm thời chờ 1500ms.
        */
        g_dash_data.comm_302_timeout =
            (now_ms >= CAN_302_TIMEOUT_MS) ? 1U : 0U;
    }
    else
    {
        /*
        * Phép trừ không dấu cũng có thể xử lý quay vòng bộ đếm mili-giây 32-bit.
        */
        g_dash_data.comm_302_timeout =
            ((uint32_t)(now_ms -
                        g_dash_data.last_302_rx_tick) >=
            CAN_302_TIMEOUT_MS) ? 1U : 0U;
    }

        /*
        * Phán đoán độc lập bản tin trạng thái lỗi có timeout hay không.
        * Bản tin điện áp hoặc nhiệt độ tiếp tục đến không thể làm mới thời gian bản tin lỗi.
        */
        if (g_dash_data.received_303 == 0U)
        {
            /*
            * Khi sau cấp nguồn vẫn chưa nhận được khung bản tin lỗi đầu tiên,

            * dành ra 500ms thời gian chờ trước.
            */
            g_dash_data.comm_303_timeout =
                (now_ms >= CAN_303_TIMEOUT_MS) ? 1U : 0U;
        }
        else
        {
            /*
            * Sau khi đã nhận bản tin lỗi hợp lệ,
            * lấy thời gian hiện tại trừ đi thời gian nhận hợp lệ cuối cùng.
            */
            g_dash_data.comm_303_timeout =
                ((uint32_t)(now_ms -
                            g_dash_data.last_303_rx_tick) >=
                CAN_303_TIMEOUT_MS) ? 1U : 0U;
        }
}


// ============================================================
// Dash_RefreshDisplay
//
// Làm mới chu kỳ OLED
//
// Được gọi bởi vòng lặp chính mỗi 100ms; ngắt nhận CAN chỉ cập nhật g_dash_data, không trực tiếp refresh màn hình.
// ============================================================

void Dash_RefreshDisplay(void)
{
    char line[32];
    const char *fault_name;
    uint32_t now_ms;
    uint8_t status_301_unavailable;
    uint8_t soc_unavailable;

    now_ms = GetTick();
    Dash_UpdateCommStatus(now_ms);

    /*
     * 0x301 mang đồng thời điện áp, SOC và trạng thái vận hành.
     * Truyền thông thực sự timeout, hoặc thông báo mất vẫn trong thời hạn giữ 2 giây,
     * cả 3 mục dữ liệu này đều không nên tiếp tục giả mạo làm dữ liệu thời gian thực.
     */
    status_301_unavailable =
        ((g_dash_data.comm_301_timeout != 0U) ||
         (g_dash_data.comm_301_notice_active != 0U)) ? 1U : 0U;
    soc_unavailable =
        ((status_301_unavailable != 0U) ||
         (g_dash_data.received_301 == 0U) ||
         (g_dash_data.soc == 0xFFU)) ? 1U : 0U;

    // Đã nhận bản tin BMS nhưng sau đó liên tục mất toàn bộ 1000ms, báo cáo ngắt toàn bộ truyền thông CAN.
    if ((g_dash_data.valid != 0U) &&
        ((uint32_t)(now_ms - g_dash_data.last_rx_tick) > 1000U))
    {
        OLED_Clear();
        OLED_ShowString(0,0,"BMS DASH");
        OLED_ShowString(0,2,"CAN LOST!");
        OLED_Refresh();
        return;
    }

    //----------------------------------------------------------
    // Chưa từng nhận dữ liệu CAN
    //----------------------------------------------------------

    if(g_dash_data.valid == 0)
    {
        OLED_Clear();

        OLED_ShowString(0,0,"BMS DASH");
        OLED_ShowString(0,2,"Waiting CAN...");

        OLED_Refresh();
			 g_dash_data.valid = 0;  // Thêm dòng này, xóa cờ, cho phép lần cập nhật tiếp theo

        return;
    }

    //----------------------------------------------------------
    // Kiểm tra timeout CAN
    //----------------------------------------------------------

    //----------------------------------------------------------
    // Hiển thị bình thường
    //----------------------------------------------------------

    OLED_Clear();

    OLED_ShowString(2,0,"=== BMS DASH ===");

    //----------------------------------------------------------
    // Trạng thái sạc / xả
    //----------------------------------------------------------

    if (status_301_unavailable != 0U)
    {
        OLED_ShowString(0,1,"STATUS:CAN LOST");
    }
    else
    {
        switch(g_dash_data.chg_state)
        {
            case CHG_STATE_DISCHARGE:

                OLED_ShowString(0,1,"DISCHARGE");

                break;

            case CHG_STATE_CHARGE:

                OLED_ShowString(0,1,"CHARGING");

                break;

            case CAN_STATUS_MONITOR:

                OLED_ShowString(0,1,"MONITOR");

                break;

            default:

                OLED_ShowString(0,1,"STANDBY");

                break;
        }
    }


    //----------------------------------------------------------
    // Điện áp
    //
    // Đơn vị cell_mv:
    // mV
    //
    // 4150
    // ->
    // 4.15V
    //----------------------------------------------------------

    if (status_301_unavailable != 0U)
    {
        OLED_ShowString(0,2,"Cell:CAN LOST");
    }
    else
    {
        snprintf(
            line,
            sizeof(line),
            "Cell:%d.%02dV",
            g_dash_data.cell_mv / 1000U,
            (g_dash_data.cell_mv % 1000U) / 10U);

        OLED_ShowString(0,2,line);
    }

    
    //----------------------------------------------------------
    // Nhiệt độ
    //
    // Khi truyền thông bình thường thì hiển thị nhiệt độ nhận gần nhất;

    // Khi liên tục 1500ms không nhận được bản tin nhiệt độ hợp lệ,

    // không tiếp tục giả mạo nhiệt độ cũ làm nhiệt độ thời gian thực.
    //----------------------------------------------------------

    if (g_dash_data.comm_302_timeout != 0U)
    {
        OLED_ShowString(0,3,"Temp:CAN LOST");
    }
    else
    {
        snprintf(
            line,
            sizeof(line),
            "Temp:%dC",
            g_dash_data.temp);

        OLED_ShowString(0,3,line);
    }

    //----------------------------------------------------------
    // SOC đến từ 0x301, được ECU-A ước tính tuyến tính dựa trên điện áp pin.
    //----------------------------------------------------------

    if (soc_unavailable != 0U)
    {
        OLED_ShowString(0,4,"SOC:--");
    }
    else
    {
        snprintf(line, sizeof(line),
                 "SOC:%u%%", (unsigned int)g_dash_data.soc);
        OLED_ShowString(0,4,line);
    }

//----------------------------------------------------------
// Hiển thị lỗi
//
// Khi truyền thông bình thường thì hiển thị lỗi đang hoạt động hiện tại;
// Khi bản tin lỗi bị timeout thì giữ lại mặt nạ lỗi tin cậy cuối cùng,
// đồng thời nêu rõ dữ liệu này đã dừng cập nhật.
//----------------------------------------------------------

    if (g_dash_data.comm_303_timeout != 0U)
    {
        if (g_dash_data.received_303 != 0U)
        {
            /*
            * Đã từng nhận bản tin lỗi hợp lệ:
            * Giữ lại mặt nạ tin cậy cuối cùng, nhưng đánh dấu truyền thông đã bị mất.
            */
            snprintf(
                line,
                sizeof(line),
                "F:%04X CAN LOST",
                (unsigned int)g_dash_data.fault);

            OLED_ShowString(0,5,line);
        }
        else
        {
            /*
            * Sau cấp nguồn chưa từng nhận bản tin lỗi hợp lệ,

            * lúc này không có mặt nạ lỗi lịch sử tin cậy.
            */
            OLED_ShowString(0,5,"Fault:CAN LOST");
        }
    }
    else if (g_dash_data.fault == 0U)
    {
        OLED_ShowString(0,5,"Fault:NONE");
    }
    else
    {
        /*
         * Một dòng OLED không thể viết hết tên tất cả các lỗi, nên hiển thị mặt nạ đầy đủ,
         * sau đó hiển thị tên viết tắt của một lỗi chính theo mức ưu tiên an toàn; mặt nạ vẫn giữ toàn bộ thông tin lỗi.
         */
        if((g_dash_data.fault & CAN_FAULT_SELF_CHECK) != 0U)
            fault_name = "SELF";
        else if((g_dash_data.fault & CAN_FAULT_ADC_TIMEOUT) != 0U)
            fault_name = "ADC";
        else if((g_dash_data.fault & CAN_FAULT_SENSOR_DATA) != 0U)
            fault_name = "DATA";
        else if((g_dash_data.fault & CAN_FAULT_NTC_OPEN) != 0U)
            fault_name = "NTCO";
        else if((g_dash_data.fault & CAN_FAULT_NTC_SHORT) != 0U)
            fault_name = "NTCS";
        else if((g_dash_data.fault & CAN_FAULT_OT) != 0U)
            fault_name = "OT";
        else if((g_dash_data.fault & CAN_FAULT_OV) != 0U)
            fault_name = "OV";
        else if((g_dash_data.fault & CAN_FAULT_UV) != 0U)
            fault_name = "UV";
        else
            fault_name = "OTHER";

        snprintf(line,
                 sizeof(line),
                 "F:%04X %s",
                 (unsigned int)g_dash_data.fault,
                 fault_name);
        OLED_ShowString(0,5,line);
    }

    //----------------------------------------------------------
    // Làm mới OLED
    //----------------------------------------------------------

    OLED_Refresh();
}
