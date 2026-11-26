/*
 * Tên file: bsp_usart.c
 *
 * Tên mô-đun: Driver nhận ngắt USART1 và bộ đệm vòng
 *
 * Trách nhiệm mô-đun:
 * 1. Cấu hình PA9/PA10 và USART1, baudrate 115200;
 * 2. Khi ngắt RXNE đến lập tức đọc DR, tránh polling task không kịp thời gây tràn phần cứng (ORE);
 * 3. Sử dụng bộ đệm vòng tĩnh 64 byte lưu các byte nhận được；
 * 4. Cho phép task đọc bộ đệm từng byte qua API non-blocking;
 * 5. Ghi nhận số byte nhận được, lỗi phần cứng và số lần bộ đệm phần mềm bị đầy.
 *
 * Luồng dữ liệu:
 * Ngắt RXNE USART1
 *        -> USART1_RxIRQHandler()
 *        -> Bộ đệm vòng tĩnh
 *        -> USART1_ReadByteNonBlocking()
 *        -> CLI_Process()
 *
 * Quy tắc đồng thời:
 * Ngắt là producer duy nhất, chỉ sửa đổi vị trí ghi;
 * Task CLI là consumer duy nhất, chỉ sửa đổi vị trí đọc;
 * Trong ngắt không thực thi printf, truy cập Flash hoặc phân tích lệnh.
 */
#include "bsp_usart.h"
#include <stdio.h>
#include "stm32f10x.h"

/*
 * 64 byte đủ để lưu nhiều lệnh ngắn CLI hiện tại, và hoàn toàn sử dụng RAM tĩnh,
 * không chiếm dụng Heap FreeRTOS vốn đã tương đối hạn hẹp hiện tại.
 */
#define USART1_RX_BUFFER_SIZE        64U

static uint8_t g_usart1_rx_buffer[USART1_RX_BUFFER_SIZE];

/* head trỏ tới vị trí ghi tiếp theo, do ngắt USART1 sửa đổi. */
static volatile uint16_t g_usart1_rx_head = 0U;

/* tail trỏ tới vị trí đọc tiếp theo, do task CLI sửa đổi. */
static volatile uint16_t g_usart1_rx_tail = 0U;

volatile uint32_t g_usart1_rx_byte_count = 0U;
volatile uint32_t g_usart1_rx_hw_error_count = 0U;
volatile uint32_t g_usart1_rx_buffer_overflow_count = 0U;


/* Trả về vị trí tiếp theo trong bộ đệm vòng. */
static uint16_t USART1_RxNextIndex(uint16_t index)
{
    index++;

    if (index >= USART1_RX_BUFFER_SIZE)
    {
        index = 0U;
    }

    return index;
}


void USART1_Init(void)
{
	volatile uint32_t clear_value;

	// 1. Bật clock GPIOA/USART1
	RCC->APB2ENR |= (1<<2);
	RCC->APB2ENR |= (1<<14);
	
	// 2. Cấu hình PA9--TX PA10--RX
	GPIOA->CRH &= ~(0XF<<4);
	GPIOA->CRH |= (0XB<<4);
	
	GPIOA->CRH &=  ~(0XF<<8);
	GPIOA->CRH |= (0X4<<8);
	
	// 3. Cấu hình baudrate 115200
	USART1->BRR = 0X0270;
	
	/* Xóa sạch trạng thái nhận phần mềm và bộ đếm chẩn đoán. */
	g_usart1_rx_head = 0U;
	g_usart1_rx_tail = 0U;
	g_usart1_rx_byte_count = 0U;
	g_usart1_rx_hw_error_count = 0U;
	g_usart1_rx_buffer_overflow_count = 0U;

	// 4. Bật USART / cho phép nhận / cho phép truyền
	USART1->CR1 |= (1<<2);
	USART1->CR1 |= (1<<3);
	USART1->CR1 |= (1<<13);

	/*
	 * Đọc SR trước rồi đọc DR, xóa trạng thái nhận và lỗi có thể còn sót trước khi khởi tạo.
	 */
	clear_value = USART1->SR;
	clear_value = USART1->DR;
	(void)clear_value;

	/*
	 * Ngắt USART không gọi API FreeRTOS, nhưng vẫn dùng độ ưu tiên chiếm trước thấp hơn là 6,
	 * để dự trữ biên độ an toàn cho việc chuyển sang dùng API FromISR sau này.
	 */
	NVIC_SetPriority(
		USART1_IRQn,
		NVIC_EncodePriority(
			NVIC_PriorityGroup_4,
			6U,
			0U));
	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);

	/* Cuối cùng mới bật ngắt nhận không rỗng, tránh vào ngắt khi khởi tạo chưa hoàn tất. */
	USART1->CR1 |= USART_CR1_RXNEIE;
}


void USART1_SendByte(uint8_t data)
{
    // Chờ thanh ghi dữ liệu truyền trống, tránh ghi đè byte trước chưa kịp gửi.
    while ((USART1->SR & (1U << 7)) == 0U)
    {
    }

    // Ghi byte mới vào thanh ghi dữ liệu truyền.
    USART1->DR = data;
}


// const -- Chỉ đọc, không được sửa đổi
void USART1_SendString(const char *str)
{
	while(*str)
	{
		USART1_SendByte((uint8_t)*str++);
	}
}


/*
 * Thử đọc một ký tự từ bộ đệm vòng tĩnh.
 *
 * Hàm này không còn đọc trực tiếp USART1->DR; DR chỉ do ngắt nhận đọc.
 * Khi bộ đệm rỗng lập tức trả về 0, không block task CLI.
 */
uint8_t USART1_ReadByteNonBlocking(uint8_t *data)
{
    uint16_t tail;

    if (data == 0)
    {
        return 0U;
    }

    tail = g_usart1_rx_tail;

    if (tail == g_usart1_rx_head)
    {
        return 0U;
    }

    *data = g_usart1_rx_buffer[tail];
    g_usart1_rx_tail = USART1_RxNextIndex(tail);

    return 1U;
}


/*
 * Hàm xử lý ngắt cấp board của ngắt nhận USART1.
 *
 * ISR bắt buộc phải đọc SR trước rồi mới đọc DR:
 * như vậy vừa nhận được byte dữ liệu, vừa xóa các trạng thái lỗi phần cứng như ORE, FE, NE và PE.
 */
void USART1_RxIRQHandler(void)
{
    uint32_t status;
    uint8_t data;
    uint16_t head;
    uint16_t next_head;

    status = USART1->SR;

    if ((status & (USART_SR_ORE |
                   USART_SR_NE |
                   USART_SR_FE |
                   USART_SR_PE)) != 0U)
    {
        g_usart1_rx_hw_error_count++;
    }

    if ((status & USART_SR_RXNE) != 0U)
    {
        data = (uint8_t)USART1->DR;
        head = g_usart1_rx_head;
        next_head = USART1_RxNextIndex(head);

        /*
         * next_head bằng tail biểu thị bộ đệm phần mềm đã đầy.
         * Loại bỏ byte mới và ghi nhận số lần, không được ghi đè lên dữ liệu chưa được CLI đọc.
         */
        if (next_head == g_usart1_rx_tail)
        {
            g_usart1_rx_buffer_overflow_count++;
            return;
        }

        g_usart1_rx_buffer[head] = data;
        g_usart1_rx_head = next_head;
        g_usart1_rx_byte_count++;
    }
    else if ((status & (USART_SR_ORE |
                        USART_SR_NE |
                        USART_SR_FE |
                        USART_SR_PE)) != 0U)
    {
        /* Khi không có RXNE nhưng tồn tại lỗi, đọc DR để hoàn tất xóa trạng thái lỗi. */
        data = (uint8_t)USART1->DR;
        (void)data;
    }
}


/* Chờ byte cuối cùng rời hoàn toàn khỏi thanh ghi dịch, dùng trước khi reset để đảm bảo thông báo đã gửi xong. */
void USART1_WaitSendComplete(void)
{
    while ((USART1->SR & USART_SR_TC) == 0U)
    {
    }
}


int fputc(int ch, FILE *f)
{
    USART1_SendByte((uint8_t)ch);
    return ch;
}
/* Tên vector ngắt USART1 trong startup_stm32f10x_md.s */
void USART1_IRQHandler(void)
{
    USART1_RxIRQHandler();
}
