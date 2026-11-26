#include "bsp_i2c.h"
#include "stm32f10x.h"

/* PB6 làm SCL, PB7 làm SDA. */
#define I2C_SCL_PIN        6U
#define I2C_SDA_PIN        7U

/* Trễ phần mềm đơn giản, dùng để điều khiển định thời I2C. */
static void I2C1_Delay(void)
{
    volatile uint32_t i;

    for(i = 0U; i < 30U; i++)
    {
        __NOP();
    }
}

/* Kéo SCL lên cao: ở chế độ open-drain thực chất là nhả bus. */
static void I2C1_SCL_High(void)
{
    GPIOB->BSRR = (1U << I2C_SCL_PIN);
}

/* Kéo SCL xuống thấp. */
static void I2C1_SCL_Low(void)
{
    GPIOB->BRR = (1U << I2C_SCL_PIN);
}

/* Kéo SDA lên cao: ở chế độ open-drain thực chất là nhả bus. */
static void I2C1_SDA_High(void)
{
    GPIOB->BSRR = (1U << I2C_SDA_PIN);
}

/* Kéo SDA xuống thấp. */
static void I2C1_SDA_Low(void)
{
    GPIOB->BRR = (1U << I2C_SDA_PIN);
}

/* Đọc mức logic thực tế hiện tại của SDA. */
static uint8_t I2C1_ReadSDA(void)
{
    if((GPIOB->IDR & (1U << I2C_SDA_PIN)) != 0U)
    {
        return 1U;
    }

    return 0U;
}

void I2C1_Init(void)
{
    /* Bật xung nhịp GPIOB. */
    RCC->APB2ENR |= (1U << 3);

    /* Cấu hình PB6 làm đầu ra open-drain thông dụng 50MHz. */
    GPIOB->CRL &= ~(0xFU << 24);
    GPIOB->CRL |=  (0x7U << 24);

    /* Cấu hình PB7 làm đầu ra open-drain thông dụng 50MHz. */
    GPIOB->CRL &= ~(0xFU << 28);
    GPIOB->CRL |=  (0x7U << 28);

    /* Trạng thái rỗi I2C: cả SCL và SDA đều ở mức cao. */
    I2C1_SCL_High();
    I2C1_SDA_High();

    I2C1_Delay();
}

void I2C1_Start(void)
{
    /* Nhả SDA và SCL trước. */
    I2C1_SDA_High();
    I2C1_SCL_High();

    I2C1_Delay();

    /* Khi SCL ở mức cao, SDA chuyển từ cao xuống thấp để tạo điều kiện START. */
    I2C1_SDA_Low();

    I2C1_Delay();

    /* Kéo SCL xuống thấp, bước vào giai đoạn truyền dữ liệu. */
    I2C1_SCL_Low();

    I2C1_Delay();
}

void I2C1_Stop(void)
{
    /* Đảm bảo SDA ở mức thấp trước. */
    I2C1_SDA_Low();

    I2C1_Delay();

    /* Nhả SCL. */
    I2C1_SCL_High();

    I2C1_Delay();

    /* Khi SCL ở mức cao, SDA chuyển từ thấp lên cao để tạo điều kiện STOP. */
    I2C1_SDA_High();

    I2C1_Delay();
}

void I2C1_SendData(uint8_t data)
{
    uint8_t i;

    /* Dữ liệu I2C được truyền bắt đầu từ bit cao nhất (MSB). */
    for(i = 0U; i < 8U; i++)
    {
        /* Gửi bit cao nhất hiện tại. */
        if((data & 0x80U) != 0U)
        {
            I2C1_SDA_High();
        }
        else
        {
            I2C1_SDA_Low();
        }

        I2C1_Delay();

        /* Kéo SCL lên cao, slave lấy mẫu SDA trong khoảng thời gian này. */
        I2C1_SCL_High();

        I2C1_Delay();

        /* Kết thúc một chu kỳ xung nhịp. */
        I2C1_SCL_Low();

        I2C1_Delay();

        /* Dịch bit tiếp theo lên vị trí cao nhất. */
        data <<= 1;
    }

    /* Sau khi gửi xong thì nhả SDA, chuẩn bị đọc ACK. */
    I2C1_SDA_High();
}

uint8_t I2C1_WaitAck(void)
{
    uint8_t ack;

    /* Nhả SDA để slave điều khiển đường SDA. */
    I2C1_SDA_High();

    I2C1_Delay();

    /* Kéo xung nhịp thứ 9 lên cao. */
    I2C1_SCL_High();

    I2C1_Delay();

    /* SDA=0 biểu thị slave phát ACK. */
    ack = I2C1_ReadSDA();

    /* Kết thúc xung nhịp ACK. */
    I2C1_SCL_Low();

    I2C1_Delay();

    if(ack == 0U)
    {
        return 0U;
    }

    return 1U;
}

void I2C1_SendAddr(uint8_t addr_7bit, uint8_t rw)
{
    uint8_t addr_byte;

    /* Dịch trái địa chỉ 7-bit, thêm bit đọc/ghi vào bit thấp nhất. */
    addr_byte = (uint8_t)((addr_7bit << 1) | (rw & 0x01U));

    /* Gửi byte địa chỉ. */
    I2C1_SendData(addr_byte);

    /* Chờ ACK từ slave. */
    (void)I2C1_WaitAck();
}

void I2C1_WriteBuffer(uint8_t dev_addr, uint8_t *buffer, uint16_t len)
{
    uint16_t i;

    /* Bảo vệ tham số. */
    if((buffer == 0) || (len == 0U))
    {
        return;
    }

    /* Tạo điều kiện START. */
    I2C1_Start();

    /* Gửi địa chỉ thiết bị, rw=0 biểu thị ghi. */
    I2C1_SendAddr(dev_addr, 0U);

    /* Gửi dữ liệu liên tiếp. */
    for(i = 0U; i < len; i++)
    {
        I2C1_SendData(buffer[i]);

        /* Mỗi byte gửi xong đều chờ ACK. */
        (void)I2C1_WaitAck();
    }

    /* Tạo điều kiện STOP. */
    I2C1_Stop();
}
