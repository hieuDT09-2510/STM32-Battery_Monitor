/*
 * Mô-đun: Driver XM25QH32 SPI Flash ngoài.
 * Kết nối: SPI1 dùng PA4-CS, PA5-SCK, PA6-MISO, PA7-MOSI.
 * Timing: Clock SPI hiện tại là 72MHz/64=1.125MHz, dự trữ biên độ tín hiệu cho dây nối DuPont.
 * Chức năng: Đọc trạng thái, đọc Device ID, đọc thông thường, nạp trang (page program) và xóa sector 4KB.
 * Ranh giới: Mô-đun này không nhận biết bản ghi FlashLog, cũng không quyết định chiến lược ghi log.
 */
#include "stm32f10x.h"
#include "bsp_spi_flash.h"

/*
 * Bộ đệm 4KB dùng cho API ghi đè toàn bộ sector chỉ thuộc về driver này.
 * Đặt trong file .c có thể tránh việc mỗi mô-đun include header lại tự tạo một mảng tĩnh riêng.
 */
static uint8_t SectorBuffer[SPI_FLASH_SECTOR_SIZE];

void SPI1_Init(void)
{
	// 1. Bật clock GPIOA/SPI1
	RCC->APB2ENR |= (1U<<12);
	RCC->APB2ENR |= (1U<<2);
	
	// 2. Cấu hình PA4-CS push-pull đa dụng, PA5-SCK push-pull alternate function 
	// PA6-MISO floating input, PA7-MOSI push-pull alternate function
	// Kéo chốt đầu ra CS lên mức cao trước, tránh xung thấp khi PA4 chuyển sang chế độ output.
	GPIOA->BSRR = (1U << 4);
	GPIOA->CRL &= ~(0xFU << 16); 
	GPIOA->CRL |= (0x2U << 16); // PA4: 2MHz push-pull đa dụng, CS chỉ cần chuyển mạch tốc độ thấp
    GPIOA->CRL &= ~(0xFU << 20); 
	GPIOA->CRL |= (0x9U << 20); // PA5: 10MHz push-pull AF, giảm rung sườn SCK
    GPIOA->CRL &= ~(0xFU << 24); 
	GPIOA->CRL |= (0x4U << 24);
    GPIOA->CRL &= ~(0xFU << 28); 
	GPIOA->CRL |= (0x9U << 28); // PA7: 10MHz push-pull AF, giảm rung sườn MOSI
	
	// 3. Chip select CS, host không chọn
	GPIOA->BSRR = (1U<<4);
	
	// 4. Cấu hình SPI1, Mode 0
	SPI1->CR1 = 0;
	SPI1->CR1 &= ~(1<<1); // Cực tính clock CPOL
	SPI1->CR1 &= ~(1<<0); // Pha clock CPHA
	
	SPI1->CR1 |= (1<<2); // MSTR chọn thiết bị Master
	/*
	 * BR=101: PCLK2 chia 64, clock SPI là 1.125MHz.
	 * Thông lượng log rất nhỏ, ưu tiên đảm bảo độ ổn định đọc/ghi dưới kết nối dây DuPont.
	 */
	SPI1->CR1 |= (5U<<3);
    SPI1->CR1 |= (1<<8); // SSI chọn Slave nội bộ
	SPI1->CR1 |= (1<<9); // SSM điều khiển Slave bằng phần mềm
	SPI1->CR1 |= (1<<6); // SPE bật SPI
	
}

/*
 * SPI1 truyền nhận một byte.
 * Chờ thanh ghi truyền trống trước, rồi mới ghi dữ liệu gửi;
 * sau đó chờ thanh ghi nhận không rỗng, đọc và trả về giá trị nhận được lần này.
 */
uint8_t SPI1_SwapByte(uint8_t tx_data)
{
    // Chờ thanh ghi dữ liệu truyền cho phép ghi.
    while ((SPI1->SR & (1U << 1)) == 0U)
    {
    }

    // Sử dụng truy cập 8-bit, ghi byte cần gửi lần này.
    *(volatile uint8_t *)&SPI1->DR = tx_data;

    // Chờ trao đổi SPI lần này nhận được một byte.
    while ((SPI1->SR & (1U << 0)) == 0U)
    {
    }

    // Đọc dữ liệu nhận được, đồng thời xóa điều kiện RXNE.
    return *(volatile uint8_t *)&SPI1->DR;
}


// Đọc thanh ghi FLASH
uint8_t XM25QH32_ReadStatusReg(void)
{
  uint8_t status;
  SPI_FLASH_CS_LOW();
  SPI1_SwapByte(0x05); // Gửi 0x05
	status = SPI1_SwapByte(0xFF); // status = giá trị nhận được
	  
	// Chờ BSY=0, cờ bận không bận
 	while(SPI1->SR & (1 << 7));
  SPI_FLASH_CS_HIGH();
	
  return status;
}


// Cờ bận FLASH
void XM25QH32_WaitBusy(void)
{
    while(XM25QH32_ReadStatusReg() & 0x01);
}


// Cho phép ghi FLASH (Write Enable)
void XM25QH32_WriteEnable(void)
{
    SPI_FLASH_CS_LOW();
    SPI1_SwapByte(0x06);
    while(SPI1->SR & (1 << 7));
    SPI_FLASH_CS_HIGH();
}


// Đọc ID FLASH
uint16_t XM25QH32_ReadID(void)
{
  uint8_t id1, id2;
  SPI_FLASH_CS_LOW();
  SPI1_SwapByte(0x90);
  SPI1_SwapByte(0x00); 
	SPI1_SwapByte(0x00); 
	SPI1_SwapByte(0x00);
  id1 = SPI1_SwapByte(0xFF);
  id2 = SPI1_SwapByte(0xFF);
  while(SPI1->SR & (1 << 7));
  SPI_FLASH_CS_HIGH();
  
	return ((uint16_t)id1 << 8) | id2;
}



// Đọc dữ liệu
void XM25QH32_ReadData(uint32_t addr, uint8_t *buffer, uint16_t len)
{
    SPI_FLASH_CS_LOW();
    SPI1_SwapByte(0x03);
    SPI1_SwapByte((addr >> 16) & 0xFF);
    SPI1_SwapByte((addr >> 8) & 0xFF);
    SPI1_SwapByte(addr & 0xFF);
    for(uint16_t i = 0; i < len; i++)
        buffer[i] = SPI1_SwapByte(0xFF);
    while(SPI1->SR & (1 << 7));
    SPI_FLASH_CS_HIGH();
}


// Nạp trang (Page Program)
void XM25QH32_PageProgram(uint32_t addr, uint8_t *buffer, uint16_t len)
{
	if(len > 256)
	{
		len = 256;
	}
    
    XM25QH32_WaitBusy();
    XM25QH32_WriteEnable();
	
    SPI_FLASH_CS_LOW();
	
    SPI1_SwapByte(0x02);
    SPI1_SwapByte((addr >> 16) & 0xFF);
    SPI1_SwapByte((addr >> 8) & 0xFF);
    SPI1_SwapByte(addr & 0xFF);
	
    for(uint16_t i = 0; i < len; i++)
	{
		SPI1_SwapByte(buffer[i]);
	}
  
    while(SPI1->SR & (1 << 7));
    SPI_FLASH_CS_HIGH();
    XM25QH32_WaitBusy();
}


// Xóa sector
void XM25QH32_SectorErase(uint32_t addr)
{
    XM25QH32_WaitBusy();
    XM25QH32_WriteEnable();
    SPI_FLASH_CS_LOW();
    SPI1_SwapByte(0x20);
    SPI1_SwapByte((addr >> 16) & 0xFF);
    SPI1_SwapByte((addr >> 8) & 0xFF);
    SPI1_SwapByte(addr & 0xFF);
    while(SPI1->SR & (1 << 7));
    SPI_FLASH_CS_HIGH();
    XM25QH32_WaitBusy();
}


// Ghi dữ liệu -- Đã xác nhận vùng Flash ở trạng thái đã xóa (toàn 0xFF), ghi trực tiếp
void XM25QH32_Write_NoCheck(uint8_t *buffer, uint32_t addr, uint16_t len)
{
    uint16_t page_remain = 256 - (addr % 256);
    if(page_remain == 0) page_remain = 256;

    while(len)
    {
        if(len <= page_remain)
        {
            XM25QH32_PageProgram(addr, buffer, len);
            break;
        }
        else
        {
            XM25QH32_PageProgram(addr, buffer, page_remain);
            addr += page_remain;
            buffer += page_remain;
            len -= page_remain;
            page_remain = 256;
        }
    }
}




// Ghi dữ liệu FLASH -- Không chắc chắn trạng thái Flash, tự động đọc sector → phán đoán → xóa → ghi lại
void XM25QH32_Write(
    uint8_t *buffer,
    uint32_t addr,
    uint16_t len)
{
    uint32_t sector_pos;
    uint16_t sector_offset;
    uint16_t sector_remain;
    uint16_t i;

    sector_pos = addr / SPI_FLASH_SECTOR_SIZE;

    sector_offset = addr % SPI_FLASH_SECTOR_SIZE;

    sector_remain = SPI_FLASH_SECTOR_SIZE - sector_offset;

    if(len <= sector_remain)
    {
        sector_remain = len;
    }

    while(1)
    {
        /* Đọc toàn bộ sector */
        XM25QH32_ReadData(
            sector_pos * SPI_FLASH_SECTOR_SIZE,
            SectorBuffer,
            SPI_FLASH_SECTOR_SIZE);

        /* Phán đoán có cần xóa không */
        for(i = 0; i < sector_remain; i++)
        {
            if(SectorBuffer[sector_offset + i] != 0xFF)
            {
                break;
            }
        }

        if(i < sector_remain)
        {
            /* Cần xóa */

            XM25QH32_SectorErase(
                sector_pos * SPI_FLASH_SECTOR_SIZE);

            /* Sửa đổi bộ đệm */

            for(i = 0; i < sector_remain; i++)
            {
                SectorBuffer[sector_offset + i]
                    = buffer[i];
            }

            /* Ghi lại toàn bộ sector */

            XM25QH32_Write_NoCheck(
                SectorBuffer,
                sector_pos * SPI_FLASH_SECTOR_SIZE,
                SPI_FLASH_SECTOR_SIZE);
        }
        else
        {
            /* Ghi trực tiếp */

            XM25QH32_Write_NoCheck(
                buffer,
                addr,
                sector_remain);
        }

        if(len == sector_remain)
        {
            break;
        }

        sector_pos++;

        sector_offset = 0;

        buffer += sector_remain;

        addr += sector_remain;

        len -= sector_remain;

        if(len > SPI_FLASH_SECTOR_SIZE)
        {
            sector_remain = SPI_FLASH_SECTOR_SIZE;
        }
        else
        {
            sector_remain = len;
        }
    }
}
