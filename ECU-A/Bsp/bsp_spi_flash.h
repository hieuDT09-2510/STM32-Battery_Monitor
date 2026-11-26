/*
 * Mô-đun: Giao diện cấp board cho bộ nhớ SPI Flash ngoài XM25QH32.
 * Chức năng: Cung cấp các hàm nhận diện thiết bị, đọc trạng thái, lập trình trang (page program), xóa sector và đọc dữ liệu.
 * Phạm vi: Mô-đun này chỉ xử lý giao thức phần cứng Flash, không giải mã định dạng dữ liệu của nhật ký lỗi.
 */
#ifndef __BSP_SPI_FLASH_H
#define __BSP_SPI_FLASH_H

#include "stm32f10x.h"

#define SPI_FLASH_CS_LOW()   GPIOA->BRR = (1 << 4)
#define SPI_FLASH_CS_HIGH()  GPIOA->BSRR = (1 << 4)
#define SPI_FLASH_SECTOR_SIZE     4096

// Khởi tạo SPI
void SPI1_Init(void);

// Thu phát dữ liệu SPI
uint8_t SPI1_SwapByte(uint8_t tx_data);

// Đọc thanh ghi trạng thái FLASH
uint8_t XM25QH32_ReadStatusReg(void);

// Chờ cờ bận FLASH
void XM25QH32_WaitBusy(void);

// Cho phép ghi FLASH
void XM25QH32_WriteEnable(void);

// Đọc ID của FLASH
uint16_t XM25QH32_ReadID(void);

// Đọc dữ liệu FLASH
void XM25QH32_ReadData(uint32_t addr, uint8_t *buffer, uint16_t len);

// Lập trình trang (Page Program)
void XM25QH32_PageProgram(uint32_t addr, uint8_t *buffer, uint16_t len);

// Xóa sector
void XM25QH32_SectorErase(uint32_t addr);

// Ghi dữ liệu -- khi đã xác nhận vùng Flash ở trạng thái đã xóa (toàn 0xFF), ghi trực tiếp
void XM25QH32_Write_NoCheck(uint8_t *buffer, uint32_t addr, uint16_t len);

// Ghi dữ liệu FLASH -- khi chưa rõ trạng thái Flash, tự động đọc sector -> kiểm tra -> xóa -> ghi lại
void XM25QH32_Write(uint8_t *buffer,uint32_t addr,uint16_t len);

#endif
