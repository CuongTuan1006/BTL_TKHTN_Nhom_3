/*
 * CCS811.h
 *
 *  Created on: Oct 27, 2024
 *      Author: DELL
 */

#ifndef INC_CCS811_H_
#define INC_CCS811_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"

// Định nghĩa các thanh ghi của CCS811
#define CCS811_ADDR 0x5A << 1
#define CCS811_STATUS 0x00
#define CCS811_MEAS_MODE 0x01
#define CCS811_ALG_RESULT_DATA 0x02
#define CCS811_APP_START 0xF4
#define CCS811_HW_ID 0x20
#define CCS811_BASELINE 0x11           // Thanh ghi baseline
#define CCS811_HW_VERSION 0x21        // Phiên bản phần cứng
#define CCS811_FW_BOOT_VERSION 0x23   // Phiên bản firmware khi khởi động
#define CCS811_FW_APP_VERSION 0x24    // Phiên bản firmware khi chạy ứng dụng
#define CCS811_ERROR_ID 0xE0          // Mã lỗi
#define CCS811_ENV_DATA 0x05          // Dữ liệu nhiệt độ và độ ẩm
#define CCS811_THRESHOLDS 0x10        // Ngưỡng ngắt
#define CCS811_RAW_DATA 0x03          // Dữ liệu thô từ ADC
#define CCS811_WAKE 0xF6              // Lệnh đánh thức cảm biến
#define CCS811_RESET 0xFF             // Lệnh reset cảm biến
// Các chế độ hoạt động của CCS811
#define CCS811_MODE_SLEEP 0x00   // Chế độ ngủ
#define CCS811_MODE_1SEC 0x01    // Chế độ lấy mẫu 1 giây
#define CCS811_MODE_10SEC 0x02    // Chế độ lấy mẫu 10 giây
#define CCS811_MODE_60SEC 0x03    // Chế độ lấy mẫu 60 giây

// Các hàm giao tiếp I2C cơ bản
uint8_t CCS811_ReadReg(uint8_t addr);
void CCS811_ReadRegMulti(uint8_t addr, uint8_t *val, uint8_t size);
void CCS811_WriteReg(uint8_t addr, uint8_t val);
void CCS811_WriteRegMulti(uint8_t addr, uint8_t *data, uint8_t length);

// Cấu hình chế độ hoạt động
void CCS811_Init();
void CCS811_Set_Mode(uint8_t mode);
uint8_t CCS811_CheckHW_ID() ;
// Khởi tạo và điều khiển cảm biến

void CCS811_DisableInt(void);
void CCS811_EnableInt(void);

// Đọc giá trị CO₂ và baseline
void CCS811_Read_Co2(uint16_t *eCO2);
void CCS811_Read_TVOC(uint16_t *Tvoc);
uint16_t CCS811_GetBaseline();
void CCS811_RestoreBaseline(uint16_t baseline);
#endif /* INC_CCS811_H_ */
