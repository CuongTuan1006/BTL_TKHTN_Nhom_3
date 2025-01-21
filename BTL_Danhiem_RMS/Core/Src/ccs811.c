/*
 * CCS811.c
 *
 *  Created on: Oct 27, 2024
 *      Author: DELL
 */
#include "CCS811.h"

extern I2C_HandleTypeDef hi2c2;

#define hi2cxc hi2c2


void CCS811_Reset(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 0);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, 1);
}

// Hàm đọc từ 1 thanh ghi (1byte)
uint8_t CCS811_ReadReg(uint8_t addr)
{
    uint8_t result = 0;
    HAL_I2C_Mem_Read(&hi2cxc, CCS811_ADDR, addr, I2C_MEMADD_SIZE_8BIT, &result, 1, 100);
    return result;
}
// hàm đọc nhiều byte từ thanh ghi
void CCS811_ReadRegMulti(uint8_t addr, uint8_t * val,uint8_t size)
{
	HAL_I2C_Mem_Read( &hi2cxc, CCS811_ADDR, ( uint8_t )addr, I2C_MEMADD_SIZE_8BIT, val, size,100 );
}

// Hàm viết vào 1 thanh ghi 1byte
void CCS811_WriteReg(uint8_t addr, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2cxc, CCS811_ADDR, addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

// Hàm viết nhiều byte vào thanh ghi
void CCS811_WriteRegMulti(uint8_t addr, uint8_t *data, uint8_t length)
{
    HAL_I2C_Mem_Write(&hi2cxc, CCS811_ADDR, addr, I2C_MEMADD_SIZE_8BIT, data, length, 100);
}
// Hàm khởi tạo CCS811
void CCS811_Init() {
    uint8_t status[1] = {0};
    int Init_OK = 0;
    while(!Init_OK)
    {
		// Kiểm tra trạng thái cảm biến
    	status[0]=CCS811_ReadReg(CCS811_STATUS);
		if ((status[0] & 0x10) == 0)
		{  // Kiểm tra nếu chưa sẵn sàng (bit 4 là 0)
			Init_OK = 0;
		}
		else
		{
			// Gửi lệnh bắt đầu ứng dụng
			uint8_t app_start = CCS811_APP_START;
			HAL_I2C_Master_Transmit(&hi2cxc, CCS811_ADDR, &app_start, 1, HAL_MAX_DELAY);
			// Kiểm tra lại trạng thái
			status[0]=CCS811_ReadReg(CCS811_STATUS);
			if ((status[0] & 0x90) != 0x90) //bit 7 và 4=1
			{  // Kiểm tra nếu sẵn sàng và không có lỗi
				Init_OK = 0;
			}
			else
			{
				Init_OK = 1;
			}

		}
    }
}
void CCS811_Set_Mode(uint8_t mode)
{
   // Chế độ đo
	mode = mode <<4;
    CCS811_WriteReg(CCS811_MEAS_MODE, mode);
}

// Hàm set chế độ
/*
        //Mode 0 = Idle
	    //Mode 1 = read every 1s
	    //Mode 2 = every 10s
	    //Mode 3 = every 60s
	    //Mode 4 = RAW mode.
 */

//Hàm đọc giá trị Co2
void CCS811_Read_Co2(uint16_t *eCO2)
{
    uint8_t buf[8]={0};
    CCS811_ReadRegMulti(CCS811_ALG_RESULT_DATA, buf, 8);
    // Ghép hai byte đầu để lấy giá trị eCO2
    *eCO2 = (buf[0] << 8) | buf[1];
}

void CCS811_Read_TVOC(uint16_t *Tvoc)
{
    uint8_t buf[8]={0};
    CCS811_ReadRegMulti(CCS811_ALG_RESULT_DATA, buf, 8);
    // Ghép hai byte đầu để lấy giá trị eCO2
    *Tvoc = (buf[2] << 8) | buf[3];
}
uint8_t CCS811_CheckHW_ID() {
    uint8_t hw_id = 0;
    // xác mã định danh phần cứng từ thanh ghi HW_ID

    hw_id = CCS811_ReadReg(CCS811_HW_ID);
    // Kiểm tra mã định danh phần cứng có khớp với giá trị mặc định không
    if (hw_id == 0x81)
    {
        return 1; // Mã định danh phần cứng đúng
    }
    else
    {
        return 0; // Mã định danh phần cứng sai
    }
}

//Enable the nINT signal
void CCS811_EnableInt(void)
{
	uint8_t setting = CCS811_ReadReg(CCS811_MEAS_MODE); //đọc giá trị lúc trước
	setting |= (1 << 3); //Set INTERRUPT bit ( chế độ khi có giá trị mới sẽ ngắt)
	CCS811_WriteReg(CCS811_MEAS_MODE, setting);
}

//Disable the nINT signal
void CCS811_DisableInt(void)
{
	uint8_t setting = CCS811_ReadReg(CCS811_MEAS_MODE); //đọc giá trị lúc trước
	setting &= ~(1 << 3); //Clear INTERRUPT bit
	CCS811_WriteReg(CCS811_MEAS_MODE, setting);
}


// Hàm lấy giá trị baseline hiện tại
uint16_t CCS811_GetBaseline()
{
    uint8_t baseline_data[2];
    CCS811_ReadRegMulti(CCS811_BASELINE, baseline_data, 2);
    return (baseline_data[0] << 8) | baseline_data[1];
}

// Hàm khôi phục giá trị baseline từ EEPROM
void CCS811_RestoreBaseline(uint16_t baseline)
{
    uint8_t baseline_data[2] = {baseline >> 8, baseline & 0xFF};
    CCS811_WriteRegMulti(CCS811_BASELINE, baseline_data, 2);
}

