/** Đặt tệp này vào thư mục src **/

#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;  // thay đổi biến handler của bạn tại đây cho phù hợp

#define SLAVE_ADDRESS_LCD 0x4E // thay đổi địa chỉ này tùy thuộc vào cấu hình của bạn
/* vì chỉ dùng 4 bit cao làm 4 bit truyền dữ liệu nên phải truyền mỗi byte thành 2 lần */
/* 3 bit thấp là các chân En, RW, RS nên cũng cần phải đặt luôn trong mỗi lần truyền */

// hàm truyền lệnh, rs=0 để truyền lệnh
void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd & 0xf0);
	data_l = ((cmd << 4) & 0xf0);
	data_t[0] = data_u | 0x0C;  // en=1, rs=0
	data_t[1] = data_u | 0x08;  // en=0, rs=0
	data_t[2] = data_l | 0x0C;  // en=1, rs=0
	data_t[3] = data_l | 0x08;  // en=0, rs=0
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

// hàm truyền dữ liệu, 1 byte, rs=1 để truyền dữ liệu
void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data & 0xf0);
	data_l = ((data << 4) & 0xf0);
	data_t[0] = data_u | 0x0D;  // en=1, rs=1
	data_t[1] = data_u | 0x09;  // en=0, rs=1
	data_t[2] = data_l | 0x0D;  // en=1, rs=1
	data_t[3] = data_l | 0x09;  // en=0, rs=1
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, (uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd(0x80);
	for (int i = 0; i < 70; i++)
	{
		lcd_send_data(' ');
	}
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd(col);
}

void lcd_init (void)
{
	// khởi tạo 4 bit
	HAL_Delay(50);  // đợi >40ms
	lcd_send_cmd(0x30);
	HAL_Delay(5);  // đợi >4.1ms
	lcd_send_cmd(0x30);
	HAL_Delay(1);  // đợi >100us
	lcd_send_cmd(0x30);
	HAL_Delay(10);
	lcd_send_cmd(0x20);  // chế độ 4 bit
	HAL_Delay(10);

  // khởi tạo hiển thị
	lcd_send_cmd(0x28); // Cài đặt chức năng --> DL=0 (chế độ 4 bit), N=1 (hiển thị 2 dòng), F=0 (ký tự 5x8)
	HAL_Delay(1);
	lcd_send_cmd(0x08); // Điều khiển bật/tắt hiển thị --> D=0, C=0, B=0 ---> tắt hiển thị
	HAL_Delay(1);
	lcd_send_cmd(0x01);  // xóa màn hình
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd(0x06); // Cài đặt chế độ nhập --> I/D=1 (tăng con trỏ) & S=0 (không dịch chuyển)
	HAL_Delay(1);
	lcd_send_cmd(0x0C); // Điều khiển bật/tắt hiển thị --> D=1, C và B=0. (Con trỏ và nhấp nháy, hai bit cuối)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data(*str++);
}
