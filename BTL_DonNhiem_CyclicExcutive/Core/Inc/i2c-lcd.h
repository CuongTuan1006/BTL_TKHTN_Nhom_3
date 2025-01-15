#include "stm32f1xx_hal.h"

void lcd_init (void);   // khoi tao lcd

void lcd_send_cmd (char cmd);  // gui lenh

void lcd_send_data (char data);  // gui du lieu

void lcd_send_string (char *str);  // gui 1 chuoi du lieu

void lcd_put_cur(int row, int col);  // chon vi tri de bat dau ghi

void lcd_clear (void);  // xoa man hinh
