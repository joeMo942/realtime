#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "main.h"

#define SLAVE_ADDRESS_LCD (0x27 << 1) // Change to 0x3F if 0x27 doesn't work

void lcd_init(I2C_HandleTypeDef *hi2c);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_clear(void);
void lcd_put_cur(int row, int col);

#endif
