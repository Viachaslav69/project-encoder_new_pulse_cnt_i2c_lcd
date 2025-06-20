#ifndef I2C_LCD_H
#define I2C_LCD_H

#include "driver/i2c.h"

#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

#define LCD_ADDR        0x27
#define LCD_BACKLIGHT   0x08
#define LCD_ENABLE      0x04
#define LCD_COMMAND     0x00
#define LCD_DATA        0x01

void lcd_init(void);
void lcd_clear(void);
void lcd_put_cursor(uint8_t row, uint8_t col);
void lcd_send_string(const char *str);

#endif // I2C_LCD_H
