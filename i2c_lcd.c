#include "i2c_lcd.h"
#include "freertos/task.h"
#include <string.h>
//#include "esp_rom/esp_rom.h"

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t lcd_write_byte(uint8_t data)
{
    return i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, &data, 1, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static void lcd_send_nibble(uint8_t nibble, uint8_t mode)
{
    uint8_t data = nibble | LCD_BACKLIGHT | mode;
    lcd_write_byte(data | LCD_ENABLE);
    esp_rom_delay_us(1);
    lcd_write_byte(data & ~LCD_ENABLE);
    esp_rom_delay_us(100);
}

static void lcd_send_byte(uint8_t byte, uint8_t mode)
{
    lcd_send_nibble(byte & 0xF0, mode);
    lcd_send_nibble((byte << 4) & 0xF0, mode);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_send_command(uint8_t cmd)
{
    lcd_send_byte(cmd, LCD_COMMAND);
}

void lcd_send_data(uint8_t data)
{
    lcd_send_byte(data, LCD_DATA);
}

void lcd_clear(void)
{
    lcd_send_command(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_put_cursor(uint8_t row, uint8_t col)
{
    const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_send_command(0x80 | (col + row_offsets[row]));
}

void lcd_send_string(const char *str)
{
    while (*str)
    {
        lcd_send_data((uint8_t)(*str));
        str++;
    }
}

void lcd_init(void)
{
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(50));

    lcd_send_nibble(0x30, LCD_COMMAND);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_nibble(0x30, LCD_COMMAND);
    esp_rom_delay_us(150);
    lcd_send_nibble(0x30, LCD_COMMAND);
    lcd_send_nibble(0x20, LCD_COMMAND); // 4-bit mode

    lcd_send_command(0x28); // 4-bit, 2 line, 5x8 dots
    lcd_send_command(0x0C); // Display on, cursor off
    lcd_send_command(0x06); // Entry mode
    lcd_clear();
}
