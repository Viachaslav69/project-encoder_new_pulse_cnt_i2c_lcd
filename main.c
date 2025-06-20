#include <string.h>
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_intr_alloc.h"
#include <stdio.h>
#include "driver/i2c.h"
#include "i2c_lcd.h"
#include <math.h>

// Конфигурация пинов
#define ENCODER_A_GPIO     18
#define ENCODER_B_GPIO     19
#define ENCODER_Z_GPIO     17
#define UART_PORT_NUM      UART_NUM_0
#define PULSES_PER_REV     1       // 1 импульс Z на 1 оборот

// Глобальные переменные
static volatile bool counting_enabled = false;
static volatile uint32_t z_pulse_total = 0;
static volatile uint32_t last_z_time = 0;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

// Дескрипторы для нового API
static pcnt_unit_handle_t pcnt_unit = NULL;
static pcnt_channel_handle_t pcnt_chan_a = NULL;
static pcnt_channel_handle_t pcnt_chan_b = NULL;

// Обработчик прерывания Z-канала
static void IRAM_ATTR z_gpio_isr_handler(void* arg) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    portENTER_CRITICAL_ISR(&spinlock);
    
    if (counting_enabled) {
        pcnt_unit_stop(pcnt_unit);
        counting_enabled = false;
    } else {
        pcnt_unit_clear_count(pcnt_unit);
        pcnt_unit_start(pcnt_unit);
        counting_enabled = true;
    }
    
    z_pulse_total++;
    last_z_time = now;
    portEXIT_CRITICAL_ISR(&spinlock);
}

void init_z_gpio_interrupt() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ENCODER_Z_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(ENCODER_Z_GPIO, z_gpio_isr_handler, NULL);
}

void init_pcnt_for_quadrature() {
    // Конфигурация модуля PCNT
    pcnt_unit_config_t unit_config = {
        .high_limit = INT16_MAX,
        .low_limit = INT16_MIN,
        .flags.accum_count = true,  // Для 32-битного счетчика
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // Конфигурация канала A
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_A_GPIO,
        .level_gpio_num = ENCODER_B_GPIO,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    // Конфигурация канала B
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_B_GPIO,
        .level_gpio_num = ENCODER_A_GPIO,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    // Настройка действий для каналов
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, 
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, 
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, 
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, 
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, 
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, 
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, 
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, 
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Включение модуля
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
}

void app_main() {
    //init_uart();
    init_pcnt_for_quadrature();
    init_z_gpio_interrupt();

    lcd_init();
    lcd_clear();
    //lcd_backlight(true);

    // Инициализация дисплея
    lcd_put_cursor(0, 0); lcd_send_string("State:");
    lcd_put_cursor(1, 0); lcd_send_string("Count:");
    lcd_put_cursor(2, 0); lcd_send_string("Z-mark:");
    lcd_put_cursor(3, 0); lcd_send_string("RPM:");

    char lcd_buffer[21];
    uint32_t last_update = 0;
    uint32_t last_z_count = 0;
    uint32_t last_z_timestamp = 0;
    float rpm = 0;
    const uint32_t update_interval_ms = 200;

    // Для фильтрации RPM
    #define RPM_SAMPLES 5
    float rpm_samples[RPM_SAMPLES] = {0};
    uint8_t sample_index = 0;

    while (1) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Чтение значений с защитой
        portENTER_CRITICAL(&spinlock);
        int pcnt_value;
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pcnt_value));
        uint32_t current_z = z_pulse_total;
        uint32_t z_time = last_z_time;
        portEXIT_CRITICAL(&spinlock);

        // Обновление дисплея
        if (now - last_update >= update_interval_ms) {
            // Расчет RPM
            if (current_z > last_z_count && z_time > last_z_timestamp) {
                float time_between_pulses = (float)(z_time - last_z_timestamp) / (current_z - last_z_count);
                rpm = 60000.0f / (time_between_pulses * PULSES_PER_REV);
                
                // Фильтрация
                rpm_samples[sample_index++] = rpm;
                if (sample_index >= RPM_SAMPLES) sample_index = 0;
                rpm = 0;
                for (int i = 0; i < RPM_SAMPLES; i++) rpm += rpm_samples[i];
                rpm /= RPM_SAMPLES;
            } else {
                rpm = 0;
            }

            last_z_count = current_z;
            last_z_timestamp = z_time;

            // Обновление LCD
            // Строка 0: Состояние
            lcd_put_cursor(0, 7);
            snprintf(lcd_buffer, sizeof(lcd_buffer), "%-6s", counting_enabled ? "RUN" : "STOP");
            lcd_send_string(lcd_buffer);

            // Строка 1: Счетчик AB
            lcd_put_cursor(1, 7);
            snprintf(lcd_buffer, sizeof(lcd_buffer), "%6d", pcnt_value);
            lcd_send_string(lcd_buffer);

            // Строка 2: Импульсы Z
            lcd_put_cursor(2, 8);
            snprintf(lcd_buffer, sizeof(lcd_buffer), "%6lu", current_z);
            lcd_send_string(lcd_buffer);

            // Строка 3: RPM
            lcd_put_cursor(3, 5);
            snprintf(lcd_buffer, sizeof(lcd_buffer), "%8.1f", rpm);
            lcd_send_string(lcd_buffer);

            last_update = now;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}