#include <stdbool.h>
#include "real_inputs.h"
#include "UV_meter.h"
#include "driver/i2c_master.h"
#include "esp_random.h"
#ifndef PCM
#define PCM


#define PCM8574T_SDA 7
#define PCM8574T_SCL 8

#define PCM8574T_FREQ_HZ  100000
#define PCM8574T_I2C_NUMBER 1

#define NUM_BANDS_2 16
#define MAX_HEIGHT 8



#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

#define LCD_ADDR 0x27

esp_err_t i2c_master_init(void);

#define LCD_RS 0x01
#define LCD_RW 0x02
#define LCD_E  0x04
#define LCD_BL 0x08


typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t p_write;
    bool volume_controls_bar_heights;
    uint32_t peak_volume[NUM_BANDS_2];
    uint32_t old_volume[NUM_BANDS_2];
    bool stale_data;
    bool mute_on;
} lcd1602_t;

typedef struct{
    lcd1602_t *lcd;
    adc_control_t *volume_nob;
    button_data_t *button_state;
    uv_meter_t *uvm;
}all_data_t;



// Basic functions
esp_err_t lcd1602_init(lcd1602_t *lcd);
void lcd1602_clear(lcd1602_t *lcd);
void lcd1602_home(lcd1602_t *lcd);
void lcd1602_set_cursor(lcd1602_t *lcd, uint8_t col, uint8_t row);
void lcd1602_print(lcd1602_t *lcd, const char *str);
void lcd1602_backlight(lcd1602_t *lcd, bool on);
void lcd1602_display(lcd1602_t *lcd, bool on);

void lcd1602_create_char(lcd1602_t *lcd, uint8_t location, uint8_t charmap[8]);
void lcd1602_load_bars(lcd1602_t *lcd);
void lcd1602_write_char(lcd1602_t *lcd, uint8_t location);
void lcd1602_create_show(lcd1602_t *lcd );
void lcd1602_write_mute(lcd1602_t *lcd);

void v_lcd1602_update_task(void *pvParameters);

#endif