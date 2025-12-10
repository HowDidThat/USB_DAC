#include "PCM8574T.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include <string.h>


static const char *TAG = "LCD1602";

// PCF8574 bit mapping:
// P7 P6 P5 P4 P3 P2 P1 P0
// D7 D6 D5 D4 BL E  RW RS

uint8_t pcm_bar_0[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111}; // 1 row
uint8_t pcm_bar_1[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111}; // 2 rows
uint8_t pcm_bar_2[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111}; // 3 rows
uint8_t pcm_bar_3[8] = {0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111}; // 4 rows
uint8_t pcm_bar_4[8] = {0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111}; // 5 rows
uint8_t pcm_bar_5[8] = {0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111}; // 6 rows
uint8_t pcm_bar_6[8] = {0b00000, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111}; // 7 rows
// This is a certain character in the display memmory, but for convinience sake
// It is declared as a custom character
uint8_t pcm_bar_7[8] = {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111}; // 8 rows (full)


static esp_err_t i2c_write_byte(lcd1602_t *lcd, uint8_t data) {
    esp_err_t ret = i2c_master_transmit(lcd->dev, &data, 1, pdMS_TO_TICKS(100));
    return ret;
}

static void lcd_pulse_enable(lcd1602_t *lcd) {
    lcd->p_write |= LCD_E;
    i2c_write_byte(lcd, lcd->p_write);
    vTaskDelay(pdMS_TO_TICKS(1));
    lcd->p_write &= ~LCD_E;
    i2c_write_byte(lcd, lcd->p_write);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static void lcd_write_nibble(lcd1602_t *lcd, uint8_t nibble) {

    lcd->p_write = (lcd->p_write & 0x0F) | (nibble << 4);
    i2c_write_byte(lcd, lcd->p_write);
    lcd_pulse_enable(lcd);
}

static void lcd_write_cmd(lcd1602_t *lcd, uint8_t cmd) {
    lcd->p_write &= ~LCD_RS; 
    lcd->p_write &= ~LCD_RW;  
    i2c_write_byte(lcd, lcd->p_write);
    
    lcd_write_nibble(lcd, cmd >> 4);   
    lcd_write_nibble(lcd, cmd & 0x0F); 
    
 
    if (cmd <= 0x03) {
        vTaskDelay(pdMS_TO_TICKS(1));
    } else {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

static void lcd_write_data(lcd1602_t *lcd, uint8_t data) {
    lcd->p_write |= LCD_RS;  
    lcd->p_write &= ~LCD_RW;  
    i2c_write_byte(lcd, lcd->p_write);
    
    lcd_write_nibble(lcd, data >> 4);    
    lcd_write_nibble(lcd, data & 0x0F); 
    
    //vTaskDelay(pdMS_TO_TICKS(1));
}

esp_err_t lcd1602_init(lcd1602_t *lcd) {
    lcd->mute_on = false;
    memset(lcd, 0, sizeof(lcd1602_t));

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = PCM8574T_I2C_NUMBER,
        .sda_io_num = PCM8574T_SDA,
        .scl_io_num = PCM8574T_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &lcd->bus);
    if (ret != ESP_OK) {
        return ret;
    }

    i2c_device_config_t dev_cfg = {
        .device_address = LCD_ADDR,
        .scl_speed_hz = PCM8574T_FREQ_HZ
    };
    
    ret = i2c_master_bus_add_device(lcd->bus, &dev_cfg, &lcd->dev);
    if (ret != ESP_OK) {
        return ret;
    }
    
   
    vTaskDelay(pdMS_TO_TICKS(50));
    
    lcd->p_write = 0x00;
    i2c_write_byte(lcd, lcd->p_write);
    vTaskDelay(pdMS_TO_TICKS(10));

    lcd_write_nibble(lcd, 0x03);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write_nibble(lcd, 0x03);
    vTaskDelay(pdMS_TO_TICKS(1));
 
    lcd_write_nibble(lcd, 0x03);
    vTaskDelay(pdMS_TO_TICKS(1));
    

    lcd_write_nibble(lcd, 0x02);
    vTaskDelay(pdMS_TO_TICKS(1));

    lcd_write_cmd(lcd, 0x28); 
    lcd_write_cmd(lcd, 0x08);
    lcd_write_cmd(lcd, 0x01);  
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_write_cmd(lcd, 0x06); 
    lcd_write_cmd(lcd, 0x0C); 

    lcd1602_backlight(lcd, true); 
    return ESP_OK;
}

void lcd1602_clear(lcd1602_t *lcd) {
    lcd_write_cmd(lcd, 0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd1602_home(lcd1602_t *lcd) {
    lcd_write_cmd(lcd, 0x02);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd1602_set_cursor(lcd1602_t *lcd, uint8_t col, uint8_t row) {
    const uint8_t row_offsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    if (col > 15) col = 15;
    lcd_write_cmd(lcd, 0x80 | (col + row_offsets[row]));
}

void lcd1602_print(lcd1602_t *lcd, const char *str) {
    while (*str) {
        lcd_write_data(lcd, *str++);
    }
}

void lcd1602_backlight(lcd1602_t *lcd, bool on) {
    if (on) {
        lcd->p_write |= LCD_BL;
    } else {
        lcd->p_write &= ~LCD_BL;
    }
    i2c_write_byte(lcd, lcd->p_write);
}

void lcd1602_display(lcd1602_t *lcd, bool on) {
    if (on) {
        lcd_write_cmd(lcd, 0x0C);  
    } else {
        lcd_write_cmd(lcd, 0x08); 
    }
}

void lcd1602_create_char(lcd1602_t *lcd, uint8_t location, uint8_t charmap[8]) {
    
    location &= 0x07;
    
    
    lcd_write_cmd(lcd, 0x40 | (location << 3));
    
   
    for (int i = 0; i < 8; i++) {
        lcd_write_data(lcd, charmap[i]);
    }
    
   
    lcd_write_cmd(lcd, 0x80);
}

void lcd1602_write_char(lcd1602_t *lcd, uint8_t location) {

    lcd_write_data(lcd, location & 0x07);
}

void lcd1602_load_bars(lcd1602_t *lcd){
    lcd1602_create_char(lcd, 0, pcm_bar_0);
    lcd1602_create_char(lcd, 1, pcm_bar_1);
    lcd1602_create_char(lcd, 2, pcm_bar_2);
    lcd1602_create_char(lcd, 3, pcm_bar_3);
    lcd1602_create_char(lcd, 4, pcm_bar_4);
    lcd1602_create_char(lcd, 5, pcm_bar_5);
    lcd1602_create_char(lcd, 6, pcm_bar_6);
    lcd1602_create_char(lcd, 7, pcm_bar_7);
}


void lcd1602_create_show(lcd1602_t *lcd){

    for (uint8_t i=0; i < 16; i++)
    {
        if (esp_random() > UINT32_MAX / 2){
            lcd1602_set_cursor(lcd,i,1);
            lcd1602_write_char(lcd,esp_random() % 8);
        }
    }
}

void lcd1602_clear_show(lcd1602_t *lcd){
    lcd1602_set_cursor(lcd,0,1);
    lcd1602_print(lcd,"                ");
}
void lcd1602_test_data(lcd1602_t *lcd){
for (int i=0;i<8;i++)
    {
     lcd1602_set_cursor(lcd,i,1);
     lcd1602_write_char(lcd,i); 
    }
}


void lcd1602_change_volume(lcd1602_t *lcd, uint8_t volume){
    //8,9,10
    char full_string[4] = "   ";
    char s_volume[4];
    sprintf(s_volume, "%d", volume);
    if (volume < 10)
        strcpy(full_string+2,s_volume);
    else if (volume < 100)
        strcpy(full_string+1, s_volume);
    else 
        strcpy(full_string, s_volume);
    
    lcd1602_set_cursor(lcd,7,0);
    lcd1602_print(lcd, full_string);
}

void lcd1602_write_info(lcd1602_t *lcd){
    lcd1602_set_cursor(lcd,0,0);
    lcd1602_print(lcd,"Volume:");
}


void lcd1602_write_mute(lcd1602_t *lcd){
    lcd1602_set_cursor(lcd, 12, 0);
    if (lcd->mute_on == false){
        lcd1602_print(lcd, "    ");
    }
    else{
        lcd1602_print(lcd, "MUTE");
    }
    lcd->mute_on = !lcd->mute_on;
};


void v_lcd1602_update_task(void *pvParameters)
{
    all_data_t *data = (all_data_t *)pvParameters;
    lcd1602_t *lcd = data->lcd;
    adc_control_t *volume = data->volume_nob;
    button_data_t *switch_data = data->button_state; 
    uv_meter_t *uvm = data->uvm; 
    lcd1602_write_info(lcd);
    lcd1602_write_mute(lcd);
    uint8_t cv;
    while (1){
        if (uvm->off == 100)
            {
                lcd1602_backlight(lcd,false);
                lcd1602_clear_show(lcd);                
            }
        else
            {
                lcd1602_backlight(lcd,true);
                if (lcd->mute_on == false)
                    lcd1602_clear_show(lcd);
                else
                    lcd1602_create_show(lcd);
            }
        
            cv = volume->current_volume;
        if(volume->old_volume != cv)
        {
            lcd1602_change_volume(lcd, cv);
            volume->old_volume = cv;
        }
        if (switch_data->button1_state != lcd->mute_on)
        {
            lcd1602_write_mute(lcd);
        }
        vTaskDelay(10);
    }
}