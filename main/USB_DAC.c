#include <stdio.h>
#include "real_inputs.h"
#include "UV_meter.h"
#include "PCM8574T.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "usb_device_uac.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"
#include "esp_dsp.h"
#include <math.h>

#define DAC_I2S_NUM 0
#define DAC_I2S_CHANNEL_MODE I2S_SLOT_MODE_STEREO
#define DAC_AUDIO_SAMPLE_RATE 48000
#define DAC_AUDIO_BIT_WIDTH I2S_DATA_BIT_WIDTH_32BIT
#define DAC_I2S_BCK_IO 10
#define DAC_I2S_WS_IO  11
#define DAC_I2S_DO_IO  12

#define FFT_SIZE       1024
#define NUM_BANDS      16
#define MAX_BAR_VALUE  8 

i2s_chan_handle_t dac_i2s_handle;
uv_meter_t uv_meter;
adc_control_t vc;
lcd1602_t lcd;
all_data_t data; 
button_data_t button_data;
led_strip_handle_t led_strip;
void dac_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(DAC_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &dac_i2s_handle, NULL));
    i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(DAC_AUDIO_SAMPLE_RATE),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(DAC_AUDIO_BIT_WIDTH, DAC_I2S_CHANNEL_MODE),
    .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = DAC_I2S_BCK_IO,
        .ws   = DAC_I2S_WS_IO,
        .dout = DAC_I2S_DO_IO,
        .din  = I2S_GPIO_UNUSED,
    },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(dac_i2s_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(dac_i2s_handle));
}

float volume_to_gain(float volume)
{
    float x = volume / 100.0f;
    float curve = 0.01f;
    return ( powf( 10.0f, x) - 1.0f )/( 10.0f - 1.0f + curve); 
}

static esp_err_t usb_uac_device_output_cb(uint8_t *buf, size_t len, void *arg)
{
    if (buf == NULL || len == 0 || len % 2 != 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    const size_t sample_count = len / sizeof(int16_t);
    const int16_t *usb_samples = (const int16_t *)buf;
    int32_t i2s_buf[sample_count];
    size_t bytes_written = 0;
    
    // Check mute state AFTER processing, not before
    bool should_mute = (button_data.button1_state == false || vc.current_volume == 0);
    
    float gain = should_mute ? 0.0f : volume_to_gain(vc.current_volume);
    
    // RMS calculation
    uint64_t sum_squares = 0;
    
    for (size_t i = 0; i < sample_count; i++) {
        int32_t sample = (int32_t)(usb_samples[i] * gain);
        sample = (sample > INT16_MAX) ? INT16_MAX : 
                 (sample < INT16_MIN) ? INT16_MIN : sample;
        i2s_buf[i] = sample << 16;
        
        // Accumulate squared values for RMS (use original sample for meter)
        int32_t normalized = usb_samples[i]; // Use original for accurate metering
        sum_squares += (uint64_t)(normalized * normalized);
    }
    
    // Calculate RMS for the meter (independent of mute state)
    float mean_square = (float)sum_squares / sample_count;
    float rms = sqrtf(mean_square);
    
    // Update meter even when muted, but you can choose to zero it
    uv_meter.new_duty = should_mute ? 0 : (uint16_t)(rms / 32768.0f * 255.0f);
    
    // ALWAYS write to I2S, even if muted (prevents buffer underrun)
    return i2s_channel_write(dac_i2s_handle,
                            (const uint8_t *)i2s_buf, 
                            sample_count * sizeof(int32_t), 
                            &bytes_written, 
                            0);
}
void usb_uac_device_init(void){
    uac_device_config_t config = {
        .output_cb = usb_uac_device_output_cb,
        .input_cb  = NULL,
        .set_mute_cb   = NULL,
        .set_volume_cb = NULL,
        .cb_ctx = NULL,
    };
    ESP_ERROR_CHECK(uac_device_init(&config));
}


void app_main(void){

    dac_i2s_init();
    uv_meter_init(&uv_meter);
    adc_init(&vc);
    buttons_init();
    lcd1602_init(&lcd);
    lcd1602_load_bars(&lcd);
    lcd1602_set_cursor(&lcd,0,0);
    led_strip_init(&uv_meter);
    data.volume_nob = &vc;
    data.lcd = &lcd;
    data.button_state = &button_data;
    data.uvm = &uv_meter;
    lcd1602_backlight(&lcd, true);
    usb_uac_device_init();
    xTaskCreatePinnedToCore(vADC_read_task, "ADC_read_task", 2048, &vc, 10, NULL,1);
    xTaskCreatePinnedToCore(vUV_meter_read_task, "UV_meter_read_task", 2048, &uv_meter, 10, NULL,1);
    xTaskCreatePinnedToCore(v_lcd1602_update_task, "LCD_update_task", 4096, &data, 10, NULL,1);
    xTaskCreatePinnedToCore(vBUTTONS_read_task, "Button update", 1024, &button_data, 10, NULL,1);
    xTaskCreatePinnedToCore(vLED_update_task, "LED update", 1024, &uv_meter, 10, NULL,1);
}