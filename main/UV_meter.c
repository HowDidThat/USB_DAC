#include "UV_meter.h"
#include "driver/ledc.h"
#include "math.h"


void  led_strip_init(uv_meter_t *uv_meter){
    led_strip_config_t strip_config = {
        .strip_gpio_num = 9, 
        .max_leds = 4,      
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = {
            .invert_out = false, 
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        
        .resolution_hz = LED_STRIP_RMT_RES_HZ, 
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS, 
        .flags = {
            .with_dma = LED_STRIP_USE_DMA,  
        }
    };
    uv_meter->off = 0;
    led_strip_new_rmt_device(&strip_config, &rmt_config, &uv_meter->led_strip);
}


void uv_meter_init(uv_meter_t *uv_meter){
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num  = UV_CHANNEL,
        .duty_resolution = UV_PWM_RESOLUTION,
        .freq_hz = UV_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t ledc_channel = {
        .gpio_num = UV_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config (&ledc_channel);

    uv_meter->channel = UV_CHANNEL;
    uv_meter->current_duty = 0;
}


void vUV_meter_read_task(void *pvParameters){
    uv_meter_t *uvm = (uv_meter_t *)pvParameters;
    
    uint8_t duty;
    while(1)
    {
        if (uvm->new_duty > uvm->current_duty)
        {
            duty = min(255, uvm->current_duty + RAMP_SPEED);
            uvm->off = 0;
        }
        else
        {
            duty = (uint8_t)max(0.0f,(float)uvm->current_duty - (float)DROP_SPEED);
            uvm->off = min(100,uvm->off + 1);

        }
        uvm->current_duty = duty;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, uvm->channel, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, uvm->channel);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

void uv_meter_show_voltage(uv_meter_t *uvm, uint8_t duty){

    ledc_set_duty(LEDC_LOW_SPEED_MODE, uvm->channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, uvm->channel);
}


void vLED_update_task(void *pvParameters)
{
    uv_meter_t *uvm = (uv_meter_t *)pvParameters;

    while (1)
    {
        uint8_t duty = uvm->current_duty;   // 0–255
        float level = duty / 255.0f;
        if (uvm->off == 100){
            for (int i=0;i < 4;i++)
            {
                led_strip_clear(uvm->led_strip);
            }
        }
        else{
            for (int i = 0; i < 4; i++)
                {
                
                    int rev_i = 3 - i;
                    float weight = (float)i / 9.0f;

                    float intensity = 0.3f + level * 2.0f * weight;

                    uint8_t r = (uint8_t)(255.0f * intensity); 
                    uint8_t g = (uint8_t)(255.0f * (1.0f - intensity));

                    led_strip_set_pixel(uvm->led_strip, rev_i, r, g, 0);
                }
        }
        led_strip_refresh(uvm->led_strip);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
