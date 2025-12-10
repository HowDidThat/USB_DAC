
#include "driver/ledc.h"
#include "led_strip.h"
#include "esp_mac.h"
#include "driver/gpio.h"


#ifndef min
#define min(a,b) (((a)<(b))?(a):(b))
#endif

#ifndef max
#define max(a,b) (((a)>(b))?(a):(b))
#endif



#ifndef UV_METER
#define UV_METER

#define UV_PWM_PIN 5
#define UV_PWM_FREQ_HZ 1000
#define UV_PWM_RESOLUTION LEDC_TIMER_8_BIT
#define UV_MAX_VOLTAGE 3.2f
#define UV_CHANNEL LEDC_CHANNEL_0

#define LED_STRIP_USE_DMA 0
#define LED_STRIP_LED_COUNT 4
#define LED_STRIP_MEMORY_BLOCK_WORDS 64

#define LED_STRIP_GPIO_PIN 9
#define LED_STRIP_RMT_RES_HZ 10000000

#define DROP_SPEED 10
#define RAMP_SPEED 100



// this is a ledc_t used to show a 
typedef struct{
    //this is for the led_strip
    led_strip_handle_t led_strip;
    uint8_t _data;
    //this is for the uv_meter
    ledc_channel_t channel;
    uint8_t current_duty;
    uint8_t new_duty;
    uint8_t off;

} uv_meter_t;


void uv_meter_init(uv_meter_t *uv_meter);
void led_strip_init(uv_meter_t *uv_meter);
void vLED_update_task(void *pvParameters);

void vUV_meter_read_task(void *pvParameters);
void uv_meter_show_voltage(uv_meter_t *uvm, uint8_t duty);

#endif