#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"

#ifndef MODULE21
#define MODULE21

#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_0
#define ADC_MIN_CHANGE 5
#define ADC_MAX_ADC_RAW 4096


#define BUTTON_1_PIN GPIO_NUM_2 
#define DEBOUNCE_TIME 50


typedef struct{
    adc_oneshot_unit_handle_t adc_handle;
    uint8_t current_volume;
    uint8_t old_volume;
    uint8_t max_volume;
    bool over_amplification;
    uint8_t lastRead[5];
} adc_control_t;

typedef struct{
    bool button1_state;
}button_data_t;



void adc_init(adc_control_t *vc);
void vADC_read_task(void *pvParameters);


void buttons_init();
void vBUTTONS_read_task(void *pvParameters);


#endif