#include "real_inputs.h" 

void adc_init(adc_control_t *vc){
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
    };
    adc_oneshot_chan_cfg_t channel_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12
    };
    adc_oneshot_new_unit(&init_config, &vc->adc_handle);
    adc_oneshot_config_channel(vc->adc_handle, ADC_CHANNEL, &channel_config);

}


void vADC_read_task(void *pvParameters){
    adc_control_t *vc = (adc_control_t *)pvParameters;
    int raw = 0;
    int mapped = 0;
    int reads[10];
    while(1)
    {   raw = 0;
        for (int i=0;i<10;i++)
        {
            adc_oneshot_read(vc->adc_handle,ADC_CHANNEL,&reads[i]);
            raw += reads[i];
            vTaskDelay(pdMS_TO_TICKS(10));    
        } 
        raw = raw / 10;
        mapped = (raw * 100) / ADC_MAX_ADC_RAW;
        if (abs(mapped - vc->current_volume) >= ADC_MIN_CHANGE) {
            vc->current_volume = mapped;
        }
        if (vc->current_volume < 5){
            vc->current_volume = 0;
        }

        else if (vc->current_volume > 95){
            vc->current_volume = 100;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void buttons_init()
{
    gpio_config_t b_config = {
        .pin_bit_mask = (1ULL << BUTTON_1_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&b_config);

}

void vBUTTONS_read_task(void *pvParameters){
    button_data_t *button_data = (button_data_t *)pvParameters;
    bool b1;
    while (1){
        b1 = gpio_get_level(BUTTON_1_PIN);
        button_data->button1_state = b1; // i want the output to be inversed
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}