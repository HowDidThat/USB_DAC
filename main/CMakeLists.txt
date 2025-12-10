idf_component_register(SRCS "PCM8574T.c" "UV_meter.c" "USB_DAC.c" "real_inputs.c"
                       PRIV_REQUIRES esp_driver_i2s esp_driver_i2c esp_driver_gpio driver esp_adc esp_driver_ledc
                       INCLUDE_DIRS "")
