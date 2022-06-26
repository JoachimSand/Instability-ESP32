#include "radar.h"

#include "driver/adc.h"
#include "hal/adc_types.h"

void init_radar_adc()
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); // ADC1_CHANNEL_4 is GPIO32
}

int read_radar_raw()
{
    return adc1_get_raw(ADC1_CHANNEL_4);
}
