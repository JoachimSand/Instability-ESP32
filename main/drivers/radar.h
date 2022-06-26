#ifndef RADAR_H
#define RADAR_H

#define RADAR_ADC_PIN 32

void init_radar_adc();

int read_radar_raw();

#endif
