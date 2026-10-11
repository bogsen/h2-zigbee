#ifndef H2_POWER_H
#define H2_POWER_H

#include <stdint.h>
#include <stdbool.h>

// Load energy total, register RX callback and start polling (also while offline).
void h2_power_init(void);

// Returns gang count detected from relay module (0 if not yet detected)
uint8_t h2_power_get_detected_gangs(void);

// Fresh wall-module temperature in degrees Celsius, with zero calibration offset.
bool h2_power_get_temperature(float *value);

// Schedule the next measurement poll (also called on NETWORK_UP).
void h2_power_start(void);

#endif // H2_POWER_H
