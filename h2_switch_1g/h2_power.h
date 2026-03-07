#ifndef H2_POWER_H
#define H2_POWER_H

#include <stdint.h>

// Initialize measurement module (register RX callback, init polling event)
void h2_power_init(void);

// Start measurement polling (call on NETWORK_UP)
void h2_power_start(void);

#endif // H2_POWER_H
