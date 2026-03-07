#ifndef H2_LED_CONFIG_H
#define H2_LED_CONFIG_H

#include "em_gpio.h"
#include "em_timer.h"

// LED up — TIMER0: CC0 = red (PC09), CC1 = blue (PC08)
#define H2_LED_UP_TIMER        TIMER0
#define H2_LED_UP_TIMER_INDEX  0
#define H2_LED_UP_R_CC         0
#define H2_LED_UP_R_PORT       gpioPortC
#define H2_LED_UP_R_PIN        9
#define H2_LED_UP_B_CC         1
#define H2_LED_UP_B_PORT       gpioPortC
#define H2_LED_UP_B_PIN        8

// LED down — not present on dimmer
#define H2_LED_HAS_DN          0

// PWM resolution (8-bit = 255 top value)
#define H2_LED_PWM_TOP         255

#endif // H2_LED_CONFIG_H
