#ifndef H2_LED_CONFIG_H
#define H2_LED_CONFIG_H

#include "em_gpio.h"
#include "em_timer.h"

// LED up (top button) — TIMER0
#define H2_LED_UP_TIMER        TIMER0
#define H2_LED_UP_TIMER_INDEX  0
#define H2_LED_UP_R_CC         0   // CC0
#define H2_LED_UP_B_CC         1   // CC1
#define H2_LED_UP_R_PORT       gpioPortD
#define H2_LED_UP_R_PIN        4
#define H2_LED_UP_B_PORT       gpioPortD
#define H2_LED_UP_B_PIN        5

// LED down (bottom button) — TIMER1
#define H2_LED_DN_TIMER        TIMER1
#define H2_LED_DN_TIMER_INDEX  1
#define H2_LED_DN_R_CC         0   // CC0
#define H2_LED_DN_B_CC         1   // CC1
#define H2_LED_DN_R_PORT       gpioPortD
#define H2_LED_DN_R_PIN        2
#define H2_LED_DN_B_PORT       gpioPortD
#define H2_LED_DN_B_PIN        3

// Gang 1 LED up (LED3) — split across two timers
// Red on TIMER1 CC2 (PC09), Blue on TIMER0 CC2 (PC08)
#define H2_LED_G1_UP_R_TIMER       TIMER1
#define H2_LED_G1_UP_R_TIMER_INDEX 1
#define H2_LED_G1_UP_R_CC          2
#define H2_LED_G1_UP_R_PORT        gpioPortC
#define H2_LED_G1_UP_R_PIN         9
#define H2_LED_G1_UP_B_TIMER       TIMER0
#define H2_LED_G1_UP_B_TIMER_INDEX 0
#define H2_LED_G1_UP_B_CC          2
#define H2_LED_G1_UP_B_PORT        gpioPortC
#define H2_LED_G1_UP_B_PIN         8

// Gang 1 LED down (LED4) — TIMER3
#define H2_LED_G1_DN_TIMER         TIMER3
#define H2_LED_G1_DN_TIMER_INDEX   3
#define H2_LED_G1_DN_R_CC          1
#define H2_LED_G1_DN_R_PORT        gpioPortC
#define H2_LED_G1_DN_R_PIN         7
#define H2_LED_G1_DN_B_CC          0
#define H2_LED_G1_DN_B_PORT        gpioPortC
#define H2_LED_G1_DN_B_PIN         6

// PWM resolution (8-bit = 255 top value)
#define H2_LED_PWM_TOP         255

#endif // H2_LED_CONFIG_H
