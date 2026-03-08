#ifndef H2_LED_H
#define H2_LED_H

#include <stdint.h>
#include <stdbool.h>

// LED mode (matches 0xFCC0 attribute 0x0201)
typedef enum {
  H2_LED_MODE_FOLLOW_RELAY = 0,
  H2_LED_MODE_FOLLOW_LIGHT = 1,
  H2_LED_MODE_MANUAL       = 2,
  H2_LED_MODE_TIME_GRADIENT = 3,
} h2_led_mode_t;

// Which LED in a gang (each gang has up + down LEDs)
typedef enum {
  H2_LED_UP   = 0,
  H2_LED_DOWN = 1,
} h2_led_position_t;

// Initialize LED PWM hardware
void h2_led_init(void);

// Enable or disable the LED indicator for a gang.
// When disabled, both LEDs are turned off regardless of mode.
void h2_led_set_enabled(uint8_t gang, bool enabled);

// Invert the on/off color mapping for a gang.
// When inverted, on_color is shown when relay is off and vice versa.
void h2_led_set_inverted(uint8_t gang, bool inverted);

// Set LED mode for a gang
void h2_led_set_mode(uint8_t gang, h2_led_mode_t mode);

// Update LEDs for a gang based on relay state (for FOLLOW_RELAY mode)
// Call this whenever relay state changes.
void h2_led_update_relay_state(uint8_t gang, bool relay_on);

// Update LEDs for a gang based on bound light state (for FOLLOW_LIGHT mode)
void h2_led_update_light_state(uint8_t gang, bool light_on);

// Update LEDs with actual brightness level (for FOLLOW_LIGHT mode)
// level: 0 = off, 1-253 = dimmed (both LEDs on), 254 = fully on
void h2_led_update_light_level(uint8_t gang, uint8_t level);

// Set LED color directly (for MANUAL mode)
// color: 0 = full red, 100 = full blue
// brightness: 0-100
void h2_led_set_manual(uint8_t gang, uint8_t color, uint8_t brightness);

// Set a single LED (up or down) independently
void h2_led_set_one(uint8_t gang, h2_led_position_t pos, uint8_t color, uint8_t brightness);

// Set the colors used in FOLLOW_RELAY mode
void h2_led_set_on_color(uint8_t gang, uint8_t color);
void h2_led_set_off_color(uint8_t gang, uint8_t color);
void h2_led_set_brightness(uint8_t gang, uint8_t brightness);

// Query tracked light state (for FOLLOW_LIGHT mode decisions)
bool h2_led_get_light_state(uint8_t gang);

// Must be called periodically (~20ms) for smooth transitions
void h2_led_tick(void);

#endif // H2_LED_H
