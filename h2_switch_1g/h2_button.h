#ifndef H2_BUTTON_H
#define H2_BUTTON_H

#include <stdint.h>
#include <stdbool.h>

// Button action types reported to the application
typedef enum {
  H2_BTN_ACTION_RELEASE = 0,
  H2_BTN_ACTION_SINGLE  = 1,
  H2_BTN_ACTION_DOUBLE  = 2,
  H2_BTN_ACTION_LONG    = 3,
} h2_button_action_t;

// Which button of a pair
typedef enum {
  H2_BTN_SIDE_UP   = 0,
  H2_BTN_SIDE_DOWN = 1,
} h2_button_side_t;

// Callback type for button events
// gang: 0-indexed gang number
// side: up or down
// action: what happened
typedef void (*h2_button_callback_t)(uint8_t gang,
                                     h2_button_side_t side,
                                     h2_button_action_t action);

// Initialize button state machines. Call once at startup.
void h2_button_init(h2_button_callback_t callback);

// Call from sl_button_on_change ISR context.
// button_index: the simple_button instance index (0 = btn1, etc.)
// pressed: true if button is now pressed, false if released
void h2_button_on_change(uint8_t button_index, bool pressed);

// Must be called periodically (e.g., from a 10ms tick event) to
// drive the state machine timers.
void h2_button_tick(void);

// Configuration
void h2_button_set_long_press_ms(uint16_t ms);
void h2_button_set_double_press_ms(uint16_t ms);

#endif // H2_BUTTON_H