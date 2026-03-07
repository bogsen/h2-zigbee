#ifndef H2_ENCODER_H
#define H2_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  H2_ENC_MODE_BRIGHTNESS,
  H2_ENC_MODE_COLOR_TEMP,
} h2_encoder_mode_t;

typedef enum {
  H2_ENC_ACTION_ROTATE,       // delta + mode
  H2_ENC_ACTION_TOGGLE,       // single click
  H2_ENC_ACTION_LONG_HOLD,    // 5s hold (commissioning)
  H2_ENC_ACTION_RELEASE,      // release from long hold
  H2_ENC_ACTION_DOUBLE,       // second press released (color temp session end)
} h2_encoder_action_t;

typedef void (*h2_encoder_callback_t)(h2_encoder_action_t action,
                                       int8_t delta,
                                       h2_encoder_mode_t mode);

// Initialize encoder. GPIO setup for PB02 (A) + PB05 (B) + button via sl_button.
void h2_encoder_init(h2_encoder_callback_t callback);

// Call at 1ms interval for quadrature decoding + button FSM.
void h2_encoder_tick(void);

// Called from sl_button_on_change ISR for the encoder push button.
void h2_encoder_on_button(bool pressed);

// Set rotation sensitivity (720=low, 360=medium, 180=high).
void h2_encoder_set_sensitivity(uint16_t value);

#endif // H2_ENCODER_H
