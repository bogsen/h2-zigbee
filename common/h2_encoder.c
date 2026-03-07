#include "h2_encoder.h"
#include "em_gpio.h"

// Encoder pins: PB02 = A, PB05 = B
#define ENC_A_PORT  gpioPortB
#define ENC_A_PIN   2
#define ENC_B_PORT  gpioPortB
#define ENC_B_PIN   5

// Button FSM timing (in ms, driven by 1ms tick)
#define LONG_PRESS_MS     5000
#define CLICK_TIMEOUT_MS  300
#define PRESS_TAP_MAX_MS  300   // max hold time to count as tap (not grip adjust)

// Encoder: 30 detents per full revolution
// Sensitivity maps to how many detents = full level range (0-254)
// 720: ~24 detents for full range (low sensitivity, ~10.6 units/detent)
// 360: ~12 detents for full range (medium, ~21.2 units/detent)
// 180: ~6 detents for full range (high, ~42.3 units/detent)
// We report raw steps and let app.c scale to level units.

typedef enum {
  FSM_IDLE,
  FSM_PRESSED,
  FSM_CLICK_WAIT,
  FSM_COLOR_HOLD,
  FSM_HELD,
} fsm_state_t;

static h2_encoder_callback_t cb;
static uint16_t sensitivity = 360;

// Quadrature state
static uint8_t last_a, last_b;
static volatile int8_t step_accum;  // accumulated steps since last FSM tick

// Button FSM state
static fsm_state_t fsm_state;
static uint16_t fsm_timer;
static bool had_rotation;
static volatile bool btn_pressed;
static volatile bool btn_changed;

// FSM tick divider (run FSM every 10ms, quadrature every 1ms)
static uint8_t fsm_divider;

void h2_encoder_init(h2_encoder_callback_t callback)
{
  cb = callback;
  fsm_state = FSM_IDLE;
  fsm_timer = 0;
  had_rotation = false;
  btn_pressed = false;
  btn_changed = false;
  step_accum = 0;
  fsm_divider = 0;

  // Setup encoder GPIO as input with pull-up
  GPIO_PinModeSet(ENC_A_PORT, ENC_A_PIN, gpioModeInputPull, 1);
  GPIO_PinModeSet(ENC_B_PORT, ENC_B_PIN, gpioModeInputPull, 1);

  // Read initial state
  last_a = GPIO_PinInGet(ENC_A_PORT, ENC_A_PIN);
  last_b = GPIO_PinInGet(ENC_B_PORT, ENC_B_PIN);
}

void h2_encoder_on_button(bool pressed)
{
  btn_pressed = pressed;
  btn_changed = true;
}

void h2_encoder_set_sensitivity(uint16_t value)
{
  sensitivity = value;
}

// Quadrature decode — called at 1ms
static void quadrature_tick(void)
{
  uint8_t a = GPIO_PinInGet(ENC_A_PORT, ENC_A_PIN);
  uint8_t b = GPIO_PinInGet(ENC_B_PORT, ENC_B_PIN);

  if (a == last_a && b == last_b) return;

  uint8_t old_state = (last_a << 1) | last_b;
  uint8_t new_state = (a << 1) | b;

  int8_t dir = 0;
  switch ((old_state << 2) | new_state) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: dir = +1; break;
    case 0b0010: case 0b1011: case 0b1101: case 0b0100: dir = -1; break;
  }

  step_accum += dir;
  last_a = a;
  last_b = b;
}

// Button FSM — called at 10ms
static void fsm_tick(void)
{
  // Grab accumulated rotation steps
  int8_t delta = step_accum;
  step_accum = 0;

  // Check for button state change
  bool changed = btn_changed;
  bool pressed = btn_pressed;
  if (changed) btn_changed = false;

  // Determine rotation mode from FSM state
  h2_encoder_mode_t mode = (fsm_state == FSM_COLOR_HOLD)
                            ? H2_ENC_MODE_COLOR_TEMP
                            : H2_ENC_MODE_BRIGHTNESS;

  switch (fsm_state) {
    case FSM_IDLE:
      if (delta != 0 && cb) {
        cb(H2_ENC_ACTION_ROTATE, delta, H2_ENC_MODE_BRIGHTNESS);
      }
      if (changed && pressed) {
        fsm_state = FSM_PRESSED;
        fsm_timer = 0;
        had_rotation = false;
      }
      break;

    case FSM_PRESSED:
      fsm_timer += 10;
      if (delta != 0) {
        had_rotation = true;
        if (cb) cb(H2_ENC_ACTION_ROTATE, delta, H2_ENC_MODE_BRIGHTNESS);
      }
      if (!had_rotation && fsm_timer >= LONG_PRESS_MS) {
        if (cb) cb(H2_ENC_ACTION_LONG_HOLD, 0, H2_ENC_MODE_BRIGHTNESS);
        fsm_state = FSM_HELD;
      } else if (changed && !pressed) {
        if (had_rotation) {
          // Was grip adjustment — suppress toggle
          fsm_state = FSM_IDLE;
        } else if (fsm_timer < PRESS_TAP_MAX_MS) {
          // Quick tap — wait for possible double click
          fsm_state = FSM_CLICK_WAIT;
          fsm_timer = 0;
        } else {
          // Held too long without rotation — no action
          fsm_state = FSM_IDLE;
        }
      }
      break;

    case FSM_CLICK_WAIT:
      fsm_timer += 10;
      if (delta != 0 && cb) {
        // Rotation during wait — fire toggle first, then rotation
        cb(H2_ENC_ACTION_TOGGLE, 0, H2_ENC_MODE_BRIGHTNESS);
        cb(H2_ENC_ACTION_ROTATE, delta, H2_ENC_MODE_BRIGHTNESS);
        fsm_state = FSM_IDLE;
      } else if (changed && pressed) {
        // Second press within timeout — enter color temp mode
        fsm_state = FSM_COLOR_HOLD;
      } else if (fsm_timer >= CLICK_TIMEOUT_MS) {
        // Timeout — single click
        if (cb) cb(H2_ENC_ACTION_TOGGLE, 0, H2_ENC_MODE_BRIGHTNESS);
        fsm_state = FSM_IDLE;
      }
      break;

    case FSM_COLOR_HOLD:
      if (delta != 0 && cb) {
        cb(H2_ENC_ACTION_ROTATE, delta, H2_ENC_MODE_COLOR_TEMP);
      }
      if (changed && !pressed) {
        if (cb) cb(H2_ENC_ACTION_DOUBLE, 0, H2_ENC_MODE_COLOR_TEMP);
        fsm_state = FSM_IDLE;
      }
      break;

    case FSM_HELD:
      if (changed && !pressed) {
        if (cb) cb(H2_ENC_ACTION_RELEASE, 0, H2_ENC_MODE_BRIGHTNESS);
        fsm_state = FSM_IDLE;
      }
      break;
  }
}

void h2_encoder_tick(void)
{
  quadrature_tick();

  if (++fsm_divider >= 10) {
    fsm_divider = 0;
    fsm_tick();
  }
}
