#include "h2_button.h"

#ifndef H2_SWITCH_GANG_COUNT
#define H2_SWITCH_GANG_COUNT 2
#endif

#define NUM_BUTTONS (H2_SWITCH_GANG_COUNT * 2) // up + down per gang

extern uint8_t h2_active_gang_count;

#define DEFAULT_LONG_PRESS_MS   300
#define DEFAULT_DOUBLE_PRESS_MS 300
#define TICK_INTERVAL_MS        10

typedef enum {
  BTN_STATE_IDLE,
  BTN_STATE_PRESSED,         // waiting to see if long press
  BTN_STATE_WAIT_FOR_DOUBLE, // released, waiting for second press
  BTN_STATE_SECOND_PRESSED,  // second press detected, waiting for release
  BTN_STATE_LONG_HOLD,       // long press active
} btn_state_t;

typedef struct {
  btn_state_t state;
  uint16_t timer_ms;  // countdown timer
  bool pending_press;  // ISR sets this
  bool pending_release;
} btn_instance_t;

static btn_instance_t buttons[NUM_BUTTONS];
static h2_button_callback_t app_callback;
static uint16_t long_press_ms = DEFAULT_LONG_PRESS_MS;
static uint16_t double_press_ms = DEFAULT_DOUBLE_PRESS_MS;

// Map button index to gang/side
// 1-gang: btn0/btn1 -> gang 0
// 2-gang: btn0/btn1 -> gang 1 (right), btn2/btn3 -> gang 0 (left)
static inline uint8_t btn_to_gang(uint8_t btn_idx)
{
  if (h2_active_gang_count >= 2) {
    return (btn_idx < 2) ? 1 : 0;
  }
  return 0;
}

static inline h2_button_side_t btn_to_side(uint8_t btn_idx)
{
  return (btn_idx % 2 == 0) ? H2_BTN_SIDE_UP : H2_BTN_SIDE_DOWN;
}

static void emit(uint8_t btn_idx, h2_button_action_t action)
{
  if (app_callback) {
    app_callback(btn_to_gang(btn_idx), btn_to_side(btn_idx), action);
  }
}

void h2_button_init(h2_button_callback_t callback)
{
  app_callback = callback;
  for (uint8_t i = 0; i < NUM_BUTTONS; i++) {
    buttons[i].state = BTN_STATE_IDLE;
    buttons[i].timer_ms = 0;
    buttons[i].pending_press = false;
    buttons[i].pending_release = false;
  }
}

void h2_button_on_change(uint8_t button_index, bool pressed)
{
  if (button_index >= h2_active_gang_count * 2) {
    return;
  }
  if (pressed) {
    buttons[button_index].pending_press = true;
  } else {
    buttons[button_index].pending_release = true;
  }
}

void h2_button_tick(void)
{
  uint8_t active_buttons = h2_active_gang_count * 2;
  for (uint8_t i = 0; i < active_buttons; i++) {
    btn_instance_t *b = &buttons[i];
    bool press = b->pending_press;
    bool release = b->pending_release;
    b->pending_press = false;
    b->pending_release = false;

    switch (b->state) {
      case BTN_STATE_IDLE:
        if (press) {
          emit(i, H2_BTN_ACTION_PRESS);
          b->state = BTN_STATE_PRESSED;
          b->timer_ms = long_press_ms;
        }
        break;

      case BTN_STATE_PRESSED:
        if (b->timer_ms <= TICK_INTERVAL_MS) {
          // Long press threshold reached
          emit(i, H2_BTN_ACTION_LONG);
          b->state = BTN_STATE_LONG_HOLD;
        } else if (release) {
          // Emit SINGLE immediately on release (controls lights right away),
          // then wait for possible second press to also emit DOUBLE.
          emit(i, H2_BTN_ACTION_SINGLE);
          b->state = BTN_STATE_WAIT_FOR_DOUBLE;
          b->timer_ms = double_press_ms;
        } else {
          b->timer_ms -= TICK_INTERVAL_MS;
        }
        break;

      case BTN_STATE_WAIT_FOR_DOUBLE:
        if (press) {
          b->state = BTN_STATE_SECOND_PRESSED;
        } else if (b->timer_ms <= TICK_INTERVAL_MS) {
          // Double press window expired — nothing more to do
          b->state = BTN_STATE_IDLE;
        } else {
          b->timer_ms -= TICK_INTERVAL_MS;
        }
        break;

      case BTN_STATE_SECOND_PRESSED:
        if (release) {
          emit(i, H2_BTN_ACTION_DOUBLE);
          b->state = BTN_STATE_IDLE;
        }
        break;

      case BTN_STATE_LONG_HOLD:
        if (release) {
          emit(i, H2_BTN_ACTION_RELEASE);
          b->state = BTN_STATE_IDLE;
        }
        break;
    }
  }
}

void h2_button_set_long_press_ms(uint16_t ms)
{
  long_press_ms = ms;
}

void h2_button_set_double_press_ms(uint16_t ms)
{
  double_press_ms = ms;
}
