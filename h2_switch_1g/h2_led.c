#include "h2_led.h"
#include "h2_led_config.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_gpio.h"

#ifndef H2_SWITCH_GANG_COUNT
#define H2_SWITCH_GANG_COUNT 1
#endif

#define TRANSITION_STEPS 10  // ~200ms at 20ms tick rate

typedef struct {
  uint8_t current_red;   // 0-255 PWM duty
  uint8_t current_blue;  // 0-255 PWM duty
  uint8_t target_red;
  uint8_t target_blue;
  TIMER_TypeDef *timer;
  uint8_t cc_red;
  uint8_t cc_blue;
} led_pwm_t;

typedef struct {
  h2_led_mode_t mode;
  uint8_t brightness;  // 0-100
  uint8_t on_color;    // 0-100 (R->B)
  uint8_t off_color;   // 0-100 (R->B)
  uint8_t manual_color;
  bool relay_on;
  bool light_on;
  led_pwm_t led_up;
  led_pwm_t led_dn;
} gang_led_t;

static gang_led_t gangs[H2_SWITCH_GANG_COUNT];

//--------------------------------------------------
// Hardware init

static void init_timer(TIMER_TypeDef *timer, uint8_t timer_index,
                       uint8_t cc_r, uint8_t r_port, uint8_t r_pin,
                       uint8_t cc_b, uint8_t b_port, uint8_t b_pin)
{
  // Timer base init — count up, no prescaling needed for LED PWM
  // With 39MHz clock and TOP=255: ~153kHz PWM, well above flicker threshold
  TIMER_Init_TypeDef init = TIMER_INIT_DEFAULT;
  init.prescale = timerPrescale64;  // 39MHz/64/256 ≈ 2.4kHz
  init.enable = false;
  TIMER_Init(timer, &init);
  TIMER_TopSet(timer, H2_LED_PWM_TOP);

  // CC init for PWM mode — active-low (inverted) since LEDs are active-low
  TIMER_InitCC_TypeDef ccInit = TIMER_INITCC_DEFAULT;
  ccInit.mode = timerCCModePWM;
  ccInit.outInvert = true;

  TIMER_InitCC(timer, cc_r, &ccInit);
  TIMER_InitCC(timer, cc_b, &ccInit);

  // Set initial duty to 0 (LED off)
  TIMER_CompareSet(timer, cc_r, 0);
  TIMER_CompareSet(timer, cc_b, 0);

  // GPIO pin mode — push-pull for PWM output
  GPIO_PinModeSet(r_port, r_pin, gpioModePushPull, 0);
  GPIO_PinModeSet(b_port, b_pin, gpioModePushPull, 0);

  // Route timer CC outputs to GPIO pins (Series 2 routing)
  uint32_t routeen = 0;
  if (cc_r == 0) {
    GPIO->TIMERROUTE[timer_index].CC0ROUTE =
      ((uint32_t)r_port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
      | ((uint32_t)r_pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
    routeen |= GPIO_TIMER_ROUTEEN_CC0PEN;
  } else if (cc_r == 1) {
    GPIO->TIMERROUTE[timer_index].CC1ROUTE =
      ((uint32_t)r_port << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)
      | ((uint32_t)r_pin << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
    routeen |= GPIO_TIMER_ROUTEEN_CC1PEN;
  } else {
    GPIO->TIMERROUTE[timer_index].CC2ROUTE =
      ((uint32_t)r_port << _GPIO_TIMER_CC2ROUTE_PORT_SHIFT)
      | ((uint32_t)r_pin << _GPIO_TIMER_CC2ROUTE_PIN_SHIFT);
    routeen |= GPIO_TIMER_ROUTEEN_CC2PEN;
  }

  if (cc_b == 0) {
    GPIO->TIMERROUTE[timer_index].CC0ROUTE =
      ((uint32_t)b_port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
      | ((uint32_t)b_pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
    routeen |= GPIO_TIMER_ROUTEEN_CC0PEN;
  } else if (cc_b == 1) {
    GPIO->TIMERROUTE[timer_index].CC1ROUTE =
      ((uint32_t)b_port << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)
      | ((uint32_t)b_pin << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
    routeen |= GPIO_TIMER_ROUTEEN_CC1PEN;
  } else {
    GPIO->TIMERROUTE[timer_index].CC2ROUTE =
      ((uint32_t)b_port << _GPIO_TIMER_CC2ROUTE_PORT_SHIFT)
      | ((uint32_t)b_pin << _GPIO_TIMER_CC2ROUTE_PIN_SHIFT);
    routeen |= GPIO_TIMER_ROUTEEN_CC2PEN;
  }

  GPIO->TIMERROUTE[timer_index].ROUTEEN = routeen;

  // Start the timer
  TIMER_Enable(timer, true);
}

//--------------------------------------------------
// Color/brightness to PWM conversion

static void color_to_pwm(uint8_t color, uint8_t brightness,
                          uint8_t *red_out, uint8_t *blue_out)
{
  uint16_t blue_pct = color;
  uint16_t red_pct = 100 - color;
  *red_out  = (uint8_t)((red_pct * brightness * 255) / (100 * 100));
  *blue_out = (uint8_t)((blue_pct * brightness * 255) / (100 * 100));
}

static void set_target(gang_led_t *g, uint8_t color, uint8_t brightness)
{
  uint8_t r, b;
  color_to_pwm(color, brightness, &r, &b);
  g->led_up.target_red = r;
  g->led_up.target_blue = b;
  g->led_dn.target_red = r;
  g->led_dn.target_blue = b;
}

static void update_targets(uint8_t gang)
{
  gang_led_t *g = &gangs[gang];

  switch (g->mode) {
    case H2_LED_MODE_FOLLOW_RELAY:
      if (g->relay_on) {
        set_target(g, g->on_color, g->brightness);
      } else {
        set_target(g, g->off_color, g->brightness);
      }
      break;

    case H2_LED_MODE_FOLLOW_LIGHT:
      if (g->light_on) {
        set_target(g, g->on_color, g->brightness);
      } else {
        set_target(g, g->off_color, g->brightness);
      }
      break;

    case H2_LED_MODE_MANUAL:
      set_target(g, g->manual_color, g->brightness);
      break;

    case H2_LED_MODE_TIME_GRADIENT:
      // TODO: Implement time-based color interpolation
      set_target(g, g->manual_color, g->brightness);
      break;
  }
}

static uint8_t step_towards(uint8_t current, uint8_t target)
{
  if (current == target) {
    return current;
  }
  int16_t diff = (int16_t)target - (int16_t)current;
  int16_t step = diff / TRANSITION_STEPS;
  if (step == 0) {
    step = (diff > 0) ? 1 : -1;
  }
  return (uint8_t)((int16_t)current + step);
}

static void apply_pwm(led_pwm_t *led)
{
  TIMER_CompareSet(led->timer, led->cc_red, led->current_red);
  TIMER_CompareSet(led->timer, led->cc_blue, led->current_blue);
}

//--------------------------------------------------
// Public API

void h2_led_init(void)
{
  // Enable clocks
  CMU_ClockEnable(cmuClock_TIMER0, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Initialize gang 0 LED state
  gangs[0].mode = H2_LED_MODE_FOLLOW_RELAY;
  gangs[0].brightness = 100;
  gangs[0].on_color = 10;
  gangs[0].off_color = 90;
  gangs[0].manual_color = 50;
  gangs[0].relay_on = false;
  gangs[0].light_on = false;

  gangs[0].led_up = (led_pwm_t){
    .timer = H2_LED_UP_TIMER,
    .cc_red = H2_LED_UP_R_CC,
    .cc_blue = H2_LED_UP_B_CC,
  };
  gangs[0].led_dn = (led_pwm_t){
    .timer = H2_LED_DN_TIMER,
    .cc_red = H2_LED_DN_R_CC,
    .cc_blue = H2_LED_DN_B_CC,
  };

  // Initialize timer hardware
  init_timer(H2_LED_UP_TIMER, H2_LED_UP_TIMER_INDEX,
             H2_LED_UP_R_CC, H2_LED_UP_R_PORT, H2_LED_UP_R_PIN,
             H2_LED_UP_B_CC, H2_LED_UP_B_PORT, H2_LED_UP_B_PIN);

  init_timer(H2_LED_DN_TIMER, H2_LED_DN_TIMER_INDEX,
             H2_LED_DN_R_CC, H2_LED_DN_R_PORT, H2_LED_DN_R_PIN,
             H2_LED_DN_B_CC, H2_LED_DN_B_PORT, H2_LED_DN_B_PIN);

#if H2_SWITCH_GANG_COUNT >= 2
  // TODO: Initialize gang 1 LEDs with TIMER2/TIMER3
  gangs[1].mode = H2_LED_MODE_FOLLOW_RELAY;
  gangs[1].brightness = 100;
  gangs[1].on_color = 10;
  gangs[1].off_color = 90;
  gangs[1].manual_color = 50;
  gangs[1].relay_on = false;
  gangs[1].light_on = false;
#endif

  // Set initial targets
  for (uint8_t i = 0; i < H2_SWITCH_GANG_COUNT; i++) {
    update_targets(i);
  }
}

void h2_led_set_mode(uint8_t gang, h2_led_mode_t mode)
{
  if (gang >= H2_SWITCH_GANG_COUNT) return;
  gangs[gang].mode = mode;
  update_targets(gang);
}

void h2_led_update_relay_state(uint8_t gang, bool relay_on)
{
  if (gang >= H2_SWITCH_GANG_COUNT) return;
  gangs[gang].relay_on = relay_on;
  if (gangs[gang].mode == H2_LED_MODE_FOLLOW_RELAY) {
    update_targets(gang);
  }
}

void h2_led_update_light_state(uint8_t gang, bool light_on)
{
  if (gang >= H2_SWITCH_GANG_COUNT) return;
  gangs[gang].light_on = light_on;
  if (gangs[gang].mode == H2_LED_MODE_FOLLOW_LIGHT) {
    update_targets(gang);
  }
}

void h2_led_set_manual(uint8_t gang, uint8_t color, uint8_t brightness)
{
  if (gang >= H2_SWITCH_GANG_COUNT) return;
  gangs[gang].manual_color = color;
  gangs[gang].brightness = brightness;
  if (gangs[gang].mode == H2_LED_MODE_MANUAL) {
    update_targets(gang);
  }
}

void h2_led_set_one(uint8_t gang, h2_led_position_t pos, uint8_t color, uint8_t brightness)
{
  if (gang >= H2_SWITCH_GANG_COUNT) return;
  uint8_t r, b;
  color_to_pwm(color, brightness, &r, &b);
  led_pwm_t *led = (pos == H2_LED_UP) ? &gangs[gang].led_up : &gangs[gang].led_dn;
  led->target_red = r;
  led->target_blue = b;
}

void h2_led_set_on_color(uint8_t gang, uint8_t color)
{
  if (gang >= H2_SWITCH_GANG_COUNT) return;
  gangs[gang].on_color = color;
  update_targets(gang);
}

void h2_led_set_off_color(uint8_t gang, uint8_t color)
{
  if (gang >= H2_SWITCH_GANG_COUNT) return;
  gangs[gang].off_color = color;
  update_targets(gang);
}

void h2_led_set_brightness(uint8_t gang, uint8_t brightness)
{
  if (gang >= H2_SWITCH_GANG_COUNT) return;
  gangs[gang].brightness = brightness;
  update_targets(gang);
}

void h2_led_tick(void)
{
  for (uint8_t g = 0; g < H2_SWITCH_GANG_COUNT; g++) {
    led_pwm_t *up = &gangs[g].led_up;
    led_pwm_t *dn = &gangs[g].led_dn;

    up->current_red  = step_towards(up->current_red,  up->target_red);
    up->current_blue = step_towards(up->current_blue, up->target_blue);
    dn->current_red  = step_towards(dn->current_red,  dn->target_red);
    dn->current_blue = step_towards(dn->current_blue, dn->target_blue);

    apply_pwm(up);
    apply_pwm(dn);
  }
}
