#include "app/framework/include/af.h"
#include "h2_led.h"
#include "h2_led_config.h"
#include "em_cmu.h"
#include "em_timer.h"
#include "em_gpio.h"

#ifndef H2_SWITCH_GANG_COUNT
#define H2_SWITCH_GANG_COUNT 2
#endif

extern uint8_t h2_active_gang_count;

#define TRANSITION_STEPS 5  // ~500ms perceived with exponential easing at 20ms tick

typedef struct {
  uint8_t current_red;   // 0-255 PWM duty
  uint8_t current_blue;  // 0-255 PWM duty
  uint8_t target_red;
  uint8_t target_blue;
  TIMER_TypeDef *timer_red;
  TIMER_TypeDef *timer_blue;
  uint8_t cc_red;
  uint8_t cc_blue;
} led_pwm_t;

typedef struct {
  h2_led_mode_t mode;
  bool enabled;        // indicator on/off
  bool inverted;       // swap on/off colors
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
  uint32_t routeen = GPIO->TIMERROUTE[timer_index].ROUTEEN;

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

// Add a single CC channel + GPIO route to an already-running timer
static void init_cc_on_existing_timer(TIMER_TypeDef *timer, uint8_t timer_index,
                                       uint8_t cc, uint8_t port, uint8_t pin)
{
  TIMER_InitCC_TypeDef ccInit = TIMER_INITCC_DEFAULT;
  ccInit.mode = timerCCModePWM;
  ccInit.outInvert = true;
  // TIMER_InitCC on Series 2 disables/re-enables the timer to write CFG,
  // but doesn't restart it. We must re-issue CMD_START after.
  TIMER_InitCC(timer, cc, &ccInit);
  timer->CMD = TIMER_CMD_START;
  TIMER_CompareSet(timer, cc, 0);

  GPIO_PinModeSet(port, pin, gpioModePushPull, 0);

  uint32_t routeen = GPIO->TIMERROUTE[timer_index].ROUTEEN;
  if (cc == 0) {
    GPIO->TIMERROUTE[timer_index].CC0ROUTE =
      ((uint32_t)port << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
      | ((uint32_t)pin << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);
    routeen |= GPIO_TIMER_ROUTEEN_CC0PEN;
  } else if (cc == 1) {
    GPIO->TIMERROUTE[timer_index].CC1ROUTE =
      ((uint32_t)port << _GPIO_TIMER_CC1ROUTE_PORT_SHIFT)
      | ((uint32_t)pin << _GPIO_TIMER_CC1ROUTE_PIN_SHIFT);
    routeen |= GPIO_TIMER_ROUTEEN_CC1PEN;
  } else {
    GPIO->TIMERROUTE[timer_index].CC2ROUTE =
      ((uint32_t)port << _GPIO_TIMER_CC2ROUTE_PORT_SHIFT)
      | ((uint32_t)pin << _GPIO_TIMER_CC2ROUTE_PIN_SHIFT);
    routeen |= GPIO_TIMER_ROUTEEN_CC2PEN;
  }
  GPIO->TIMERROUTE[timer_index].ROUTEEN = routeen;
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

static void set_target_one(led_pwm_t *led, uint8_t color, uint8_t brightness)
{
  uint8_t r, b;
  color_to_pwm(color, brightness, &r, &b);
  led->target_red = r;
  led->target_blue = b;
}

static void set_target(gang_led_t *g, uint8_t color, uint8_t brightness)
{
  set_target_one(&g->led_up, color, brightness);
  set_target_one(&g->led_dn, color, brightness);
}

static void update_targets(uint8_t gang)
{
  gang_led_t *g = &gangs[gang];

  if (!g->enabled) {
    set_target(g, 0, 0);
    return;
  }

  switch (g->mode) {
    case H2_LED_MODE_FOLLOW_RELAY: {
      led_pwm_t *on_led  = g->inverted ? &g->led_dn : &g->led_up;
      led_pwm_t *off_led = g->inverted ? &g->led_up : &g->led_dn;
      if (g->relay_on) {
        set_target_one(on_led, g->on_color, g->brightness);
        set_target_one(off_led, 0, 0);
      } else {
        set_target_one(on_led, 0, 0);
        set_target_one(off_led, g->off_color, g->brightness);
      }
      break;
    }

    case H2_LED_MODE_FOLLOW_LIGHT: {
      led_pwm_t *on_led  = g->inverted ? &g->led_dn : &g->led_up;
      led_pwm_t *off_led = g->inverted ? &g->led_up : &g->led_dn;
      if (g->light_on) {
        set_target_one(on_led, g->on_color, g->brightness);
        set_target_one(off_led, 0, 0);
      } else {
        set_target_one(on_led, 0, 0);
        set_target_one(off_led, g->off_color, g->brightness);
      }
      break;
    }

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
  TIMER_CompareSet(led->timer_red, led->cc_red, led->current_red);
  TIMER_CompareSet(led->timer_blue, led->cc_blue, led->current_blue);
}

//--------------------------------------------------
// Gang defaults init helper

static void init_gang_defaults(uint8_t g)
{
  gangs[g].mode = H2_LED_MODE_FOLLOW_RELAY;
  gangs[g].enabled = true;
  gangs[g].brightness = 100;
  gangs[g].on_color = 10;
  gangs[g].off_color = 90;
  gangs[g].manual_color = 50;
  gangs[g].relay_on = false;
  gangs[g].light_on = false;
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
  init_gang_defaults(0);

  gangs[0].led_up = (led_pwm_t){
    .timer_red = H2_LED_UP_TIMER,
    .timer_blue = H2_LED_UP_TIMER,
    .cc_red = H2_LED_UP_R_CC,
    .cc_blue = H2_LED_UP_B_CC,
  };
  gangs[0].led_dn = (led_pwm_t){
    .timer_red = H2_LED_DN_TIMER,
    .timer_blue = H2_LED_DN_TIMER,
    .cc_red = H2_LED_DN_R_CC,
    .cc_blue = H2_LED_DN_B_CC,
  };

  // Initialize timer hardware for gang 0
  init_timer(H2_LED_UP_TIMER, H2_LED_UP_TIMER_INDEX,
             H2_LED_UP_R_CC, H2_LED_UP_R_PORT, H2_LED_UP_R_PIN,
             H2_LED_UP_B_CC, H2_LED_UP_B_PORT, H2_LED_UP_B_PIN);

  init_timer(H2_LED_DN_TIMER, H2_LED_DN_TIMER_INDEX,
             H2_LED_DN_R_CC, H2_LED_DN_R_PORT, H2_LED_DN_R_PIN,
             H2_LED_DN_B_CC, H2_LED_DN_B_PORT, H2_LED_DN_B_PIN);

  if (h2_active_gang_count >= 2) {
    CMU_ClockEnable(cmuClock_TIMER3, true);

    init_gang_defaults(1);

    // 2-gang physical layout: LED3/LED4 = left (gang 0), LED1/LED2 = right (gang 1)
    // Swap: gang 0 gets LED3/LED4, gang 1 gets LED1/LED2 (already inited on TIMER0/TIMER1)

    // Move gang 0 from LED1/LED2 to LED3/LED4
    // LED3 (gang 0 up): red on TIMER1 CC2 (PC09), blue on TIMER0 CC2 (PC08)
    gangs[0].led_up = (led_pwm_t){
      .timer_red = H2_LED_G1_UP_R_TIMER,
      .timer_blue = H2_LED_G1_UP_B_TIMER,
      .cc_red = H2_LED_G1_UP_R_CC,
      .cc_blue = H2_LED_G1_UP_B_CC,
    };
    init_cc_on_existing_timer(H2_LED_G1_UP_R_TIMER, H2_LED_G1_UP_R_TIMER_INDEX,
                               H2_LED_G1_UP_R_CC, H2_LED_G1_UP_R_PORT, H2_LED_G1_UP_R_PIN);
    init_cc_on_existing_timer(H2_LED_G1_UP_B_TIMER, H2_LED_G1_UP_B_TIMER_INDEX,
                               H2_LED_G1_UP_B_CC, H2_LED_G1_UP_B_PORT, H2_LED_G1_UP_B_PIN);

    // LED4 (gang 0 dn): both colors on TIMER3
    gangs[0].led_dn = (led_pwm_t){
      .timer_red = H2_LED_G1_DN_TIMER,
      .timer_blue = H2_LED_G1_DN_TIMER,
      .cc_red = H2_LED_G1_DN_R_CC,
      .cc_blue = H2_LED_G1_DN_B_CC,
    };
    init_timer(H2_LED_G1_DN_TIMER, H2_LED_G1_DN_TIMER_INDEX,
               H2_LED_G1_DN_R_CC, H2_LED_G1_DN_R_PORT, H2_LED_G1_DN_R_PIN,
               H2_LED_G1_DN_B_CC, H2_LED_G1_DN_B_PORT, H2_LED_G1_DN_B_PIN);

    // Gang 1 gets LED1/LED2 (right side, already initialized on TIMER0 CC0/CC1 + TIMER1 CC0/CC1)
    gangs[1].led_up = (led_pwm_t){
      .timer_red = H2_LED_UP_TIMER,
      .timer_blue = H2_LED_UP_TIMER,
      .cc_red = H2_LED_UP_R_CC,
      .cc_blue = H2_LED_UP_B_CC,
    };
    gangs[1].led_dn = (led_pwm_t){
      .timer_red = H2_LED_DN_TIMER,
      .timer_blue = H2_LED_DN_TIMER,
      .cc_red = H2_LED_DN_R_CC,
      .cc_blue = H2_LED_DN_B_CC,
    };
  }

  // Set initial targets
  for (uint8_t i = 0; i < h2_active_gang_count; i++) {
    update_targets(i);
  }

  // Debug: verify timers are running after init
  sl_zigbee_app_debug_println("LED init: T0 STATUS=0x%X T1 STATUS=0x%X",
    TIMER0->STATUS, TIMER1->STATUS);
  if (h2_active_gang_count >= 2) {
    sl_zigbee_app_debug_println("LED init: T3 STATUS=0x%X", TIMER3->STATUS);
  }
}

void h2_led_set_enabled(uint8_t gang, bool enabled)
{
  if (gang >= h2_active_gang_count) return;
  gangs[gang].enabled = enabled;
  update_targets(gang);
}

void h2_led_set_inverted(uint8_t gang, bool inverted)
{
  if (gang >= h2_active_gang_count) return;
  gangs[gang].inverted = inverted;
  update_targets(gang);
}

void h2_led_set_mode(uint8_t gang, h2_led_mode_t mode)
{
  if (gang >= h2_active_gang_count) return;
  gangs[gang].mode = mode;
  update_targets(gang);
}

void h2_led_update_relay_state(uint8_t gang, bool relay_on)
{
  if (gang >= h2_active_gang_count) return;
  gangs[gang].relay_on = relay_on;
  if (gangs[gang].mode == H2_LED_MODE_FOLLOW_RELAY) {
    update_targets(gang);
  }
}

void h2_led_update_light_state(uint8_t gang, bool light_on)
{
  if (gang >= h2_active_gang_count) return;
  gangs[gang].light_on = light_on;
  if (gangs[gang].mode == H2_LED_MODE_FOLLOW_LIGHT) {
    update_targets(gang);
  }
}

void h2_led_set_manual(uint8_t gang, uint8_t color, uint8_t brightness)
{
  if (gang >= h2_active_gang_count) return;
  gangs[gang].manual_color = color;
  gangs[gang].brightness = brightness;
  if (gangs[gang].mode == H2_LED_MODE_MANUAL) {
    update_targets(gang);
  }
}

void h2_led_set_one(uint8_t gang, h2_led_position_t pos, uint8_t color, uint8_t brightness)
{
  if (gang >= h2_active_gang_count) return;
  uint8_t r, b;
  color_to_pwm(color, brightness, &r, &b);
  led_pwm_t *led = (pos == H2_LED_UP) ? &gangs[gang].led_up : &gangs[gang].led_dn;
  led->target_red = r;
  led->target_blue = b;
}

void h2_led_set_on_color(uint8_t gang, uint8_t color)
{
  if (gang >= h2_active_gang_count) return;
  gangs[gang].on_color = color;
  update_targets(gang);
}

void h2_led_set_off_color(uint8_t gang, uint8_t color)
{
  if (gang >= h2_active_gang_count) return;
  gangs[gang].off_color = color;
  update_targets(gang);
}

void h2_led_set_brightness(uint8_t gang, uint8_t brightness)
{
  if (gang >= h2_active_gang_count) return;
  gangs[gang].brightness = brightness;
  update_targets(gang);
}

void h2_led_tick(void)
{
  for (uint8_t g = 0; g < h2_active_gang_count; g++) {
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
