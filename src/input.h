#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <driver/gpio.h>
#include <driver/adc.h>

#ifdef __cplusplus
extern "C" {
#endif

// GPIO Pins for Input-Buttons
#define INPUT_GPIO_BOOT GPIO_NUM_0
#define INPUT_GPIO_A GPIO_NUM_15
#define INPUT_GPIO_B GPIO_NUM_5
#define INPUT_GPIO_SELECT GPIO_NUM_16
#define INPUT_GPIO_START GPIO_NUM_17
#define INPUT_GPIO_MENU GPIO_NUM_18
#define INPUT_GPIO_OPTION GPIO_NUM_8

// Unused GPIO definitions (for reference, not configured)
#define INPUT_GPIO_UP GPIO_NUM_NC
#define INPUT_GPIO_DOWN GPIO_NUM_NC
#define INPUT_GPIO_LEFT GPIO_NUM_NC
#define INPUT_GPIO_RIGHT GPIO_NUM_NC

// ADC channels for UP/DOWN and LEFT/RIGHT
#define ADC_CHANNEL_UP_DOWN ADC1_CHANNEL_6    // GPIO_NUM_7 for UP/DOWN
#define ADC_CHANNEL_LEFT_RIGHT ADC1_CHANNEL_5 // GPIO_NUM_6 for LEFT/RIGHT

#define ADC_THRESHOLD_UP 1800    // UP is at high values
#define ADC_THRESHOLD_DOWN 300  // DOWN is at low-medium values
#define ADC_THRESHOLD_LEFT 1800  // LEFT is at high values
#define ADC_THRESHOLD_RIGHT 300 // RIGHT is at low-medium values

#define ADC_FILTER_WINDOW 150 // Stability filter for ADC

// Input button definitions
enum {
    INPUT_BOOT = 0,
    INPUT_UP,
    INPUT_DOWN,
    INPUT_LEFT,
    INPUT_RIGHT,
    INPUT_A,
    INPUT_B,
    INPUT_SELECT,
    INPUT_START,
    INPUT_MENU,
    INPUT_OPTION,

    INPUT_MAX
};

// Public Input subsystem API
void input_init(void);
uint32_t input_get_state(void);
int input_wait_for_press(int timeout_ticks);

#ifdef __cplusplus
}
#endif
