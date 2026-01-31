#pragma once

#include <stdint.h>

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

// ADC channels for UP/DOWN and LEFT/RIGHT
#define ADC_CHANNEL_UP_DOWN ADC1_CHANNEL_6
#define ADC_CHANNEL_LEFT_RIGHT ADC1_CHANNEL_5

#define ADC_THRESHOLD_UP    2800 // UP is at high values
#define ADC_THRESHOLD_DOWN   500 // DOWN is at low-medium values
#define ADC_THRESHOLD_LEFT  2800 // LEFT is at high values
#define ADC_THRESHOLD_RIGHT  500 // RIGHT is at low-medium values

#define ADC_FILTER_WINDOW    800 // Stability filter for ADC


enum
{
    INPUT_UP = 0,
    INPUT_RIGHT,
    INPUT_DOWN,
    INPUT_LEFT,
    INPUT_SELECT,
    INPUT_START,
    INPUT_A,
    INPUT_B,
    INPUT_MENU,
    INPUT_OPTION,
    INPUT_BOOT,

    INPUT_MAX
};

void input_init(void);
uint16_t input_read_raw();
uint16_t input_get_state(void);

int input_wait_for_button_press(int ticks);

int input_raw_x_value();
int input_raw_y_value();

#ifdef __cplusplus
}
#endif
