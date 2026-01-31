#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_log.h>

#include "input.h"


static uint16_t gamepad_state = 0;
static int adc_up_down_value = 0;
static int adc_left_right_value = 0;

// Helper function for ADC reading with filtering
static int adc_read_stable(adc1_channel_t channel, int *old_value) {
    int new_value = adc1_get_raw(channel);
    if (abs(*old_value - new_value) > ADC_FILTER_WINDOW) {
        *old_value = new_value;
        return new_value;
    }
    return *old_value;
}


uint16_t input_read_raw(void)
{
    uint16_t state = 0;

    //int joyY = adc1_get_raw(ADC_CHANNEL_UP_DOWN);
    int joyY = adc_read_stable(ADC_CHANNEL_UP_DOWN, &adc_up_down_value);
    //int joyX = adc1_get_raw(ADC_CHANNEL_LEFT_RIGHT);
    int joyX = adc_read_stable(ADC_CHANNEL_LEFT_RIGHT, &adc_left_right_value);

    if (joyX > ADC_THRESHOLD_LEFT)
        state |= (1 << INPUT_LEFT);
    else if (joyX > ADC_THRESHOLD_RIGHT)
        state |= (1 << INPUT_RIGHT);

    if (joyY > ADC_THRESHOLD_UP)
        state |= (1 << INPUT_UP);
    else if (joyY > ADC_THRESHOLD_DOWN)
        state |= (1 << INPUT_DOWN);

    state |= (!gpio_get_level(INPUT_GPIO_SELECT)) ? (1 << INPUT_SELECT) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_START)) ? (1 << INPUT_START) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_A)) ? (1 << INPUT_A) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_B)) ? (1 << INPUT_B) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_MENU)) ? (1 << INPUT_MENU) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_OPTION)) ? (1 << INPUT_OPTION) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_BOOT)) ? (1 << INPUT_BOOT) : 0;

    return state;
}

int input_wait_for_button_press(int ticks)
{
    uint16_t previousState = gamepad_state;
    uint16_t timeout = xTaskGetTickCount() + ticks;

    while (true)
    {
        uint16_t state = gamepad_state;

        for (int i = 0; i < INPUT_MAX; i++)
        {
            if (!(previousState & (1 << i)) && (state & (1 << i))) {
                return i;
            }
        }

        if (ticks > 0 && timeout < xTaskGetTickCount()) {
            break;
        }

        previousState = state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    return -1;
}

static void input_task(void *arg)
{
    uint8_t debounce[INPUT_MAX];

    // Initialize state
    for (int i = 0; i < INPUT_MAX; ++i)
    {
        debounce[i] = 0xff;
    }

    while (1)
    {
        // Read hardware
        uint16_t state = input_read_raw();

        // Debounce
        for (int i = 0; i < INPUT_MAX; ++i)
        {
            debounce[i] <<= 1;
            debounce[i] |= (state >> i) & 1;
            switch (debounce[i] & 0x03)
            {
                case 0x00:
                    gamepad_state &= ~(1 << i);
                    break;

                case 0x03:
                    gamepad_state |= (1 << i);
                    break;

                default:
                    // ignore
                    break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(60));
    }

    vTaskDelete(NULL);
}

// Get current input state
uint16_t input_get_state(void) {
    return gamepad_state;
}

void input_init()
{
    gpio_set_direction(INPUT_GPIO_SELECT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_SELECT, GPIO_PULLUP_ONLY);

    gpio_set_direction(INPUT_GPIO_START, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_START, GPIO_PULLUP_ONLY);

    gpio_set_direction(INPUT_GPIO_A, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_A, GPIO_PULLUP_ONLY);

    gpio_set_direction(INPUT_GPIO_B, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_B, GPIO_PULLUP_ONLY);

    adc1_config_width(ADC_WIDTH_MAX);
    adc1_config_channel_atten(ADC_CHANNEL_UP_DOWN, ADC_ATTEN_DB_6);
    adc1_config_channel_atten(ADC_CHANNEL_LEFT_RIGHT, ADC_ATTEN_DB_6);

    gpio_set_direction(INPUT_GPIO_MENU, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_MENU, GPIO_PULLUP_ONLY);

    gpio_set_direction(INPUT_GPIO_OPTION, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_OPTION, GPIO_PULLUP_ONLY);
    gpio_set_direction(INPUT_GPIO_BOOT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_OPTION, GPIO_PULLUP_ONLY);

    // Start background polling
    xTaskCreatePinnedToCore(&input_task, "input_task", 1024 * 2, NULL, 5, NULL, 1);

    ESP_LOGI(__func__, "done.");
}

int input_raw_y_value(){
    return adc1_get_raw(ADC_CHANNEL_UP_DOWN);
}

int input_raw_x_value(){
    return adc1_get_raw(ADC_CHANNEL_LEFT_RIGHT);
}
