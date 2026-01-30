#include <esp_log.h>

#include "input.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdlib.h>

static const char* TAG = "INPUT";

// Internal state
static volatile uint32_t gamepad_state = 0;
static int adc_up_down_value = 0;
static int adc_left_right_value = 0;

// Helper function for ADC reading with filtering
static int adc_read_stable(adc1_channel_t channel, int *old_value) {
    int new_value = adc1_get_raw(channel);
    if (abs(*old_value - new_value) < ADC_FILTER_WINDOW) {
        *old_value = new_value;
        return new_value;
    }
    return *old_value;
}

// Read raw input state (internal use only)
static uint32_t input_read_raw(void) {
    uint32_t state = 0;
    
    state |= (!gpio_get_level(INPUT_GPIO_BOOT)) ? (1 << INPUT_BOOT) : 0;

    // Reading ADC-based cursor keys
    int up_down = adc_read_stable(ADC_CHANNEL_UP_DOWN, &adc_up_down_value);
    int left_right = adc_read_stable(ADC_CHANNEL_LEFT_RIGHT, &adc_left_right_value);

    ESP_LOGI(TAG,"UP/DOWN: %d, LEFT/RIGHT: %d", up_down, left_right);
    
    // UP/DOWN 
    if (up_down > ADC_THRESHOLD_UP) {
        state |= (1 << INPUT_UP);
    } else if (up_down > ADC_THRESHOLD_DOWN) {
        state |= (1 << INPUT_DOWN);
    }
    
    // LEFT/RIGHT 
    if (left_right > ADC_THRESHOLD_LEFT) {
        state |= (1 << INPUT_LEFT);
    } else if (left_right > ADC_THRESHOLD_RIGHT) {
        state |= (1 << INPUT_RIGHT);
    }
    
    // GPIO-based buttons
    state |= (!gpio_get_level(INPUT_GPIO_A)) ? (1 << INPUT_A) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_B)) ? (1 << INPUT_B) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_SELECT)) ? (1 << INPUT_SELECT) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_START)) ? (1 << INPUT_START) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_MENU)) ? (1 << INPUT_MENU) : 0;
    state |= (!gpio_get_level(INPUT_GPIO_OPTION)) ? (1 << INPUT_OPTION) : 0;
    
    return state;
}

// Background task
static void input_task(void *arg) {
    uint8_t debounce[INPUT_MAX];
    for (int i = 0; i < INPUT_MAX; ++i) {
        debounce[i] = 0xff;
    }
    
    while (1) {
        uint32_t state = input_read_raw();
        
        for (int i = 0; i < INPUT_MAX; ++i) {
            debounce[i] <<= 1;
            debounce[i] |= (state >> i) & 1;
            
            switch (debounce[i] & 0x03) {
                case 0x00:
                    gamepad_state &= ~(1 << i);
                    break;
                case 0x03:
                    gamepad_state |= (1 << i);
                    break;
                default:
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Initialize input subsystem
void input_init(void) {
    ESP_LOGI(TAG, "Initializing input system...");
    
    // Initialize ADC for cursor keys
    adc1_config_width((adc_bits_width_t)(ADC_WIDTH_MAX - 1));
    adc1_config_channel_atten(ADC_CHANNEL_UP_DOWN, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC_CHANNEL_LEFT_RIGHT, ADC_ATTEN_DB_12);
    
    // Initialize GPIO-based buttons
    gpio_set_direction(INPUT_GPIO_BOOT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_BOOT, GPIO_PULLUP_ONLY);
    
    gpio_set_direction(INPUT_GPIO_A, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_A, GPIO_PULLUP_ONLY);
    
    gpio_set_direction(INPUT_GPIO_B, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_B, GPIO_PULLUP_ONLY);
    
    gpio_set_direction(INPUT_GPIO_SELECT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_SELECT, GPIO_PULLUP_ONLY);
    
    gpio_set_direction(INPUT_GPIO_START, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_START, GPIO_PULLUP_ONLY);
    
    gpio_set_direction(INPUT_GPIO_MENU, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_MENU, GPIO_PULLUP_ONLY);
    
    gpio_set_direction(INPUT_GPIO_OPTION, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INPUT_GPIO_OPTION, GPIO_PULLUP_ONLY);
    
    // Start background polling task
    xTaskCreatePinnedToCore(&input_task, "input_task", 1024 * 2, NULL, 5, NULL, 1);
    
    ESP_LOGI(TAG, "Input system initialized.");
}

// Get current input state
uint32_t input_get_state(void) {
    return gamepad_state;
}

// Wait for button press with timeout
int input_wait_for_press(int timeout_ticks) {
    uint32_t previousState = gamepad_state;
    uint32_t timeout = xTaskGetTickCount() + timeout_ticks;
    
    while (true) {
        uint32_t state = gamepad_state;
        
        // Check for new button presses (transition from 0 to 1)
        for (int i = 0; i < INPUT_MAX; i++) {
            if (!(previousState & (1 << i)) && (state & (1 << i))) {
                return i;
            }
        }
        
        if (timeout_ticks > 0 && timeout < xTaskGetTickCount()) {
            break;
        }
        
        previousState = state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    return -1;
}
