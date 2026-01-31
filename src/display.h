#pragma once

#include <stdint.h>
#include <driver/gpio.h>

// Brutzelboy Display Configuration (ILI9341)
#define LCD_PIN_NUM_MISO            GPIO_NUM_NC
#define LCD_PIN_NUM_MOSI            GPIO_NUM_12
#define LCD_PIN_NUM_CLK             GPIO_NUM_48
#define LCD_PIN_NUM_CS              GPIO_NUM_NC
#define LCD_PIN_NUM_DC              GPIO_NUM_47
#define LCD_PIN_NUM_BCKL            GPIO_NUM_39
#define LCD_PIN_NUM_RST             GPIO_NUM_3

#define SCREEN_WIDTH  292   // Nutzbare Breite
#define SCREEN_HEIGHT 240   // Volle Höhe
#define SCREEN_OFFSET_LEFT 0  // Linke Seite des Displays
#define SCREEN_OFFSET_TOP  0

#ifdef __cplusplus
extern "C" {
#endif

static inline uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | ((uint16_t)(b & 0xF8) >> 3);
}

void ili9341_init(void);
void ili9341_deinit(void);
void ili9341_writeBE(const uint16_t *buffer);
void ili9341_write_frame(const uint16_t *buffer);

#ifdef __cplusplus
}
#endif
