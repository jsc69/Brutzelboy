#pragma once

#include <cstdint>
#include "display.h"
#include "input.h"

// Include µGUI with C linkage
extern "C"
{
#include "ugui/ugui.h"
}

// Battery
#define RG_BATTERY_DRIVER 1
#define RG_BATTERY_ADC_CHANNEL ADC1_CHANNEL_3
#define RG_BATTERY_CALC_PERCENT(raw) (((raw)*2.f - 3500.f) / (4200.f - 3500.f) * 100.f)
#define RG_BATTERY_CALC_VOLTAGE(raw) ((raw)*2.f * 0.001f)

// Status LED
#define RG_GPIO_LED GPIO_NUM_38

// Display backlight
#define RG_GPIO_LCD_BCKL GPIO_NUM_39

// SPI
#define RG_GPIO_SDSPI_MISO GPIO_NUM_9
#define RG_GPIO_SDSPI_MOSI GPIO_NUM_11
#define RG_GPIO_SDSPI_CLK GPIO_NUM_13
#define RG_GPIO_SDSPI_CS GPIO_NUM_10

// External I2S DAC
#define RG_GPIO_SND_I2S_BCK GPIO_NUM_41
#define RG_GPIO_SND_I2S_WS GPIO_NUM_42
#define RG_GPIO_SND_I2S_DATA GPIO_NUM_40

// I2C
#define RG_I2C_SDA GPIO_NUM_2
#define RG_I2C_CLK GPIO_NUM_1

// Serial IO
#define RG_TXD GPIO_NUM_43
#define RG_RXD GPIO_NUM_44
#define RG_USB_DP GPIO_NUM_20
#define RG_USB_DM GPIO_NUM_19

// Hardware Flags
#define INIT_LCD 1
#define INIT_BUTTONS 2
#define INIT_SD_CARD 4
#define INIT_SPIFFS 8
#define INIT_WIFI 16
#define INIT_AUDIO 32
#define INIT_CARTRIDGE 64
#define INIT_INFRARED 128

// Sound events
#define EVENT_SOUND_START 1
#define EVENT_SOUND_STOP 2

class Brutzelboy {
private:
  void initDisplay();
  void initAudio();
  void initSPIFFS();
  void initSDCard();
  void readWifiConfig();
  void initWiFi();
  void initCartridge();
  void initInfrared();

  UG_GUI gui;
  uint16_t* framebuffer;

  // for the callback function
  static Brutzelboy* current_instance;

  // static callback for µGUI pixel rendering
  static void gui_pixel_callback(int16_t x, int16_t y, uint16_t color);

  uint8_t hardwareSupport = 0;
  bool isHardwareSupported(const uint8_t hardware) {
    return hardwareSupport & hardware;
  }

  uint16_t keys;
  void checkButtons();
  void processButton(uint16_t key, bool pressed);

  void (*buttonEventHandler)(const uint8_t event, const uint16_t value);
  void (*soundEventHandler)(const uint8_t event, const uint16_t value);

  void parseCredentialValues(const char* s);
  char ssid[200];
  char pwd[200];

public:
  /**
   * @brief Constuctor for a Brutzelboy instance with chosen hardware support.
   * @param uint8_t Flgs of hardware that should be supported. See Hardware Flags (#define INIT_xxx)
   */
  Brutzelboy(uint8_t hardware);

  /**
   * @brief Constuctor for a Brutzelboy instance with complete hardware supprt.
   */
  Brutzelboy();

  ~Brutzelboy();

  /**
   * @brief Init the Brutzelboy with wifi credentials from SDCard. Should be called in setup()
   */
  void begin();
  /**
   * @brief Init the Brutzelboy with given wifi credentials. Should be called in setup()
   */
  void begin(const char* ssid, const char* pwd);
  /**
   * @brief Loop for the Brutzelboy.Should call frequently to allow the Brutzelboy doing its thing. Call it in loop()
   */
  void loop();

  /**
   * @brief Set the green led on the left side of the display
   * @param bool
   */
  void setLed(const bool on);

  //--------------------------- DISPLAY -----------------------------
  /**
   * @brief Set the background light of the display
   * @param bool
   */
  void setLcd(const bool on);

  /**
   * @brief Get pointer for the framebuffer
   * @return uint16_t[292,240]
   */
  uint16_t* getFramebuffer() const {
    return framebuffer;
  }

  /**
   * @brief Get pointer for UG_GUI
   * @return UG_GUI*
   */
  UG_GUI* getGUI() {
    return &gui;
  }

  /**
   * @brief Set brightness of display
   * @param new brightness (0-255)
   */
  void setBrightness(uint8_t bightness);

  // Frame update
  void updateDisplay();

  /**
   * @brief Print text on the given position
   * @param uint16_t x position
   * @param uint16_t y position
   * @param char* text
   */
  void drawString(int16_t x, int16_t y, const char* str);

  /**
   * @brief Set the drawing color in RGB565. Default: write
   * @param uint16_t color rgb565
   */
  void setTextcolor(uint16_t color);

  /**
   * @brief Set the background color in RGB565. Default: black
   * @param uint16_t color rgb565
   */
  void setBackcolor(uint16_t color);

  /**
   * @brief Set the font for text
   * @param uint16_t color rgb565
   */
  void setFont(const UG_FONT* font);

  /**
   * @brief Clear the display in the background color
   * @param uint16_t color rgb565
   */
  void fillScreen(uint16_t color);

  void drawPixel(int16_t x, int16_t y, uint16_t color);
  void drawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
  void drawFrame(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
  void fillFrame(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
  void drawRoundFrame(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t r, uint16_t color);
  void fillRoundFrame(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t r, uint16_t color);
  void drawTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t h, uint16_t color);
  void fillTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t h, uint16_t color);
  void drawCircle(int16_t x, int16_t y, int16_t radius, uint16_t color);
  void fillCircle(int16_t x, int16_t y, int16_t radius, uint16_t color);
  void drawArc(int16_t x0, int16_t y0, int16_t r, uint8_t s, uint16_t c);
  void drawMesh(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
  void drawBitmap16(int16_t x, int16_t y, uint16_t w, uint16_t h, uint8_t* data);

  //--------------------------- SOUND -----------------------------
  /**
   * @brief Add a text in queue to be play via TTS
   * @param char* text to be play
   * @param char* language to use in TTS
   */
  void addTtsSoundToQueue(const char* source, const char* language);

  /**
   * @brief Add a sound file from SDCard in queue to be played
   * @param char* filename incl. absolute path
   */
  void addFileSoundToQueue(const char* source);

  /**
   * @brief Add a sound file from web in queue to be played
   * @param char* url of sound file
   */
  void addUrlSoundToQueue(const char* source);

  /**
   * @brief Play text directly via TTS
   * @param char* text to play
   * @param char* language to use in TTS
   */
  void playTts(const char* text, const char* language);

  /**
   * @brief Play sound file from web directly
   * @param char* url of sound file
   */
  void playUrl(const char* url);

  /**
   * @brief Play sound file from SDCard directly
   * @param char* filename incl. absolute path
   */
  void playFile(const char* path);

  /**
   * @brief Set the volume (0-21)
   * @param uint8_t new volume
   */
  void setVolume(uint8_t volume);

  /**
   * @brief Start playing next sound from queue if no sound is playing
   */
  void playQueuedSound();

  //------------------------------------------------------
  // Input functions
  //------------------------------------------------------
  int waitForInput(int ticks);
  uint32_t getInputState(void);

  //--------------------------- BUTTONS -----------------------------
  /**
   * @brief Check if a button is pressed
   * @param uint16_t id of button
   * @return true if pressed
   */
//  bool isButtonPressed(const uint16_t key);
  /**
   * @brief Add handler for button event
   * @param void (*userDefinedEventHandler) function to handle button events
   */
//  void setButtonEventHandler(void (*userDefinedEventHandler)(const uint8_t event, const uint16_t value)) {
//    buttonEventHandler = userDefinedEventHandler;
//  }
  /**
   * @brief Add handler for sound event
   * @param void (*userDefinedEventHandler) function to handle sound events
   */
//  void setSoundEventHandler(void (*userDefinedEventHandler)(const uint8_t event, const uint16_t value)) {
//    soundEventHandler = userDefinedEventHandler;
//  }
};
