#include <esp_log.h>

#include "Brutzelboy.h"
#include <Audio.h>
#include <FS.h>
#include "SPIFFS.h"

extern "C"
{
#include "ugui/ugui.h"
}
#define SOUND_QUEUE_LENGTH 16

static const char* TAG = "BRUTZELBOY";
const String wifiConfig = "/retro-go/config/wifi.json";

Brutzelboy* Brutzelboy::current_instance = nullptr;


Audio audio;
enum SoundType { tts,
                 file,
                 url };
struct Sound {
  SoundType type;
  char source[161];
  char language[6];
};
Sound soundQueue[SOUND_QUEUE_LENGTH];
volatile bool soundIsPlaying = false;
volatile uint8_t soundPlayed = 255;
volatile uint8_t queuePointer = 255;



TaskHandle_t TaskSound;

uint16_t upCount = 0;
uint16_t downCount = 0;
uint16_t leftCount = 0;
uint16_t rightCount = 0;
uint16_t trigger = 300;
uint16_t topBorder = 1600;
uint16_t middleBorder = 1500;
uint16_t bottomBorder = 900;


SPIClass spi = SPIClass();

// Dummy function, if handlers are not used
void doNothing(const uint8_t event, const uint16_t value) {}

// audio callbacks
void taskPlaySound(void* pvParameters) {
  Brutzelboy* boy = (Brutzelboy*)pvParameters;
  for (;;) {
    audio.loop();

    if (!soundIsPlaying) {
      boy->playQueuedSound();
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void audio_info_callback(Audio::msg_t m) {
  switch (m.e) {
    case Audio::evt_eof:
      ESP_LOGI(TAG, "Sound done");
      soundIsPlaying = false;
      break;
    case Audio::evt_info:
      if (String(m.msg).startsWith("End of")) {
        ESP_LOGI(TAG, "Sound done");
        soundIsPlaying = false;
      }
      break;
  }
}

// Static callback wrapper for display
void Brutzelboy::gui_pixel_callback(int16_t x, int16_t y, uint16_t color) {
  if (current_instance && current_instance->framebuffer && x >= 0
      && x < SCREEN_WIDTH && y >= 0 && y < SCREEN_HEIGHT) {
    uint16_t swap_color = (color >> 8) | (color << 8);
    current_instance->framebuffer[y * SCREEN_WIDTH + x] = swap_color;
  }
}

Brutzelboy::Brutzelboy() {
  Brutzelboy(255);
}

Brutzelboy::Brutzelboy(uint8_t hardware)
  : framebuffer(nullptr) {
  current_instance = this;

  buttonEventHandler = doNothing;
  soundEventHandler = doNothing;
  hardwareSupport = hardware;
}

Brutzelboy::~Brutzelboy() {
  if (framebuffer) {
    free(framebuffer);
  }
  if (current_instance == this) {
    current_instance = nullptr;
  }
}


void Brutzelboy::begin(const char* ssid, const char* pwd) {
  strcpy(this->ssid, ssid);
  strcpy(this->pwd, pwd);
  begin();
}

void Brutzelboy::begin() {
  ESP_LOGI(TAG, "Free heap: %d\n", ESP.getFreeHeap());
  ESP_LOGI(TAG, "Free PSRAM: %d\n", ESP.getFreePsram());
  
  if (hardwareSupport & INIT_BUTTONS) {
    input_init();
  }
  pinMode(RG_GPIO_LED, OUTPUT);
  pinMode(RG_GPIO_LCD_BCKL, OUTPUT);

  if (hardwareSupport & INIT_SPIFFS) {
    initSPIFFS();
  }

  if (hardwareSupport & INIT_SD_CARD) {
    initSDCard();
  }

  if (hardwareSupport & INIT_LCD) {
    initDisplay();
  } else {
    setLcd(false);
  }
  if (hardwareSupport & INIT_AUDIO) {
    initAudio();
    xTaskCreatePinnedToCore(
      taskPlaySound,  // Task function
      "TaskSound",    // name of task
      16384,          // Stack size of task
      (void*)this,    // parameter of the task
      1,              // priority of the task
      &TaskSound,     // Task handle to keep track of created task
      0);             // pin task to core 1
  }
  if (hardwareSupport & INIT_WIFI) {
    initWiFi();
  }
  if (hardwareSupport & INIT_CARTRIDGE) {
    initCartridge();
  }
  if (hardwareSupport & INIT_INFRARED) {
    initInfrared();
  }

  Serial.println("Der Brutzelboy meldet sich zum Dienst. Folgendes wird unterstützt:");
  if (hardwareSupport & INIT_LCD) Serial.println("\t* Display");
  if (hardwareSupport & INIT_BUTTONS) Serial.println("\t* Buttons");
  if (hardwareSupport & INIT_SPIFFS) Serial.println("\t* SPIFFS (nicht getestet)");
  if (hardwareSupport & INIT_SD_CARD) Serial.println("\t* SD Card");
  if (hardwareSupport & INIT_WIFI) Serial.println("\t* Wifi");
  if (hardwareSupport & INIT_AUDIO) Serial.println("\t* Audio");
  if (hardwareSupport & INIT_CARTRIDGE) Serial.println("\t* Cartridge (nicht implementiert)");
  if (hardwareSupport & INIT_INFRARED) Serial.println("\t* IR Leds (nicht implementiert)");
  Serial.println();
}

void Brutzelboy::initDisplay() {
  ESP_LOGI(TAG, "initDisplay");
  // Allocate framebuffer in PSRAM
  size_t buffer_size = SCREEN_WIDTH * SCREEN_HEIGHT * sizeof(uint16_t);
  framebuffer = (uint16_t*)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  if (!framebuffer) {
    ESP_LOGE(TAG, "ERROR: Failed to allocate framebuffer!\n");
    hardwareSupport &= ~INIT_LCD;
    return;
  }
  ESP_LOGI(TAG, "Framebuffer allocated at: %p\n", framebuffer);

  ili9341_init();
  UG_Init(&gui, gui_pixel_callback, SCREEN_WIDTH, SCREEN_HEIGHT);

  memset(framebuffer, 0, buffer_size);
  updateDisplay();
  ESP_LOGI(TAG, "LCD initialized successfully");
  hardwareSupport |= INIT_LCD;
}

void Brutzelboy::initWiFi() {
  ESP_LOGI(TAG, "initWiFi");
  if (strlen(ssid) == 0 || strlen(pwd) == 0) {
    readWifiConfig();
  }
  if (strlen(ssid) == 0 || strlen(pwd) == 0) {
    Serial.println("ERROR: WIFI konnte nicht verbunden werden!");
    Serial.println("Prüfe die Datei \"/retro-go/config/wifi.json\" auf der SDCard oder stelle SSID und PWD im INO ein!");
    delay(2000);
    hardwareSupport &= ~INIT_WIFI;
    return;
  }
  ESP_LOGI(TAG, "Connect to WiFi");
  WiFi.begin(ssid, pwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    ESP_LOGI(TAG, ".");
  }
  ESP_LOGI(TAG, "WiFi connected!");
  hardwareSupport |= INIT_WIFI;
}

void Brutzelboy::initSPIFFS() {
  ESP_LOGI(TAG, "initSPIFFS");

  if (!SPIFFS.begin(false)) {
    ESP_LOGW(TAG, "SPIFFS not formatted, formatting now...");

    // Formatieren des internen Flash, NICHT die SD-Karte
    if (!SPIFFS.begin(true)) {
      ESP_LOGE(TAG, "ERROR: SPIFFS format failed");
      delay(2000);
      hardwareSupport &= ~INIT_SPIFFS;
      return;
    }
    ESP_LOGI(TAG, "SPIFFS formatted successfully");
  }

  ESP_LOGI(TAG, "SPIFFS mounted successfully");
  hardwareSupport |= INIT_SPIFFS;
}

void Brutzelboy::initAudio() {
  ESP_LOGI(TAG, "initAudio");
  Audio::audio_info_callback = audio_info_callback;
  audio.setPinout(RG_GPIO_SND_I2S_BCK, RG_GPIO_SND_I2S_WS, RG_GPIO_SND_I2S_DATA);
  queuePointer = 0;
  soundPlayed = 0;
  audio.setVolume(16);
  ESP_LOGI(TAG, "Audio initialized successfully");
  hardwareSupport |= INIT_AUDIO;
}

void Brutzelboy::initSDCard() {
  ESP_LOGI(TAG, "initSDCard");
  spi.begin(RG_GPIO_SDSPI_CLK, RG_GPIO_SDSPI_MISO, RG_GPIO_SDSPI_MOSI, RG_GPIO_SDSPI_CS);
  while (!SD.begin(RG_GPIO_SDSPI_CS, spi, 80000000)) {
    ESP_LOGE(TAG, "ERROR: SD CARD FAILED, OR NOT PRESENT!");
    hardwareSupport &= ~INIT_SD_CARD;
    delay(2000);
  }
  ESP_LOGI(TAG, "SD-Card mounted successfully");
  hardwareSupport |= INIT_SD_CARD;
}

void Brutzelboy::initCartridge() {
  ESP_LOGI(TAG, "initCartridge");
  // TO BE DONE!
  ESP_LOGI(TAG, "Cartridge is not supported by now");
  hardwareSupport &= ~INIT_CARTRIDGE;
}

void Brutzelboy::initInfrared() {
  ESP_LOGI(TAG, "initInfrared");
  // TO BE DONE!
  ESP_LOGI(TAG, "Infrared leds are not supported by now");
  hardwareSupport &= ~INIT_INFRARED;
}

/************************************************************************
 * SOUND FUNCTION
 ************************************************************************/
void Brutzelboy::playTts(const char* text, const char* language) {
  if (!(hardwareSupport & INIT_AUDIO) || !(hardwareSupport & INIT_WIFI)) {
    ESP_LOGE(TAG, "ERROR: Hardware does not support function \"playTTS\". Please check initialization of Brutzelboy.");
    return;
  }
  if (queuePointer == soundPlayed)
    return;

  if (!soundIsPlaying) {
    soundIsPlaying = true;
    char buf[150];
    strncpy(buf, text, 150);
    ESP_LOGI(TAG, "talk(%d:%d): '%s'\n", soundPlayed % SOUND_QUEUE_LENGTH, queuePointer % SOUND_QUEUE_LENGTH, buf);
    audio.connecttospeech(buf, language);
  }
}

void Brutzelboy::playUrl(const char* url) {
  if (!(hardwareSupport & INIT_AUDIO) || !(hardwareSupport & INIT_WIFI)) {
    ESP_LOGE(TAG, "ERROR: Hardware does not support function \"playURL\". Please check initialization of Brutzelboy.");
    return;
  }
  if (!soundIsPlaying) {
    soundIsPlaying = true;
    audio.connecttohost(url);
  }
}

void Brutzelboy::playFile(const char* path) {
  if (!(hardwareSupport & INIT_AUDIO) || !(hardwareSupport & INIT_SD_CARD)) {
    ESP_LOGE(TAG, "ERROR: Hardware does not support function \"playFile\". Please check initialization of Brutzelboy.");
    return;
  }
  if (!soundIsPlaying) {
    soundIsPlaying = true;
    audio.connecttoFS(SD, path);
  }
}

void Brutzelboy::addTtsSoundToQueue(const char* source, const char* language) {
  if (!(hardwareSupport & INIT_AUDIO) || !(hardwareSupport & INIT_WIFI)) {
    ESP_LOGE(TAG, "ERROR: Hardware does not support function \"addTTSSoundToQueue\". Please check initialization of Brutzelboy.");
    return;
  }
  uint8_t nextSound = (queuePointer + 1) % SOUND_QUEUE_LENGTH;

  if (soundPlayed % SOUND_QUEUE_LENGTH == nextSound) {
    ESP_LOGI(TAG, "Skip(%d:%d): '%s'\n", soundPlayed % SOUND_QUEUE_LENGTH, nextSound, source);
    return;
  }
  ESP_LOGI(TAG, "Add(%d:%d): '%s'\n", soundPlayed % SOUND_QUEUE_LENGTH, nextSound, source);

  Sound current;
  current.type = tts;
  strncpy(current.source, source, 160);
  current.source[160] = '\0';
  strncpy(current.language, language, 5);
  current.language[5] = '\0';
  soundQueue[nextSound] = current;
  queuePointer = nextSound;
}

void Brutzelboy::addFileSoundToQueue(const char* source) {
  if (!(hardwareSupport & INIT_AUDIO) || !(hardwareSupport & INIT_SD_CARD)) {
    ESP_LOGE(TAG, "ERROR: Hardware does not support function \"addFileSoundToQueue\". Please check initialization of Brutzelboy.");
    return;
  }
  uint8_t nextSound = (queuePointer + 1) % SOUND_QUEUE_LENGTH;
  if (soundPlayed % SOUND_QUEUE_LENGTH == nextSound) {
    ESP_LOGI(TAG, "Skip(%d:%d): '%s'\n", soundPlayed % SOUND_QUEUE_LENGTH, nextSound, source);
    return;
  }

  Sound current;
  current.type = file;
  strncpy(current.source, source, 160);
  current.source[160] = '\0';
  soundQueue[nextSound] = current;
  queuePointer = nextSound;
}

void Brutzelboy::addUrlSoundToQueue(const char* source) {
  if (!(hardwareSupport & INIT_AUDIO) || !(hardwareSupport & INIT_WIFI)) {
    ESP_LOGE(TAG, "ERROR: Hardware does not support function \"addUrlSoundToQueue\". Please check initialization of Brutzelboy.");
    return;
  }
  uint8_t nextSound = (queuePointer + 1) % SOUND_QUEUE_LENGTH;
  if (soundPlayed % SOUND_QUEUE_LENGTH == nextSound) {
    ESP_LOGI(TAG, "Skip(%d:%d): '%s'\n", soundPlayed % SOUND_QUEUE_LENGTH, nextSound, source);
    return;
  }

  Sound current;
  current.type = file;
  strncpy(current.source, source, 160);
  current.source[160] = '\0';
  soundQueue[nextSound] = current;
  queuePointer = nextSound;
}

void Brutzelboy::playQueuedSound() {
  if (soundPlayed == queuePointer) {
    return;
  }

  uint8_t nextSound = (soundPlayed + 1) % SOUND_QUEUE_LENGTH;
  Sound current = soundQueue[nextSound];

  soundIsPlaying = true;
  ESP_LOGI(TAG, "Talk(%d:%d): '%s'\n", nextSound, queuePointer, current.source);

  if (current.type == tts) {
    audio.connecttospeech(current.source, current.language);
  } else if (current.type == file) {
    audio.connecttoFS(SD, current.source);
  } else if (current.type == url) {
    audio.connecttohost(current.source);
  }
  soundPlayed = nextSound;
}

void Brutzelboy::setVolume(uint8_t volume) {
  if (!(hardwareSupport & INIT_AUDIO)) {
    ESP_LOGE(TAG, "ERROR: Hardware does not support function \"setVolume\". Please check initialization of Brutzelboy.");
    return;
  }
  if (volume > 21) {
    volume = 21;
  }
  if (volume < 0) {
    volume = 0;
  }
  audio.setVolume(volume);
}

/************************************************************************
 * DISPLAY FUNCTION
 ************************************************************************/
void Brutzelboy::setLcd(const bool on) {
  if (!(hardwareSupport & INIT_LCD)) {
    ESP_LOGE(TAG, "ERROR: Hardware does not support function \"setLcd\". Please check initialization of Brutzelboy.");
    return;
  }
  if (on) {
    analogWrite(RG_GPIO_LCD_BCKL, 255);
  } else {
    analogWrite(RG_GPIO_LCD_BCKL, 0);
  }
}

void Brutzelboy::setBrightness(uint8_t bightness) {
  if (!(hardwareSupport & INIT_LCD)) {
    ESP_LOGE(TAG, "ERROR: Hardware does not support function \"setBrightness\". Please check initialization of Brutzelboy.");
    return;
  }
  analogWrite(RG_GPIO_LCD_BCKL, bightness);
}

void Brutzelboy::updateDisplay() {
  if (framebuffer) {
    ili9341_write_frame(framebuffer);
  }
}

void Brutzelboy::fillScreen(uint16_t color) {
  UG_FillScreen(color);
}

void Brutzelboy::drawPixel(int16_t x, int16_t y, uint16_t color) {
  gui_pixel_callback(x, y, color);
}

void Brutzelboy::drawLine(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
  UG_DrawLine(x1, y1, x2, y2, color);
}

void Brutzelboy::drawFrame(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
  UG_DrawFrame(x1, y1, x2, y2, color);
}

void Brutzelboy::fillFrame(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
  UG_FillFrame(x1, y1, x2, y2, color);
}

void Brutzelboy::drawRoundFrame(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t r, uint16_t color) {
  UG_DrawRoundFrame(x1, y1, x2, y2, r, color);
}

void Brutzelboy::fillRoundFrame(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t r, uint16_t color) {
  UG_FillRoundFrame(x1, y1, x2, y2, r, color);
}

void Brutzelboy::drawTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t h, uint16_t color) {
  UG_DrawTriangle(x1, y1, x2, y2, h, color);
}

void Brutzelboy::fillTriangle(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t h, uint16_t color) {
  UG_FillTriangle(x1, y1, x2, y2, h, color);
}

void Brutzelboy::drawCircle(int16_t x, int16_t y, int16_t radius, uint16_t color) {
  UG_DrawCircle(x, y, radius, color);
}

void Brutzelboy::fillCircle(int16_t x, int16_t y, int16_t radius, uint16_t color) {
  UG_FillCircle(x, y, radius, color);
}

void Brutzelboy::drawArc(int16_t x0, int16_t y0, int16_t radius, uint8_t sector, uint16_t color) {
  UG_DrawArc(x0, y0, radius, sector, color);
}
void Brutzelboy::drawMesh(int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color) {
  UG_DrawMesh(x1, y1, x2, y2, color);
}

void Brutzelboy::setTextcolor(uint16_t color) {
  UG_SetForecolor(color);
}

void Brutzelboy::setBackcolor(uint16_t color) {
  UG_SetBackcolor(color);
}

void Brutzelboy::setFont(const UG_FONT* font) {
  UG_FontSelect((UG_FONT*)font);
}

void Brutzelboy::drawString(int16_t x, int16_t y, const char* str) {
  UG_PutString(x, y, str);
}


void Brutzelboy::drawBitmap16(int16_t x, int16_t y, uint16_t w, uint16_t h, uint8_t* data) {
  if (!framebuffer || !(hardwareSupport & INIT_LCD)) {
    return;
  }

  int16_t xStart = x;
  int16_t yStart = y;
  int16_t xEnd = x + w;
  int16_t yEnd = y + h;

  if (xStart < 0) xStart = 0;
  if (yStart < 0) yStart = 0;
  if (xEnd > SCREEN_WIDTH)  xEnd = SCREEN_WIDTH;
  if (yEnd > SCREEN_HEIGHT) yEnd = SCREEN_HEIGHT;

  if (xStart >= xEnd || yStart >= yEnd) {
    return;
  }

  uint16_t drawWidth  = xEnd - xStart;
  uint16_t drawHeight = yEnd - yStart;
  uint16_t srcOffsetX = xStart - x;
  uint16_t srcOffsetY = yStart - y;

  uint16_t* src = (uint16_t*)data;
  for (int16_t row = 0; row < drawHeight; row++) {
    uint16_t* dst = &framebuffer[(yStart + row) * SCREEN_WIDTH + xStart];
    uint16_t* lineSrc = &src[(srcOffsetY + row) * w + srcOffsetX];
    memcpy(dst, lineSrc, drawWidth * 2);
  }
}

/************************************************************************
 * KEY FUNCTIONS
 ************************************************************************/
//------------------------------------------------------
// Input functions
//------------------------------------------------------
int Brutzelboy::waitForInput(int ticks)
{
  return input_wait_for_button_press(ticks);
}

uint32_t Brutzelboy::getInputState(void)
{
  return input_get_state();
}

/*
void Brutzelboy::checkButtons() {
  if (!(hardwareSupport & INIT_BUTTONS)) {
    ESP_LOGE(TAG, "ERROR: Hardware does not support function \"checkKeys\". Please check initialization of Brutzelboy.");
    return;
  }
  // Analog Keys
  uint16_t updown = analogRead(RG_ADC_UP_DOWN);
  uint16_t leftright = analogRead(RG_ADC_LEFT_RIGHT);

  if (updown > topBorder) {
    if (upCount <= trigger) upCount++;
    downCount = 0;
  } else if (updown < middleBorder && updown > bottomBorder) {
    if (downCount <= trigger) downCount++;
    upCount = 0;
  } else {
    upCount = 0;
    downCount = 0;
  }
  if (leftright > topBorder) {
    if (leftCount <= trigger) leftCount++;
    rightCount = 0;
  } else if (leftright < middleBorder && leftright > bottomBorder) {
    if (rightCount <= trigger) rightCount++;
    leftCount = 0;
  } else {
    leftCount = 0;
    rightCount = 0;
  }
  processButton(KEY_UP, upCount >= trigger);
  processButton(KEY_DOWN, downCount >= trigger);
  processButton(KEY_LEFT, leftCount >= trigger);
  processButton(KEY_RIGHT, rightCount >= trigger);

  // Digital Keys
  uint8_t gpio = 0;
  for (uint16_t i = 16; i <= 1024; i = i << 1) {
    switch (i) {
      case KEY_SELECT:
        gpio = RG_GPIO_KEY_SELECT;
        break;
      case KEY_START:
        gpio = RG_GPIO_KEY_START;
        break;
      case KEY_MENU:
        gpio = RG_GPIO_KEY_MENU;
        break;
      case KEY_OPTION:
        gpio = RG_GPIO_KEY_OPTION;
        break;
      case KEY_A:
        gpio = RG_GPIO_KEY_A;
        break;
      case KEY_B:
        gpio = RG_GPIO_KEY_B;
        break;
      case KEY_BOOT:
        gpio = RG_GPIO_KEY_BOOT;
        break;
    }
    processButton(i, !digitalRead(gpio));
  }
}

void Brutzelboy::processButton(uint16_t key, bool pressed) {
  if (pressed) {
    if (!(keys & key)) {
      buttonEventHandler(EVENT_KEY_DOWN, key);
    }
    keys |= key;
  } else {
    if (keys & key) {
      buttonEventHandler(EVENT_KEY_UP, key);
    }
    keys &= ~key;
  }
}

bool Brutzelboy::isButtonPressed(const uint16_t key) {
  return keys & key;
}
*/

/************************************************************************
 * LED
 ************************************************************************/
void Brutzelboy::setLed(const bool on) {
  if (on) {
    digitalWrite(RG_GPIO_LED, HIGH);
  } else {
    digitalWrite(RG_GPIO_LED, LOW);
  }
}


/************************************************************************
 * LOOP
 ************************************************************************/
void Brutzelboy::loop() {
}


/************************************************************************
 * READ CONFIG
 ************************************************************************/
void Brutzelboy::parseCredentialValues(const char* s) {
  uint8_t pos[4];
  uint8_t index = 0;

  for (uint8_t i = 0; s[i] != '\0'; i++) {
    if (s[i] == '\"') {
      pos[index++] = i;
    }
  }
  if (index != 4) {
    ESP_LOGE(TAG, "Error parsing wifi.json line %s\n", s);
    return;
  }
  char key[1000];
  uint8_t i = 0;
  pos[0]++;
  pos[2]++;
  for (i = 0; i < pos[1] - pos[0] && s[pos[0] + i] != '\0'; i++) {
    key[i] = s[pos[0] + i];
  }
  key[i] = '\0';

  char value[1000];
  for (i = 0; i < pos[3] - pos[2] && s[pos[2] + i] != '\0'; i++) {
    value[i] = s[pos[2] + i];
  }
  value[i] = '\0';

  if (strcmp(key, "ssid0") == 0) {
    strcpy(this->ssid, value);
  }
  if (strcmp(key, "password0") == 0) {
    strcpy(this->pwd, value);
  }
}

void Brutzelboy::readWifiConfig() {
  strcpy(ssid, "");
  strcpy(pwd, "");
  if (!(hardwareSupport & INIT_SD_CARD)) {
    return;
  }

  File file = SD.open(wifiConfig);
  if (!file) {
    ESP_LOGW(TAG, "wifi.json does not exists");
    return;
  }
  char line[1000];
  uint16_t pos = 0;
  while (file.available()) {
    int c = file.read();
    if (c == '\n') {
      line[pos] = '\0';
      pos = 0;
      char buf[1000];
      strcpy(buf, line);
      if (strstr(buf, "{") == NULL) {
        parseCredentialValues(buf);
      }
    } else {
      line[pos++] = (char)c;
    }
    if (strlen(ssid) > 0 && strlen(pwd) > 0) {
      break;
    }
  }
  file.close();
}
