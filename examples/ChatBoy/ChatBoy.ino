#include <WiFiClient.h>
#include <IRCClient.h>
#include "Brutzelboy.h"
#include <esp_log.h>
#include <JPEGDecoder.h>
#include <HTTPClient.h>

// Twitch Daten - Bot muss justinfan<magic number> heißen
#define IRC_SERVER          "irc.chat.twitch.tv"
#define IRC_PORT            6667
#define BOT_NAME_PREFIX     "justinfan" // Prefix für annonymen Zugriff
#define TWITCH_OAUTH_TOKEN  ""          // kann leer bleiben bei annonymen Zugriff mit jutinfan.....

#define MAX_ROWS 8
#define MAX_COLS 47
#define FONT_HEIGHT 10

#define IMAGE_BUFFER_SIZE 1024

#define LOG_TAG "ChatBoy"


// WIFI-Anmeldung. Wenn leer werden die Werte von der SDCard gelesen (/retro-go/config/wifi.json)
const String SSID = "";
const String PWD = "";

// Name des Channels in Kleinbuchstaben - z.B. "thebrutzler"
const String twitchChannelName = "thebrutzler";
const String urlThumbnail  = "https://static-cdn.jtvnw.net/previews-ttv/live_user_" + twitchChannelName + "-288x162.jpg";
uint32_t botNumber;

WiFiClient wifiClient;
IRCClient client(IRC_SERVER, IRC_PORT, wifiClient);


// Der Brutzelboy
//Brutzelboy boy(255);
// oder zur Bestimmung welche Hardware benutzt wird:
Brutzelboy boy(INIT_LCD | INIT_BUTTONS |INIT_SPIFFS | INIT_SD_CARD | INIT_WIFI | INIT_AUDIO);

char textBuffer[MAX_ROWS][MAX_COLS+1];
String message;

uint8_t currentRow = 0;

// Timer für das Laden der Thumbnails
const uint32_t refreshJpegInterval = 30000;     // 10 Sekunden in Millisekunden
uint32_t previousMillis = -refreshJpegInterval; // Zeitpunkt des letzten Ladens


SET_LOOP_TASK_STACK_SIZE(8192);

uint16_t flash = 0;
uint8_t volume = 16;
uint8_t brightness = 127;

// Handler für ButtonEvents
void handleButtons() {
  int value = boy.getInputState();
    if (value == (1 << INPUT_OPTION) || value == (1 << INPUT_RIGHT)) {
      if (volume < 21) {
        boy.setVolume(++volume);
      }
      ESP_LOGI(LOG_TAG, "Volume set to %d\n", volume);
    } else if (value == (1 << INPUT_MENU) || value == (1 << INPUT_LEFT)) {
      if (volume > 0) {
        boy.setVolume(--volume);
      }
      ESP_LOGI(LOG_TAG, "Volume set to %d\n", volume);
    }

    if (value == (1 << INPUT_UP) && brightness < 255) {
      brightness += 16;
      boy.setBrightness(brightness);
    }
    if (value == (1 << INPUT_DOWN) && brightness > 15) {
      brightness -= 16;
      boy.setBrightness(brightness);
    }

    if (value == (1 << INPUT_A)) boy.setLcd(true);
    if (value == (1 << INPUT_B)) boy.setLcd(false);
    if (value == (1 << INPUT_START)) flash=10000;

}

bool displayImageFromUrl(const uint16_t x, const uint16_t y, const char* url) {
  HTTPClient http;
  ESP_LOGI(LOG_TAG, "Loading image from %s\n", url);
  
  http.setTimeout(2000); 
  http.begin(url);
  
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    if (contentLength <= 0) {
      ESP_LOGE(LOG_TAG, "Incorrect Content-Length!");
      http.end();
      return false;
    }

    uint8_t* jpgData = (uint8_t*)heap_caps_malloc(contentLength, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    
    if (!jpgData) {
      ESP_LOGE(LOG_TAG, "Image data do not fit in PSRAM (%d Bytes needed)!", contentLength);
      http.end();
      return false;
    }

    WiFiClient* stream = http.getStreamPtr();
    int bytesRead = 0;
    int totalRead = 0;

    while (http.connected() && totalRead < contentLength) {
      if (stream->available()) {
        bytesRead = stream->readBytes(jpgData + totalRead, contentLength - totalRead);
        totalRead += bytesRead;
      }
      delay(1); // feed watchdog
    }

    http.end();

    // decoding JPEG
    if (totalRead == contentLength && JpegDec.decodeArray(jpgData, contentLength)) {
      renderJpeg(x, y);
      heap_caps_free(jpgData);
      return true;
    } else {
      ESP_LOGW(LOG_TAG, "JPEG error or download was not complete (%d/%d)", totalRead, contentLength);
      heap_caps_free(jpgData);
    }
  } else {
    ESP_LOGE(LOG_TAG, "HTTP error: %d\n", httpCode);
    http.end();
  }
  return false;
}

void renderJpeg(const uint16_t offsetX, const uint16_t offsetY) {
  while (JpegDec.read()) {
    // Reale Maße dieses Blocks (MCUs sind meist 16x16)
    uint16_t mcuW = JpegDec.MCUWidth;
    uint16_t mcuH = JpegDec.MCUHeight;
    
    // Position im Bild
    int16_t x = JpegDec.MCUx * mcuW;
    int16_t y = JpegDec.MCUy * mcuH;

    // Clipping: Wie viel zeichnen wir wirklich? (Wichtig für Bildränder!)
    uint16_t drawW = mcuW;
    uint16_t drawH = mcuH;
    if ((x + drawW) > JpegDec.width)  drawW = JpegDec.width - x;
    if ((y + drawH) > JpegDec.height) drawH = JpegDec.height - y;

    // Puffer für die Pixel dieses Blocks (2 Bytes pro Pixel)
    uint8_t buffer[2 * drawW * drawH];

    // DER FIX FÜR DEN VARIABLEN VERSATZ & FARBEN:
    for (uint16_t row = 0; row < drawH; row++) {
      for (uint16_t col = 0; col < drawW; col++) {
        // Wir springen im Quell-Array (pImage) immer um die VOLLE mcuW (16)
        // Aber wir schreiben in unseren Ziel-Puffer nur die drawW (z.B. 8)
        uint16_t pixel = JpegDec.pImage[col + (row * mcuW)];
        
        // Index im Ziel-Puffer berechnen
        uint32_t targetIdx = 2 * (col + (row * drawW));
        
        // Byte-Swap (Big Endian für das Display)
        buffer[targetIdx]     = pixel >> 8;
        buffer[targetIdx + 1] = pixel & 0xFF;
      }
    }

    // Zeichnen über deinen Treiber
    boy.drawBitmap16(x + offsetX, y + offsetY, drawW, drawH, buffer);
  }
  boy.updateDisplay();
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  delay(500);

  esp_log_level_set("BRUTZELBOY", ESP_LOG_INFO);//kann wech?
  esp_log_level_set("DISPLAY", ESP_LOG_INFO);   //kann wech?
  esp_log_level_set("*", ESP_LOG_ERROR);

  Serial.println("Welcome to ChatBoy");

  if (SSID.isEmpty() || PWD.isEmpty()) {
    boy.begin();
  } else {
    boy.begin(SSID.c_str(), PWD.c_str());
  }

  boy.setTextcolor(RGB565(255, 255, 255));
  boy.setBackcolor(RGB565(0, 0, 0));
  boy.setFont(&FONT_5X8);
  boy.setBrightness(brightness);

  client.setCallback(callback);
  char buf[255];
  sprintf(buf, "Waiting for thumbnail from %s", twitchChannelName.c_str());
  boy.drawString(5, 10, buf);

  // Event handler definieren
  handleButtons();
  uint64_t chipId = ESP.getEfuseMac();
  botNumber = chipId % 90000 + 10000;
  boy.updateDisplay();
}


void loop() {
  client.loop();
  boy.loop();

  if (!client.connected()) {
    connectToTwitch();
  }
  
  if (flash > 0) {
    if (flash % 2000 == 0) {
      boy.setLed(false);
    } else if (flash % 1000 == 0) {
      boy.setLed(true);
    }
    flash--;
  } else {
    boy.setLed(false);
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= refreshJpegInterval) {
    displayImageFromUrl(0, 0, urlThumbnail.c_str());
    previousMillis = currentMillis;
  }
}

void connectToTwitch() {
  ESP_LOGI(LOG_TAG, "Attempting to connect to %s", twitchChannelName);
  char botname[50];
  sprintf(botname, "%s%d", BOT_NAME_PREFIX, botNumber);
  if (client.connect(botname, "", TWITCH_OAUTH_TOKEN)) {
    ESP_LOGI(LOG_TAG, "sending JOIN as %s...\n", botname);
    char message[150];
    sprintf(message, "Listening to %s as %s", twitchChannelName.c_str(), botname);
    printText(message);
//    client.sendRaw("CAP REQ :twitch.tv/tags");
    client.sendRaw("JOIN #" + twitchChannelName);
  } else {
    ESP_LOGW(LOG_TAG, "failed... try again in 5 seconds");
    delay(5000);
  }
}

// not used
void sendTwitchMessage(String message) {
  client.sendMessage(twitchChannelName, message);
}

void callback(IRCMessage ircMessage) {
  if (ircMessage.command == "PRIVMSG" && ircMessage.text[0] != '\001') {
    /*
    String talk(ircMessage.nick + " schreibt \""+ircMessage.text + "\"");
    if (talk.indexOf("!tts") >= 0) {
      talk = String(ircMessage.nick + " sagt \""+ircMessage.text + "\"");
      talk.replace("!tts", "");
    }
    */
    String talk = ircMessage.text;
    talk.replace("!tts", "");
    talk.replace("_", "");
    talk.replace("^", "");
    if(talk.isEmpty()) {
      return;
    }

    boy.addTtsSoundToQueue(talk.c_str(), "de");
    
    if (talk.equals("!ttscn ^") || talk.indexOf("jensefu") >= 0) {
      flash=10000;
    }
    if (talk.equals("^blink")) {
      flash=30000;
    }
    
    ircMessage.nick.toUpperCase();
    message = "<" + ircMessage.nick + "> " + ircMessage.text;
    printText(message.c_str());
  }
}

void printText(const char* text) {
  uint8_t x = 0;
  
  for(int i = 0; i < strlen(text); i++) {
    if (text[i] == 0) {
      break;
    }
    
    if (x >= MAX_COLS) {
      textBuffer[currentRow][MAX_COLS] = '\0';
      x = 0;
      currentRow++;
    }

    if (currentRow >= MAX_ROWS) {
      scrollText();
      currentRow = MAX_ROWS - 1;
    }
    textBuffer[currentRow][x] = text[i];
    x++;
  }
  
  textBuffer[currentRow][x] = '\0';
  currentRow++;
  if (currentRow >= MAX_ROWS) {
    scrollText();
    currentRow = MAX_ROWS - 1;
  }
  boy.fillFrame(0, 162, 292, 240, C_BLACK);
  for (int i = 0; i < MAX_ROWS; i++) {
    boy.drawString(5, 162 + i * FONT_HEIGHT, textBuffer[i]);
  }
  boy.updateDisplay();
}

void scrollText() {
  for (int i = 0; i < MAX_ROWS - 1; i++) {
    strcpy(textBuffer[i], textBuffer[i + 1]);
  }
  memset(textBuffer[MAX_ROWS - 1], '\0', MAX_COLS + 1);
}

