#include <Brutzelboy.h>

Brutzelboy *bb = nullptr;

uint16_t keyStatus = 0;

// Handler für ButtonEvents
void onButtonEvent(const uint8_t event, const uint16_t value) {
  Serial.println("press");
  if (event == EVENT_KEY_DOWN) {
    keyStatus |= value;
  } else if (event == EVENT_KEY_UP) {
    keyStatus &= ~value;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  bb = new Brutzelboy(INIT_LCD | INIT_BUTTONS);
  // Init Brutzelboy
  bb->begin();

  bb->setFont(&FONT_8X12);
  bb->setBrightness(255);
  bb->fillScreen(0);
  bb->updateDisplay();

  // Event Handler für die Buttons definieren
  bb->setButtonEventHandler(onButtonEvent);
}

void loop() {
  // Gib den Brutzelboy seine Zeit
  bb->loop();

  bb->setBackcolor(C_BLACK);
  bb->setTextcolor(keyStatus & KEY_UP ? C_GREEN : C_WHITE);
  bb->drawString(5,5,"KEY_UP");
  bb->setTextcolor(keyStatus & KEY_DOWN ? C_GREEN : C_WHITE);
  bb->drawString(5,25,"KEY_DOWN");
  bb->setTextcolor(keyStatus & KEY_LEFT ? C_GREEN : C_WHITE);
  bb->drawString(5,45,"KEY_LEFT");
  bb->setTextcolor(keyStatus & KEY_RIGHT ? C_GREEN : C_WHITE);
  bb->drawString(5,65,"KEY_RIGHT");

  bb->setTextcolor(keyStatus & KEY_A ? C_GREEN : C_WHITE);
  bb->drawString(5,85,"KEY_A");
  bb->setTextcolor(keyStatus & KEY_B ? C_GREEN : C_WHITE);
  bb->drawString(5,105,"KEY_B");

  bb->setTextcolor(keyStatus & KEY_SELECT ? C_GREEN : C_WHITE);
  bb->drawString(5,125,"KEY_SELECT");
  bb->setTextcolor(keyStatus & KEY_START ? C_GREEN : C_WHITE);
  bb->drawString(5,145,"KEY_START");

  bb->setTextcolor(keyStatus & KEY_MENU ? C_GREEN : C_WHITE);
  bb->drawString(5,165,"KEY_MENU");
  bb->setTextcolor(keyStatus & KEY_BOOT ? C_GREEN : C_WHITE);
  bb->drawString(5,185,"KEY_BOOT");
  bb->setTextcolor(keyStatus & KEY_OPTION ? C_GREEN : C_WHITE);
  bb->drawString(5,205,"KEY_OPTION");

  bb->updateDisplay();

  Serial.println(keyStatus);
}
