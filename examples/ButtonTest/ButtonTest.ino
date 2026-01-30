#include <Brutzelboy.h>

Brutzelboy *bb = nullptr;
uint16_t keyState = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  delay(500);

  bb = new Brutzelboy(INIT_LCD | INIT_BUTTONS);
  // Init Brutzelboy
  bb->begin();

  bb->setFont(&FONT_8X12);
  bb->setBrightness(255);
  bb->fillScreen(0);
  bb->updateDisplay();
}

void loop() {
  // Gib den Brutzelboy seine Zeit
  bb->loop();
  
  keyState = bb->getInputState();

  bb->setBackcolor(C_BLACK);
  bb->setTextcolor(keyState & (1 << INPUT_UP) ? C_GREEN : C_WHITE);
  bb->drawString(5,5,"KEY_UP");
  bb->setTextcolor(keyState & (1 << INPUT_DOWN) ? C_GREEN : C_WHITE);
  bb->drawString(5,25,"KEY_DOWN");
  bb->setTextcolor(keyState & (1 << INPUT_LEFT) ? C_GREEN : C_WHITE);
  bb->drawString(5,45,"KEY_LEFT");
  bb->setTextcolor(keyState & (1 << INPUT_RIGHT) ? C_GREEN : C_WHITE);
  bb->drawString(5,65,"KEY_RIGHT");

  bb->setTextcolor(keyState & (1 << INPUT_A) ? C_GREEN : C_WHITE);
  bb->drawString(5,85,"KEY_A");
  bb->setTextcolor(keyState & (1 << INPUT_B) ? C_GREEN : C_WHITE);
  bb->drawString(5,105,"KEY_B");

  bb->setTextcolor(keyState & (1 << INPUT_SELECT) ? C_GREEN : C_WHITE);
  bb->drawString(5,125,"KEY_SELECT");
  bb->setTextcolor(keyState & (1 << INPUT_START) ? C_GREEN : C_WHITE);
  bb->drawString(5,145,"KEY_START");

  bb->setTextcolor(keyState & (1 << INPUT_MENU) ? C_GREEN : C_WHITE);
  bb->drawString(5,165,"KEY_MENU");
  bb->setTextcolor(keyState & (1 << INPUT_BOOT) ? C_GREEN : C_WHITE);
  bb->drawString(5,185,"KEY_BOOT");
  bb->setTextcolor(keyState & (1 << INPUT_OPTION) ? C_GREEN : C_WHITE);
  bb->drawString(5,205,"KEY_OPTION");

  bb->updateDisplay();
}
