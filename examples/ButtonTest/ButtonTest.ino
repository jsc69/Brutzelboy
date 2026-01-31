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

static int adc_up_down_value = 0;
static int adc_left_right_value = 0;

void loop() {
  keyState = bb->getInputState();
  char adcValue[255];

  bb->setBackcolor(C_BLACK);
  bb->setTextcolor(keyState & (1 << INPUT_UP) ? C_GREEN : C_WHITE);
  bb->drawString(5,5,"UP");
  sprintf(adcValue, "<%4d>", input_raw_y_value());
  bb->drawString(50, 5, adcValue);
  bb->setTextcolor(keyState & (1 << INPUT_DOWN) ? C_GREEN : C_WHITE);
  bb->drawString(5,25,"DOWN");
  bb->setTextcolor(keyState & (1 << INPUT_LEFT) ? C_GREEN : C_WHITE);
  bb->drawString(5,45,"LEFT");
  sprintf(adcValue, "<%4d>", input_raw_x_value());
  bb->drawString(50, 45, adcValue);
  bb->setTextcolor(keyState & (1 << INPUT_RIGHT) ? C_GREEN : C_WHITE);
  bb->drawString(5,65,"RIGHT");

  bb->setTextcolor(keyState & (1 << INPUT_A) ? C_GREEN : C_WHITE);
  bb->drawString(5,85,"A");
  bb->setTextcolor(keyState & (1 << INPUT_B) ? C_GREEN : C_WHITE);
  bb->drawString(5,105,"B");

  bb->setTextcolor(keyState & (1 << INPUT_SELECT) ? C_GREEN : C_WHITE);
  bb->drawString(5,125,"SELECT");
  bb->setTextcolor(keyState & (1 << INPUT_START) ? C_GREEN : C_WHITE);
  bb->drawString(5,145,"START");

  bb->setTextcolor(keyState & (1 << INPUT_MENU) ? C_GREEN : C_WHITE);
  bb->drawString(5,165,"MENU");
  bb->setTextcolor(keyState & (1 << INPUT_BOOT) ? C_GREEN : C_WHITE);
  bb->drawString(5,185,"BOOT");
  bb->setTextcolor(keyState & (1 << INPUT_OPTION) ? C_GREEN : C_WHITE);
  bb->drawString(5,205,"OPTION");

  bb->updateDisplay();
}
