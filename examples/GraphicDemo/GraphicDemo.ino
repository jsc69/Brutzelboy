#include <Brutzelboy.h>

Brutzelboy *bb = nullptr;

float offset = 0.0f;

void setup() {
  Serial.begin(115200);
  delay(500);

  bb = new Brutzelboy(INIT_LCD);
  // Init Brutzelboy
  bb->begin();

  bb->setBrightness(255);
  bb->fillScreen(0);
  bb->updateDisplay();
}

void loop() {
  geometry_demo(0.2f, 6);
  bb->updateDisplay();
}

static inline uint16_t hsv_to_rgb565(float h) {
  // h ist in [0, 360]
  int hi = ((int)h / 60) % 6;
  float f = (h / 60.0f) - (int)(h / 60.0f);

  int r, g, b;

  // Für s=1, v=1 vereinfacht
  switch (hi) {
    case 0:
      r = 255;
      g = (int)(f * 255);
      b = 0;
      break;
    case 1:
      r = (int)((1.0f - f) * 255);
      g = 255;
      b = 0;
      break;
    case 2:
      r = 0;
      g = 255;
      b = (int)(f * 255);
      break;
    case 3:
      r = 0;
      g = (int)((1.0f - f) * 255);
      b = 255;
      break;
    case 4:
      r = (int)(f * 255);
      g = 0;
      b = 255;
      break;
    default:
      r = 255;
      g = 0;
      b = (int)((1.0f - f) * 255);
      break;
  }

  return RGB565(r, g, b);
}

void geometry_demo(const float speed, const int count) {

  bb->fillScreen(0);  // Display löschen (Schwarz)

  offset += speed;  // Zeit-Variable für die Bewegung

  for (int i = 0; i < count; i++) {
    // Wir nutzen Sinus/Cosinus für flüssige Bewegungen
    // Die Koordinaten dürfen bewusst außerhalb von 0-292 liegen!

    // Pulsierende Kreise (Zentrum)
    int16_t circ_x = 146 + (int16_t)(sin(offset * 0.5f + i) * 100.0f);
    int16_t circ_y = 120 + (int16_t)(cos(offset * 0.8f + i) * 80.0f);
    uint16_t color1 = hsv_to_rgb565(fmod(offset * 20.0f + i * 40.0f, 360.0f));
    bb->fillCircle(circ_x, circ_y, 20 + i * 3, color1);

    // Rotierende/Fliegende Dreiecke
    // x1, y1 ist die Spitze, x2, y2 die Basis-Ecke
    int16_t tri_x = 146 + (int16_t)(sin(offset + i) * 150.0f);
    int16_t tri_y = 120 + (int16_t)(cos(offset * 1.2f + i) * 130.0f);
    bb->fillTriangle(tri_x, tri_y, tri_x + 20, tri_y + 20, 15, 0xF800);  // Rot

    // Driftende Rechtecke (Hintergrund)
    int16_t rect_x = (int16_t)(fmod(i * 50.0f + offset * 30.0f, 400.0f)) - 50;
    bb->fillFrame(rect_x, 20 + i * 30, rect_x + 40, 50 + i * 30, 0x7BEF);  // Grau/Blau
  }
}
