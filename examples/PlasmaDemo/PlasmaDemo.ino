#include <Brutzelboy.h>
#include <display.h>

Brutzelboy *bb = nullptr;

const float f_min = 0.02f;
const float f_max = 0.50f;
const float f_step = 0.01f;
const float t_min = 0.1f;
const float t_max = 1.0f;
const float t_step = 0.1f;
const int step_min = 1;
const int step_max = 10;
const int step_step = 1;

float f_x = 0.03f;     // Dichte der Muster (horizontal)
float f_y = 0.03f;     // Dichte der Muster (vertikal)
float f_xy = 0.06f;    // Diagonale Komplexität (erzeugt die organischen Kurven)
float t_scale = 0.1f;  // Geschwindigkeit der Animation (Zeit-Skalierung)
int step = 2;          // Pixelschrittweite

float fps = 0.0f;

// 256-Einträge Sinus Lookup-Table für glatte Übergänge
static const float sin_table[256] = {
  0.000000f, 0.024541f, 0.049068f, 0.073565f, 0.098017f, 0.122411f, 0.146730f, 0.170962f,
  0.195090f, 0.219101f, 0.242980f, 0.266713f, 0.290285f, 0.313682f, 0.336890f, 0.359895f,
  0.382683f, 0.405241f, 0.427555f, 0.449611f, 0.471397f, 0.492898f, 0.514103f, 0.534998f,
  0.555570f, 0.575808f, 0.595699f, 0.615232f, 0.634393f, 0.653173f, 0.671559f, 0.689541f,
  0.707107f, 0.724247f, 0.740951f, 0.757209f, 0.773010f, 0.788346f, 0.803208f, 0.817585f,
  0.831470f, 0.844854f, 0.857729f, 0.870087f, 0.881921f, 0.893224f, 0.903989f, 0.914213f,
  0.923880f, 0.932993f, 0.941544f, 0.949528f, 0.956940f, 0.963776f, 0.970031f, 0.975702f,
  0.980785f, 0.985278f, 0.989177f, 0.992480f, 0.995185f, 0.997290f, 0.998795f, 0.999699f,
  1.000000f, 0.999699f, 0.998795f, 0.997290f, 0.995185f, 0.992480f, 0.989177f, 0.985278f,
  0.980785f, 0.975702f, 0.970031f, 0.963776f, 0.956940f, 0.949528f, 0.941544f, 0.932993f,
  0.923880f, 0.914213f, 0.903989f, 0.893224f, 0.881921f, 0.870087f, 0.857729f, 0.844854f,
  0.831470f, 0.817585f, 0.803208f, 0.788346f, 0.773010f, 0.757209f, 0.740951f, 0.724247f,
  0.707107f, 0.689541f, 0.671559f, 0.653173f, 0.634393f, 0.615232f, 0.595699f, 0.575808f,
  0.555570f, 0.534998f, 0.514103f, 0.492898f, 0.471397f, 0.449611f, 0.427555f, 0.405241f,
  0.382683f, 0.359895f, 0.336890f, 0.313682f, 0.290285f, 0.266713f, 0.242980f, 0.219101f,
  0.195090f, 0.170962f, 0.146730f, 0.122411f, 0.098017f, 0.073565f, 0.049068f, 0.024541f,
  0.000000f, -0.024541f, -0.049068f, -0.073565f, -0.098017f, -0.122411f, -0.146730f, -0.170962f,
  -0.195090f, -0.219101f, -0.242980f, -0.266713f, -0.290285f, -0.313682f, -0.336890f, -0.359895f,
  -0.382683f, -0.405241f, -0.427555f, -0.449611f, -0.471397f, -0.492898f, -0.514103f, -0.534998f,
  -0.555570f, -0.575808f, -0.595699f, -0.615232f, -0.634393f, -0.653173f, -0.671559f, -0.689541f,
  -0.707107f, -0.724247f, -0.740951f, -0.757209f, -0.773010f, -0.788346f, -0.803208f, -0.817585f,
  -0.831470f, -0.844854f, -0.857729f, -0.870087f, -0.881921f, -0.893224f, -0.903989f, -0.914213f,
  -0.923880f, -0.932993f, -0.941544f, -0.949528f, -0.956940f, -0.963776f, -0.970031f, -0.975702f,
  -0.980785f, -0.985278f, -0.989177f, -0.992480f, -0.995185f, -0.997290f, -0.998795f, -0.999699f,
  -1.000000f, -0.999699f, -0.998795f, -0.997290f, -0.995185f, -0.992480f, -0.989177f, -0.985278f,
  -0.980785f, -0.975702f, -0.970031f, -0.963776f, -0.956940f, -0.949528f, -0.941544f, -0.932993f,
  -0.923880f, -0.914213f, -0.903989f, -0.893224f, -0.881921f, -0.870087f, -0.857729f, -0.844854f,
  -0.831470f, -0.817585f, -0.803208f, -0.788346f, -0.773010f, -0.757209f, -0.740951f, -0.724247f,
  -0.707107f, -0.689541f, -0.671559f, -0.653173f, -0.634393f, -0.615232f, -0.595699f, -0.575808f,
  -0.555570f, -0.534998f, -0.514103f, -0.492898f, -0.471397f, -0.449611f, -0.427555f, -0.405241f,
  -0.382683f, -0.359895f, -0.336890f, -0.313682f, -0.290285f, -0.266713f, -0.242980f, -0.219101f,
  -0.195090f, -0.170962f, -0.146730f, -0.122411f, -0.098017f, -0.073565f, -0.049068f, -0.024541f
};

// Fast sin mit linearer Interpolation - 256 Einträge
static inline float fast_sin(float x) {
  // 256 / (2*PI) = 40.7436
  float scaled = x * 40.7436f;
  int idx = (int)scaled;
  float frac = scaled - (float)idx;

  int idx0 = idx & 0xFF;  // 256 = 0x100, Maske = 0xFF
  int idx1 = (idx + 1) & 0xFF;

  // Lineare Interpolation
  return sin_table[idx0] + (sin_table[idx1] - sin_table[idx0]) * frac;
}

// Fast cos mit linearer Interpolation
static inline float fast_cos(float x) {
  // Cos = Sin verschoben um PI/2 = 64 Einträge bei 256er Table
  float scaled = x * 40.7436f + 64.0f;
  int idx = (int)scaled;
  float frac = scaled - (float)idx;

  int idx0 = idx & 0xFF;
  int idx1 = (idx + 1) & 0xFF;

  return sin_table[idx0] + (sin_table[idx1] - sin_table[idx0]) * frac;
}

void drawInfo() {
  bb->setTextcolor(C_WHITE);
  bb->setBackcolor(C_BLACK);
  bb->setFont(&FONT_6X10);

  char buf[100];
  snprintf(buf, sizeof(buf), "x=%.2f,y=%.2f,xy=%.2f,s=%.1f,%.1ffps", f_x, f_y, f_xy, t_scale, fps);
  bb->drawString(5, 228, buf);
}

int frames_since_last_fps = 0;
int total_frames = 0;
uint64_t start_us = esp_timer_get_time();
void plasma_effect(const float f_x, const float f_y, const float f_xy, const float t_scale, const int step, const char *name) {

  bb->fillScreen(0);

  float time = total_frames * t_scale;

  for (int y = 0; y < SCREEN_HEIGHT; y += step) {
    // v2 nur einmal pro Zeile berechnen
    float v2 = fast_sin(y * f_y + time * 0.7f);

    for (int x = 0; x < SCREEN_WIDTH; x += step) {
      float v1 = fast_sin(x * f_x + time);
      float v3 = fast_sin((x + y) * f_xy + time * 1.3f);

      float plasma = (v1 + v2 + v3 + 3.0f) / 6.0f;
      float hue = plasma * 360.0f;

      uint16_t color = hsv_to_rgb565(hue);

      if (step > 1) {
        bb->fillFrame(x, y, x + step - 1, y + step - 1, color);
      } else {
        bb->drawPixel(x, y, color);
      }
    }
  }

  total_frames++;
  frames_since_last_fps++;
  uint64_t now = esp_timer_get_time();
  if (now - start_us >= 1000000) {
    fps = (float)frames_since_last_fps * 1000000.0f / (now - start_us);
    frames_since_last_fps = 0;
    start_us = now;
  }
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

// Handler für ButtonEvents
void handleButtons() {
  uint16_t keyState = bb->getInputState();

  if (keyState & (1 << INPUT_UP))
    if (f_x < f_max)
      f_x += f_step;

  if (keyState & (1 << INPUT_DOWN))
    if (f_x > f_min)
      f_x -= f_step;

  if (keyState & (1 << INPUT_LEFT))
    if (f_y > f_min)
      f_y -= f_step;


  if (keyState & (1 << INPUT_RIGHT))
    if (f_y < f_max)
      f_y += f_step;

  if (keyState & (1 << INPUT_MENU))
    if (t_scale > t_min)
      t_scale -= t_step;

  if (keyState & (1 << INPUT_OPTION))
    if (t_scale < t_max)
      t_scale += t_step;

  if (keyState & (1 << INPUT_A))
    if (f_xy > f_min)
      f_xy -= f_step;

  if (keyState & (1 << INPUT_B))
    if (f_xy < f_max)
      f_xy += f_step;
}


void setup() {
  Serial.begin(115200);
  delay(500);

  bb = new Brutzelboy(INIT_LCD | INIT_BUTTONS);
  // Init Brutzelboy
  bb->begin();

  bb->setBrightness(255);
  bb->fillScreen(0);
  bb->updateDisplay();
}

void loop() {
  // Gib den Brutzelboy seine Zeit
  bb->loop();

  handleButtons();

  plasma_effect(f_x, f_y, f_xy, t_scale, step, "Diagonal Sweep");
  drawInfo();
  bb->updateDisplay();
}
