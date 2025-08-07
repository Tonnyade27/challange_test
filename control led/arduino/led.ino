struct LED {
  uint8_t pin;
  unsigned long interval;
  unsigned long lastToggle;
  bool state;
};


LED leds[] = {
  {2, 270, 0, false},    // Merah
  {4, 440, 0, false},    // Kuning
  {5, 710, 0, false},    // Hijau
  {18, 1330, 0, false},  // Biru
  {19, 1850, 0, false}   // Putih
};

const int ledCount = sizeof(leds) / sizeof(LED);

void setup() {
  for (int i = 0; i < ledCount; i++) {
    pinMode(leds[i].pin, OUTPUT);
    digitalWrite(leds[i].pin, LOW);
    leds[i].lastToggle = millis();
  }
}

void loop() {
  unsigned long now = millis();
  for (int i = 0; i < ledCount; i++) {
    if (now - leds[i].lastToggle >= leds[i].interval) {
      leds[i].state = !leds[i].state;
      digitalWrite(leds[i].pin, leds[i].state);
      leds[i].lastToggle = now;
    }
  }
}
