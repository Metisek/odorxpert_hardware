#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "odor_graphics.h"
#include <Fonts/FreeSansBold18pt7b.h>

// Konfiguracja pinów ESP32 (30 pin)
#define TFT_CS    5
#define TFT_RST   21
#define TFT_DC    22
#define TFT_SCLK  18
#define TFT_MOSI  19

#define ENC_CLK   36  // GPIO36 - wejście tylko
#define ENC_DT    39  // GPIO39 - wejście tylko
#define ENC_SW    34  // GPIO34 - wejście tylko

#define ANALOG_PIN 35 // GPIO35 - ADC1_CH7

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

volatile bool showSensor = false;
unsigned long logoStartTime;
bool inSubMenu = false;

volatile int encoderPos = 0;
volatile int lastEncoderPos = 0;
volatile bool buttonPressed = false;
volatile unsigned long lastInterrupt = 0;

int currentMenu = 0;
const char* menuItems[] = {"Pomiar", "Menu 1", "Menu 2", "Menu 3"};

void IRAM_ATTR handleEncoder() {
  if (millis() - lastInterrupt < 5) return;
  static uint8_t oldState = 3;
  uint8_t newState = (digitalRead(ENC_DT) << 1) | digitalRead(ENC_CLK);
  if (newState == oldState) return;
  if ((oldState == 0x3 && newState == 0x2) || 
      (oldState == 0x2 && newState == 0x0) || 
      (oldState == 0x0 && newState == 0x1) || 
      (oldState == 0x1 && newState == 0x3)) {
    encoderPos++;
  } else {
    encoderPos--;
  }
  oldState = newState;
  lastInterrupt = millis();
}

void IRAM_ATTR handleButton() {
  if (millis() - lastInterrupt < 300) return;
  buttonPressed = true;
  lastInterrupt = millis();
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_SW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT), handleEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_SW), handleButton, FALLING);

  SPI.begin(TFT_SCLK, -1, TFT_MOSI);
  tft.init(240, 320);
  tft.fillScreen(tft.color565(0, 175, 185));
  drawCenteredLogo();
  logoStartTime = millis();
}

void loop() {
  if (!showSensor) {
    if (millis() - logoStartTime >= 2000) {
      tft.fillScreen(tft.color565(0, 175, 185));
      showSensor = true;
      drawMainMenu();
    }
    return;
  }

  if (!inSubMenu) {
    int newPos = encoderPos % 4;
    if (newPos < 0) newPos += 4;
    if (newPos != currentMenu) {
      currentMenu = newPos;
      drawMainMenu();
    }
  }

  if (buttonPressed) {
    buttonPressed = false;
    handleMenuAction();
  }

  if (inSubMenu && currentMenu == 0) {
    updateLiveMeasurement();
  }
}

void drawMainMenu() {
  inSubMenu = false;
  tft.fillScreen(tft.color565(0, 175, 185));
  tft.setFont(&FreeSansBold18pt7b);
  for (int i = 0; i < 4; i++) {
    if (i == currentMenu) {
      tft.setTextColor(ST77XX_YELLOW, tft.color565(0, 175, 185));
    } else {
      tft.setTextColor(ST77XX_WHITE, tft.color565(0, 175, 185));
    }
    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(menuItems[i], 0, 0, &x, &y, &w, &h);
    tft.setCursor((240 - w)/2, 80 + i*60);
    tft.print(menuItems[i]);
  }
}

void handleMenuAction() {
  inSubMenu = true;
  switch(currentMenu) {
    case 0:
      startLiveMeasurement();
      break;
    default:
      showMenuAction();
      break;
  }
}

void startLiveMeasurement() {
  tft.fillScreen(tft.color565(0, 175, 185));
  tft.setFont(&FreeSansBold18pt7b);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(50, 50);
  tft.print("Pomiar na zywo");
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(70, 280);
  tft.print("[BACK]");
}

void updateLiveMeasurement() {
  static unsigned long lastUpdate = 0;
  static int lastValue = -1;
  if (millis() - lastUpdate < 500) return;
  lastUpdate = millis();
  int sensorValue = analogRead(ANALOG_PIN);
  if (sensorValue != lastValue) {
    lastValue = sensorValue;
    tft.fillRect(50, 100, 140, 60, tft.color565(0, 175, 185));
    tft.setFont(&FreeSansBold18pt7b);
    tft.setTextColor(ST77XX_WHITE);
    String text = String(sensorValue);
    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(text, 0, 0, &x, &y, &w, &h);
    tft.setCursor((240 - w)/2, 150);
    tft.print(text);
  }
  if (buttonPressed) {
    buttonPressed = false;
    drawMainMenu();
  }
}

void showMenuAction() {
  tft.fillScreen(tft.color565(0, 175, 185));
  tft.setFont(&FreeSansBold18pt7b);
  tft.setTextColor(ST77XX_RED);
  String text = "Wybrano: " + String(menuItems[currentMenu]);
  int16_t x, y;
  uint16_t w, h;
  tft.getTextBounds(text, 0, 0, &x, &y, &w, &h);
  tft.setCursor((240 - w)/2, 120);
  tft.print(text);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(70, 280);
  tft.print("[BACK]");
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    if (buttonPressed) {
      buttonPressed = false;
      break;
    }
    delay(50);
  }
  drawMainMenu();
}

void drawCenteredLogo() {
  int startX = (240 - odor_w) / 2;
  int startY = (320 - odor_h) / 2;
  for (int y = 0; y < odor_h; y++) {
    for (int x = 0; x < odor_w; x++) {
      uint16_t color = pgm_read_word(&(odor[y * odor_w + x]));
      if (color != 0x0000) {
        tft.drawPixel(startX + x, startY + y, color);
      }
    }
  }
}
