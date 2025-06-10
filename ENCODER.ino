#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Fonts/FreeSansBold18pt7b.h>

// Piny TFT
#define TFT_CS    5
#define TFT_RST   21
#define TFT_DC    22
#define TFT_SCLK  18
#define TFT_MOSI  19

// Piny enkodera
#define ENC_CLK   36  // wejście tylko
#define ENC_DT    39  // wejście tylko
#define ENC_SW    34  // wejście tylko

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

volatile int encoderPos = 0;
volatile bool buttonPressed = false;
volatile unsigned long lastEncoderInterrupt = 0;
volatile unsigned long lastButtonInterrupt = 0;

void IRAM_ATTR handleEncoder() {
  unsigned long now = millis();
  if (now - lastEncoderInterrupt < 3) return; // debounce

  int clkState = digitalRead(ENC_CLK);
  int dtState = digitalRead(ENC_DT);

  if (dtState != clkState) {
    encoderPos++;
  } else {
    encoderPos--;
  }

  lastEncoderInterrupt = now;
}

void IRAM_ATTR handleButton() {
  unsigned long now = millis();
  if (now - lastButtonInterrupt < 300) return; // debounce
  buttonPressed = true;
  lastButtonInterrupt = now;
}

void setup() {
  Serial.begin(115200);

  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_SW, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_CLK), handleEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_SW), handleButton, FALLING);

  SPI.begin(TFT_SCLK, -1, TFT_MOSI);
  tft.init(240, 320);
  tft.setRotation(0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setFont(&FreeSansBold18pt7b);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextWrap(false);
}

int lastDisplayed = -9999;

void loop() {
  static unsigned long lastUpdate = 0;

  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();

    if (encoderPos != lastDisplayed) {
      lastDisplayed = encoderPos;
      tft.fillScreen(ST77XX_BLACK);
      tft.setCursor(40, 150);
      tft.print("Licznik:");
      tft.setCursor(80, 240);
      tft.setTextColor(ST77XX_YELLOW);
      tft.print(encoderPos);
      tft.setTextColor(ST77XX_WHITE);
    }
  }

  if (buttonPressed) {
    buttonPressed = false;
    showButtonMessage();
  }
}

void showButtonMessage() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_GREEN);
  tft.setCursor(30, 140);
  tft.print("Wcisnieto");
  tft.setCursor(30, 200);
  tft.print("przycisk!");
  delay(1000);
  lastDisplayed = -9999; // wymusza odświeżenie
}
