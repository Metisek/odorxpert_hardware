//  ======================== BIBLIOTEKI =======================
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "odor_graphics.h"
#include <Fonts/FreeSansBold18pt7b.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_SGP30.h>

// 🔄 ================== KONFIGURACJA SPRZĘTOWA ==================
#define TFT_CS    5
#define TFT_RST   21
#define TFT_DC    22
#define TFT_SCLK  18
#define TFT_MOSI  19
#define ANALOG_PIN 35 // GPIO35 - ADC1 bateria
#define SDA_PIN 27
#define SCL_PIN 14
#define ENC_CLK   36  // wejście tylko
#define ENC_DT    39  // wejście tylko
#define ENC_SW    34  // wejście tylko

TwoWire myWire(0);
Adafruit_SGP30 sgp;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

bool screenNeedsUpdate = true;

// 📶 ================== KONFIGURACJA BLE ========================
static const char* DEVICE_NAME = "MY_DEVICE_SIGNATURE";
static BLEUUID serviceUUID("0000abcd-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID   ("0000dcba-0000-1000-8000-00805f9b34fb");
BLEServer* pServer;
BLECharacteristic* pChar;
BLEAdvertising* pAdv;
Preferences prefs;
bool deviceConnected = false;

// 📊 ================== STRUKTURY DANYCH ========================
struct Sample { 
  int tvoc; 
  int eco2; 
  int ethanol; 
  int h2; 
  bool valid = false;
};

enum MeasState : uint8_t { 
  IDLE = 0, 
  PREP = 1, 
  BLOW = 2, 
  ANALYSE = 3, 
  DONE = 4 
};


//  ================== STAN ENKODERA ===========================

volatile int encoderDelta = 0;        // Zmiana względna
volatile int lastClkState = HIGH;     // Poprzedni stan CLK
volatile bool buttonPressed = false;
volatile unsigned long lastEncoderInterrupt = 0;
volatile unsigned long lastButtonInterrupt = 0;

// ================== FUNKCJE POMIARU =========================

void IRAM_ATTR handleEncoder() {
  unsigned long now = millis();
  if (now - lastEncoderInterrupt < 3) return;

  int clkState = digitalRead(ENC_CLK);
  int dtState = digitalRead(ENC_DT);

  if (clkState != lastClkState) {
    if (dtState != clkState) {
      encoderDelta++;
    } else {
      encoderDelta--;
    }
  }
  
  lastClkState = clkState;
  lastEncoderInterrupt = now;
}

void IRAM_ATTR handleButton() {
  unsigned long now = millis();
  if (now - lastButtonInterrupt < 300) return; // debounce
  buttonPressed = true;
  lastButtonInterrupt = now;
}

// Funkcja pomocnicza do odczytu zmian
int getEncoderDelta() {
  noInterrupts();
  int delta = encoderDelta;
  encoderDelta = 0;
  interrupts();
  return delta;
}

void encoder_setup() {

  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT, INPUT);
  pinMode(ENC_SW, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_CLK), handleEncoder, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENC_SW), handleButton, FALLING);
}

// 🔁 ================== STAN POMIARU ============================
MeasState measurementState = IDLE;       // Aktualny stan
Sample baseline;                         // Linia bazowa
Sample currentMeasurement;               // Bieżący pomiar (dla wyświetlacza)
Sample measurementResult;                // Wynik końcowy (dla wyświetlacza)
unsigned long tState = 0;                // Czas zmiany stanu
bool bigChange = false;                  // Flaga dużej zmiany

// ================== ZMIENNE INTERFEJSU =======================
unsigned long logoStartTime;
bool inSubMenu = false;
int currentMenu = 0;
const char* menuItems[] = {"Live Measurement", "History"};
int historySelection = 0;
int lastHistoryPos = 0;
unsigned long lastDisplayUpdate = 0;
MeasState lastMeasurementState = IDLE;
Sample lastSample = {0};

// 📈 ================== HISTORIA POMIARÓW =======================
struct Record { 
  uint32_t idx; 
  Sample s; 
};
Record fifo[20];
uint8_t fifoCnt = 0;
uint32_t nextIdx = 1;
#define STAB_SAMPLES 5     // Wymagane próbki

void pushFifo(const Sample& s) {
  if(fifoCnt >= 20) {
    // Przesuń bufor (usuń najstarszy)
    memmove(&fifo[0], &fifo[1], (fifoCnt-1)*sizeof(Record));
    fifoCnt--;
  }
  
  fifo[fifoCnt] = {nextIdx++, s};
  fifoCnt++;
  
  Serial.printf("[FIFO] Saved #%lu → TVOC:%d eCO2:%d Eth:%d H2:%d\n",
                fifo[fifoCnt-1].idx, s.tvoc, s.eco2, s.ethanol, s.h2);
}

// 📐 ============== FUNKCJE POMOCNICZE POMIARU ===================
int mapEco2(int eco2) {
  eco2 = constrain(eco2, 400, 56000);
  return map(eco2, 400, 56000, 0, 10000);
}

int mapEthanol(int raw) {
  raw = constrain(raw, 12000, 18700);
  return map(raw, 12000, 18700, 3000, 0);
}

int mapH2(int raw) {
  raw = constrain(raw, 9000, 13800);
  return map(raw, 9000, 13800, 0, 3000);
}

// 📡 ================== OBSŁUGA BLE ============================
class SvrCb : public BLEServerCallbacks {
  void onConnect(BLEServer*, esp_ble_gatts_cb_param_t* p) override {
    deviceConnected = true;
    char a[18]; 
    sprintf(a, "%02X:%02X:%02X:%02X:%02X:%02X",
            p->connect.remote_bda[0], p->connect.remote_bda[1], p->connect.remote_bda[2],
            p->connect.remote_bda[3], p->connect.remote_bda[4], p->connect.remote_bda[5]);
            
    prefs.begin("bt", false); 
    prefs.putString("peer", a); 
    prefs.end();
    
    Serial.printf("[BLE] Connected: %s\n", a);
  }
  
  void onDisconnect(BLEServer*) override { 
    deviceConnected = false; 
    Serial.println("[BLE] Disconnected");
  }
};

void sendStatus() {
  DynamicJsonDocument doc(64);
  doc["status"] = measurementState;
  
  String output;
  serializeJson(doc, output);
  pChar->setValue(output.c_str());
  
  if(deviceConnected) {
    pChar->notify();
    Serial.printf("[BLE] Sent status: %d\n", measurementState);
  }
}

void sendData(const Sample& smp) {
  DynamicJsonDocument doc(128);
  JsonObject data = doc.createNestedObject("data");
  data["TVOC"] = smp.tvoc; 
  data["eCO2"] = smp.eco2;
  data["Ethanol"] = smp.ethanol; 
  data["H2"] = smp.h2;
  
  String output;
  serializeJson(doc, output);
  pChar->setValue(output.c_str());
  
  if(deviceConnected) {
    pChar->notify();
    Serial.printf("[BLE] Sent data: TVOC=%d, eCO2=%d, Eth=%d, H2=%d\n",
                 smp.tvoc, smp.eco2, smp.ethanol, smp.h2);
  }
}

class ChCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    DynamicJsonDocument doc(256);
    deserializeJson(doc, c->getValue());
    
    const char* req = doc["request_type"] | doc["requestType"] | "";
    Serial.printf("[BLE] Received: %s\n", req);
    
    if(strcmp(req, "startMeasurement") == 0) {
      measurementState = PREP; 
      tState = millis(); 
      bigChange = false;
      baseline = currentMeasurement;
    
      // Automatyczne przejście do ekranu pomiaru
      if (!inSubMenu || currentMenu != 0) {
        currentMenu = 0;
        inSubMenu = true;
        screenNeedsUpdate = true;
      }
      
      sendStatus();
    }
    else if(strcmp(req, "checkMeasurement") == 0) {
      sendStatus();
    }
    else if(strcmp(req, "getMeasurement") == 0 && measurementState == DONE) {
      sendData(measurementResult);
      measurementState = IDLE;
      sendStatus();
    }
    else if(strcmp(req, "loadHistoryFromDevice") == 0) {
      DynamicJsonDocument j(1024);
      JsonArray arr = j.createNestedArray("history");
      
      for(uint8_t i = 0; i < fifoCnt; i++) {
        JsonObject o = arr.createNestedObject();
        o["index"] = fifo[i].idx;
        o["TVOC"] = fifo[i].s.tvoc;
        o["eCO2"] = fifo[i].s.eco2;
        o["Ethanol"] = fifo[i].s.ethanol;
        o["H2"] = fifo[i].s.h2;
      }
      
      String s;
      serializeJson(j, s);
      c->setValue(s.c_str());
      
      if(deviceConnected) {
        c->notify();
        Serial.printf("[BLE] Sent history (%d items)\n", fifoCnt);
      }
      
      fifoCnt = 0;  // Wyczyść historię po wysłaniu
    }
    else if(strcmp(req, "cancelMeasurement") == 0) {
      measurementState = IDLE; 
      sendStatus();
    }
  }
};

void setup_ble() {
  delay(100);
  
  // Inicjalizacja czujnika
  myWire.begin(SDA_PIN, SCL_PIN, 100000); 
  Wire = myWire;
  
  if(!sgp.begin(&myWire)) { 
    Serial.println("Sensor error!"); 
    while(1); 
  }
  
  sgp.setHumidity(0);  // Brak czujnika wilgotności
  
  // Inicjalizacja BLE
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new SvrCb());
  
  BLEService* svc = pServer->createService(serviceUUID);
  pChar = svc->createCharacteristic(
      charUUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY);
      
  pChar->setCallbacks(new ChCb());
  pChar->setValue("{\"status\":0}");  // Stan początkowy
  svc->start();

  pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(serviceUUID);
  pAdv->setScanResponse(true);
  pAdv->start();
  
  Serial.println("BLE ready!");
}

// 📊 ================== FUNKCJE POMIAROWE =======================
bool readSensor(Sample& out) {
  if(!sgp.IAQmeasure() || !sgp.IAQmeasureRaw()) {
    out.valid = false;
    return false;
  }
  
  out.tvoc    = sgp.TVOC;
  out.eco2    = mapEco2(sgp.eCO2);
  out.ethanol = mapEthanol(sgp.rawEthanol);
  out.h2      = mapH2(sgp.rawH2);
  out.valid   = true;
  
  return true;
}

#define STAB_WINDOW 500    // Okno stabilności 500ms
#define STAB_SAMPLES 5     // Wymagane próbki
Sample stabHistory[STAB_SAMPLES];
uint8_t stabIdx = 0;

bool isStable() {
  int minTvoc = INT_MAX, maxTvoc = 0;
  int minEco2 = INT_MAX, maxEco2 = 0;
  int minEth = INT_MAX, maxEth = 0;
  int minH2  = INT_MAX, maxH2  = 0;
  
  int validCount = 0;
  
  for(int i = 0; i < STAB_SAMPLES; i++) {
    if(!stabHistory[i].valid) continue;
    
    Sample s = stabHistory[i];
    
    minTvoc = min(minTvoc, s.tvoc);
    maxTvoc = max(maxTvoc, s.tvoc);
    
    minEco2 = min(minEco2, s.eco2);
    maxEco2 = max(maxEco2, s.eco2);
    
    minEth = min(minEth, s.ethanol);
    maxEth = max(maxEth, s.ethanol);
    
    minH2 = min(minH2, s.h2);
    maxH2 = max(maxH2, s.h2);
    
    validCount++;
  }
  
  if(validCount < 3) return false;  // Za mało próbek
  
  return (maxTvoc - minTvoc <= 80) &&
         (maxEco2 - minEco2 <= 50) &&
         (maxEth  - minEth  <= 40) &&
         (maxH2   - minH2   <= 40);
}

bool isBigChange(const Sample& base, const Sample& current) {
  return (current.tvoc - base.tvoc) >= 70 ||
         (current.eco2 - base.eco2) >= 70 ||
         (base.ethanol - current.ethanol) >= 70 ||
         (base.h2 - current.h2) >= 70;
}

// 🕒 ================== MASZYNA STANÓW ==========================
void measurementTick() {
  static unsigned long lastRead = 0;
  const unsigned long READ_INTERVAL = 20;  // 50Hz
  pinMode(ANALOG_PIN, INPUT); // Dodajemy inicjalizację ADC
  lastClkState = digitalRead(ENC_CLK); // Inicjalizacja stanu 
  
  if(millis() - lastRead < READ_INTERVAL) return;
  lastRead = millis();
  
  // Odczytaj dane z czujnika
  Sample newSample;
  if(readSensor(newSample)) {
    currentMeasurement = newSample;  // Aktualizuj globalną zmienną
    
    // Aktualizuj bufor stabilności
    stabHistory[stabIdx] = newSample;
    stabIdx = (stabIdx + 1) % STAB_SAMPLES;
  }
}

void measurementProcess() {
  if(!currentMeasurement.valid) return;
  
  switch(measurementState) {
    case PREP:
      if(isStable()) {
        if(millis() - tState > 2000) {  // Stabilne przez 2s
          measurementState = BLOW;
          tState = millis();
          bigChange = false;
          sendStatus();
        }
      } else {
        tState = millis();  // Resetuj timer przy niestabilności
      }
      break;
      
    case BLOW:
      if(isBigChange(baseline, currentMeasurement)) {
        measurementState = ANALYSE;
        tState = millis();
        bigChange = true;
        sendStatus();
      } 
      else if(millis() - tState >= 10000) {  // Timeout 10s
        measurementState = DONE;
        measurementResult = currentMeasurement;  // Zapisz wynik
        pushFifo(currentMeasurement);
        sendStatus();
      }
      break;
      
    case ANALYSE:
      if(isStable()) {
        if(millis() - tState > 2000) {  // Stabilne przez 2s
          measurementState = DONE;
          measurementResult = currentMeasurement;  // Zapisz wynik
          pushFifo(currentMeasurement);
          sendStatus();
        }
      } else {
        tState = millis();  // Resetuj timer
      }
      break;
      
    case DONE:
    case IDLE:
    default:
      // Brak akcji
      break;
  }
}

// ================== FUNKCJE INTERFEJSU =======================
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

void drawMainMenu() {
  tft.fillScreen(tft.color565(0, 175, 185));
  tft.setFont(&FreeSansBold18pt7b);
  
  for (int i = 0; i < 2; i++) {
    if (i == currentMenu) {
      tft.setTextColor(ST77XX_YELLOW);
    } else {
      tft.setTextColor(ST77XX_WHITE);
    }
    
    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(menuItems[i], 0, 0, &x, &y, &w, &h);
    tft.setCursor((240 - w) / 2, 100 + i * 60);
    tft.print(menuItems[i]);
  }
}

void handleMenuAction() {
  inSubMenu = true;
  screenNeedsUpdate = true;
  switch(currentMenu) {
    case 0: // Live Measurement
      measurementState = PREP;
      lastMeasurementState = IDLE;
      drawLiveMeasurementScreen();
      break;
    case 1: // History
      historySelection = fifoCnt > 0 ? fifoCnt - 1 : 0;
      lastHistoryPos = -1;
      drawHistoryScreen();
      break;
  }
}

void drawLiveMeasurementScreen() {
  if (measurementState == BLOW) {
    tft.fillScreen(ST77XX_RED);
  } else {
    tft.fillScreen(tft.color565(0, 175, 185));
  }

  tft.setFont(&FreeSansBold18pt7b);

  // Dodaj ADC value w prawym górnym rogu
  int adcValue = analogRead(ANALOG_PIN);
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(200, 10);
  tft.print("ADC:");
  tft.print(adcValue);
  
  // Stan pomiaru
  const char* stateText = "";
  switch(measurementState) {
    case IDLE: stateText = "READY"; break;
    case PREP: stateText = "PREPARING"; break;
    case BLOW: stateText = "BLOW NOW!"; break;
    case ANALYSE: stateText = "ANALYZING"; break;
    case DONE: stateText = "DONE"; break;
  }
  
  tft.setTextColor(ST77XX_WHITE);
  int16_t x, y;
  uint16_t w, h;
  tft.getTextBounds(stateText, 0, 0, &x, &y, &w, &h);
  tft.setCursor((240 - w) / 2, 50);
  tft.print(stateText);

  // Wartości pomiarowe
  if (currentMeasurement.valid) {
    char buffer[50];
    sprintf(buffer, "TVOC: %d", currentMeasurement.tvoc);
    tft.getTextBounds(buffer, 0, 0, &x, &y, &w, &h);
    tft.setCursor((240 - w) / 2, 110);
    tft.print(buffer);
    
    sprintf(buffer, "eCO2: %d", currentMeasurement.eco2);
    tft.getTextBounds(buffer, 0, 0, &x, &y, &w, &h);
    tft.setCursor((240 - w) / 2, 150);
    tft.print(buffer);
  }

  // Przycisk powrotu
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(80, 280);
  tft.print("[BACK]");
}

void drawHistoryScreen() {
  tft.fillScreen(tft.color565(0, 175, 185));
  tft.setFont(&FreeSansBold18pt7b);

  // Nagłówek
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(70, 40);
  tft.print("HISTORY");

  if (fifoCnt == 0) {
    tft.setCursor(30, 120);
    tft.print("NO DATA");
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(80, 280);
    tft.print("[BACK]");
    return;
  }

  // Wyświetlanie pomiarów
  int startIdx = max(0, historySelection - 2);
  int endIdx = min(fifoCnt - 1, historySelection + 2);
  
  int yPos = 80;
  for (int i = startIdx; i <= endIdx; i++) {
    if (i == historySelection) {
      tft.setTextColor(ST77XX_YELLOW);
    } else {
      tft.setTextColor(ST77XX_WHITE);
    }
    
    char buffer[20];
    sprintf(buffer, "#%lu: TVOC %d", fifo[i].idx, fifo[i].s.tvoc);
    
    int16_t x, y;
    uint16_t w, h;
    tft.getTextBounds(buffer, 0, 0, &x, &y, &w, &h);
    tft.setCursor((240 - w) / 2, yPos);
    tft.print(buffer);
    
    yPos += 30;
  }

  // Przycisk powrotu
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(80, 280);
  tft.print("[BACK]");
}



// ================== FUNKCJE SETUP I LOOP =====================
void setup() {
  Serial.begin(115200);
  encoder_setup();
  setup_ble();
  
  SPI.begin(TFT_SCLK, -1, TFT_MOSI);
  tft.init(240, 320);
  tft.setRotation(2);
  tft.fillScreen(tft.color565(0, 175, 185));
  drawCenteredLogo();
  logoStartTime = millis();
}


void loop() {
  static unsigned long lastTick = 0;
  if (millis() - lastTick >= 20) {
    measurementTick();
    measurementProcess();
    lastTick = millis();
  }

  // Nowa obsługa enkodera:
  int delta = getEncoderDelta();
  if (delta != 0) {
    if (!inSubMenu) {
      currentMenu = (currentMenu + delta) % 2;
      if (currentMenu < 0) currentMenu = 1;
      screenNeedsUpdate = true;
    } else if (inSubMenu && currentMenu == 1) {
      if (fifoCnt > 0) {
        historySelection = (historySelection + delta) % fifoCnt;
        if (historySelection < 0) historySelection = fifoCnt - 1;
        screenNeedsUpdate = true;
      }
    }
  }


    // Sprawdzanie czy wymagana aktualizacja ekranu
  if (screenNeedsUpdate) {
    if (!inSubMenu) {
      drawMainMenu();
    } else if (currentMenu == 0) {
      drawLiveMeasurementScreen();
    } else if (currentMenu == 1) {
      drawHistoryScreen();
    }
    screenNeedsUpdate = false;
    lastDisplayUpdate = millis();
  }

  // Obsługa przycisku
  if (buttonPressed) {
    buttonPressed = false;
    
    if (inSubMenu) {
      inSubMenu = false;
      sendStatus();
      drawMainMenu();
    } else {
      handleMenuAction();
    }
  }

  // Aktualizacja ekranu pomiaru
  if (inSubMenu && currentMenu == 0) {
    if (measurementState != lastMeasurementState || 
        !sampleEqual(currentMeasurement, lastSample)) {
      drawLiveMeasurementScreen();
      lastMeasurementState = measurementState;
      lastSample = currentMeasurement;
    }
  }
}

// Pomocnicza funkcja do porównywania próbek
bool sampleEqual(const Sample& a, const Sample& b) {
  return a.tvoc == b.tvoc && 
         a.eco2 == b.eco2 && 
         a.ethanol == b.ethanol && 
         a.h2 == b.h2;
}
