/*******************************************************************
 *  BLE Gas-Breath Analyzer - ESP32 + SGP30
 *
 *  ┌─ Stany maszyny pomiarowej ────────────────────────────────────┐
 *  │ 0  IDLE / anulowano / po getMeasurement                       │
 *  │ 1  „Przygotowanie” – czekamy aż czujnik się ustabilizuje      │
 *  │      - zmiany każdego parametru ±40 w oknie 2 s               │
 *  │ 2  „Dmuchaj” – aktywny pomiar:                                │
 *  │      • max 10 s, jeśli Δ+70 nie nastąpi  → od razu 4          │
 *  │      • jeżeli Δ≥70 (lub -70 w raw-parametrach)  → 3           │
 *  │ 3  „Analiza” – znowu czekamy na stabilizację jak w 1          │
 *  │ 4  „Gotowe” – wynik do bufora, status 4 do aplikacji          │
 *  └───────────────────────────────────────────────────────────────┘
 *
 *  BLE UUIDs:
 *    • Sygnał:    MY_DEVICE_SIGNATURE
 *    • Serwis:    0000abcd-0000-1000-8000-00805f9b34fb
 *    • Char.:     0000dcba-0000-1000-8000-00805f9b34fb
 *******************************************************************/
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_SGP30.h>

#define SDA_PIN 27
#define SCL_PIN 14
TwoWire myWire(0);
Adafruit_SGP30 sgp;

static const char* DEVICE_NAME = "MY_DEVICE_SIGNATURE";
static BLEUUID serviceUUID("0000abcd-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID   ("0000dcba-0000-1000-8000-00805f9b34fb");
BLEServer* pServer;
BLECharacteristic* pChar;
BLEAdvertising* pAdv;
Preferences prefs;

bool deviceConnected = false;

struct Sample { int tvoc, eco2, ethanol, h2; };
enum MeasState : uint8_t { IDLE=0, PREP=1, BLOW=2, ANALYSE=3, DONE=4 };

MeasState status = IDLE;
Sample baseline, cur;
unsigned long tState = 0;
bool bigChange = false;

struct Record { uint32_t idx; Sample s; };
Record fifo[20]; uint8_t fifoCnt = 0; uint32_t nextIdx = 1;
void pushFifo(const Sample& s){
  if(fifoCnt == 20){ memmove(&fifo[0], &fifo[1], (19)*sizeof(Record)); fifoCnt = 19; }
  fifo[fifoCnt++] = {nextIdx++, s};
  Serial.printf("[FIFO] Saved sample #%lu → TVOC=%d, eCO2=%d, Eth=%d, H2=%d\n",
    fifo[fifoCnt-1].idx, s.tvoc, s.eco2, s.ethanol, s.h2);
}


// 🔧 MAPPING – z wartości surowych na zakres [0–3000]
int mapEco2(int eco2){
  long v = eco2;
  if(v < 400) v = 400;
  if(v > 56000) v = 56000;
  return (v - 400) * 10000L / 55600;
}

int mapEthanol(int raw){
  if(raw > 18700) raw = 18700;
  if(raw < 12000) raw = 12000;
  return (18700 - raw) * 3000L / (18700 - 12000);
}

int mapH2(int raw){
  if(raw > 13800) raw = 13800;
  if(raw < 9000) raw = 9000;
  return (13800 - raw) * 3000L / (13800 - 9000);
}

#define STAB_WINDOW 500  // 0.5 sekundy
#define STAB_BUF_SIZE 10

struct TimedSample {
  unsigned long t;
  Sample s;
};

TimedSample stabBuf[STAB_BUF_SIZE];
uint8_t stabBufCnt = 0;


bool readSample(Sample& out){
  if(!sgp.IAQmeasure() || !sgp.IAQmeasureRaw()) return false;
  out.tvoc    = sgp.TVOC;
  out.eco2    = mapEco2(sgp.eCO2);
  out.ethanol = mapEthanol(sgp.rawEthanol);
  out.h2      = mapH2(sgp.rawH2);

  Serial.printf("[SGP] Raw: TVOC=%d, eCO2=%d, Eth=%d, H2=%d | Mapped: eCO2=%d, Eth=%d, H2=%d\n",
    sgp.TVOC, sgp.eCO2, sgp.rawEthanol, sgp.rawH2,
    out.eco2, out.ethanol, out.h2);

  return true;
}

bool stableRecent(){
  unsigned long now = millis();
  int minTvoc = 9999, maxTvoc = 0;
  int minEco2 = 9999, maxEco2 = 0;
  int minEth = 9999, maxEth = 0;
  int minH2  = 9999, maxH2  = 0;

  int valid = 0;

  for(int i = 0; i < STAB_BUF_SIZE; ++i){
    if(now - stabBuf[i].t <= STAB_WINDOW){
      const auto& s = stabBuf[i].s;
      if(s.tvoc < minTvoc) minTvoc = s.tvoc;
      if(s.tvoc > maxTvoc) maxTvoc = s.tvoc;

      if(s.eco2 < minEco2) minEco2 = s.eco2;
      if(s.eco2 > maxEco2) maxEco2 = s.eco2;

      if(s.ethanol < minEth) minEth = s.ethanol;
      if(s.ethanol > maxEth) maxEth = s.ethanol;

      if(s.h2 < minH2) minH2 = s.h2;
      if(s.h2 > maxH2) maxH2 = s.h2;

      valid++;
    }
  }

  if(valid < 3) return false; // zbyt mało danych

  return (maxTvoc - minTvoc <= 80) &&
         (maxEco2 - minEco2 <= 50) &&
         (maxEth  - minEth  <= 40) &&
         (maxH2   - minH2   <= 40);
}


inline bool stable(const Sample&a, const Sample&b){
  return abs(a.tvoc - b.tvoc) <= 40 &&
         abs(a.eco2 - b.eco2) <= 40 &&
         abs(a.ethanol - b.ethanol) <= 40 &&
         abs(a.h2 - b.h2) <= 40;
}
inline bool isBigUp(const Sample&base, const Sample&now){
  return (now.tvoc - base.tvoc) >= 70 ||
         (now.eco2 - base.eco2) >= 70 ||
         (base.ethanol - now.ethanol) >= 70 ||
         (base.h2 - now.h2) >= 70;
}

class SvrCb : public BLEServerCallbacks{
  void onConnect(BLEServer*, esp_ble_gatts_cb_param_t* p) override{
    deviceConnected = true;
    char a[18]; sprintf(a, "%02X:%02X:%02X:%02X:%02X:%02X",
      p->connect.remote_bda[0], p->connect.remote_bda[1], p->connect.remote_bda[2],
      p->connect.remote_bda[3], p->connect.remote_bda[4], p->connect.remote_bda[5]);
    prefs.begin("bt", false); prefs.putString("peer", a); prefs.end();
  }
  void onDisconnect(BLEServer*) override{ deviceConnected = false; }
};

class ChCb : public BLECharacteristicCallbacks{
  void onWrite(BLECharacteristic*c) override{
  DynamicJsonDocument doc(256);
  if(deserializeJson(doc, c->getValue()) != DeserializationError::Ok){
    Serial.println("[BLE] ❌ JSON deserialization error");
    return;
  }

  const char* req = doc["request_type"] | doc["requestType"] | "";
  Serial.printf("[BLE] 📥 Received request: %s\n", req);

  if(strcmp(req, "startMeasurement") == 0){
    status = PREP; tState = millis(); bigChange = false;
    readSample(baseline); cur = baseline;
    sendStatus(); Serial.println("[STATE] → PREP (1) – Waiting for stability");
  }
  else if(strcmp(req, "checkMeasurement") == 0){
    sendStatus(); Serial.println("[BLE] ✅ Sent status");
  }
  else if(strcmp(req, "getMeasurement") == 0 && status == DONE){
    sendData(cur); status = IDLE;
    Serial.println("[STATE] → IDLE (0) after getMeasurement");
  }
  else if(strcmp(req, "loadHistoryFromDevice") == 0){
    DynamicJsonDocument j(1024);
    auto arr = j.createNestedArray("history");
    for(uint8_t i = 0; i < fifoCnt; ++i){
      auto o = arr.createNestedObject();
      o["index"] = fifo[i].idx;
      o["TVOC"] = fifo[i].s.tvoc;
      o["eCO2"] = fifo[i].s.eco2;
      o["Ethanol"] = fifo[i].s.ethanol;
      o["H2"] = fifo[i].s.h2;
    }
    fifoCnt = 0;
    String s; serializeJson(j, s); c->setValue(s.c_str()); c->notify();
    Serial.printf("[BLE] 📤 Sent history (%d items)\n", fifoCnt);
  }
  else if(strcmp(req, "cancelMeasurement") == 0){
    status = IDLE; sendStatus(); Serial.println("[STATE] → IDLE (cancelled)");
  } else {
    Serial.println("[BLE] ⚠️ Unknown or invalid request");
  }
}
};

  void sendStatus(){
  DynamicJsonDocument j(64); j["status"] = status;
  String s; serializeJson(j, s);
  pChar->setValue(s.c_str()); pChar->notify();
  Serial.printf("[BLE] 🔄 Sent status: %d\n", status);
}

void sendData(const Sample&smp){
  DynamicJsonDocument j(128);
  auto d = j.createNestedObject("data");
  d["TVOC"] = smp.tvoc; d["eCO2"] = smp.eco2;
  d["Ethanol"] = smp.ethanol; d["H2"] = smp.h2;
  String s; serializeJson(j, s);
  pChar->setValue(s.c_str()); pChar->notify();
  Serial.printf("[BLE] 📤 Sent measurement → TVOC=%d, eCO2=%d, Eth=%d, H2=%d\n",
    smp.tvoc, smp.eco2, smp.ethanol, smp.h2);
}


void setup(){
  Serial.begin(115200); delay(500);
  pinMode(SDA_PIN, INPUT_PULLUP); pinMode(SCL_PIN, INPUT_PULLUP);
  myWire.begin(SDA_PIN, SCL_PIN, 100000); Wire = myWire;
  if(!sgp.begin(&myWire)){ Serial.println("SGP30 missing"); while(1); }
  sgp.setHumidity(0);

  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer(); pServer->setCallbacks(new SvrCb());
  auto svc = pServer->createService(serviceUUID);
  pChar = svc->createCharacteristic(charUUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY);
  pChar->setCallbacks(new ChCb());
  pChar->setValue("{\"status\":0}");
  svc->start();

  pAdv = BLEDevice::getAdvertising(); pAdv->addServiceUUID(serviceUUID);
  pAdv->setScanResponse(true); pAdv->start();
}

void loop(){
  unsigned long now = millis();
  Sample s;
  if(readSample(s)){
    cur = s;
    stabBuf[stabBufCnt % STAB_BUF_SIZE] = {millis(), cur};
    stabBufCnt++;
  }

  switch(status){
  case PREP:{
    static unsigned long stabT = 0;
    if(stableRecent()){
      if(stabT == 0) stabT = now;
      else if(now - stabT >= 2000){
        status = BLOW; tState = now; bigChange = false;
        pChar->setValue("{\"status\":2}"); pChar->notify();
        Serial.println("[STATE] → BLOW (2) – Start blowing");
      }
    } else stabT = 0;
  } break;

  case BLOW:{
    if(isBigUp(baseline, cur)){
      status = ANALYSE; tState = now;
      pChar->setValue("{\"status\":3}"); pChar->notify();
      Serial.println("[STATE] → ANALYSE (3) – Big change detected");
    } else if(now - tState >= 10000){
      status = DONE; pushFifo(cur);
      pChar->setValue("{\"status\":4}"); pChar->notify();
      Serial.println("[STATE] → DONE (4) – Timeout");
    }
  } break;

  case ANALYSE:{
    static unsigned long stabT = 0;
    if(stableRecent()){
      if(stabT == 0) stabT = now;
      else if(now - stabT >= 2000){
        status = DONE; pushFifo(cur);
        pChar->setValue("{\"status\":4}"); pChar->notify();
        Serial.println("[STATE] → DONE (4) – Stable after blow");
      }
    } else stabT = 0;
  } break;
}
}
