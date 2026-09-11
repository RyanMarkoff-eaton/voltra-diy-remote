#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include "ProtocolData.h"
#include "ControllerUi.h"
#include "LocalConfig.h"

// Bench prototype: no Wi-Fi, no automatic resistance writes on connection.
// Deliberately keeps reported state separate from requested weight.
NimBLEClient* client = nullptr;
NimBLERemoteCharacteristic* writer = nullptr;
bool ready = false, enabled = false, polling = false;
// User-confirmed Voltra BLE address. Automatic connection performs only the
// protocol handshake and current-settings read; it never writes resistance.
constexpr const char* kVoltraAddress = VOLTRA_BLE_ADDRESS;
uint32_t lastAutoConnectAttempt = 0;
constexpr uint32_t kAutoConnectRetryMs = 10000;
constexpr uint32_t kAutoSleepMs = 15UL * 60UL * 1000UL;
Preferences preferences;
bool autoSleepEnabled=true;
uint32_t lastActivityAt=0;
uint32_t lastSettingsPollAt=0;
constexpr uint32_t kSettingsPollMs=5000;
// A requested activation and the Voltra's reported motor state are separate.
// Never infer that a successful BLE write engaged or released resistance.
bool activationRequested=false;
bool confirmedLoaded=false;
bool motorStateKnown=false;
bool dropEnabled=false, dropArmed=false;
int dropAmount=5, dropHoldSeconds=2;
volatile uint32_t lastRepAt=0, lastReturnAt=0;
// No invented startup weight. This remains unavailable until the Voltra sends
// BP_BASE_WEIGHT during the read-only settings synchronization.
int target = 0;
int requestedEccentric=0, requestedChains=0, requestedInverseChains=0;
bool eccentricEnabled=false, chainsEnabled=false, inverseChainsEnabled=false;
bool eccentricInitialized=false, chainsInitialized=false, inverseChainsInitialized=false;
volatile int confirmedWeight=-32768, confirmedEccentric=-32768;
volatile int confirmedChains=-32768, confirmedInverseChains=-32768;
volatile int confirmedMotorState=-32768;
volatile uint32_t weightConfirmedAt=0;
volatile uint32_t motorConfirmedAt=0;
int pendingWeightDelta=0;
bool targetInitialized=false;
uint32_t lastWeightDetentAt=0;
constexpr uint32_t kWeightFastTurnMs=120;
constexpr uint32_t kWeightVeryFastTurnMs=45;
uint32_t pollStart = 0, lastPoll = 0;
String line;
enum class ConfirmedMode : uint8_t { Unknown=255, Idle=0, WeightTraining=1,
  ResistanceBand=2, Rowing=3, Damper=4, CustomCurves=6, Isokinetic=7, Isometric=8 };
volatile ConfirmedMode confirmedMode = ConfirmedMode::Unknown;
volatile uint32_t modeConfirmedAt = 0;
enum class PendingAction : uint8_t { None, KnobWeight, KnobEccentric, KnobChains,
  KnobInverseChains, ButtonActivate, GuidedLoad, AutoDrop };
PendingAction pendingAction = PendingAction::None;
uint32_t modeQueryAt = 0;
enum class LoadStep : uint8_t { Idle, SettleAfterStop, StartAfterSetup };
LoadStep loadStep = LoadStep::Idle;
uint32_t loadStepAt = 0;
uint8_t crc8(const uint8_t* data, size_t n);
uint16_t crc16(const uint8_t* data, size_t n);
void processReceivedFrame(const uint8_t* data, size_t n);

int acceleratedWeightDelta(int detents) {
  const uint32_t now=millis();
  const uint32_t elapsed=lastWeightDetentAt ? now-lastWeightDetentAt : UINT32_MAX;
  lastWeightDetentAt=now;
  // A deliberate turn remains 1 lb/detent. Consecutive detents speed up to
  // 5 lb, then 10 lb, without affecting the precision of modifier controls.
  const int step=elapsed<=kWeightVeryFastTurnMs ? 10 :
                 elapsed<=kWeightFastTurnMs ? 5 : 1;
  return detents*step;
}

bool validFrame(const uint8_t* data, size_t n) {
  if (n < 5 || data[0] != 0x55) return false;
  const size_t expected=data[2]==0x09 ? size_t(0x100+data[1]) : size_t(data[1]);
  if(expected!=n) return false;
  if (crc8(data,3) != data[3]) return false;
  return crc16(data,n-2) == (uint16_t(data[n-1])<<8 | data[n-2]);
}

void updateConfirmedMode(uint8_t raw) {
  switch(raw) {
    case 0: case 1: case 2: case 3: case 4: case 6: case 7: case 8:
      confirmedMode=static_cast<ConfirmedMode>(raw); modeConfirmedAt=millis();
      Serial.printf("DEVICE CONFIRMED training_mode=%u%s\n",raw,
                    raw==1?" (Weight Training)":"");
      break;
    default: confirmedMode=ConfirmedMode::Unknown; modeConfirmedAt=0; break;
  }
}

void decodeMode(const uint8_t* data, size_t n) {
  // Voltra's legacy 0x55/0x2E settings cascade has the same parameter
  // layout as cmd 0x10, but no inner command byte. It is used for settings
  // changed from the Voltra itself, including inverse chains.
  const bool legacySettingsUpdate=n==46 && data[0]==0x55 && data[1]==0x2e;
  if (!legacySettingsUpdate && !validFrame(data,n)) return;
  // Response to our one-register cmd 0x0F read. The device-to-app envelope
  // reverses 0xAA/0x10 and returns <count=1><reserved><id LE><uint8 value>.
  if(n==19 && data[4]==0x10 && data[5]==0xaa && data[10]==0x0f &&
     data[12]==1 && data[13]==0 && data[14]==0xb0 && data[15]==0x4f)
    updateConfirmedMode(data[16]);
  // General cmd 0x0F register-read response. Values follow count/reserved as
  // <id LE><value>, with widths defined by the pinned protocol catalog.
  if(n>18 && data[4]==0x10 && data[5]==0xaa && data[10]==0x0f) {
    uint16_t count=uint16_t(data[12])|(uint16_t(data[13])<<8); size_t at=14;
    for(uint16_t i=0;i<count && at+2<n-2;i++) {
      uint16_t id=uint16_t(data[at]) | uint16_t(data[at+1])<<8; at+=2;
      size_t width=(id==0x3e86 || id==0x3e87 || id==0x3e88 || id==0x3e89)?2:1;
      if(at+width>n-2) return;
      if(id==0x4fb0) updateConfirmedMode(data[at]);
      else if(id==0x3e86) {
        int value=uint16_t(data[at])|(uint16_t(data[at+1])<<8);
        if(value>=5 && value<=230) {
          confirmedWeight=value; weightConfirmedAt=millis();
          // A read response represents the current device value; use it as
          // the controller's target instead of retaining an old local value.
          target=value; targetInitialized=true;
          Serial.printf("DEVICE CONFIRMED base_weight=%d lb; display synchronized\n",value);
        }
      }
      else if(id==0x3e87) {
        confirmedChains=uint16_t(data[at])|(uint16_t(data[at+1])<<8);
        if(!chainsInitialized) { requestedChains=confirmedChains; chainsEnabled=confirmedChains!=0; chainsInitialized=true; }
      }
      else if(id==0x3e88) {
        confirmedEccentric=int16_t(uint16_t(data[at])|(uint16_t(data[at+1])<<8));
        if(!eccentricInitialized) { requestedEccentric=confirmedEccentric; eccentricEnabled=confirmedEccentric!=0; eccentricInitialized=true; }
      }
      else if(id==0x3e89) {
        confirmedMotorState=uint16_t(data[at])|(uint16_t(data[at+1])<<8);
        motorConfirmedAt=millis(); motorStateKnown=true;
        // The tested single-device strength flow reports 0 unloaded and 1
        // loaded. Do not treat a request value (4 or 5) as confirmation.
        if(confirmedMotorState==0) confirmedLoaded=false;
        else if(confirmedMotorState==1) confirmedLoaded=true;
        Serial.printf("DEVICE CONFIRMED motor_state=%d loaded=%d\n",
                      int(confirmedMotorState),confirmedLoaded);
      }
      else if(id==0x53b0) {
        confirmedInverseChains=data[at];
        // 0x53B0 is a profile selector (0=Chains, 1=Inverse), not a pound
        // value. The shared amount lives in 0x3E87.
        if(!inverseChainsInitialized) {
          inverseChainsEnabled=confirmedInverseChains==1 && confirmedChains>0;
          inverseChainsInitialized=true;
        }
        if(chainsInitialized && confirmedInverseChains==1) {
          requestedInverseChains=confirmedChains; inverseChainsEnabled=confirmedChains!=0;
          chainsEnabled=false; inverseChainsInitialized=true;
        }
      }
      at+=width;
    }
  }
  // cmd 0x10/settings update: count,reserved,<param-id LE><value>.
  if (n > 15 && (data[10] == 0x10 || legacySettingsUpdate)) {
    uint16_t count=uint16_t(data[11])|(uint16_t(data[12])<<8); size_t at=13;
    for(uint16_t i=0;i<count && at+2<n-2;i++) {
      uint16_t id=uint16_t(data[at]) | uint16_t(data[at+1])<<8; at+=2;
      size_t width=(id==0x3e86 || id==0x3e87 || id==0x3e88 || id==0x3e89)?2:1;
      if(at+width>n-2) return;
      if(id==0x4fb0) updateConfirmedMode(data[at]);
      else if(id==0x3e86) {
        int value=uint16_t(data[at])|(uint16_t(data[at+1])<<8);
        if(value>=5 && value<=230) {
          confirmedWeight=value; weightConfirmedAt=millis();
          // Voltra publishes this when its own controls change the setting.
          // Redraw the center value from the newly confirmed device state.
          target=value; targetInitialized=true;
          Serial.printf("DEVICE CONFIRMED base_weight=%d lb; display synchronized\n",value);
        }
      }
      else if(id==0x3e87) {
        confirmedChains=uint16_t(data[at])|(uint16_t(data[at+1])<<8);
        if(!chainsInitialized && confirmedInverseChains!=1) {
          requestedChains=confirmedChains; chainsEnabled=confirmedChains!=0; chainsInitialized=true;
        }
        if(!inverseChainsInitialized && confirmedInverseChains==1) {
          requestedInverseChains=confirmedChains;
          inverseChainsEnabled=confirmedChains!=0; inverseChainsInitialized=true;
        }
      }
      else if(id==0x3e88) confirmedEccentric=int16_t(uint16_t(data[at])|(uint16_t(data[at+1])<<8));
      else if(id==0x3e89) {
        confirmedMotorState=uint16_t(data[at])|(uint16_t(data[at+1])<<8);
        motorConfirmedAt=millis(); motorStateKnown=true;
        if(confirmedMotorState==0) confirmedLoaded=false;
        else if(confirmedMotorState==1) confirmedLoaded=true;
      }
      else if(id==0x53b0) confirmedInverseChains=data[at];
      at+=width;
    }
  }
  // Vendor state dump: payload byte zero is the active training mode.
  if(n>=52 && data[10]==0xaa && data[11]==0x80 && data[12]==0x25)
    updateConfirmedMode(data[13]);
}

void decodeDropSetTelemetry(const uint8_t* data, size_t n) {
  // Pinned SDK vendor per-rep frame: cmd 0xAA at byte 10, subtype 0x82/0x3B,
  // 74-byte frame; phase at byte 13 (1=pull, 2=return).
  if(!validFrame(data,n) || n<74 || data[10]!=0xaa || data[11]!=0x82 || data[12]!=0x3b) return;
  const uint32_t now=millis();
  lastRepAt=now;
  if(data[13]==2) {
    lastReturnAt=now;
    dropArmed=dropEnabled && activationRequested;
    Serial.printf("DROP SET: return detected; %ds hold armed\n",dropHoldSeconds);
  } else if(dropArmed) {
    dropArmed=false;
    if(pendingAction==PendingAction::AutoDrop) pendingAction=PendingAction::None;
    Serial.println("DROP SET: new rep detected; countdown cancelled");
  }
}

void processReceivedFrame(const uint8_t* data, size_t n) {
  // Read replies (cmd 0x0F) are controller-initiated polling and must not
  // keep the remote awake. Published settings/state events are Voltra activity.
  if(validFrame(data,n) && (data[1]==0x2e || data[10]==0x10 ||
      (data[10]==0xaa && data[11]==0x80 && data[12]==0x25))) lastActivityAt=millis();
  decodeMode(data,n);
  decodeDropSetTelemetry(data,n);
  Serial.print("RX ");
  for (size_t i = 0; i < n; ++i) Serial.printf("%02x", data[i]);
  Serial.println();
}
void received(NimBLERemoteCharacteristic*, uint8_t* data, size_t n, bool) {
  // Notifications are a byte stream in practice: a frame may be fragmented or
  // several frames can arrive together. Only dispatch complete short frames.
  static uint8_t buffer[512]; static size_t used=0;
  if(n>sizeof(buffer)-used) used=0;
  memcpy(buffer+used,data,n); used+=n;
  while(used) {
    size_t start=0; while(start<used && buffer[start]!=0x55) ++start;
    if(start) { memmove(buffer,buffer+start,used-start); used-=start; }
    if(used<3) return;
    const size_t frameLength=buffer[2]==0x09 ? size_t(0x100+buffer[1]) : size_t(buffer[1]);
    if(frameLength<5 || frameLength>sizeof(buffer)) { memmove(buffer,buffer+1,--used); continue; }
    if(used<frameLength) return;
    if(validFrame(buffer,frameLength)) processReceivedFrame(buffer,frameLength);
    else Serial.println("RX discarded: invalid frame");
    memmove(buffer,buffer+frameLength,used-frameLength); used-=frameLength;
  }
}

bool writePacket(const uint8_t* data, size_t n) {
  if (!client || !client->isConnected() || !writer) return false;
  // Match SDK write-with-response; never split protocol frames arbitrarily.
  if (client->getMTU() < n + 3) {
    Serial.println("ERROR: negotiated MTU too small");
    return false;
  }
  bool ok = writer->writeValue(data, n, true);
  if (!ok) {
    enabled = false; activationRequested=false; confirmedLoaded=false; motorStateKnown=false;
    polling = false;
    Serial.println("ERROR: write failed; command outcome unknown; writes disabled");
  }
  return ok;
}
template<size_t N> bool send(const uint8_t (&p)[N]) { return writePacket(p, N); }

uint8_t crc8(const uint8_t* data, size_t n) {
  uint8_t c = 0x77;
  while (n--) { c ^= *data++; for (int i=0;i<8;i++) c=(c>>1)^((c&1)?0x8c:0); }
  return c;
}
uint16_t crc16(const uint8_t* data, size_t n) {
  uint16_t c = 0x3692;
  while (n--) { c ^= *data++; for (int i=0;i<8;i++) c=(c>>1)^((c&1)?0x8408:0); }
  return c;
}
void pollStatus() {
  uint8_t p[] = {0x55,23,4,0,0xaa,0x10,0,0x20,0x20,0,0x0f,4,0,
                 0x8d,0x53,0xc7,0x53,0xc8,0x53,0xc9,0x53,0,0};
  p[3] = crc8(p,3);
  uint16_t c = crc16(p,sizeof(p)-2);
  p[21]=c&255; p[22]=c>>8;
  writePacket(p,sizeof(p));
}
bool sendWeight(int pounds) {
  if(pounds<5 || pounds>230) return false;
  // Standard BP_BASE_WEIGHT (0x3E86) uint16 setter. Sequence mirrors the
  // upstream table: 0x2000 + (pounds - 5). Values 201..230 are the explicit
  // beta extension requested for this device; 5..200 reproduce SDK packets.
  uint16_t sequence=0x2000+uint16_t(pounds-5);
  uint8_t p[] = {0x55,19,4,0,0xaa,0x10,uint8_t(sequence),uint8_t(sequence>>8),
                 0x20,0,0x11,1,0,0x86,0x3e,uint8_t(pounds),uint8_t(pounds>>8),0,0};
  p[3]=crc8(p,3); uint16_t c=crc16(p,sizeof(p)-2); p[17]=c&255; p[18]=c>>8;
  return writePacket(p,sizeof(p));
}
bool sendModifier(uint16_t wireId,int value,int minimum,int maximum) {
  if(value<minimum || value>maximum) return false;
  uint16_t sequence=0x2000+uint16_t(value-minimum);
  uint16_t encoded=uint16_t(int16_t(value));
  uint8_t p[] = {0x55,19,4,0,0xaa,0x10,uint8_t(sequence),uint8_t(sequence>>8),
                 0x20,0,0x11,1,0,uint8_t(wireId>>8),uint8_t(wireId),
                 uint8_t(encoded),uint8_t(encoded>>8),0,0};
  p[3]=crc8(p,3); uint16_t c=crc16(p,sizeof(p)-2); p[17]=c&255; p[18]=c>>8;
  return writePacket(p,sizeof(p));
}
bool sendChainDirection(bool inverse) {
  // 0x53b0 is a uint8 direction selector. It is not a two-byte pound value.
  const uint16_t sequence=0x2000+(inverse?1:0);
  uint8_t p[] = {0x55,18,4,0,0xaa,0x10,uint8_t(sequence),uint8_t(sequence>>8),
                 0x20,0,0x11,1,0,0xb0,0x53,uint8_t(inverse?1:0),0,0};
  p[3]=crc8(p,3); const uint16_t c=crc16(p,sizeof(p)-2); p[16]=c&255; p[17]=c>>8;
  return writePacket(p,sizeof(p));
}
bool sendInverseChains(int value, bool enabled) {
  // Retain the proven chain-value command and correct only the inverse
  // direction field. Clearing the value disables the shared chain effect.
  if(!enabled) return sendChainDirection(false) && sendModifier(0x873e,0,0,100);
  return sendChainDirection(true) && sendModifier(0x873e,value,0,100);
}
void queryTrainingMode(bool includeWeight=false) {
  uint8_t p[] = {0x55,19,4,0,0xaa,0x10,0,0x20,0x20,0,0x0f,2,0,
                 0xb0,0x4f,0x86,0x3e,0,0};
  size_t length=includeWeight?sizeof(p):17;
  if(!includeWeight) { p[1]=17; p[11]=1; }
  p[3]=crc8(p,3); uint16_t c=crc16(p,length-2);
  p[length-2]=c&255; p[length-1]=c>>8;
  // Set the request timestamp before writeValue: a notification can arrive
  // synchronously before writeValue returns on a fast connection.
  modeQueryAt=millis();
  if(!writePacket(p,length)) pendingAction=PendingAction::None;
}
void queryCurrentSettings() {
  uint8_t p[] = {0x55,27,4,0,0xaa,0x10,0,0x20,0x20,0,0x0f,6,0,
                 0xb0,0x4f,0x86,0x3e,0x87,0x3e,0x88,0x3e,0x89,0x3e,
                 0xb0,0x53,0,0};
  p[3]=crc8(p,3); uint16_t c=crc16(p,sizeof(p)-2);
  p[25]=c&255; p[26]=c>>8;
  modeQueryAt=millis();
  lastSettingsPollAt=modeQueryAt;
  writePacket(p,sizeof(p));
}
void cleanup() {
  ready = enabled = polling = false; activationRequested=false; confirmedLoaded=false; motorStateKnown=false;
  dropArmed=false; lastRepAt=lastReturnAt=0;
  confirmedMode=ConfirmedMode::Unknown; modeConfirmedAt=0;
  confirmedWeight=confirmedEccentric=confirmedChains=confirmedInverseChains=confirmedMotorState=-32768;
  weightConfirmedAt=motorConfirmedAt=0; pendingWeightDelta=0; target=0; targetInitialized=false; lastWeightDetentAt=0;
  eccentricInitialized=chainsInitialized=inverseChainsInitialized=false;
  pendingAction=PendingAction::None; loadStep=LoadStep::Idle;
  writer = nullptr;
  if (client) { client->disconnect(); NimBLEDevice::deleteClient(client); client=nullptr; }
}
void connectDevice(String address) {
  if (client && client->isConnected()) { Serial.println("Already connected"); return; }
  cleanup();
  auto scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  Serial.println("Scanning 5 seconds...");
  auto results = scan->getResults(5000, false);
  const NimBLEAdvertisedDevice* chosen = nullptr;
  for (int i=0;i<results.getCount();i++) {
    auto d = results.getDevice(i);
    if (!d->isAdvertisingService(NimBLEUUID(protocol::serviceUuid))) continue;
    Serial.printf("VOLTRA %s %s\n", d->getAddress().toString().c_str(), d->getName().c_str());
    if (address.equalsIgnoreCase(d->getAddress().toString().c_str())) chosen=d;
  }
  if (!chosen) { Serial.println("Use: connect <listed-address>"); return; }
  client=NimBLEDevice::createClient();
  client->setConnectTimeout(10000);
  if (!client->connect(chosen)) { cleanup(); Serial.println("Connect failed"); return; }
  auto service=client->getService(protocol::serviceUuid);
  writer=service?service->getCharacteristic(protocol::writeCharUuid):nullptr;
  auto notify=service?service->getCharacteristic(protocol::notifyCharUuid):nullptr;
  if (!writer || !writer->canWrite() || !notify || !notify->canNotify() ||
      !notify->subscribe(true,received)) { cleanup(); Serial.println("GATT setup failed"); return; }
  if (writer->canNotify() && !writer->subscribe(true,received)) {
    cleanup(); Serial.println("Write-channel subscription failed"); return;
  }
  if (!send(protocol::auth)) { cleanup(); return; }
  delay(3000);
  if (!send(protocol::init0)) { cleanup(); return; }
  delay(20);
  if (!send(protocol::init1)) { cleanup(); return; }
  delay(20);
  ready=client->isConnected();
  Serial.println("Handshake sent. Device acceptance still requires hardware verification.");
  Serial.println("No weight applied. Reading the current Voltra weight before controls become available.");
  if(ready) queryCurrentSettings(); // read-only synchronization; never writes resistance
}
void command(String s) {
  s.trim();
  if (s=="status") {
    Serial.printf("UI ready=%d PSRAM=%u\n",uiReady(),ESP.getPsramSize());
    Serial.printf("BLE connected=%d handshake_sent=%d writes_enabled=%d requested_lb=%d motor=%d known=%d loaded=%d\n",
                  client && client->isConnected(), ready, enabled, target,
                  int(confirmedMotorState),motorStateKnown,confirmedLoaded);
    Serial.printf("requested modifiers: eccentric=%d chains=%d inverse_chains=%d lb; confirmed=%d/%d/%d/%d\n",
                  requestedEccentric,requestedChains,requestedInverseChains,
                  int(confirmedWeight),int(confirmedEccentric),int(confirmedChains),int(confirmedInverseChains));
    Serial.printf("modifier enabled: eccentric=%d chains=%d inverse_chains=%d\n",
                  eccentricEnabled,chainsEnabled,inverseChainsEnabled);
    Serial.printf("drop_sets enabled=%d amount=%d lb hold=%ds armed=%d\n",
                  dropEnabled,dropAmount,dropHoldSeconds,dropArmed);
    Serial.printf("confirmed_training_mode=%u mode_age_ms=%lu pending_action=%u load_step=%u\n",
                  unsigned(confirmedMode), modeConfirmedAt?millis()-modeConfirmedAt:0,
                  unsigned(pendingAction),unsigned(loadStep));
    return;
  }
  if (s=="scan") { connectDevice(""); return; }
  if (s.startsWith("connect ")) { connectDevice(s.substring(8)); return; }
  if (s=="disconnect") { cleanup(); return; }
  if (s.startsWith("target ")) {
    String number=s.substring(7);
    if (!number.length() || number.length()>3) { Serial.println("Range: 5..230 lb"); return; }
    for (unsigned i=0;i<number.length();i++) if (!isDigit(number[i])) { Serial.println("Integer required"); return; }
    int v=number.toInt();
    if (v<5 || v>230) { Serial.println("Range: 5..230 lb"); return; }
    target=v; targetInitialized=true; Serial.printf("Requested target: %d lb (not sent)\n",target); return;
  }
  if (s=="stop") {
    polling=false; enabled=false; activationRequested=false; dropArmed=false; motorStateKnown=false;
    bool stopped=send(protocol::stop); if(stopped) queryCurrentSettings();
    Serial.println(stopped?"STOP written; awaiting unloaded device report":"STOP failed; use Voltra controls");
    return;
  }
  if (!ready || !client || !client->isConnected()) { Serial.println("Connect first"); return; }
  if (s=="enable") { enabled=true; Serial.println("Bench writes enabled until stop/disconnect/error"); return; }
  if (!enabled) { Serial.println("Use enable before weight/load commands"); return; }
  if (polling) { Serial.println("Load polling in progress; use stop"); return; }
  if (s=="weight") {
    if(!targetInitialized) { Serial.println("Current Voltra weight has not been confirmed yet"); return; }
    if (sendWeight(target)) Serial.println("Weight written; check Voltra display");
  } else if (s=="load") {
    if(!targetInitialized) { Serial.println("Current Voltra weight has not been confirmed yet"); return; }
    // Start from a known unloaded cable. This command releases tension first.
    if (!send(protocol::stop)) return;
    delay(500); // bench settling delay, must be validated on the real device
    if (!sendWeight(target)) return;
    if (!send(protocol::trigger)) return;
    activationRequested=true;
    polling=true; pollStart=lastPoll=millis();
    queryCurrentSettings();
    Serial.println("Guided-load trigger written. RX contains raw status; activation unconfirmed.");
  } else Serial.println("Commands: scan, connect MAC, target N, enable, weight, load, stop, disconnect");
}
void setup() {
  Serial.begin(115200);
  uiBegin();
  NimBLEDevice::init("voltraRemote");
  NimBLEDevice::setMTU(247);
  delay(1500);
  preferences.begin("voltra",false);
  autoSleepEnabled=preferences.getBool("auto_sleep",true);
  lastActivityAt=millis();
  Serial.println("Standalone BLE bench test. Commands: scan, connect MAC, target N, enable, weight, load, stop, disconnect");
  // Direct connection to the configured Voltra. This does not send a weight,
  // GO, trigger, or cable-position command.
  lastAutoConnectAttempt=millis();
  connectDevice(kVoltraAddress);
}
void loop() {
  // Keep the standalone controller usable after the Voltra is switched on,
  // moves back into range, or disconnects. Each retry is read-only after the
  // BLE handshake; activation always requires a fresh touchscreen action.
  if((!client || !client->isConnected()) && millis()-lastAutoConnectAttempt>=kAutoConnectRetryMs) {
    lastAutoConnectAttempt=millis();
    Serial.println("BLE offline: retrying configured Voltra connection");
    connectDevice(kVoltraAddress);
  }
  if(uiTakeDropToggle()) {
    lastActivityAt=millis();
    dropEnabled=!dropEnabled;
    dropArmed=false;
    Serial.printf("DROP SETS %s; amount=%d lb hold=%ds\n",dropEnabled?"ON":"OFF",dropAmount,dropHoldSeconds);
  }
  if(uiTakeSleepToggle()) {
    lastActivityAt=millis();
    autoSleepEnabled=!autoSleepEnabled;
    preferences.putBool("auto_sleep",autoSleepEnabled);
    Serial.printf("AUTO SLEEP %s: 15 minutes idle\n",autoSleepEnabled?"ON":"OFF");
  }
  UiSelection toggled=uiTakeModifierToggle();
  if(toggled!=UiSelection::None) {
    lastActivityAt=millis();
    if(toggled==UiSelection::Eccentric) {
      eccentricEnabled=!eccentricEnabled;
      // A zero value is the Voltra's disabled setting. Give a newly enabled
      // modifier a small, visible starting value; later OFF/ON cycles retain
      // the user's dial-selected value.
      if(eccentricEnabled && requestedEccentric==0) requestedEccentric=5;
    }
    else if(toggled==UiSelection::Chains) {
      chainsEnabled=!chainsEnabled;
      if(chainsEnabled) inverseChainsEnabled=false;
      if(chainsEnabled && requestedChains==0) requestedChains=5;
    }
    else {
      inverseChainsEnabled=!inverseChainsEnabled;
      if(inverseChainsEnabled) chainsEnabled=false;
      // A newly enabled inverse-chain setting cannot use zero: zero is the
      // Voltra's explicit disabled value. Subsequent toggles retain the value.
      if(inverseChainsEnabled && requestedInverseChains==0) requestedInverseChains=5;
    }
    const char* name=toggled==UiSelection::Eccentric?"eccentric":
      toggled==UiSelection::Chains?"chains":"inverse chains";
    bool on=toggled==UiSelection::Eccentric?eccentricEnabled:
      toggled==UiSelection::Chains?chainsEnabled:inverseChainsEnabled;
    Serial.printf("Touch toggled %s %s; saved value retained\n",name,on?"ON":"OFF");
    if(loadStep!=LoadStep::Idle || polling) {
      Serial.println("Modifier toggle local only: guided load in progress");
    } else if(ready && client && client->isConnected()) {
      pendingAction=toggled==UiSelection::Eccentric?PendingAction::KnobEccentric:
        toggled==UiSelection::Chains?PendingAction::KnobChains:PendingAction::KnobInverseChains;
      queryTrainingMode();
    } else Serial.println("Modifier toggle local only: BLE disconnected");
  }
  int knobDelta=uiTakeEncoderDelta();
  UiDropEdit dropEdit=uiDropEdit();
  if(knobDelta && dropEdit!=UiDropEdit::None) {
    lastActivityAt=millis();
    if(dropEdit==UiDropEdit::Amount) dropAmount=constrain(dropAmount+knobDelta,1,100);
    else dropHoldSeconds=constrain(dropHoldSeconds+knobDelta,1,10);
    Serial.printf("DROP SETS %s=%d\n",dropEdit==UiDropEdit::Amount?"amount_lb":"hold_seconds",
                  dropEdit==UiDropEdit::Amount?dropAmount:dropHoldSeconds);
    knobDelta=0;
  }
  if(knobDelta) {
    lastActivityAt=millis();
    UiSelection selection=uiSelection();
    if(selection==UiSelection::None) {
      Serial.println("Dial ignored: tap LBS or a modifier label to select what to adjust");
      return;
    }
    if(selection==UiSelection::Weight) {
      const int weightDelta=acceleratedWeightDelta(knobDelta);
      if(ready && client && client->isConnected()) {
        pendingWeightDelta=constrain(pendingWeightDelta+weightDelta,-225,225);
        if(confirmedWeight>=5 && confirmedWeight<=230) {
          target=constrain(int(confirmedWeight)+pendingWeightDelta,5,230);
          targetInitialized=true;
        }
      } else if(targetInitialized) target=constrain(target+weightDelta,5,230);
      Serial.printf("Weight dial: detents=%d applied_step=%d requested=%d lb\n",
                    knobDelta,knobDelta?abs(weightDelta/knobDelta):0,target);
    }
    else if(selection==UiSelection::Eccentric) requestedEccentric=constrain(requestedEccentric+knobDelta,-195,195);
    else if(selection==UiSelection::Chains) requestedChains=constrain(requestedChains+knobDelta,0,100);
    else requestedInverseChains=constrain(requestedInverseChains+knobDelta,0,100);
    Serial.printf("Knob requested: weight=%d delta=%d eccentric=%d chains=%d inverse=%d lb; checking device mode\n",
                  target,pendingWeightDelta,requestedEccentric,requestedChains,requestedInverseChains);
    if((loadStep!=LoadStep::Idle)||polling) {
      Serial.println("Knob change local only: guided load in progress");
    } else if(selection!=UiSelection::Weight &&
              !((selection==UiSelection::Eccentric && eccentricEnabled) ||
                (selection==UiSelection::Chains && chainsEnabled) ||
                (selection==UiSelection::InverseChains && inverseChainsEnabled))) {
      Serial.println("Knob change saved locally: selected modifier is OFF; tap row to enable");
    } else if(ready && client && client->isConnected()) {
      pendingAction=selection==UiSelection::Weight?PendingAction::KnobWeight:
        selection==UiSelection::Eccentric?PendingAction::KnobEccentric:
        selection==UiSelection::Chains?PendingAction::KnobChains:PendingAction::KnobInverseChains;
      queryTrainingMode(selection==UiSelection::Weight);
    } else Serial.println("Knob change local only: BLE disconnected");
  }
  UiButtonAction button=uiTakeButtonAction();
  if(button!=UiButtonAction::None) lastActivityAt=millis();
  if(button==UiButtonAction::Stop) {
    pendingAction=PendingAction::None; loadStep=LoadStep::Idle; polling=false; enabled=false; activationRequested=false; dropArmed=false; motorStateKnown=false;
    bool stopped=send(protocol::stop); if(stopped) queryCurrentSettings();
    Serial.println(stopped?"Double-click STOP written; awaiting unloaded device report":"STOP failed; use Voltra controls");
  } else if(button==UiButtonAction::ApplyWeight && (activationRequested || confirmedLoaded)) {
    pendingAction=PendingAction::None; loadStep=LoadStep::Idle; polling=false;
    activationRequested=false; enabled=false; dropArmed=false; motorStateKnown=false;
    bool stopped=send(protocol::stop); if(stopped) queryCurrentSettings();
    Serial.println(stopped
      ?"Center tap STOP written; awaiting unloaded device report"
      :"Center tap STOP failed; use Voltra controls");
  } else if(button==UiButtonAction::ApplyWeight || button==UiButtonAction::GuidedLoad) {
    if(!ready || !client || !client->isConnected()) {
      Serial.println("Button action not sent: BLE disconnected");
    } else if(loadStep!=LoadStep::Idle || polling) {
      Serial.println("Button action ignored: guided load in progress; double-click to STOP");
    } else {
      pendingAction=button==UiButtonAction::ApplyWeight?PendingAction::ButtonActivate:PendingAction::GuidedLoad;
      Serial.printf(button==UiButtonAction::ApplyWeight
        ?"Button click: checking mode before activating %d lb\n"
        :"Button hold: checking mode before guided load at %d lb\n",target);
      // Include the actual base weight: no activation is allowed from a local default.
      queryTrainingMode(true);
    }
  }
  // A mode response must follow this knob event/query. Never authorize from a
  // boot/reconnect report, a stale report, or a locally requested mode.
  if(dropEnabled && activationRequested && dropArmed && lastReturnAt && lastRepAt==lastReturnAt &&
     pendingAction==PendingAction::None && loadStep==LoadStep::Idle && !polling &&
     millis()-lastReturnAt>=uint32_t(dropHoldSeconds)*1000) {
    dropArmed=false;
    pendingAction=PendingAction::AutoDrop;
    Serial.printf("DROP SET: %ds elapsed after return; reading current weight before decrease\n",dropHoldSeconds);
    queryTrainingMode(true);
  }
  bool needsFreshWeight=pendingAction==PendingAction::KnobWeight ||
    pendingAction==PendingAction::AutoDrop ||
    pendingAction==PendingAction::ButtonActivate ||
    pendingAction==PendingAction::GuidedLoad;
  bool freshWeightForAction=!needsFreshWeight ||
    (targetInitialized && weightConfirmedAt>=modeQueryAt && millis()-weightConfirmedAt<1500);
  if(pendingAction!=PendingAction::None && freshWeightForAction &&
     modeConfirmedAt>=modeQueryAt && millis()-modeConfirmedAt<1500) {
    PendingAction action=pendingAction; pendingAction=PendingAction::None;
    if(confirmedMode==ConfirmedMode::WeightTraining) {
      if(action==PendingAction::GuidedLoad) {
        if(send(protocol::stop)) {
          loadStep=LoadStep::SettleAfterStop; loadStepAt=millis();
          Serial.println("Guided load: STOP written; cancellable 500 ms settle started");
        }
      } else if(action==PendingAction::ButtonActivate) {
        if(sendWeight(target) && send(protocol::workoutSetup)) {
          loadStep=LoadStep::StartAfterSetup; loadStepAt=millis();
          Serial.printf("Touch activation: weight %d lb and SETUP written; cancellable 300 ms settle started\n",target);
        }
      } else if(action==PendingAction::AutoDrop) {
        int lowered=constrain(int(confirmedWeight)-dropAmount,5,230);
        if(lowered==confirmedWeight) {
          Serial.println("DROP SET: already at minimum weight; no write sent");
        } else if(sendWeight(lowered)) {
          target=lowered; targetInitialized=true;
          Serial.printf("DROP SET: weight %d -> %d lb written; cable position command was not sent\n",
                        int(confirmedWeight),lowered);
        }
      } else {
        bool sent=false; const char* setting="weight"; int value=target;
        if(action==PendingAction::KnobEccentric) { setting="eccentric"; value=eccentricEnabled?requestedEccentric:0; sent=sendModifier(0x883e,value,-195,195); }
        else if(action==PendingAction::KnobChains) {
          setting="chains"; value=chainsEnabled?requestedChains:0;
          sent=(confirmedInverseChains==1 ? sendChainDirection(false) : true) && sendModifier(0x873e,value,0,100);
        }
        else if(action==PendingAction::KnobInverseChains) { setting="inverse chains"; value=inverseChainsEnabled?requestedInverseChains:0; sent=sendInverseChains(value,inverseChainsEnabled); }
        else {
          target=constrain(int(confirmedWeight)+pendingWeightDelta,5,230);
          pendingWeightDelta=0; targetInitialized=true; value=target;
          sent=sendWeight(target);
        }
        if(sent) Serial.printf("Requested %s %d lb written: awaiting device confirmation\n",setting,value);
      }
    } else {
      Serial.printf("Action not sent: confirmed mode=%u is not Weight Training\n",unsigned(confirmedMode));
    }
  } else if(pendingAction!=PendingAction::None && millis()-modeQueryAt>1500) {
    pendingAction=PendingAction::None;
    pendingWeightDelta=0;
    Serial.println("Action not sent: no fresh mode confirmation");
  }
  if(loadStep==LoadStep::SettleAfterStop && millis()-loadStepAt>=500) {
    loadStep=LoadStep::Idle;
    if(sendWeight(target) && send(protocol::trigger)) {
      activationRequested=true;
      polling=true; pollStart=lastPoll=millis();
      queryCurrentSettings();
      Serial.println("Guided-load trigger written. Activation remains device-unconfirmed; double-click to STOP.");
    }
  }
  if(loadStep==LoadStep::StartAfterSetup && millis()-loadStepAt>=300) {
    loadStep=LoadStep::Idle;
    if(send(protocol::workoutGo)) {
      activationRequested=true;
      queryCurrentSettings();
      Serial.printf("Normal workout GO written at %d lb; awaiting motor-state report\n",target);
    }
  }
  // Some firmware versions omit inverse chains from their pushed settings
  // cascade. Periodically read the canonical register list while safely idle.
  // This is read-only and intentionally does not reset the idle-sleep timer.
  if(ready && client && client->isConnected() && !activationRequested && !confirmedLoaded &&
     !polling && loadStep==LoadStep::Idle && pendingAction==PendingAction::None &&
     millis()-lastSettingsPollAt>=kSettingsPollMs) {
    queryCurrentSettings();
  }
  uiTick(target,requestedEccentric,requestedChains,requestedInverseChains,
         int(confirmedWeight),int(confirmedEccentric),int(confirmedChains),int(confirmedInverseChains),
         eccentricEnabled,chainsEnabled,inverseChainsEnabled,
         confirmedLoaded,dropEnabled,dropAmount,dropHoldSeconds,dropArmed,autoSleepEnabled,
         client && client->isConnected());
  if (ready && (!client || !client->isConnected())) {
    ready=enabled=polling=false; activationRequested=false; confirmedLoaded=false; motorStateKnown=false; dropArmed=false; writer=nullptr;
    confirmedMode=ConfirmedMode::Unknown; modeConfirmedAt=0;
    confirmedWeight=confirmedEccentric=confirmedChains=confirmedInverseChains=confirmedMotorState=-32768;
    weightConfirmedAt=motorConfirmedAt=0; pendingWeightDelta=0; target=0; targetInitialized=false;
    pendingAction=PendingAction::None; loadStep=LoadStep::Idle;
    Serial.println("Disconnected. Motor state unknown. No automatic replay or reconnect.");
  }
  while (Serial.available()) {
    char c=Serial.read();
    if (c=='\n' || c=='\r') { if(line.length()) { lastActivityAt=millis(); command(line); line=""; } }
    else if(line.length()<80) line+=c;
  }
  if(polling) {
    if(millis()-pollStart>=18000) {
      polling=false;
      Serial.println("Status polling ended; this does NOT unload the cable. Use stop.");
    } else if(millis()-lastPoll>=500) { lastPoll=millis(); pollStatus(); }
  }
  // Deep sleep is only allowed while this controller is safely idle. It never
  // runs during activation, a load sequence, or a pending Voltra request.
  if(autoSleepEnabled && !activationRequested && !confirmedLoaded && motorStateKnown && !dropArmed &&
     pendingAction==PendingAction::None && loadStep==LoadStep::Idle && !polling &&
     millis()-lastActivityAt>=kAutoSleepMs) {
    Serial.println("AUTO SLEEP: 15 minutes idle; entering deep sleep");
    uiPowerDown();
    cleanup();
    delay(100);
    esp_deep_sleep_start();
  }
  delay(2);
}
