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
bool activationTriggered=false;
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
// Chains and inverse chains are one device feature: a shared percentage plus
// a direction selector. Do not treat the direction byte as a pound amount.
volatile int confirmedChainPercent=-32768;
volatile int8_t confirmedChainDirection=-1, confirmedChainVariant=-1;
volatile uint32_t weightConfirmedAt=0;
int pendingWeightDelta=0;
bool targetInitialized=false;
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

bool validFrame(const uint8_t* data, size_t n) {
  if (n < 5 || data[0] != 0x55 || data[1] != n) return false;
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

int settingWidth(uint16_t id) {
  switch(id) {
    case 0x4fb0: case 0x53b0: case 0x556f: return 1;
    case 0x3e86: case 0x3e87: case 0x3e88: case 0x3e89:
    case 0x5551: case 0x5552: return 2;
    case 0x54da: case 0x53d0: case 0x556a: case 0x556b: return 4;
    default: return 0;
  }
}

void reconcileChainUi() {
  if(confirmedChainPercent<0 || confirmedChainDirection<0 || confirmedChainVariant<0) return;
  const int percent=constrain((confirmedChainPercent+50)/100,0,100);
  const bool supported=confirmedChainVariant==0;
  if(confirmedChainDirection==0) {
    requestedChains=percent;
    chainsEnabled=supported && confirmedChainPercent>0;
    inverseChainsEnabled=false;
    chainsInitialized=inverseChainsInitialized=true;
  } else if(confirmedChainDirection==1) {
    requestedInverseChains=percent;
    inverseChainsEnabled=supported && confirmedChainPercent>0;
    chainsEnabled=false;
    chainsInitialized=inverseChainsInitialized=true;
  }
}

void applySetting(uint16_t id, const uint8_t* value, int width) {
  if(id==0x4fb0 && width==1) updateConfirmedMode(value[0]);
  else if(id==0x3e86 && width==2) {
    int pounds=uint16_t(value[0])|(uint16_t(value[1])<<8);
    if(pounds>=5 && pounds<=230) {
      confirmedWeight=pounds; weightConfirmedAt=millis(); target=pounds; targetInitialized=true;
      Serial.printf("DEVICE CONFIRMED base_weight=%d lb; display synchronized\n",pounds);
    }
  } else if(id==0x3e88 && width==2) {
    confirmedEccentric=int16_t(uint16_t(value[0])|(uint16_t(value[1])<<8));
    if(!eccentricInitialized) { requestedEccentric=confirmedEccentric; eccentricEnabled=confirmedEccentric!=0; eccentricInitialized=true; }
  } else if(id==0x53b0 && width==1) confirmedChainDirection=value[0];
  else if(id==0x556f && width==1) confirmedChainVariant=value[0];
  else if(id==0x54da && width==4) {
    uint32_t raw=uint32_t(value[0])|(uint32_t(value[1])<<8)|(uint32_t(value[2])<<16)|(uint32_t(value[3])<<24);
    if(raw<=10000) confirmedChainPercent=int(raw);
  }
}

bool decodeSettings(const uint8_t* data, size_t n, size_t at, uint16_t count) {
  const size_t payloadEnd=n-2;
  for(uint16_t i=0;i<count;i++) {
    if(at+2>payloadEnd) return false;
    const uint16_t id=uint16_t(data[at]) | uint16_t(data[at+1])<<8; at+=2;
    const int width=settingWidth(id);
    // Unknown widths cannot be skipped safely; reject this frame rather than
    // misaligning the remainder and fabricating a confirmed state.
    if(width==0 || at+size_t(width)>payloadEnd) return false;
    applySetting(id,data+at,width);
    at+=width;
  }
  if(at!=payloadEnd) return false;
  reconcileChainUi();
  return true;
}

void decodeMode(const uint8_t* data, size_t n) {
  // Voltra's legacy 0x55/0x2E settings cascade has the same parameter
  // layout as cmd 0x10, but no inner command byte. It is used for settings
  // changed from the Voltra itself, including inverse chains.
  if (!validFrame(data,n)) return;
  // Read response: status at 11, uint16 LE count at 12-13, fields at 14.
  if(n>16 && data[4]==0x10 && data[5]==0xaa && data[10]==0x0f)
    decodeSettings(data,n,14,uint16_t(data[12]) | uint16_t(data[13])<<8);
  // Settings notification: uint16 LE count at 11-12, fields at 13.
  if(n>15 && data[10]==0x10)
    decodeSettings(data,n,13,uint16_t(data[11]) | uint16_t(data[12])<<8);
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
    dropArmed=dropEnabled && activationTriggered;
    Serial.printf("DROP SET: return detected; %ds hold armed\n",dropHoldSeconds);
  } else if(dropArmed) {
    dropArmed=false;
    if(pendingAction==PendingAction::AutoDrop) pendingAction=PendingAction::None;
    Serial.println("DROP SET: new rep detected; countdown cancelled");
  }
}

void received(NimBLERemoteCharacteristic*, uint8_t* data, size_t n, bool) {
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

bool writePacket(const uint8_t* data, size_t n) {
  if (!client || !client->isConnected() || !writer) return false;
  // Match SDK write-with-response; never split protocol frames arbitrarily.
  if (client->getMTU() < n + 3) {
    Serial.println("ERROR: negotiated MTU too small");
    return false;
  }
  bool ok = writer->writeValue(data, n, true);
  if (!ok) {
    enabled = false; activationTriggered=false;
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
uint16_t nextSettingSequence() {
  static uint16_t sequence=0x3000;
  return sequence++;
}
bool writeUnsignedSetting(uint16_t id, uint32_t value, uint8_t width, uint16_t sequence) {
  if(width<1 || width>4 || (width<4 && value >= (uint32_t(1)<<(width*8)))) return false;
  const size_t n=17+width;
  uint8_t p[21] = {0x55,0,4,0,0xaa,0x10,uint8_t(sequence),uint8_t(sequence>>8),
                   0x20,0,0x11,1,0,uint8_t(id),uint8_t(id>>8),0,0,0,0,0,0};
  p[1]=uint8_t(n);
  for(uint8_t i=0;i<width;i++) p[15+i]=uint8_t(value>>(8*i));
  p[3]=crc8(p,3); const uint16_t c=crc16(p,n-2); p[n-2]=c&255; p[n-1]=c>>8;
  return writePacket(p,n);
}
bool sendChainSettings(bool inverse, bool enabled, int percent) {
  if(percent<0 || percent>100) return false;
  // The direction survives when the shared amount is zero. Clear first so a
  // failed direction/amount write cannot leave the previous curve active.
  if(!writeUnsignedSetting(0x54da,0,4,nextSettingSequence())) return false;
  if(!enabled) return true;
  if(!writeUnsignedSetting(0x556f,0,1,nextSettingSequence())) return false;
  if(!writeUnsignedSetting(0x53b0,inverse?1:0,1,nextSettingSequence())) return false;
  return writeUnsignedSetting(0x54da,uint32_t(percent)*100,4,nextSettingSequence());
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
  // Base/mode and the complete shared chain state. The corresponding decoder
  // knows every requested width, so it can reject malformed replies safely.
  uint8_t p[] = {0x55,29,4,0,0xaa,0x10,0,0x20,0x20,0,0x0f,7,0,
                 0xb0,0x4f,0x86,0x3e,0x88,0x3e,0x6f,0x55,0xb0,0x53,
                 0xda,0x54,0x87,0x3e,0,0};
  p[3]=crc8(p,3); uint16_t c=crc16(p,sizeof(p)-2);
  p[27]=c&255; p[28]=c>>8;
  modeQueryAt=millis();
  lastSettingsPollAt=modeQueryAt;
  writePacket(p,sizeof(p));
}
void cleanup() {
  ready = enabled = polling = false; activationTriggered=false;
  dropArmed=false; lastRepAt=lastReturnAt=0;
  confirmedMode=ConfirmedMode::Unknown; modeConfirmedAt=0;
  confirmedWeight=confirmedEccentric=-32768;
  confirmedChainPercent=-32768; confirmedChainDirection=confirmedChainVariant=-1;
  weightConfirmedAt=0; pendingWeightDelta=0; target=0; targetInitialized=false;
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
    Serial.printf("BLE connected=%d handshake_sent=%d writes_enabled=%d requested_lb=%d device_state=unknown\n",
                  client && client->isConnected(), ready, enabled, target);
    Serial.printf("requested modifiers: eccentric=%d lb chains=%d%% inverse_chains=%d%%; confirmed weight/ecc=%d/%d chain=%d/100%% direction=%d variant=%d\n",
                  requestedEccentric,requestedChains,requestedInverseChains,
                  int(confirmedWeight),int(confirmedEccentric),int(confirmedChainPercent),
                  int(confirmedChainDirection),int(confirmedChainVariant));
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
    polling=false; enabled=false; activationTriggered=false; dropArmed=false;
    Serial.println(send(protocol::stop)?"STOP written; verify release on device":"STOP failed; use Voltra controls");
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
    activationTriggered=true;
    polling=true; pollStart=lastPoll=millis();
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
  if(toggled!=UiSelection::Weight) {
    lastActivityAt=millis();
    if(toggled==UiSelection::Eccentric) eccentricEnabled=!eccentricEnabled;
    else if(toggled==UiSelection::Chains) {
      chainsEnabled=!chainsEnabled;
      if(chainsEnabled) { inverseChainsEnabled=false; if(requestedChains==0) requestedChains=5; }
    }
    else {
      inverseChainsEnabled=!inverseChainsEnabled;
      if(inverseChainsEnabled) { chainsEnabled=false; if(requestedInverseChains==0) requestedInverseChains=5; }
    }
    const char* name=toggled==UiSelection::Eccentric?"eccentric":
      toggled==UiSelection::Chains?"chains":"inverse chains";
    bool on=toggled==UiSelection::Eccentric?eccentricEnabled:
      toggled==UiSelection::Chains?chainsEnabled:inverseChainsEnabled;
    Serial.printf("Touch toggled %s %s; saved value retained\n",name,on?"ON":"OFF");
    if(loadStep!=LoadStep::Idle || polling || activationTriggered) {
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
    if(selection==UiSelection::Weight && ready && client && client->isConnected())
      pendingWeightDelta=constrain(pendingWeightDelta+knobDelta,-225,225);
    else if(selection==UiSelection::Weight && targetInitialized) target=constrain(target+knobDelta,5,230);
    else if(selection==UiSelection::Eccentric) requestedEccentric=constrain(requestedEccentric+knobDelta,-195,195);
    else if(selection==UiSelection::Chains) requestedChains=constrain(requestedChains+knobDelta,0,100);
    else requestedInverseChains=constrain(requestedInverseChains+knobDelta,0,100);
    Serial.printf("Knob requested: weight=%d delta=%d eccentric=%d lb chains=%d%% inverse=%d%%; checking device mode\n",
                  target,pendingWeightDelta,requestedEccentric,requestedChains,requestedInverseChains);
    if((loadStep!=LoadStep::Idle)||polling||activationTriggered) {
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
    pendingAction=PendingAction::None; loadStep=LoadStep::Idle; polling=false; enabled=false; activationTriggered=false; dropArmed=false;
    Serial.println(send(protocol::stop)?"Double-click STOP written; verify release on device":"STOP failed; use Voltra controls");
  } else if(button==UiButtonAction::ApplyWeight && activationTriggered) {
    pendingAction=PendingAction::None; loadStep=LoadStep::Idle; polling=false;
    activationTriggered=false; enabled=false; dropArmed=false;
    Serial.println(send(protocol::stop)
      ?"Center tap STOP written; verify release on device"
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
  if(dropEnabled && activationTriggered && dropArmed && lastReturnAt && lastRepAt==lastReturnAt &&
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
        else if(action==PendingAction::KnobChains) { setting="chains"; value=chainsEnabled?requestedChains:0; sent=sendChainSettings(false,chainsEnabled,requestedChains); }
        else if(action==PendingAction::KnobInverseChains) { setting="inverse chains"; value=inverseChainsEnabled?requestedInverseChains:0; sent=sendChainSettings(true,inverseChainsEnabled,requestedInverseChains); }
        else {
          target=constrain(int(confirmedWeight)+pendingWeightDelta,5,230);
          pendingWeightDelta=0; targetInitialized=true; value=target;
          sent=sendWeight(target);
        }
        if(sent) {
          const bool isChain=action==PendingAction::KnobChains || action==PendingAction::KnobInverseChains;
          Serial.printf("Requested %s %d%s written: awaiting combined device confirmation\n",setting,value,isChain?"%":" lb");
          if(isChain) queryCurrentSettings();
        }
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
      activationTriggered=true;
      polling=true; pollStart=lastPoll=millis();
      Serial.println("Guided-load trigger written. Activation remains device-unconfirmed; double-click to STOP.");
    }
  }
  if(loadStep==LoadStep::StartAfterSetup && millis()-loadStepAt>=300) {
    loadStep=LoadStep::Idle;
    if(send(protocol::workoutGo)) {
      activationTriggered=true;
      Serial.printf("Normal workout GO written at %d lb; motor state remains device-unconfirmed\n",target);
    }
  }
  // Some firmware versions omit inverse chains from their pushed settings
  // cascade. Periodically read the canonical register list while safely idle.
  // This is read-only and intentionally does not reset the idle-sleep timer.
  if(ready && client && client->isConnected() && !activationTriggered &&
     !polling && loadStep==LoadStep::Idle && pendingAction==PendingAction::None &&
     millis()-lastSettingsPollAt>=kSettingsPollMs) {
    queryCurrentSettings();
  }
  uiTick(target,requestedEccentric,requestedChains,requestedInverseChains,
         int(confirmedWeight),int(confirmedEccentric),int(confirmedChainPercent<0?-32768:(confirmedChainPercent+50)/100),
         int(confirmedChainPercent<0?-32768:(confirmedChainPercent+50)/100),
         eccentricEnabled,chainsEnabled,inverseChainsEnabled,
         activationTriggered,dropEnabled,dropAmount,dropHoldSeconds,dropArmed,autoSleepEnabled,
         client && client->isConnected());
  if (ready && (!client || !client->isConnected())) {
    ready=enabled=polling=false; activationTriggered=false; dropArmed=false; writer=nullptr;
    confirmedMode=ConfirmedMode::Unknown; modeConfirmedAt=0;
    confirmedWeight=confirmedEccentric=-32768;
    confirmedChainPercent=-32768; confirmedChainDirection=confirmedChainVariant=-1;
    weightConfirmedAt=0; pendingWeightDelta=0; target=0; targetInitialized=false;
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
  if(autoSleepEnabled && !activationTriggered && !dropArmed &&
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
