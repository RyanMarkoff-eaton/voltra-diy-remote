#ifdef VOLTRA_DISPLAY
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include "ControllerUi.h"
#include "UiAssets.h"
#include <math.h>

// Elecrow 2.1 V1.0 factory/full example wiring. The small encoder demo
// in the vendor repository describes another board and must not be used.
static Arduino_SWSPI initBus(-1, 16, 2, 1, -1);
static Arduino_ESP32RGBPanel rgb(
  40,7,15,41, 46,3,8,18,17, 14,13,12,11,10,9, 5,45,48,47,21,
  1,10,4,20, 1,10,4,20, 0,12000000);
static Arduino_RGB_Display screen(480,480,&rgb,0,false,&initBus,-1,
  st7701_type5_init_operations,sizeof(st7701_type5_init_operations));
static bool available = false;
bool uiReady() { return available; }
static portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int pending = 0;
static volatile uint8_t previous = 0;
static volatile int quarterSteps = 0;
static UiButtonAction buttonAction = UiButtonAction::None;
static UiSelection selected = UiSelection::None;
static UiSelection modifierToggle = UiSelection::None;
enum class UiPage : uint8_t { Main, DropSets };
static UiPage page=UiPage::Main;
static UiDropEdit dropEdit=UiDropEdit::None;
static bool dropToggle=false;
static bool sleepToggle=false;
static bool buttonRaw = false, buttonDown = false, holdFired = false;
static uint32_t rawChangedAt = 0, buttonDownAt = 0, firstClickAt = 0;
static uint8_t clickCount = 0;
static bool touchAvailable=false, touchDown=false;
static int touchStartX=0, touchStartY=0;
static int touchLastX=0, touchLastY=0;
static uint32_t touchStartAt=0;
static bool touchHoldFired=false;

void IRAM_ATTR encoderChanged() {
  const uint8_t next = (digitalRead(42)<<1) | digitalRead(4);
  portENTER_CRITICAL_ISR(&encoderMux);
  // Reject impossible transitions; opposite edges cancel contact bounce.
  const uint8_t transition = (previous<<2) | next;
  int delta = 0;
  switch (transition) {
    case 1: case 7: case 14: case 8: delta=1; break;
    case 2: case 11: case 13: case 4: delta=-1; break;
    default: if (next != previous) quarterSteps=0; break;
  }
  quarterSteps += delta;
  if (quarterSteps >= 4) { if(pending<230) ++pending; quarterSteps=0; }
  if (quarterSteps <= -4) { if(pending>-230) --pending; quarterSteps=0; }
  previous=next;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

static bool expander(uint8_t value) {
  Wire.beginTransmission(0x21);
  Wire.write(value);
  return Wire.endTransmission()==0;
}
static bool touchPoint(int& x,int& y) {
  Wire.beginTransmission(0x15); Wire.write(0x02);
  if(Wire.endTransmission(false)!=0 || Wire.requestFrom(0x15,1)!=1) return false;
  if((Wire.read()&0x0f)==0) return false;
  Wire.beginTransmission(0x15); Wire.write(0x03);
  if(Wire.endTransmission(false)!=0 || Wire.requestFrom(0x15,4)!=4) return false;
  uint8_t d[4]; for(auto& b:d) b=Wire.read();
  x=((d[0]&0x0f)<<8)|d[1]; y=constrain((((d[2]&0x0f)<<8)|d[3])-20,0,479);
  return true;
}
static bool readKnobButton() {
  if(Wire.requestFrom(0x21,1)!=1) return false;
  return (Wire.read() & (1<<5)) == 0; // PCF8574 P5, active low
}
static uint16_t color(int r,int g,int b) {
  return ((r&248)<<8)|((g&252)<<3)|(b>>3);
}
static void mask(const uiAssets::Mask& m,int x,int y,int foreground,int background) {
  for(int row=0;row<m.h;row++) for(int col=0;col<m.w;col++) {
    int a=m.data[row*m.w+col];
    int c=(foreground*a+background*(255-a))/255;
    screen.drawPixel(x+col,y+row,color(c,c,c));
  }
}
static void centered(const uiAssets::Mask& m,int y,int foreground=235,int background=25) {
  mask(m,(480-m.w)/2,y,foreground,background);
}
static void arc(int radius,int first,int last,int thickness,uint16_t c) {
  for(int degree=first;degree<=last;degree++) {
    float a=degree*PI/180.0f;
    screen.fillCircle(240+lroundf(cosf(a)*radius),240+lroundf(sinf(a)*radius),thickness/2,c);
  }
}
static void modifierArc(int radius,int value,int base,bool enabled,uint16_t accent) {
  if(value==0 || base<=0) return;
  int degrees=constrain(lroundf(abs(value)*359.0f/base),1,359);
  uint16_t shown=enabled?accent:color(65,65,70);
  if(value>0) arc(radius,-90,-90+degrees,5,shown);
  else arc(radius,-90-degrees,-90,5,shown);
}
static void textCentered(const char* text,int y,uint8_t size,uint16_t c) {
  screen.setTextSize(size); screen.setTextColor(c); screen.setCursor(240-int(strlen(text)*3*size),y);
  screen.print(text);
}
static void modifierRow(int y,const char* label,int value,int base,int confirmed,bool enabled,
                        uint16_t accent,UiSelection row) {
  bool active=selected==row;
  uint16_t edge=enabled?accent:color(70,70,70);
  // The dial target needs more than a subtle gray fill: make its entire row
  // use the setting's color, while an unselected row remains transparent.
  if(active) screen.fillRoundRect(70,y-4,340,42,18,accent);
  screen.drawRoundRect(70,y-4,340,42,18,active?color(15,15,20):edge);
  // A large explicit toggle target is easier to use than the former dot.
  screen.fillRoundRect(78,y+2,78,30,12,active?color(25,25,30):(enabled?accent:color(52,52,58)));
  screen.drawRoundRect(78,y+2,78,30,12,accent);
  screen.setTextSize(1); screen.setTextColor((active || !enabled)?accent:color(15,15,20));
  screen.setCursor(enabled?101:98,y+13); screen.print(enabled?"ON":"OFF");
  screen.setTextSize(2); screen.setTextColor(active?color(15,15,20):edge); screen.setCursor(170,y+9); screen.print(label);
  int pct=base?lroundf(value*100.0f/base):0;
  char amount[30]; snprintf(amount,sizeof(amount),"%+d lb  %+d%%",value,pct);
  screen.setTextColor(active?color(15,15,20):color(235,235,235)); screen.setCursor(235,y+9); screen.print(amount);
  int effective=enabled?value:0;
  screen.setTextSize(1); screen.setTextColor(active?color(15,15,20):(confirmed==effective?color(100,210,130):color(180,150,80)));
  screen.setCursor(350,y+25);
  if(!enabled) screen.print("OFF / SAVED");
  else screen.print(confirmed==effective?"ON / CONFIRMED":"ON / UNVERIFIED");
}
static void dropRow(int y,const char* label,int value,const char* unit,bool active,uint16_t accent) {
  screen.fillRoundRect(80,y,320,58,18,active?color(44,44,50):color(31,31,35));
  screen.drawRoundRect(80,y,320,58,18,accent);
  screen.setTextSize(2); screen.setTextColor(accent); screen.setCursor(105,y+12); screen.print(label);
  char text[18]; snprintf(text,sizeof(text),"%d %s",value,unit);
  screen.setTextColor(color(240,240,240)); screen.setCursor(265,y+25); screen.print(text);
}
static void renderDropSets(bool dropEnabled,int dropAmount,int dropHoldSeconds,bool dropArmed,bool autoSleepEnabled,bool connected) {
  screen.fillScreen(0);
  screen.fillCircle(240,240,208,color(25,25,25));
  textCentered("DROP SETS",72,3,color(235,235,235));
  textCentered("SWIPE LEFT TO RETURN",108,1,color(125,125,135));
  uint16_t modeColor=dropEnabled?color(95,210,135):color(95,95,100);
  screen.fillRoundRect(105,140,270,44,22,dropEnabled?color(32,74,49):color(43,43,46));
  screen.drawRoundRect(105,140,270,44,22,modeColor);
  textCentered(dropEnabled?"AUTO DROP  ON":"AUTO DROP  OFF",153,2,modeColor);
  dropRow(195,"DROP",dropAmount,"LB",dropEdit==UiDropEdit::Amount,color(244,157,70));
  dropRow(260,"HOLD",dropHoldSeconds,"SEC",dropEdit==UiDropEdit::Hold,color(110,178,226));
  uint16_t sleepColor=autoSleepEnabled?color(130,205,155):color(100,100,105);
  screen.fillRoundRect(80,330,320,42,18,autoSleepEnabled?color(33,66,47):color(38,38,42));
  screen.drawRoundRect(80,330,320,42,18,sleepColor);
  textCentered(autoSleepEnabled?"AUTO SLEEP  15 MIN  ON":"AUTO SLEEP  15 MIN  OFF",343,1,sleepColor);
  textCentered("TAP A ROW, THEN TURN DIAL",390,1,color(190,190,195));
  textCentered(dropArmed?"COUNTDOWN ARMED":"AFTER LAST RETURN",410,1,
               dropArmed?color(244,157,70):color(135,135,145));
  textCentered(connected?"BLE CONNECTED":"BLE OFFLINE",435,1,connected?color(100,190,240):color(140,140,140));
  screen.flush();
}
static void render(int weight,int eccentric,int chains,int inverseChains,
                   int confirmedWeight,int confirmedEccentric,int confirmedChains,
                   int confirmedInverseChains,bool eccentricEnabled,bool chainsEnabled,
                   bool inverseChainsEnabled,bool activationTriggered,bool dropEnabled,
                   int dropAmount,int dropHoldSeconds,bool dropArmed,bool autoSleepEnabled,bool connected) {
  if(page==UiPage::DropSets) { renderDropSets(dropEnabled,dropAmount,dropHoldSeconds,dropArmed,autoSleepEnabled,connected); return; }
  screen.fillScreen(0);
  screen.fillCircle(240,240,208,color(25,25,25));
  // Requested-weight progress begins only after the controller has read a
  // valid value from the Voltra. 5 lb is empty and 230 lb completes the ring.
  arc(223,-90,269,10,color(45,55,65));
  int weightDegrees=weight>=5 ? lroundf((constrain(weight,5,230)-5)*359.0f/225.0f) : 0;
  if(weightDegrees>0) arc(223,-90,-90+weightDegrees,10,color(110,178,226));
  modifierArc(211,eccentric,weight,eccentricEnabled,color(255,145,55));
  modifierArc(201,chains,weight,chainsEnabled,color(55,205,225));
  modifierArc(191,inverseChains,weight,inverseChainsEnabled,color(185,105,255));
  screen.fillRoundRect(198,67,84,30,15,color(105,25,30));
  textCentered("STOP",75,2,color(255,205,205));
  if(weight>=5) {
    char text[5]; snprintf(text,sizeof(text),"%d",weight);
    int width=0;
    for(const char* p=text;*p;p++) width+=uiAssets::digits[*p-'0'].w+2;
    int x=(480-width+2)/2;
    for(const char* p=text;*p;p++) {
      const auto& digit=uiAssets::digits[*p-'0'];
      mask(digit,x,122,activationTriggered?240:105,25); x+=digit.w+2;
    }
  } else {
    textCentered(connected?"READING VOLTRA":"BLE OFFLINE",185,2,color(130,130,140));
  }
  // Separate selection target from the number's Load/Unload touch area.
  const bool weightSelected=selected==UiSelection::Weight;
  const uint16_t weightButton=color(110,178,226);
  if(weightSelected) screen.fillRoundRect(192,234,96,40,12,weightButton);
  screen.drawRoundRect(192,234,96,40,12,weightButton);
  screen.setTextSize(2); screen.setTextColor(weightSelected?color(15,15,20):weightButton);
  screen.setCursor(219,246); screen.print("LBS");
  modifierRow(286,"ECC",eccentric,weight,confirmedEccentric,eccentricEnabled,color(255,145,55),UiSelection::Eccentric);
  modifierRow(330,"CHAIN",chains,weight,confirmedChains,chainsEnabled,color(55,205,225),UiSelection::Chains);
  modifierRow(374,"INV",inverseChains,weight,confirmedInverseChains,inverseChainsEnabled,color(185,105,255),UiSelection::InverseChains);
  textCentered(connected?"BLE CONNECTED":"BLE OFFLINE",425,1,connected?color(100,190,240):color(140,140,140));
  screen.flush();
}
void uiBegin() {
  pinMode(6,OUTPUT); digitalWrite(6,LOW);
  Wire.begin(38,39);
  // PCF8574: P3 panel power, P4 reset; leave input/unrelated pins high.
  if(!expander(0xff)) { Serial.println("UI ERROR: expander unavailable"); return; }
  delay(100); expander(0xef); delay(120); expander(0xff); delay(120);
  expander(0xfe); delay(120); expander(0xff); delay(120); // CST826 reset on P0
  if(!psramFound() || !screen.begin()) { Serial.println("UI ERROR: PSRAM/display init failed"); return; }
  pinMode(42,INPUT); pinMode(4,INPUT);
  previous=(digitalRead(42)<<1)|digitalRead(4);
  attachInterrupt(digitalPinToInterrupt(42),encoderChanged,CHANGE);
  attachInterrupt(digitalPinToInterrupt(4),encoderChanged,CHANGE);
  available=true;
  int tx,ty; touchAvailable=touchPoint(tx,ty) || [](){
    Wire.beginTransmission(0x15); return Wire.endTransmission()==0;
  }();
  render(0,0,0,0,-32768,-32768,-32768,-32768,false,false,false,false,false,5,2,false,false,false);
  digitalWrite(6,HIGH);
  Serial.printf("UI ready: 480x480, touch=%d, encoder A=42 B=4, auto-submit mode-gated\n",touchAvailable);
}
int uiTakeEncoderDelta() {
  if(!available) return 0;
  portENTER_CRITICAL(&encoderMux);
  int delta=pending; pending=0;
  portEXIT_CRITICAL(&encoderMux);
  return delta;
}
UiSelection uiSelection() { return selected; }
UiSelection uiTakeModifierToggle() {
  UiSelection result=modifierToggle; modifierToggle=UiSelection::None; return result;
}
bool uiTakeDropToggle() { bool result=dropToggle; dropToggle=false; return result; }
bool uiTakeSleepToggle() { bool result=sleepToggle; sleepToggle=false; return result; }
UiDropEdit uiDropEdit() { return page==UiPage::DropSets?dropEdit:UiDropEdit::None; }
UiButtonAction uiTakeButtonAction() {
  UiButtonAction result=buttonAction;
  buttonAction=UiButtonAction::None;
  return result;
}
void uiTick(int weight,int eccentric,int chains,int inverseChains,
            int confirmedWeight,int confirmedEccentric,int confirmedChains,
            int confirmedInverseChains,bool eccentricEnabled,bool chainsEnabled,
            bool inverseChainsEnabled,bool activationTriggered,bool dropEnabled,
            int dropAmount,int dropHoldSeconds,bool dropArmed,bool autoSleepEnabled,bool connected) {
  if(!available) return;
  const uint32_t now=millis();
  static uint32_t lastButtonPoll=0;
  // The physical knob switch is deliberately ignored. Touch provides all
  // activation/load/STOP actions to avoid accidental whole-device presses.
  static uint32_t lastTouchPoll=0;
  if(touchAvailable && now-lastTouchPoll>=20) {
    lastTouchPoll=now; int x=0,y=0; bool down=touchPoint(x,y);
    if(down && !touchDown) {
      touchStartX=x; touchStartY=y; touchStartAt=now; touchHoldFired=false;
    }
    if(down) { touchLastX=x; touchLastY=y; }
    if(page==UiPage::Main && down && touchDown && !touchHoldFired &&
       abs(x-touchStartX)<20 && abs(y-touchStartY)<20 &&
       touchStartX>=145 && touchStartX<=335 &&
       touchStartY>=112 && touchStartY<230 && now-touchStartAt>=1000) {
      touchHoldFired=true; buttonAction=UiButtonAction::GuidedLoad;
    }
    if(!down && touchDown) {
      int dx=touchLastX-touchStartX;
      if(page==UiPage::Main && dx>=80) { page=UiPage::DropSets; dropEdit=UiDropEdit::None; }
      else if(page==UiPage::DropSets && dx<=-80) { page=UiPage::Main; dropEdit=UiDropEdit::None; }
      else if(page==UiPage::DropSets) {
        if(touchStartY>=135 && touchStartY<195) dropToggle=true;
        else if(touchStartY>=185 && touchStartY<255) dropEdit=UiDropEdit::Amount;
        else if(touchStartY>=255 && touchStartY<325) dropEdit=UiDropEdit::Hold;
        else if(touchStartY>=325 && touchStartY<380) sleepToggle=true;
      } else if(touchStartX>=198 && touchStartX<=282 && touchStartY>=60 && touchStartY<=105)
        buttonAction=UiButtonAction::Stop;
      else if(abs(dx)<20 && abs(touchLastY-touchStartY)<20 &&
              touchStartX>=192 && touchStartX<=288 && touchStartY>=234 && touchStartY<274) {
        selected=selected==UiSelection::Weight?UiSelection::None:UiSelection::Weight;
      }
      else if(abs(dx)<20 && abs(touchLastY-touchStartY)<20 &&
              touchStartX>=70 && touchStartX<=410 && touchStartY>=282 && touchStartY<412) {
        UiSelection row=touchStartY<324?UiSelection::Eccentric:
          touchStartY<368?UiSelection::Chains:UiSelection::InverseChains;
        if(touchStartX<164) {
          modifierToggle=row;
          // A toggle is an action, not a dial target. Clear its old selection
          // immediately so the filled selection background cannot stick.
          selected=UiSelection::None;
        } else {
          selected=selected==row?UiSelection::None:row;
        }
      }
      else if(!touchHoldFired && abs(dx)<20 && abs(touchLastY-touchStartY)<20 &&
              touchStartX>=145 && touchStartX<=335 && touchStartY>=112 && touchStartY<230)
        buttonAction=UiButtonAction::ApplyWeight;
    }
    touchDown=down;
  }
  static int drawnWeight=-1,drawnEcc=-999,drawnChains=-1,drawnInverse=-1;
  static int drawnCW=-999,drawnCE=-999,drawnCC=-999,drawnCI=-999;
  static bool drawnEE=false,drawnCEna=false,drawnIE=false;
  static bool drawnTriggered=false;
  static bool drawnDropEnabled=false,drawnDropArmed=false;
  static bool drawnAutoSleep=false;
  static int drawnDropAmount=-1,drawnDropHold=-1;
  static UiPage drawnPage=UiPage::DropSets;
  static UiSelection drawnSelection=UiSelection::InverseChains;
  static bool wasConnected=false;
  static uint32_t lastDraw=0;
  if((drawnWeight!=weight || drawnEcc!=eccentric || drawnChains!=chains || drawnInverse!=inverseChains ||
      drawnCW!=confirmedWeight || drawnCE!=confirmedEccentric || drawnCC!=confirmedChains ||
      drawnCI!=confirmedInverseChains || drawnEE!=eccentricEnabled || drawnCEna!=chainsEnabled ||
      drawnIE!=inverseChainsEnabled || drawnTriggered!=activationTriggered ||
      drawnDropEnabled!=dropEnabled || drawnDropAmount!=dropAmount || drawnDropHold!=dropHoldSeconds ||
      drawnDropArmed!=dropArmed || drawnAutoSleep!=autoSleepEnabled || drawnPage!=page ||
      drawnSelection!=selected || wasConnected!=connected) && now-lastDraw>=40) {
    render(weight,eccentric,chains,inverseChains,confirmedWeight,confirmedEccentric,
           confirmedChains,confirmedInverseChains,eccentricEnabled,chainsEnabled,inverseChainsEnabled,
           activationTriggered,dropEnabled,dropAmount,dropHoldSeconds,dropArmed,autoSleepEnabled,connected);
    drawnWeight=weight; drawnEcc=eccentric; drawnChains=chains; drawnInverse=inverseChains;
    drawnCW=confirmedWeight; drawnCE=confirmedEccentric; drawnCC=confirmedChains; drawnCI=confirmedInverseChains;
    drawnEE=eccentricEnabled; drawnCEna=chainsEnabled; drawnIE=inverseChainsEnabled;
    drawnTriggered=activationTriggered;
    drawnDropEnabled=dropEnabled; drawnDropAmount=dropAmount; drawnDropHold=dropHoldSeconds;
    drawnDropArmed=dropArmed; drawnAutoSleep=autoSleepEnabled; drawnPage=page;
    drawnSelection=selected; wasConnected=connected; lastDraw=now;
  }
}
void uiPowerDown() {
  if(!available) return;
  screen.fillScreen(0); screen.flush();
  digitalWrite(6,LOW);
}
#endif
