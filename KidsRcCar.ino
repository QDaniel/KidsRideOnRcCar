#include <Wire.h>
#include <SeeedOLED.h>
#include <FastLED.h>
#include <EEPROM.h>
#include "DHT.h" //DHT Bibliothek laden
#include <AltSoftSerial.h>

#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include "BTS7960.h"

//#define DHTPIN 2 //Der Sensor wird an PIN 2 angeschlossen    
#define DATA_PIN 3
#define NUM_LEDS 12

#define L_EN 7
#define L_PWM 5
#define R_EN 10
#define R_PWM 6
#define FOOT_PIN A6

#define SM_L A0 //LenkMotor -> Links
#define SM_R A1 //LenkMotor -> Rechts
#define HORN A2 //Lenkrad Horn


CRGB leds[NUM_LEDS];

//#define DHTTYPE DHT11    // Es handelt sich um den DHT11 Sensor

//DHT dht(DHTPIN, DHTTYPE); //Der Sensor wird ab jetzt mit „dth“ angesprochen

float vinSumm = 0;
float vinCell = 0;

float minCellV = 3.4;
float maxCellV = 4;
byte   cntCells = 4;
float difCellV = maxCellV - minCellV;
unsigned int   critTone = 440;
float critCell = 3.3;
float warnCell = 3.4;
unsigned int   critPIN = 11;

float   volt_R1 = 100000.0;
float   volt_R2 = 10000.0;

bool blinkState = true;
unsigned int ledState = 255;

unsigned int btnState = 0;

const unsigned char BasicFont[][8] PROGMEM = {
    {0xff, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81}, //  0, L0
    {0xff, 0x81, 0xbd, 0x81, 0x81, 0x81, 0x81, 0x81}, //  1, L1
    {0xff, 0x81, 0xbd, 0xbd, 0x81, 0x81, 0x81, 0x81}, //  2, L2
    {0xff, 0x81, 0xbd, 0xbd, 0xbd, 0x81, 0x81, 0x81}, //  3, L3
    {0xff, 0x81, 0xbd, 0xbd, 0xbd, 0xbd, 0x81, 0x81}, //  4, L4
    {0xff, 0x81, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0x81}, //  5, L5
    {0xff, 0x81, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd}, //  6, L6

    {0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81}, //  7, M0
    {0xbd, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81}, //  8, M1
    {0xbd, 0xbd, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81}, //  9, M2
    {0xbd, 0xbd, 0xbd, 0x81, 0x81, 0x81, 0x81, 0x81}, // 10, M3
    {0xbd, 0xbd, 0xbd, 0xbd, 0x81, 0x81, 0x81, 0x81}, // 11, M4
    {0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0x81, 0x81, 0x81}, // 12, M5
    {0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0x81, 0x81}, // 13, M6
    {0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0x81}, // 14, M7
    {0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd}, // 15, M8
  
    {0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xff}, // 16, R0
    {0xbd, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xff}, // 17, R1
    {0xbd, 0xbd, 0x81, 0x81, 0x81, 0x81, 0x81, 0xff}, // 18, R2
    {0xbd, 0xbd, 0xbd, 0x81, 0x81, 0x81, 0x81, 0xff}, // 19, R3
    {0xbd, 0xbd, 0xbd, 0xbd, 0x81, 0x81, 0x81, 0xff}, // 20, R4
    {0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0x81, 0x81, 0xff}, // 21, R5
    {0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0xbd, 0x81, 0xff}, // 22, R6
};

SoftwareSerial Mp3Serial(2, 4); // RX = 9, TX=10
AltSoftSerial BTSerial; // RX = 9, TX=10
BTS7960 motor1(L_EN, R_EN, L_PWM, R_PWM);

DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

long lastmill = millis();

long lastVoltM = 0;
long lastSett = 0;
float tempC = 0;
long lastTemp = 0;

int PIN_US_TRIG = 8; //Trigger-Pin des Ultraschallsensors an Pin7 des Arduino-Boards 
int PIN_US_ECHO = 7; // Echo-Pim des Ultraschallsensors an Pin6 des Arduino-Boards 
long dauer=0; // Das Wort dauer ist jetzt eine Variable, unter der die Zeit gespeichert wird, die eine Schallwelle bis zur Reflektion und zurück benötigt. Startwert ist hier 0.
int entfernung = 500;
struct funcTimer
  {
      long interval;
      long lastRun;
      void (* func)();
      bool enabled;
  };
typedef struct funcTimer funcTimer;

struct EEPROMSettings {
  byte Ver;
  bool ParentMode;
};
EEPROMSettings sett;

funcTimer ft[11];

int maxSpeed = 1024;
int minSpeed = -512;
int curSpeed = 0;
int stpSpeed = 25;
bool lastParentState = true;
bool ParentState = false;
bool Bremse = true;
char ParentControl = (char)0;

long lastBTtime = 0; 
char btBuf[10]; 

void setup() 
{ 
  Wire.begin();
  SeeedOled.init();  //initialze SEEED OLED display
  SeeedOled.clearDisplay();          //clear the screen and set start position to top left corner
  SeeedOled.setNormalDisplay();      //Set display to normal mode (i.e non-inverse mode)
  SeeedOled.setPageMode();           //Set addressing mode to Page Mode
  SeeedOled.setTextXY(3, 3);         //Set the cursor to Xth Page, Yth Column
  SeeedOled.putString("Starting ..."); //Print the String

  //dht.begin(); //DHT11 Sensor starten
  Serial.begin(9600);
  BTSerial.begin(9600); 
  Mp3Serial.begin(9600);
  
  pinMode(0, INPUT_PULLUP);
  pinMode(FOOT_PIN, INPUT); // Foot Switch
  pinMode(HORN, INPUT); // Lenkrad Horn
  pinMode(SM_L, OUTPUT); 
  pinMode(SM_R, OUTPUT);
  
  //pinMode(critPIN, OUTPUT);
  //pinMode(PIN_US_TRIG, OUTPUT); // Trigger-Pin ist ein Ausgang
  //pinMode(PIN_US_ECHO, INPUT); // Echo-Pin ist ein Eingang

  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);

  myDFPlayer.begin(Mp3Serial);
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
  delay(100);
  myDFPlayer.play(3);  //Play the first mp3
  motor1.Disable();
  motor1.Stop();

  ft[0].enabled = true;
  ft[0].interval = 500;
  ft[0].func = &critVoltage;

  ft[1].enabled = true;
  ft[1].interval = 2000;
  ft[1].func = &dispVoltmeterOLED;
  
  ft[2].enabled = false;
  ft[2].interval = 10000;
  ft[2].func = &readTemperatur;

  ft[3].enabled = false;
  ft[3].interval = 500;
  ft[3].func = &readAbstand;

  ft[4].enabled = true;
  ft[4].interval = 500;
  ft[4].func = &dispSettings;

  ft[5].enabled = true;
  ft[5].interval = 2000;
  ft[5].func = &checkHorn;
  
  ft[6].enabled = true;
  ft[6].interval = 800;
  ft[6].func = &setLedsForce;

  ft[7].enabled = true;
  ft[7].interval = 100;
  ft[7].func = &checkBluetoothInput;
  
  ft[8].enabled = false;
  ft[8].interval = 800*3*2;
  ft[8].func = &setBlinkerOff;

  ft[9].enabled = true;
  ft[9].interval = 100;
  ft[9].func = &checkFootSwitch;

  ft[10].enabled = true;
  ft[10].interval = 100;
  ft[10].func = &checkButtons;


  EEPROM.get(0, sett);
  if(sett.Ver != 2)
  {
    sett.Ver=2;
    sett.ParentMode=true;
    EEPROM.put(0, sett);
  }

   while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  SeeedOled.clearDisplay();
}

void loop()
{
  long curmillis = millis();
  for(int i=0; i<=10; i++){
    if(ft[i].enabled && (ft[i].lastRun == 0 || curmillis - ft[i].lastRun >  ft[i].interval)) {
      ft[i].lastRun = curmillis;
      ft[i].func();
    }
  }
  
  if (BTSerial.available() > 0) {
    int r = BTSerial.read();
    Serial.write(r);
    handleBluetoothInput(r);
  }
  if (Serial.available()) {
    BTSerial.write(Serial.read());
  }

  dispTiming();
}

void checkBluetoothInput(){
  if(ParentState && millis() - lastBTtime > 500) ParentState = false; 
}

char lastLR = 'x';
void handleBluetoothInput(int data){
  lastBTtime = millis();
  ParentState = true;
  char lr = 'X';
  char ch = (char)data;  
  SeeedOled.setTextXY(7, 0);         //Set the cursor to Xth Page, Yth Column
  SeeedOled.putChar(ch); //Print the String

  if(ch == 'L' || ch == 'R' || ch == 'F' || ch == 'B' || ch == 'G' || ch == 'I' || ch == 'H' || ch == 'J') Bremse = false;
  if(ch == 'L' || ch == 'R') { ParentControl = ch; lr = ch; setBlinkerOn(ch); }
  else if(ch == 'F' || ch == 'B' || ch == 'D'|| ch == 'S') ParentControl = ch;
  else if(ch == 'G' || ch == 'I') { ParentControl = 'F'; lr = ch == 'G' ? 'L': 'R'; setBlinkerOn(lr); }
  else if(ch == 'H' || ch == 'J') { ParentControl = 'B'; lr = ch == 'H' ? 'L': 'R'; setBlinkerOn(lr); }
  else if(ch == 'V') playHorn();
  else if(ch == 'W') setLEDBit(true, 0);
  else if(ch == 'w') setLEDBit(false, 0);
  else if(ch == 'U') setLEDBit(true, 6); 
  else if(ch == 'u') setLEDBit(false, 6); 
  else if(ch == 'X') { Bremse = true; setBlinkerOn('W'); }
  else if(ch == 'x') { Bremse = false; setBlinkerOff(); }

  else if(ch == '1') maxSpeed = 100;
  else if(ch == '2') maxSpeed = 200;
  else if(ch == '3') maxSpeed = 300;
  else if(ch == '4') maxSpeed = 400;
  else if(ch == '5') maxSpeed = 500;
  else if(ch == '6') maxSpeed = 600;
  else if(ch == '7') maxSpeed = 700;
  else if(ch == '8') maxSpeed = 800;
  else if(ch == '9') maxSpeed = 900;
  else if(ch == 'q') maxSpeed = 1024;

  if(lastLR != lr)
  {
    digitalWrite(SM_L, lr=='L' ? HIGH : LOW);  
    digitalWrite(SM_R, lr=='R' ? HIGH : LOW);
    lastLR = lr;
  }  
}

void critVoltage() {
  vinSumm = readSpannung(7, volt_R1, volt_R2);
  vinCell = vinSumm / cntCells;
  if(vinCell < critCell && vinSumm > 10) {
    tone(critPIN, critTone);
    critVoltageMP3();
  } else {
    noTone(critPIN);
  }   
}

long lastPlay = 0;
void critVoltageMP3(){
  if(millis() - lastPlay > 3000) {
    lastPlay = millis();
    myDFPlayer.play(7);  //Play the first mp3
  }
}

void dispVoltmeterOLED() {
  char buffer [10];
  if(vinSumm>=10){
     sprintf (buffer, "%2d.%1dV", (int)vinSumm, (int)((int)(vinSumm*100)%100)/10);   
  } else {
     sprintf (buffer, "%1d.%02dV", (int)vinSumm, (int)(vinSumm*100)%100);
  }
  SeeedOled.setTextXY(0, 11);         //Set the cursor to Xth Page, Yth Column
  SeeedOled.putString(&buffer[0]); //Print the String
  
  if(vinCell<=warnCell) {
    SeeedOled.setTextXY(0, 0);
    for (int i = 0; i < 10; ++i)
    {
      SeeedOled.putChar('!');
    }    
  } else {
    dispOLEDProgress(0, 0, 10, vinCell, minCellV, maxCellV);
  }
}

void readTemperatur(){
  //  tempC = dht.readTemperature(); //die Temperatur auslesen und unter „Temperatur“ speichern 
}

void checkHorn() {
  if(entfernung < 60) setBlinkerOn('W');
  if(entfernung < 30) playHorn();
  if(btnState & 1)
  {
    btnState = bitClear(btnState, 0);
    playHorn();
  }
}

void checkButtons() {
   int bt = analogRead(HORN);
   if(bt > 800) btnState = bitSet(btnState, 0);
}

void readAbstand(){
  digitalWrite(PIN_US_TRIG, HIGH); //Jetzt sendet man eine Ultraschallwelle los.
  delay(10); //Dieser „Ton“ erklingt für 10 Millisekunden.
  digitalWrite(PIN_US_TRIG, LOW);//Dann wird der „Ton“ abgeschaltet.
  dauer = pulseIn(PIN_US_ECHO, HIGH); //Mit dem Befehl „pulseIn“ zählt der Mikrokontroller die Zeit in Mikrosekunden, bis der Schall zum Ultraschallsensor zurückkehrt.
  long entf = (dauer/2) * 0.03432; //Nun berechnet man die Entfernung in Zentimetern. Man teilt zunächst die Zeit durch zwei (Weil man ja nur eine Strecke berechnen möchte und nicht die Strecke hin- und zurück). Den Wert multipliziert man mit der Schallgeschwindigkeit in der Einheit Zentimeter/Mikrosekunde und erhält dann den Wert in Zentimetern.
  if (entf < 500 && entf > 0) //Wenn die gemessene Entfernung über 500cm oder unter 0cm liegt,…
  {
    entfernung = (int)entf; //dann soll der serial monitor ausgeben „Kein Messwert“, weil Messwerte in diesen Bereichen falsch oder ungenau sind.
  } else {
    entfernung = 500;
  }
}

int lbt = -1;
void checkFootSwitch() {
  int bt = analogRead(FOOT_PIN);
  if(bt!=lbt)
  {
    SeeedOled.setTextXY(7, 12);
    SeeedOled.putNumber(bt);
    lbt=bt;
  }

  if(sett.ParentMode){
    if(!ParentState) setBlinkerOn('W');
    else if(ParentState != lastParentState) setBlinkerOff();
  }
  lastParentState = ParentState;

  int fakt = 0;
  
  if(!sett.ParentMode || ParentState) {
    if(ParentControl == 'D' || ParentControl == 'F' || ParentControl == 'B'){
      if(ParentControl == 'F') fakt = 1;
      if(ParentControl == 'B') fakt = -1;
     } else {  
      if(bt > 800 && bt < 1000) fakt = -1;
      else if(bt > 1000) fakt = 1;
    }
  }
  
  if (Bremse) {
    curSpeed = 0; 
  }
  else if (fakt == 1) {
    if(curSpeed < 0) curSpeed = 0; 
    if(curSpeed < maxSpeed) curSpeed += stpSpeed; 
    if(curSpeed > maxSpeed) curSpeed = maxSpeed;
  } else if (fakt == -1) {  
    if(curSpeed > 0) curSpeed = 0; 
    if(curSpeed > minSpeed) curSpeed -= stpSpeed; 
    if(curSpeed < minSpeed) curSpeed = minSpeed;
  } else {
    curSpeed = 0;
  }
  
  setLEDBit(fakt == 0, 7); 
  setLEDBit(fakt == -1, 4); 
  setMotor(curSpeed);
}

int lastSpeed = 0;
void setMotor(int cSpeed) {
  if(cSpeed != lastSpeed) {
    if(cSpeed == 0) {
      motor1.Stop();
      motor1.Disable();
    }
    else {
      motor1.Enable();  
      if(cSpeed>0) motor1.TurnRight(map(cSpeed, 0, 1024, 0, 255));
      else if(cSpeed<0) motor1.TurnLeft(map(cSpeed * -1, 0, 1024, 0, 255)); 
    }
    lastSpeed=cSpeed;
  }
} 

void setLedsChanges() {
  setLEDs(false);
}

void setBlinkerOn(char lr) {
  ft[8].lastRun = millis();
  ft[8].enabled = true;
  setLEDBit(lr=='L' || lr=='W', 2);
  setLEDBit(lr=='R' || lr=='W', 3);
}

void setBlinkerOff() {
  setLEDBit(false, 2);
  setLEDBit(false, 3);
}

void setLEDBit(bool on, int iBit)
{
  unsigned int lastLed = ledState;
  ledState = (on) ? bitSet(ledState, iBit) : bitClear(ledState, iBit);
  if(lastLed!=ledState) setLEDs(true);
}

void setLedsForce() {
  setLEDs(true);
}
void setLEDs(bool force) {
//  RF-L ; RF-R ; R-L ; R-R ; W-L ; W-R ; F-L ; F-R ; B-L ; B-R ; N-L ; N-R 

  //1 = Front
  leds[8] = leds[7] = (ledState & 1) ? CRGB::White : CRGB::Black;
  //2 = Rücklicht
  //leds[1] = leds[2] = (ledState & (1 << 1)) ? CRGB::Red : CRGB::Black;
  
  //3 = Blinker L
  leds[6] = ((ledState & (1 << 2)) && blinkState) ? CRGB::Orange : CRGB::Black;
  //4 = Blinker R
  leds[9] = ((ledState & (1 << 3))  && blinkState) ? CRGB::Orange : CRGB::Black;
  
  //5 = Rückfahrlicht
  leds[1] = leds[2] = (ledState & (1 << 4)) ? CRGB::White : CRGB::Black;
  if(!(ledState & (1 << 4)))
  {
    leds[1] = leds[6];
    leds[2] = leds[9];
  }

 //Warnblinker / Arbeitslicht
  if((ledState & (1 << 2)) &&  (ledState & (1 << 3)) ){
    leds[4] = leds[5] = leds[6];
  } else {
    //7 = Arbeitslicht
    leds[4] = leds[5] = (ledState & (1 << 6)) ? CRGB::White : CRGB::Black;
  }
  
  //6 = Nebel
  //leds[10] = leds[11] = (ledState & (1 << 5)) ? CRGB::White : CRGB::Black;
  // 8 = Bremslicht
  leds[0] = leds[3] = (ledState & (1 << 7)) ? CRGB::Red : CRGB::Black;
  // 9 = Cockpit
  //leds[4] = leds[5] = (ledState & (1 << 7)) ? CRGB::Blue : CRGB::Black;


  if((ledState & ((1 << 2) | (1 << 3))) || !blinkState) blinkState = !blinkState; 
  FastLED.show();
}

void dispSettings() {
  dispSettingBits(4, "Licht", ledState);
  dispSettingInt4(2, "Speed", curSpeed);
  dispSettingONOFF(3, "Parent", sett.ParentMode);
  SeeedOled.putChar(' ');
  SeeedOled.putChar(ParentState ? 'C' : 'D');
  
  dispSettingInt4(5, "Entf.", entfernung);
  //dispSettingGrad(6, "Temp.", tempC);
}
void dispTiming() {
  SeeedOled.setTextXY(6, 11);
  long newmill = millis();
  char buffer [10];
  sprintf(buffer, "%4d", newmill -lastmill);   
  SeeedOled.putString(&buffer[0]);
  lastmill = newmill; 
}

void dispSettingONOFF(int row, const char* caption, bool val) {
  dispSetting(row, caption);
  SeeedOled.putString(val ? " ON" : "OFF");
}
void dispSettingProz0(int row, const char* caption, byte val) {
  dispSetting(row, caption);
  SeeedOled.putNumber(val);
  SeeedOled.putChar('%');
}
void dispSettingGrad(int row, const char* caption, float val) {
  dispSetting(row, caption);
  SeeedOled.putFloat(val);
  SeeedOled.putChar('C');
}
void dispSettingInt4(int row, const char* caption, int val) {
  dispSetting(row, caption);
  char buffer[10];
  sprintf(buffer, "%4d", val);   
  SeeedOled.putString(&buffer[0]);
}
void dispSettingBits(int row, const char* caption, int val) {
  dispSetting(row, caption);
  SeeedOled.putChar(' ');
  for (int i=7; i>=0; i--)
  SeeedOled.putChar(bitRead(val,i) ? '1': '0'); 
}

void dispSetting(int row, const char* caption) {
  SeeedOled.setTextXY(row, 0);
  SeeedOled.putString(caption);
  SeeedOled.putChar(':');
}

void dispOLEDProgress(int row , int col, int len, float value, float minVal, float maxVal) {
    
    int countMax = (len * 8) - 4;
    int countMax6 = countMax - 6;
    int fc = len - 2;
    
    int count = (value - minVal) / ((maxVal - minVal) / countMax); 
    if(count > countMax) count = countMax;

    char cFirst = constrain(count, 0, 6);
    int cLast = constrain(count - countMax6, 0, 6);
      
    SeeedOled.setTextXY(row, col);

    SeeedOLED_putChar(cFirst);
    for (int i = 0; i < fc; ++i)
    {
      int diffNext = constrain(count - (6 + (i*8)), 0, 8);
      SeeedOLED_putChar(char(7 + diffNext));
    }
    SeeedOLED_putChar(char(16 + cLast));
}

void SeeedOLED_putChar(unsigned char C) {
    if(C > 23) {
      SeeedOled.putChar(C);
    } else {
        unsigned char i = 0;
        for (i = 0; i < 8; i++) {
            //read bytes from code memory
            SeeedOled.sendData(pgm_read_byte(&BasicFont[C][i])); //font array starts at 0, ASCII starts at 32. Hence the translation
        }
    }
}

float readSpannung(int Apin, float R1, float R2 ) {
 // Werte am analogen Pin lesen
 float values = analogRead(Apin);
 // Messwerte in Volt umrechnen = Spannung am Ausgang des Spannungsteilers
 float vout = (values * 5.0) / 1024.0;
 // Berechnen, welche Spannung am Eingang des Spannungsteilers anliegt.
 if(R1 == 0) return vout;
 float ret = vout / (R2 / (R1 + R2));  
 return ret;
}

long lastHorn = 0;
void playHorn() {
 if(millis() - lastHorn < 2000) return;
 lastHorn = millis();
 myDFPlayer.play(5);
}
