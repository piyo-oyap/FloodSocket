#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define NBEEPDEBUG
#define NGYRODEBUG
#define NALARMDEBUG
#define NACCELDEBUG
#define NGPSDEBUG
#define NBTDEBUG
#define NBLUETOOTH

#define GYROBUFFERSIZE 15
#define GYROINTERVAL 250

template <typename T>
struct Coords {
  T X;
  T Y;
  T Z;
};

Coords<float>& operator+=(Coords<float> &a, const Coords<float> &b) {
  a.X += b.X;
  a.Y += b.Y;
  a.Z += b.Z;
  return a;
}

Coords<float> operator/(Coords<float> a, const int &b) {
  a.X /= b;
  a.Y /= b;
  a.Z /= b;
  return a;
}

struct AlarmVars {
  int Repetition, OnInterval, OffInterval;
  int * ptrCurInterval;
  bool flagEnabled, flagOngoing;
  byte Sensitivity;
  AlarmVars() : Repetition{0}, OnInterval{0}, OffInterval{0}, ptrCurInterval{nullptr},
                flagEnabled{false}, flagOngoing{false}, Sensitivity{0}  { }
};

template <typename T>
class Buffer {
  private:
  T m_data[GYROBUFFERSIZE];
  int counterElements;
  void shift() {
    for (int i = 0; i < GYROBUFFERSIZE - 1; i++) {
      m_data[i] = m_data[i+1];
    }
  }
  
  public:
  Buffer() : m_data{}, counterElements{0} { }
  
  T& operator[](int i) {
    return m_data[i];
  }
  
  void insert(T entry) {
    if (counterElements < GYROBUFFERSIZE) {
      m_data[counterElements] = entry; 
      counterElements++;
    } else {
      shift();
      m_data[counterElements-1] = entry;
    }
  }
  
  void flush() {
    counterElements = 0;
    for (int i = 0; i < GYROBUFFERSIZE; i++) {
        m_data[i] = {};
      }
    }
  int getCount() {
    return counterElements;
  }
};

Coords<float> movementAlarm(); //remove this and get a free compile error
  

Coords<int> accel, gyro, pAccel, compensateVal;
Coords<float> gForce, rot;
AlarmVars alarm{};
Buffer<Coords<float>> gyroBuffer{};

String textMessage = "", mainNumber = "+639503610262";
unsigned long previousMillisAlarm = 0, previousMillisGyro = 0, previousMillisResponse = 0;
double Lat = 0.0, Lon = 0.0;
const byte gpsRXPin = 15, gpsTXPin = 14, simRXpin = 16, simTXpin = 10, btRXpin = 5, btTXpin = 8, buzz = 7, btn = 9;
int gpsData[6];
bool gpsWarn = false, location_warn = false, parkAlarmEnabled = false, isAccident = false;

TinyGPSPlus gps;
//SoftwareSerial gpsSerial(gpsTXPin, gpsRXPin);
//SoftwareSerial Serial2(simTXpin, simRXpin);
//SoftwareSerial BT(btTXpin, btRXpin);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  pinMode(btn, INPUT_PULLUP);
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  delay(1000);
  Serial2.print("AT+ CMGF=1\r");
  delay(1000);
  Serial2.print("AT+CNMI=2,2,0,0,0\r");
  delay(1000);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  Wire.begin();
  setupMPU();
  getCompensateVal();
  send_msg("Accident Detection System Started\n\nDrive safely and God bless.", "9503610262");
  
  //Serial.println("Started");
  delay(1000);
  //BT.println("Device Started");
  // BT.begin(9600);
  // delay(1000);
}

void loop() {
  while(Serial1.available()>0){
    if(!gpsWarn){
      gpsWarn = true;
    }
    if(gps.encode(Serial1.read())){
      if(gps.date.isValid()&&gps.time.isValid()&&gps.location.isValid()){
        gpsData[0] = gps.date.month();
        gpsData[1] = gps.date.day();
        gpsData[2] = gps.date.year();
        gpsData[3] = gps.time.hour();
        gpsData[4] = gps.time.minute();
        gpsData[5] = int(gps.speed.kmph());
        Lat = gps.location.lat();
        Lon = gps.location.lng();
        if(!location_warn){
          send_msg("GPS Location Available", mainNumber);
          location_warn = true;
        }
      }
    }
  }
  if(millis()>5000&&gps.charsProcessed()<10){
    lcd.clear();
    lcd.print("No GPS");
  }
  //delay(100);
  #ifndef NGPSDEBUG
  Serial.println(String(Lon, 5) + "," + String(Lat, 5));
  #endif
  
  if (!isAccident) {
    Tilt();
  
    Gyro();

    Alarm();

    recv_msg();

    // btGetString();
  } else {
    accidentResponseCancel();
    accidentResponse();
  }

  #ifndef NGYRODEBUG
  recordGyroRegisters();
  printCoords(rot);
  #endif

  
}

void accidentResponse() {
  if (millis() - previousMillisResponse > 20000) {
    if (Lat > 0.0 || Lon > 0.0){
      send_msg("Requesting assistance for vehicular accident at coordinates: https://www.google.com/maps/place/" + String(Lon,5) + "," + String(Lat,5) , mainNumber);
    }
    else {
      send_msg("Requesting assistance for vehicular accident, unfortunately the location was not determined during the accident.", mainNumber);
    }
    isAccident = false;
  }
}

void accidentResponseCancel() {
  unsigned long previousMillisCancel = millis();
  
  while (digitalRead(btn) == 0) {
    if (millis() - previousMillisCancel > 5000) {
      isAccident = false;
      Serial.println("Request for assistance is cancelled.");
      break;
    }
  }
}

#ifndef NBLUETOOTH
void btGetString() {
  String str = "";
  while (BT.available()) {
    str = BT.readString();
    #ifndef NBTDEBUG
    Serial.println(str);
    #endif

    if (str.charAt(0) == 'i') {
    BT.print("p");
    BT.print(alarm.flagEnabled);
    BT.print(";s");
    BT.print(alarm.Sensitivity);
    BT.print(";g");
    (gpsWarn) ? BT.print(1) : BT.print(0);
    } else if (str.charAt(0) == 'c') {
      calibrate();
    } else {
      btCom(str);
    }
  }
  
}

#endif

void setupMPU(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);//waking up the module
  Wire.write(0x1B);//gyro parameters
  Wire.write(0x00000000); // range +/- 250 rpm
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0b00000000); //range +/- 8g 2048 LSB/g
  Wire.endTransmission();
}

void recordAccelRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available()<6);
  accel.X = Wire.read()<<8|Wire.read();
  accel.Y = Wire.read()<<8|Wire.read();
  accel.Z = Wire.read()<<8|Wire.read();
  processAccelData();
}

void processAccelData(){
  gForce.X = fabs(accel.X / 16384.0);
  gForce.Y = fabs(accel.Y / 16384.0);
  gForce.Z = fabs(accel.Z / 16384.0);
  compensate();
}

void recordGyroRegisters(){
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6);
  while(Wire.available()<6);
  gyro.X = Wire.read()<<8|Wire.read();
  gyro.Y = Wire.read()<<8|Wire.read();
  gyro.Z = Wire.read()<<8|Wire.read();
  processGyroData();
}

void processGyroData(){
  rot.X = gyro.X / 131.0;
  rot.Y = gyro.Y / 131.0;
  rot.Z = gyro.Z / 131.0;
}

void calibrate(){
  EEPROM.write(500, 143); //just a flag
  recordAccelRegisters();
  EEPROM.write(501, gForce.X);
  EEPROM.write(502, gForce.Y);
  EEPROM.write(503, gForce.Z);
}

void getCompensateVal(){
  if(EEPROM.read(500)==143){
    compensateVal.X = EEPROM.read(501);
    compensateVal.Y = EEPROM.read(502);
    compensateVal.Z = EEPROM.read(503);
  }else{
    calibrate();
  }
}

void compensate(){
  gForce.X = abs(gForce.X - compensateVal.X);
  gForce.Y = abs(gForce.Y - compensateVal.Y);
  gForce.Z = abs(gForce.Z - compensateVal.Z);
}

void printData(){
//  Serial.print("Gyro");
//  Serial.print(" X= ");
//  Serial.print(rot.X);
//  Serial.print("\t\tY= ");
//  Serial.print(rot.Y);
//  Serial.print("\t\tZ= ");
//  Serial.print(rot.Z);
  //Serial.print("\t\tAccel");
  Serial.print(" X=");
  Serial.print(gForce.X*90);
  Serial.print("\t\tY= ");
  Serial.print((1-gForce.Y)*90);
  Serial.print("\t\tZ= ");
  Serial.println((1-gForce.Z)*90);
}

void lcdPrint(){
  lcd.setCursor(0,0);
  lcd.print(Lat);
  lcd.setCursor(0,1);
  lcd.print(Lon);
}

void recv_msg(){
  Serial2.print("AT+ CMGF=1\r");
  delay(50);
  Serial2.print("AT+CNMI=2,2,0,0,0\r");
  delay(50);
  if(Serial2.available()){
    textMessage = Serial2.readString();
    delay(100);
    tone(buzz, 20000, 1);
    Serial.println(textMessage);
    delay(100);
    if (textMessage.length() >= 53) {
        Serial2.print("AT+CMGD=1,3\r"); //deletes recv read sms
    }
    if(textMessage.indexOf("Location")>=0 && (Lat>0||Lon>0)>0){
      textMessage = "https://www.google.com/maps/place/" + String(Lat, 5) + "," + String(Lon, 5);
      send_msg(textMessage, mainNumber);
    }else if(textMessage.indexOf("Location")>=0 && (Lat==0.0||Lon==0.0)){
      textMessage = "Location is not determined, Poor GPS signal";
      send_msg(textMessage, mainNumber);
    }else if(textMessage.indexOf("Alarm ON")){
      if(EEPROM.read(100)==1){
        send_msg("Alarm turned ON", mainNumber);
        //set alarm flag on
      }
    }else if(textMessage.indexOf("Alarm OFF")>0){
      if(EEPROM.read(100)==1){
        send_msg("Alarm turned OFF", mainNumber);
        //set alarm flag Off
      }
    }else if(textMessage.indexOf("Enable Reply")>0){
      EEPROM.write(100,1);
    }else if(textMessage.indexOf("Disable Reply")>0){
      EEPROM.write(100,0);
    }else if(textMessage.indexOf("Park Mode ON")>0){
      alarm.flagEnabled = true;
    }else if(textMessage.indexOf("Park Mode OFF")>0){
      alarm.flagEnabled = false;
    }else if(textMessage.indexOf("POWER DOWN") > 0){
      Serial2.print("AT+ CMGF=1\r");
      delay(50);
      Serial2.print("AT+CNMI=2,2,0,0,0\r");
      delay(50);
    }

  }
}

void send_msg(String txt, String number){
  Serial2.print("AT+CMGF=1\r");
  delay(100);
  Serial2.println("AT+CMGS=\""+ number +"\"");
  delay(100);
  Serial2.println(txt);
  delay(100);
  Serial2.println((char)26);
  delay(1000);
  Serial2.println();
  delay(2000);
}

void btCom(String x){
  if(x.indexOf("p1")>=0){
    alarm.flagEnabled = true;
  }else if(x.indexOf("p0")>=0){
    alarm.flagEnabled = false;
  }else if(x.indexOf("s1")>=0){
    alarm.Sensitivity = 1;
  }else if(x.indexOf("s2")>=0){
    alarm.Sensitivity = 2;
  }else if(x.indexOf("s3")>=0){
    alarm.Sensitivity = 3;
  }else if(x.indexOf("s0")>=0){
    alarm.Sensitivity = 0;
  }
}

void activateAlarm(Coords<float> avg) {
  int xRaw = 0;
  xRaw += abs((int)avg.X);
  xRaw += abs((int)avg.Y);
  xRaw += abs((int)avg.Z);
  xRaw /= 3;
  #ifndef NALARMDEBUG
  Serial.println(xRaw);
  #endif

  float x = xRaw;
  switch (alarm.Sensitivity) {  // todo: tweak the multipliers
    case 0:
      x = 0;
    case 1:
      x *= 0.75;
      break;
    case 2:
      break;
    case 3:
      x *= 1.5;
      break;
  }
  
  if (!alarm.flagOngoing)
  { 
    if(0<=x && x<=5){
    } else if (5<x && x<=15) {
      turnOnAlarm(3, 250, 750);
    } else if(15<x && x<=20){
      turnOnAlarm(5, 500, 500);
    }else if(20<x && x<=25){
      turnOnAlarm(7, 750, 250);
    }else{
      turnOnAlarm(10, 1000, 0);
    }
  }
}

void turnOnAlarm(int j, int x, int y){
  alarm.flagOngoing = true;
  alarm.Repetition = j;
  alarm.OnInterval = x;
  alarm.OffInterval = y;
  // int i = 0;
  // for(i;i<j;i++){
  // digitalWrite(buzz, HIGH);
  //   delay(x);
  //   digitalWrite(buzz, LOW);
  //   delay(y);
  // }
}

void Alarm() {
  if (alarm.flagOngoing) {
    if (alarm.Repetition > 0) {
      unsigned long currentMillisAlarm = millis();
      if (alarm.ptrCurInterval == nullptr) {    // check if it's the first time triggering the alarm
          previousMillisAlarm = currentMillisAlarm;
          analogWrite(buzz, 255);
          alarm.ptrCurInterval = &alarm.OnInterval;
          
          #ifndef NBEEPDEBUG
          Serial.println("*BEEEEEEEEEP*");
          #endif
      } 
      else if (currentMillisAlarm - previousMillisAlarm >= *alarm.ptrCurInterval) {
        previousMillisAlarm = currentMillisAlarm;
        
        if (alarm.ptrCurInterval == &alarm.OnInterval) {
          analogWrite(buzz, 0);
          alarm.ptrCurInterval = &alarm.OffInterval;
          alarm.Repetition--;

          #ifndef NBEEPDEBUG
          Serial.println("**");
          #endif
        }
        else {
          analogWrite(buzz, 255);
          alarm.ptrCurInterval = &alarm.OnInterval;

          #ifndef NBEEPDEBUG
          Serial.println("*BEEEEEEEEEP*");
          #endif
        }
      }
    } else {
      analogWrite(buzz, 0);
      alarm.flagOngoing = false;
      alarm.ptrCurInterval = nullptr;

      #ifndef NBEEPDEBUG
      Serial.println("**");
      #endif
    }
  }
}

void Gyro() {
  unsigned long currentMillisGyro = millis();
  if (currentMillisGyro - previousMillisGyro >= GYROINTERVAL) {
    previousMillisGyro = currentMillisGyro;
    
    recordGyroRegisters();
    gyroBuffer.insert(rot);

    if (gyroBuffer.getCount() == GYROBUFFERSIZE) {
      Coords<float> avg {};
      for (int i = 0; i<GYROBUFFERSIZE; i++)
        avg += gyroBuffer[i];
      avg = avg/GYROBUFFERSIZE;

      if (alarm.flagEnabled) {
        activateAlarm(avg); // rename this func name?
      }
      #ifndef NALARMDEBUG
      printCoords(avg);
      #endif
    }
  }
}


void Tilt() {
  recordAccelRegisters();
  
  if (gForce.X > 0.8 || gForce.Y > 0.8 || gForce.Z > 0.8) {
    isAccident = true;
    previousMillisResponse = millis();
  }
  
  #ifndef NACCELDEBUG
  printCoords(gForce);
  #endif
}

template <typename T>
void printCoords(Coords<T> coords) {
  Serial.print(coords.X);
  Serial.print('\t');
  Serial.print(coords.Y);
  Serial.print('\t');
  Serial.println(coords.Z);
}


// +CMT: "+639503610262","","19/08/23,21:13:18+32"
// Tndnxnc
// ===============
// UNDER-VOLTAGE WARNNING
