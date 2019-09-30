#define debugRelay
#define enSMS
//#define mulSMS

#ifdef mulSMS
  String num1 = "+639XXXXXXXXXX", num2 = "+639XXXXXXXXX";
#endif

#include <LiquidCrystal_I2C.h>
#include<EEPROM.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define sim Serial1
#define buzz 13

RF24 radio(7, 8); // CE, CSN
LiquidCrystal_I2C lcd(0x27, 16, 2);

struct LCD{
  int counter = 30;
  bool mode = true;
} lcdDisplay;

float freq;
unsigned long prevSMS = millis();
bool tpin1 = false, tpin2 = false, livePower = true, newSMS = false;
int distance = 0, compensate = EEPROM.read(1) + EEPROM.read(2), wLevel = 0, radioConnection = 10;
byte power_int = EEPROM.read(0), alert = 0, socket1 = 10, socket2  = 12, prevAlert = 0;
const byte address[6] = "00001";
bool power = true, prev_state = true;
String textMessage, number = "+639283128714", freqString;
char str[32],str2[32];


void setup() {
  Serial.begin(9600);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
//  pinMode(buzz, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(10, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(buzz, LOW);
  lcd.print("Booting Up . . .");
  Serial.println("Booting Up . . .");
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
  Serial.println("listening");
  sim.begin(9600);
  delay(1000);
  sim.print("AT+ CMGF=1\r");
  delay(1000);
  sim.print("AT+CNMI=2,2,0,0,0\r");
  delay(1000);
  sim.print("AT+CMGD=1,3\r");
  delay(1000);
  beep(1, 200, 100);
  beep(1, 100, 0);
  send_msg("Flood detection started.", number);

}

void loop() {
  getWlevel();
//  Serial.println(wLevel);
  action();
  recv_msg();
  powerDetection();
  lcdPrint();
  if(livePower){
    digitalWrite(socket1, !tpin1);
    digitalWrite(socket2, !tpin2);
  }
  
#ifdef debugRelay
  Serial.println(livePower);
  if (Serial.available()){
    if(Serial.read()=='a') {
      tpin1 = true;
      tpin2 = true;
      digitalWrite(socket1, !tpin1);
      digitalWrite(socket2, !tpin2);
    }
    if(Serial.read()=='s'){
      beep(3, 75, 50);
      beep(2, 250, 50);
      beep(3, 75, 50);
    }
  }
  Serial.println(wLevel);
#endif
}

void recv_msg() {
//  sim.println("at+cmgf?");
//  sim.print("AT+CNMI=2,2,0,0,0\r");
  delay(100);
  if (sim.available()) {
    textMessage = sim.readString();
    delay(100);
//    Serial.println(textMessage);
    textMessage.toLowerCase();
    if (textMessage.indexOf("+cmgf: 0") > 0 || textMessage.indexOf("+cmti") > 0 ) {
      sim.print("AT+ CMGF=1\r");
      beep(2, 50, 50);
      sim.print("AT+CNMI=2,2,0,0,0\r");
    }
    if (textMessage.indexOf("+cmt") > 0 || textMessage.indexOf("+cmti") > 0) {
      beep(3, 75, 50);
      beep(2, 250, 50);
      beep(3, 75, 50);
      newSMS = true;
      prevSMS = millis();
      sim.print("AT+CMGD=1,3\r"); //deletes recv read sms
    }
    if (textMessage.indexOf("off1") > 0) {
      tpin1 = false;
      sprintf(str,"  Socket 1 OFF  ");
#ifdef debugSMS
      Serial.println("Socket 1 turned OFF");
#endif
    } else if (textMessage.indexOf("on1") > 0) {
      if(alert == 3){
        send_msg("Can't turn on socket 1. Water level is at RED status.", number);
      }else if(!livePower){
        send_msg("Can't turn on socket 1. There's a power interruption, this is to protect appliances from voltage surge.", number);
      }else{
        tpin1 = true;
        sprintf(str,"  Socket 1 ON   ");
      }
#ifdef debugSMS
      Serial.println("Socket 1 turned ON");
#endif
    } else if (textMessage.indexOf("off2") > 0) {
      tpin2 = false;
      sprintf(str,"  Socket 2 OFF  ");
#ifdef debugSMS
      Serial.println("Socket 2 turned OFF");
#endif
    } else if (textMessage.indexOf("on2") > 0) {
      if(alert == 3){
        send_msg("Can't turn on socket 2. Water level is at RED status.", number);
      }else if(!livePower){
        send_msg("Can't turn on socket 2. There's a power interruption, this is to protect appliances from voltage surge.", number);
      }else{
        tpin2 = true;
        sprintf(str,"  Socket 2 ON   ");
      }
#ifdef debugSMS
      Serial.println("Socket 2 turned ON");
#endif
    } else if (textMessage.indexOf("power inq") > 0) {
      if (livePower) {
        send_msg("Electric power is available. " + String(power_int) + " power interruption occured.", number);
      } else {
        send_msg("Power interruption still on going. " + String(power_int) + " power interruption occured.", number);
      }
      sprintf(str, " Power Inquiry. ");
#ifdef debugSMS
      Serial.println("Power Inquiry");
#endif
    } else if (textMessage.indexOf("calibrate") > 0) {
      calibrate();
      sprintf(str,"Calibrating.....");
#ifdef debugSMS
      Serial.println("Calibrating ultrasonic sensor");
#endif
    }else if (textMessage.indexOf("reset") > 0){
      resetCounter();
      sprintf(str,"Resetting.......");
    }
  }

}

void send_msg(String txt, String number) {
#ifdef enSMS
  sim.print("AT+CMGF=1\r");
  delay(100);
  sim.println("AT+CMGS=\"" + number + "\"");
  delay(100);
  sim.println(txt);
  delay(100);
  sim.println((char)26);
  delay(1000);
  sim.println();
  #ifdef mulSMS
    sim.println("AT+CMGS=\"" + number + "\"");
    delay(100);
    sim.println(txt);
    delay(100);
    sim.println((char)26);
    delay(1000);
    sim.println();
    sim.println("AT+CMGS=\"" + number + "\"");
    delay(100);
    sim.println(txt);
    delay(100);
    sim.println((char)26);
    delay(1000);
    sim.println();
  #endif
  textMessage = "";
#endif
}

void beep(int x, int t1, int t0) {
  for (int i = 1; i <= x; i++) {
    analogWrite(13, 255);
    delay(t1);
    digitalWrite(13, LOW);
    delay(t0);
  }
  delay(150);
}

void calibrate() {
  EEPROM.update(1, distance);
  delay(100);
  compensate = EEPROM.read(1);
}

void lcdPrint() {
  lcd.setCursor(0, 0);
  if(radioConnection<1){
    lcd.print("Radio connection");
    lcd.setCursor(0,1);
    lcd.print("Rx error . . . .");
  }else{
    if(livePower){
      if(!newSMS) sprintf(str, "Water Level  %3d", wLevel);
      freqString = "Frequency " + String(freq, 3);
      lcd.setCursor(0, 1);
      lcd.print(freqString);
    }else{
      if(!newSMS) sprintf(str, "    No Power    ");
      sprintf(str2, " Power Intr %3d ", int(power_int));
      lcd.setCursor(0, 1);
      lcd.print(str2);
    }
    lcd.setCursor(0,0);
    lcd.print(str);
    if(millis()-prevSMS>4000){
      newSMS = false;
    }
  }
}

void powerInterruption() {
  if (power_int >= 255) {
    power_int = 0;
  }
  EEPROM.update(0, power_int += 1);
  delay(100);
}

void resetCounter() {
  EEPROM.update(0, 0);
  delay(100);
  power_int = EEPROM.read(0);
}

void getWlevel() {
  if(radioConnection>0){
    radioConnection--;
  }
  while(radio.available()) {
    radio.read(&distance, sizeof(distance));
    wLevel = (compensate - distance);
    if(livePower){
      radioConnection = 10;
    }else{
      radioConnection = 10;
    }
  }
}

void action() {
  if (wLevel > 24) {
    alert = 3;
    digitalWrite(10, HIGH);
    digitalWrite(12, HIGH);
  } else if (wLevel <= 23 && wLevel >= 20) {
    alert = 2;
  } else if (wLevel <=19 && wLevel >= 15) {
    alert = 1;
  } else {
  alert = 0;
  }
  bool smsBool;
  if(prevAlert!= alert){
    prevAlert = alert;
    smsBool = true;
    beep(5, 500, 1000);
  }else{
    smsBool = false;
  }
  
  if (alert == 3) {
  tpin1 = false;
  tpin2 = false;
  if(smsBool) send_msg("Water level is at RED status. Sockets are turned off.", number);
  } else if (alert == 2) {
    if(smsBool) send_msg("Water Level Orange Warning", number);
  } else if (alert == 1) {
    if(smsBool) send_msg("Water Level Yellow Warning", number);
  } else if (alert == 0) {
    if(smsBool) send_msg("Water level is at normal state", number);
  }
}

void powerDetection() {
  int low_time, high_time;
  high_time = pulseIn(9, HIGH, 100000);
  low_time = pulseIn(9, LOW, 100000);
  freq = (high_time + low_time)/275.5;
  freqString = String(freq,3);
  Serial.println(freqString);
  if (low_time > 1) {
    livePower = true;
  } else {
    livePower = false;
    digitalWrite(socket1, HIGH);
    digitalWrite(socket2, HIGH);
  }
  if (prev_state != livePower) {
    if (!livePower) {
      Serial.println("Power Interruption Detected");
      powerInterruption();
      beep(5, 100, 50);
      digitalWrite(10, HIGH);
      digitalWrite(12, HIGH);
      send_msg("Power Interruption Detected", number);
    } else {
      digitalWrite(10, HIGH);
      digitalWrite(12, HIGH);
      sprintf(str, " Power Resumed. ");
      lcd.setCursor(0, 0);
      lcd.print(str);
      beep(1, 2000, 10);
      Serial.println(" Power Resumed ");
      send_msg("Power Resumed", number);
      
      delay(5000);
      digitalWrite(socket1, !tpin1);
      digitalWrite(socket2, !tpin2);
    }
  }

  prev_state = livePower;
}
