#include <LiquidCrystalRus.h>
#include <ServoSmooth.h>
ServoSmooth servo;
ServoSmooth servo2;
#include <IRremote.h>
#include <iarduino_DHT.h>
#include <SparkFun_TB6612.h>  
#include <SPI.h>
#include <MFRC522.h>

#define PIN_TRIG 2
#define PIN_ECHO 3
#define PIN_TRIG2 17
#define PIN_ECHO2 1
#define IR_RECEIVE_PIN A1
#define DHTPIN 32
#define SS_PIN 53
#define RST_PIN 49
#define AIN1 22
#define holl A0
#define AIN2 23
#define PWMA 14
#define STBY 15
#define light_sensor A7
#define relay 10  

const int rs = 26, en = 27, d4 = 28, d5 = 29, d6 = 30, d7 = 31;
LiquidCrystalRus lcd(rs, en, d4, d5, d6, d7);

const int offsetA = 1;
const int offsetB = 1;
const int led = 13;
const int pirPin = 9;
const int pirPin2 = 10;
int sensorValue = 0;
int sensorValue2 = 0;
int hollValue = 0;
int closing = 10;
int opening = 120;

const int buttonOn = 6;
const int piezo = 44;
const int buttonOff = 18;
int val = 0;
long duration, cm;
long duration2, cm2;
unsigned long prev_millis = 0;
unsigned long prev_millis2 = 0;
unsigned long prev_millis_hc = 0;
unsigned long prev_millis_servo = 0;
unsigned long prev_millis_hc2 = 0;
unsigned long prev_millis_servo2 = 0;

bool isSafeMode = false;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

iarduino_DHT dht(DHTPIN);

MFRC522 mfrc522(SS_PIN, RST_PIN);
byte uidCard[4] = {0xC3, 0xA2, 0x60, 0x04};

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_TRIG2, OUTPUT);
  pinMode(PIN_ECHO2, INPUT);

  servo.attach(5);        // подключить
  servo.setSpeed(250);    // ограничить скорость
  servo.setAccel(1.5);       // установить ускорение (разгон и торможение)
  servo2.attach(8);        // подключить
  servo2.setSpeed(250);    // ограничить скорость
  servo2.setAccel(1.5); 
  pinMode(light_sensor, INPUT);
  pinMode(relay, OUTPUT);
  pinMode(A1, INPUT); // пин A1 будет входом (англ. «input»)
  pinMode( pirPin, INPUT );
  pinMode( pirPin2, INPUT);
  pinMode( led, OUTPUT );
  pinMode( buttonOn, INPUT );
  pinMode( buttonOff, INPUT );
  pinMode( holl, INPUT );
  SPI.begin();
  mfrc522.PCD_Init();
  Serial.begin(9600);
  lcd.begin(16, 2);
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}


void dhtsensor(){
  if(!isSafeMode){
    if(millis() - prev_millis2 >= 2000){
      lcd.clear();
      prev_millis2 = millis();
      dht.read();
      int h = dht.hum;
      int t = dht.tem;
      Serial.println(t);
      Serial.println(h);
      lcd.setCursor(0,0);
      lcd.print("Temperature: ");
      lcd.setCursor(14, 0);
      lcd.print(t);
      lcd.setCursor(0,1);
      lcd.print("Humidity: ");
      lcd.setCursor(11,1);
      lcd.print(h);
      if(t > 35 || h > 75){
        motor1.drive(1500, 10000);
        motor1.brake();
      }
    }
  }
}


void printSecMode(){
  if(isSafeMode){
    lcd.clear();
    lcd.print("Security Mode");
  }
}


void doorOpenClose(){
  if(!isSafeMode){
    servo.tick();
    servo2.tick();
    if(millis()- prev_millis_hc >= 250){
      prev_millis_hc = millis();
      digitalWrite(PIN_TRIG, LOW);
      delayMicroseconds(5);
      digitalWrite(PIN_TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(PIN_TRIG, LOW);
      duration = pulseIn(PIN_ECHO, HIGH);
      cm = (duration / 2) / 29.1;
      if(cm <= 10){servo.setTargetDeg(opening);}
      else{servo.setTargetDeg(closing);}
    }
    if(millis()- prev_millis_hc2 >= 250){
      prev_millis_hc2 = millis();
      digitalWrite(PIN_TRIG2, LOW);
      delayMicroseconds(5);
      digitalWrite(PIN_TRIG2, HIGH);
      delayMicroseconds(10);
      digitalWrite(PIN_TRIG2, LOW);
      duration2 = pulseIn(PIN_ECHO2, HIGH);
      cm2 = (duration2 / 2) / 29.1;
      if(cm2 <= 10){servo2.setTargetDeg(opening);}
      else{servo2.setTargetDeg(closing);}
    }
  }
}


void light_sensor(){
  int light_val = analogRead(light_sensor);
  if (light_val > 500) {
    digitalWrite(relay, HIGH);
  } else {
    digitalWrite(relay, LOW);
  }
}

void irReceive(){
  if (IrReceiver.decode()) // если данные пришли выполняем команды
  {
    unsigned long res = IrReceiver.decodedIRData.command;
    Serial.println(res); // отправляем полученные данные на порт

//    if (res == 3) {
//      motor2.drive(50, 2000);
//      motor2.drive(-50, 2000);
//      motor2.brake();
//    }

    IrReceiver.resume(); // принимаем следующий сигнал на ИК приемнике
  }
}


void securityMode(){
  if (digitalRead(buttonOn) == HIGH && !isSafeMode) {
    isSafeMode = true;
    Serial.println("Security Mode is ON");
    printSecMode();
    while (isSafeMode) {
      if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
        // запишем метку в 4 байта
        for (byte i = 0; i < 4; i++) {
          // Если идентификатор не совпадает с заданным номером
          if (uidCard[i] != mfrc522.uid.uidByte[i]) {
            Serial.println("У вас нет доступа");
          } else {
            Serial.println("Проходите");
            lcd.clear();
            lcd.print("Welcome");
            isSafeMode = false;
            Serial.println("Security Mode is OFF");
            delay(1000);
          }

        }
      }
      sensorValue = digitalRead(pirPin);
      sensorValue2 = digitalRead(pirPin2);
      hollValue = analogRead(holl);
      if (sensorValue == 1 || hollValue > 1000 || sensorValue2 == 1) {
        Serial.println("Дома кто-то есть!");
        Serial.println(hollValue);
        tone(piezo, 1000, 500);
        digitalWrite(led, HIGH);
      }
      else {
        digitalWrite(led, LOW);
        printSecMode();
      }
      if (digitalRead(buttonOff) == HIGH && isSafeMode) {
        isSafeMode = false;
        Serial.println("Security Mode is OFF");
      }
    }
  }
}


void loop() {
  
  light_sensor();
  
  irReceive();

  securityMode();

  dhtsensor();
  
  doorOpenClose();
}
