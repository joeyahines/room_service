
#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN         9           
#define SS_PIN          10          
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

void motor_setup() {
  pinMode(PIN2, OUTPUT);  
  pinMode(PIN3, OUTPUT);
  pinMode(PIN4, OUTPUT);     
  pinMode(PIN5, OUTPUT);  
  pinMode(PIN6, OUTPUT);
  pinMode(PIN7, OUTPUT);     
  pinMode(LED_BUILTIN, OUTPUT);     
}

void set_motor_speed(float power, int pwm_pin, int in1, int in2) {
  int pwm_value = power * 2.55;

  if (pwm_value < -5) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, -pwm_value);
    
  }
  else if (pwm_value > 5) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm_pin, pwm_value);
  } 
  else {
    analogWrite(pwm_pin, 0);
  }     
}

void set_left_motor_speed(float power) {
  set_motor_speed(power, PIN3, PIN2, PIN4);
}

void set_right_motor_speed(float power) {
  set_motor_speed(power, PIN5, PIN6, PIN7);
}

bool card_detected() {
  return mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial();
}

int get_room_number() {
  byte block;
  byte len;
  byte buffer[18];
  MFRC522::StatusCode status;
  Serial.print(F("Room Number: "));

  block = 5;
  len = 18;

  status = mfrc522.MIFARE_Read(block, buffer, &len);
  if (status != MFRC522::STATUS_OK) {
    Serial.print(F("Reading failed: "));
    Serial.println(mfrc522.GetStatusCodeName(status));
    return -1;
  }

  return buffer[3];
}

void setup() {
  Serial.begin(9600);   
  Serial.println("Setting up Arduino!");

  Serial.println("Setting motor speed!");
  motor_setup();
  set_left_motor_speed(0);
  set_right_motor_speed(0);

  Serial.println("Intializing MFRC");
  SPI.begin();   
  mfrc522.PCD_Init();
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
}

int state = LOW;
void loop() {
  if (card_detected()) {
    Serial.println(F("**Card Detected:**"));
    int room_number = get_room_number();

    Serial.print("Room number: ");
    Serial.println(room_number);
  }
}