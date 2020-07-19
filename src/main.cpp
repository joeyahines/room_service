// Room Service
// 
// Robot room delivery system prototype
//
// Created by: Joey Hines


#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <LineFollower.h>

#define SENSITIVITY 10000
#define BASE_MOTOR_POWER 25
#define DEBUG 

LineFollower line_follower;
int lastError = 0;
int olddir = 0;

#define RST_PIN         9           
#define SS_PIN          10          
MFRC522 mfrc522(SS_PIN, RST_PIN);

int target_room = 3;

enum STATE
{
    STRAIGHT,
    LEFT_TURN,
    RIGHT_TURN,
    DONE,
    TURN_AROUND,
};

uint8_t room_number = 1;
enum STATE last_turn = LEFT_TURN;

void motor_setup() {
  pinMode(PIN2, OUTPUT);  
  pinMode(PIN3, OUTPUT);
  pinMode(PIN4, OUTPUT);     
  pinMode(PIN5, OUTPUT);  
  pinMode(PIN6, OUTPUT);
  pinMode(PIN7, OUTPUT);     
  pinMode(LED_BUILTIN, OUTPUT);     
}

void set_motor_power(float power, int pwm_pin, int in1, int in2) {
  int pwm_value = power * 2.55;

  if (pwm_value < -10) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, -pwm_value);
    
  }
  else if (pwm_value > 10) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm_pin, pwm_value);
  } 
  else {
    analogWrite(pwm_pin, 0);
  }     
}

void set_left_motor_power(float power) {
  set_motor_power(power, PIN3, PIN2, PIN4);
}

void set_right_motor_power(float power) {
  set_motor_power(power, PIN5, PIN6, PIN7);
}

bool card_detected() {
  return mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial();
  Serial.println("Card detected");
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

int16_t prev_error = 0;
int16_t integral = 0;

void follow_line(int16_t distance_from_line) {
    //Base power of the motors
    unsigned int base_power = BASE_MOTOR_POWER;
    //Change in the base power
    int power;

    int16_t error = -distance_from_line;
    integral = (integral + error)/5;
    int16_t derivative = (error - prev_error)/5;
    float kp = 2.0;
    float kd = 0.0;
    float ki = 0.0;
    power = kp * error + ki * integral + kd * derivative;
    prev_error = error;

    if (power > 100 - BASE_MOTOR_POWER) {
      power = 100 - BASE_MOTOR_POWER;
    }
    else if (power < -100 - BASE_MOTOR_POWER) {
      power = -100 - BASE_MOTOR_POWER;
    }

    int right_motor_power = base_power - power;
    int left_motor_power = base_power + power;
#ifdef DEBUG
    //Set motor power
    Serial.print("Motors: ");
    Serial.print(left_motor_power);
    Serial.print(", ");
    Serial.print(right_motor_power);
    Serial.println();
#endif
set_right_motor_power(right_motor_power);
set_left_motor_power(left_motor_power);
}

void setup() {
  Serial.begin(9600);   
  Serial.println("Setting up Arduino!");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("Setting up motors!");
  motor_setup();
  set_left_motor_power(0);
  set_right_motor_power(0);

  Serial.println("Intializing MFRC");
  SPI.begin();   
  mfrc522.PCD_Init();
  delay(5000);
  Serial.println("Rock and Roll");
  digitalWrite(LED_BUILTIN, HIGH);
}

void stop() {
    Serial.println("Stopping...");
    set_left_motor_power(0);
    set_right_motor_power(0);
    bool state = false;
    while (true)
    {
      if (state) {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println(state);
      }
      else {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println(state);
      }
      state = !state;
      delay(500);
    }
}

int missing_count = 0;
void loop() {
  int start_time = millis();
  if (card_detected()) {
    int room_number = get_room_number();

#ifdef DEBUG
    Serial.println(F("**Card Detected:**"));
    Serial.print("Room number: ");
    Serial.println(room_number);
#endif

    if (room_number == target_room) {
      set_left_motor_power(30);
      set_right_motor_power(0);
      bool new_line_found = false;

      while (!new_line_found) {
        uint8_t line_reading  = line_follower.get_line_reading(SENSITIVITY);

        new_line_found =  line_reading & 0b1100000;
      }
      set_left_motor_power(BASE_MOTOR_POWER);
      set_right_motor_power(BASE_MOTOR_POWER);
    }
    
  }
  else {
    uint8_t line_reading  = line_follower.get_line_reading(SENSITIVITY);
    int16_t distance = line_follower.get_distance(line_reading);
    if (line_reading == 0) {
      missing_count++;
      if (missing_count > 100) {
        stop();
      }
    }
    else {
      missing_count = 0;
      follow_line(distance);  
    }

#ifdef DEBUG
    Serial.print("Line follower: ");
    for (int i = 7; i >= 0; i--) {
      Serial.print((line_reading >> i) & 0x1);
    }
    Serial.print(", ");
    Serial.print(distance);
    Serial.println();
#endif

  }
#ifndef DELAY
  int end_time = millis();

  if ((end_time - start_time) < 5) {
    delay(5 - (end_time - start_time));
  }
#else
  delay(500);
#endif
}