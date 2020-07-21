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
#define BASE_MOTOR_POWER 20
//#define DEBUG 

LineFollower line_follower;
int lastError = 0;
int olddir = 0;

#define RST_PIN 9           
#define SS_PIN 10          
MFRC522 mfrc522(SS_PIN, RST_PIN);

int target_room = 2;

enum DELIVERY_STATUS {
  FINDING_ROOM,
  ROOM_FOUND,
  RETURN_TO_HALLWAY,
  STRAIGHTEN,
  RETURN,
  RESET
};

enum DELIVERY_STATUS delivery_status = FINDING_ROOM;

uint8_t room_number = 1;
uint8_t cross = 0;

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

int32_t prev_error = 0;
int32_t integral = 0;

void follow_line(int16_t distance_from_line, bool reverse) {
    //Base power of the motors
    unsigned int base_power = BASE_MOTOR_POWER;
    //Change in the base power
    int power;
    int rev;

    if (reverse) {
      rev = -1;
    }
    else {
      rev = 1;
    }

    int16_t error = -distance_from_line;
    integral = (integral + error)/3;
    int16_t derivative = (error - prev_error)/3;
    float kp = 0.7;
    float kd = 0.0;
    float ki = 0.0;
    power = kp * error + ki * integral + kd * derivative;
    prev_error = error;

    if (power > 50 - BASE_MOTOR_POWER) {
      power = 50 - BASE_MOTOR_POWER;
    }
    else if (power < -BASE_MOTOR_POWER) {
      power = -BASE_MOTOR_POWER;
    }

    int right_motor_power = rev*(base_power - power);
    int left_motor_power = rev*(base_power + power);
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

void stop() {
    Serial.println("Stopping...");
    set_left_motor_power(0);
    set_right_motor_power(0);
    bool state = false;
    while (true)
    {
      if (state) {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else {
        digitalWrite(LED_BUILTIN, LOW);
      }
      state = !state;
      delay(1000);
    }
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
      cross = 0;
      if (delivery_status == FINDING_ROOM) {
        delivery_status = ROOM_FOUND;
      }
      else if (delivery_status == RETURN) {
        delivery_status = RESET;
      }
    }
  }

  uint8_t line_reading  = line_follower.get_line_reading(SENSITIVITY);
  int16_t distance = line_follower.get_distance(line_reading);


  switch (delivery_status) {
    case RESET:
      set_left_motor_power(BASE_MOTOR_POWER);
      set_right_motor_power(-45);

      if ((line_reading & 0b00001110) && cross > 1) {
        stop();
      }
      else {
        cross++;
      }
      break;
    case RETURN:
      target_room = 3;
    case FINDING_ROOM:
      follow_line(distance, false);
      break;
    case ROOM_FOUND:
      set_left_motor_power(BASE_MOTOR_POWER);
      set_right_motor_power(0);

      if (line_reading & 0b11000000) {
        cross = 1;
      }
      if ((line_reading & 0b00111100) && cross > 0) {
        set_left_motor_power(0);
        set_right_motor_power(0);
        delay(5000);
        delivery_status = RETURN_TO_HALLWAY;
        cross = 0;
      }
      break;
    case RETURN_TO_HALLWAY:
      set_left_motor_power(BASE_MOTOR_POWER);
      set_right_motor_power(0);
      if (line_reading & 0b11000000) {
        cross = 1;
      }
      else if ((line_reading & 0b11110000) && cross > 0) {
        set_left_motor_power(0);
        set_right_motor_power(0);
        delivery_status = STRAIGHTEN;
      }

    break;
    case STRAIGHTEN:
      set_left_motor_power(0);
      set_right_motor_power(35);
      if (line_reading & 0b00111100) {
        delay(100);
        set_left_motor_power(0);
        set_right_motor_power(0);
        delivery_status = RETURN;
      }
      break;
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

#ifndef DEBUG
  int end_time = millis();

  if ((end_time - start_time) < 3) {
    delay(3 - (end_time - start_time));
  }
#else
  delay(500);
#endif
}