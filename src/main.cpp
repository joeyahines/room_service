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

#define RST_PIN         9           
#define SS_PIN          10          
MFRC522 mfrc522(SS_PIN, RST_PIN);

int target_room = 1;

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

void follow_line(int16_t distance_from_line)
{
    //Base power of the motors
    unsigned int base_power = BASE_MOTOR_POWER;
    //Change in the base power
    int power;

    //Power of each motor
    int right_motor_power;
    int left_motor_power;

    int forward;

    if (distance_from_line < 0) {
        distance_from_line = distance_from_line * -1;
        forward = 0;
    }
    else {
        forward = 1;
    }

    //Find how much how power is needed based of the distance
    power = ((((uint16_t) distance_from_line << 2) / (uint16_t) 3000)
            * (base_power)) >> 2;

    //Set the power based off which way needs to be corrected
    if (!forward)
    {
        right_motor_power = base_power - power;
        left_motor_power = base_power + power;
    }
    else
    {
        right_motor_power = base_power + power;
        left_motor_power = base_power - power;
    }
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
  Serial.println("Rock and Roll");
  delay(5000);
  digitalWrite(LED_BUILTIN, HIGH);
}

void stop() {
    set_left_motor_power(0);
    set_right_motor_power(0);
    digitalWrite(LED_BUILTIN, LOW);
    while (true)
    {
    }
}

int missing_count = 0;
void loop() {
  if (card_detected()) {
    int room_number = get_room_number();

#ifdef DEBUG
    Serial.println(F("**Card Detected:**"));
#endif

    if (room_number == target_room) {
      set_left_motor_power(30);
      set_right_motor_power(0);
      bool new_line_found = false;

      while (!new_line_found) {
        uint8_t line_reading  = line_follower.get_line_reading(10000);

        new_line_found =  line_reading & 0b1100000;
      }
      set_left_motor_power(BASE_MOTOR_POWER);
      set_right_motor_power(BASE_MOTOR_POWER);
    //Serial.print("Room number: ");
    //Serial.println(room_number);
    }
    
  }
  else {
    uint8_t line_reading  = line_follower.get_line_reading(10000);

    if (line_reading == 0) {
      missing_count++;
      if (missing_count > 50) {
        stop();
      }
    }
    else {
      missing_count = 0;
    }

    int16_t distance = line_follower.get_distance(line_reading);
    follow_line(distance);  

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
}