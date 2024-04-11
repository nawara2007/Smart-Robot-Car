#define IR_RECEIVE_PIN A0
#define NO_LED_FEEDBACK_CODE

#include <MotorDriver.h>
#include "TinyIRReceiver.hpp"
#include <Servo.h>
#include <NewPing.h>

MotorDriver m;

const uint64_t FORWARD_CODE = 0x1B;
const uint64_t BACKWARD_CODE = 0xF;
const uint64_t LEFT_CODE = 0xC;
const uint64_t RIGHT_CODE = 0xE;
const uint64_t STOP_CODE = 0xD;
const uint64_t TOP_RIGHT_CODE = 0x1F;
const uint64_t TOP_LEFT_CODE = 0xA;
const uint64_t BOTTOM_RIGHT_CODE = 0x19;
const uint64_t BOTTOM_LEFT_CODE = 0x0;

const uint64_t minus_CODE = 0x6;
const uint64_t plus_CODE = 0x5;

const uint64_t line_tracking_CODE = 0x2;
const uint64_t obstacle_avoiding_CODE = 0x3;

#define LineTeacking_Pin_Right A5
#define LineTeacking_Pin_Middle 2
#define LineTeacking_Pin_Left A3
#define LineTeacking_Read_Right digitalRead(A5) //Right
#define LineTeacking_Read_Middle digitalRead(2) //Middle
#define LineTeacking_Read_Left digitalRead(A3) 

#define trig_pin A2
#define echo_pin A4
#define MAX_DISTANCE 400

#define front_lamps 13
#define back_lamps A1

unsigned long IR_PreMillis;
unsigned long LT_PreMillis;
unsigned long OA_PreMillis;

int distanceLeft;
int distanceRight;
int distanceforward;
bool counting = false;

NewPing sonar(trig_pin, echo_pin, MAX_DISTANCE);

Servo myservo;

int speed = 200;

#if !defined(STR_HELPER)
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#endif

enum FUNCTIONMODE
{
  LineTeacking,          /*Line Teacking Mode*/
  ObstaclesAvoidance,    /*Obstacles Avoidance Mode*/
  IRremote,              /*Infrared Control Mode*/
} func_mode = IRremote;

enum MOTIONMODE
{
  Left,    /*left*/
  Right,   /*right*/
  Forward, /*forward*/
  Back,    /*back*/
  Stop,    /*stop*/
  Left_forward,
  Left_back,
  Right_forward,
  Right_back,
} mov_mode = Stop; /*move mode*/

static boolean function_xxx(long xd, long sd, long ed) //f(x)
{
  if (sd < xd && xd < ed)
    return true;
  else
    return false;
}

void delays(unsigned long t)
{
  for (unsigned long i = 0; i < t; i++)
  {
    getIRData();      //Infrared Communication Data Acquisition
    delay(1);
  }
}

void changespeed(int new_speed){
  speed += new_speed;
  if(speed < 70){
    speed = 70;
  }
  if(speed > 255){
    speed = 255;
  }
}

void getIRData(){
  if (TinyIRReceiverData.justWritten) {
    TinyIRReceiverData.justWritten = false;

    Serial.print(F("Command=0x"));
    Serial.println(TinyIRReceiverData.Command, HEX);
    // Print the received IR code (for debugging purposes)

    IR_PreMillis = millis();

    int results = TinyIRReceiverData.Command;
    // Check the received IR code and control the motors accordingly
    switch (results) {
      case FORWARD_CODE:
        func_mode = IRremote;
        mov_mode = Forward;
        break;
      case BACKWARD_CODE:
        func_mode = IRremote;
        mov_mode = Back;
        break;
      case LEFT_CODE:
        func_mode = IRremote;
        mov_mode = Left;
        break;
      case RIGHT_CODE:
        func_mode = IRremote;
        mov_mode = Right;
        break;
      case STOP_CODE:
        func_mode = IRremote;
        mov_mode = Stop;
        break;
      case TOP_RIGHT_CODE:
        func_mode = IRremote;
        mov_mode = Right_forward;
        break;
      case TOP_LEFT_CODE:
        func_mode = IRremote;
        mov_mode = Left_forward;
        break;
      case BOTTOM_RIGHT_CODE:
        func_mode = IRremote;
        mov_mode = Right_back;
        break;
      case BOTTOM_LEFT_CODE:
        func_mode = IRremote;
        mov_mode = Left_back;
        break;
      case plus_CODE:
        changespeed(10);
        break;
      case minus_CODE:
        changespeed(-10);
        break;
      case obstacle_avoiding_CODE:
        func_mode = ObstaclesAvoidance;
        break;
      case line_tracking_CODE:
        func_mode = LineTeacking;
        break;
    }
  } 
}

void irremote_mode(void)
{
  if (func_mode == IRremote)
  {
    switch (mov_mode)
    {
    case Forward:
      goForward(speed);
      break;
    case Back:
      goBackward(speed);
      break;
    case Left:
      goLeft(speed);
      break;
    case Right:
      goRight(speed);
      break;
    case Stop:
      stopMotors();
      break;
    case Right_forward:
      goTop_Right(speed);
      break;
    case Left_forward:
      goTop_Left(speed);
      break;
    case Right_back:
      goBack_Right(speed);
      break;
    case Left_back:
      goBack_Left(speed);
      break;
    }
    if (millis() - IR_PreMillis > 500)
    {
      mov_mode = Stop;
      IR_PreMillis = millis();
    }
  }
}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Car V2!");
  ServoControl(90);
  pinMode(LineTeacking_Pin_Left, INPUT);
  pinMode(LineTeacking_Pin_Right, INPUT);
  pinMode(LineTeacking_Pin_Middle, INPUT);
  pinMode(A5, INPUT);
  pinMode(front_lamps, OUTPUT);
  pinMode(back_lamps, OUTPUT);

  if (!initPCIInterruptForTinyReceiver()) {
        Serial.println(F("No interrupt available for pin " STR(IR_RECEIVE_PIN))); // optimized out by the compiler, if not required :-)
  }
}

void loop() {
  getIRData();
  irremote_mode();
  obstacles_avoidance_mode();
  line_teacking_mode();
}

void obstacles_avoidance_mode(void)
{
  static boolean first_is = true;
  if (func_mode == ObstaclesAvoidance)
  {
    if (first_is == true) //Enter the mode for the first time, and modulate the steering gear to 90 degrees
    {
      ServoControl(90);
      first_is = false;
    }

    distanceforward = sonar.ping_cm();

    if (function_xxx(distanceforward, 0, 25))
    {
      stopMotors();

      ServoControl(15);
      distanceRight = sonar.ping_cm();
      
      ServoControl(165);
      distanceLeft = sonar.ping_cm();

      goBackward(250);
      delays(500);
      if(distanceforward >= distanceLeft)
      {
        goRight(250);
        delays(500);
        stopMotors();
      } else {
        goLeft(250);
        delays(500);
        stopMotors();
      }
    }
    else
    {
      goForward(180); //Control car forwar
    }
    
    ServoControl(90);
  }
  else
  {
    first_is = true;
  }
}

void ServoControl(uint8_t angleSetting)
{
  if (angleSetting > 175)
  {
    angleSetting = 175;
  }
  else if (angleSetting < 5)
  {
    angleSetting = 5;
  }
  myservo.attach(10);
  myservo.write(angleSetting); //sets the servo position according to the  value
  delays(500);
  myservo.detach();
}

void line_teacking_mode(void)
{
  if(func_mode == LineTeacking)
  {
    if(LineTeacking_Read_Middle)     // Middle Sensor On Line
    {
      if(!LineTeacking_Read_Left && !LineTeacking_Read_Right) //LS and RS not on line
      {
        Serial.println("move forward");
        goForward(180);
      }
      else if(LineTeacking_Read_Left && !LineTeacking_Read_Left) //Sharp Left
      {
        Serial.println("Sharp Left");
        goLeft(250);
      }
      else if(!LineTeacking_Read_Left && LineTeacking_Read_Right) //Sharp Right
      {
        Serial.println("Sharp Right");
        goRight(250);
      }
      else if(LineTeacking_Read_Left && LineTeacking_Read_Right) 
      {
        stopMotors();
        Serial.println("Stop");
      }
    }
    else
    {
    if(LineTeacking_Read_Left && !LineTeacking_Read_Right)     // Turn left
    {
      goLeft(180);
      Serial.println("Left");
    }
    else if(!LineTeacking_Read_Left && LineTeacking_Read_Right)     // turn right
    {
      goRight(180);
      Serial.println("Right");
    } 
      else if(!LineTeacking_Read_Left && !LineTeacking_Read_Right)     // turn right
    {
      stopMotors();
    }
    }
    delays(5);
  }
}

void goForward(int mvspeed) {
  digitalWrite(front_lamps, HIGH);
  m.motor(1,FORWARD,mvspeed);
  m.motor(2,FORWARD,mvspeed);
  m.motor(3,FORWARD,mvspeed);
  m.motor(4,FORWARD,mvspeed);
}

void goBackward(int mvspeed) {
  digitalWrite(back_lamps, HIGH);
  m.motor(1,BACKWARD,mvspeed);
  m.motor(2,BACKWARD,mvspeed);
  m.motor(3,BACKWARD,mvspeed);
  m.motor(4,BACKWARD,mvspeed);
}

void goLeft(int mvspeed) {
  digitalWrite(front_lamps, HIGH);
  m.motor(1,FORWARD,mvspeed);
  m.motor(2,FORWARD,mvspeed);
  m.motor(3,BACKWARD,mvspeed);
  m.motor(4,BACKWARD,mvspeed);
}

void goRight(int mvspeed) {
  digitalWrite(front_lamps, HIGH);
  m.motor(1,BACKWARD,mvspeed);
  m.motor(2,BACKWARD,mvspeed);
  m.motor(3,FORWARD,mvspeed);
  m.motor(4,FORWARD,mvspeed);
}

void goTop_Right(int mvspeed) {
  digitalWrite(front_lamps, HIGH);
  m.motor(1,FORWARD,mvspeed / 3.1);
  m.motor(2,FORWARD,mvspeed / 3.1);
  m.motor(3,FORWARD,mvspeed);
  m.motor(4,FORWARD,mvspeed);
}

void goTop_Left(int mvspeed) {
  digitalWrite(front_lamps, HIGH);
  m.motor(1,FORWARD,mvspeed);
  m.motor(2,FORWARD,mvspeed);
  m.motor(3,FORWARD,mvspeed / 3.1);
  m.motor(4,FORWARD,mvspeed / 3.1);
}

void goBack_Left(int mvspeed) {
  digitalWrite(back_lamps, HIGH);
  m.motor(1,BACKWARD,mvspeed);
  m.motor(2,BACKWARD,mvspeed);
  m.motor(3,BACKWARD,mvspeed / 3.1);
  m.motor(4,BACKWARD,mvspeed / 3.1);
}

void goBack_Right(int mvspeed) {
  digitalWrite(back_lamps, HIGH);
  m.motor(1,BACKWARD,mvspeed / 3.1);
  m.motor(2,BACKWARD,mvspeed / 3.1);
  m.motor(3,BACKWARD,mvspeed);
  m.motor(4,BACKWARD,mvspeed);
}

void stopMotors() {
  digitalWrite(front_lamps, LOW);
  digitalWrite(back_lamps, LOW);
  m.motor(1,RELEASE,0);
  m.motor(2,RELEASE,0);
  m.motor(3,RELEASE,0);
  m.motor(4,RELEASE,0);
}
