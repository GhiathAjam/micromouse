////////////////////  IR Sensor Constants  ////////////////////

#define NUM_OF_SENSORS_Follower 3
#define NUM_OF_SENSORS 5// Number of sensors
#define MAPPED_RANGE_SENSORS 1000 // Sensor readings are mapped to this range
#define WEIGHT_UNIT 1000          // Unit for weighted average

#include "stack.h"

const int RIGHT_SENSOR = 4;
const int STRAIGHT_SENSOR = 2;
const int LEFT_SENSOR = 0;
const int SENSOR_UNO[NUM_OF_SENSORS] = {A0, A1, A2, A3, A4}; // Arduino pins LEFT to RIGHT
const int SENSOR_UNO_Follower[NUM_OF_SENSORS_Follower] = { A1, A2, A3}; // Arduino pins LEFT to RIGHT For Line follower

const int THRESHOLD[NUM_OF_SENSORS] = {450, 650, 650, 650, 300};

float linePos = 0;
float lastLinePos = 0;

const float KP = .055;
const float KD = .3;
const float KI = 0;


///////////////////   MOTOR CONSTANTS   //////////////////////

const float SPEED = 90;
// PWM for speed control
const int HBRIDGE_EN2 = 11; // left
const int HBRIDGE_EN1 = 9; // right

// H-Bridge pins for polarity
// left motor
const int HBRIDGE_IN1 = 12;
const int HBRIDGE_IN2 = 13;
// right motor
const int HBRIDGE_IN3 = 10;
const int HBRIDGE_IN4 = 8;

////////////////////
// MAZE ///////////////
// path string
Stack path;

const int TURN_SPEED = 90;
const int TURN_DELAY = 500;
const int FORWARD_TURN_DELAY = 200;
const int MOVE_DELAY = 300;
const int BIT_DELAY = 40;

/*
  1. LBR = “B”
  2. LBS = “R”
  3. RBL = “B”
  4. SBL = “R”
  5. SBS = “B”
  6. LBL = “S”
*/

void initializeCNYSensors();
void motors_setup();
int ir_read(int);
void motor_move(int, int);
void motor_stop();
void rotate_left();
void rotate_right();
void move_cell();
void move_bit();

void setup()
{
  Serial.begin(9600);
  initializeCNYSensors();
  motors_setup();

}

void temp()
{
  while (ir_read(LEFT_SENSOR) == 0)
  {
    move_cell();
  }
  rotate_left();
}

void loop()
{
  //  temp();
  //  static bool backM = false;
  //
  ////delay(500);
  //  for (int i = 0; i < NUM_OF_SENSORS; ++i)
  //    {
  //      Serial.print(" ");
  //      Serial.print(ir_read(i));
  //      Serial.print(" ");
  //    }
  //    Serial.println();
  //
  ////  move_cell();
  ////  return;
  //
  ////  rotate_right();
  ////  delay(5000);
  ////  return;
  //
  //  int right = ir_read(RIGHT_SENSOR);
  //  int straight = ir_read(STRAIGHT_SENSOR);
  //  int left = ir_read(LEFT_SENSOR);
  //
  ////   print out the sensor readings
  //   Serial.print("left: ");
  //   Serial.print(left);
  //   Serial.print("  straight: ");
  //   Serial.print(straight);
  //   Serial.print("  right: ");
  //   Serial.println(right);
  //
  //  bool leftM = false;
  //  bool rightM = false;
  //  bool straightM = false;
  //  bool record = true;
  //
  //  char nextBranch = 'L';
  //  if (left && right)
  //  {
  //    // go left
  //    Serial.println("Left and right, going left");
  //    nextBranch = 'L';
  //    // path.push('L')
  //
  //    rotate_left();
  ////    move_cell();
  //  }
  //
  //  else if (left || right)
  //  {
  //    Serial.println("Left or right, reading straight");
  //    move_bit();
  //    straight = ir_read(STRAIGHT_SENSOR);
  //
  //    if (left && !straight)
  //    {
  //      // go left, dont register
  //      // nextBranch = 'L';
  //      // No pushing
  //      record = false;
  //      rotate_left();
  ////      move_cell();
  //    }
  //    else if (left && straight)
  //    {
  //      // go left, register
  //          Serial.println("Left and straight, going left");
  //      nextBranch = 'L';
  //      // path.push('L');
  //      rotate_left();
  ////      move_cell();
  //    }
  //    else if (right && !straight)
  //    {
  //      // go right, dont register
  //          Serial.println("right only, going right");
  //      // nextBranch = 'R';
  //      // No pushing
  //      record = false;
  //      rotate_right();
  ////      move_cell();
  //    }
  //    else if (right && straight)
  //    {
  //      // go right, register
  //          Serial.println("right and straight, going straight");
  //      nextBranch = 'S';
  //      // path.push('S');
  //      rotate_right();
  ////      move_cell();
  //    }
  //  }
  //  else if (straight)
  //  {
  //    // nextBranch = 'S';
  ////    move_bit();
  //    straight = ir_read(STRAIGHT_SENSOR);
  //
  //    if (straight)
  //    {
  //          Serial.println("Straight only");
  //
  //      move_cell();
  //    }
  //    else
  //    {
  //      // go back
  //          Serial.println("None, BACK! ");
  //      nextBranch = 'B';
  //      backM = true;
  //
  //      rotate_left();
  //      rotate_left();
  ////      move_cell();
  //
  //      return;
  //    }
  //  }
  //  else
  //  {
  //      // go back
  //          Serial.println("None, back");
  //
  //      nextBranch = 'B';
  //      backM = true;
  //
  //      rotate_left();
  //      rotate_left();
  ////      move_cell();
  //
  //      return;
  //  }
  //
  //  if (record && nextBranch != 'B')
  //  {
  //    if (backM)
  //    {
  //      char prev = path.peek();
  //      path.pop();
  //
  //      /*
  //      LBR = “B”
  //      LBS = “R”
  //      RBL = “B”
  //      SBL = “R”
  //      SBS = “B”
  //      LBL = “S”
  //      */
  //
  //      if (prev == 'L' && nextBranch == 'R')
  //      {
  //        nextBranch = 'B';
  //      }
  //      else if (prev == 'L' && nextBranch == 'S')
  //      {
  //        nextBranch = 'R';
  //      }
  //      else if (prev == 'R' && nextBranch == 'L')
  //      {
  //        nextBranch = 'B';
  //      }
  //      else if (prev == 'S' && nextBranch == 'L')
  //      {
  //        nextBranch = 'R';
  //      }
  //      else if (prev == 'S' && nextBranch == 'S')
  //      {
  //        nextBranch = 'B';
  //      }
  //      else if (prev == 'L' && nextBranch == 'L')
  //      {
  //        nextBranch = 'S';
  //      }
  //    }
  //
  //    if (nextBranch != 'B')
  //    {
  //      path.push(nextBranch);
  //      backM = false;
  //    }
  //  }

  if (ir_read(LEFT_SENSOR) == 1 )
  {
    rotate_left();
    
  }
  else if(ir_read(RIGHT_SENSOR) == 1)
  {
    //    delay(2000);
    rotate_right();
  }
  else {
    move_cell();
    }

  //    rotate_right();
  //move_bit();
  //    move_cell();
  //  delay(5000);
  //  printt();
  //  delay(500);
}

/////////////////////////////// IR Functions

int ir_read(int idx)
{
  return (MAPPED_RANGE_SENSORS - analogRead(SENSOR_UNO[idx])) > THRESHOLD[idx];
}

int check()
{
  int x[NUM_OF_SENSORS];
  for (int i = 0 ; i < NUM_OF_SENSORS; i++)
    x[i] = ir_read(i);

  if (x[0] == 1 && x[NUM_OF_SENSORS - 1] == 1 && ( x[1] == 0 || x[2] == 0 || x[3] == 0))
    return 1;

  return 0;
}

// rotate arduino robot right
void rotate_right()
{
 
  Serial.println("Rot right");
  motor_forward();
  motor_move(SPEED, SPEED);
  delay(FORWARD_TURN_DELAY);
  motor_right();

  //  motor_move(TURN_SPEED, TURN_SPEED);

  delay(400 );
//  while (analogRead(A3) < 500);
  //  while (!(analogRead(A0) > 500 && analogRead(A4) > 500 && analogRead(A1)  > 500 && analogRead(A2) >500 && analogRead(A3) >500));
  while (!(analogRead(A0) > 500 && analogRead(A4) > 500 && (analogRead(A1) < 500 || analogRead(A2) < 500 || analogRead(A3) < 500)));

  //  motor_move(SPEED, SPEED);

  motor_forward();
}

// rotate arduino robot left
void rotate_left()
{
  Serial.println("Rot left");
  motor_forward();
  motor_move(SPEED, SPEED);
  delay(FORWARD_TURN_DELAY);
  motor_left();

  //  motor_move(TURN_SPEED, TURN_SPEED);

  delay(400 );
//  while (analogRead(A3) < 500);
  //  while (!(analogRead(A0) > 500 && analogRead(A4) > 500 && analogRead(A1)  > 500 && analogRead(A2) >500 && analogRead(A3) >500));
  while (!(analogRead(A0) > 500 && analogRead(A4) > 500 && (analogRead(A1) < 500 || analogRead(A2) < 500 || analogRead(A3) < 500)));

  //  motor_move(SPEED, SPEED);

  motor_forward();
}

// move one cell up
void move_cell()
{
  //  motor_forward();
  //  motor_move(SPEED, SPEED);
  //  delay(100);
  //  motor_move(0, 0);



  lastLinePos = linePos;
  linePos = getLinePosition((lastLinePos > 0));
  //  linePos = -1 * linePos;
  float correction = getPIDCorrection(linePos, lastLinePos, KP, KD, KI);
  float maxCorrection = SPEED * .5;

//  Serial.print("line position:");
//  Serial.println(linePos);

  if (correction > 0)
  {
    correction = (correction > maxCorrection) ? maxCorrection : correction;
//    Serial.print("Speeds");
//    Serial.println(SPEED - correction);
//    Serial.println(SPEED);
    motor_move(SPEED - correction, SPEED);
  }
  else
  {
    correction = (correction < -maxCorrection) ? -maxCorrection : correction;
//    Serial.print("Speeds");
//    Serial.println(SPEED);
//    Serial.println(SPEED + correction);
    motor_move(SPEED , SPEED + correction);
  }
}

// move a bit to read straight
void move_bit()
{
//  Serial.println("Moving a bit");
  motor_forward();
  motor_move(SPEED + 5, SPEED);
  delay(BIT_DELAY);
  motor_move(0, 0);
}

void printt()
{
  Serial.println("S1\t\tS2\t\tS3\t\tS4\t\tS5");
  Serial.print(ir_read(0));
  Serial.print("\t\t");

  Serial.print(ir_read(1));
  Serial.print("\t\t");

  Serial.print(ir_read(2));
  Serial.print("\t\t");
  Serial.print(ir_read(3));
  Serial.print("\t\t");
  Serial.println(ir_read(4));
}

/*
  initializeCNYSensors():
  Initializes the CNY sensors.
  The sensors are connected to the Arduino pins in the following order:
  A4, A3, A2, A1, A0 from left to right.
*/

void initializeCNYSensors()
{
  for (int x = 0; x < NUM_OF_SENSORS; x++)
  {
    pinMode(SENSOR_UNO[x], INPUT);
    pinMode(SENSOR_UNO_Follower[x], INPUT);
  }
}

/*
  motors_setup():
  Initializes the H-Bridge pins for the motors.
*/

void motors_setup()
{
  // define motors output pins
  pinMode(HBRIDGE_IN1, OUTPUT);
  pinMode(HBRIDGE_IN2, OUTPUT);
  pinMode(HBRIDGE_IN3, OUTPUT);
  pinMode(HBRIDGE_IN4, OUTPUT);

  // PWM pins for motors speed
  pinMode(HBRIDGE_EN1, OUTPUT);
  pinMode(HBRIDGE_EN2, OUTPUT);

  motor_forward();

}

void motor_forward()
{
  digitalWrite(HBRIDGE_IN1, 1);
  digitalWrite(HBRIDGE_IN2, 0);

  digitalWrite(HBRIDGE_IN3, 1);
  digitalWrite(HBRIDGE_IN4, 0);
}

void motor_backward()
{
  digitalWrite(HBRIDGE_IN1, 0);
  digitalWrite(HBRIDGE_IN2, 1);

  digitalWrite(HBRIDGE_IN3, 0);
  digitalWrite(HBRIDGE_IN4, 1);
}

void motor_left()
{
  digitalWrite(HBRIDGE_IN1, 0);
  digitalWrite(HBRIDGE_IN2, 1);

  digitalWrite(HBRIDGE_IN3, 1);
  digitalWrite(HBRIDGE_IN4, 0);
}

void motor_right()
{
  digitalWrite(HBRIDGE_IN1, 1);
  digitalWrite(HBRIDGE_IN2, 0);

  digitalWrite(HBRIDGE_IN3, 0);
  digitalWrite(HBRIDGE_IN4, 1);
}

/*
  motor_move():
  Moves the motors.
  @param speed1: The speed of the right motor.
  @param speed2: The speed of the left motor.
*/

void motor_move(int motor1_speed, int motor2_speed)
{
//  Serial.println("Moving Motor");
  analogWrite(HBRIDGE_EN1, motor1_speed);
  analogWrite(HBRIDGE_EN2, motor2_speed);
}


float getLinePosition(int lastDir)
{
  // lastDir is 1 if line has a positive value and 0 if negative

  float line = 0;
  float sensorsScaled[NUM_OF_SENSORS_Follower] = {0, 0, 0};
  float wightedReadings = 0; // Average numerator
  float sumReadings = 0;     // Average denominator
  int lineDetected = 0;
  Serial.print("Sensor readings: ");

  for (int i = 0; i < NUM_OF_SENSORS_Follower; i++)
  {

    sensorsScaled[i] = analogRead(SENSOR_UNO_Follower[i]);

    sensorsScaled[i] = MAPPED_RANGE_SENSORS - sensorsScaled[i]; // Reverse scale to have 1000 as pure black and 0 as pure white

    //    Serial.print(analogRead(SENSOR_UNO[i]));
    Serial.print(sensorsScaled[i]);

    Serial.print(" ");

    if (sensorsScaled[i] >= THRESHOLD[i])
    {
      lineDetected = 1;
    }

    wightedReadings += sensorsScaled[i] * i * WEIGHT_UNIT;
    sumReadings += sensorsScaled[i];
  }
  Serial.println();

  if (lineDetected)
  {
    line = wightedReadings / sumReadings;                   // Weighted average 0-4000
    line = line - (WEIGHT_UNIT * (NUM_OF_SENSORS_Follower - 1) / 2); // Change scale from 0 _ 4000 to -2000 _ 2000
//    Serial.println("line");

    // -2000 means the line exactly under the most left sensor
    // -1000 means the line exactly under the second from left sensor
    // 0 means the line exactly under the center sensor
    // 1000 means the line exactly under the first right sensor
    // 2000 means the line exactly under the most right sensor
  }
  else
  {
    line = WEIGHT_UNIT * (NUM_OF_SENSORS_Follower - 1) * lastDir;    // Use last direction to calculate error as the maximum value
    line = line - (WEIGHT_UNIT * (NUM_OF_SENSORS_Follower - 1) / 2); // Change scale from 0 _ 4000 to -2000 _ 2000
  }
  return line;
}

float getPIDCorrection(float line, float posLastLine, float kp, float kd, float ki)
{
  float proportional = line;
  float derivative = line - posLastLine;
  float integral = line + posLastLine;

  float correction = kp * proportional + kd * derivative + ki * integral;
  return correction;
}
