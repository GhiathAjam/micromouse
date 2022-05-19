////////////////////  IR Sensor Constants  ////////////////////

#define NUM_OF_SENSORS 5          // Number of sensors
#define MAPPED_RANGE_SENSORS 1000 // Sensor readings are mapped to this range
#define WEIGHT_UNIT 1000          // Unit for weighted average

const int SENSOR_UNO[NUM_OF_SENSORS] = {A4, A3, A2, A1, A0}; // Arduino pins
float linePos = 0;
float lastLinePos = 0;

const float SPEED = 120;
const float KP = .3;
const float KD = .5;
const float KI = 0;

///////////////////   MOTOR CONSTANTS   //////////////////////

// PWM for speed control
const int HBRIDGE_EN1 = 10; // right
const int HBRIDGE_EN2 = 11; // left

// H-Bridge pins for polarity
// right motor
const int HBRIDGE_IN1 = 8;
const int HBRIDGE_IN2 = 9;
// left motor
const int HBRIDGE_IN3 = 12;
const int HBRIDGE_IN4 = 13;

////////////////////

void setup()
{
  Serial.begin(9600);
  initializeCNYSensors();
  motors_setup();
}

void loop()
{
  lastLinePos = linePos;
  linePos = getLinePosition((lastLinePos > 0));
  float correction = getPIDCorrection(linePos, lastLinePos, KP, KD, KI);
  float maxCorrection = SPEED;

  Serial.print("line position:");
  Serial.println(linePos);

  if (correction > 0)
  {
    correction = (correction > maxCorrection) ? maxCorrection : correction;
    Serial.print("Speeds");
    Serial.println(SPEED - correction);
    Serial.println(SPEED);
    motor_move(SPEED, SPEED - correction);
  }
  else
  {
    correction = (correction < -maxCorrection) ? -maxCorrection : correction;
    Serial.print("Speeds");
    Serial.println(SPEED);
    Serial.println(SPEED + correction);
    motor_move(SPEED + correction, SPEED);
  }
}

/////////////////////////////// IR Functions

/*
  getLinePosition()
  @return the position of the line.
  @param lastDir - the last direction the line was moving
  The position is a value between -2000 and 2000.
  The closer to 0, the closer to the line.
  The closer to abs(2000), the further from the line.
*/

float getLinePosition(int lastDir)
{
  // lastDir is 1 if line has a positive value and 0 if negative

  float line = 0;
  float sensorsScaled[NUM_OF_SENSORS] = {0, 0, 0, 0, 0};
  float wightedReadings = 0; // Average numerator
  float sumReadings = 0;     // Average denominator
  int lineDetected = 0;
  Serial.print("Sensor readings: ");
  for (int i = 0; i < NUM_OF_SENSORS; i++)
  {
    sensorsScaled[i] = analogRead(SENSOR_UNO[i]);
    sensorsScaled[i] = MAPPED_RANGE_SENSORS - sensorsScaled[i]; // Reverse scale to have 1000 as pure black and 0 as pure white

    //    Serial.print(analogRead(SENSOR_UNO[i]));
    Serial.print(sensorsScaled[i]);
    Serial.print(" ");

    if (sensorsScaled[i] >= 850)
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
    line = line - (WEIGHT_UNIT * (NUM_OF_SENSORS - 1) / 2); // Change scale from 0 _ 4000 to -2000 _ 2000
    Serial.println("line");

    // -2000 means the line exactly under the most left sensor
    // -1000 means the line exactly under the second from left sensor
    // 0 means the line exactly under the center sensor
    // 1000 means the line exactly under the first right sensor
    // 2000 means the line exactly under the most right sensor
  }
  else
  {
    line = WEIGHT_UNIT * (NUM_OF_SENSORS - 1) * lastDir;    // Use last direction to calculate error as the maximum value
    line = line - (WEIGHT_UNIT * (NUM_OF_SENSORS - 1) / 2); // Change scale from 0 _ 4000 to -2000 _ 2000
  }
  return line;
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
  }
}

/*
  getPIDCorrection():
  @returns the correction for the PID controller.
  @param linePos: The current line position.
  @param lastLinePos: The last line position.
  @param KP: The proportional constant.
  @param KD: The derivative constant.
  @param KI: The integral constant.
*/

float getPIDCorrection(float line, float posLastLine, float kp, float kd, float ki)
{
  float proportional = line;
  float derivative = line - posLastLine;
  float integral = line + posLastLine;

  float correction = kp * proportional + kd * derivative + ki * integral;
  return correction;
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

  digitalWrite(HBRIDGE_IN1, 1);
  digitalWrite(HBRIDGE_IN2, 0);
  digitalWrite(HBRIDGE_IN3, 1);
  digitalWrite(HBRIDGE_IN4, 0);

  analogWrite(HBRIDGE_EN1, SPEED);
  analogWrite(HBRIDGE_EN2, SPEED);
}

/*
  motor_move():
  Moves the motors.
  @param speed1: The speed of the right motor.
  @param speed2: The speed of the left motor.
*/

void motor_move(int motor1_speed, int motor2_speed)
{
  analogWrite(HBRIDGE_EN1, motor1_speed);
  analogWrite(HBRIDGE_EN2, motor2_speed);
}
