////////////////////  IR Sensor

#define WHITE 1
#define BLACK 0
#define COLOR BLACK // Line color

#define N_SENS 5         // Number of sensors
#define R_SENS 1000      // Sensor readings are mapped to this range
#define WEIGHT_UNIT 1000 // Unit for weighted average
#define P_LINE_MIN 0.5   // Minimum brightness percentage to consider part of the line (=reading/100)

const int SENSOR_UNO[N_SENS] = {A0, A1, A2, A3, A4}; // Arduino pins
const int sensorsMax[N_SENS] = {0};                  // Maximum value each sensor measures
const int sensorsMin[N_SENS] = {0};                  // Minimum value each sensor measures
float linePos = 0;
float lastLinePos = 0;
int sensorShowsLine = 0b00000;

const float SPEED = 255;
const float KP = .4;
const float KD = 12;
const float KI = 0;

///////////////////   MOTOR

// PWM for speed control
const int HBRIDGE_EN1 = 10; //right 
const int HBRIDGE_EN2 = 11; //left

// H-Bridge pins for polarity
// right motor
const int HBRIDGE_IN1 = 8;
const int HBRIDGE_IN2 = 9;
// left motor
const int HBRIDGE_IN3 = 12;
const int HBRIDGE_IN4 = 13;

////////////////////

void setup() {
  Serial.begin(9600);
  initializeCNYSensors();
  // may need to increase freq.
  // H-Bridge enable pins as PWM output
  pinMode(HBRIDGE_EN1, OUTPUT);
  pinMode(HBRIDGE_EN2, OUTPUT);
  // H-Bridge pins as output
  pinMode(HBRIDGE_IN1, OUTPUT);
  pinMode(HBRIDGE_IN2, OUTPUT);
  pinMode(HBRIDGE_IN3, OUTPUT);
  pinMode(HBRIDGE_IN4, OUTPUT);

  motor_stop();
    
}


void loop() {
  linePos = getLinePosition(BLACK, (lastLinePos > 0));
  lastLinePos = linePos;
  float correction = getPIDCorrection(linePos, lastLinePos, KP, KD, KI);
  float maxCorrection = SPEED;

  if (correction > 0)
  {
    correction = (correction > maxCorrection) ? maxCorrection : correction;
    int direction = linePos > 0 ? 1 : linePos < 0 ? 0
                                                  : 2;
    motor_move(SPEED,SPEED-correction,direction);
  }
  else
  {
    correction = (correction < -maxCorrection) ? -maxCorrection : correction;
    int direction = linePos > 0 ? 1 : linePos < 0 ? 0
                                                  : 2;
    motor_move(SPEED+correction,SPEED,direction);
  }

  Serial.print("line position: ");
  Serial.println(linePos);
}

/////////////////////////////// IR Functions 

void initializeCNYSensors()
{
  for (int x = 0; x <= N_SENS; x++)
  {
    pinMode(SENSOR_UNO[x], INPUT);
  }
}

float getLinePosition(int color, int lastDir)
{
  // lastDir is 1 if line has a positive value and 0 if negative

  float line = 0;
  int lineDetected = 0;
  float sensorsScaled[N_SENS];
  float wightedReadings = 0; // Average numerator
  float avgReadings = 0;     // Average denominator

  int start = 0;

  for (int i = 0; i < N_SENS; i++)
  {
    sensorShowsLine = 0b00000;
    int bitmask = 0b10000;

    sensorsScaled[i] = analogRead(SENSOR_UNO[i]) - sensorsMin[i];

    sensorsScaled[i] *= R_SENS;
    sensorsScaled[i] /= (sensorsMax[i] - sensorsMin[i]); // Now readings values range from 0 to 1000

    Serial.print("sensor ");
    Serial.print(i);
    Serial.println(sensorsScaled[i]);

    if (color == BLACK)
    {
      sensorsScaled[i] = R_SENS - sensorsScaled[i]; // Reverse scale to have 1000 as pure black and 0 as pure white
    }

    if (sensorsScaled[i] >= (float)R_SENS * ((float)P_LINE_MIN / 100.0)) // At least one sensor has to detect a line
    {
      lineDetected = 1;
      sensorShowsLine |= bitmask;
    }

    // startring point
    // TODO motor team go ahead with maximum speed
    if (sensorShowsLine == 0b01110)
    {
      return 0;
    }
    // cross lines
    // TODO motor team go ahead with average speed
    // 2001 is a special case not in the range [-2000:2000]
    if (sensorShowsLine == 0b11111)
    {
      return 2001;
    }

    wightedReadings += sensorsScaled[i] * i * WEIGHT_UNIT;
    avgReadings += sensorsScaled[i];
  }
  if (lineDetected == 1)
  {
    line = wightedReadings / avgReadings;           // Weighted average 0-4000
    line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2); // Change scale from 0 _ 4000 to -2000 _ 2000

    // -2000 means the line exactly under the most left sensor
    // -1000 means the line exactly under the second from left sensor
    // 0 means the line exactly under the center sensor
    // 1000 means the line exactly under the first right sensor
    // 2000 means the line exactly under the most right sensor
  }
  else
  {
    line = WEIGHT_UNIT * (N_SENS - 1) * lastDir;    // Use last direction to calculate error as the maximum value
    line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2); // Change scale
  }

  return line;
}

float getPIDCorrection(float line, float last_line, float kp, float kd, float ki)
{
  float proportional = line;
  float derivative = line - last_line;
  float integral = line + last_line;

  float correction = kp * proportional + kd * derivative + ki * integral;
  return correction;
}

/////////////////////////////////////////Motor functions

void move_right_motor(int direction) 
{
  // 1 forward, 0 backward, 2 stop
  if(direction==1){
      digitalWrite(HBRIDGE_IN1, HIGH);
      digitalWrite(HBRIDGE_IN2, LOW);
  }
  else if(direction==0){
      digitalWrite(HBRIDGE_IN1, LOW);
      digitalWrite(HBRIDGE_IN2, HIGH);
  }
  else{
      digitalWrite(HBRIDGE_IN1, LOW);
      digitalWrite(HBRIDGE_IN2, LOW);
  }
}
void move_left_motor(int direction) 
{
  // 1 forward, 0 backward, 2 stop
  if(direction==1){
      digitalWrite(HBRIDGE_IN3, HIGH);
      digitalWrite(HBRIDGE_IN4, LOW);
  }
  else if(direction==1){
      digitalWrite(HBRIDGE_IN3, LOW);
      digitalWrite(HBRIDGE_IN4, HIGH);
  }
  else{
      digitalWrite(HBRIDGE_IN3, LOW);
      digitalWrite(HBRIDGE_IN4, LOW);
  }
}
//:)
void motor_backward()
{ 
  // both Motors polarity should be backward
  digitalWrite(HBRIDGE_IN1, LOW);
  digitalWrite(HBRIDGE_IN2, HIGH);
  
  digitalWrite(HBRIDGE_IN3, LOW);
  digitalWrite(HBRIDGE_IN4, HIGH);
}

void motor_move(int right_motor_speed, int left_motor_speed, int direction)
{
  // 1 turn right, 0 turn left, 2 forward, 3 stop 
  if(direction==1){
    move_right_motor(0);
    move_left_motor(1);
  }
  else if(direction==0){
    move_right_motor(1);
    move_left_motor(0);
  }
  else if (direction ==2){
    move_right_motor(1);
    move_left_motor(1);
  }
  else{
    move_right_motor(2);
    move_left_motor(2);
  }
  // 0 <= Speed <= 255
  analogWrite(HBRIDGE_EN1, right_motor_speed);
  analogWrite(HBRIDGE_EN2, left_motor_speed);
}
void motor_stop()
{
  motor_move(0, 0, 3);
}
////////////////////////////////////////aa