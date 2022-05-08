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

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  initializeCNYSensors();
}

void loop()
{
  // put your main code here, to run repeatedly:

  linePos = getLinePosition(BLACK, (lastLinePos > 0));
  lastLinePos = linePos;
  float correction = getPIDCorrection(linePos, lastLinePos, KP, KD, KI);
  float maxCorrection = SPEED;

  if (correction > 0)
  {
    correction = (correction > maxCorrection) ? maxCorrection : correction;
    int direction = linePos > 0 ? 1 : linePos < 0 ? 0
                                                  : 2;
    // motorspeed(SPEED,SPEED-correction,direction);
  }
  else
  {
    correction = (correction < -maxCorrection) ? -maxCorrection : correction;
    int direction = linePos > 0 ? 1 : linePos < 0 ? 0
                                                  : 2;
    // motorspeed(SPEED+correction,SPEED,direction);
  }

  Serial.print("line position: ");
  Serial.println(linePos);
}

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

void getPIDCorrection(float line, float last_line, float kp, float kd, float ki)
{
  float proportional = line;
  float derivative = line - last_line;
  float integral = line + last_line;

  float correction = kp * proportional + kd * derivative + ki * integral;
  return correction;
}