////////////////////  IR Sensor

#define WHITE_LINE_LINE 1
#define BLACK_LINE 0
#define COLOR BLACK_LINE // Line color

#define NUM_OF_SENSORS 5          // Number of sensors
#define MAPPED_RANGE_SENSORS 1000 // Sensor readings are mapped to this range
#define WEIGHT_UNIT 1000          // Unit for weighted average
#define BRIGHTNESS_LINE_MIN 0.5   // Minimum brightness percentage to consider part of the line (=reading/100)

const int SENSOR_UNO[NUM_OF_SENSORS] = {A4, A3, A2, A1, A0};           // Arduino pins
const int sensorsMax[NUM_OF_SENSORS] = {1023, 1023, 1023, 1023, 1023}; // Maximum value each sensor measures
const int sensorsMin[NUM_OF_SENSORS] = {0, 0, 0, 0, 0};                // Minimum value each sensor measures
float linePos = 0;
float lastLinePos = 0;
int sensorShowsLine = 0b00000;

const float SPEED = 170;
const float sharpTurn = 0.6;
const float mediumTurn = 0.4;
const float slightTurn = 0.3;
const float weakRatio = 1.2;
#define LSPEED (weakRatio * SPEED)

const float KP = .4;
const float KD = 12;
const float KI = 0;

///////////////////   MOTOR

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
    pinMode(HBRIDGE_EN1, OUTPUT);
    pinMode(HBRIDGE_EN2, OUTPUT);
    // H-Bridge pins as output
    pinMode(HBRIDGE_IN1, OUTPUT);
    pinMode(HBRIDGE_IN2, OUTPUT);
    pinMode(HBRIDGE_IN3, OUTPUT);
    pinMode(HBRIDGE_IN4, OUTPUT);

    motor_stop();
}

void loop()
{
    uint8_t binaryReadings = getSensorsBinary();
    Serial.print("Binary: ");
    Serial.println(binaryReadings, BIN);

    motor_move(70, 70, 2);

    return;

    if (binaryReadings == 0b00001)
    {
        motor_move(sharpTurn * LSPEED, SPEED, 1);
        // Serial.println("turn sharp right");
    }
    else if (binaryReadings == 0b10000)
    {
        motor_move(LSPEED, (sharpTurn * SPEED), 0);
        // Serial.println("turn sharp left");
    }
    else if (binaryReadings == 0b00100)
    {
        motor_move(LSPEED, SPEED, 2);
        // Serial.println("forward");
    }

    else if (binaryReadings == 0b00011)
    {
        motor_move((mediumTurn * LSPEED), SPEED, 0);
        // Serial.println("turn medium right");
    }
    else if (binaryReadings == 0b11000)
    {
        motor_move(LSPEED, (mediumTurn * SPEED), 1);
        // Serial.println("turn medium left");
    }
    else if (binaryReadings == 0b00110)
    {
        motor_move((slightTurn * LSPEED), SPEED, 0);
        // Serial.println("turn slight right");
    }
    else if (binaryReadings == 0b01100)
    {
        motor_move(LSPEED, (slightTurn * SPEED), 1);
        // Serial.println("turn slightleft");
    }
    else if (binaryReadings == 0b01110)
    {
        motor_move(LSPEED, SPEED, 2);
        // Serial.println("forward");
    }
    else
    {
        motor_move(0, 0, 2);
        // Serial.println("Stop");
    }
}

/////////////////////////////// IR Functions

void initializeCNYSensors()
{
    for (int x = 0; x <= NUM_OF_SENSORS; x++)
    {
        pinMode(SENSOR_UNO[x], INPUT);
    }
}

uint8_t getSensorsBinary()
{
    uint16_t binaryArray[NUM_OF_SENSORS] = {0};
    uint8_t binary = 0, bitmask = 0b10000;

    for (uint8_t i = 0; i < NUM_OF_SENSORS; i++)
    {
        binaryArray[i] = analogRead(SENSOR_UNO[i]);
        if (binaryArray[i] <= 100.0)
            binary |= bitmask;
        else
            binary |= 0;
        bitmask = bitmask >> 1;
    }
    return binary;
}

uint8_t countBinary(uint8_t binary)
{
    uint8_t bitmask = 0b10000, count = 0;
    for (int i = 0; i < NUM_OF_SENSORS; i++)
    {
        if (binary & bitmask)
            count++;
        bitmask = bitmask >> 1;
    }
    return count;
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
    if (direction == 1)
    {
        digitalWrite(HBRIDGE_IN1, HIGH);
        digitalWrite(HBRIDGE_IN2, LOW);
    }
    else if (direction == 0)
    {
        digitalWrite(HBRIDGE_IN1, LOW);
        digitalWrite(HBRIDGE_IN2, HIGH);
    }
    else
    {
        digitalWrite(HBRIDGE_IN1, LOW);
        digitalWrite(HBRIDGE_IN2, LOW);
    }
}
void move_left_motor(int direction)
{
    // 1 forward, 0 backward, 2 stop
    if (direction == 1)
    {
        digitalWrite(HBRIDGE_IN3, HIGH);
        digitalWrite(HBRIDGE_IN4, LOW);
    }
    else if (direction == 1)
    {
        digitalWrite(HBRIDGE_IN3, LOW);
        digitalWrite(HBRIDGE_IN4, HIGH);
    }
    else
    {
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

void motor_move(int left_motor_speed, int right_motor_speed, int direction)
{
    // 1 turn right, 0 turn left, 2 forward, 3 stop
    if (direction == 1)
    {
        move_right_motor(0);
        move_left_motor(1);
    }
    else if (direction == 0)
    {
        move_right_motor(1);
        move_left_motor(0);
    }
    else if (direction == 2)
    {
        move_right_motor(1);
        move_left_motor(1);
    }
    else
    {
        move_right_motor(2);
        move_left_motor(2);
    }

    // right_motor_speed = map(abs(right_motor_speed), 0, 1000, 0, 255);
    // left_motor_speed = map(abs(left_motor_speed), 0, 1000, 0, 255);

    // right_motor_speed = (right_motor_speed >= 0) ? right_motor_speed : 255 - right_motor_speed;
    // left_motor_speed = (left_motor_speed > 0) ? 255 - left_motor_speed : left_motor_speed;

    // 0 <= Speed <= 255
    analogWrite(HBRIDGE_EN1, right_motor_speed);
    analogWrite(HBRIDGE_EN2, left_motor_speed);
}
void motor_stop()
{
    motor_move(0, 0, 3);
}