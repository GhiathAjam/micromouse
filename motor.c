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

void motor_move(int right_motor_speed, int left_motor_speed, int direction)
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

  right_motor_speed = map(abs(right_motor_speed), 0, 1000, 0, 255);
  left_motor_speed = map(abs(left_motor_speed), 0, 1000, 0, 255);

  right_motor_speed = (right_motor_speed >= 0) ? right_motor_speed : 255 - right_motor_speed;
  left_motor_speed = (left_motor_speed > 0) ? 255 - left_motor_speed : left_motor_speed;

  // 0 <= Speed <= 255
  analogWrite(HBRIDGE_EN1, right_motor_speed);
  analogWrite(HBRIDGE_EN2, left_motor_speed);
}
void motor_stop()
{
  motor_move(0, 0, 3);
}

void setup()
{
  // may need to increase freq.
  // debugging
  Serial.begin(9600);
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

void loop()
{
  int left_motor_speed, right_motor_speed;
  while (!Serial.available())
    left_motor_speed = Serial.parseInt();
  while (!Serial.available())
    right_motor_speed = Serial.parseInt();

  motor_move(right_motor_speed, left_motor_speed, 2);
  delay(2000);

  motor_stop();
}
