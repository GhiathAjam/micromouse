

// PWM for speed control
const int HBRIDGE_EN1 = 10;
const int HBRIDGE_EN2 = 11;
// H-Bridge pins for polarity
const int HBRIDGE_IN1 = 8;
const int HBRIDGE_IN2 = 9;
const int HBRIDGE_IN3 = 12;
const int HBRIDGE_IN4 = 13;

void motor_stop()
{
  motor_move(0, 0);
}

void motor_forward()
{
  // both Motors polarity should be forward
  digitalWrite(HBRIDGE_IN1, HIGH);
  digitalWrite(HBRIDGE_IN2, LOW);
  
  digitalWrite(HBRIDGE_IN3, HIGHT);
  digitalWrite(HBRIDGE_IN4, LOW);
}

void motor_backward()
{ 
  // both Motors polarity should be backward
  digitalWrite(HBRIDGE_IN1, LOW);
  digitalWrite(HBRIDGE_IN2, HIGH);
  
  digitalWrite(HBRIDGE_IN3, LOW);
  digitalWrite(HBRIDGE_IN4, HIGH);
}

void motor_move(int left_motor_speed, int right_motor_speed)
{
  // 0 <= Speed <= 255
  analogWrite(HBRIDGE_EN1, left_motor_speed);
  analogWrite(HBRIDGE_EN2, left_motor_speed);
}

void setup()
{
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

  motor_forward();
}

void loop()
{  
  int left_motor_speed, right_motor_speed;
  while (! Serial.available())
    left_motor_speed = Serial.parseInt();
  while (! Serial.available())
    right_motor_speed = Serial.parseInt();
  
  motor_move(left_motor_speed, right_motor_speed);
  delay(2000);

  motor_stop();
}
