#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver();

#define SERVO_MIN 100
#define SERVO_MAX 500

// the following values are the angles for the standing pose, you can adjust them to calibrate the robot's posture
// STANDING CALIBRATION

int RH = 100;
int RK = 95;
int RA = 115;

int LH = 89;
int LK = 99;
int LA = 111;

// MOVEMENT TUNING

int LEAN = 12;
int LIFT = 24;
int STEP = 18;
int PUSH = 14;

// LEG LENGTHS FOR FK / IK

float L1 = 10.0;
float L2 = 10.0;

// BALL POSITION

float ballX = 10;
float ballY = 5;

String inputCommand = "";

// ROBOT STATES

enum RobotState
{
  IDLE,
  WALK_FORWARD,
  TURN_LEFT_STATE,
  TURN_RIGHT_STATE,
  SEARCH_STATE,
  KICK_STATE
};

RobotState currentState = IDLE;

// ANGLE TO PWM
// This function converts a servo angle (0-180) to the corresponding PWM pulse width for the PCA9685

int angleToPulse(int angle)
{
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}


// SAFE SERVO LIMITS
// This function ensures that the servo angles stay within the defined limits to prevent damage

int safeServo(int angle, int minA, int maxA)
{
  return constrain(angle, minA, maxA);
}


// BASIC MOVE
// This function moves a single servo to the specified angle

void moveServo(int channel, int angle)
{
  int pulse = angleToPulse(angle);
  pca.setPWM(channel, 0, pulse);

  Serial.print("CH ");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.println(angle);
}


// SMOOTH MOVE
// This function moves a servo smoothly from a start angle to an end angle by incrementing the angle in small steps with delays

void moveServoSmooth(int channel, int startAngle, int endAngle)
{
  if(startAngle < endAngle)
  {
    for(int a = startAngle; a <= endAngle; a++)
    {
      moveServo(channel, a);
      delay(8);
    }
  }
  else
  {
    for(int a = startAngle; a >= endAngle; a--)
    {
      moveServo(channel, a);
      delay(8);
    }
  }
}


// SMART DELAY
// This function implements a delay that also checks for emergency stop commands
// from the serial input, allowing the robot to stop immediately if needed

void smartDelay(unsigned long ms)
{
  unsigned long start = millis();

  while(millis() - start < ms)
  {
    if(Serial.available())
    {
      String emergency = Serial.readStringUntil('\n');
      emergency.trim();

      if(emergency == "STOP")
      {
        standPose();
        currentState = IDLE;
        return;
      }
    }
  }
}


// STANDING POSE
// This function sets all servos to the angles defined for the standing pose (reset position)

void standPose()
{
  moveServo(0, RH);
  moveServo(1, RK);
  moveServo(2, RA);

  moveServo(3, LH);
  moveServo(4, LK);
  moveServo(5, LA);
}


// FORWARD KINEMATICS
// This function calculates the (x, y) position of the foot based on the angles of the hip and knee joints using trigonometric functions

void forwardKinematics(float theta1, float theta2)
{
  float t1 = radians(theta1);
  float t2 = radians(theta2);

  float x = L1 * cos(t1) + L2 * cos(t1 + t2);
  float y = L1 * sin(t1) + L2 * sin(t1 + t2);

  Serial.print("FK X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.println(y);
}

// INVERSE KINEMATICS
// This function calculates the required angles for the hip and knee joints
// to reach a desired (x, y) position of the foot using geometric relationships and trigonometric functions

void solveIK(float x, float y)
{
  float D = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);

  D = constrain(D, -1, 1);

  float theta2 = acos(D);

  float theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2));

  theta1 = degrees(theta1);
  theta2 = degrees(theta2);

  int hipAngle = safeServo(RH + theta1, 70, 130);
  int kneeAngle = safeServo(RK + theta2, 70, 150);

  Serial.print("IK Hip: ");
  Serial.println(hipAngle);

  Serial.print("IK Knee: ");
  Serial.println(kneeAngle);

  moveServoSmooth(0, RH, hipAngle);
  moveServoSmooth(1, RK, kneeAngle);
}

// SETUP

void setup()
{
  Serial.begin(115200);

  pca.begin();
  pca.setPWMFreq(50);

  delay(2000);

  standPose();

  delay(2000);

  Serial.println("ESP32 READY");
}

// LOOP

void loop()
{
  readSerialCommands();

  switch(currentState)
  {
    case WALK_FORWARD:
      walkForwardOneCycle();
      currentState = IDLE;
      break;

    case TURN_LEFT_STATE:
      turnLeft();
      currentState = IDLE;
      break;

    case TURN_RIGHT_STATE:
      turnRight();
      currentState = IDLE;
      break;

    case SEARCH_STATE:
      searchMotion();
      currentState = IDLE;
      break;

    case KICK_STATE:
      kickRight();
      currentState = IDLE;
      break;

    case IDLE:
      break;
  }
}

// SERIAL COMMANDS
// This function reads commands from the serial input, processes them, and updates the robot's state or parameters accordingly

void readSerialCommands()
{
  if(Serial.available())
  {
    inputCommand = Serial.readStringUntil('\n');
    inputCommand.trim();

    Serial.print("Received: ");
    Serial.println(inputCommand);

    handleCommand(inputCommand);
  }
}


// COMMAND HANDLER
// This function takes a command string, checks its content, and updates the robot's state or parameters based on the command


void handleCommand(String cmd)
{
  if(cmd == "FORWARD")
  {
    currentState = WALK_FORWARD;
  }
  else if(cmd == "LEFT")
  {
    currentState = TURN_LEFT_STATE;
  }
  else if(cmd == "RIGHT")
  {
    currentState = TURN_RIGHT_STATE;
  }
  else if(cmd == "SEARCH")
  {
    currentState = SEARCH_STATE;
  }
  else if(cmd == "KICK")
  {
    currentState = KICK_STATE;
  }
  else if(cmd == "STAND")
  {
    standPose();
  }
  else if(cmd.startsWith("BALL:"))
  {
    int comma = cmd.indexOf(',');

    String xs = cmd.substring(5, comma);
    String ys = cmd.substring(comma + 1);

    ballX = xs.toFloat();
    ballY = ys.toFloat();

    Serial.print("Ball X: ");
    Serial.println(ballX);

    Serial.print("Ball Y: ");
    Serial.println(ballY);
  }
  else
  {
    Serial.println("Unknown command");
  }
}

// WALK FORWARD

void walkForwardOneCycle()
{
  walkRightStep();
  smartDelay(200);

  walkLeftStep();
  smartDelay(200);

  standPose();
  smartDelay(300);
}

// RIGHT STEP


void walkRightStep()
{
  moveServoSmooth(2, RA, RA + LEAN);
  moveServoSmooth(5, LA, LA + LEAN);

  smartDelay(300);

  moveServoSmooth(1, RK, RK + LIFT);

  smartDelay(300);

  moveServoSmooth(0, RH, RH + STEP);

  smartDelay(300);

  moveServoSmooth(1, RK + LIFT, RK);

  smartDelay(300);

  moveServoSmooth(3, LH, LH + PUSH);

  smartDelay(300);

  moveServoSmooth(2, RA + LEAN, RA);
  moveServoSmooth(5, LA + LEAN, LA);

  smartDelay(300);

  moveServoSmooth(0, RH + STEP, RH);
  moveServoSmooth(3, LH + PUSH, LH);

  smartDelay(300);
}

// LEFT STEP

void walkLeftStep()
{
  moveServoSmooth(2, RA, RA - LEAN);
  moveServoSmooth(5, LA, LA - LEAN);

  smartDelay(300);

  moveServoSmooth(4, LK, LK + LIFT);

  smartDelay(300);

  moveServoSmooth(3, LH, LH - STEP);

  smartDelay(300);

  moveServoSmooth(4, LK + LIFT, LK);

  smartDelay(300);

  moveServoSmooth(0, RH, RH - PUSH);

  smartDelay(300);

  moveServoSmooth(2, RA - LEAN, RA);
  moveServoSmooth(5, LA - LEAN, LA);

  smartDelay(300);

  moveServoSmooth(0, RH - PUSH, RH);
  moveServoSmooth(3, LH - STEP, LH);

  smartDelay(300);
}

// TURN LEFT

void turnLeft()
{
  moveServoSmooth(2, RA, RA - 8);
  moveServoSmooth(5, LA, LA - 8);

  smartDelay(300);

  moveServoSmooth(0, RH, RH + 12);
  moveServoSmooth(3, LH, LH + 12);

  smartDelay(400);

  standPose();
}

// TURN RIGHT

void turnRight()
{
  moveServoSmooth(2, RA, RA + 8);
  moveServoSmooth(5, LA, LA + 8);

  smartDelay(300);

  moveServoSmooth(0, RH, RH - 12);
  moveServoSmooth(3, LH, LH - 12);

  smartDelay(400);

  standPose();
}

// SEARCH MOTION

void searchMotion()
{
  for(int i = -15; i <= 15; i += 5)
  {
    moveServo(0, RH + i);
    moveServo(3, LH - i);

    smartDelay(150);
  }

  for(int i = 15; i >= -15; i -= 5)
  {
    moveServo(0, RH + i);
    moveServo(3, LH - i);

    smartDelay(150);
  }

  standPose();
}

// KICK

void kickRight()
{
  standPose();
  delay(700);

  moveServo(0, RH + 6);   
  delay(300);

  moveServo(1, RK + 14);  
  delay(300);

  moveServo(0, RH + 28);
  delay(220);

  moveServo(0, RH + 8);
  delay(250);

  moveServo(1, RK);
  delay(250);

  standPose();
  delay(400);
}