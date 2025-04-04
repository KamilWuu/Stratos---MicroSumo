#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

// DIGITAL_OUTPUTS
#define LED_1 5
#define LED_2 2
#define LED_3 42
#define STARTER_LED 38

#define XSHUT 21
#define RIGHT_XSHUT 8

// ANALOG_OUTPUTS
#define L_MOT_FORW 7 // LEFT_MOTOR_FORWARD
#define L_MOT_BACK 6 // LEFT_MOTOR_BACKWARD

#define R_MOT_FORW 40 // RIGHT_MOTOR_FORWARD
#define R_MOT_BACK 41 // RIGHT_MOTOR_BACKWARD

#define LEFT_SERVO 39
// #define RIGHT_SERVO 8

// DIGITAL_INPUTS
#define BUTTON_LEFT 1
#define BUTTON_RIGHT 4
#define STARTER_PIN 37

// ANALOG INPUTS
#define LEFT_GROUND_SENSOR 10
#define RIGHT_GROUND_SENSOR 9
#define BATTERY_VOLTAGE 3
// I2C
#define SDA_PIN 13
#define SCL_PIN 14

// VARIABLES
#define GROUND_THRESHOLD 3900
#define START_ROT_TIME 2000
#define WHITE_LINE_DETECT_ROT_TIME 700
#define WHITE_LINE_DETECT_BREAKING_TIME 500
#define AFTER_WHITE_LINE_DETECT_FORWARD_TIME 800

#define SEARCH_ROT_TIME 2000 
#define SEARCH_FORW_TIME 1000

#define min_pwm 0                     // Minimalna wartość PWM, żeby silniki się obracały
#define max_pwm 255                   // Maksymalna wartość PWM
#define threshold_detect_distance 255 // Odległość wykrycia przeciwnika (mm)
#define threshold_near_distance 20   // Bardzo bliska odległość (mm)
#define search_pwm 125                // PWM w trybie szukania przeciwnika

void initGPIO()
{
  // Konfiguracja wyjść cyfrowych
  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);
  pinMode(LED_3, OUTPUT);
  pinMode(STARTER_LED, OUTPUT);

  // Konfiguracja pinów pwm
  pinMode(L_MOT_FORW, OUTPUT);
  pinMode(L_MOT_BACK, OUTPUT);
  pinMode(R_MOT_FORW, OUTPUT);
  pinMode(R_MOT_BACK, OUTPUT);
  pinMode(LEFT_SERVO, OUTPUT);
  // pinMode(RIGHT_SERVO, OUTPUT);

  // Konfiguracja wejść cyfrowych
  pinMode(BUTTON_LEFT, INPUT);
  pinMode(BUTTON_RIGHT, INPUT);
  pinMode(STARTER_PIN, INPUT);

  // Konfiguracja wejść analogowych (ESP32 używa ADC)
  pinMode(LEFT_GROUND_SENSOR, INPUT);
  pinMode(RIGHT_GROUND_SENSOR, INPUT);
  pinMode(BATTERY_VOLTAGE, INPUT);

  // Konfiguracja magistrali I2C
  Wire.begin(SDA_PIN, SCL_PIN);
}

class DistanceSensors
{
public:
  // Konstruktor klasy
  DistanceSensors();

  // Destruktor klasy
  ~DistanceSensors() {};
  void initDistanceSensors();
  void readDistances();
  uint16_t getLeftDistance() { return left_distance; };
  uint16_t getRightDistance() { return right_distance; };

private:
  VL53L0X leftTOF;
  VL53L0X rightTOF;
  uint16_t left_distance;
  uint16_t right_distance;
};

DistanceSensors::DistanceSensors()
{
}

void DistanceSensors::initDistanceSensors()
{

  pinMode(XSHUT, OUTPUT);
  pinMode(RIGHT_XSHUT, OUTPUT);
  digitalWrite(XSHUT, LOW);
  digitalWrite(RIGHT_XSHUT, LOW);

  delay(500);

  digitalWrite(RIGHT_XSHUT, HIGH);

  delay(100);

  // Inicjalizacja I²C na wybranych pinach
  Wire.begin(SDA_PIN, SCL_PIN);

  rightTOF.setTimeout(500);
  if (!rightTOF.init())
  {
    // Serial.println("Failed to detect and initialize rightTOF!");
    while (1)
    {
      digitalWrite(LED_3, !digitalRead(LED_3));
      delay(400);
    }
  }

  rightTOF.setAddress(0x30);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).

  digitalWrite(XSHUT, HIGH);
  leftTOF.setTimeout(500);
  if (!leftTOF.init())
  {
    // ln("Failed to detect and initialize leftTOF!");
    while (1)
    {
      digitalWrite(LED_3, !digitalRead(LED_3));
      delay(400);
    }
  }

  rightTOF.startContinuous();
  leftTOF.startContinuous();
}

void DistanceSensors::readDistances()
{
  left_distance = leftTOF.readRangeContinuousMillimeters();
  right_distance = rightTOF.readRangeContinuousMillimeters();

  if (leftTOF.timeoutOccurred())
  {
    // ln(" TIMEOUT_LEFT_TOF");
  }
  if (rightTOF.timeoutOccurred())
  {
    // ln(" TIMEOUT_RIGHT_TOF");
  }

  /*//("Left Distance: ");
  //(left_distance);
  //("Right Distance: ");
  //ln(right_distance);*/
}

class GroundSensors
{
public:
  GroundSensors();
  ~GroundSensors() {};
  void readGroundSensors();
  bool white_on_left;
  bool white_on_right;

private:
  uint16_t left_ground_read;
  uint16_t right_ground_read;
};

GroundSensors::GroundSensors()
{
  left_ground_read = 4096;
  right_ground_read = 4096;
  white_on_left = false;
  white_on_right = false;
}

void GroundSensors::readGroundSensors()
{
  left_ground_read = analogRead(LEFT_GROUND_SENSOR);
  right_ground_read = analogRead(RIGHT_GROUND_SENSOR);

  if (left_ground_read < GROUND_THRESHOLD)
  {
    white_on_left = true;
  }
  else
  {
    white_on_left = false;
  }
  if (right_ground_read < GROUND_THRESHOLD)
  {
    white_on_right = true;
  }
  else
  {
    white_on_right = false;
  }

  /*// Wypisanie wyników dla lewego czujnika
  //("GROUND_SENSORS: left-> ");
  //(left_ground_read);
  //("\t white-> ");
  //(white_on_left);
  //("\t");

  // Wypisanie wyników dla prawego czujnika
  //("right-> ");
  //(right_ground_read);
  //("\t white-> ");
  //ln(white_on_right);  // Zakończenie linii*/
}

class Robot
{
public:
  Robot();            // Konstruktor
  ~Robot();           // Destruktor
  void readSensors(); // Funkcja do odczytu czujników
  bool readStarter()
  {
    starter_state = digitalRead(STARTER_PIN);
    return starter_state;
  };
  void checkButtonClicks();
  bool get_start_rot_state() { return start_rot_state; };
  bool get_start_tactic_state() { return start_tactic_state; };
  void updateMotors();
  bool isOponnentFound();
  bool blackOnDojo();
  void whiteLineDetected(uint16_t left_pwm, uint16_t right_pwm);
  void searchOponnent();
  DistanceSensors tofs;         // Obiekt czujników odległości
  GroundSensors ground_sensors; // Obiekt czujników podłoża
private:
  bool starter_state;
  bool start_rot_state;
  bool start_tactic_state;
};

Robot::Robot()
{
  starter_state = false;
  start_tactic_state = true;
  start_rot_state = true;
  initGPIO(); // Inicjalizacja GPIO
}

void Robot::readSensors()
{
  tofs.readDistances();
  // ground_sensors.readGroundSensors();
}

void Robot::checkButtonClicks()
{
  static bool leftPressed = false;
  static bool rightPressed = false;

  if (digitalRead(BUTTON_RIGHT) == HIGH && !leftPressed)
  {
    leftPressed = true;
    start_tactic_state = !start_tactic_state;
  }
  if (digitalRead(BUTTON_RIGHT) == LOW && leftPressed)
  {
    leftPressed = false;
  }

  if (digitalRead(BUTTON_LEFT) == HIGH && !rightPressed)
  {
    rightPressed = true;
    start_rot_state = !start_rot_state;
  }
  if (digitalRead(BUTTON_LEFT) == LOW && rightPressed)
  {
    rightPressed = false;
  }

  if (start_tactic_state)
  {
    digitalWrite(LED_2, HIGH);
  }
  else
  {
    digitalWrite(LED_2, LOW);
  }

  if (start_rot_state)
  {
    digitalWrite(LED_1, HIGH);
    digitalWrite(LED_3, LOW);
  }
  else
  {
    digitalWrite(LED_1, LOW);
    digitalWrite(LED_3, HIGH);
  }
}

void ledWaveEffect()
{
  int delay_time = 50; // Czas opóźnienia między zmianami
  int leds[] = {LED_1, LED_2, LED_3, STARTER_LED};
  int num_leds = sizeof(leds) / sizeof(leds[0]);

  // Fala świetlna w przód
  for (int i = 0; i < num_leds; i++)
  {
    digitalWrite(leds[i], HIGH);
    delay(delay_time);
  }

  // Fala świetlna w tył
  for (int i = num_leds - 1; i >= 0; i--)
  {
    digitalWrite(leds[i], LOW);
    delay(delay_time);
  }

  // Powtórz efekt kilka razy
  for (int j = 0; j < 3; j++)
  {
    for (int i = 0; i < num_leds; i++)
    {
      digitalWrite(leds[i], HIGH);
      delay(delay_time);
      digitalWrite(leds[i], LOW);
    }
    for (int i = num_leds - 1; i >= 0; i--)
    {
      digitalWrite(leds[i], HIGH);
      delay(delay_time);
      digitalWrite(leds[i], LOW);
    }
  }
}

bool Robot::isOponnentFound()
{
  readSensors();
  uint16_t L_dist = tofs.getLeftDistance();
  uint16_t R_dist = tofs.getRightDistance();
  if (L_dist < threshold_detect_distance || R_dist < threshold_detect_distance)
  {
    digitalWrite(LED_3, HIGH);
    return true;
  }
  else
  {
    analogWrite(LED_1, 0);
    analogWrite(LED_2, 0);
    digitalWrite(LED_3, 0);
    return false;
  }
}

void Robot::updateMotors()
{
  // readSensors();
  // readSensors();

  uint16_t L_pwm = 0;
  uint16_t R_pwm = 0;
  uint16_t L_dist = tofs.getLeftDistance();
  uint16_t R_dist = tofs.getRightDistance();
  if (L_dist < threshold_near_distance || R_dist < threshold_near_distance)
  {
    L_pwm = max_pwm;
    R_pwm = max_pwm;
  }
  else
  {
    if (L_dist < threshold_detect_distance && R_dist < threshold_detect_distance)
    {
      L_pwm = max_pwm;
      R_pwm = max_pwm;
    }
    else if (L_dist < threshold_detect_distance)
    {
      L_pwm = 0 + map(L_dist, threshold_near_distance, threshold_detect_distance, min_pwm, max_pwm);
      R_pwm = max_pwm; // 2 + map(L_dist, threshold_near_distance, threshold_detect_distance, min_pwm, max_pwm/2);
    }
    else if (R_dist < threshold_detect_distance)
    {
      L_pwm = max_pwm; // 2 + map(R_dist, threshold_near_distance, threshold_detect_distance, min_pwm, max_pwm/2);
      R_pwm = 0 + map(R_dist, threshold_near_distance, threshold_detect_distance, min_pwm, max_pwm);
    }
  }

  // Ograniczenie wartości PWM
  L_pwm = constrain(L_pwm, min_pwm, max_pwm);
  R_pwm = constrain(R_pwm, min_pwm, max_pwm);
  // if(oponnent_found){
  analogWrite(L_MOT_FORW, L_pwm);
  analogWrite(R_MOT_FORW, R_pwm);

  analogWrite(LED_1, R_pwm);
  analogWrite(LED_2, L_pwm);
}

void Robot::whiteLineDetected(uint16_t left_pwm, uint16_t right_pwm)
{
    uint32_t phaseStartTime = millis();
    bool reversing = true;
    bool rotating = false;
    bool movingForward = false;

    analogWrite(L_MOT_FORW, 0);
    analogWrite(R_MOT_FORW, 0);
    analogWrite(L_MOT_BACK, max_pwm);
    analogWrite(R_MOT_BACK, max_pwm);

    while (!isOponnentFound())
    {
        if (reversing && millis() - phaseStartTime > WHITE_LINE_DETECT_BREAKING_TIME)
        {
            reversing = false;
            rotating = true;
            phaseStartTime = millis();
            analogWrite(L_MOT_BACK, 0);
            analogWrite(R_MOT_BACK, 0);
            analogWrite(L_MOT_FORW, left_pwm);
            analogWrite(R_MOT_FORW, right_pwm);
        }
        else if (rotating && millis() - phaseStartTime > WHITE_LINE_DETECT_ROT_TIME)
        {
            rotating = false;
            movingForward = true;
            phaseStartTime = millis();
            analogWrite(L_MOT_FORW, max_pwm);
            analogWrite(R_MOT_FORW, max_pwm);
        }
        else if (movingForward && (millis() - phaseStartTime > AFTER_WHITE_LINE_DETECT_FORWARD_TIME || ground_sensors.white_on_left || ground_sensors.white_on_right))
        {
            break;
        }
  }
  
            analogWrite(L_MOT_BACK, 0);
            analogWrite(R_MOT_BACK, 0);
            analogWrite(L_MOT_FORW, max_pwm);
            analogWrite(R_MOT_FORW, max_pwm);
}


bool Robot::blackOnDojo()
{
  if (analogRead(LEFT_GROUND_SENSOR) < GROUND_THRESHOLD)
  {
    whiteLineDetected(max_pwm, min_pwm);
    return false;
  }
  else if (analogRead(RIGHT_GROUND_SENSOR) < GROUND_THRESHOLD)
  {
    whiteLineDetected(min_pwm, max_pwm);
    return false;
  }
  else
  {
    return true;
  }
}

void Robot::searchOponnent()
{
    uint32_t startTime = millis();
    uint32_t phaseStartTime = millis();
    bool rotating = true;

    while (!isOponnentFound() && blackOnDojo() && readStarter())
    {
        if (rotating)
        {
            analogWrite(L_MOT_FORW, search_pwm);
            analogWrite(R_MOT_FORW, min_pwm);
            if (millis() - phaseStartTime > SEARCH_ROT_TIME)
            {
                rotating = false;
                phaseStartTime = millis();
            }
        }
        else
        {
            analogWrite(L_MOT_FORW, search_pwm);
            analogWrite(R_MOT_FORW, search_pwm);
            if (millis() - phaseStartTime > SEARCH_FORW_TIME)
            {
                rotating = true;
                phaseStartTime = millis();
            }
        }
    }
    // analogWrite(L_MOT_FORW, 0);
    // analogWrite(R_MOT_FORW, 0);
}

Robot Stratos; // Tworzymy obiekt klasy Robot

void setup()
{
    analogWrite(L_MOT_FORW, 0);
    analogWrite(R_MOT_FORW, 0);
    Serial.begin(115200);
    Stratos.tofs.initDistanceSensors();
    ledWaveEffect();

    while (!Stratos.readStarter())
    {
        Stratos.checkButtonClicks();
    }

    digitalWrite(STARTER_LED, HIGH);
    uint32_t phaseStartTime = millis();
    bool rotating = true;

    if (Stratos.get_start_rot_state())
    {
        analogWrite(L_MOT_FORW, max_pwm);
        analogWrite(R_MOT_FORW, min_pwm);
    }
    else
    {
        analogWrite(L_MOT_FORW, min_pwm);
        analogWrite(R_MOT_FORW, max_pwm);
    }

    while (!Stratos.isOponnentFound() && rotating)
    {
        if (millis() - phaseStartTime > START_ROT_TIME)
        {
            rotating = false;
        }
    }
}


void loop()
{

  // glowna petla jazdy
  /*if(Stratos.get_start_tactic_state()){
    //pierwsza taktyka
  }else{
    //druga taktyka
  }*/
  // na razie bez taktyk
  // Stratos.readSensors();
  if (Stratos.blackOnDojo())
  {
    if (Stratos.isOponnentFound())
    {
      Stratos.updateMotors();
    }
    else
    {
      Stratos.searchOponnent();
    }
  }

  while (!Stratos.readStarter())
  {
    // czeka na reset po walces
    analogWrite(L_MOT_FORW, 0);
    analogWrite(R_MOT_FORW, 0);
    digitalWrite(STARTER_LED, LOW);
    delay(300);
    digitalWrite(STARTER_LED, HIGH);
    delay(300);
  }
}
