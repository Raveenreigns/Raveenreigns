 #include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <AFMotor.h>
#include <NewPing.h>
#include <Servo.h>
#include <DHT.h>
#include <Adafruit_BMP085.h>
#include <MPU6050.h>

#define TRIG_PIN A0
#define ECHO_PIN A1
#define LDR_PIN A2 // Analog pin connected to the LDR module
#define MAX_DISTANCE 200
#define MAX_SPEED 190 // sets speed of DC motors
#define MAX_SPEED_OFFSET 20

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);
Servo myservo;

boolean goesForward = false;
int distance = 100;
int speedSet = 0;

// DHT11 setup
#define DHTPIN 2 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 // DHT 11

DHT dht(DHTPIN, DHTTYPE);

// BMP180 setup
Adafruit_BMP085 bmp;

// MPU6050 setup
MPU6050 mpu;

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

void setup() {
  Serial.begin(9600);
  Wire.begin();
  dht.begin();
  bmp.begin();
  mpu.begin();

  lcd.init();                      // Initialize the LCD
  lcd.backlight();                 // Turn on the backlight

  myservo.attach(10);
  myservo.write(115);
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);

  lcd.clear();                     // Clear the LCD screen
  lcd.setCursor(0, 0);             // Set cursor to first column of first row
  lcd.print("Distance: ");
}

void loop() {
  int distanceR = 0;
  int distanceL = 0;
  delay(40);

  if (distance <= 15)
  {
    moveStop();
    delay(100);
    moveBackward();
    delay(300);
    moveStop();
    delay(200);
    distanceR = lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);

    if (distanceR >= distanceL)
    {
      turnRight();
      moveStop();
    }
    else
    {
      turnLeft();
      moveStop();
    }
  }
  else
  {
    moveForward();
  }
  distance = readPing();

  // Read and print DHT11 data
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t))
  {
    Serial.println("Failed to read from DHT sensor!");
  }
  else
  {
    lcd.setCursor(0, 1);             // Set cursor to first column of second row
    lcd.print("Humidity: ");
    lcd.print(h);
    lcd.print(" %  ");
    lcd.print("Temp: ");
    lcd.print(t);
    lcd.print(" C");
  }

  // Read and print BMP180 data
  lcd.setCursor(0, 2);             // Set cursor to first column of third row
  lcd.print("Pressure: ");
  lcd.print(bmp.readPressure());
  lcd.print(" Pa  ");

  // Read and print MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  lcd.setCursor(0, 3);             // Set cursor to first column of fourth row
  lcd.print("Acc: ");
  lcd.print(ax);
  lcd.print(" ");
  lcd.print(ay);
  lcd.print(" ");
  lcd.print(az);
  lcd.print("  ");
  lcd.print("Gyro: ");
  lcd.print(gx);
  lcd.print(" ");
  lcd.print(gy);
  lcd.print(" ");
  lcd.print(gz);

  // Read and print LDR value
  int ldrValue = analogRead(LDR_PIN);
  lcd.setCursor(0, 4);             // Set cursor to first column of fifth row
  lcd.print("LDR: ");
  lcd.print(ldrValue);
}

int lookRight() {
  myservo.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(115);
  return distance;
}

int lookLeft() {
  myservo.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  myservo.write(115);
  return distance;
  delay(100);
}

int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0)
  {
    cm = 250;
  }
  return cm;
}

void moveStop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void moveForward() {

  if (!goesForward)
  {
    goesForward = true;
    motor1.run(FORWARD);
    motor2.run(FORWARD);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) // slowly bring the speed up to avoid loading down the batteries too quickly
    {
      motor1.setSpeed(speedSet);
      motor2.setSpeed(speedSet);
      motor3.setSpeed(speedSet);
      motor4.setSpeed(speedSet);
      delay(5);
    }
  }
}

void moveBackward() {
  goesForward = false;
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  for (speedSet = 0; speedSet < MAX_SPEED; speedSet += 2) // slowly bring the speed up to avoid loading down the batteries too quickly
  {
    motor1.setSpeed(speedSet);
    motor2.setSpeed(speedSet);
    motor3.setSpeed(speedSet);
    motor4.setSpeed(speedSet);
    delay(5);
  }
}

void turnRight() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  delay(500);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void turnLeft() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(500);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}
