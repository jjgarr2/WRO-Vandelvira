Our team has designed a car that uses a camera and ultrasonic sensors to detect obstacles and decide the direction the car should take.
The movement is provided by a motor connected to the wheels, and the steering is controlled with a servomotor.
All of this is managed by an Arduino board.
The chassis and necessary supports have been designed and printed with a 3D printer.
The first code developed was to control the motor:
// Define pins connected to the L298
const int IN1 = 8;
const int IN2 = 9;
const int EN = 10; // PWM pin for speed control

void setup() {
  // Set pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN, OUTPUT);
  // Start with motor stopped
  analogWrite(EN, 0);
}

void loop() {
  // Forward movement at medium speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(EN, 128); // PWM value from 0 to 255
  delay(2000); // Runs for 2 seconds

  // Stop the motor
  analogWrite(EN, 0);
  delay(1000);

  // Reverse movement at medium speed
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(EN, 128);
  delay(2000);

  // Stop the motor
  analogWrite(EN, 0);
  delay(1000);
}
The second step involved controlling the ultrasonic sensors:
// Define pins for ultrasonic sensor
const int trigPin = 9;
const int echoPin = 10;

void setup() {
  Serial.begin(9600); // Start serial communication
  pinMode(trigPin, OUTPUT); // Trigger pin as output
  pinMode(echoPin, INPUT);  // Echo pin as input
}

void loop() {
  long duration;
  int distance;

  // Clear the trigger pin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10 microsecond pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the time it takes to receive the echo
  duration = pulseIn(echoPin, HIGH);

  // Calculate distance in centimeters
  distance = duration * 0.034 / 2;

  // Display distance on serial monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(500); // Wait half a second before next measurement
}




The third step was learning how to use the Huskylens camera.
https://tienda.bricogeek.com/sensores-imagen/1410-huskylens-camara-de-vision-artificial.html?gad_source=1&gad_campaignid=21357420419&gbraid=0AAAAADkb14c8pbOZMZvrRHhXSlpFI41Do&gclid=Cj0KCQjwucDBBhDxARIsANqFdr0yABA2sA7NPtO3XTR2A8D_mi5mY-dF5QxYfudwRpO-3IjBt_v5u5QaAskGEALw_wcB
#include <Huskylens.h>

Huskylens huskylens;

void setup() {
  Serial.begin(9600);
  // Connect Huskylens to the I2C interface
  huskylens.begin();
  // Configure the camera to detect objects of a specific color
  huskylens.write(0x32, 0x00); // Initial setup
  Serial.println("Huskylens initialized");
}

void loop() {
  // Request object detection
  if (huskylens.request()) {
    if (huskylens.isAppear()) {
      // Get information about the detected object
      uint8_t objectType = huskylens.read(0x54);
      int16_t xCenter = huskylens.read(0x56);
      int16_t yCenter = huskylens.read(0x58);
      
      Serial.print("Object detected - Type: ");
      Serial.print(objectType);
      Serial.print(" - X: ");
      Serial.print(xCenter);
      Serial.print(" - Y: ");
      Serial.println(yCenter);
      
      // If it detects an object of a specific type (e.g., red color)
      if (objectType == 1) { // 1 could represent a specific color
        digitalWrite(LED_BUILTIN, HIGH); // Turn on the LED
      } else {
        digitalWrite(LED_BUILTIN, LOW); // Turn off the LED
      }
    }
  }
  delay(100);
}
The fourth step is also to use a servo motor to control the direction.
#include <Servo.h>

Servo myServo;  // create a servo object to control the servo motor

void setup() {
  myServo.attach(9);  // connect the servo motor to digital pin 9
}

void loop() {
  // Rotate the servo to 0 degrees
  myServo.write(0);
  delay(1000); // wait 1 second

  // Rotate the servo to 90 degrees
  myServo.write(90);
  delay(1000); // wait 1 second

  // Rotate the servo to 180 degrees
  myServo.write(180);
  delay(1000); // wait 1 second
}

Current status. Once we separately control these components, we need to integrate them all together, which has taken many hours and is still in progress.
Initial code tested and causing issues.
#include <Servo.h>
// Pins for the motor with L298N
const int motorPin1 = 2; // IN1
const int motorPin2 = 3; // IN2
const int enablePin = 4; // ENA (PWM)
// Pins for the ultrasonic sensor
const int trigPin = 9;
const int echoPin = 10;
// Pins for Huskylens

// Pin for the servo motor
const int servoPin = 6;
Servo direccionServo;
// Variables
long duration;
int distance;
void setup() {
  // Pin configuration
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  // Start serial communication
  Serial.begin(9600);
  // Configure servo
  direccionServo.attach(servoPin);
  // Start motor
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  analogWrite(enablePin, 255); // maximum speed
}
void loop() {
  // Measure distance with ultrasonic sensor
  distance = readUltrasonicDistance();
  if (distance < 10) {
    // Obstacle detected, stop motor
    analogWrite(enablePin, 0);
    // Detect color with Huskylens
    String color = detectColor();

    if (color == "green") {
      // Turn left
      girarIzquierda();
    } else if (color == "pink") {
      // Turn right
      girarDerecha();
    }
    // After turning, move forward again
    delay(1000);
    avanzar();
  } else {
    // No obstacles, move forward
    avanzar();
  }
  delay(100); // small delay
}
int readUltrasonicDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  int distanceCm = duration * 0.034 / 2;
  return distanceCm;
}
String detectColor() {
  // Function implementation not provided
}

