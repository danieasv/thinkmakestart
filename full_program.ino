#define BROKER_IP    "192.168.1.212"
#define DEV_NAME     "mqttdevice"
#define MQTT_USER    "mqtt_user"
#define MQTT_PW      "mqtt_password"
const char ssid[] = "TMSBatch9";
const char pass[] = "12345678";
#include <MQTT.h>
#include <WiFiNINA.h>

#include <Wire.h>
#include <I2Cdev.h>
#include <SparkFun_TB6612.h>
#include <Servo.h>

#define BOARD_WIFI 1
#ifdef BOARD_WIFI
  // Pins for Arduino MEGA
  #define ENABLE_WIFI 1
  #define AIN1 4
  #define AIN2 5
  #define BIN1 2
  #define BIN2 3
  #define PWMA 6
  #define PWMB 7
  #define STBY 12
  #define VACUUM 8
  #define SERVO 9
  //#define TRIGGER_PIN 11
  //#define ECHO_PIN 12
#elif
// Pins for MEGA
  #define ENABLE_WIFI 0
  #define AIN1 22
  #define AIN2 23
  #define BIN1 24
  #define BIN2 25
  #define PWMA 2
  #define PWMB 3
  #define STBY 13
  #define VACUUM 48
  #define SERVO 9
  #define TRIGGER_PIN 11
  #define ECHO_PIN 12
#endif

// Setup motors & servo
const int offsetMotor1 = 1;
const int offsetMotor2 = 1;
const int defaultSpeed = 100;
Servo myservo;
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetMotor1, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetMotor2, STBY);
bool servoOn = false;
bool vacuumOn = false;
bool motorOn = false;
bool ledOn = false;
int buttCounter = 5;

// Obstacle avoidance
long duration;
int distance;
 
// Servo state
int servoMaxSpeed = 3;
int servoCurrent = 90;
int servoTarget = 90;

// Global settings
int sleepTime = 15;

#ifdef ENABLE_WIFI
  WiFiClient net;
  MQTTClient client;
  unsigned long lastMillis = 0;
#endif



void setup() {

 
 // Vacuum setup
  pinMode(VACUUM, OUTPUT);
  //pinMode(TRIGGER_PIN, OUTPUT);
  
  //pinMode(ECHO_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT); // Built in LED
  
  // Servo 1 setup
  myservo.attach(SERVO);
  
  // Serial communication
  Serial.begin(9600);

 // MQTT brokers usually use port 8883 for secure connections.
 #ifdef ENABLE_WIFI
   WiFi.begin(ssid, pass);
   client.begin(BROKER_IP, 1883, net);
   client.onMessage(messageReceived);
   connect();
 #endif
}
void loop() {
#ifdef ENABLE_WIFI
 client.loop();
 /*
 if (!client.connected()) {
   connect();
 }
 */
  // publish a message roughly every second
 if (millis() - lastMillis > 1000) {
   lastMillis = millis();
   client.publish("/hello", "Alive"); //PUBLISH TO TOPIC /hello MSG: Alive
 }
#endif
// Update targets
  updateCommands();
  
  // Accelerate the servo
  int diff = servoTarget - servoCurrent;
  if(diff != 0){
    Serial.print("updating servo");
    Serial.println(servoCurrent);
    int servoSpeed = min(diff, servoMaxSpeed);
    servoSpeed = max(servoSpeed, -servoMaxSpeed);
    servoCurrent = servoCurrent + servoSpeed;
    myservo.write(servoCurrent); 
    delay(sleepTime);
  }
}

void updateCommands(){
  const unsigned int cmdSize = 4;
  int avail = Serial.available();
  if (avail >= cmdSize) {
    Serial.print("-------- buffer full - avail: ");
    Serial.println(avail);
    char cmdRaw[cmdSize];
    for (int i=0; i<cmdSize; i++) {
      cmdRaw[i] = Serial.read();
    }
    // byte recLen = Serial.readBytes(cmdRaw, cmdSize);
    
    // decode command:
    int cmdAngle = (unsigned int) (byte) cmdRaw[0];
    int cmdVacuum = (unsigned int) (byte) cmdRaw[1];
    int cmdMotor1 = (unsigned int) (byte) cmdRaw[2];
    int cmdMotor2 = (unsigned int) (byte) cmdRaw[3];
    
    // debug info over serial
    Serial.print("servo angle: ");
    Serial.println(cmdAngle);
    Serial.print("vacuum: ");
    Serial.println(cmdVacuum);
    Serial.print("motor1: ");
    Serial.println(cmdMotor1);
    Serial.print("motor2: ");
    Serial.println(cmdMotor2);
    
    // action
    controlServo(cmdAngle);
    controlMotors(cmdMotor1, cmdMotor2);
    controlVacuum(cmdVacuum);
    // wait for more data in the buffer
    return;
  }
}
void controlServo(int cmdAngle) {
  if (cmdAngle >= 40 && cmdAngle <= 140){
    servoTarget = cmdAngle;
  } else {
    Serial.println("Angle out of range!");
  }
}

void controlMotors(int cmdMotor1, int cmdMotor2){
  if (motorOn == true) {
    motor1.drive(cmdMotor1);
    motor2.drive(cmdMotor2);
  }
  else {
    motor1.drive(0);
    motor2.drive(0);
  }
}

void controlVacuum(int cmdVacuum){
  if(cmdVacuum == 0 or vacuumOn == false){
    digitalWrite(VACUUM, LOW);
    //digitalWrite(LED_BUILTIN, LOW);
  }
  else {
    digitalWrite(VACUUM, HIGH);
    //digitalWrite(LED_BUILTIN, HIGH);
  }
}
/*
void moveAround() {
  for(int i = 0; i<5; i++) {
    moveMotor(true);
    delay(1000);
    moveRight();
    delay(2500);  
  }
}

void moveMotor(bool startMotor) {
  digitalWrite(VACUUM, LOW);
  forward(motor1, motor2, defaultSpeed);  
  measureDistance(); 
// DISTANCE CALCULATOR & AVOIDANCE - need more calibration
//  if(startMotor) {
//    if (distance < 40) {
//      Serial.print("Avoiding obstacle: ");
//      stopVacuum();
//      moveRight(); 
//    } else {
//      Serial.print("Moving forward: ");
//      forward(motor1, motor2, defaultSpeed);  
//    } 
//  } else {
//    brake(motor1, motor2);
//  }
}
*/
/*
void moveRight() {
  back(motor1, motor2, defaultSpeed);
  delay(600);
  motor1.brake();
  motor2.drive(defaultSpeed);
}

void moveLeft() {
  back(motor1, motor2, defaultSpeed);
  delay(600);
  motor2.brake();
  motor1.drive(defaultSpeed);
}
*/
/*
void measureDistance() {
  delay(350);
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.println(distance);
}
*/
/*
void startVacuum() {
  Serial.println("set vacuum on");
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(VACUUM, HIGH);   
}

void stopVacuum() {
  Serial.println("set vacuum off");
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(VACUUM, LOW );   
}
*/
#ifdef ENABLE_WIFI
void connect() {
 Serial.print("Checking WiFi...");
 while (WiFi.status() != WL_CONNECTED) {
   Serial.print(".");
   delay(300);
 }
 Serial.print("\nConnecting...");
 while (!client.connect(DEV_NAME, MQTT_USER, MQTT_PW)) {
   Serial.print(".");
   delay(1000);
 }
 Serial.println("\nConnected!");
 client.subscribe("/led");
 client.subscribe("/vacuume");
 client.subscribe("/motor");
 client.subscribe("/hello");
}
void messageReceived(String &topic, String &payload) {
 Serial.println("incoming: " + topic + " - " + payload);
 if (topic == "/led") {
   if (payload == "open") {
     Serial.println("LED on");
     ledOn = true;
     digitalWrite(LED_BUILTIN, HIGH); 
   } else if (payload == "closed") {
     Serial.println("LED off");
     ledOn = false;
     digitalWrite(LED_BUILTIN, LOW); 
   }
 }
 else if (topic == "/vacuume") {
   if (payload == "open") {
     Serial.println("Vacuume On");
     vacuumOn = true;
     digitalWrite(LED_BUILTIN, HIGH); 
   } else if (payload == "closed") {
     Serial.println("Vacuume Off");
     vacuumOn = false;
     digitalWrite(LED_BUILTIN, LOW); 
   }
 }
 else if (topic == "/motor") {
   if (payload == "open") {
     Serial.println("Motor On");
     motorOn = true;
     //digitalWrite(LED_BUILTIN, HIGH); 
   } else if (payload == "closed") {
     Serial.println("Motor Off");
     motorOn = false;
     digitalWrite(LED_BUILTIN, LOW); 
   }
 }
}
#endif
