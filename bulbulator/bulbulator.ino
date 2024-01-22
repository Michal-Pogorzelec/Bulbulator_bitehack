
#include <ArduinoHttpClient.h>
// #include <WiFiNINA.h>  // use this for MKR1010 and Nano 33 IoT
#include <WiFiS3.h>    // use this for MKR1000
// your passwords go in arduino_secrets.h
#include "wifi_secrets.h"

#include <Servo.h>

/* DEFINES */
#define HIGH_ANALOG (255)
#define STEP_PERIOD (300)
#define TURNING_TIME (2000)
#define TURNING_STEP (200)
#define KARETKA_TIME (3500)
#define LAST_BULB_INDEX (4)
#define TURNING_ITERS (3)
#define HIGH_SPEED (200)
#define LOW_SPEED (50)

#define IDLE (0)
#define START_CHECKING (1)
#define MOVE_FORWARD (3)
#define PULL_BULB (4)
#define GO_BACKWARD (5)
#define EXCHANGE_BULB (6)
#define GET_NEW_BULB (7)
#define GET_BACK_TO_BROKEN_BULB (8)
#define REPLACING_BULB (9)
#define GET_BACK_TO_BASE (10)


/* WEBSOCKET */
// settings for a test on postman.com's websocket echo server
// set up a WiFi client ( using SSL):
WiFiSSLClient wifi;
const char serverAddress[] = "bitehack-bulbulator-server.up.railway.app";
const int port = 443;           // standard HTTPS port
 char endpoint[] = "/raw";
// initialize the webSocket client
WebSocketClient client = WebSocketClient(wifi, serverAddress, port);

/* SERVO */
const int leftServoPin = 4; // pin sterujący Arduino
Servo leftServo;
int leftServoAngle = 0; // pozycja startowa serwa w stopniach

const int rightServoPin = 7; // pin sterujący Arduino
Servo rightServo;
int rightServoAngle = 0; // pozycja startowa serwa w stopniach

const int leftLimitSwitchPin = 12;
const int rightLimitSwitchPin = 13;

/* ENGINES */
const int engine1_PIN1 = 2;
const int engine1_PIN2 = 3;

// int engine_speed = A5;

const int karetkaEngine_PIN1 = A2;
const int karetkaEngine_PIN2 = A3;

const int bulbTurnEngine_PIN1 = A4;
const int bulbTurnEngine_PIN2 = A5;

/* GENERAL */
const int fotoresistorPIN = A0;
const int tcrt5000PIN = A1;
// message sending interval, in ms:
int interval = 1000;
// last time a message was sent, in ms:
long lastSend = 0;
int Status;
int internalState = IDLE;
int currentBulbIndex = -1;
int backBulbIndex = -1;
int goingToBaseBulbIndex = -1;
bool lastBulbCheck = false;
int bulbsChecked[LAST_BULB_INDEX] = {0};

bool ignoreNextMsgFromServer = false;
bool inProcess = false;



void moveForward() {
  digitalWrite(engine1_PIN1, HIGH);
  digitalWrite(engine1_PIN2, LOW);
}

void moveBackward() {
  digitalWrite(engine1_PIN1, LOW);
  digitalWrite(engine1_PIN2, HIGH);
}

void stopMoving() {
  digitalWrite(engine1_PIN1, LOW);
  digitalWrite(engine1_PIN2, LOW);
}

void turnBulbOutEngine() {
  digitalWrite(bulbTurnEngine_PIN1, HIGH);
  digitalWrite(bulbTurnEngine_PIN2, LOW);
}

void turnBulbInEngine() {
  digitalWrite(bulbTurnEngine_PIN1, LOW);
  digitalWrite(bulbTurnEngine_PIN2, HIGH);
}

void stopTurning() {
  digitalWrite(bulbTurnEngine_PIN1, LOW);
  digitalWrite(bulbTurnEngine_PIN2, LOW);
}

void moveKaretkaIn() {
  digitalWrite(karetkaEngine_PIN1, HIGH);
  digitalWrite(karetkaEngine_PIN2, LOW);
}

void moveKaretkaOut() {
  digitalWrite(karetkaEngine_PIN1, LOW);
  digitalWrite(karetkaEngine_PIN2, HIGH);
}

void stopKaretka() {
  digitalWrite(karetkaEngine_PIN1, LOW);
  digitalWrite(karetkaEngine_PIN2, LOW);
}

void grabBulb() {
  /* impl z krańcówkami */
  // int leftLimitSwitchRead = LOW;
  // int rightLimitSwitchRead = LOW;
  // bool touched;
  // do {
  //   if (leftLimitSwitchRead == LOW) {
  //     leftServoAngle++;
  //     leftServo.write(leftServoAngle);
  //     delay(10);
  //   }
  //   if (rightLimitSwitchRead == LOW) {
  //     rightServoAngle++;
  //     rightServo.write(rightServoAngle);
  //     delay(10);
  //   }
  //   leftLimitSwitchRead = digitalRead(leftLimitSwitchPin);
  //   rightLimitSwitchRead = digitalRead(rightLimitSwitchPin);
  //   touched = (leftLimitSwitchRead == LOW || rightLimitSwitchRead == LOW) ? true : false;
  // } while (!touched);

  /* bez krańcówek */
  bool grabbed;
  do {
    if (leftServoAngle <= 90) {
      leftServoAngle++;
      leftServo.write(leftServoAngle);
      delay(10);
    }
    if (rightServoAngle <= 90) {
      rightServoAngle++;
      rightServo.write(rightServoAngle);
      delay(10);
    }
    grabbed = (leftServoAngle == 90 && rightServoAngle == 90) ? true : false;
  } while (!grabbed);
}

void releaseBulbToTheEnd() {
  bool released;
  do {
    if (leftServoAngle >= 1) {
      leftServoAngle--;
      leftServo.write(leftServoAngle);
      delay(10);
    }
    if (rightServoAngle >= 1) {
      rightServoAngle--;
      rightServo.write(rightServoAngle);
      delay(10);
    }
    released = (leftServoAngle != 0 && rightServoAngle != 0) ? false : true;
  } while (!released);
}

void releaseBulbPartialy() {
  /* impl z krańcówkami
  int leftLimitSwitchRead = HIGH;
  int rightLimitSwitchRead = HIGH;
  bool touched;
  do {
    if (leftLimitSwitchRead == HIGH) {
      leftServoAngle--;
      leftServo.write(leftServoAngle);
      delay(10);
    }
    if (rightLimitSwitchRead == HIGH) {
      rightServoAngle--;
      rightServo.write(rightServoAngle);
      delay(10);
    }
    leftLimitSwitchRead = digitalRead(leftLimitSwitchPin);
    rightLimitSwitchRead = digitalRead(rightLimitSwitchPin);
    touched = (leftLimitSwitchRead == LOW || rightLimitSwitchRead == LOW) ? true : false;

  } while (touched);
  */
  // bez krańcówek
  bool released;
  int iters = 15;
  do {
    if (leftServoAngle >= 1) {
      leftServoAngle--;
      leftServo.write(leftServoAngle);
      delay(10);
    }
    if (rightServoAngle >= 1) {
      rightServoAngle--;
      rightServo.write(rightServoAngle);
      delay(10);
    }
    iters--;
    released = (iters > 0 || (leftServoAngle != 0 && rightServoAngle != 0)) ? false : true;
  } while (!released);

}

void turnOutTheBulb() {
  for (int number = 0; number < TURNING_ITERS; number++) {
    turnBulbOutEngine();
    delay(TURNING_STEP);
    stopTurning();

    releaseBulbPartialy();

    turnBulbInEngine();
    delay(TURNING_STEP);
    stopTurning();

    grabBulb();

  }
}

void turnInTheBulb() {
  for (int number = 0; number < TURNING_ITERS; number++) {
    turnBulbInEngine();
    delay(TURNING_STEP);
    stopTurning();

    releaseBulbPartialy();

    turnBulbOutEngine();
    delay(TURNING_STEP);
    stopTurning();

    grabBulb();
  }
  releaseBulbToTheEnd();
}

void pullBulb() {
  moveKaretkaIn();
  delay(KARETKA_TIME);
  stopKaretka();

  // chwyć żarówkę
  grabBulb();

  // odkręć żarówkę
  turnOutTheBulb();

  moveKaretkaOut();
  delay(KARETKA_TIME);
  stopKaretka();
}

void putBulb() {
  moveKaretkaIn();
  delay(KARETKA_TIME);
  stopKaretka();

  turnInTheBulb();

  moveKaretkaOut();
  delay(KARETKA_TIME);
  stopKaretka();
}

void sendMSG(String message, int param) {
    message.replace("PARAM", String(param));
    // send the message:
    client.beginMessage(TYPE_TEXT);
    client.print(message);
    client.endMessage();
    ignoreNextMsgFromServer = true;
}

void iAmRobotMSG() {
    if (client.connected()) {
    client.beginMessage(TYPE_TEXT);
    client.print("{\"type\": \"i-am-robot\"}");
    client.endMessage();
    ignoreNextMsgFromServer = true;
  }
}



void setup() {
  pinMode(engine1_PIN1, OUTPUT);
  pinMode(engine1_PIN2, OUTPUT);
  pinMode(karetkaEngine_PIN1, OUTPUT);
  pinMode(karetkaEngine_PIN2, OUTPUT);
  pinMode(bulbTurnEngine_PIN1, OUTPUT);
  pinMode(bulbTurnEngine_PIN2, OUTPUT);

  pinMode(leftLimitSwitchPin, INPUT);
  pinMode(rightLimitSwitchPin, INPUT);

  digitalWrite(engine1_PIN1, LOW);
  digitalWrite(engine1_PIN2, LOW);
  digitalWrite(karetkaEngine_PIN1, LOW);
  digitalWrite(karetkaEngine_PIN2, LOW);
  digitalWrite(bulbTurnEngine_PIN1, LOW);
  digitalWrite(bulbTurnEngine_PIN2, LOW);

  Serial.begin(9600);
  if (!Serial) delay(3000);

  // connect to WIFi:
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to Network named: ");
    Serial.println(SECRET_SSID);
    // Connect to WPA/WPA2 network:
    WiFi.begin(SECRET_SSID, SECRET_PASS);
  }

  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  // If there's an API endpoint to connect to, add it here.
  // leave blank if there's no endpoint:
  int connectStatus;
  connectStatus = client.begin(endpoint);
  Serial.print("Status: ");
  Serial.println(connectStatus);
  iAmRobotMSG();

  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
}

void loop() {
  // if not connected to the socket server, try to connect:
  if (!client.connected()) {
    client.begin();
    delay(1000);
    Serial.println("Attempting to connect to server");
    if (client.connected()) { iAmRobotMSG();}
    // skip the rest of the loop:
    // return;
  }
  
  if (!inProcess) {
    // check if a message is available to be received
    int messageSize = client.parseMessage();
    // if there's a string with length > 0:
    if (messageSize > 0) {
      Status = client.read();
      if (Status == 1) {
        internalState = START_CHECKING;
        Serial.print("Received a message:");
        Serial.println(Status);
      } 
      else if (Status < 10) {
        internalState = IDLE;
        Serial.print("Received a message:");
        Serial.println(Status);
      }
    }
  }

  if (millis() - lastSend > interval) {
    // read sensor:
    int fototranzystorSensor = analogRead(fotoresistorPIN);
    int tcrt5000Sensor = analogRead(tcrt5000PIN);
    Serial.print("TRCT5000: ");
    Serial.println(tcrt5000Sensor);
    Serial.print("fototranzystorSensor: ");
    Serial.println(fototranzystorSensor);

    sendMSG("{\"state\": PARAM}", internalState);
    switch (internalState) {
      case IDLE: {
        Serial.println("IDLE state");
        break;
      }
      case START_CHECKING: {
        Serial.println("Start checking");
        inProcess = true;
        interval = 50;
        internalState = MOVE_FORWARD;
        break;
      }
      case MOVE_FORWARD: {
        Serial.println("Moving Forward");

        // ustaw predkosc
        // analogWrite(engine_speed, HIGH_SPEED);
        // jedz do przodu po szynach
        moveForward();
        delay(STEP_PERIOD);
        stopMoving();
        
        int tcrt5000BulbCheck = analogRead(tcrt5000PIN);
        if (tcrt5000BulbCheck > 500) { // sprawdz czy jesteś przy żarówce
          if (!lastBulbCheck) { // sprawdz czy nie jestesmy przy tej samej żarówce
            Serial.println("Bulb detected");
            currentBulbIndex++;  // zwiększ index sprawdzonej żarówki
            sendMSG("{\"progress\": PARAM}", currentBulbIndex);
            lastBulbCheck = true;
            int fototranzystorLightCheck = analogRead(fotoresistorPIN);
            if (fototranzystorLightCheck < 100) { // jeśli żarówka się nie świeci internalState = PULL_BULB;
              Serial.println("Broken bulb found!");
              sendMSG("{\"brokenBulbIndex\": PARAM}", currentBulbIndex);
              internalState = PULL_BULB;
              lastBulbCheck = false;
            }
            else if (currentBulbIndex == LAST_BULB_INDEX) { // jeśli żarówka jest ok i osiągnąłeś ostatnią pozycję
              internalState = GET_BACK_TO_BASE;
            }
          } else {Serial.println("Same bulb detected");}
        }
        else { // jeśli nie jesteśmy przy żarówce
          lastBulbCheck = false;
        }
        break;
      }
      case PULL_BULB: {
        Serial.println("Pull up the bulb");
        sendMSG("{\"pulledBulbIndex\": PARAM}", currentBulbIndex);

        pullBulb();

        // wróć do bazy
        internalState = GO_BACKWARD;
        goingToBaseBulbIndex = currentBulbIndex;
        break;
      }
      case GO_BACKWARD: {
        Serial.println("Go backward to change bulb");

        // ustaw predkosc
        // analogWrite(engine_speed, LOW_SPEED);
        // wróć po nową żarówkę
        moveBackward();
        delay(STEP_PERIOD);
        stopMoving();
        // jeśli wróciłeś do bazy

        int tcrt5000BulbCheck = analogRead(tcrt5000PIN);
        if (tcrt5000BulbCheck > 500) { // sprawdz czy jesteś przy żarówce
          if (!lastBulbCheck) { // sprawdz czy nie jestesmy przy tej samej żarówce
            Serial.println("Point detected");
            goingToBaseBulbIndex--;  // zmniejsz index minionej żarówki
            lastBulbCheck = true;
            if (goingToBaseBulbIndex == -2) {
              lastBulbCheck = false;
              internalState = EXCHANGE_BULB;
            }
          }
        }
        else {
          lastBulbCheck = false;
        }
        sendMSG("{\"progress\": PARAM}", goingToBaseBulbIndex);
        break;
      }
      case EXCHANGE_BULB: {
        // odłóż starą żarówkę
        Serial.println("Exchange the bulb");
        sendMSG("{\"progress\": PARAM}", -2);

        moveBackward();
        delay(STEP_PERIOD);
        stopMoving();
        // jeśli wróciłeś do bazy

        // int tcrt5000BulbCheck = analogRead(tcrt5000PIN);
        // if (tcrt5000BulbCheck > 500) { // sprawdz czy jesteś przy żarówce
        //   if (!lastBulbCheck) { // sprawdz czy nie jestesmy przy tej samej żarówce
        //     Serial.println("Point detected");
        //     goingToBaseBulbIndex--;  // zmniejsz index minionej żarówki
        //     lastBulbCheck = true;
        //     if (goingToBaseBulbIndex == -3) {
        //       lastBulbCheck = false;
        //       internalState = EXCHANGE_BULB;
        //     }
        //   }
        // }

        putBulb();

        internalState = GET_NEW_BULB;
        break;
      }
      case GET_NEW_BULB: {
        Serial.println("Get a new bulb");
        // ustaw predkosc
        // analogWrite(engine_speed, HIGH_SPEED);
        moveForward();
        delay(STEP_PERIOD);
        stopMoving();

        int tcrt5000BulbCheck = analogRead(tcrt5000PIN);
        if (tcrt5000BulbCheck > 500) { // sprawdz czy jesteś w bazie (pierwszy napotkany point po punkcie -2)
            Serial.println("Back in base");
            sendMSG("{\"progress\": PARAM}", -1);
            
            pullBulb();

            internalState = GET_BACK_TO_BROKEN_BULB;
        }
        break;
      }
      case GET_BACK_TO_BROKEN_BULB: {
        // wróć do lokalizacji z której żarówka została wyjęta
        Serial.println("Get back to broken bulb");
        // ustaw predkosc
        // analogWrite(engine_speed, LOW_SPEED);
        moveForward();
        delay(STEP_PERIOD);
        stopMoving();

        int tcrt5000BulbCheck = analogRead(tcrt5000PIN);
        if (tcrt5000BulbCheck > 500) { // sprawdz czy jesteś przy żarówce
          if (!lastBulbCheck) { // sprawdz czy nie jestesmy przy tej samej żarówce
            Serial.println("Bulb detected");
            backBulbIndex++;  // zwiększ index sprawdzonej żarówki
            sendMSG("{\"progress\": PARAM}", backBulbIndex);
            lastBulbCheck = true;
            if (backBulbIndex == currentBulbIndex) {
              lastBulbCheck = false;
              internalState = REPLACING_BULB;
            }
          }
        }
        else {
          lastBulbCheck = false;
        }
        break;
      }
      case REPLACING_BULB: {
        // wsadź nową żarówkę
        Serial.println("Replacing the bulb");
        sendMSG("{\"replacedBulbIndex\": PARAM}", currentBulbIndex);

        putBulb();

        if (currentBulbIndex == LAST_BULB_INDEX) { // jeśli to była ostatnia żarówka - wróć do bazy
          internalState = GET_BACK_TO_BASE;
          goingToBaseBulbIndex = LAST_BULB_INDEX;
        }
        else { // jeśli nie, idź do move forward
          internalState = MOVE_FORWARD;
        }
        break;
      }
      case GET_BACK_TO_BASE: {
        Serial.println("Get back to the base");

        // ustaw predkosc
        // analogWrite(engine_speed, HIGH_SPEED);
        // wróć do bazy
        moveBackward();
        delay(STEP_PERIOD);
        stopMoving();
        // jeśli wróciłeś do bazy

        int tcrt5000BulbCheck = analogRead(tcrt5000PIN);
        if (tcrt5000BulbCheck > 500) { // sprawdz czy jesteś przy żarówce
          if (!lastBulbCheck) { // sprawdz czy nie jestesmy przy tej samej żarówce
            Serial.println("Bulb detected");
            sendMSG("{\"progress\": PARAM}", goingToBaseBulbIndex);
            goingToBaseBulbIndex--;  // zmniejsz index minionej żarówki
            lastBulbCheck = true;
            if (goingToBaseBulbIndex == -1) {
              // jeśli w Bazie:
              lastBulbCheck = false;
              inProcess = false;
              internalState = IDLE;
              interval = 3000;
            }
          }
        }
        else {
          lastBulbCheck = false;
        }
        break;
      }

      default: {
        Serial.println("Default state");
        break;
      }
    }
    // update the timestamp:
    lastSend = millis();
  }

}