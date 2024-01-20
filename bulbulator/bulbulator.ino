
#include <ArduinoHttpClient.h>
// #include <WiFiNINA.h>  // use this for MKR1010 and Nano 33 IoT
#include <WiFiS3.h>    // use this for MKR1000
// your passwords go in arduino_secrets.h
#include "wifi_secrets.h"

#include <Servo.h>

/* DEFINES */
#define IDLE (0)
#define START_CHECKING (1)
#define MOVE_FORWARD (3)
#define PULL_BULB (4)
#define GO_BACKWARD (5)
#define EXCHANGE_BULB (6)
#define GET_BACK_TO_BROKEN_BULB (7)
#define REPLACING_BULB (8)
#define GET_BACK_TO_BASE (10)

#define LAST_BULB_INDEX (5)

/* WEBSOCKET */
// settings for a test on postman.com's websocket echo server
// set up a WiFi client ( using SSL):
WiFiSSLClient wifi;
char serverAddress[] = "bitehack-bulbulator-server.up.railway.app";
int port = 443;           // standard HTTPS port
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

/* ENGINE */
const int engine1_PIN1 = 2;
const int engine1_PIN2 = 3;

const int engine2_PIN1 = 8;
const int engine2_PIN2 = 9;

/* GENERAL */
// message sending interval, in ms:
int interval = 3000;
// last time a message was sent, in ms:
long lastSend = 0;
int Status;
int internal_state = IDLE;
int bulbIndex = 0;
bool lastBulbCheck = false;
int bulbsChecked[LAST_BULB_INDEX] = {0};

bool ignoreNextMsgFromServer = false;
bool in_process = false;

const int leftLimitSwitchPin = 12;
const int rightLimitSwitchPin = 13;



void moveForward() {
  digitalWrite(engine1_PIN1, HIGH);
  digitalWrite(engine1_PIN2, LOW);

  digitalWrite(engine2_PIN1, HIGH);
  digitalWrite(engine2_PIN2, LOW);
}

void moveBackward() {
  digitalWrite(engine1_PIN1, LOW);
  digitalWrite(engine1_PIN2, HIGH);

  digitalWrite(engine2_PIN1, LOW);
  digitalWrite(engine2_PIN2, HIGH);
}

void stopMoving() {
  digitalWrite(engine1_PIN1, LOW);
  digitalWrite(engine1_PIN2, LOW);

  digitalWrite(engine2_PIN1, LOW);
  digitalWrite(engine2_PIN2, LOW);
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
  pinMode(leftLimitSwitchPin, INPUT);
  pinMode(rightLimitSwitchPin, INPUT);

  digitalWrite(engine1_PIN1, LOW);
  digitalWrite(engine1_PIN2, LOW);

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
  int connect_status;
  connect_status = client.begin(endpoint);
  Serial.print("Status: ");
  Serial.println(connect_status);
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
    iAmRobotMSG();
    // skip the rest of the loop:
    return;
  }
  
  if (!in_process) {
    // check if a message is available to be received
    int messageSize = client.parseMessage();
    // if there's a string with length > 0:
    if (messageSize > 0) {
      if (ignoreNextMsgFromServer) {
          ignoreNextMsgFromServer = false;
        }
        else {
        Status = client.read();
        Serial.print("Received a message:");
        Serial.println(Status);
        if (Status == 1) {
          internal_state = START_CHECKING;
        } 
        else if (Status < 10) {
          internal_state = IDLE;
        }
      }
    }
  }

  if (millis() - lastSend > interval) {
    // read sensor:
    int fototranzystor_sensor = analogRead(A5);
    int tcrt5000_sensor = analogRead(A1);
    Serial.print("TRCT5000: ");
    Serial.println(tcrt5000_sensor);
    Serial.print("fototranzystor_sensor: ");
    Serial.println(fototranzystor_sensor);

    sendMSG("{\"state\": PARAM}", internal_state);
    switch (internal_state) {
      case IDLE: {
        Serial.println("IDLE state");
        break;
      }
      case START_CHECKING: {
        Serial.println("Start checking");
        in_process = true;
        interval = 1000;
        internal_state = MOVE_FORWARD;
        break;
      }
      case MOVE_FORWARD: {
        Serial.println("Moving Forward");
        // jedz do przodu po szynach
        moveForward();
        
        int tcrt5000_bulbCheck = analogRead(A1);
        if (tcrt5000_bulbCheck > 500) { // sprawdz czy jesteś przy żarówce
          if (!lastBulbCheck) { // sprawdz czy nie jestesmy przy tej samej żarówce
            Serial.println("Bulb detected");
            bulbIndex++;  // zwiększ index sprawdzonej żarówki
            sendMSG("{\"progress\": PARAM}", bulbIndex);
            lastBulbCheck = true;
            int fototranzystor_lightCheck = analogRead(A5);
            if (fototranzystor_lightCheck < 100) { // jeśli żarówka się nie świeci internal_state = PULL_BULB;
              Serial.println("Broken bulb found!");
              sendMSG("{\"brokenBulbIndex\": PARAM}", bulbIndex);
              internal_state = PULL_BULB;
              lastBulbCheck = false;
            }
            else if (bulbIndex == LAST_BULB_INDEX) { // jeśli żarówka jest ok i osiągnąłeś ostatnią pozycję
              internal_state = GET_BACK_TO_BASE;
            }
          }
        }
        else { // jeśli nie jesteśmy przy żarówce
          lastBulbCheck = false;
        }
        break;
      }
      case PULL_BULB: {
        Serial.println("Pull up the bulb");
        // chwyć żarówkę
        leftServoAngle = 0;
        rightServoAngle = 0;
        // while ()

        // wykręć żarówkę

        // podnieś żarówkę do góry
        // wróć do bazy
        internal_state = GO_BACKWARD;
        break;
      }
      case GO_BACKWARD: {
        Serial.println("Go backward to change bulb");
        // wróć po nową żarówkę
        moveBackward();
        // jeśli wróciłeś do bazy 
        internal_state = EXCHANGE_BULB;
        sendMSG("{\"progress\": PARAM}", -1);
        break;
      }
      case EXCHANGE_BULB: {
        Serial.println("Exchange the bulb");
        // odłóż starą żarówkę
        // weź nową
        internal_state = GET_BACK_TO_BROKEN_BULB;
        break;
      }
      case GET_BACK_TO_BROKEN_BULB: {
        moveForward();
        Serial.println("Get back to broken bulb");
        // wróć do lokalizacji z której żarówka została wyjęta
        internal_state = REPLACING_BULB;
        break;
      }
      case REPLACING_BULB: {
        Serial.println("Replacing the bulb");
        // wsadź nową żarówkę
        // jeśli to była ostatnia żarówka - wróć do bazy
        if (bulbIndex == LAST_BULB_INDEX) {
          internal_state = GET_BACK_TO_BASE;
        }
        else {
          internal_state = MOVE_FORWARD;
        }
        // jeśli nie, idź do move forward
        break;
      }
      case GET_BACK_TO_BASE: {
        // wróć do bazy
        // moveBackward();
        // jeśli w Bazie:
        in_process = false;
        internal_state = IDLE;
        interval = 3000;
        stopMoving();
        sendMSG("{\"progress\": PARAM}", -1);
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

  // int limitSwitchRead = digitalRead(limitSwitchPin);
  // if (limitSwitchRead == HIGH) {
  //   Serial.println("TOUCHED");
  // }
  // else {
  //   Serial.println("UNTOUCHED");
  // }
  // delay(2000);

  // for(leftServoAngle = 0; leftServoAngle < 180; leftServoAngle++) //przekręć serwo od 0 stopni do 180 stopni
  // {
  //   leftServo.write(leftServoAngle);
  //   delay(30);
  // }

  // for(leftServoAngle = 180; leftServoAngle > 0; leftServoAngle--) //teraz cofnij mikro serwo od 0 stopni do 180 stopni
  // {
  //   leftServo.write(leftServoAngle);
  //   delay(30);
  // }

}