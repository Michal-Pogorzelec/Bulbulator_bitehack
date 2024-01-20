
#include <ArduinoHttpClient.h>
// #include <WiFiNINA.h>  // use this for MKR1010 and Nano 33 IoT
#include <WiFiS3.h>    // use this for MKR1000
// your passwords go in arduino_secrets.h
#include "wifi_secrets.h"

#include <Servo.h>

/* DEFINES */
#define IDLE (0)
#define START_CHECKING (1)
#define IN_PROGRESS (2)
#define MOVE_FORWARD (3)
#define PULL_BULB (4)
#define GO_BACKWARD (5)
#define EXCHANGE_BULB (6)
#define GET_BACK_TO_BROKEN_BULB (7)
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
int leftServoPin = 4; // pin sterujący Arduino
Servo leftServo;
int leftServoAngle = 0; // pozycja startowa serwa w stopniach

int rightServoPin = 7; // pin sterujący Arduino
Servo rightServo;
int rightServoAngle = 0; // pozycja startowa serwa w stopniach

/* GENERAL */
// message sending interval, in ms:
int interval = 3000;
// last time a message was sent, in ms:
long lastSend = 0;
int Status;
int bulbIndex = 0;
bool lastBulbCheck = false;
int bulbsChecked[LAST_BULB_INDEX] = {0};

void setup() {
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
  if (client.connected()) {
    client.beginMessage(TYPE_TEXT);
    client.print("{\"type\": \"i-am-robot\"}");
    client.endMessage();
  }

  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);
}

void loop() {
  bool in_process = false;
  // if not connected to the socket server, try to connect:
  if (!client.connected()) {
    client.begin();
    delay(1000);
    Serial.println("Attempting to connect to server");
    // skip the rest of the loop:
    return;
  }

  if (millis() - lastSend > interval) {
    // read sensor:
    int fototranzystor_sensor = analogRead(A5);
    int tcrt5000_sensor = analogRead(A1);
    Serial.print("TRCT5000: ");
    Serial.println(tcrt5000_sensor);
    // format the message as JSON string:
    String message = "{\"sensor\": READING}";
    // replace READING with the reading:
    message.replace("READING", String(fototranzystor_sensor));
    // send the message:
    client.beginMessage(TYPE_TEXT);
    client.print(message);
    client.endMessage();
    Serial.print("sending: ");
    Serial.println(message);

    if (!in_process) {
      // check if a message is available to be received
      int messageSize = client.parseMessage();
      // if there's a string with length > 0:
      if (messageSize > 0) {
        Status = client.read();
        Serial.print("Received a message:");
        Serial.println(Status);
      }
    }

  switch (Status) {
    case IDLE: {
      Serial.println("IDLE state");
      break;
    }
    case START_CHECKING: {
      Serial.println("Start checking");
      in_process = true;
      Status = MOVE_FORWARD;
      interval = 100;
      break;
    }
    case MOVE_FORWARD: {
      // jedz do przodu po szynach
      // engine.move()
      
      int tcrt5000_bulbCheck = analogRead(A1);
      if (tcrt5000_bulbCheck > 500) { // sprawdz czy jesteś przy żarówce
        if (!lastBulbCheck) { // sprawdz czy nie jestesmy przy tej samej żarówce
          bulbIndex++;  // zwiększ index sprawdzonej żarówki
          lastBulbCheck = true;
          int fototranzystor_lightCheck = analogRead(A5);
          if (fototranzystor_lightCheck < 100) { // jeśli żarówka się nie świeci Status = PULL_BULB;
            Status = PULL_BULB;
            lastBulbCheck = false;
          }
          else if (bulbIndex == LAST_BULB_INDEX) { // jeśli żarówka jest ok i osiągnąłeś ostatnią pozycję
            Status = GET_BACK_TO_BASE;
          }
        }
      }
      else { // jeśli nie jesteśmy przy żarówce
        lastBulbCheck = false;
      }

      break;
    }
    case PULL_BULB: {
      // chwyć żarówkę
      // servo.write(x);
      // wykręć żarówkę

      // podnieś żarówkę do góry
      Status = GO_BACKWARD;
      break;
    }
    case GO_BACKWARD: {
      // wroc po nową żarówkę
      // engine.moveBack();
      // jeśli wróciłeś do bazy Status = EXCHANGE_BULB;
      break;
    }
    case EXCHANGE_BULB: {
      // odłóż starą żarówkę
      // weź nową
      // Status = GET_BACK_TO_BROKEN_BULB;
      break;
    }
    case GET_BACK_TO_BROKEN_BULB: {
      // wróć do lokalizacji z której żarówka została wyjęta
      // wsadź nową żarówkę
      // jeśli to była ostatnia żarówka - wróć do bazy
      // jeśli nie, idź do move forward
      break;
    }
    case GET_BACK_TO_BASE: {
      // wróć do bazy
      // jeśli w Bazie: 
      /*
      in_process = false;
      Status = IDLE;
      interval = 3000;
      */
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