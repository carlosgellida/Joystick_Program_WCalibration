#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <ArduinoJson.h>


/* // WiFi credentials
#define NUM_NETWORKS 1
// Add your networks credentials here
const char *ssidTab[NUM_NETWORKS] = {
    "INFINITUMF6A8E9",
};
const char *passwordTab[NUM_NETWORKS] = {
    "9189D8C5B8",
}; */

char ssid[] = "INFINITUMF6A8E9" ; 
char pass[] = "9189D8C5B8" ;
int status = WL_IDLE_STATUS;     // the Wifi radio's status

#define HTTP_PORT 8000
#define WEBSOCKET_PORT 8001

// you can provide credentials to multiple WiFi networks
WiFiMulti wifiMulti;

// HTTP server on port 8000
WebServer server(HTTP_PORT);

// WebSocket server
WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);

StaticJsonDocument<200> jsonDocTx;

bool wsconnected = false;

void printWifiData() {

  // print your WiFi shield's IP address:

  IPAddress ip = WiFi.localIP();

  Serial.print("IP Address: ");

  Serial.println(ip);

  Serial.println(ip);

  // print your MAC address:

  byte mac[6];

  WiFi.macAddress(mac);

  Serial.print("MAC address: ");

  Serial.print(mac[5], HEX);

  Serial.print(":");

  Serial.print(mac[4], HEX);

  Serial.print(":");

  Serial.print(mac[3], HEX);

  Serial.print(":");

  Serial.print(mac[2], HEX);

  Serial.print(":");

  Serial.print(mac[1], HEX);

  Serial.print(":");

  Serial.println(mac[0], HEX);

}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
  {
    wsconnected = false;
    Serial.printf("[%u] Disconnected\r\n", num);
  }
  break;
  case WStype_CONNECTED:
  {
    wsconnected = true;
    Serial.printf("\r\n[%u] Connection from local web site \r\n", num);
  }
  break;

  case WStype_TEXT:
  {
    Serial.printf("[%u] Text:\r\n", num);
    for (int i = 0; i < length; i++)
    {
      Serial.printf("%c", (char)(*(payload + i)));
    }
    Serial.println();
  }
  break;

  case WStype_BIN:
  case WStype_ERROR:
  case WStype_FRAGMENT_TEXT_START:
  case WStype_FRAGMENT_BIN_START:
  case WStype_FRAGMENT:
  case WStype_FRAGMENT_FIN:
  default:
    break;
  }
}

/* 
const char *html =
#include "html.h"
    ;

void onHttpReqFunc() // Aún no enviaremos documentos html
{
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", html);
} */

void taskWifi(void *parameter);
void taskStatus(void *parameter);

SemaphoreHandle_t mtx;

const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

void setup()
{
  Serial.begin(115200);

  mtx = xSemaphoreCreateMutex();
  xSemaphoreGive(mtx);

  taskWifi(NULL) ; 
  //taskStatus(NULL) ; 
}

void taskWifi(void *parameter)
{

  while (status != WL_CONNECTED) {

    Serial.print("Attempting to connect to WPA SSID: ");

    Serial.println(ssid);

    // Connect to WPA/WPA2 network:

    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:

    delay(10000);

  }

  // you're connected now, so print out the data:

  Serial.print("You're connected to the network");

  printWifiData();

  //Serial.printf("WiFi connected\r\n", (int)stat);
  //Serial.printf("IP address: ");
  Serial.println(WiFi.localIP());

  /* Start Husarnet */
  /* Husarnet.selfHostedSetup(dashboardURL);
  Husarnet.join(husarnetJoinCode, hostName);
  Husarnet.start(); */ 

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  /* Confgiure HTTP server */
  /* server.on("/", HTTP_GET, onHttpReqFunc);
  server.on("/index.html", HTTP_GET, onHttpReqFunc);
  server.begin(); */

  while (1)
  {
      if (xSemaphoreTake(mtx, 5) == pdTRUE)
      {
        webSocket.loop();
        server.handleClient();
        xSemaphoreGive(mtx);
        //Serial.println("I'm working") ; 
        taskStatus(NULL) ; 
      }
    delay(1000);
  }
}

void taskStatus(void *parameter)
{
  String output;

    
          /*float q0 = mpu.calcQuat(mpu.qw);
          float q1 = mpu.calcQuat(mpu.qx);
          float q2 = mpu.calcQuat(mpu.qy);
          float q3 = mpu.calcQuat(mpu.qz);*/

          float q0 = millis()/1000 ; 
          float q1 = 0 ; 
          float q2 = 0 ; 
          float q3 = 0 ; 

          output = "";

          // imu::Quaternion quat = bno.getQuat();

          /* Serial.printf("Qmpu=[%f,%f,%f,%f]\r\n", q0, q1, q2, q3);
          Serial.printf("---------------------\r\n"); */
          //rootTx["roll"] = mpu.roll;
          //rootTx["pitch"] = mpu.pitch;
          //rootTx["yaw"] = mpu.yaw;

          jsonDocTx.clear();
          jsonDocTx["q0"] = q0;
          jsonDocTx["q1"] = q1;
          jsonDocTx["q2"] = q2;
          jsonDocTx["q3"] = q3;
          serializeJson(jsonDocTx, output);

          //          Serial.print(F("Sending: "));
          //          Serial.println(output);

          if (wsconnected == true)
          {
            if (xSemaphoreTake(mtx, 5) == pdTRUE)
            {
              webSocket.sendTXT(0, output);
              xSemaphoreGive(mtx);
            }
          }
    
}