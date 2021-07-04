/* Aquí el NodeMCU se comporta cómo un servidor al cuál se conectan los clientes 
de la computadora, sin embargo, aún no logré enviar el mensaje a dos clientes, 
por lo anterior se creará una nueva versión en la que el microcontrolador se 
comporte como un cliente, y sea el servidor en la computadora el que se encargue
de proyectar las imagenes tridimensionales, ésto servirá además para la posterior
introducción de trayectorias desde la computadora.  */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <ArduinoJson.h>


/* // WiFi credentials */

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

void taskWifi(void);
void taskStatus(void);

//SemaphoreHandle_t mtx;

//const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

float limite_inferior(float time0, int i){
  float limite = time0 + 100 + i*30  ; 
  return limite ; 
}

float limite_superior(float time0, int i){
  float limite = time0 + 100 + i*30 + 1 ; 
  return limite ; 
}

void setup()
{
  Serial.begin(115200);

  //mtx = xSemaphoreCreateMutex();
  //xSemaphoreGive(mtx);

  taskWifi() ; 
}

void taskWifi(void)
{

  while (status != WL_CONNECTED) {

    Serial.print("Attempting to connect to WPA SSID: ");

    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);

  }

  Serial.print("You're connected to the network");

  printWifiData();

  Serial.println(WiFi.localIP());

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  while(1){
  webSocket.loop();
  float time0 = millis() ;
  int i = 1 ;
  while(wsconnected == true){
    float current_time = millis() ; 
    //Serial.println("I'm here") ; 
    if(current_time >= limite_inferior(time0, i) && current_time <= limite_superior(time0, i)){
      taskStatus() ; 
      Serial.println(current_time) ; 
      i++ ; 
    }
    if(current_time > (limite_superior(time0, i) + 50)){
      break ; 
    }
  }

  }  
}

void taskStatus(void)
{
  String output;

  float q0 = millis() ; 
  float q1 = 0 ; 
  float q2 = 0 ; 
  float q3 = 0 ; 

  output = "";

  jsonDocTx.clear();
  jsonDocTx["q0"] = q0;
  jsonDocTx["q1"] = q1;
  jsonDocTx["q2"] = q2;
  jsonDocTx["q3"] = q3;
  serializeJson(jsonDocTx, output);
  webSocket.sendTXT(0, output);         
}

void loop()
{
  Serial.printf("loop() running on core %d\r\n", xPortGetCoreID());
  while (1)
  {
    Serial.printf("[RAM: %d]\r\n", esp_get_free_heap_size());
    delay(1000);
  }
}