/* Este programa es capaz de enviar datos desde el nodeMCU a un servidor.js en 
    la computadora, sin embargo, el ciclo principal de trabajo se encuentra dentro
    de la función webSocketEvent que se activa cuando comienza la conexión con el 
    servidor, esto es peligroso para el proyecto porque la conexión puede romperse
    y esto evitaría el envío de datos por radio control, por lo anterior, se 
    creará una nueva versión del programa en donde la conexión con el servidor no 
    sea una tarea necesaria para el control. */


#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
//#include <WebSocketsServer.h>
#include <WebSocketsClient.h>
//#include <ArduinoWebsockets.h>
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
//WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);

WebSocketsClient webSocket;

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

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
	const uint8_t* src = (const uint8_t*) mem;
	Serial.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
	for(uint32_t i = 0; i < len; i++) {
		if(i % cols == 0) {
			Serial.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
		}
		Serial.printf("%02X ", *src);
		src++;
	}
	Serial.printf("\n");
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
  webSocket.sendTXT(output);         
}

float limite_inferior(float time0, int i){
  float limite = time0 + 100 + i*30  ; 
  return limite ; 
}

float limite_superior(float time0, int i){
  float limite = time0 + 100 + i*30 + 1 ; 
  return limite ; 
}

void Send_each_second(void){
while(1){
  float time0 = millis() ;
  int i = 1 ;
  while(true){
    float current_time = millis() ; 
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

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

	switch(type) {
		case WStype_DISCONNECTED:
			Serial.printf("[WSc] Disconnected!\n");
			break;
		case WStype_CONNECTED:
			Serial.printf("[WSc] Connected to url: %s\n", payload);
            wsconnected = true;
			// send message to server when Connected
			webSocket.sendTXT("Connected from ESP32");
      Send_each_second() ; 
			break;
		case WStype_TEXT:
			Serial.printf("[WSc] get text: %s\n", payload);

			// send message to server
			// webSocket.sendTXT("message here");
			break;
		case WStype_BIN:
			Serial.printf("[WSc] get binary length: %u\n", length);
			hexdump(payload, length);

			// send data to server
			// webSocket.sendBIN(payload, length);
			break;
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
	}

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

  webSocket.begin("192.168.1.67", 8080, "/");

  Serial.println("Websocket begin finihed"); 
  //webSocket.begin();
  webSocket.onEvent(webSocketEvent) ;

  webSocket.setReconnectInterval(5000);  
}

//SemaphoreHandle_t mtx;

//const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

void setup()
{
  Serial.begin(115200);

  //mtx = xSemaphoreCreateMutex();
  //xSemaphoreGive(mtx);
  taskWifi() ; 
}

void loop()
{
  webSocket.loop();
}