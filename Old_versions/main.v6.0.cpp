/* Esta versión es completamente funcional, y permite el envío de los ángulos 
Euler medidos en la tarjeta via WiFI, aún será refactorizada para disminuir la 
complejidad del código y documentada mejor. */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <WebSocketsClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <MPU_personalized_functions.h>
//#include "ArduinoOSC.h"

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
bool mensaje_recibido = true; 

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

/* void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
} */

float get_roll_pitch_yaw() {
    float roll_pitch_yaw[3] ; 
    roll_pitch_yaw[0] = mpu.getRoll() ; 
    roll_pitch_yaw[1] = mpu.getPitch() ; 
    roll_pitch_yaw[2] = mpu.getYaw() ; 
}

float get_Quaternion() {
    float Quaternion[4] ; 
    Quaternion[0] = mpu.getQuaternionW() ; 
    Quaternion[1] = mpu.getQuaternionX() ; 
    Quaternion[2] = mpu.getQuaternionY() ; 
    Quaternion[3] = mpu.getQuaternionZ() ; 
}



void taskStatus(void)
{
  String output;
  float roll ; 
  float pitch ; 
  float yaw ; 
  float Quaternion[4] ; 

  if (mpu.update()){
    Quaternion[0] = mpu.getQuaternionW() ; 
    Quaternion[1] = mpu.getQuaternionX() ; 
    Quaternion[2] = mpu.getQuaternionY() ; 
    Quaternion[3] = mpu.getQuaternionZ() ; 
  }

  output = "";

  jsonDocTx.clear();
  jsonDocTx["QW"] = Quaternion[0];
  jsonDocTx["QX"] = Quaternion[1];
  jsonDocTx["QY"] = Quaternion[2];
  jsonDocTx["QZ"] = Quaternion[3]; 
  //jsonDocTx["q3"] = q3;
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
  webSocket.loop();
  float time0 = millis() ;
  float pitch_roll_yaw ; 
  int i = 1 ;
  while(wsconnected == true){
    webSocket.loop(); 
    float current_time = millis() ; 
    if(current_time >= limite_inferior(time0, i) && current_time <= limite_superior(time0, i)){
      taskStatus() ; 
      //Serial.println(current_time) ;
      i++ ; 
    }
    if(current_time > (limite_superior(time0, i) + 50)){
      break ; 
    } 
    //delay(10) ; 
  }
  delay(50); 
  }
}

void sendAndRecieve(void){
  String output;
  while(true){
    webSocket.loop();  
    if(mensaje_recibido == true){
      output = "";
      jsonDocTx.clear();
      jsonDocTx["millis = "] = float(millis());
      serializeJson(jsonDocTx, output);
      webSocket.sendTXT(output);  
      mensaje_recibido = false; 
    } 
    delay(50) ; 
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

	switch(type) {
		case WStype_DISCONNECTED:
			Serial.printf("[WSc] Disconnected!\n");
      wsconnected = false;
			break;
		case WStype_CONNECTED:
			Serial.printf("[WSc] Connected to url: %s\n", payload);
      wsconnected = true;
			// send message to server when Connected
			webSocket.sendTXT("Connected from ESP32"); 
      mensaje_recibido = false;
			break;
		case WStype_TEXT:
			Serial.printf("[WSc] get text: %s\n", payload);
      mensaje_recibido = true; 
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

  webSocket.setReconnectInterval(50);  
}

//SemaphoreHandle_t mtx;

//const signed char orientationDefault[9] = {0, 1, 0, 0, 0, 1, 1, 0, 0};

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  //mtx = xSemaphoreCreateMutex();
  //xSemaphoreGive(mtx);
  taskWifi() ; 

  delay(2000);

  //Serial.println(F("BMP280 test"));
  
  //inicializeBMP280(bmp) ;
  inicializeMPU9250(mpu) ; 

  // load from eeprom
  loadCalibration();

  print_calibration();
  mpu.verbose(false);
}

void loop()
{
  Serial.println(wsconnected) ; 
  Send_each_second() ;
  //sendAndRecieve() ; 
}