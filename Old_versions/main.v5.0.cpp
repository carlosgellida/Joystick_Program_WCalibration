

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <Wire.h>
#include <WebSocketsClient.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include "MPU9250.h"
//#include "ArduinoOSC.h"

MPU9250 mpu;

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

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
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
  webSocket.loop();
  float time0 = millis() ;
  int i = 1 ;
  while(wsconnected == true){
    webSocket.loop(); 
    float current_time = millis() ; 
    if(current_time >= limite_inferior(time0, i) && current_time <= limite_superior(time0, i)){
      taskStatus() ; 
      //Serial.println(current_time) ;
      print_roll_pitch_yaw();
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

  //mtx = xSemaphoreCreateMutex();
  //xSemaphoreGive(mtx);
  taskWifi() ; 

  delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // calibrate anytime you want to
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();

    print_calibration();
    mpu.verbose(false);
}

void loop()
{
  Serial.println(wsconnected) ; 
  Send_each_second() ;
  //sendAndRecieve() ; 
}