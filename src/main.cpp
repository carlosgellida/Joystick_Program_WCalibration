#include <Arduino.h>
#include <Wire.h>
#include <MPU_personalized_functions.h>
#include <Wifi_personalized_functions.h>

void getAndSendQuaternion(void)
{
  //Declare a string where data is saved
  String output;

  //Declare quaterion
  float Quaternion[4] ;

  //Sí la imu ha actualizado su valor obtener datos de orientación
  if (mpu.update()){
    Quaternion[0] = mpu.getQuaternionW() ; 
    Quaternion[1] = mpu.getQuaternionX() ; 
    Quaternion[2] = mpu.getQuaternionY() ; 
    Quaternion[3] = mpu.getQuaternionZ() ; 
  }

  //Resetar string output
  output = "";

  //Resetear documento JSON
  jsonDocTx.clear();

  //Almacenar info del quaternio en documento JSON
  jsonDocTx["QW"] = Quaternion[0];
  jsonDocTx["QX"] = Quaternion[1];
  jsonDocTx["QY"] = Quaternion[2];
  jsonDocTx["QZ"] = Quaternion[3]; 

  //Convertir documento JSON en un string
  serializeJson(jsonDocTx, output);

  //Enviar string con info del documento JSON
  webSocket.sendTXT(output);  
}



void SendInfoInRegularTimeIntervals(void){
// Esta función envía la información en intervalos regulares de tiempo (100hz)
float current_time = millis() ; 
float lastTime = 0 ; 
while(1){
    current_time = millis() ; 
    if((current_time - lastTime) >= 10){
      webSocket.loop();
      Serial.println(current_time - lastTime); 
      getAndSendQuaternion() ;
      lastTime = current_time; 
    }
  }
}

void taskWifi(void)
{
  // Esta función inicia la conexión WIFI, y se conecta a un 
  // servidor Web Socket
  while (status != WL_CONNECTED) {

    Serial.print("Attempting to connect to WPA SSID: ");

    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(3000);

  }

  Serial.print("You're connected to the network");

  printWifiData();

  Serial.println(WiFi.localIP());

  webSocket.begin("192.168.0.5", 8080, "/"); 

  Serial.println("Websocket has begin"); 

  webSocket.onEvent(webSocketEvent) ;

  webSocket.setReconnectInterval(50);  
}


void setup()
{
  // Inicializar conexión serial
  Serial.begin(115200); 

  // Inicializar I2C
  Wire.begin(); 

  // Inicializar WiFi y WebSocket
  taskWifi() ; 

  delay(2000);

  // Inizializar sensor de movimiento IMU
  inicializeMPU9250(mpu) ; 
}

void loop()
{
  
  //Serial.println(wsconnected) ; 

  SendInfoInRegularTimeIntervals() ;
}