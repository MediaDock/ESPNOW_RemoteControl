/*
                #####
              ##########
             ###     ###
             ###     ###
             ###     ###
             ###     ###
             ###     ###
             ###     ##########
             ###     ##################
             ###     ###     ##############
             ###     ###     ####     #######
             ###     ###     ####     ###   ###
  ######     ###     ###     ####     ###   ######
###########  ###     ###     ####     ###       ###
####     #######                      ###       ###
#######      ###                                 ###
  #####                                          ###
  #######                                        ###
     ####                                        ###
      ######                                     ###
         ###                                  #####
         #######                             ####
            ####                             ###
             #######                     #######
                ####                     ####
                #############################
                #############################
                Based on: Seeedstudio XIAO ESP32-C3
                The reason we chose this board, 
                was it`s small formfactor and its external antenna for better signal transmission. 
                Based on the following examples: 
                https://dronebotworkshop.com/esp-now/
                https://github.com/un0038998/ESPNOW_Transmitter_Receiver
*/


/* ESP-NOW GLOBALS *********************************************************/
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
// QT Py ESP32-S2 Receiver MAC-Adress: D4:F9:8D:71:1E:9C
// QT Py ESP32-S2 Receiver MAC-Adress: D4:F9:8D:70:88:98 (Clone)
uint8_t receiverMacAddress[] = { 0xD4, 0xF9, 0x8D, 0x71, 0x1E, 0x9C };

struct PacketData {
  int ForwardForce0;
   int UpForce1;
   int DownForce2;
   int LeftForce3;
   // int RightForce4;
};
PacketData data;


// Pins for the force sens ors
// const int ForwardForceSensor1 = A0;
// const int UpForceSensor1 = A1;
// ... not sure if necessary to define this

// Force threshold
const int ForceThreshold = 500;


/* GET DATA FUNCTION ******************************************************/

int mapAndAdjustPressureSensorValues(int value, bool reverse) {

  // Check force on sensor 
  if (value > ForceThreshold) {
    Serial.print("Pressure Sensor Value:");
    Serial.println(value);
    return value;
  }
  delay(50);
}





/* ESP-NOW CALLBACK *********************************************************/
// callback when data is sent

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t ");
  Serial.println(status);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
}



/* SETUP FUNCTION *************************************************************/

void setup() {

  // Initialize the force sensors as input
  // pinMode(ForwardForceSensor1, INPUT); // ... not sure if necessary to define this

  delay(50);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  } else {
    Serial.println("Succes: Initialized ESP-NOW");
  }

  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  } else {
    Serial.println("Succes: Added peer");
  }
}


/* LOOP FUNCTION *************************************************************/

void loop() {
  data.ForwardForce0 = mapAndAdjustPressureSensorValues(analogRead(A0), false);
  Serial.print("Pressure Sensor0 LoopValue:");
  Serial.println(analogRead(A0));
  data.UpForce1 = mapAndAdjustPressureSensorValues(analogRead(A1), false);
  Serial.print("Pressure Sensor1 LoopValue:");
  Serial.println(analogRead(A1));
  data.DownForce2 = mapAndAdjustPressureSensorValues(analogRead(A2), false);
  Serial.print("Pressure Sensor2 LoopValue:");
  Serial.println(analogRead(A2));
  data.LeftForce3 = mapAndAdjustPressureSensorValues(analogRead(A3), false);
  Serial.print("Pressure Sensor3 LoopValue:");
  Serial.println(analogRead(A3));


  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&data, sizeof(data));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  delay(1000);
}