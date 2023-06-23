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
uint8_t receiverMacAddress[] = { 0x34, 0x85, 0x18, 0x06, 0x6A, 0x2C };  //Receiver MAC-Adress: 34:85:18:06:6A:2C

struct PacketData {
  byte ForwardForce0;
  // byte UpForce1;
  // byte DownForce2;
  // byte LeftForce3;
  // byte RightForce4;
};
PacketData data;


// Pins for the force sens ors
//const int ForwardForceSensor1 = A0;
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
  data.ForwardForce0 = mapAndAdjustPressureSensorValues(analogRead(0), false);
  Serial.print("Pressure Sensor LoopValue:");
  Serial.println(analogRead(0));
  // data.UpForce1 = mapAndAdjustPressureSensorValues(analogRead(1), false);
  // Serial.println(analogRead(1));
  // data.DownForce2 = mapAndAdjustPressureSensorValues(analogRead(2), false);
  // Serial.println(analogRead(2));
  // ... and so on

  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&data, sizeof(data));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  delay(1000);
}