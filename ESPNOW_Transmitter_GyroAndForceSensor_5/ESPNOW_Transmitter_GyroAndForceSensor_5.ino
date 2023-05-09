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
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_MPU6050 mpu;

/* ESP-NOW GLOBALS *********************************************************/
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
// QT Py ESP32-S2 Receiver MAC-Adress: D4:F9:8D:71:1E:9C
// QT Py ESP32-S2 Receiver MAC-Adress: D4:F9:8D:70:88:98 (Clone)
uint8_t receiverMacAddress[] = { 0xD4, 0xF9, 0x8D, 0x71, 0x1E, 0x9C };

struct PacketData {
  int ForwardForce0;
   int RightForce1;
   int LeftForce2;
   int UpDownForce3;
   // int RightForce4;
};
PacketData data;


// Pins for the force sens ors
// const int ForwardForceSensor1 = A0;
// const int UpForceSensor1 = A1;
// ... not sure if necessary to define this

// Force threshold
const int ForceThreshold = 500;


/* GET Force Sensor Data FUNCTION ******************************************************/

int mapAndAdjustPressureSensorValues(int value, bool reverse) {

  // Check force on sensor 
  if (value > ForceThreshold) {
    Serial.print("Pressure Sensor Value:");
    Serial.println(value);
    return value;
  }

  delay(50);
}


/* GET Accelerometer Sensor Data FUNCTION ******************************************************/
int mapAndAdjustAccelerometerSensorValues(int value, bool reverse)
{
  if (value >= 4)
    {
      value = map(value, 4.0, 10.0, 127, 254);
    }
  else if (value <= -4)
    {
      value = (value == 0 ? 0 : map(value, -4.0, -10.0, 127, 0));  
    }
  else
    {
      value = 127;
    }

  if (reverse)
    {
      value = 254 - value;
    }
    Serial.println(value);  
    return value;
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

  // Initialize Accelerometer
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  delay(100);

}


/* LOOP FUNCTION *************************************************************/

void loop() {

  // Get new Accelerometer sensor events with the readings 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  data.ForwardForce0 = mapAndAdjustPressureSensorValues(analogRead(A0), false);
  Serial.print("Pressure Sensor0 LoopValue:");
  Serial.println(analogRead(A0));
  data.RightForce1 = mapAndAdjustPressureSensorValues(analogRead(A1), false);
  Serial.print("Pressure Sensor1 LoopValue:");
  Serial.println(analogRead(A1));
  data.LeftForce2 = mapAndAdjustPressureSensorValues(analogRead(A2), false);
  Serial.print("Pressure Sensor2 LoopValue:");
  Serial.println(analogRead(A2));
  data.UpDownForce3 = mapAndAdjustAccelerometerSensorValues(a.acceleration.y, false);
  Serial.print("Acceleration LoopValue:");
  Serial.println(a.acceleration.y);


  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&data, sizeof(data));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  delay(1000);
}