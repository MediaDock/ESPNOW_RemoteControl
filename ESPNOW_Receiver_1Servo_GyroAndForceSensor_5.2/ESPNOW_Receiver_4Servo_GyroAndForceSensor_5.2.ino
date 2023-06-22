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
                Based on: Adafruit QT Py ESP32-S2 
                The reason we chose this board, 
                was it`s small formfactor and its ability to 
                produce a usable PMW signal on many PINs.
                For a weird reason it wasn`t possible to make the 
                PWM work on the Seeedstudio XIAO ESP-C3. 
                Therefore we took the QT Py ESP32-S2...
                Based on the following examples: 
                https://dronebotworkshop.com/esp-now/ 
                https://github.com/un0038998/ESPNOW_Transmitter_Receiver
                Library Used: 
                https://github.com/khoih-prog/ESP32_S2_ISR_Servo
*/

/* ESP-NOW GLOBALS *********************************************************/

#include <esp_now.h>
#include <WiFi.h>

// This is signal timeout in milli seconds. We will reset the data if no signal
#define SIGNAL_TIMEOUT 1000  
unsigned long lastRecvTime = 0;

typedef struct PacketData 
{
  byte ForwardForce0;
  byte RightForce1;
  byte LeftForce2;
  byte UpDownForce3;
} PacketData;

PacketData receiverData;


/* SERVO GLOBALS *********************************************************/

#include "ESP32_S2_ISR_Servo.h"
#define TIMER_INTERRUPT_DEBUG       1
#define ISR_SERVO_DEBUG             1

// Select different ESP32 timer number (0-3) to avoid conflict
#define USE_ESP32_TIMER_NO          3


//See Adafruit QT Py ESP32-S2 : https://cdn-learn.adafruit.com/assets/assets/000/107/493/large1024/adafruit_products_Adafruit_QT_Py_ESP32-S2_Pinout.png?1640130293
#define PIN_D5            5         // TX = PIN_D5 
#define PIN_D6            6         // SCL = PIN_D6 
#define PIN_D7            7         // SDA = PIN_D7 
#define PIN_D8            8         // A3 = PIN_D8 
#define PIN_D9            9         // A2 = PIN_D9 
#define PIN_D17           17        // A1 = PIN_D17    
#define PIN_D18           18        // A0 = PIN_D18


// Published values for SG90 servos; adjust if needed
#define MIN_MICROS      800  //544
#define MAX_MICROS      2450

int servoA0  = -1;
int servoA1  = -1;
int servoA2  = -1;
int servoA3  = -1;

int valForwardForce0 = 0;
int valRightForce1 = 0;
int valLeftForce2 = 0;
int valUpDownForce3 = 0;

int maxSensorValue = valForwardForce0;  // Assume sensor1 is the largest initially
int maxSensorIndex = 1;        // Variable to store the sensor number
int UpDownNeutralIndex = 1;

int position = 40;
int backFinDeg = 80;



/* SETUP DEFAULT ***********************************************************/
void setInputDefaultValues()
{ 
  // The middle position for joystick. (254/2=127)
  receiverData.ForwardForce0 = valForwardForce0;
  //receiverData.UpForce2 = valUpForce1;
}



/* ESP-NOW CALLBACK *********************************************************/
/* callback function that will be executed when data is received ************/

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) 
{
  unsigned long now = millis();
  if ( now - lastRecvTime > SIGNAL_TIMEOUT ) 
    {
      if (len == 0)
        {
          return;
        }
      
      if (len > 0)
        {
        memcpy(&receiverData, incomingData, sizeof(receiverData));
        
        mapAndWriteValues();  
        lastRecvTime = millis(); 
        moveServosBasedOnValues();
        }
    }
}



/* RECEIVED DATA: MAP_AND_WRITE_VALUES *************************************/

void mapAndWriteValues()
{
  valForwardForce0 = receiverData.ForwardForce0;
  valRightForce1 = receiverData.RightForce1;
  valLeftForce2 = receiverData.LeftForce2;
  valUpDownForce3 = receiverData.UpDownForce3;

  maxSensorValue = valForwardForce0;  // Assume sensor1 is the largest initially
  maxSensorIndex = 1;        // Variable to store the sensor number

  //valForwardForce0 = map(receiverData.ForwardForce0, 0, 255, 0, 180);
  //Serial.print("valForwardForce0Mapped: ");
  Serial.print("valForwardForce0: ");
  Serial.println(valForwardForce0);
  Serial.print("valRightForce1: ");
  Serial.println(valRightForce1);
  Serial.print("valLeftForce2: ");
  Serial.println(valLeftForce2);
  Serial.print("valUpDownForce3: ");
  Serial.println(valUpDownForce3);
  delay(100);


// Is Up or Down bigger? Map and Index it!

  if (valUpDownForce3 >= 128) {
    valUpDownForce3 = map(valUpDownForce3,128,255,0,255);
    // Index Up 
    UpDownNeutralIndex = 2;
    Serial.print("Up-Down-Index:");
    Serial.println(UpDownNeutralIndex);    
  } 

  else if (valUpDownForce3 <= 126) {
    valUpDownForce3 = map(valUpDownForce3,0,126,0,255);
    // Index Down
    UpDownNeutralIndex = 0;
    Serial.print("Up-Down-Index:");
    Serial.println(UpDownNeutralIndex);
  }

  else {
    valUpDownForce3 = 127;
    // Index Neutral
    UpDownNeutralIndex = 1;
    Serial.print("Up-Down-Index:");
    Serial.println(UpDownNeutralIndex);    
  }

  if (valRightForce1 > maxSensorValue) {
    maxSensorValue = valRightForce1;
    maxSensorIndex = 2;
    Serial.print("maxSensorValue: ");
    Serial.println(maxSensorValue);
    Serial.print("maxSensorIndex: ");
    Serial.println(maxSensorIndex);
  }
  delay(100);

  if (valLeftForce2 > maxSensorValue) {
    maxSensorValue = valLeftForce2;
    maxSensorIndex = 3;
    Serial.print("maxSensorValue: ");
    Serial.println(maxSensorValue);
    Serial.print("maxSensorIndex: ");
    Serial.println(maxSensorIndex);
  }
  delay(100);

  if (valUpDownForce3 > maxSensorValue) {
    maxSensorValue = valUpDownForce3;
    maxSensorIndex = 4;
    Serial.print("maxSensorValue: ");
    Serial.println(maxSensorValue);
    Serial.print("maxSensorIndex: ");
    Serial.println(maxSensorIndex);
  }
  delay(100);

}



/* SETUP SERVOS *************************************************************/

void setUpServos()
{
	while (!Serial && millis() < 5000);
  setInputDefaultValues();
  delay(500);
	
  Serial.print(F("\nStarting ISR_MultiServos on "));
	Serial.println(ARDUINO_BOARD);
	Serial.println(ESP32_S2_ISR_SERVO_VERSION);

	ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);   //  Select ESP32 timer USE_ESP32_TIMER_NO
	servoA0 = ESP32_ISR_Servos.setupServo(PIN_D18, MIN_MICROS, MAX_MICROS);
  servoA1 = ESP32_ISR_Servos.setupServo(PIN_D17, MIN_MICROS, MAX_MICROS);
  servoA2 = ESP32_ISR_Servos.setupServo(PIN_D9, MIN_MICROS, MAX_MICROS);

/* UNCOMMENT / ADD AS FOLLOWS - IF MORE SERVOS ARE NEEDED */  

//	servoA3 = ESP32_ISR_Servos.setupServo(PIN_D8, MIN_MICROS, MAX_MICROS);


	if (servoA0 != -1)
		Serial.println(F("Setup Servo0 OK"));
	else
		Serial.println(F("Setup Servo0 failed"));

	if (servoA1 != -1)
		Serial.println(F("Setup Servo1 OK"));
	else
		Serial.println(F("Setup Servo1 failed"));

  if (servoA2 != -1)
		Serial.println(F("Setup Servo2 OK"));
	else
		Serial.println(F("Setup Servo2 failed"));  


}



/* MOVE THE SERVOS ************************************************************/

void moveServosBasedOnValues()
{

/* ServoA0 THROTTLE (FORWARD- FORCE)*******************************************/

  // Full Swing
  if ( ( servoA0 != -1) && ( valForwardForce0 > 70 ) )
	{
		for (position = 40; position <= backFinDeg; position += 5)
		{
			// goes from 40 degrees to 80 degrees
			// in steps of 1 degree

			if (position % 30 == 0)
			{
				Serial.print(F("Servo0 pos = "));
				Serial.print(position);
			}

			ESP32_ISR_Servos.setPosition(servoA0, position);
			// waits 30ms for the servo to reach the position
			delay(30);
		}

		delay(100);

		for (position = backFinDeg; position >= 0; position -= 5)
		{
			// goes from 80 degrees to 0 degrees
			if (position % 30 == 0)
			{
				Serial.print(F("Servo0 pos = "));
				Serial.print(position);
			}

			ESP32_ISR_Servos.setPosition(servoA0, position);
			// waits 30ms for the servo to reach the position
			delay(30);
		}
    
    delay(100);

    for (position = 0; position <= 40; position += 5)
		{
			// goes from 0 degrees to 40 degrees
			if (position % 30 == 0)
			{
				Serial.print(F("Servo0 pos = "));
				Serial.print(position);
			}

			ESP32_ISR_Servos.setPosition(servoA0, position);
			// waits 30ms for the servo to reach the position
			delay(30);
		}
  }

/***** UP DOWN NEUTRAL ****************************************************************************/

// Servo Index Down

  if ( ( servoA1 != -1) && ( servoA2 != -1) && ( maxSensorIndex == 4 ) && ( UpDownNeutralIndex == 0 ) ) {
    ESP32_ISR_Servos.setPosition(servoA1, 135);
    delay(50);
    ESP32_ISR_Servos.setPosition(servoA2, 55);
    delay(50);    
    Serial.println("Servo Down");
  }

// Servo Index Up

  else if ( ( servoA1 != -1) && ( servoA2 != -1) && ( maxSensorIndex == 4 ) && ( UpDownNeutralIndex == 2 ) ) {
    ESP32_ISR_Servos.setPosition(servoA1, 55);
    delay(50);     
    ESP32_ISR_Servos.setPosition(servoA2, 135);
    delay(50);  
    Serial.println("Servo Up");      
  }

// Servo Index Neutral
  else if ( ( servoA1 != -1) && ( servoA2 != -1) && ( maxSensorIndex == 4 ) && ( UpDownNeutralIndex == 1 ) ) {
    ESP32_ISR_Servos.setPosition(servoA1, 90);
    delay(50);
    ESP32_ISR_Servos.setPosition(servoA2, 90);
    delay(50);
    Serial.println("Servo Neutral");
  }


/***** UP DOWN NEUTRAL ****************************************************************************/

// Servo Index Right

  if ( ( servoA1 != -1) && ( servoA2 != -1) && ( maxSensorIndex == 2 ) ) {
    ESP32_ISR_Servos.setPosition(servoA1, 135);
    delay(50);
    ESP32_ISR_Servos.setPosition(servoA2, 90);
    delay(50);    
    Serial.println("Servo Down");
  }

// Servo Index Left

  else if ( ( servoA1 != -1) && ( servoA2 != -1) && ( maxSensorIndex == 3 ) ) {
    ESP32_ISR_Servos.setPosition(servoA1, 90);
    delay(50);     
    ESP32_ISR_Servos.setPosition(servoA2, 135);
    delay(50);  
    Serial.println("Servo Up");      
  }

}


/* SETUP FUNCTION *************************************************************/

void setup() 
{
  Serial.begin(115200);
  setUpServos();
// SETUP Wifi   
  WiFi.mode(WIFI_STA);

// Init ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
}
 


/* LOOP FUNCTION *************************************************************/

void loop()
{

}
