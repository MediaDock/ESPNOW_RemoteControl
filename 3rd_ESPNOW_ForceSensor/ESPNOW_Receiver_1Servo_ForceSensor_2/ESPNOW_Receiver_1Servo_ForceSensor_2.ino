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
  int ForwardForce0;
  // int UpForce1;
  // int DownForce2;
  // int LeftForce3;
  // int RightForce4;
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
int valUpForce1 = 90;
int valDownForce2 = 0;
int valLeftForce3 = 0;
int valRightForce4 = 0;

int position = 0;

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
  //valForwardForce0 = map(receiverData.ForwardForce0, 0, 255, 0, 180);
  //Serial.print("valForwardForce0Mapped: ");
  Serial.println(valForwardForce0);
  delay(100);

/* UNCOMMENT / ADD AS FOLLOWS - IF MORE SERVOS ARE NEEDED */  
// valUpForce1 = map(receiverData.lyAxisValue, 0, 254, 0, 180);
// Serial.print("PositionMapped: ");
// Serial.println(map(receiverData.lxAxisValue, 0, 254, 0, 180));
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

/* UNCOMMENT / ADD AS FOLLOWS - IF MORE SERVOS ARE NEEDED */  
//	servoA1 = ESP32_ISR_Servos.setupServo(PIN_D17, MIN_MICROS, MAX_MICROS);
//	servoA2 = ESP32_ISR_Servos.setupServo(PIN_D9, MIN_MICROS, MAX_MICROS);
//	servoA3 = ESP32_ISR_Servos.setupServo(PIN_D8, MIN_MICROS, MAX_MICROS);


	if (servoA0 != -1)
		Serial.println(F("Setup Servo0 OK"));
	else
		Serial.println(F("Setup Servo0 failed"));

/* UNCOMMENT / ADD AS FOLLOWS - IF MORE SERVOS ARE NEEDED */  
/*
	if (servoA1 != -1)
		Serial.println(F("Setup Servo2 OK"));
	else
		Serial.println(F("Setup Servo2 failed"));

  (...)  
*/
}



/* MOVE THE SERVOS ************************************************************/

void moveServosBasedOnValues()
{

/* ServoA0 THROTTLE (FORWARD- FORCE)*******************************************/

  // Full Swing
  if ( ( servoA0 != -1) && ( valForwardForce0 > 75 ) )
	{
		for (position = 0; position <= 90; position++)
		{
			// goes from 0 degrees to 180 degrees
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

		for (position = 90; position >= 0; position--)
		{
			// goes from 180 degrees to 0 degrees
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

/* ServoA1 TILT-UP (UP-FORCE) *************************************************/

/*
  // Full Swing
  if ( ( servoA1 != -1) && ( valUpForce1 > 600 ) )
	{
		for (position1 = 0; position1 <= 180; position++)
		{
			// goes from 0 degrees to 180 degrees
			// in steps of 1 degree

			if (position % 30 == 0)
			{
				Serial.print(F("Servo1 pos = "));
				Serial.print(position);
			}

			ESP32_ISR_Servos.setPosition(servoA0, position);
			// waits 30ms for the servo to reach the position
			delay(30);
		}

		delay(100);

		for (position = 180; position >= 0; position--)
		{
			// goes from 180 degrees to 0 degrees
			if (position % 30 == 0)
			{
				Serial.print(F("Servo1 pos = "));
				Serial.print(position);
				Serial.print(F(", Servo2 pos = "));
				Serial.println(180 - position);
			}

			ESP32_ISR_Servos.setPosition(servoA0, position);
			// waits 30ms for the servo to reach the position
			delay(30);
		}
  }


/* UNCOMMENT / ADD AS FOLLOWS - IF MORE SERVOS ARE NEEDED */    
//  ESP32_ISR_Servos.setPosition(servoA1, valUpForce1);
//  ESP32_ISR_Servos.setPosition(servoA1, valUpForce1);
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
