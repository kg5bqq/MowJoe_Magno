/*
 * MowJoe Magnometer located as far as possible from steel chassis and communicates
 * to the MowJoe Master via ESPNow protocol.
 * This does not require a router bu in the event we want to run a server on MowJoe Master,
 * we need to find out which WiFi channel the router has assigned to the web client and hence
 * also to MowJoe Master.
 * If none is found (ie no router connection) we'll default both MowJoe Master and this ESPNow
 * unit to channel '1'.
 *
*/

#include "Mowjoe_Magno.h"

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
//#include <Adafruit_ADXL345_U.h>
#include <tgmath.h>
#include <RunningMedian.h>


#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/portmacro.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_now.h"
#include "esp_crc.h"


// MAC Address of MowJoe central processor
//uint8_t broadcastAddress[] = {0xC4, 0x4F, 0x33, 0x3E, 0xC5, 0xF1};  //Standalone ESP32 test master
uint8_t broadcastAddress[] = {0xC4,0x4F,0x33,0x3E,0xE8,0x25};  //Mowjoe_Master - Test2 - not yet final

//-- Sensor typedefs
#define ACCEL  1
#define MAG   2

#define EEADDR 98 //66 // Start location to write EEPROM data.
#define EEADDR 98 //66 // Start location to write EEPROM data.
#define CALTIME 10000  // In ms.
#define SMOOTH_ACCELL 20

//ESP32 Default I2C pins
#define I2C_SDA 21
#define I2C_SCL 22
#define MAGNO_RDY_PIN 19



enum Commands{START,STOP,REPEAT,FREQ,REQUIRE_ACK};

volatile int Begin_Sample_Start_Time = 0;

double Target_Heading = 352;
volatile double Input;

volatile double _heading = 0;
volatile double _position = 0;

volatile bool start_sending = false;

//*********************** Quazi-SWAG of Magnetic Bias for  HMC5883L magnetometer ******************
double xMax = 25.181818;
double yMax = -10.363636;
double xMin = 0.000000;
double yMin = -10.36363;
double zMax = 69;
double zMin = 0;

static int offx=0,offy=0,offz=0;

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
RunningMedian samples = RunningMedian(21);

TaskHandle_t compass_task_handle;

// Define variables to store BME280 readings to be sent
float temperature;
float humidity;
float pressure;

// Define variables to store incoming readings
char incomingCommand[ESPNOW_MAX_COMMAND_LEN] = {0};
float incomingTemp;
float incomingHum;
float incomingPres;

// Variable to store if sending data was successful
String success;

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
	char command[ESPNOW_MAX_COMMAND_LEN];
    float temp;
    float bearing;
    float heading;
} struct_message;

// Create a struct_message called outgoingReadings to hold sensor readings
struct_message outgoingReadings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;


void displaySensorDetails(int type) {
    sensor_t sensor;
#ifdef USE_ADXL345
    if(type == ACCEL) {
      accel.getSensor(&sensor);
    } else if (type== MAG) {
    mag.getSensor(&sensor);
    }
#else
    mag.getSensor(&sensor);
#endif
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
    Serial.println("------------------------------------");
    Serial.println("");
  }

float filter_heading(double x) {
  samples.add(x);
    float l = samples.getLowest();
    float m = samples.getMedian();
    float a = samples.getAverage(5);
    float h = samples.getHighest();
 return m; //Return the Lowest Rnning sample collection

}


char *command_arg[10][10] = {};
String user_command[20];

int parse_command(char* cmd_sent)
{

  int i = 0;
  int j = 0;
  char delimiters[] = "=:,";
//  char *token;
  int ret_val = 0;


  char* token;
  char* rest = cmd_sent;
  enum Commands command;

	  while ((token = strtok_r(rest, delimiters, &rest))) {
			 //printf("--->%s\n\r", token);
			 user_command[i++] = token;

	  }

	  for(j=0;j<i;j++)
	   Serial.printf("user_command[%d]%s\n\r",j, user_command[j]);


   if(user_command[0].indexOf("START") >= 0) {
	   start_sending = true;
	   if(user_command[1].indexOf("NOW") >= 0) {
		   Begin_Sample_Start_Time = user_command[2].toInt();
		   Serial.printf("%s %s %d\n\r", user_command[1], user_command[1], Begin_Sample_Start_Time);
				start_sending = true;
	   }
   }

   if(user_command[0].indexOf("STOP") >= 0) {
	   start_sending = false;
   }




////
////   ret_val = validate_command();
   return ret_val;

}








// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//  if (status ==0){
//    success = "Delivery Success :)";
//  }
//  else{
//    success = "Delivery Fail :(";
//  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  //Clear out any previous message/data
  //memset(incomingCommand,0,sizeof(incomingCommand));
  //Now copy in the latest..
  memcpy(incomingCommand,incomingReadings.command,sizeof(incomingCommand));

  parse_command(incomingReadings.command);

  incomingTemp = incomingReadings.temp;
  incomingHum = incomingReadings.bearing;
  incomingPres = incomingReadings.heading;
  Serial.printf("incomingReadings.heading = %lf\n\r", incomingReadings.heading);
}

//For some strange reason this next declaration needs to be located here!
esp_now_peer_info_t peerInfo;

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

      /* Initialize the Compass sensor (HMC5882 */
  if(!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check connections */
    Serial.println("No HMC5883 detected !");
    while(1);  //Don't proceed until fixed!
  }
  /* Display some basic information on this sensor */
  displaySensorDetails(MAG);



  xTaskCreatePinnedToCore(compass_task, "compass_task"  , 2048 //4096 //2048
    , NULL, 7         // Priority High to Low=0.
    , &compass_task_handle, ARDUINO_RUNNING_CORE);

}
 
void loop() {
}

void updateDisplay(){
  // Display Readings in Serial Monitor
//  Serial.printf("THIS UNIT[WIFI] channel %d, MAC: %s RSSI: %d, \n\r", WiFi.channel(),WiFi.macAddress().c_str(), WiFi.RSSI());
  Serial.println("INCOMING READINGS");
  Serial.print("Command: ");
  Serial.println(incomingCommand);
//  Serial.print("Heading: ");
//  Serial.println(incomingReadings.temp);
//  Serial.print("Heading: ");
//  Serial.println(incomingReadings.hum);
//  Serial.print("Heading: ");
//  Serial.println(incomingReadings.pres);
//  Serial.println();
}



void compass_task(void *arg) {
//  EventBits_t uxBits;
  sensors_event_t event;

  volatile bool ok_to_Send;
  Serial.printf("Compass Task Started\n\r");
  
  while(1) {

          mag.getEvent(&event);
          // Calculate heading when the magnetometer is level, then correct for signs of axis.
          //yMax and xMax were were pre-determined/calculated via the '360 rotation techneque'.
              double heading = atan2((event.magnetic.y - ((yMax + yMin) / 2.0)), (event.magnetic.x - ((xMax + xMin) / 2.0)));
          //TODO:
          //Add tile compensation here BEFORE applying the heading calculation (ref Honeywell "3-Axis Digital Compass IC
          // HMC5883L" datasheet.
        /*  To compensate a compass for tilt, knowing the
			roll and pitch is only half the battle. The magnetometer
			must now rely on all three magnetic axes (X, Y, Z) so
			that the earth's field can be fully rotated back to a
			horizontal orientation.

			The X, Y, and Z magnetic readings can be
			transformed back to the horizontal plane (XH, YH) by
			applying the rotational equations shown below:
			XH = X*cos(f) + Y*sin(q)*sin(f) - Z*cos(q)*sin(f)
			YH = Y*cos(q) + Z*sin(q) (4)
			Azimuth = arcTan (YH / XH)
			Once the X and Y magnetic readings are in the
			horizontal plane, equations (3) can be used to determine
			the azimuth. For speed in processing the rotational
			operations, a sine and cosine lookup table can be stored
			in program memory to minimized computation time.
      */
               //Add 'magnetic north' offset for current location.
              double declinationAngle = 0.05235988; //0.22;  //LAKE DALLAS - TX on 7/27/21 jab
              heading += declinationAngle;
              
              // Correct for when signs are reversed.
              if(heading < 0)
                heading += 2*PI;
                
              // Check for wrap due to addition of declination.
              if(heading > 2*PI)
                heading -= 2*PI;
               
              // Convert radians to degrees for readability.
              double headingDegrees = heading * 180/M_PI; 
              
              //Send it to the Filter.  _position will now contains the filtered collection of headings
              double temp_pos = filter_heading(headingDegrees);
              _heading = temp_pos;
              _position = nearbyint(temp_pos);  // Remove the decimal degrees (rounded out)
              Input = _position;//'Input comes out of the filter_heading() filter and goes to the PID

            Serial.printf("Heading: %lf\n\r", Input);

            // Send message via ESP-NOW
            outgoingReadings.temp = headingDegrees;
            outgoingReadings.bearing = temp_pos;
            outgoingReadings.heading = Input;


//            Serial.printf("outgoingReadings.pres: %lf\n\r", outgoingReadings.heading);

            if(ok_to_Send == true) {
            	esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
            	Serial.printf("SENT -> outgoingReadings.heading: %lf\n\r", outgoingReadings.heading);
            }

            ok_to_Send = start_sending;


         vTaskDelay(250 / portTICK_RATE_MS); //Normal/default max sample rate (continious mode of HMC5883L) is 66ms.
      } // End of while(1) loop

    //Should NEVER get here..
    vTaskDelete( compass_task_handle );

} // End of compass_task