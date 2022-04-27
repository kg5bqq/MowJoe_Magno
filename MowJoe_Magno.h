#ifndef MOWJOE_MAGNO_H
#define MOWJOE_MAGNO_H

// MAC Address of MowJoe central processor
//uint8_t broadcastAddress[] = {0x78, 0xE3, 0x6D, 0x10, 0x1F, 0x94};	//MowJoe_Magno - this sensor!
uint8_t broadcastAddress[] = {0x24,0x0a,0xc4,0xe8,0x2b,0x04};  //Mowjoe_Master - with external WiFi antenna
uint8_t broadcastAddress2[] = {0xC4, 0x4F, 0x33, 0x3E, 0xE8, 0x25};		//MowJoe_MotorControl Pink Highlight on back of ant.
uint8_t broadcastAddress3[] = {0xC4, 0x4F, 0x33, 0x3E, 0xC5, 0xF1};		//MowJoe_Gyro

#define MOWJOE_MAGNO	1
#define MOWJOE_GYRO		2

//-- Onboard Sensor types
#define ACCEL  1
#define MAG   2

#define EEADDR 98 //66 // Start location to write EEPROM data.
#define CALTIME 10000  // In ms.
#define SMOOTH_ACCELL 20

//ESP32 Default I2C pins
#define I2C_SDA 21
#define I2C_SCL 22
#define MAGNO_RDY_PIN 19

//Status LEDs
#define RED_LED_PIN		17
#define GREEN_LED_PIN	18
#define BLUE_LED_PIN	19

#define ESPNOW_MAX_COMMAND_LEN 40

//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
	uint8_t sensorID;
	char command[ESPNOW_MAX_COMMAND_LEN];
    float temp;
    float bearing;
    float heading;
} struct_message;

// Create a struct_message called outgoingReadings to hold sensor readings
struct_message outgoingReadings;

// Create a struct_message to hold incoming sensor readings
struct_message incomingReadings;


#endif		//MOWJOE_MAGNO_H
