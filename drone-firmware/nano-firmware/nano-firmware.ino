#include "DHT.h" //DHT22 - AM2302
#include "NewPing.h"

//defines
#define DHTTYPE DHT22

//PINS
//dht
#define DHTPIN			6
#define LED_PIN		   13

#define SONAR_NUM		4

#define MAX_DISTANCE  300 // Maximum distance (in cm) to ping.
//sonar 1
#define TRIGGER_PIN_1   7
#define ECHO_PIN_1      8
//sonar 2
#define TRIGGER_PIN_2   9
#define ECHO_PIN_2     10
//sonar 3
#define TRIGGER_PIN_3  11
#define ECHO_PIN_3     12
//sonar 4
#define TRIGGER_PIN_4  4
#define ECHO_PIN_4     5

//LDR
#define LDR_READ_PIN   A3
 
NewPing sonar1(TRIGGER_PIN_1, ECHO_PIN_1, MAX_DISTANCE); // Each sensor's trigger pin, echo pin, and max distance to ping.
NewPing sonar2(TRIGGER_PIN_2, ECHO_PIN_2, MAX_DISTANCE);
NewPing sonar3(TRIGGER_PIN_3, ECHO_PIN_3, MAX_DISTANCE);
NewPing sonar4(TRIGGER_PIN_4, ECHO_PIN_4, MAX_DISTANCE);


DHT dht(DHTPIN, DHTTYPE); //DHT22

//readings
float dhtHumidity, dhtTemperature;
//sonar
unsigned long cmSonar1, cmSonar2, cmSonar3,cmSonar4;
//ldr
uint16_t ldr_reading = 0;

int time_var, last_time, lastMicros;

//led usage
bool blinkState = false;

//auxiliare - intertravamentos
uint8_t travaDHT = 10;

typedef struct {
	uint16_t sonar[4];
	int16_t humidityDHT;
	int16_t temperatureDHT;
	uint16_t ldrADC;
} nano_readings_struct;

int structSize;

void setup()
{
	structSize = sizeof(nano_readings_struct);
	
	// initialize devices
	dht.begin();

	pinMode(LED_PIN, OUTPUT);

	time_var = millis();
	last_time = time_var;
	
	// initialize serial communication
	Serial.begin(115200);
	
	digitalWrite(LED_PIN, LOW);

}

nano_readings_struct readingsToSend;
char requestFromGalileo = 0;

void loop()
{	
	
	if (Serial.available() > 0 ){
		requestFromGalileo = Serial.read();
	}
	
	if (requestFromGalileo == 'r') {
		
		digitalWrite(LED_PIN, HIGH);
	
		time_var = millis() - last_time;
		last_time = millis();

		travaDHT++;

		sonarReading(); 
		ldrReading();
		
		if (travaDHT >= 1) {
			dhtReading();
			travaDHT = 0;
		}
		
		// print
		printMessage();
		
		digitalWrite(LED_PIN, LOW);
		
		requestFromGalileo = 0;
	
	}

}

void dhtReading() {

	dhtHumidity = dht.readHumidity();
	dhtTemperature = dht.readTemperature();

}

void sonarReading() {


	cmSonar1 = sonar1.ping_cm();
	delay(2);
	cmSonar2 = sonar2.ping_cm();
	delay(2);
	cmSonar3 = sonar3.ping_cm();
	delay(2);
	//cmSonar4 = sonar4.ping_cm();
	cmSonar4 = 0;

}

void ldrReading() {
	ldr_reading = analogRead(LDR_READ_PIN);
}

void printMessage() {

	nano_readings_struct readingsToSend;
	
	readingsToSend.sonar[0] = static_cast<uint16_t>(cmSonar1);
	readingsToSend.sonar[1] = static_cast<uint16_t>(cmSonar2);
	readingsToSend.sonar[2] = static_cast<uint16_t>(cmSonar3);
	readingsToSend.sonar[3] = static_cast<uint16_t>(cmSonar4);
	
	readingsToSend.humidityDHT = static_cast<int16_t>(dhtHumidity * 100);
	readingsToSend.temperatureDHT = static_cast<int16_t>(dhtTemperature * 100);
		
	readingsToSend.ldrADC = static_cast<uint16_t>(ldr_reading);
	
	void *ptr = static_cast<void*>(&readingsToSend);
	
	char *buffer = static_cast<char*>(ptr);
	
	Serial.write(buffer, structSize);
	

	/*
	Serial.print("Humidity:\t");
	Serial.println(dhtHumidity);

	Serial.print("Temperature:\t");
	Serial.println(dhtTemperature);

	Serial.print("Sonares:\t");
	Serial.print(cmSonar1);
	Serial.print("\t");
	Serial.print(cmSonar2);
	Serial.print("\t");
	Serial.print(cmSonar3);
	Serial.print("\t");
	Serial.println();

	Serial.print("LDR:\t\t");
	Serial.println(ldr_reading);

	Serial.print("Time:\t\t");
	Serial.println(time_var);

	Serial.println();
	*/
}
