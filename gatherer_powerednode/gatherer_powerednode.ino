// All the dependant Libraries
#include <Arduino.h>

//#define LEDS // Enable WS2812 LEDS 
#define CLI //enable command line interface 

#ifdef CLI
	#include <SerialCommand.h>
	SerialCommand cli; //The serial command object
#endif

#ifdef LEDS
	#include <Adafruit_NeoPixel.h>
	Adafruit_NeoPixel strip = Adafruit_NeoPixel(6,15, NEO_GRB + NEO_KHZ800);
#endif	

void setup() {
 Serial.begin(115200);

#ifdef LEDS
	strip.begin();
#endif	
#ifdef CLI
	cli.setDefaultHandler(cliUnrecognized);
#endif

}
void loop(){

#ifdef CLI
	cli.readSerial(); //check the serial buffer for commands
#endif
}	
void cliUnrecognized(const char *command){
	Serial.println("?");
	Serial.println(command);
}

