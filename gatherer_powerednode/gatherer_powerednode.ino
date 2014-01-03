// All the dependant Libraries

#define DEBUG
#define LEDS // Enable WS2812 LEDS 
#define CLI //enable command line interface 
#define RADIO //enables nRF24l01+ module
#define ONEWIRE
#include <Arduino.h>
#include <EEPROM.h>

#ifdef ONEWIRE
	#include <OneWire.h>
	#include <DallasTemperature.h>
	#define ONE_WIRE_BUS 2// 16 on nano jhack ortherwire 3 (2 on gatherer)
	int dowCount =0; // #devices found on 1wire bus
	// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
	OneWire oneWire(ONE_WIRE_BUS);
	// Pass our oneWire reference to Dallas Temperature.
	DallasTemperature dow(&oneWire);
	// arrays to hold device addresses
	DeviceAddress dows[9];
#endif
#ifdef CLI
	#include <SerialCommand.h>
	SerialCommand cli; //The serial command object
#endif

#ifdef LEDS
	#include <Adafruit_NeoPixel.h>
	Adafruit_NeoPixel strip = Adafruit_NeoPixel(6,15, NEO_GRB + NEO_KHZ800);
#endif

#ifdef RADIO
	#include <SPI.h>
	#include "nRF24L01.h"
	#include "RF24.h"
	#define PACKETSIZE 20
	RF24 radio(9,10); //set the radio pins
	// this is the base address for all the pipes
	// the last 2 bytes of the address is stores in eeprom bytes 10 - 16
	// and added at setup
	
	uint64_t pipes[6] = { 0xF0F0F0DD, 0xF0F0F0DD,0xF0F0F0DD,0xF0F0F0DD,0xF0F0F0DD,0xF0F0F0DD }; 
	byte snd[PACKETSIZE]; // send buffer
	byte rec[PACKETSIZE]; // rec buffer
	int rfsuccess[6]; // packet acked count
	int rffail[6]; // packets not acked count
 
#endif



void setup() {
 Serial.begin(115200);
#ifdef LEDS
	strip.begin();
	for(int i=0; i < 6; ++i){
		strip.setPixelColor(i,255,0,0);
		delay(20);
		strip.show();
	}

	for(int i=0; i < 6; ++i){
		strip.setPixelColor(i,0,255,0);
		delay(25);
		strip.show();
	}	

	for(int i=0; i < 6; ++i){
		strip.setPixelColor(i,0,0,255);
		delay(25);
		strip.show();
	}	

	for(int i=0; i < 6; ++i){
		strip.setPixelColor(i,255,255,255);
		delay(25);
		strip.show();
	}
	for(int i=0; i < 6; ++i){
		strip.setPixelColor(i,0,0,0);
		delay(25);
		strip.show();
	}


#endif	

#ifdef ONEWIRE
 dow.begin();
	
 
dowCount=dow.getDeviceCount();
Serial.print("1Wire Device count:");
Serial.println(dowCount,DEC);

if (dowCount > 9){ //limit to 10 1wire sensors so transmitt doesnt break - possible to increase to about 14
dowCount=9;}
	
 for (int i=0; i < dowCount; i++){
dow.getAddress(dows[i],i);
dow.setResolution(dows[i],12);
#ifdef DEBUG
	Serial.print("Found 1wire address:");
	printAddress(dows[i]);
#endif
}
dow.requestTemperatures();
dow.setWaitForConversion(FALSE);

#endif
#ifdef CLI
	cli.addCommand("SET",cliSet); 
	cli.setDefaultHandler(cliUnrecognized);
	println("CLI Active");
#endif

#ifdef RADIO
	for (int i = 0; i< 6; ++i)
	{
		pipes[i] = ((pipes[i] << 8)+EEPROM.read(10+i)) ;
	}
	radio.begin();
	radio.setRetries(2,5); // delay 3 = 2*250 uS + 150 base = 750 uS delay between retries and 5 retries max
	radio.setPayloadSize(PACKETSIZE); // packet 32 bytes (max is 32)
	radio.setPALevel(RF24_PA_MAX);
	radio.setCRCLength(RF24_CRC_16); // 2 byte crc
	radio.setDataRate(RF24_250KBPS);
	radio.setAutoAck(1); 
	radio.setChannel(76);
	for (int i=1; i < 6; ++i)
	{
		radio.openReadingPipe(i,pipes[i]);  
	}
	radio.startListening();
	#ifdef CLI
		Serial.println("Radio Active");
	#endif

#endif

}
void loop(){
#ifdef RADIO
	readRadio();
#endif
#ifdef CLI
	cli.readSerial(); //check the serial buffer for commands
#endif

}	
void println(const char* data){
#ifdef CLI
	Serial.println(data);
#endif

}

void cliUnrecognized(const char *command){
	Serial.println("?");
	Serial.println(command);
}
#ifdef CLI
void cliSet() {
	char *arg;
	Serial.print("SET ");
	arg = cli.next();
	if (arg != NULL) {
		if (strncmp(arg,"R",1) == 0 )
		{
			Serial.println("Radio ");
				for (int q = 0; q < 6; ++q)
				{

					arg = cli.next();

					if (arg != NULL)
					{
						EEPROM.write(q+10,atoi(arg));
					}
						Serial.print("Pipe:");
						Serial.print(q);
						Serial.print(" Set to ");
						Serial.println(EEPROM.read(q+10));
					
				}
	

		}
			else
			{
				Serial.print("?:");
				Serial.println(arg);
			}
		}

	else
		{
					Serial.println("IRTemperature, 1wire, Display, Radio, Flip");
		}
}
#endif
#ifdef RADIO
void readRadio(){

uint8_t pipe_num;
if ( radio.available(&pipe_num) )
	//
{
	//Serial.println("incoming radio");
	// Dump the payloads until we've gotten everything
//	bool done = false;
//	while (!done)
//	{
		// Fetch the payload, and see if this was the last one.
	//	done = radio.read( &rec,PACKETSIZE );

//	}
	
	radio.read( &rec,PACKETSIZE );	
	//  byte 0 of the packet is the destination address
	
	byte destaddress = rec[0];
	if ((pipe_num == 5) && (destaddress != EEPROM.read(11))) // dont route stuff comming in on pipe 5 unless its addressed to us (address in 11 eeprom)
	{
		return;
	}
	#ifdef DEBUG
	// byte 1 is origination address
	Serial.print(rec[1]);
	Serial.print(" sent a packet for:");
	Serial.print(destaddress);
	Serial.print(" type:");
	Serial.println(rec[2]);
	

	#endif
		
		if ((destaddress == EEPROM.read(11)) || ((EEPROM.read(10) == 255) && (destaddress == 255))) // check pipe 1 and not end unit
		//process the packet instead of fowarding it
		// this happens when a message is comming in on pipe 1(for this unit) or on any pipe if this units parent is 255 (i think)
		{
			//Serial.print("Packet for THIS unit ");
			//Serial.println(destaddress);
			
			// byte 2 is the packet type 
			
			if (rec[2] == 1) // packet type 1wire temp
			{
			
			//report1wireF(true);
			return;
			}
					if (rec[2] == 2) // packet type 1ir temp
					{
						//reportIrF(true);
						return;
					}
		} 
		else
		{
			for (int i=1; i < 6; ++i)
			{
				if (destaddress <= EEPROM.read(10+i)) // check pipe 1 and not end unit
				{
				
					radio.stopListening();
					 radio.openWritingPipe(pipes[i]);
		if (radio.write( &rec, PACKETSIZE ) == 1){++rfsuccess[i];}else{++rffail[i];}

				
					radio.startListening();
		
					Serial.print("Forwarded packet to route ");
							Serial.println(EEPROM.read(10+i));
					
					return;
					}				
			}
		
					radio.stopListening();
					 radio.openWritingPipe(pipes[0]);
		if (radio.write( &rec, PACKETSIZE ) == 1){++rfsuccess[0];}else{++rffail[0];}

					


					radio.startListening();
					Serial.print("Forwarded UP packet to route ");
					Serial.println(EEPROM.read(10));
					return;
		
			
		}
		
		
		byte packettype = rec[2];
		float temperature;
		
		for (int i = 2; i < 20; i=i+2)
		{
			temperature= ((float)(rec[i]<<8)+rec[i+1])/100;
			if (temperature == 0) {break;}
			Serial.println(temperature);
		}

	Serial.print("Unprocessed packet");

}
}
#endif
#ifdef DEBUG
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
#endif
