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
	
	uint64_t pipe = 0xF0F0DD00 ;
	byte radioData[PACKETSIZE]; // radio send/rec data
	int radiosuccess; // packet acked count
	int radiofail; // packets not acked count
	byte radioThis = EEPROM.read(10);
	
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

dow.setWaitForConversion(FALSE);
dow.requestTemperatures();

#endif
#ifdef CLI
	cli.addCommand("SET",cliSet); 
	cli.addCommand("R",cliRadio); 
	cli.setDefaultHandler(cliUnrecognized);
	println("CLI Active");
#endif

#ifdef RADIO

	radio.begin();
	radio.setRetries(15,15); // delay 3 = 2*250 uS + 150 base = 750 uS delay between retries and 5 retries max
	radio.setPayloadSize(PACKETSIZE); // packet 32 bytes (max is 32)
	radio.setPALevel(RF24_PA_MAX);
	radio.setCRCLength(RF24_CRC_16); // 2 byte crc
	radio.setDataRate(RF24_250KBPS);
	radio.setAutoAck(1); 
	radio.setChannel(76);
	radio.openReadingPipe(1,pipe+radioThis);
	
	radio.startListening();
	#ifdef CLI
		Serial.print("Radio ");
		Serial.print(radioThis);
		Serial.println(" Active ");
	#endif

#endif

}
void loop(){
#ifdef RADIO
	radioRead();
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


#ifdef CLI
void cliUnrecognized(const char *command){
	Serial.println("?");
	Serial.println(command);
}
void cliRadio()
{
	char *arg;
	radioData[0] = atoi(cli.next()); // to address
	radioData[1] = radioThis;
	for (byte i = 2; i < PACKETSIZE ;++i)
	{
		radioData[i] = atoi(cli.next()); //packet type
	}
	radioWrite(radioData[0]);

}
void cliSet()
{
	char *arg;
	Serial.print("SET ");
	arg = cli.next();
	if (arg != NULL)
	{
		if (strncmp(arg,"I",1) == 0 )
		{
			arg = cli.next();
			if (arg != NULL)
			{
				EEPROM.write(10,atoi(arg));
			}
	
			radio.stopListening();			
			radioThis = EEPROM.read(10);
			radio.openReadingPipe(1,pipe+radioThis);
			Serial.print("ID:");
			Serial.println(radioThis);
			radio.startListening();
		}	
	
		else
			{
				Serial.print("?:");
				Serial.println(arg);
			}
		}
	else
	{
		Serial.println("Address");
	}
}
#endif
#ifdef RADIO
void radioRead(){
	byte pipe_num;
	 // check if this is any radio data in the buffer	
	if (radio.available(&pipe_num))
	{
		radio.read( &radioData,PACKETSIZE );	// read data from buffer into the global array buf
		//process this command - allows for routing the packet later
		// dont think I care about pipe number right now
		// pipe 0 needs to be ignored
		if (pipe_num == 1){	
			radioProcess();
		}
	}
}
void radioWrite(byte destaddress){
	if (destaddress == radioThis){
		radioProcess();
		return;
	}
	radio.stopListening();

	radio.openWritingPipe(pipe+destaddress);
#ifdef DEBUG
	Serial.print(radioThis);
	Serial.print("sending packet to :");
	Serial.println(destaddress);
#endif
	delay(25);
				
	if (radio.write( &radioData, PACKETSIZE ) == 1){
		++radiosuccess;
		#ifdef DEBUG
		Serial.println("Sucess");
		#endif
	}
	else{
		++radiofail;
		#ifdef DEBUG
		Serial.println("Fail");
		#endif
	}
	radio.startListening();
}
void radioProcess()
{
	//  byte 0 of the packet is the destination address
		byte destaddress = radioData[0];
		#ifdef DEBUG
		// byte 1 is origination address
			Serial.print(radioData[1]);
			Serial.print(" sent a packet for:");
			Serial.print(destaddress);
			Serial.print(" type:");
			Serial.println(radioData[2]);
	
		#endif
		// byte 2 contains the packet type
		float onetemp;
		switch (radioData[2])
		{
		
		case 10: // onewire data recieved
			Serial.print("{\"ID\":");
			Serial.print(radioThis);	
			for (int i=0; i < 9 ; i++)
			{
				onetemp= ((float)(radioData[(i*2)+3]<<8)+radioData[(i*2)+4])/100;
				if (onetemp == 0) {break;}	
				Serial.print(",\"Temp_");
				Serial.print(radioThis);
				Serial.print('_');
				Serial.print(i);
				Serial.print("\":");
				Serial.print(onetemp);
			
					
			}
			Serial.println("}");
			
	
			break;
		case 50: //request 1wire data
			reportOnewire(radioData[1]); // report 1wire data to the requesting gatherer			
			break;
		case 100: //set led color (led,R,G,B)
			strip.setPixelColor(radioData[3],radioData[4],radioData[5],radioData[6]);
			strip.show();
			break;
		}
		//if (radioData[2] == 1) // packet type 1wire temp
				//{
					////report1wireF(true);
					//return;
				//}
//
}

#endif
void reportOnewire(byte sendAddress){
dow.requestTemperatures();
for(byte i=0; i < PACKETSIZE; ++i){radioData[i]=0;}

radioData[0] = sendAddress; //dest
radioData[1] = radioThis; //source
radioData[2] = 10; // packet type 1wire data

	for (byte i=0; i < dowCount; i++){
		int inttemp = ((double) dow.getTempF(dows[i])*100);
		unsigned int hbyte =(inttemp>>8);
		radioData[(i*2)+3] =(hbyte);
		radioData[(i*2)+4] = ((byte) (inttemp));
	}
	radioWrite(sendAddress);
}


#ifdef DEBUG
// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
	Serial.println("");
}
#endif
