
// All the dependant Libraries
#define HUMIDITY
#define POWER
#define LCD 
#define CONTACT
#define DEBUG
#define LEDS // Enable WS2812 LEDS 
#define CLI //enable command line interface 
#define RADIO //enables nRF24l01+ module
#define ONEWIRE
#include <Arduino.h>
#include <EEPROM.h>
#ifdef HUMIDITY
#include "DHT.h"
DHT dht;
#endif
#ifdef POWER
#include "EmonLib.h"                   // Include Emon Library
EnergyMonitor emon1;                   // Create an instance

#endif

#ifdef CONTACT 
	#include <PinChangeInt.h>
	#define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts (D8-D13)
	#define NO_PORTD_PINCHANGES // to indicate that port d will not be used for pin change interrupts (D0-D7)
	#define DISABLE_PCINT_MULTI_SERVICE //to limit the handler to servicing a single interrupt per invocation.
	boolean contactChange; 

#endif
#ifdef LCD
	#include <Adafruit_ST7735.h>
	#include <Adafruit_GFX.h>
	#define sclk 4
	#define mosi 5
	#define cs   8
	#define dc   7
	#define rst  -1  // you can also connect this to the Arduino reset
	#define	ST7735_BLACK   0x0000
	#define	ST7735_BLUE    0x001F
	#define	ST7735_RED     0xF800         
	#define	ST7735_GREEN   0x07E0
	#define ST7735_CYAN    0x07FFb
	#define ST7735_MAGENTA 0xF81F
	#define ST7735_YELLOW  0xFFE0
	#define ST7735_WHITE   0xFFFF
	Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, mosi, sclk, rst);

	#endif
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
	byte radioParent = 3;
#endif



void setup() {
 Serial.begin(115200);
 pinMode(3,OUTPUT); // two transistor outputs
 pinMode(6,OUTPUT);
 pinMode(A7,INPUT);
 digitalWrite(6,LOW); //
 digitalWrite(3,LOW); //lcd bacllight
#ifdef HUMIDITY


  dht.setup(A2); // data pin 2

#endif

#ifdef POWER
	emon1.current(3, 111.1);             // Current: input pin, calibration.
#endif
#ifdef LCD
	tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab  
	tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
//if (bitRead(EEPROM.read(0),4)){tft.setRotation(2);}//flipdisplay

//tft.fillRect(0,0, 128, 20, ST7735_BLACK);
 digitalWrite(3,HIGH); //lcd bacllight
 //tft.fillScreen(ST7735_BLACK);

#endif

#ifdef CONTACT
	pinMode(A0,INPUT_PULLUP);  // set a0  as contact closeure 
	PCintPort::attachInterrupt(A0, &contactInterupt, CHANGE);

#endif
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
#ifdef CLI
	Serial.print("1Wire Device count:");
	Serial.println(dowCount,DEC);
#endif

if (dowCount > 9){ //limit to 10 1wire sensors so transmitt doesnt break - possible to increase to about 14
dowCount=9;}
	
 for (int i=0; i < dowCount; i++){
dow.getAddress(dows[i],i);
dow.setResolution(dows[i],12);


	//Serial.print("Found 1wire address:");
	//printAddress(dows[i]);

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
#ifdef CONTACT
if (contactChange){
	startRadioPacket(radioParent,15); // packet type contact change data
	if (digitalRead(A0)){
		radioData[3] = 1;
	}
	else
	{
		radioData[3] = 0;
	}
	Serial.println("contact radio send");
	radioWrite(radioParent);
	contactChange = false;
}
#endif
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
		if (i == 6 && radioData[2] == 110 && radioData[3] == 10){ // special case for print text to lcd - sets 7th parameter to char array 
			arg = cli.next();
			byte j=0;
			while (true){
				radioData[i+j]=arg[j];
				++j;
				if (arg[j]==0){break;}
			}
			break;
		}
		else
		{
		radioData[i] = atoi(cli.next()); //packet type
		}
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
		double Irms;
		float humidity;
		float temperature;
		int inttemp;
		switch (radioData[2])
		{
		
		case 10: // onewire data recieved
			Serial.print("{\"ID\":");
			Serial.print(radioData[1]);	
			for (byte i=0; i < 9 ; i++)
			{
				onetemp= ((float)(radioData[(i*2)+3]<<8)+radioData[(i*2)+4])/100;
				if (onetemp == 0) {break;}	
				Serial.print(",\"Temp_");
				Serial.print(radioData[1]);
				Serial.print('_');
				Serial.print(i);
				Serial.print("\":");
				Serial.print(onetemp);
			}
			Serial.println("}");
			break;
		case 11: // Light data recieved
			Serial.print("{\"ID\":");
			Serial.print(radioData[1]);	
			Serial.print(",\"Light_");
			Serial.print(radioData[1]);	
			Serial.print("_0:");
			Serial.print(((int)(radioData[3]<<8)+radioData[4]));	
			Serial.println("}");
			break;
		case 13: // Humidity data recieved
			Serial.print("{\"ID\":");
			Serial.print(radioData[1]);	
			Serial.print(",\"Humidity_");
			Serial.print(radioData[1]);	
			Serial.print("_0:");
			Serial.print(((float)(radioData[3]<<8)+radioData[4])/100);	
			Serial.println("}");
			break;
		case 14: // Vin data recieved
			Serial.print("{\"ID\":");
			Serial.print(radioData[1]);	
			Serial.print(",\"Vin_");
			Serial.print(radioData[1]);	
			Serial.print("_0:");
			Serial.print(((int)(radioData[3]<<8)+radioData[4]));	
			Serial.println("}");
			break;	
		case 15: // Contact data recieved
			Serial.print("{\"ID\":");
			Serial.print(radioData[1]);	
			Serial.print(",\"Contact_");
			Serial.print(radioData[1]);	
			Serial.print("_0:");
			Serial.print(radioData[3]);	
			Serial.println("}");
			break;
		case 50: //request 1wire data
			reportOnewire(radioData[1]); // report 1wire data to the requesting gatherer			
			break;
		case 51: // request light sensor value
			startRadioPacket(radioData[1],11);
			inttemp = analogRead(A7);
			radioData[3] =(inttemp>>8);
			radioData[4] = ((byte) (inttemp));
			radioWrite(radioData[1]);
			break;
 
  		case 52: // request power value
			
			 Irms = emon1.calcIrms(148*(radioData[3]+1));  // Calculate Irms only
  			 Serial.print(Irms*9.51) ;	       // Apparent power
			 Serial.print(" ");
			 Serial.println(Irms);		       // Irms
			Serial.println(analogRead(A3));
			break;
		case 53: // requiest humidity
			  humidity = dht.getHumidity();
			//  temperature = dht.getTemperature();
			//  Serial.print(dht.getStatusString());
			 // Serial.print("\t");
			 // Serial.print(humidity, 1);
			 // Serial.print("\t\t");
			  //Serial.print(temperature, 1);
			  //Serial.print("\t\t");
			  //Serial.println(dht.toFahrenheit(temperature), 1);
			startRadioPacket(radioData[1],13);
			inttemp = humidity*100;
			radioData[3] =(inttemp>>8);
			radioData[4] = ((byte) (inttemp));
			radioWrite(radioData[1]);
			break;	
		case 54: //request Vin
			startRadioPacket(radioData[1],14);
			inttemp =  readVcc();
			radioData[3] =(inttemp>>8);
			radioData[4] = ((byte) (inttemp));
			radioWrite(radioData[1]);
		//  Serial.println( readVcc(), DEC );
		  break;
		case 55: //request contact status
			startRadioPacket(radioData[1],15); // packet type contact change data
			if (digitalRead(A0)){radioData[3] = 1;}	else {radioData[3] = 0;} // read the contact
			radioWrite(radioData[1]);

		case 100: //set led color (led,R,G,B)
			strip.setPixelColor(radioData[3],radioData[4],radioData[5],radioData[6]);
			strip.show();
			break;
#ifdef LCD
		case 200: // LCD Commands
		int j=6;
		char text[20];
		switch (radioData[3]) {
			case 0: // backlight (0-255)
				analogWrite(3,radioData[4]); //lcd bacllight
				break;
			case 1: // clear screen
				tft.fillScreen(ST7735_BLACK);
				break;	
			case 2: // draw pixel
				tft.drawPixel(radioData[4],radioData[5],(int) radioData[6]<<8+radioData[7]);
				break;
			case 3: // draw line
				tft.drawLine(radioData[4],radioData[5],radioData[6],radioData[7],(int) (radioData[8]<<8)+radioData[9]);
				break;
			case 4: // draw verticle line
				tft.drawFastVLine(radioData[4],radioData[5],radioData[6],(int) (radioData[7]<<8)+radioData[8]);
				break;
			case 5: // draw Horizontal line
				tft.drawFastHLine(radioData[4],radioData[5],radioData[6],(int) (radioData[7]<<8)+radioData[8]);
				break;
			case 6: // draw line
				tft.drawRect(radioData[4],radioData[5],radioData[6],radioData[7],(int) (radioData[8]<<8)+radioData[9]);
				break;
			case 7: // draw line
				tft.fillRect(radioData[4],radioData[5],radioData[6],radioData[7],(int) (radioData[8]<<8)+radioData[9]);
				break;
			case 10: // text
				tft.setCursor(radioData[4],radioData[5]);
				 
				while (true){
					if (radioData[j] == 95){ // replace underscore with space
					text[j-6]=32;
					}else{
					text[j-6]=char(radioData[j]);
					}
					if (radioData[j]==0){break;}
					++j;
				}
				//Serial.print(text);	
				//Serial.print('asdf');
				tft.print(text);
			case 11: //text size
			break;
			case 12: // text color with background
			break;
		}
		
		
#endif	
		}
	
}

#endif
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}
void startRadioPacket(byte sendAddress,byte packetType){
	for(byte i=0; i < PACKETSIZE; ++i){radioData[i]=0;} // clear buffer
	radioData[0] = sendAddress; //dest
	radioData[1] = radioThis; //source
	radioData[2] = packetType; // packet type 1wire data	
	return;
}

#ifdef ONEWIRE
void reportOnewire(byte sendAddress){
	dow.requestTemperatures();
	startRadioPacket(sendAddress,10);


	for (byte i=0; i < dowCount; i++){
		int inttemp = ((double) dow.getTempF(dows[i])*100);
		unsigned int hbyte =(inttemp>>8);
		radioData[(i*2)+3] =(hbyte);
		radioData[(i*2)+4] = ((byte) (inttemp));
	}
	radioWrite(sendAddress);
}
#endif



#ifdef CONTACT
	void contactInterupt(){   //interupt happens on state change
	contactChange = true;
	}
#endif


//// function to print a device address
	////#ifdef ONEWIRE
//void printAddress((uint8_t[8]) deviceAddress)
//{
	//for (uint8_t i = 0; i < 8; i++)
	//{
	//if (deviceAddress[i] < 16) Serial.print("0");
	//Serial.print(deviceAddress[i], HEX);
	//}
	//Serial.println("");
//}
	////#endif