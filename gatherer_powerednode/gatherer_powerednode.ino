
// All the dependant Libraries
#define IR
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
#include <Metro.h>
Metro uiCheck = Metro(75,true);
byte uiMenuCount = 0;

byte uiSelect[6] = {0,0,0,0,0,0};
boolean uiMenuMode = false;
Metro screenupdate = Metro(2000,true);


byte screenupdatecount = 15;
#ifdef IR
	#include <i2cmaster.h>

#endif
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
	#define EEPROMPACKETSIZE 16
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

Metro t0 = Metro(getinterval(0),true);
Metro t1 = Metro(getinterval(1),true);
Metro t2 = Metro(getinterval(2),true);
Metro t3 = Metro(getinterval(3),true);
Metro t4 = Metro(getinterval(4),true);
//Metro t5 = Metro(getinterval(5));
//Metro t6 = Metro(getinterval(6));
//Metro t7 = Metro(getinterval(7));
//

void setup() {
   pinMode(3,OUTPUT); // two transistor outputs
   // digitalWrite(3,LOW); //lcd bacllight
  digitalWrite(3,HIGH); //lcd bacllight
 Serial.begin(115200);
 
 



 pinMode(6,OUTPUT);
 pinMode(A7,INPUT);
 digitalWrite(6,LOW); //

#ifdef IR
	#include <i2cmaster.h>
	i2c_init(); //Initialize the i2c bus
	PORTC = (1 << PORTC4) | (1 << PORTC5);//enable pullups
#endif

#ifdef HUMIDITY


  dht.setup(A2); // data pin 2

#endif

#ifdef POWER
	emon1.current(3, 111.1);             // Current: input pin, calibration.
	emon1.calcIrms(1480);  // Calculate Irms only .. first value usually not correct
		emon1.calcIrms(1480);  // Calculate Irms only .. first value usually not correct
#endif

#ifdef LCD
	tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab  
	tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
//if (bitRead(EEPROM.read(0),4)){tft.setRotation(2);}//flipdisplay

//tft.fillRect(0,0, 128, 20, ST7735_BLACK);
 
 //tft.fillScreen(ST7735_BLACK);

#endif

#ifdef CONTACT
	pinMode(A0,INPUT_PULLUP);  // set a0  as contact closeure 
	PCintPort::attachInterrupt(A0, &contactInterupt, CHANGE);

#endif
//#ifdef LEDS
	strip.begin();
	strip.setPixelColor(0,5,5,5);
	strip.show();


#ifdef ONEWIRE
 dow.begin();
	
 
dowCount=dow.getDeviceCount();
#ifdef CLI
//	Serial.print("1Wire Device count:");
//	Serial.println(dowCount,DEC);
#endif

if (dowCount > 9){ //limit to 10 1wire sensors so transmitt doesnt break - possible to increase to about 14
dowCount=9;}
	
 for (int i=0; i < dowCount; i++){
dow.getAddress(dows[i],i);
dow.setResolution(dows[i],12);


}

dow.setWaitForConversion(FALSE);
dow.requestTemperatures();

#endif
#ifdef CLI
	cli.addCommand("SET",cliSet); 
	cli.addCommand("R",cliRadio); 
	cli.setDefaultHandler(cliUnrecognized);
	//println("CLI Active");
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


#endif
// first 15 eeprom packet executed at startup
for (int i= 0; i < 15; ++i){
	//Serial.println(i);
	radiowritefromeeprom(i);
	}

}
void loop(){
	if (uiCheck.check() == 1){ui();}
	if (t0.check() == 1){
		radiowritefromeeprom(30);
		radiowritefromeeprom(31);
	}
	//if (t1.check() == 1){
		//radiowritefromeeprom(32);
		//radiowritefromeeprom(33);
	//}
	//if (t2.check() == 1){
		//radiowritefromeeprom(34);
		//radiowritefromeeprom(35);
	//}
	//if (t3.check() == 1){
		//radiowritefromeeprom(36);
		//radiowritefromeeprom(37);
	//}
	//if (t4.check() == 1){
		//radiowritefromeeprom(38);
		//radiowritefromeeprom(39);
	//}
	//if (t5.check() == 1){
		//radiowritefromeeprom(40);
		//radiowritefromeeprom(41);
	//}
	//if (t6.check() == 1){
		//radiowritefromeeprom(42);
		//radiowritefromeeprom(43);
	//}
	//if (t7.check() == 1){
		//radiowritefromeeprom(44);
		//radiowritefromeeprom(45);
	//}

	if(screenupdate.check() == 1 and radioThis == 5){
	//eeprom packets 15-29 run constantly one ever 2 seconds	
	if(screenupdatecount > 29) {screenupdatecount = 15;}
//		Serial.print(screenupdatecount);
		radiowritefromeeprom(screenupdatecount);
		++screenupdatecount;
		//radioData = {3,5,65,0,6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
		//radioData[0] = 3;
		//radioData[1] = 5;
		//radioData[2] = 50;
		//radioData[3] = 0;
		//radioData[4] = 6;
		//radioData[5] = 0;
		//radioData[6] = 0;
		//radioWrite(radioData[0]);
		//radioData[0] = 5;
		//radioData[1] = 5;
		//radioData[2] = 56;
		//radioData[3] = 0;
		//radioData[4] = 6;
		//radioData[5] = 0;
		//radioData[6] = 40;
		//radioWrite(radioData[0]);
	}
#ifdef CONTACT
if (contactChange){
	radioData[4] = 0;
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
	for (byte i = 2; i < PACKETSIZE;++i)
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
	//
		//else
			//{
				//Serial.print("?:");
				//Serial.println(arg);
			//}
		}
	else
	{
		Serial.println("ID");
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
	//Serial.print(radioThis);
	//Serial.print("sending packet to :");
	//Serial.println(destaddress);
#endif
	delay(25); //?
			Serial.print("{\"ID\":");
			Serial.print(radioThis);
	if (radio.write( &radioData, PACKETSIZE ) == 1){
		//++radiosuccess;
//#ifdef DEBUG

		Serial.println(",\"Packet\":true}");
//#endif
	}
	else{
		//++radiofail;
//#ifdef DEBUG
		
		Serial.println(",\"Packet\":false}");
//#endif
	}
	radio.startListening();
		delay(25);//?
}
void radioProcess()
{
	//  byte 0 of the packet is the destination address
		byte destaddress = radioData[0];
		//#ifdef DEBUG
		// byte 1 is origination address
			
			//Serial.print(radioData[1]);
			//Serial.print(" sent a packet for:");
			//Serial.print(destaddress);
			//Serial.print(" type:");
			//Serial.print(radioData[2]);
			//Serial.print(" +");
		//Serial.print(radioData[3]);
	//Serial.print(":");
	//Serial.print(radioData[4]);
	//Serial.print(":");
	//Serial.print(radioData[5]);
	//Serial.print(":");
	//Serial.println(radioData[6]);
	
		//#endif
		// byte 2 contains the packet type
		float onetemp;
		double Irms;
		float humidity;
		float temperature;
		int inttemp;
	//		int celcius; //in sensor
		byte parm = radioData[3] ;
double tempData = 0x0000; // zero out the data // ir sensor
byte x;
  	switch (radioData[2])
		{
		
		case 10: // onewire data recieved
	printid();
			if (radioData[3]>9){
				for (byte i=0; i < 9 ; i++)
				{
					//onetemp= ((float)(radioData[(i*2)+4]<<8)+radioData[(i*2)+5])/100;
					if (radioData[(i*2)+4]+radioData[(i*2)+5] == 0) {break;}	
					printint(10,i);

					Serial.print("\":");
					Serial.print(radiofloat((i*2)+4));
				}
			} else
			{
				//onetemp= ((float)(radioData[4]<<8)+radioData[5])/100;
				//if (onetemp == 0) {break;}	
				printint(10,radioData[3]);
				Serial.print("\":");
				Serial.print(radiofloat(4));
			}
			Serial.println("}");
			break;
		case 11: // Light data recieved
			printid();
			
			printint(11,0);
				Serial.print("\":");
			Serial.print(radioint(4));	

			Serial.println("}");
			break;
		case 12: // Power data recieved
			printid();
			printint(12,0);
				Serial.print("\":");
			Serial.print(radioint(4));	
			Serial.println("}");
			break;
		case 13: // Humidity data recieved
			printid();
			printint(13,0);
				Serial.print("\":");
			Serial.print(radiofloat(4));	
			Serial.println("}");
			break;
		case 14: // Vin data recieved
			printid();
			printint(14,0);
			Serial.print("\":");
			Serial.print(radioint(4));	
			Serial.println("}");
			break;	
		case 15: // Contact data recieved
			printid();
			
			printint(15,0);
			Serial.print("\":");
			Serial.print(radioint(4));	
			Serial.println("}");
			break;
		case 16: // IR data recieved
			printid();
			printint(16,0);
				Serial.print("\":");
			Serial.print(radiofloat(4));	
			Serial.println("}");
			break;
		case 19: 
			printid();
				
				printint(10,radioData[3]);
				Serial.print("\":");
				Serial.print(radiofloat(4));
				printint(11,0);
					Serial.print("\":");
				Serial.print(radioint(6));	
				printint(12,0);
				Serial.print("\":");
				Serial.print(radioint(8));	
				printint(13,0);
				Serial.print("\":");
				Serial.print(radiofloat(10));	
				printint(14,0);
					Serial.print("\":");
				Serial.print(radioint(12));	
				printint(15,0);
					Serial.print("\":");
				Serial.print(radioint(14));	
				printint(16,0);
				Serial.print("\":");
				Serial.print(radiofloat(16));	
				Serial.println("}");
				break;
		case 20: // menu event
		printid();
			Serial.print(",\"Menu_");
			Serial.print(radioData[1]);	
			Serial.print("\":");
			Serial.print(radioData[4]);
			Serial.println("}");
			break;
		case 50: //request 1wire data
				dow.requestTemperatures();
				startRadioPacket(radioData[1],10);

				//radioData[3] = parm;
				if (parm > 9 ) {
					for (byte i=0; i < dowCount; i++){
						int inttemp = ((double) dow.getTempF(dows[i])*100);

						radioData[(i*2)+4] =(inttemp>>8);
						radioData[(i*2)+5] = ((byte) (inttemp));
					}
						radioData[(dowCount*2)+4] = 0;
						radioData[(dowCount*2)+5] = 0;
						radioWrite(radioData[0]);
						
				}
				else
				{
					radiobyte((double) dow.getTempF(dows[parm])*100);
			
					//radioData[6] = 0;
					//radioData[7] = 0;
				}
				
				//

			//reportOnewire(); // report 1wire data to the requesting gatherer			
			break;
		case 51: // request light sensor value
			startRadioPacket(radioData[1],11);
			radiobyte(analogRead(A7));
		
			break;
 
  		case 52: // request power value
			
			 //Irms = emon1.calcIrms(148*(parm+1));  // Calculate Irms only
  			// inttemp = (Irms*9.51)-3;
			 //Serial.print(Irms*9.51) ;// Apparent with correction for 330 ohm burdon
			 //startRadioPacket(radioData[1],12);
			//radioData[4] =(inttemp>>8);
			//radioData[5] = ((byte) (inttemp));
			
			startRadioPacket(radioData[1],12);
			radiobyte( (emon1.calcIrms(148*(parm+1))*9.51)-3);
			//radioWrite(radioData[0]); moved to radiobyte
			break;
		case 53: // requiest humidity

			//  temperature = dht.getTemperature();
			//  Serial.print(dht.getStatusString());
			 // Serial.print("\t");
			 // Serial.print(humidity, 1);
			 // Serial.print("\t\t");
			  //Serial.print(temperature, 1);
			  //Serial.print("\t\t");
			  //Serial.println(dht.toFahrenheit(temperature), 1);
			startRadioPacket(radioData[1],13);
			radiobyte(dht.getHumidity()*100);
			break;	
		case 54: //request Vin
			startRadioPacket(radioData[1],14);
			radiobyte(readVcc());
		
		//  Serial.println( readVcc(), DEC );
		  break;
		case 55: //request contact status
			startRadioPacket(radioData[1],15); // packet type contact change data
			radioData[4]=0;
			radioData[5] = digitalRead(A0);
			radioWrite(radioData[0]);
			break;
		case 56: // ir
		//	inttemp = readIr();
			startRadioPacket(radioData[1],16);  //make type 16 ir respose
			radiobyte(readIr());
			
			break;
		case 59: // all sensors
		startRadioPacket(radioData[1],19);  //make type 16 ir respose
		 valtoradio((double) dow.getTempF(dows[parm])*100,4); //1wire
		 valtoradio(analogRead(A7),6); //light
		 valtoradio((emon1.calcIrms(1480)*9.51)-3,8); //power
		 valtoradio(dht.getHumidity()*100,10); //humidity
		 valtoradio(readVcc(),12); //proc vcc
		 valtoradio( digitalRead(A0),14); //contact
		 valtoradio(readIr(),16); // ir
		 radioWrite(radioData[0]);
			//radiobyte(readIr());
		case 100: //set led color (led,R,G,B)
			x=1;
			for (byte i = 0; i<6; ++i){
			if (radioData[3] & x){
			strip.setPixelColor(i,radioData[4],radioData[5],radioData[6]);


			}
			x=x+x;
			}
			strip.show();
			break;
		case 150: //send radio packet stored in eeprom (or exicute it locally)
			radiowritefromeeprom(radioData[3]);
			break;
		case 151: // write radio packet to eeprom (or exicute it locally) trims of the first 4 bytes
			radiowritetoeeprom(radioData[3]);
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
				tft.drawPixel(radioData[4],radioData[5],radioint(6));
				break;
			case 3: // draw line
				tft.drawLine(radioData[4],radioData[5],radioData[6],radioData[7],radioint(8));
				break;
			case 4: // draw verticle line
				tft.drawFastVLine(radioData[4],radioData[5],radioData[6],radioint(7));
				break;
			case 5: // draw Horizontal line
				tft.drawFastHLine(radioData[4],radioData[5],radioData[6],radioint(7));
				break;
			case 6: // draw rectangle
				tft.drawRect(radioData[4],radioData[5],radioData[6],radioData[7],radioint(8));
				break;
			case 7: // draw fill rectangle
				tft.fillRect(radioData[4],radioData[5],radioData[6],radioData[7],radioint(8));
				break;
			case 10: // text
				tft.setCursor(radioData[4],radioData[5]);
				tft.setTextSize(1); 
				while (true){
					if (radioData[j] == 95){ // replace underscore with space
					text[j-6]=32;
					}else{
					text[j-6]=char(radioData[j]);
					}
					if (radioData[j]==0){break;}
					++j;
				}
				tft.print(text);
				break;
			case 11: // text
				tft.setCursor(radioData[4],radioData[5]);
				tft.setTextSize(2);
				while (true){
					if (radioData[j] == 95){ // replace underscore with space
					text[j-6]=32;
					}else{
					text[j-6]=char(radioData[j]);
					}
					if (radioData[j]==0){break;}
					++j;
				}
				tft.print(text);
				break;
			case 12: // text
				//tft.setCursor(radioData[6],radioData[7]);
				//tft.setTextSize(1);
				setcursorsize(1);
				tft.print(radioint(4));	
				break;
			case 13: // text
				setcursorsize(1);
				tft.print(((float)(radioData[4]<<8)+radioData[5])/100);	
				break;
			case 14: // text
				setcursorsize(2);
				tft.print(radioint(4));	
				break;
			case 15: // text
				setcursorsize(2);
				tft.print(((float)(radioData[4]<<8)+radioData[5])/100);	
				break;
			case 16: // text
				setcursorsize(3);
				tft.print(radioint(4));	
				break;
			case 17: // text
				setcursorsize(3);
				tft.print(((float)(radioData[4]<<8)+radioData[5])/100);	
				break;
	
			case 20: // text color with background
			break;
		}
		
		break;
		
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
	//for(byte i=0; i < PACKETSIZE; ++i){radioData[i]=0;} // clear buffer
	radioData[2] = packetType; // packet type 1wire data	
	if (radioData[4] > 0  && packetType < 19 ) //changed from 50 for type 19 and 20 all sensor and menu to be excluded
	{
		
		radioData[7] = radioData[6]; // xpos of text
		radioData[6] = radioData[5]; // ypos of text
	//this is a results to screen as int packet requestradioData[3]
		radioData[2] = 200; // change packet type to write on screen
		radioData[3] = 11+radioData[4];  // 1 print int 2 print int/100 3 print double size in 4 double size int4/100
	}
	radioData[0] = sendAddress; //dest
	radioData[1] = radioThis; //source
	
	return;
}




#ifdef CONTACT
	void contactInterupt(){   //interupt happens on state change
	contactChange = true;
	}
#endif
int readIr()
{
		  int dev = 0x5A<<1; // ir sensor
		  int data_low = 0;// ir sensor
		  int data_high = 0;// ir sensor
		  int pec = 0;// ir sensor
		  int inttemp;
			  i2c_start_wait(dev+I2C_WRITE);
			  i2c_write(0x07);
		//    i2c_write(0x06); //internal temp
			  i2c_rep_start(dev+I2C_READ);
			  data_low = i2c_readAck(); //Read 1 byte and then send ack
			  data_high = i2c_readAck(); //Read 1 byte and then send ack
			  pec = i2c_readNak();
			  i2c_stop();
			  //This converts high and low bytes together and processes temperature, MSB is a error bit and is ignored for temps
  			  // This masks off the error bit of the high byte, then moves it left 8 bits and adds the low byte.
			  inttemp = (int)(((data_high & 0x007F) << 8) + data_low);
			  inttemp = (inttemp * 2)-1;// 0.02 degrees per LSB (measurement resolution of the MLX90614) this will be 100x the temp
			  inttemp = inttemp - 27315;
			  //return 100X temp fehrenheit
			  return (inttemp*1.80)+3200; 
			  
}

void ui()
{
int ir = readIr();
if (uiMenuMode == false)	{
	// if temp > threshold start counting to see if we should go to menu mode
	if (ir > 7800){
	++uiMenuCount;
	} 
	else{
	if (uiMenuCount != 0) {--uiMenuCount;}
	}
	//strip.setPixelColor(5,0,0,uiMenuCount*2);
	if (uiMenuCount > 20){
	menuchange(0);
	uiMenuMode = true;
	uiMenuCount = 40;
	strip.setPixelColor(5,0,40,0);
	strip.show();
//send menu started packet
	}
	//strip.show();
} else
{
	for (int i = 0; i < 5;++i){	strip.setPixelColor(i,0,0,0);}
	if (ir > 7700)
	{
		uiselect(1);
		
	} else 
	if (ir > 7650)
	{
		uiselect(2);
	} else
	if (ir > 7600)
	{
		uiselect(3);

	} else
	if (ir > 7550)
	{
		uiselect(4);	
	} else
	if (ir > 7500)
	{
		uiselect(5);
	} else
		// if nothing selected decdecrement the menuexit counter
		if (uiMenuCount != 0) {--uiMenuCount;
		} else
		{//stop menu
			uiMenuMode = false;
	menuchange(9);

	for (int i = 0; i < 5;++i){	strip.setPixelColor(i,0,0,0);}
	strip.show();

		}
	strip.setPixelColor(5,0,uiMenuCount*.5,0);

strip.show();
	}
}


void menuchange(byte item){
	//send packet
		radioData[4] = 0;
		startRadioPacket(radioParent,20); // packet type contact change data
		radioData[4] = item;
		radioWrite(radioParent);

}
void printid(){
			Serial.print("{\"ID\":");
			Serial.print(radioData[1]);	
		
}
void printint(byte id,byte num){
				
			switch (id){
				case 10:
					Serial.print(",\"Temp");
					Serial.print(num);
					break;
				case 11:
					Serial.print(",\"Light");
					break;
				case 12:
					Serial.print(",\"Power");
					break;
				case 13:
					Serial.print(",\"Humidity");
					break;
				case 14:
					Serial.print(",\"Vin");
					break;
				case 15:
					Serial.print(",\"Contact");
					break;
				case 16:
					Serial.print(",\"IR");
					break;
			}
			Serial.print("_");
			Serial.print(radioData[1]);	
		//Serial.print(":");
			
}

void uiselect(byte id){
		uiSelect[0] = uiSelect[id];
		for (int i =1;i<6;++i){uiSelect[i]=0;}
	uiSelect[id] = uiSelect[0]+2;
		strip.setPixelColor(id-1,0,0,50-uiSelect[id]);
	
		if (uiSelect[id] > 50){
		uiSelect[id] = 0;	
		menuchange(id);
		}
		uiMenuCount = 80;

}
int radioint(byte start){
	
	return (radioData[start]<<8)+radioData[start+1];
	}
	
float radiofloat(byte start){
	//return 11.1;
	return ((float)((radioData[start]<<8)+radioData[start+1])/100);

}
void radiobyte(int val){
	valtoradio(val,4);
//	radioData[4] =(val>>8);
//	radioData[5] = ((byte) (val));
	radioWrite(radioData[0]);
}
void valtoradio(int val,byte where){
	radioData[where] =(val>>8);
	radioData[where+1] = ((byte) (val));
}
void setcursorsize(byte size){
				tft.setCursor(radioData[6],radioData[7]);
				tft.setTextSize(size);
}
void radiowritefromeeprom(byte packetnumber){
	int j = 0;
	for (int i=(packetnumber*EEPROMPACKETSIZE)+101; i < (packetnumber*EEPROMPACKETSIZE)+101+EEPROMPACKETSIZE; ++i){
		radioData[j] = EEPROM.read(i);
		
		
		++j;
	}

	if(radioData[0] > 0){
		radioWrite(radioData[0]);
		//Serial.print("Sent");
		//Serial.println(packetnumber);
		
	}
}
void radiowritetoeeprom(byte packetnumber){
//	Serial.println("write to eeprom");
	int j = 4;

	for (int i=(packetnumber*EEPROMPACKETSIZE)+101; i < ((packetnumber*EEPROMPACKETSIZE)+101)+EEPROMPACKETSIZE; ++i){
		EEPROM.write(i,radioData[j]);
			//Serial.print(j);
		//Serial.print(":");
		//Serial.println(radioData[j]);
		++j;
		
	}
}
long getinterval(byte timer){
	//interval info is stored in block 46
	int location = (46*EEPROMPACKETSIZE)+101+(timer*2);
	//Serial.print(timer);
	//Serial.print(" : ");
	Serial.println((EEPROM.read(location)*60000)+(EEPROM.read(location+1)*100));
	return ((EEPROM.read(location)*60000)+(EEPROM.read(location+1)*100));
//return 1000;
}
	