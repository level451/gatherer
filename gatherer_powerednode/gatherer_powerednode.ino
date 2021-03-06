//laptop test
//int cn = 0;
// All the Dependant Libraries
#define IR
#define HUMIDITY
//#define POWER
//#define LCD 
#define CONTACT
#define DEBUG
#define LEDS // Enable WS2812 LEDS 
#define CLI //enable command line interface 
#define RADIO //enables nRF24l01+ module
#define ONEWIRE
#include <Arduino.h>
#include <EEPROM.h>
#include <Metro.h>
Metro uiCheck = Metro(100,true);
byte uiMenuCount = 0;

byte uiSelect[6] = {0,0,0,0,0,0};
boolean uiMenuMode = false;
Metro screenupdate = Metro(2000,true);



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
EnergyMonitor emon2;                   // Create an instance
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
	//#define sclk 4
	//#define mosi 5
	#define cs   8
	#define dc   7
	#define rst  4  // you can also connect this to the Arduino reset
	#define	ST7735_BLACK   0x0000
	#define	ST7735_BLUE    0x001F
	#define	ST7735_RED     0xF800         
	#define	ST7735_GREEN   0x07E0
	#define ST7735_CYAN    0x07FFb
	#define ST7735_MAGENTA 0xF81F
	#define ST7735_YELLOW  0xFFE0
	#define ST7735_WHITE   0xFFFF
	//Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc, mosi, sclk, rst);
	Adafruit_ST7735 tft = Adafruit_ST7735(cs, dc,rst);
	
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
	int radiosuccess; // packet acked count.
	int radiofail; // packets not acked count
	byte radioThis = EEPROM.read(10);
	byte radioParent = 3;

#endif
byte t0packet = 15;
Metro t0 = Metro(getinterval(0),true);
Metro t1 = Metro(getinterval(1),true);
Metro t2 = Metro(getinterval(2),true);
Metro t3 = Metro(getinterval(3),true);


void setup() {
	strip.begin();
		strip.setPixelColor(0,5,0,0);
		strip.show();
		


 pinMode(3,OUTPUT); // lcd backlight transitor
 pinMode(6,OUTPUT); // transitor 2 output
   // digitalWrite(3,LOW); //lcd bacllight
 digitalWrite(3,HIGH); //lcd bacllight
 Serial.begin(115200);

 



 pinMode(6,OUTPUT);
 digitalWrite(6,LOW); //
 pinMode(A7,INPUT); // adc for light sensor
 

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
	emon2.current(6, 111.1);             // Current: input pin, calibration.

#endif

#ifdef LCD
	tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab  
	//tft.setTextColor(ST7735_WHITE,ST7735_BLACK);
	tft.setTextWrap(false);
//if (bitRead(EEPROM.read(0),4)){tft.setRotation(2);}//flipdisplay

//tft.fillRect(0,0, 128, 20, ST7735_BLACK);
 
 //tft.fillScreen(ST7735_BLACK);

#endif

#ifdef CONTACT
	pinMode(A0,INPUT_PULLUP);  // set a0  as contact closeure 
	PCintPort::attachInterrupt(A0, &contactInterupt, CHANGE);

#endif



 
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
	radio.setRetries(3,15); // delay 3 = 2*250 uS + 150 base = 750 uS delay between retries and 5 retries max
	radio.setPayloadSize(PACKETSIZE); // packet 32 bytes (max is 32)
	radio.setPALevel(RF24_PA_MAX);
	radio.setCRCLength(RF24_CRC_16); // 2 byte crc
	radio.setDataRate(RF24_250KBPS);
	radio.setAutoAck(1); 
	radio.setChannel(96); //was 76
	radio.openReadingPipe(1,pipe+radioThis);	
	radio.startListening();


#endif




// first 15 eeprom packet executed at startup
for (int i= 0; i < 15; ++i){
	//Serial.println(i);
	radiowritefromeeprom(i);
strip.setPixelColor(0,17*i,17*i,17*i);
strip.show();

	}
		strip.setPixelColor(0,0,1,0);
		strip.show();
		
}
void loop(){

	//++cn;
	//if (uiCheck.check() == 1){ui();}
	if (t1.check() == 1){
		radiowritefromeeprom(30);
		radiowritefromeeprom(31);
		radiowritefromeeprom(32);
		
		//cn = 0;
	}
	//Serial.println("end");
	if (t2.check() == 1){

		radiowritefromeeprom(33);
		radiowritefromeeprom(34);
		radiowritefromeeprom(35);
		
	}
	if (t3.check() == 1){
		radiowritefromeeprom(36);
		radiowritefromeeprom(37);
		radiowritefromeeprom(38);
	}


	if(t0.check() == 1 ){
	//eeprom packets 15-29 run constantly one ever 2 seconds	
	if(t0packet > 29) {t0packet = 15;}
//		Serial.print(t0packet);
		radiowritefromeeprom(t0packet);
		++t0packet;

	}
#ifdef CONTACT
if (contactChange){
	radioData[4] = 0;
	startRadioPacket(radioParent,15); // packet type contact change data
	boolean state = digitalRead(A0);
	if (state){
		radioData[3] = 1;
	}
	else
	{
		radioData[3] = 0;
	}
	Serial.println("contact radio send");
	radioWrite(radioParent);
	if (state){
		radiowritefromeeprom(53);
		radiowritefromeeprom(54);
		radiowritefromeeprom(55);
	}else
	{
		radiowritefromeeprom(56);
		radiowritefromeeprom(57);
		radiowritefromeeprom(58);
	}
	
		
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
	//radioData[1] = radioThis;
	radioData[1] = atoi(cli.next());

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
	//	Serial.println(pipe_num);
		
		if (pipe_num == 1){	
	strip.setPixelColor(0,0,5,0);
	strip.show();
			radioProcess();
			
		}
		strip.setPixelColor(0,0,0,0);
		strip.show();
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
	//delay(15); //?
	radio.write( &radioData, PACKETSIZE );
			//Serial.print("{\"ID\":");
			//Serial.print(radioThis);
	//if (radio.write( &radioData, PACKETSIZE ) == 1){
		//++radiosuccess;
//#ifdef DEBUG

		//Serial.println(",\"Packet\":true}");
//#endif
	//}
	//else{
		//++radiofail;
//#ifdef DEBUG
		
		//Serial.println(",\"Packet\":false}");
//#endif
	//}
	radio.startListening();
	//	delay(5);//?
}
void radioProcess()
{
	//  byte 0 of the packet is the destination address
		byte destaddress = radioData[0];
		boolean ping = false;
		if (radioData[2] & B10000000){

			radioData[2] = radioData[2] & B01111111;
			ping = true;
		}
		
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
		
		case 1:
			printid(1);
			Serial.println("}");
			break;
		case 10: // onewire data recieved
	printid(10);
			if (radioData[3]>9){
				for (byte i=0; i < 9 ; i++)
				{
					//onetemp= ((float)(radioData[(i*2)+4]<<8)+radioData[(i*2)+5])/100;
					if (radioData[(i*2)+4]+radioData[(i*2)+5] == 0) {break;}	
					printbody(10,i);

					Serial.print("\":");
					Serial.print(radiofloat((i*2)+4));
				}
			} else
			{
				//onetemp= ((float)(radioData[4]<<8)+radioData[5])/100;
				//if (onetemp == 0) {break;}	
				printbody(10,radioData[3]);
				Serial.print("\":");
				Serial.print(radiofloat(4));
			}
			Serial.println("}");
			break;
		case 11: // Light data recieved
			printid(11);
			
			printbody(11,0);
				Serial.print("\":");
			Serial.print(radioint(4));	

			Serial.println("}");
			break;
		case 12: // Power data recieved
			printid(12);
			printbody(12,1);
				Serial.print("\":");
			Serial.print(radioint(4));	
			Serial.println("}");
			break;
		case 13: // Power data recieved
			printid(12);
			printbody(12,2);
				Serial.print("\":");
			Serial.print(radioint(4));	
			Serial.println("}");
			break;	
		case 14: // Humidity data recieved
			printid(13);
			printbody(13,2);
				Serial.print("\":");
			Serial.print(radiofloat(4));	
			Serial.println("}");
			break;
		case 15: // Vin data recieved
			printid(14);
			printbody(14,0);
			Serial.print("\":");
			Serial.print(radioint(4));	
			Serial.println("}");
			break;	
		case 16: // Contact data recieved
			printid(15);
			
			printbody(15,0);
			Serial.print("\":");
			Serial.print(radioint(4));	
			Serial.println("}");
			break;
		case 17: // IR data recieved
			printid(16);
			printbody(16,0);
				Serial.print("\":");
			Serial.print(radiofloat(4));	
			Serial.println("}");
			break;
		case 19: 
			printid(19);
				
				printbody(10,radioData[3]);
				Serial.print("\":");
				Serial.print(radiofloat(4));
				printbody(11,0);
					Serial.print("\":");
				Serial.print(radioint(6));	
				printbody(12,0);
				Serial.print("\":");
				Serial.print(radioint(8));	
				printbody(12,1);
				Serial.print("\":");
				Serial.print(radioint(10));	
				printbody(13,0);
				Serial.print("\":");
				Serial.print(radiofloat(12));	
				printbody(14,0);
					Serial.print("\":");
				Serial.print(radioint(14));	
				printbody(15,0);
					Serial.print("\":");
				Serial.print(radioint(16));	
				printbody(16,0);
				Serial.print("\":");
				Serial.print(radiofloat(18));	
				Serial.println("}");
				break;
		case 20: // menu event
		printid(20);
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
			
		
			startRadioPacket(radioData[1],12);
			#ifdef POWER
			radiobyte( (emon1.calcIrms(148*(parm+1))*9.51)-3);
			#endif
			break;
		case 53: // request power value
			
		
			startRadioPacket(radioData[1],13);
			#ifdef POWER
			radiobyte( (emon2.calcIrms(148*(parm+1))*9.51)-3);
			#endif
			break;
		case 54: // requiest humidity

			//  temperature = dht.getTemperature();
			//  Serial.print(dht.getStatusString());
			 // Serial.print("\t");
			 // Serial.print(humidity, 1);
			 // Serial.print("\t\t");
			  //Serial.print(temperature, 1);
			  //Serial.print("\t\t");
			  //Serial.println(dht.toFahrenheit(temperature), 1);
			startRadioPacket(radioData[1],14);
			
			radiobyte(dht.getHumidity()*100);
			break;	
		case 55: //request Vin
			startRadioPacket(radioData[1],15);
			radiobyte(readVcc());
		
		//  Serial.println( readVcc(), DEC );
		  break;
		case 56: //request contact status
			startRadioPacket(radioData[1],16); // packet type contact change data
			radioData[4]=0;
			radioData[5] = digitalRead(A0);
			radioWrite(radioData[0]);
			break;
		case 57: // ir
		//	inttemp = readIr();
			
			startRadioPacket(radioData[1],17);  //make type 16 
			radiobyte(readIr());
	

			break;
		case 58: // 	
			 digitalWrite(6,parm);
			break;
		case 59: // all sensors
		dow.requestTemperatures(); // got to read 1wire
		startRadioPacket(radioData[1],19);  //make type 16 ir respose
		 valtoradio((double) dow.getTempF(dows[parm])*100,4); //1wire
		 valtoradio(analogRead(A7),6); //light
		 #ifdef POWER 
		 valtoradio((emon1.calcIrms(444)*9.51)-3,8); //power
		 valtoradio((emon2.calcIrms(444)*9.51)-3,10); //power
		 #endif
		 valtoradio(dht.getHumidity()*100,12); //humidity
		 valtoradio(readVcc(),14); //proc vcc
		 valtoradio( digitalRead(A0),16); //contact
		 valtoradio(readIr(),18); // ir
		 radioWrite(radioData[0]);
			//radiobyte(readIr());
		break;
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
		case 110: //send radio packet stored in eeprom (or exicute it locally)
			radiowritefromeeprom(radioData[3]);
			break;
		case 111: // write radio packet to eeprom (or exicute it locally) trims of the first 4 bytes
			radiowritetoeeprom(radioData[3]);
			if (radioData[3] == 59) // timer block - reset timers
			{
				t0.interval(getinterval(0));
				t1.interval(getinterval(1));
				t2.interval(getinterval(2));
				t3.interval(getinterval(3));
				Serial.println(getinterval(0));
				Serial.println(getinterval(1));
				Serial.println(getinterval(2));
				Serial.println(getinterval(3));
				t0.reset();
				t1.reset();
				t2.reset();
				t3.reset();
				
			}
		break;
		case 112:
		// set id to return address
			EEPROM.write(10,radioData[1]);
			radioData[4] = 0;
			startRadioPacket(1,1);
			radioWrite(1);

			radio.stopListening();
			radioThis = EEPROM.read(10);
			radio.openReadingPipe(1,pipe+radioThis);
			radio.startListening();
		break;
		
		case 199: // clear eeprom
		
	for (byte i = 0; i < 51; ++i){
		byte radioData[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	
	radiowritetoeeprom(i);
	
	}
#ifdef LCD
		case 120: // LCD Commands
		int color;
		switch ((radioData[3] & B11110000) >> 4){
		case 0:
			color = 0xFFFF; //WHITE
			break;
		case 1:
			color = 0x001F;	//BLUE
			break;
		case 2:
			color = 0xF800;	//RED
			break;
		case 3:
			color = 0x07E0;	//GREEN
			break;
		case 4:
			color = 0x007FF;	//CYAN
			break;
		case 5:
			color = 0xF81F;	//MEGENTA
			break;
		
		
		}
		tft.setTextColor(color,0);
		switch (radioData[3] & B00001111) {
			case 0: // backlight (0-255)
				analogWrite(3,radioData[4]); //lcd bacllight
				break;
			case 1: // clear screen
				tft.fillScreen(color);
				break;	
			case 2: // draw line
				tft.drawLine(radioData[4],radioData[5],radioData[6],radioData[7],color);
				break;
			case 3: // draw verticle line
				tft.drawFastVLine(radioData[4],radioData[5],radioData[6],color);
				break;
			case 4: // draw Horizontal line
				tft.drawFastHLine(radioData[4],radioData[5],radioData[6],color);
				break;
			case 5: // draw rectangle
				tft.drawRect(radioData[4],radioData[5],radioData[6],radioData[7],color);
				break;
			case 6: // draw fill rectangle
				tft.fillRect(radioData[4],radioData[5],radioData[6],radioData[7],color);
				break;
			case 7: // text
				tfttext(1);
				break;
			case 8: // text
				
				tfttext(2);
				break;
			case 9: // text
				
				tfttext(3);
				break;

			case 10: // text
				tftint(1);
				break;
			case 11: // text
				
				tftfloat(1);
				break;
			case 12: // text
				tftint(2);
				break;
			case 13: // text
				tftfloat(2);
				break;
			case 14: // text
				tftint(3);
				break;
			case 15: // text
				tftfloat(3);
				break;
	
		}
		
		break;
		
#endif	
		}
	
if (ping == true){
	// make a type 1 radio pack3et
	radioData[4] = 0;
	startRadioPacket(radioData[1],1);
	radioWrite(radioData[0]);
	
	
}

}

#endif
#ifdef LCD
	void tfttext(byte textsize){
		tft.setCursor(radioData[4],radioData[5]);
		tft.setTextSize(textsize); 
		int j=6;
			char text[10];
			while (true){
			//if (radioData[j] == 95){ // replace underscore with space
			//text[j-6]=32;
			//}else{
			text[j-6]=char(radioData[j]);
			//}
			if (radioData[j]==0){break;}
			++j;
		}
		tft.print(text);
	}
	void tftfloat(byte cursorsize)
	{
	setcursorsize(cursorsize);
	float data =(((float)(radioData[4]<<8)+radioData[5])/100);
	if (data<10){tft.print(" ");}
	if (data<100){tft.print(" ");}

	tft.print((float)data);	
	}
	void tftint(byte cursorsize){
		setcursorsize(cursorsize);
		if (radioint(4)<10){tft.print(" ");}
		if (radioint(4)<100){tft.print(" ");}
		if (radioint(4)<1000){tft.print(" ");}
		tft.print(radioint(4));	

	}
	void setcursorsize(byte size){
					tft.setCursor(radioData[6],radioData[7]);
					tft.setTextSize(size);
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
		radioData[2] = 120; // change packet type to write on screen
		radioData[3] = 9+radioData[4];  // 1 print int 2 print int/100 3 print double size in 4 double size int4/100
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
#ifdef IR
		  	
		  	strip.setPixelColor(0,5,0,0);
			strip.show();
		  int dev = 0x5A<<1; // ir sensor
		  int data_low = 0;// ir sensor
		  int data_high = 0;// ir sensor
		  int pec = 0;// ir sensor
		  int inttemp;
			  i2c_start_wait(dev+I2C_WRITE);
			
			  strip.setPixelColor(0,0,5,0);
			strip.show();
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
#endif	  
}

void ui()
{
		strip.show();
int ir = readIr();
if (uiMenuMode == false)	{
	// if temp > threshold start counting to see if we should go to menu mode
	if (ir > 9400){
	++uiMenuCount;
	} 
	else{
	if (uiMenuCount != 0) {--uiMenuCount;}
	}
	//strip.setPixelColor(5,0,0,uiMenuCount*2);
	if (uiMenuCount > 10){
	menuchange(0);
	uiMenuMode = true;
	uiMenuCount = 10;
	strip.setPixelColor(5,0,40,0);
	strip.show();
//send menu started packet
	}
	//strip.show();
} else
{
	for (int i = 0; i < 5;++i){	strip.setPixelColor(i,0,0,0);}
	if (ir > 7400)
	{
		uiselect(1);
		
	} else 
	if (ir > 7300)
	{
		uiselect(2);
	} else
	if (ir > 7200)
	{
		uiselect(3);

	} else
	if (ir > 7100)
	{
		uiselect(4);	
	} else
	if (ir > 7000)
	{
		uiselect(5);
	} else
		// if nothing selected decdecrement the menuexit counter
		if (uiMenuCount != 0) {--uiMenuCount;
		} else
		{//stop menu
			uiMenuMode = false;
			t0packet = 15; // reset the 15-29 packet pointer when exiting the menu
			menuchange(6);

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
		radiowritefromeeprom(39+(item*2)); // send EEprom packet 39-52
		radiowritefromeeprom(40+(item*2)); 
}
void printid(byte type){
			Serial.print("{\"ID\":");
			Serial.print(radioData[1]);	
				Serial.print(",\"Type\":");
				Serial.print(type);
		
}
void printbody(byte type,byte num){
			
			switch (type){
				case 10:
					Serial.print(",\"Temp");
					Serial.print(num);
					break;
				case 11:
					Serial.print(",\"Light");
					break;
				case 12:
					Serial.print(",\"Power");
					Serial.print(num);
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
		strip.setPixelColor(id-1,0,0,50-(uiSelect[id])*3);
	
		if (uiSelect[id] > 15){
		strip.setPixelColor(id-1,150,150,150);
		strip.show();
		uiSelect[id] = 0;	
		menuchange(id);
		strip.setPixelColor(id-1,0,0,0);
		strip.show();
		uiMenuMode = FALSE;
		uiMenuCount = 0;
		return;
		}
		uiMenuCount = 20;

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

void radiowritefromeeprom(byte packetnumber){
	int j = 0;
	for (int i=(packetnumber*EEPROMPACKETSIZE)+63; i < (packetnumber*EEPROMPACKETSIZE)+63+EEPROMPACKETSIZE; ++i){
		radioData[j] = EEPROM.read(i);
		
		
		++j;
	}
	//Serial.print("EEprom");
	//Serial.println(packetnumber);
	if(radioData[0] > 0){
		radioWrite(radioData[0]);
		//Serial.print("Sent");
		//Serial.println(packetnumber);
		
	}
}
void radiowritetoeeprom(byte packetnumber){
	//Serial.print("write to eeprom");
	//Serial.println(packetnumber);
	int j = 4;
	for (int i=(packetnumber*EEPROMPACKETSIZE)+63; i < ((packetnumber*EEPROMPACKETSIZE)+63)+EEPROMPACKETSIZE; ++i){
		EEPROM.write(i,radioData[j]);
			//Serial.print(j);
		//Serial.print(":");
		//Serial.println(radioData[j]);
		++j;
		
	}
}
long getinterval(byte timer){
	//interval info is stored in block 59
	int location = (59*EEPROMPACKETSIZE)+63+(timer*2);
	//Serial.print(timer);
	//Serial.print(" : ");
	//Serial.println((EEPROM.read(location)*60000)+(EEPROM.read(location+1)*100));
	return ((EEPROM.read(location)*60000)+(EEPROM.read(location+1)*250));
//return 1000;
}
	