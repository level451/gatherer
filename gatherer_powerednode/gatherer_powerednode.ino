#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip = Adafruit_NeoPixel(6,15, NEO_GRB + NEO_KHZ800);
void setup(){
Serial.begin(115200);
Serial.print("test");
 strip.begin();
 for (int i=0; i < 6; i++) {
strip.setPixelColor(i,strip.Color(1,1,1));
 }
  strip.show();
}
void loop(){
Serial.print("test - loop");
delay(1000);
}
