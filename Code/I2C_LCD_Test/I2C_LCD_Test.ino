// Simple Hello world demo using H/W SPI
#include <SPI.h>
#include <SSD1306_text.h>

long randNumber1;
long randNumber2;

SSD1306_text oled(0);

//------------------------------------------------------------------------------
void setup() {
  // Initialize, optionally clear the screen
    oled.init();
    oled.clear();                 // clear screen
    // Hello world - single sized character at row 0, pixel 0    
    oled.write("Hello world!");

    randomSeed(analogRead(0));
}
//------------------------------------------------------------------------------

void loop() {

  delay(1000);
  oled.clear();                 // clear screen
    
// Scaled characters, extra spacing
    oled.setCursor(3, 10);        // move cursor to row 3, pixel column 10
    oled.setTextSize(3, 8);       // 3X character size, spacing 8 pixels
    oled.write("Abc");

// Use print()
    float floatVal = 23.792;
    oled.setCursor(6,40);
    oled.setTextSize(2,1);
    oled.print(floatVal,3);

    delay(5000);
    
// Pseudo-graphics: Draw a box using direct writes  
    for(int n=0;n<100;n++)
    {
      randNumber1 = random(1, 100);
      randNumber2 = random(1, 40);
     
      oled.setCursor(randNumber2, randNumber1);
      oled.sendData(0xFF);
      for (int i=0; i<14; i++) oled.sendData(0x01);
      oled.sendData(0xFF);
      oled.setCursor(randNumber2+1,randNumber1);
      oled.sendData(0xFF);
      for (int i=0; i<14; i++) oled.sendData(0x80);
      oled.sendData(0xFF);
      delay(40);
    }

    
    
    

    for(int n=0;n<10;n++)
    {
      oled.clear();                 // clear screen
      // Scaled characters, extra spacing
      oled.setCursor(1, 10);        // move cursor to row 3, pixel column 10
      oled.setTextSize(4, 6);       // 4X character size, spacing 6pixels
      oled.write("LCD");
    
      randNumber1 = random(-1000, 1000);
    // Use print()
      floatVal = randNumber1/3.14159;
      oled.setCursor(6,40);
      oled.setTextSize(2,1);
      oled.print(floatVal,3);
      delay(500);
    }
    delay(500);
}
