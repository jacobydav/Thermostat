/* Convert RF signal into bits (temperature sensor version) 
 * Written by : Ray Wang (Rayshobby LLC)
 * http://rayshobby.net/?p=8827
 */
#include <EEPROM.h>

#include <SPI.h>
#include <SSD1306_text.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

//LCD begin
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//0 means there is no reset pin used
SSD1306_text oled(0);
int intervalLCDUpdate = 10000;  //update the LCD every x milliseconds
unsigned long lastLCDUpdate_ms = 0;   //when the LCD was last updated
bool forceLCDUpdate = true;    //Used to update the LCD immediately when a value changes such as the temperature.
//LCD end

//RF Receiver begin
// ring buffer size has to be large enough to fit
// data between two successive sync signals
#define RING_BUFFER_SIZE  256

#define SYNC_LENGTH  9000
#define SEP_LENGTH   500
#define BIT1_LENGTH  4000
#define BIT0_LENGTH  2000

#define DATAPIN  3  // D3 is interrupt 1

unsigned long timings[RING_BUFFER_SIZE];
unsigned int syncIndex1 = 0;  // index of the first sync signal
unsigned int syncIndex2 = 0;  // index of the second sync signal
bool received = false;
//RF Receiver end

//Temperature decision variables begin
#define TEM_VAL_BUFFER_SIZE  4        //how many temperature readings to store
int temVals[TEM_VAL_BUFFER_SIZE]; //array to hold the previous temperature readings
int currTemValInd = 0;            //current value in the buffer of temperature readings
int minOnTime_ms = 10*60*1000;     //5 minutes in milliseconds, min time to run heater
int minOffTime_ms = 10*60*1000;     //5 minutes in milliseconds, min time between running heater
int maxOnTime_ms = 30*60*1000;    //30 minutes in milliseconds, max time to run 
bool isHeatOn = false;
byte desiredTem = 74;             //The desired temperature. Will be stored in EEPROM when the user updates the value.
int desiredTemEEPROMAddress = 0;
byte desiredTemMin = 55;
byte desiredTemMax = 80;
int desiredTemUpPin = 7;          //The input that the desired temp up button is connected to.
int desiredTemDownPin = 9;          //The input that the desired temp down button is connected to.
bool temUpPressed = false;        //used to prevent multiple readings of the same button press
bool temDownPressed = false;        //used to prevent multiple readings of the same button press
unsigned long heatStartTime = 0; //The time when the heater was turned on;
unsigned long heatStopTime = 0;   //The time when the heater was turned off;
float currAvgTemp = 0;    //The current average temperature.
float desiredTemTol = 2.0;   //The tolerance used with the desired Temperature
unsigned long heatOffTime = 0;
unsigned long heatOnTime = 0;
//Temperature decision variables end

//Relay begin
int relayOutPin = 5;
//Relay end

// detect if a sync signal is present
bool isSync(unsigned int idx) {
  unsigned long t0 = timings[(idx+RING_BUFFER_SIZE-1) % RING_BUFFER_SIZE];
  unsigned long t1 = timings[idx];

  // on the temperature sensor, the sync signal
  // is roughtly 9.0ms. Accounting for error
  // it should be within 8.0ms and 10.0ms
  if (t0>(SEP_LENGTH-100) && t0<(SEP_LENGTH+100) &&
    t1>(SYNC_LENGTH-1000) && t1<(SYNC_LENGTH+1000) &&
    digitalRead(DATAPIN) == HIGH) {
    return true;
  }
  return false;
}

// Interrupt 1 handler
//For input from RF receiver (temperature sensor)
void handler() {
  static unsigned long duration = 0;
  static unsigned long lastTime = 0;
  static unsigned int ringIndex = 0;
  static unsigned int syncCount = 0;

  // ignore if we haven't processed the previous received signal
  if (received == true) {
    return;
  }
  // calculating timing since last change
  long time = micros();
  duration = time - lastTime;
  lastTime = time;

  // store data in ring buffer
  ringIndex = (ringIndex + 1) % RING_BUFFER_SIZE;
  timings[ringIndex] = duration;

  // detect sync signal
  if (isSync(ringIndex)) {
    syncCount ++;
    // first time sync is seen, record buffer index
    if (syncCount == 1) {
      syncIndex1 = (ringIndex+1) % RING_BUFFER_SIZE;
    } 
    else if (syncCount == 2) {
      // second time sync is seen, start bit conversion
      syncCount = 0;
      syncIndex2 = (ringIndex+1) % RING_BUFFER_SIZE;
      unsigned int changeCount = (syncIndex2 < syncIndex1) ? (syncIndex2+RING_BUFFER_SIZE - syncIndex1) : (syncIndex2 - syncIndex1);
      // changeCount must be 66 -- 32 bits x 2 + 2 for sync
      if (changeCount != 66) {
        received = false;
        syncIndex1 = 0;
        syncIndex2 = 0;
      } 
      else {
        received = true;
      }
    }

  }
}


void setup() {
  Serial.begin(9600);
  Serial.println("Started.");
  //Desired temperature buttons begin
  pinMode(desiredTemUpPin, INPUT_PULLUP);    // desired temp up button
  pinMode(desiredTemDownPin, INPUT_PULLUP);    // desired temp down button
  //Relay output begin
  pinMode(relayOutPin, OUTPUT);
  //get the desired temperature from EEPROM
  desiredTem = EEPROM.read(desiredTemEEPROMAddress);
  //If the value is not valid, start at 68 degrees
  if(desiredTem<desiredTemMin || desiredTem>desiredTemMax)
  {    
    desiredTem = 68;
    EEPROM.write(desiredTemEEPROMAddress, desiredTem);
  }
  //Desired temperature buttons end
  //Temperature sensor begin
  pinMode(3, INPUT);
  attachInterrupt(1, handler, CHANGE);
  //Temperature sensor end
  //initialize the values in the temperature value array
  for(int i=0;i<TEM_VAL_BUFFER_SIZE;i++)
    temVals[i]=-100;
  
  //LCD begin
  // Initialize, optionally clear the screen
  oled.init();
  oled.clear();                 // clear screen
  /*
  for(int n=0;n<100;n++)
  {
    long randNumber1 = random(1, 100);
    long randNumber2 = random(1, 40);
   
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
  oled.clear();                 // clear screen
  */
  //LCD end
}

void loop() 
{
  //**********Desired Temperature Up/Down Begin*****************
  //Check for Desired Temperature Up or Down button
  //If either is pressed then we will enter into "adjust desired temp mode" for a few seconds  
  if(digitalRead(desiredTemUpPin)==0 || digitalRead(desiredTemDownPin)==0)
  {
    updateDesTemp();    
  }  
  //**********Desired Temperature Up/Down End*****************
  //If temp sensor interrupt occured.
  if (received == true) {
    // disable interrupt to avoid new data corrupting the buffer
    detachInterrupt(1);
    /*
    // loop over buffer data
    for(unsigned int i=syncIndex1; i!=syncIndex2; i=(i+2)%RING_BUFFER_SIZE) {
      unsigned long t0 = timings[i], t1 = timings[(i+1)%RING_BUFFER_SIZE];
      if (t0>(SEP_LENGTH-100) && t0<(SEP_LENGTH+100)) {
       if (t1>(BIT1_LENGTH-1000) && t1<(BIT1_LENGTH+1000)) {
         Serial.print("1");
       } else if (t1>(BIT0_LENGTH-1000) && t1<(BIT0_LENGTH+1000)) {
         Serial.print("0");
       } else {
         Serial.print("SYNC");  // sync signal
       }
       } else {
       Serial.print("?");  // undefined timing
       }
    }
    Serial.println("");
    */
    // loop over the lowest 12 bits of the middle 2 bytes
    unsigned long temp = 0;
    bool negative = false;
    bool fail = false;
    for(unsigned int i=(syncIndex1+24)%RING_BUFFER_SIZE; i!=(syncIndex1+48)%RING_BUFFER_SIZE; i=(i+2)%RING_BUFFER_SIZE) {
      unsigned long t0 = timings[i], t1 = timings[(i+1)%RING_BUFFER_SIZE];
      if (t0>(SEP_LENGTH-100) && t0<(SEP_LENGTH+100)) {
        if (t1>(BIT1_LENGTH-1000) && t1<(BIT1_LENGTH+1000)) {
          if(i == (syncIndex1+24)%RING_BUFFER_SIZE) negative = true;
          temp = (temp << 1) + 1;
        } 
        else if (t1>(BIT0_LENGTH-1000) && t1<(BIT0_LENGTH+1000)) {
          temp = (temp << 1) + 0;
        } 
        else {
          fail = true;
        }
      } 
      else {
        fail = true;
      }
    }


    if (!fail) {
      if (negative) {
        temp = 4096 - temp; 
        Serial.print("-");
      }
      Serial.print((temp+5)/10);  // round to the nearest integer
      Serial.write(176);    // degree symbol
      Serial.print("C/");
      Serial.print((temp+5)*9/50+32);  // convert to F
      Serial.write(176);    // degree symbol
      Serial.println("F");
      newTemperatureReading((temp+5)*9/50+32);
    } else {
      Serial.println(F("Decoding error."));
    } 
    // delay for 1 second to avoid repetitions
    delay(1000);
    received = false;
    syncIndex1 = 0;
    syncIndex2 = 0;

    // re-enable interrupt
    attachInterrupt(1, handler, CHANGE);

    //Force the LCD to update
    forceLCDUpdate=true;
  }
  else
  {
    delay(100);
  }
  //update the display
  updateLCD(0);
}

//When a new temperature reading is available
void newTemperatureReading(int newTemVal)
{
  bool isBuffFull = true;
  int buffTotal = 0;
  double buffAvgDiff = 0;
  //temVals is a circular buffer so here we check if we
  //need to go back to beginning of buffer
  if(currTemValInd>=TEM_VAL_BUFFER_SIZE)
    currTemValInd=0;
  
  //We will not make a decision about turning heater on and off until the buffer is full
  //check if buffer is filled with values by checking for default initial values
  for(int i=0;i<TEM_VAL_BUFFER_SIZE;i++)
  {
    if(temVals[i]==-100)
      isBuffFull=false;
  }
  //If the buffer is NOT full, then we can add the new val to buffer and end the routine.
  if(isBuffFull==false)
  {
    Serial.println(F("Buffer not full yet"));
    //add the new value to the buffer
    temVals[currTemValInd] = newTemVal;
    currTemValInd++;
    return;
  }
  //Check that the new temperature value is valid
  //we assume the temperature should not change by more than 5 degrees from the
  //average value in the buffer
  //only do this once we have a full buffer of values
  //Get the average value
  for(int i=0;i<TEM_VAL_BUFFER_SIZE;i++)
  {
    buffTotal = buffTotal + temVals[i];
  }
  currAvgTemp = buffTotal/TEM_VAL_BUFFER_SIZE;
  
  //Get the difference between the buffer average and the new value
  buffAvgDiff = abs(currAvgTemp-newTemVal);
  Serial.print(F("currAvgTemp="));
  Serial.println(currAvgTemp);
  Serial.print(F("buffAvgDiff="));
  Serial.println(buffAvgDiff);
  //check if it is within 5 degrees
  //if YES then update the buffer and advance the buffer index
  if(buffAvgDiff<=5)
  {
    if(isHeatOn==true)
    {
      heatOnTime = millis()-heatStartTime;
      heatOffTime = 0;
    }
    else
    {
      heatOnTime = 0;
      heatOffTime = millis()-heatStopTime;
    }
    //update the buffer
    temVals[currTemValInd]=newTemVal;
    currTemValInd++;
    //make a decision about turning the heater on or off
    //if it is colder than the desired temperature
    float desiredTemMin = desiredTem-desiredTemTol;
    Serial.print(F("desiredTem-desiredTemTol = "));
    Serial.println(desiredTemMin);
    float desiredTemMax = desiredTem+desiredTemTol;
    Serial.print(F("desiredTem+desiredTemTol = "));
    Serial.println(desiredTemMax);
    if(currAvgTemp<desiredTemMin)
    {
      if(isHeatOn==false)
      {        
        Serial.print(F("heatOffTime = "));
        Serial.println(heatOffTime);
        if(heatOffTime>minOffTime_ms)
        {
          isHeatOn=true;
          heatStartTime = millis();
          //turn heater on
          Serial.println(F("Turning heater on"));
        }
      }
      else
      {
        //If heat is already on, check how long it has been on
        //If it has been on for longer than the max time, then turn it off.
        if(heatOnTime>maxOnTime_ms)
        {
          //turn heater off
          Serial.println(F("Turning heater off because maxOnTime was exceded"));
          isHeatOn=false;
          heatStopTime = millis(); 
        }
      }
    }
    else if(currAvgTemp>desiredTemMax)
    {
      //if we are at desired temperature and heater is on, then turn heater off      
      if(isHeatOn==true)
      {
        Serial.print(F("Heat on time = "));
         Serial.println(heatOnTime);
        //Only turn off the heater if it has been on for the minimum time.
        if(heatOnTime>minOnTime_ms)
        {
          //turn heater off
          Serial.println(F("Turning heater off"));
          isHeatOn=false;
          heatStopTime = millis();
        }
        else
        {
          Serial.println(F("Not turning heater off because minOnTime_ms was not reached"));
        }
      }
    }
  }
  else
  {
    //if the new value is not valid, don't advance the buffer index
    Serial.println(F("Value is out of tolerance."));
  }
}

//update desired temperature
//In this mode we will monitor pushbutton states for temperature up/down.
//Interrupts from the Temperature sensor will be disabled.
//The LCD will be updated quickly
void updateDesTemp()
{  
  Serial.println(F("Entering updateDesTemp"));
  // disable interrupt to avoid new data corrupting the buffer
  detachInterrupt(1);
  //Use the index of the for loop to keep track of when the last press occurred.
  int lastUpPress=0;
  int lastDownPress=0;
  int minPressInterval = 10;

  for(int i=0;i<1000;i++)
  {
    if(digitalRead(desiredTemUpPin)==0)
    {
      //Check that the value is below max and we meet the debounce interval
      if(desiredTem<desiredTemMax && i-lastUpPress>minPressInterval)
      {
        Serial.println(F("Temp Up"));
        lastUpPress=i;
        desiredTem=desiredTem+1;
        //Force the LCD to update
        forceLCDUpdate=true;
        updateLCD(1);
      }
    }

    if(digitalRead(desiredTemDownPin)==0)
    {
      //Check that the value is below min and we meet the debounce interval
      if(desiredTem>desiredTemMin && i-lastDownPress>minPressInterval)
      {
        Serial.println(F("Temp Down"));
        lastDownPress=i;
        desiredTem=desiredTem-1;
        //Force the LCD to update
        forceLCDUpdate=true;
        updateLCD(1);
      }
    }
        
    //Delay
    delay(10);         
  }
  

  //Update temp in memory
  EEPROM.write(desiredTemEEPROMAddress, desiredTem);

  // re-enable interrupt
  attachInterrupt(1, handler, CHANGE);

  //Testing begin
  digitalWrite(relayOutPin,LOW);
  //TEsting end

  Serial.println(F("Exiting updateDesTemp"));
}




//Update the data on the display
//dispMode controls what is displayed on the screen
//dispMode=0 is default readout
//dispMode=1 is updateDesTemp mode.
void updateLCD(int dispMode)
{
  unsigned long timeSinceLastUpdate = millis()-lastLCDUpdate_ms;
  //LCD is updated on regular intervals. It can also be forced to update using the forceLCDUpdate flag
  if(timeSinceLastUpdate>intervalLCDUpdate || forceLCDUpdate==true)
  {
    //Clear the force update flag
    forceLCDUpdate=false;
    
    oled.clear();
    //Current temp label
    oled.setCursor(0, 10);        // cursor row , pixel column 
    oled.setTextSize(1, 3);       // character size (x), spacing (pixels)
    //In default mode: show the label "Temp"
    //In updateDesTemp mode: show the label "Update"
    if(dispMode==0)
    {
      oled.write("Temp");
    }
    else if(dispMode==1)
    {
      oled.write("Update");
    }
    //Desired temp label
    oled.setCursor(0, 100);        
    oled.setTextSize(1, 3);       
    oled.write("Set");
    //Current temp value
    oled.setCursor(2, 10);        
    oled.setTextSize(2, 3); 
    //In default mode, show the curr temp
    if(dispMode==0)
    {
      oled.print(currAvgTemp,0);
    }
    //Set temp value
    oled.setCursor(2, 90);        
    oled.setTextSize(2, 3);
    float desTemInt = desiredTem;
    oled.print(desTemInt,0);
    //heat on
    oled.setCursor(4, 50);        
    oled.setTextSize(1, 3); 
    if(isHeatOn == true)
      oled.write("ON");
    else
      oled.write("OFF");
    //heat on time
    oled.setCursor(6, 10);        
    oled.setTextSize(1, 3); 
    oled.print(heatOnTime/60000);
    //heat off time
    oled.setCursor(6, 90);        
    oled.setTextSize(1, 3); 
    oled.print(heatOffTime/60000);
    
    lastLCDUpdate_ms=millis();
  }
}
