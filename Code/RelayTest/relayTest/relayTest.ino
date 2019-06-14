//Test the 5 Volt relay by turning on and off
//a digital output

//Relay is connected to digital output 5
#define relayPin 5

void setup() 
{
  pinMode(relayPin, OUTPUT);
}

void loop() 
{
  digitalWrite(relayPin, HIGH);

  delay(5000);

  digitalWrite(relayPin, LOW);

  delay(5000);
}
