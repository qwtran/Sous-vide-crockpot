#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 3
#define RELAY_PIN  8
#define TARGET 165
#define DELAY 15000

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Setup themometer address
DeviceAddress thermometer = { 0x28, 0xFF, 0xE3, 0xC8, 0x64, 0x15, 0x02, 0x6D };

// Is the relay on counter
boolean relaySwitch;

void setup(void)
{
  // Start serial port
  Serial.begin(9600);
  
  // Start up the library
  sensors.begin();

  // Set the resolution to 10 bit (good enough?)
  sensors.setResolution(thermometer, 10);

  // Set relay pin 8 to output pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  relaySwitch = true;
}

float getTemp(DeviceAddress deviceAddress)
{
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    Serial.print("Error getting temperature");
  } else {
    return DallasTemperature::toFahrenheit(tempC);
  }
}

void printData(float temp)
{
  Serial.print(millis()/1000);
  Serial.print("\t");
  Serial.print(temp);
  Serial.print("\t");
  Serial.print(relaySwitch);
  Serial.print("\n");
}

void loop(void)
{
  float tempF = getTemp(thermometer);
  printData(tempF);

  if(tempF > TARGET)
  {
    digitalWrite(RELAY_PIN, LOW);
    relaySwitch = false;
  }

  delay(DELAY);
}

