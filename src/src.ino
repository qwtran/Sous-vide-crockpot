#include <OneWire.h>
#include <DallasTemperature.h>

#define TARGET_TEMP 165
#define ONE_WIRE_BUS 3
#define RELAY_PIN  8
#define DELAY 15000

OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature. 
DeviceAddress thermoAddress = { 0x28, 0xFF, 0xE3, 0xC8, 0x64, 0x15, 0x02, 0x6D }; // Setup themometer address

boolean relaySwitchOn;

float Kp;
float Ki = 0;
float Kd = 0;

float input, output;
float error, pastError, errorSum;
unsigned long pastTime, currentTime;


void setup(void)
{
  Serial.begin(9600);
  sensors.begin();  // Start up the library
  sensors.setResolution(thermoAddress, 10); // Set the resolution to 10 bit (good enough?)

  pinMode(RELAY_PIN, OUTPUT); // Set relay pin 8 to output pin
  digitalWrite(RELAY_PIN, HIGH);
  relaySwitchOn = true;

  pastError = errorSum = 0;
}

float calculatePID()
{
  float proportional, integral, derivative;

  // Setup
  pastTime = currentTime;
  currentTime = millis();
  pastError = error;
  error = TARGET_TEMP - input;

  proportional = Kp * error;

  errorSum = errorSum + error * ( currentTime - pastTime );
  integral = Ki * errorSum;
  
  derivative = Kd * ( error - pastError) / (currentTime - pastTime );

  output = proportional + integral + derivative;
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

void printData()
{
  Serial.print(currentTime/1000);
  Serial.print("\t");
  Serial.print(input);
  Serial.print("\t");
  Serial.print(relaySwitchOn);
  Serial.print("\t");
  Serial.print(output);
  Serial.print("\n");
}

void loop(void)
{
  input = getTemp(thermoAddress);
  calculatePID();
  printData();

  if(input > TARGET_TEMP)
  {
    digitalWrite(RELAY_PIN, LOW);
    relaySwitchOn = false;
  }

  delay(DELAY);
}

