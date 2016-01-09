#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>

#define TARGET_TEMP 140
#define PERIOD 15000

#define ONE_WIRE_BUS 3
#define RELAY_PIN  8
#define I2C_SLAVE_ADDRESS 0x04

static float initialIntegralError = 3700;   // Initialize controller integral error in msec
static float integratorRange = 1;     // The maximum error allowable for integrator to be active


OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature. 
DeviceAddress thermoAddress = { 0x28, 0xFF, 0xE3, 0xC8, 0x64, 0x15, 0x02, 0x6D }; // Setup themometer address

float Kp = 3000;
float Ki = .0025;
float Kd = 300000000;

float input, output;
float error, pastError, errorSum;
unsigned long pastTime, currentTime;

float proportional, integral, derivative;

void setup(void)
{
  Serial.begin(9600);
  Wire.begin(I2C_SLAVE_ADDRESS);    // I2C Set up Arduino as slave
  sensors.begin();  // Start up the library
  sensors.setResolution(thermoAddress, 12); // Set the resolution to 10 bit (good enough?)

  pinMode(RELAY_PIN, OUTPUT); // Set relay pin 8 to output pin

  input = output = 0;
  error = pastError = 0;
  errorSum = initialIntegralError / Ki;
  pastTime = currentTime = 0;
  proportional = integral = derivative = 0;
}

float calculatePID()
{
  // PID setup
  pastTime = currentTime;
  currentTime = millis();
  pastError = error;
  error = TARGET_TEMP - input;

  proportional = Kp * error;    // calculate the proportional term

  if( (error >= (-1)*integratorRange) && (error <= integratorRange) ) {  // Only sum integrator term if error is within range
    errorSum = errorSum + error * ( currentTime - pastTime );
  }
  integral = Ki * errorSum;
  
  derivative = Kd * ((error - pastError) / (currentTime - pastTime));   // calculate the derivative term

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
  Serial.print( ((currentTime/1000)%60)/100.0 + currentTime/60000);
  Serial.print("\t");
  Serial.print(currentTime/1000);
  Serial.print("\t");
  Serial.print(input);
  Serial.print("\t");
  Serial.print(output/1000);
  Serial.print("\t");
  Serial.print(proportional/1000);
  Serial.print("\t");
  Serial.print(integral/1000);
  Serial.print("\t");
  Serial.print(derivative/1000);
  Serial.print("\n");
}

void loop(void)
{
  input = getTemp(thermoAddress);
  calculatePID();
  printData();

  if(output < 0) {
    digitalWrite(RELAY_PIN, LOW);
    delay(PERIOD);
  } else if (output >= 0 && output <= PERIOD) {
    digitalWrite(RELAY_PIN, HIGH);
    delay(output);
    digitalWrite(RELAY_PIN, LOW);
    delay(PERIOD - output);
  } else if (output > PERIOD) {
    digitalWrite(RELAY_PIN, HIGH);
    delay(PERIOD);
    //digitalWrite(RELAY_PIN, LOW);   // This comment out is to prevent switching too frequently on high
  }
}
