#include <OneWire.h>
#include <DallasTemperature.h>

#define TARGET_TEMP 130
#define PERIOD 15000

#define ONE_WIRE_BUS 3
#define RELAY_PIN  8

static float initialIntegralError = 0;   // Initialize controller integral error in seconds
static float integratorRange = 2;     // The maximum error allowable for integrator to be active


OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature. 
DeviceAddress thermoAddress = { 0x28, 0xFF, 0xE3, 0xC8, 0x64, 0x15, 0x02, 0x6D }; // Setup themometer address

float Kp = 3000;
float Ki = .005;
float Kd = 0;

float input, output;
float error, pastError, errorSum;
unsigned long pastTime, currentTime;


void setup(void)
{
  Serial.begin(9600);
  sensors.begin();  // Start up the library
  sensors.setResolution(thermoAddress, 12); // Set the resolution to 10 bit (good enough?)

  pinMode(RELAY_PIN, OUTPUT); // Set relay pin 8 to output pin

  input = output = 0;
  error = pastError = 0;
  errorSum = initialIntegralError / Ki;
  pastTime = currentTime = 0;
}

float calculatePID()
{
  float proportional, integral, derivative;

  // Setup
  pastTime = currentTime;
  currentTime = millis();
  pastError = error;
  error = TARGET_TEMP - input;

  proportional = Kp * error;    // calculate the proportional term

  if( (error >= (-1)*integratorRange) && (error <= integratorRange) ) {  // Only sum integrator term if error is within range
    errorSum = errorSum + error * ( currentTime - pastTime );
  }
  integral = Ki * errorSum;
  
  derivative = Kd * ( error - pastError) / (currentTime - pastTime );   // calculate the derivative term

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
  Serial.print(output/1000);
  Serial.print("\n");
}

void loop(void)
{
  input = getTemp(thermoAddress);
  calculatePID();
  printData();

  digitalWrite(RELAY_PIN, HIGH);

   //VN code
   //off_time = PERIOD - output
   //if off_time < 0
   //turn off
   //delay(PERIOD)
   //else if off_time between 0 and PERIOD
   //delay(output)
   //turn off
   //else
   //delay(PERIOD)
   //turn off
   //end

  if(output < 0)
  {
    delay(0);
  } else if(output > PERIOD)
  {
    delay(PERIOD);
  } else
  {
    delay(output);
  }

  digitalWrite(RELAY_PIN, LOW);

  if(PERIOD - output < 0)
  {
    delay(0);
  } else if(PERIOD - output > PERIOD)
  {
    delay(PERIOD);
  } else
  {
    delay(PERIOD - output);
  }
}

