#include <OneWire.h>
#include <DallasTemperature.h>

#define TARGET_TEMP 135
#define PERIOD 15000

#define ONE_WIRE_BUS 3
#define RELAY_PIN  8
#define I2C_SLAVE_ADDRESS 0x04

static float initialIntegralError = 1700;   // Initialize controller integral error in msec
static float integratorRange = 1;     // The maximum error allowable for integrator to be active

OneWire oneWire(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.
DeviceAddress thermoAddress = { 0x28, 0xFF, 0xE3, 0xC8, 0x64, 0x15, 0x02, 0x6D }; // Setup themometer address

OneWire ds(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices
//DeviceAddress thermoAddress = { 0x28, 0xFF, 0xE3, 0xC8, 0x64, 0x15, 0x02, 0x6D }; // Setup themometer address
float temp_0[2];
byte ds_addr0[8];
long previousMillis = 0;        // will store last time DS was updated
long interval = 1000;           // interval at which to read temp (milliseconds)

float Kp = 3000;
float Ki = .0025;
float Kd = 300000000;

float input, output;
float error, pastError, errorSum;
unsigned long pastTime, currentTime;

float proportional, integral, derivative;

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

void send_for_temp(byte addr[8]) {
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
}

void read_temp(byte addr[8], int number) {
  byte i;
  byte data[12];

  ds.reset();
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  for (i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;

  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit

  if (SignBit) { // negative
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }

  Tc_100 = (6 * TReading) + TReading / 4;
  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  Fract = Tc_100 % 100;

 switch (number){
  case 0:
    temp_0[0] = Whole;
    temp_0[1] = Fract;
    break;
  default:
    break;
  }

  temp_0[0] = (float) Tc_100 / 100;
  temp_0[0] = temp_0[0] * 1.8 + 32;
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

void setup(void)
{
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT); // Set relay pin 8 to output pin

  ds.search(ds_addr0);
  send_for_temp(ds_addr0);

  sensors.begin();  // Start up the library
  sensors.setResolution(thermoAddress, 12); // Set the resolution to 10 bit (good enough?)

  input = output = 0;
  error = pastError = 0;
  errorSum = initialIntegralError / Ki;
  pastTime = currentTime = 0;
  proportional = integral = derivative = 0;
}

void loop(void)
{/*
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
  }*/

  if (millis() - previousMillis > interval) {  // OVERFLOW????
    previousMillis = millis();
    //reading data from old requests:
    read_temp(ds_addr0, 0);
    //sending new requests:
    send_for_temp(ds_addr0);
    Serial.print(temp_0[0]);
    //Serial.print(".");
    // Serial.print(temp_0[1]);
    Serial.print("\t");
    Serial.print(getTemp(thermoAddress));
    Serial.print("\n");
  }
}
