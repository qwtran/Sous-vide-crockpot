#include <OneWire.h>

#define TARGET_TEMP 135
#define PERIOD 15000

#define ONE_WIRE_BUS 3
#define RELAY_PIN  8
#define I2C_SLAVE_ADDRESS 0x04

static float initialIntegralError = 1700;   // Initialize controller integral error in msec
static float integratorRange = 1;     // The maximum error allowable for integrator to be active

OneWire ds(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices
DeviceAddress thermoAddress = { 0x28, 0xFF, 0xE3, 0xC8, 0x64, 0x15, 0x02, 0x6D }; // Setup themometer address
int temp_0[2];
long previousMillis = 0;        // will store last time DS was updated
long interval = 1000;           // interval at which to read temp (milliseconds)

float Kp = 3000;
float Ki = .0025;
float Kd = 300000000;

float input, output;
float error, pastError, errorSum;
unsigned long pastTime, currentTime;

float proportional, integral, derivative;

byte Data[8];
volatile byte* Float1ArrayPtr;
volatile byte* Float2ArrayPtr;

void setup(void)
{
  Serial.begin(9600);
  Wire.begin(I2C_SLAVE_ADDRESS);    // I2C Set up Arduino as slave
  sensors.begin();  // Start up the library
  sensors.setResolution(thermoAddress, 12); // Set the resolution to 10 bit (good enough?)

  Wire.onRequest(sendData);
  Wire.onReceive(receiveData);

  pinMode(RELAY_PIN, OUTPUT); // Set relay pin 8 to output pin

  input = output = 0;
  error = pastError = 0;
  errorSum = initialIntegralError / Ki;
  pastTime = currentTime = 0;
  proportional = integral = derivative = 0;
}


void sendData() {
  Float1ArrayPtr = (byte*) &currentTime;
  Data[0] = Float1ArrayPtr[0];
  Data[1] = Float1ArrayPtr[1];
  Data[2] = Float1ArrayPtr[2];
  Data[3] = Float1ArrayPtr[3];
  Float2ArrayPtr = (byte*) &input;
  Data[4] = Float2ArrayPtr[0];
  Data[5] = Float2ArrayPtr[1];
  Data[6] = Float2ArrayPtr[2];
  Data[7] = Float2ArrayPtr[3];
  Wire.write(Data, 8);
  Serial.println("Data Reqested!");
}

void receiveData(int num) {
  Serial.println("Received Data!");
  while (Wire.available() > 0) {
    byte i = Wire.read();
    Serial.println(i);
  }
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

  Tc_100 = 50 * TReading; // multiply by 100 * 0.5
  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  Fract = Tc_100 % 100;

 switch (number){
  case 0:
    temp_0[0] = Whole;
    temp_0[1] = Fract;
    break;
  case 1:
    temp_1[0] = Whole;
    temp_1[1] = Fract;
    break;
  default:
    break;
  }
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
