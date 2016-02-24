#include <OneWire.h>
#include <DallasTemperature.h>

#define TARGET_TEMP 65
#define PID_INTERVAL 15000 // interval in milliseconds
#define TEMP_INTERVAL 1000 // interval at which to read temp (milliseconds)

#define ONE_WIRE_BUS 3
#define RELAY_PIN  8
#define I2C_SLAVE_ADDRESS 0x04

unsigned long pastTemp, pastPID;

OneWire ds(ONE_WIRE_BUS);  // Setup a oneWire instance to communicate with any OneWire devices
byte temperatureAddress[8];

class PID {
  public:
    float Kp = 3000;
    float Ki = .0025;
    float Kd = 300000000;

    float input, output;
    float error, pastError, errorSum;
    unsigned long pastTime, currentTime;
    float proportional, integral, derivative;

    float initialIntegralError = 1700;   // Initialize controller integral error in msec
    float integratorRange = 1;     // The maximum error allowable for integrator to be active

    PID() {
      input = output = 0;
      error = pastError = 0;
      errorSum = initialIntegralError / Ki;
      pastTime = currentTime = 0;
      proportional = integral = derivative = 0;
      pastTime = currentTime = 0;
    }

    void calculatePID() { // make into class with static variables in future
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
};

PID myPID = PID();

void printData() {
  Serial.print( ((myPID.currentTime/1000)%60)/100.0 + myPID.currentTime/60000);
  Serial.print("\t");
  Serial.print(myPID.currentTime/1000);
  Serial.print("\t");
  Serial.print(myPID.input);
  Serial.print("\t");
  Serial.print(myPID.output/1000);
  Serial.print("\t");
  Serial.print(myPID.proportional/1000);
  Serial.print("\t");
  Serial.print(myPID.integral/1000);
  Serial.print("\t");
  Serial.print(myPID.derivative/1000);
  Serial.print("\n");
}

void send_for_temp(byte temperatureAddress[8]) {
  ds.reset();
  ds.select(temperatureAddress);
  ds.write(0x44,1);         // start conversion, with parasite power on at the end
}

void read_temp(byte temperatureAddress[8]) {
  byte data[12];

  ds.reset();
  ds.select(temperatureAddress);
  ds.write(0xBE);         // Read Scratchpad

  for (byte i = 0; i < 9; i++) {           // we need 9 bytes
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

  Tc_100 = (6 * TReading) + TReading / 4;  // multiply by (100 * 0.0625) or 6.25
  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  Fract = Tc_100 % 100;

  myPID.input = (float) Tc_100 / 100;
  myPID.input = myPID.input * 1.8 + 32;
}

void setup(void) {
  Serial.begin(9600);
  pinMode(RELAY_PIN, OUTPUT); // Set relay pin 8 to output pin

  ds.search(temperatureAddress);  // Set up temperature
  send_for_temp(temperatureAddress);  // Get temperature
}

void loop(void) {
  if (millis() - pastTemp > TEMP_INTERVAL) {  // Update temp every interval
    pastTemp = millis();
    read_temp(temperatureAddress);  // Get temp from device
    send_for_temp(temperatureAddress);  // Ask device to update temperature for next read
  }

  if(millis() - pastPID > PID_INTERVAL) { // Run PID every interval
    pastPID = pastPID + PID_INTERVAL;
    myPID.calculatePID();
    printData();
  }
  if(millis() - pastPID < myPID.output)
    digitalWrite(RELAY_PIN,HIGH);
  else
    digitalWrite(RELAY_PIN,LOW);
}
