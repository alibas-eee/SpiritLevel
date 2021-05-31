//avrdude -c arduino -p t85 -P COM3 -b 19200 -U flash:w:tiny.hex


#include "TinyWireM.h"
#include "TinyOzOled.h"
#include "SimpleKalmanFilter.h"

#define RangePerDigit2G   .000061f
#define GRAVITY           9.80665f
#define BATTERY_VOLTAGE_PIN A2
#define RESET_BTN_PIN       3
struct Vector{
    float XAxis;
    float YAxis;
    float ZAxis;
};

Vector normAccel, tilt, defaultAccel, filteredAccel;
int accelX, accelY, accelZ;

SimpleKalmanFilter FilterX(1, 1, 0.5);
SimpleKalmanFilter FilterY(1, 1, 0.5);
SimpleKalmanFilter FilterZ(1, 1, 0.5);

char mpu = 0x68;  
OzOLED oled;

void setup() {

  analogReference(INTERNAL); //1.1V ref voltage
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  TinyWireM.begin();
  oled.init();
  oled.clearDisplay();  
  oled.setNormalDisplay();
  oled.sendCommand(0xA1);        // set Orientation
  oled.sendCommand(0xC8);
  TinyWireM.beginTransmission(mpu); 
  TinyWireM.write(0x6B); //  Power setting address
  TinyWireM.write(0b00000000); // Disable sleep mode (just in case)
  TinyWireM.endTransmission();
  TinyWireM.beginTransmission(mpu); 
  TinyWireM.write(0x1B); // Config register for Gyro
  TinyWireM.write(0x00000000); // 250° per second range (default)
  TinyWireM.endTransmission();
  TinyWireM.beginTransmission(mpu); //I2C address of the MPU
  TinyWireM.write(0x1C); // Accelerometer config register
  TinyWireM.write(0b00000000); // 2g range +/- (default)
  TinyWireM.endTransmission();
}
void loop() {
 getAccel();
 printTilt();
 printBattery(); 
 CheckResetValues();
 delay(100);
//oled.clearDisplay(); 
}

void CheckResetValues(){
  if(digitalRead(RESET_BTN_PIN) == 0){
    defaultAccel.XAxis = normAccel.XAxis;
    defaultAccel.YAxis = normAccel.YAxis;
    defaultAccel.ZAxis = normAccel.ZAxis;
  }  
}

void getAccel() {
  TinyWireM.beginTransmission(mpu); //I2C address of the MPU
  TinyWireM.write(0x3B); //  Acceleration data register
  TinyWireM.endTransmission();
  TinyWireM.requestFrom(mpu, 6); // Get 6 bytes, 2 for each DoF
  accelX = TinyWireM.read() << 8|TinyWireM.read(); 
  accelY = TinyWireM.read() << 8|TinyWireM.read();
  accelZ = TinyWireM.read() << 8|TinyWireM.read();

  normAccel.XAxis = accelX * RangePerDigit2G * GRAVITY;
  normAccel.YAxis = accelY * RangePerDigit2G * GRAVITY;
  normAccel.ZAxis = accelZ * RangePerDigit2G * GRAVITY;

  filteredAccel.XAxis = FilterX.updateEstimate(normAccel.XAxis) - defaultAccel.XAxis;  
  filteredAccel.YAxis = FilterY.updateEstimate(normAccel.YAxis) - defaultAccel.YAxis;
  filteredAccel.ZAxis = FilterZ.updateEstimate(normAccel.ZAxis) - defaultAccel.ZAxis;
}

void printTilt(){
  char tmp[10]; 
  
  tilt.XAxis = mapfloat(filteredAccel.XAxis, -10 , 10, -90, 90);
  tilt.YAxis = mapfloat(filteredAccel.YAxis, -10 , 10, -90, 90);
  tilt.ZAxis = mapfloat(filteredAccel.ZAxis, -10 , 10, -90, 90);    
  
  dtostrf(tilt.XAxis, 5, 2, tmp);    
  oled.printChar('X', 0, 2);
  oled.printBigNumber(tmp, 1, 0, 5);//row 4 col 3

  dtostrf(tilt.YAxis, 5, 2, tmp);
  oled.printChar('Y', 0, 6);
  oled.printBigNumber(tmp, 1, 4, 5);//row 4 col 3
}

//print battery indicator 8x8 pixel 
void printBattery(){
char battery[8] = {0xff, 0x81, 0x81, 0x81, 0x81, 0x81, 0xc3, 0x3c} ;

  float batADC = analogRead(BATTERY_VOLTAGE_PIN);
  batADC/=1024;
  float batVoltage = batADC * 1.1; //1,1v internal ref voltage
  batVoltage *=5;
  int batVal= ceil( mapfloat( batVoltage, 3.6, 4.2, 0, 5));
  
  oled.setCursorXY(0, 0);
  for(int i = 0; i< 8; i++){
    if(i > 0 &&  i < 6 && batVal > (i - 2)) 
      oled.sendData(battery[i] | 0x3C );
    else
      oled.sendData(battery[i]); 
  }  
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
