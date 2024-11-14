/************************************
    SCL-----A5   SDA-----A4
*************************************/
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "MPU6050.h"
#include <stdio.h>
#include <wiring_private.h>

/**********PIN ASSIGNMENTS**********/
#define BUFFER_SIZE 100
#define SENSOR_FORE_FINGER A0    //adc channel 0 used for forefinger, plugged into a0 pin
#define SENSOR_MIDDLE_FINGER A1  //adc channel 1 used for middle finger, plugged into a1 pin
#define SENSOR_RING_FINGER A2    //adc channel 2 used for ring finger, plugged into a2 pin
#define SENSOR_PINKIE_FINGER A3  //adc channel 3 used for pinkie finger, plugged into a3 pin
#define LED_PIN 13
#define SCL A5
#define SDA A4
#define RX 0
#define TX 1

MPU6050 mpu;

/**********ADC ASSIGNMENTS**********/
const float VCC = 5;                   // input voltage for the sensor
const float R_DIV = 10000.0;            // resistor used to create voltage divider 10K ohm
const float flatResistance = 25000.0;  // resistance value of flex sensor when flat 25k ohm
const float bendResistance = 100000.0; // resistance at 90 deg 100k ohm

// ADC Calibration variables
int sensorMinFore = 1023;
int sensorMaxFore = 0;

int sensorMinMiddle = 1023;
int sensorMaxMiddle = 0;

int sensorMinRing = 1023;
int sensorMaxRing = 0;

int sensorMinPinkie = 1023;
int sensorMaxPinkie = 0;

volatile int raw_fore_finger[BUFFER_SIZE] = {0}, raw_middle_finger[BUFFER_SIZE] = {0}, raw_ring_finger[BUFFER_SIZE] = {0}, raw_pinkie_finger[BUFFER_SIZE] = {0};

int fore_finger = 0;
int middle_finger = 0;
int ring_finger = 0;
int pinkie_finger = 0;

void averageADC(){
  int fore_finger_avg = 0;
  int middle_finger_avg = 0;
  int ring_finger_avg = 0;
  int pinkie_finger_avg = 0;

  for(int i = 0 ; i<BUFFER_SIZE ; i++){
    raw_fore_finger[i] = analogRead(SENSOR_FORE_FINGER);
    raw_middle_finger[i] = analogRead(SENSOR_MIDDLE_FINGER);
    raw_ring_finger[i] = analogRead(SENSOR_RING_FINGER);
    raw_pinkie_finger[i] = analogRead(SENSOR_PINKIE_FINGER);

    fore_finger_avg += fore_finger[i];
    middle_finger_avg += middle_finger[i];
    ring_finger_avg += ring_finger[i];
    pinkie_finger_avg += pinkie_finger[i];
  }

  fore_finger_avg = fore_finger_avg/BUFFER_SIZE;
  middle_finger_avg = middle_finger_avg/BUFFER_SIZE;
  ring_finger_avg = ring_finger_avg/BUFFER_SIZE;
  pinkie_finger_avg = pinkie_finger_avg/BUFFER_SIZE;

  fore_finger = fore_finger_avg;
  middle_finger = middle_finger_avg;
  ring_finger = ring_finger_avg;
  pinkie_finger = pinkie_finger_avg;
}

/**********FUNCTION TO CALIBRATE ADC INPUTS FOR FLEX SENSOR**********/
void calibrateADC(){
  averageADC();
  
  // Maximum values
  if(fore_finger>sensorMaxFore){
    sensorMaxFore = fore_finger;
  }
  if(middle_finger>sensorMaxMiddle){
    sensorMaxMiddle = middle_finger;
  }
  if(ring_finger>sensorMaxRing){
    sensorMaxRing = ring_finger;
  }
  if(pinkie_finger>sensorMaxPinkie){
    sensorMaxPinkie = pinkie_finger;
  }
  
  // Minimum values
  if(fore_finger<sensorMinFore){
    sensorMinFore = fore_finger;
  }
  if(middle_finger<sensorMinMiddle){
    sensorMinMiddle = middle_finger;
  }
  if(ring_finger<sensorMinRing){
    sensorMinRing = ring_finger;
  }
  if(pinkie_finger<sensorMinPinkie){
    sensorMinPinkie = pinkie_finger;
  }
}

/**********ACTUAL VALUES USING THE DATASHEET OF THE FLEX SENSOR**********/
float angle_flex(int value){
  float Vflex = value * (VCC/1023.0);
  float Rflex = R_DIV * (VCC / Vflex - 1.0);
  Serial.println("Resistance: " + String(Rflex) + " ohms");
  float angle = map(Rflex, flatResistance, bendResistance,0,90.0);

  return angle;
}

void checkSettings(){ 
  Serial.println();  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource()){
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  Serial.println();
}

void setup() {  
  // Led for signalling
  pinMode(LED_PIN,OUTPUT);

  // Set adc pins for input
  pinMode(SENSOR_FORE_FINGER, INPUT);
  pinMode(SENSOR_MIDDLE_FINGER, INPUT);
  pinMode(SENSOR_RING_FINGER, INPUT);
  pinMode(SENSOR_PINKIE_FINGER, INPUT);

  digitalWrite(LED_PIN,LOW);
  Serial.begin(9600);

  // Serial.println("Starting ADC Calibration..");
  
  // // Start calibration
  // while(millis()<5000){
  //   calibrateADC();
  // }

  // Serial.println("Ending ADC Calibration..");
  // Serial.println("Values: ");
  // char printstr[150];
  // snprintf(printstr,sizeof(printstr),"min fore:%d, max fore:%d\n min middle:%d max middle:%d\n min ring:%d max ring:%d\n min pinkie:%d max pinkie:%d\n",sensorMinFore,sensorMaxFore,sensorMinMiddle,sensorMaxMiddle,sensorMinRing,sensorMaxRing,sensorMinPinkie,sensorMaxPinkie);
  // Serial.println(printstr);
  // delay(500);

  // Ended calibration
  // digitalWrite(LED_PIN, HIGH);
 
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)){
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  checkSettings();
}

void checkMovement(Vector newMovement){
  // Motion Sensor implemented, flex sensors to be implemented
  static Vector lastMovement;

  // Currently based on motion sensor values alone, flex sensors needed for finger tracking
  if(newMovement.ZAxis > 9){
    Serial.println("Bye!");
  }else if(abs(newMovement.XAxis) > 7){
    Serial.println("Stop!");
  }else if((abs(lastMovement.ZAxis - newMovement.ZAxis) < 2) && (abs(newMovement.XAxis-newMovement.YAxis) > 10)){
    Serial.println("Hello!");
  }
}

void loop(){
  // averageADC();

  // fore_finger = map(fore_finger, sensorMinFore, sensorMaxFore, 1, 255);
  // middle_finger = map(middle_finger, sensorMinMiddle, sensorMaxMiddle, 1, 255);
  // ring_finger = map(ring_finger, sensorMinRing, sensorMaxRing, 1, 255);
  // pinkie_finger = map(pinkie_finger, sensorMinPinkie, sensorMaxPinkie, 1, 255);

  // delay(10);
  
  // float ff = angle_flex(fore_finger);
  // float mf = angle_flex(middle_finger);
  // float rf = angle_flex(ring_finger);
  // float pf = angle_flex(pinkie_finger);
  
  // MPU 6050 values
  Vector rawAccel = mpu.readRawAccel();
  Vector normAccel = mpu.readNormalizeAccel();

  // Function to Check movement and print output based on movement
  checkMovement(normAccel);

  delay(500);
}
