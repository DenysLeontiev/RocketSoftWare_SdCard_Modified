#include<Wire.h>
#include<Adafruit_MPU6050.h>
#include<Adafruit_Sensor.h>
#include<Adafruit_BMP085_U.h>
#include<Servo.h>
#include<SPI.h>
#include<SD.h>

Adafruit_MPU6050 mpuModule;
Adafruit_BMP085_Unified bmpModule = new Adafruit_BMP085_Unified(10085);
Servo myServo;

File fileAccX;
File fileAccY;

int servoPin = A0; // 18
int initServoPos = 0; // servo start position
int activeServoPos = 180; // servo active position

const int SD_PIN = 10;

const String FILE_NAME_ACC_X = "Acc_X.txt"; 
const String FILE_NAME_ACC_Y = "Acc_Y.txt";
const String FILE_NAME_ACC_Z = "Acc_Z.txt";

#define FILE_NAME_ALTITUDE = "Altitude.txt";
#define FILE_NAME_PRESSURE = "Pressure.txt";
#define FILE_NAME_TEMPERATURE = "Temprature.txt";



void setup(void) {
  Serial.begin(9600);
//  Serial.println("here lololol");


  if(!mpuModule.begin()) {
    Serial.println("Failed to find MPU6050 module");
    while(1) {} // enter infinite loop to stop the execution
  }
  Serial.println("MPU6050 moudule is found");

  if(!SD.begin(SD_PIN)) {
    Serial.println("SD Card is not found");  
    while(1) {}
  }

  Serial.println("SD card is found");

  if(!bmpModule.begin()) {
    Serial.println("Failed to find MPU6050 module");
    while(1) {}
  }
  Serial.println("BMP180 moudule is found");

  myServo.attach(servoPin);

  setUpMPU6050();
  myServo.write(initServoPos);
}
File dataFile;

void setUpMPU6050(void) {
  mpuModule.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpuModule.setGyroRange(MPU6050_RANGE_500_DEG);
  mpuModule.setFilterBandwidth(MPU6050_BAND_5_HZ);
  delay(100);
}

float averageTemp;
float bmpTemp;
float absoluteValue;

void loop(void) {
//  Serial.println("here lololol");
  sensors_event_t acceleration, gyro, temp;
  sensors_event_t bmpEvent;

  mpuModule.getEvent(&acceleration, &gyro, &temp);

  float xAcc = acceleration.acceleration.x;
  float yAcc = acceleration.acceleration.y;
  float zAcc = acceleration.acceleration.z;


  File file = SD.open("denaaa.txt", FILE_WRITE);
  if (file) {
    file.println(xAcc);
    file.close();
    // print to the serial port too:
  }
  Serial.println(xAcc);
//    myFile.close();

//    fileAccX.write(',');
//    fileAccX.write('\n');
  

//  Serial.print("x: "); Serial.println(xAcc);
//  Serial.print("y: "); Serial.println(yAcc);
//  Serial.print("z: "); Serial.println(zAcc);

  absoluteValue = sqrt(xAcc * xAcc + yAcc * yAcc + zAcc * zAcc);
  if(absoluteValue < 2) {
    myServo.write(activeServoPos);
    Serial.println("Free fall");
  }
  Serial.println(absoluteValue);

  printMPU6050Values(acceleration, gyro, temp);
//  Serial.println("==================Test==================");
//  printBMP180Values(bmpEvent);
//  Serial.println("==================Test==================");
//
//  bmpModule.getTemperature(&bmpTemp);
//  averageTemp = (temp.temperature + bmpTemp) / 2.0;
//  Serial.print("KEK: "); Serial.println(averageTemp);
//  delay(1000);
}

void printMPU6050Values(sensors_event_t &a, sensors_event_t &g, sensors_event_t &temp) {
  mpuModule.getEvent(&a, &g, &temp);
  /* Print out the values */
//  Serial.print("Acceleration X: ");
//  Serial.print(a.acceleration.x);
//  Serial.print(", Y: ");
//  Serial.print(a.acceleration.y);
//  Serial.print(", Z: ");
//  Serial.print(a.acceleration.z);
//  Serial.println(" m/s^2");
//
//  Serial.print("Rotation X: ");
//  Serial.print(g.gyro.x);
//  Serial.print(", Y: ");
//  Serial.print(g.gyro.y);
//  Serial.print(", Z: ");
//  Serial.print(g.gyro.z);
//  Serial.println(" rad/s");
//
//  Serial.print("Temperature: ");
//  Serial.print(temp.temperature);
//  Serial.println(" degC");
//
//  Serial.println("");
}

void printBMP180Values(sensors_event_t &event) {
  bmpModule.getEvent(&event);

  if(event.pressure) {
    Serial.print("Pressure: "); Serial.print(event.pressure); Serial.println(" hPa");

    float temperature;
    bmpModule.getTemperature(&temperature);
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");

    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    Serial.print("Altitude:    "); Serial.print(bmpModule.pressureToAltitude(seaLevelPressure,event.pressure)); Serial.println(" m");
    
    Serial.println("");
  }
}
