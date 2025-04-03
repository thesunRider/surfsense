#include <Wire.h>  //Needed for I2C to GNSS

struct sensor_data {
  float accel_X;
  float accel_Y;
  float accel_Z;
  float gyro_X;
  float gyro_Y;
  float gyro_Z;
  float mag_X;
  float mag_Y;
  float mag_Z;
  float temp;
  long latitude;
  long longitude;
  long altitude;
  int siv;
  int force;
};


sensor_data sensor;

#define SDA_IMU 5
#define SCL_IMU 6

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS
#include "src/FastIMU.h"
SFE_UBLOX_GNSS myGNSS;


#define IMU_ADDRESS 0x68  //Change to the address of the IMU
MPU6500 IMU;              //Change to the name of any supported IMU! , better upgrade to compass based sensor


calData calib = { 0 };  //Calibration data
AccelData accelData;    //Sensor data
GyroData gyroData;
MagData magData;


void calibrate_IMU() {
  Serial.println("FastIMU calibration & data example");
  if (IMU.hasMagnetometer()) {
    delay(1000);
    Serial.println("Move IMU in figure 8 pattern until done.");
    delay(3000);
    IMU.calibrateMag(&calib);
    Serial.println("Magnetic calibration done!");
  } else {
    delay(5000);
  }

  delay(5000);
  Serial.println("Keep IMU level.");
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  }
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
}

void getdata_IMU() {
  IMU.update();
  IMU.getAccel(&accelData);
  Serial.print(accelData.accelX);
  Serial.print("\t");
  Serial.print(accelData.accelY);
  Serial.print("\t");
  Serial.print(accelData.accelZ);
  Serial.print("\t");
  IMU.getGyro(&gyroData);
  Serial.print(gyroData.gyroX);
  Serial.print("\t");
  Serial.print(gyroData.gyroY);
  Serial.print("\t");
  Serial.print(gyroData.gyroZ);
  if (IMU.hasMagnetometer()) {
    IMU.getMag(&magData);
    Serial.print("\t");
    Serial.print(magData.magX);
    Serial.print("\t");
    Serial.print(magData.magY);
    Serial.print("\t");
    Serial.print(magData.magZ);
  }
  if (IMU.hasTemperature()) {
    Serial.print("\t");
    Serial.println(IMU.getTemp());
    sensor.temp = IMU.getTemp();
  } else {
    Serial.println();
  }
  sensor.accel_X = accelData.accelX;
  sensor.accel_Y = accelData.accelY;
  sensor.accel_Z = accelData.accelZ;
  sensor.gyro_X = gyroData.gyroX;
  sensor.gyro_Y = gyroData.gyroY;
  sensor.gyro_Z = gyroData.gyroZ;
}

#define PRESSURE_SENSOR_PIN A1

void setup() {
  Serial.begin(115200);
  Serial.println("Starting surfsense");
  pinMode(PRESSURE_SENSOR_PIN, INPUT);

  Wire.begin();
  Wire1.begin(SDA_IMU, SCL_IMU);
  Wire1.setClock(400000);  //400khz clock

  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  if (myGNSS.begin() == false) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA);  //Set the I2C port to output both NMEA and UBX messages
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  //Save (only) the communications port settings to flash and BBR

  //This will pipe all NMEA sentences to the serial port so we can see them
  myGNSS.setNMEAOutputPort(Serial);
}


void getdata_GPS() {
  long latitude = myGNSS.getLatitude();
  Serial.print(F("Lat: "));
  Serial.print(latitude);

  long longitude = myGNSS.getLongitude();
  Serial.print(F(" Long: "));
  Serial.print(longitude);
  Serial.print(F(" (degrees * 10^-7)"));

  long altitude = myGNSS.getAltitude();
  Serial.print(F(" Alt: "));
  Serial.print(altitude);
  Serial.print(F(" (mm)"));

  byte SIV = myGNSS.getSIV();
  Serial.print(F(" SIV: "));
  Serial.print(SIV);

  Serial.println();
  sensor.latitude = latitude;
  sensor.longitude = longitude;
  sensor.altitude = altitude;
  sensor.siv = SIV;
}

void displaysensor_data() {
  Serial.print(">>sns dat:");
  Serial.print(millis());
  Serial.print(":"),
  Serial.print(sensor.accel_X);
  Serial.print(":");
  Serial.print(sensor.accel_Y);
  Serial.print(":");
  Serial.print(sensor.accel_Z);
  Serial.print(":");
  Serial.print(sensor.gyro_X);
  Serial.print(":");
  Serial.print(sensor.gyro_Z);
  Serial.print(":");
  Serial.print(sensor.mag_X);
  Serial.print(":");
  Serial.print(sensor.mag_Y);
  Serial.print(":");
  Serial.print(sensor.mag_Z);
  Serial.print(":");
  Serial.print(sensor.temp);
  Serial.print(":");
  Serial.print(sensor.latitude);
  Serial.print(":");
  Serial.print(sensor.longitude);
  Serial.print(":");
  Serial.print(sensor.altitude);
  Serial.print(":");
  Serial.print(sensor.siv);
  Serial.print(":");
  Serial.print(sensor.force);
  Serial.print("<<");
  Serial.println();
}


unsigned long lastTime_GPS, lastTime_IMU;

void loop() {

  if (millis() - lastTime_GPS > 1000) {  //1 sec for GPS update
    lastTime_GPS = millis();             //Update the timer
    Serial.println("loop GPS");
    getdata_GPS();
  }

  if (millis() - lastTime_IMU > 100) {  //100 msec for IMU update
    lastTime_IMU = millis();            //Update the timer
    Serial.println("loop IMU");
    getdata_IMU();
    sensor.force = analogRead(PRESSURE_SENSOR_PIN);
    Serial.println("Force Val:");
    Serial.println(sensor.force);
    displaysensor_data();
  }

  delay(10);  //Don't pound too hard on the I2C bus
}
