#include <Wire.h>  //Needed for I2C to GNSS
#include <Preferences.h>
#include <ezButton.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_MMC56x3.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS

#define DEBUG_ON

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
  long gps_xang_rate;
  long gps_yang_rate;
  long gps_zang_rate;
  long gps_xaccel;
  long gps_yaccel;
  long gps_zaccel;
  float mag_EX;
  float mag_EY;
  float mag_EZ;

  long NedNorthVel, NedEastVel, NedDownVel, VerticalAccEst, HorizontalAccEst, SpeedAccEst, HeadingAccEst;
  int headVeh, magDec, magAcc;


  unsigned long time_gps;
  unsigned long time_imu;
  unsigned long time_pressure;
};

File myFile;
sensor_data sensor;
Preferences preferences;

sh2_SensorValue_t sensorValue;
sensors_event_t mag_event;
SFE_UBLOX_GNSS myGNSS;
unsigned int file_counter = 0;
bool measuring_state = false;

#define PRESSURE_SENSOR_PIN A0
#define BNO08X_INT 9
#define BNO08X_RST 12

#define BNO08X_ADDR 0x4A
#define SCL_EXTRA 6
#define SDA_EXTRA 5
#define CALBUTTON A1
#define MEASBUTTON A2
#define LED_RED 10
#define LED_YELLOW 11

#define SCK_CARD A5

Adafruit_BNO08x bno08x(BNO08X_RST);

/* Assign a unique ID to this sensor at the same time */
Adafruit_MMC5603 mmc = Adafruit_MMC5603(127);

ezButton calbutton(CALBUTTON);    // create ezButton object that attach to pin 7;
ezButton measbutton(MEASBUTTON);  // create ezButton object that attach to pin 7;


// Hard-iron calibration settings
const float hard_iron[3] = {
  -32.34, -1.19, 6.25
};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
  { 0.993, 0.040, -0.002 },
  { 0.040, 1.003, -0.009 },
  { -0.002, -0.009, 1.006 }
};

// Magnetic declination from magnetic-declination.com
// East is positive ( ), west is negative (-)
// mag_decl = ( /-)(deg   min/60   sec/3600)
// Set to 0 to get magnetic heading instead of geo heading
const float mag_decl = -1.233;

//calibration values from CALC: https://www.digikey.com/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b
void get_calibrated_mag_values() {
  static float hi_cal[3];
  static float heading = 0;

  float mag_data[] = { sensor.mag_EX,
                       sensor.mag_EY,
                       sensor.mag_EZ };

  for (uint8_t i = 0; i < 3; i++) {
    hi_cal[i] = mag_data[i] - hard_iron[i];
  }

  // Apply soft-iron scaling
  for (uint8_t i = 0; i < 3; i++) {
    mag_data[i] = (soft_iron[i][0] * hi_cal[0])*(soft_iron[i][1] * hi_cal[1])*(soft_iron[i][2] * hi_cal[2]);
  }

  sensor.mag_EX = mag_data[0];
  sensor.mag_EY = mag_data[1];
  sensor.mag_EZ = mag_data[2];

  // Calculate angle for heading, assuming board is parallel to
  // the ground and  Y points toward heading.
  heading = -1 * (atan2(mag_data[0], mag_data[1]) * 180) / M_PI;

  // Apply magnetic declination to convert magnetic heading
  // to geographic heading
  heading = mag_decl;

  // Convert heading to 0..360 degrees
  if (heading < 0) {
    heading = 360;
  }
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER))
    Serial.println("Could not enable accelerometer");

  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED))
    Serial.println("Could not enable gyroscope");

  if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED))
    Serial.println("Could not enable magnetic field calibrated");
}

void save_data_tofile() {
  // if the file opened okay, write to it:
  if (myFile) {
    //String message = String(millis()) + "," + String(sensor.time_imu) + "," + String(sensor.accel_X) + "," + String(sensor.accel_Y) + "," + String(sensor.accel_Z) + "," + String(sensor.gyro_X) + "," + String(sensor.gyro_Y) + "," + String(sensor.gyro_Z) + "," + String(sensor.mag_X) + "," + String(sensor.mag_Y) + "," + String(sensor.mag_Z) + "," + String(sensor.temp) + "," + String(sensor.time_gps) + "," + String(sensor.latitude) + "," + String(sensor.longitude) + "," + String(sensor.altitude) + "," + String(sensor.siv) + "," + String(sensor.time_pressure) + "," + String(sensor.force);
    myFile.flush();  // this should be before writing so we can eliminate the possibility of waiting for any transfers to complete before the write happens
    myFile.printf(
      "%lu,%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%ld,%lu,%ld,%ld,%ld,%d,"
      "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%d,%d,%.2f,%.2f,%.2f\n",

      millis(),
      sensor.time_imu,
      sensor.accel_X, sensor.accel_Y, sensor.accel_Z,
      sensor.gyro_X, sensor.gyro_Y, sensor.gyro_Z,
      sensor.mag_X, sensor.mag_Y, sensor.mag_Z,
      sensor.temp,
      sensor.time_gps,
      sensor.latitude,
      sensor.longitude,
      sensor.altitude,
      sensor.siv,
      sensor.NedNorthVel,
      sensor.NedEastVel,
      sensor.NedDownVel,
      sensor.VerticalAccEst,
      sensor.HorizontalAccEst,
      sensor.SpeedAccEst,
      sensor.HeadingAccEst,
      sensor.headVeh,
      sensor.magDec,
      sensor.magAcc,
      sensor.time_pressure,
      sensor.force,
      sensor.mag_EX,
      sensor.mag_EY,
      sensor.mag_EZ);

    //myFile.println(message);
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening file ");
  }
}

void getdata_GPS(void* pvParameters) {
  unsigned long lastTime_GPS = millis();
  long latitude, longitude, altitude, NedNorthVel, NedEastVel, NedDownVel, VerticalAccEst, HorizontalAccEst, SpeedAccEst, HeadingAccEst;
  int headVeh, magDec, magAcc;
  byte SIV;
  SFE_UBLOX_GNSS* gnss = static_cast<SFE_UBLOX_GNSS*>(pvParameters);
  while (1) {

    if (measuring_state) {      //200 msec for GPS update
      lastTime_GPS = millis();  //Update the timer
      if (gnss->getPVT() && (gnss->getInvalidLlh() == false)) {
        latitude = gnss->getLatitude();
        longitude = gnss->getLongitude();
        altitude = gnss->getAltitude();
        SIV = gnss->getSIV();
        NedNorthVel = gnss->getNedNorthVel();
        NedEastVel = gnss->getNedEastVel();
        NedDownVel = gnss->getNedDownVel();
        VerticalAccEst = gnss->getVerticalAccEst();
        HorizontalAccEst = gnss->getHorizontalAccEst();
        SpeedAccEst = gnss->getSpeedAccEst();
        HeadingAccEst = gnss->getHeadingAccEst();

        if (gnss->getHeadVehValid() == true) {
          headVeh = gnss->getHeadVeh();
          magDec = gnss->getMagDec();
          magAcc = gnss->getMagAcc();
        }

        sensor.time_gps = millis();

        sensor.latitude = latitude;
        sensor.longitude = longitude;
        sensor.altitude = altitude;
        sensor.siv = SIV;

        sensor.NedNorthVel = NedNorthVel;
        sensor.NedEastVel = NedEastVel;
        sensor.NedDownVel = NedDownVel;
        sensor.VerticalAccEst = VerticalAccEst;
        sensor.HorizontalAccEst = HorizontalAccEst;
        sensor.SpeedAccEst = SpeedAccEst;
        sensor.HeadingAccEst = HeadingAccEst;
        sensor.headVeh = headVeh;
        sensor.magDec = magDec;
        sensor.magAcc = magAcc;

        gnss->flushPVT();
      }
    }
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  delay(5000);

  preferences.begin("trax", false);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);

  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_YELLOW, HIGH);

  delay(1000);  // Wait for Serial to become available.
  Serial.println("Starting surfsense");
  pinMode(PRESSURE_SENSOR_PIN, INPUT);

  Wire.begin();
  Wire1.begin(SDA_EXTRA, SCL_EXTRA);

  file_counter = preferences.getUInt("counter", 0);

  if (!SD.begin(SCK_CARD)) {
    Serial.println("initialization failed!");
    return;
  }

  if (!bno08x.begin_I2C(BNO08X_ADDR, &Wire1, 178)) {
    Serial.println("Failed to find BNO08x chip");
    return;
  }

  Serial.println("BNO08x found!");
  setReports();

  if (!mmc.begin(MMC56X3_DEFAULT_ADDRESS, &Wire)) {  // I2C mode
    /* There was a problem detecting the MMC5603 ... check your connections */
    Serial.println("Ooops, no MMC5603 detected ... Check your wiring!");
    return;
  }


  mmc.printSensorDetails();
  mmc.setDataRate(100);  // in Hz, from 1-255 or 1000
  mmc.setContinuousMode(true);


//gps not working
/**
  if (myGNSS.begin(Wire) == false) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    return;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);  //Set the I2C port to output both NMEA and UBX messages
  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);  //Save (only) the communications port settings to flash and BBR

  //measurement time (inms)
  if (myGNSS.setMeasurementRate(60) == false) {
    Serial.println(F("Could not set the measurement rate. Freezing."));
    return;
  }

  myGNSS.setI2CpollingWait(25);

  //number of measurements to average to make a solution (higher is better quality position but takes more time for measurements to load)
  if (myGNSS.setNavigationRate(5) == false) {
    Serial.println(F("Could not set the navigation rate. Freezing."));
    return;
  }

  myGNSS.setAutoPVT(true);  //Tell the GNSS to "send" each solution

  byte rate = myGNSS.getNavigationFrequency();  //Get the update rate of this module
  Serial.print("Current update rate: ");
  Serial.println(rate);

  xTaskCreatePinnedToCore(
    getdata_GPS,    // Function
    "getdata_GPS",  // Name
    5000,           // Stack size
    &myGNSS,        // 👈 Pointer to pass
    1,              // Priority
    NULL,           // Task handle
    0               // Core
  );
  **/

  delay(500);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YELLOW, LOW);
}

void getdata_IMU() {
  if (bno08x.getSensorEvent(&sensorValue)) {
    sensor.time_imu = millis();
    if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
      sensor.gyro_X = sensorValue.un.gyroscope.x;
      sensor.gyro_Y = sensorValue.un.gyroscope.y;
      sensor.gyro_Z = sensorValue.un.gyroscope.z;
    }

    if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      sensor.accel_X = sensorValue.un.accelerometer.x;
      sensor.accel_Y = sensorValue.un.accelerometer.y;
      sensor.accel_Z = sensorValue.un.accelerometer.z;
    }

    if (sensorValue.sensorId == SH2_MAGNETIC_FIELD_CALIBRATED) {
      sensor.mag_X = sensorValue.un.magneticField.x;
      sensor.mag_Y = sensorValue.un.magneticField.y;
      sensor.mag_Z = sensorValue.un.magneticField.z;
    }
  }

  mmc.getEvent(&mag_event);

  sensor.mag_EX = mag_event.magnetic.x;
  sensor.mag_EY = mag_event.magnetic.y;
  sensor.mag_EZ = mag_event.magnetic.z;

  get_calibrated_mag_values();
}


void displaysensor_data() {
  Serial.print(">>sns dat:");
  Serial.print(millis());
  Serial.print(",IMU-ticks:");
  Serial.print(sensor.time_imu);
  Serial.print(",aX:");
  Serial.print(sensor.accel_X);
  Serial.print(",aY:");
  Serial.print(sensor.accel_Y);
  Serial.print(",aZ:");
  Serial.print(sensor.accel_Z);
  Serial.print(",gX:");
  Serial.print(sensor.gyro_X);
  Serial.print(",gY:");
  Serial.print(sensor.gyro_Y);
  Serial.print(",gZ:");
  Serial.print(sensor.gyro_Z);
  Serial.print(",mX:");
  Serial.print(sensor.mag_X);
  Serial.print(",mY:");
  Serial.print(sensor.mag_Y);
  Serial.print(",mZ:");
  Serial.print(sensor.mag_Z);
  Serial.print(",tX:");
  Serial.print(sensor.temp);
  Serial.print(",GPS-ticks:");
  Serial.print(sensor.time_gps);
  Serial.print(",lat:");
  Serial.print(sensor.latitude);
  Serial.print(",long:");
  Serial.print(sensor.longitude);
  Serial.print(",alt:");
  Serial.print(sensor.altitude);
  Serial.print(",siv:");
  Serial.print(sensor.siv);
  Serial.print(",pressure-ticks:");
  Serial.print(sensor.time_pressure);
  Serial.print(",force:");
  Serial.print(sensor.force);
  Serial.print(",magx:");
  Serial.print(sensor.mag_EX);
  Serial.print(",magy:");
  Serial.print(sensor.mag_EY);
  Serial.print(",magz:");
  Serial.print(sensor.mag_EZ);
  Serial.print("<<");
  Serial.println();
}


unsigned long lastTime_IMU;

void loop() {
  calbutton.loop();
  measbutton.loop();



  if (calbutton.isPressed()) {
    digitalWrite(LED_YELLOW, HIGH);
    Serial.println("Calbutton is pressed");
    //write calibration code here
    if (myFile) myFile.flush();
    if (myFile) myFile.close();

    String filename = "/trax_calib_#" + String(file_counter) + ".csv";
    myFile = SD.open(filename, FILE_WRITE);
    if (myFile) Serial.println("File opened for Writing");
    myFile.println(F(
      "timestamp,IMU-ticks,aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,tX,"
      "GPS-ticks,lat,long,alt,siv,"
      "NedNorthVel,NedEastVel,NedDownVel,"
      "VerticalAccEst,HorizontalAccEst,SpeedAccEst,HeadingAccEst,"
      "headVeh,magDec,magAcc,"
      "pressure-ticks,force1,magEx,magEY,magEZ"));
    measuring_state = true;


    for (int i = 0; i < 100; i++) {
      getdata_IMU();
      sensor.force = analogRead(PRESSURE_SENSOR_PIN);
      sensor.time_pressure = millis();
      save_data_tofile();
      delay(100);
    }

    //wait 10 seconds and save calibrated data to bno085 flash
    digitalWrite(LED_YELLOW, LOW);
  }

  if (measbutton.isPressed()) {
    Serial.println("Measbutton is pressed");
    if (myFile) myFile.flush();
    if (myFile) myFile.close();
    if (!measuring_state) {
      digitalWrite(LED_RED, HIGH);
      preferences.putUInt("counter", ++file_counter);
      String filename = "/trax_" + String(myGNSS.getYear()) + "_" + String(myGNSS.getMonth()) + "_" + String(myGNSS.getDay()) + "_" + String(myGNSS.getHour()) + "_" + String(myGNSS.getMinute()) + "_" + String(myGNSS.getSecond()) + "_#" + String(file_counter) + ".csv";
      myFile = SD.open(filename, FILE_WRITE);
      if (myFile) Serial.println("File opened for Writing");
      myFile.println(F(
        "timestamp,IMU-ticks,aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,tX,"
        "GPS-ticks,lat,long,alt,siv,"
        "NedNorthVel,NedEastVel,NedDownVel,"
        "VerticalAccEst,HorizontalAccEst,SpeedAccEst,HeadingAccEst,"
        "headVeh,magDec,magAcc,"
        "pressure-ticks,force1,magEx,magEY,magEZ"));
      measuring_state = true;
    } else {
      digitalWrite(LED_RED, LOW);
      measuring_state = false;
    }
  }

  if (measuring_state) {
    if (bno08x.wasReset()) {
      Serial.print("sensor was reset ");
      setReports();
    }

    if (millis() - lastTime_IMU > 10) {  //10 msec for IMU update
      lastTime_IMU = millis();           //Update the timer
      getdata_IMU();
      sensor.force = analogRead(PRESSURE_SENSOR_PIN);
      sensor.time_pressure = millis();
      save_data_tofile();

#ifdef DEBUG_ON
      displaysensor_data();
#endif
    }
  }

  delay(10);  //Don't pound too hard on the I2C bus
}
