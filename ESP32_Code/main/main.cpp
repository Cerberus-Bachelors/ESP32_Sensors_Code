#include "Arduino.h"
#include <Wire.h>

#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "SparkFun_ISM330DHCX.h"
#include <vector>

#define sda_pin 26
#define scl_pin 27
#define sda_pin1 22
#define scl_pin1 23

SparkFun_ISM330DHCX ISM1;
SparkFun_ISM330DHCX ISM2;

SFE_MMC5983MA Mag1;
SFE_MMC5983MA Mag2;

SFE_UBLOX_GNSS myGNSS;

long lastTime = 0; 

void intilize_IMU(SparkFun_ISM330DHCX &ISM, TwoWire &Wire){
  if( !ISM.begin(Wire) ){
		Serial.println("Did not begin.");
		while(1);
	}

  ISM.deviceReset();

  Serial.println("Reset.");
	Serial.println("Applying settings.");
	delay(100);

  ISM.setDeviceConfig();
	ISM.setBlockDataUpdate();

  	// Set the output data rate and precision of the accelerometer
	ISM.setAccelDataRate(ISM_XL_ODR_104Hz);
	ISM.setAccelFullScale(ISM_16g); 

	// Set the output data rate and precision of the gyroscope
	ISM.setGyroDataRate(ISM_GY_ODR_104Hz);
	ISM.setGyroFullScale(ISM_4000dps); 

	// Turn on the accelerometer's filter and apply settings. 
	ISM.setAccelFilterLP2();
	ISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

	// Turn on the gyroscope's filter and apply settings. 
	ISM.setGyroFilterLP1();
	ISM.setGyroLP1Bandwidth(ISM_MEDIUM);
}

void intilize_Mag(SFE_MMC5983MA &Mag, TwoWire &Wire){
  if (Mag.begin(Wire) == false){
      Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
      while (true);
  }
  Mag.softReset();
}

void intilize_GNSS(SFE_UBLOX_GNSS &GNSS, TwoWire &Wire){
  if (GNSS.begin(Wire) == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  GNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  GNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
}

void getPosition(SFE_UBLOX_GNSS &GNSS){
  if (millis() - lastTime > 1000)
  {
    lastTime = millis(); //Update the timer
    
    long latitude = GNSS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(latitude);

    long longitude = GNSS.getLongitude();
    Serial.print(F(" Long: "));
    Serial.print(longitude);
    Serial.print(F(" (degrees * 10^-7)"));

    long altitude = GNSS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude);
    Serial.print(F(" (mm)"));

    byte SIV = GNSS.getSIV();
    Serial.print(F(" SIV: "));
    Serial.print(SIV);

    Serial.println();
  }

}

void checkIMU(SparkFun_ISM330DHCX &ISM){
  sfe_ism_data_t accelData; 
  sfe_ism_data_t gyroData; 
  ISM.getAccel(&accelData);
  ISM.getGyro(&gyroData);
  Serial.print("Accelerometer: ");
  Serial.print(" X: ");
  Serial.print(accelData.xData);
  Serial.print(" ");
  Serial.print(" Y: ");
  Serial.print(accelData.yData);
  Serial.print(" ");
  Serial.print(" Z: ");
  Serial.print(accelData.zData);
  Serial.print("  ");
  Serial.print("Gyroscope: ");
  Serial.print(" X: ");
  Serial.print(gyroData.xData);
  Serial.print(" ");
  Serial.print(" Y: ");
  Serial.print(gyroData.yData);
  Serial.print(" ");
  Serial.print(" Z: ");
  Serial.print(gyroData.zData);
  Serial.print(" ");
}

void checkMag(SFE_MMC5983MA &Mag){

  uint32_t currentX = 0;
  uint32_t currentY = 0;
  uint32_t currentZ = 0;
  double scaledX = 0;
  double scaledY = 0;
  double scaledZ = 0;

  // This reads the X, Y and Z channels consecutively
  // (Useful if you have one or more channels disabled)
  currentX = Mag.getMeasurementX();
  currentY = Mag.getMeasurementY();
  currentZ = Mag.getMeasurementZ();

  // Or, we could read all three simultaneously
  //myMag.getMeasurementXYZ(&currentX, &currentY, &currentZ);
  Serial.print("Magnetometer");
  Serial.print(" X: ");
  Serial.print(currentX);
  Serial.print(" Y: ");
  Serial.print(currentY);
  Serial.print(" Z: ");
  Serial.print(currentZ);
}

class main
{
private:
  /* data */
public:
  main(/* args */);
  ~main();

  void initlizeIMU(SparkFun_ISM330DHCX &ISM, TwoWire &Wire);
  void initlizeMag(SFE_MMC5983MA &Mag, TwoWire &Wire);
  void initlizeGNSS(SFE_UBLOX_GNSS &GNSS, TwoWire &Wire);

  void getSensorData();

  void getImuData();
  void getMagData();
  void getPosition();

  void sendSensorData();
};

main::main(/* args */)
{
}

main::~main()
{
}


void setup() {
  Serial.begin(115200);

  Wire.begin(sda_pin, scl_pin);
  Wire1.begin(sda_pin1, scl_pin1);

  intilize_IMU(ISM1, Wire);
  intilize_IMU(ISM2, Wire1);

  intilize_Mag(Mag1, Wire);
  intilize_Mag(Mag2, Wire1);

  intilize_GNSS(myGNSS, Wire);

}

void loop() {

  //checkIMU(ISM1);
  //checkMag(Mag1);
  //Serial.println(" ");

  //checkIMU(ISM2);
  //heckMag(Mag2);
  //Serial.println(" ");
  //Serial.println(" ");

  //delay(100);

  getPosition(myGNSS);
}
