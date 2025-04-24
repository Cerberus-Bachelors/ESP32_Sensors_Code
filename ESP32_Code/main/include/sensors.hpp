#ifndef SENSORS_HPP
#define SENSORS_HPP

#include "Arduino.h"
#include <Wire.h>

#include "SparkFun_ISM330DHCX.h"
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

struct IMUData
{
    sfe_ism_data_t accelData;
    sfe_ism_data_t gyroData;
    double heading = 0;
};

struct SensorData
{
    IMUData dataIMU1;
    IMUData dataIMU2;

    int32_t longitude = 0;
    int32_t latitude = 0;
    int32_t altitude = 0;
    u_int8_t numSatelites = 0;
};

class Sensors
{
private:
    SparkFun_ISM330DHCX ISM1_;
    SparkFun_ISM330DHCX ISM2_;

    SFE_MMC5983MA Mag1_;
    SFE_MMC5983MA Mag2_;

    SFE_UBLOX_GNSS GNSS_;

    SensorData sensorData_;

    double publishRate_;
    double updateRateIMU_;
    double updateRateMag_;
    double updateRateGNSS_;

    u_int32_t lastPublish_ = 0;
    u_int32_t lastIMUupdate_ = 0;
    u_int32_t lastMagUpdate_ = 0;
    u_int32_t lastGNSSUpdate_ = 0;

    void initializeIMU(SparkFun_ISM330DHCX &ISM, TwoWire &Wire)
    {
        if (!ISM.begin(Wire))
        {
            Serial.println("ISM Did not begin.");
            while (true)
                ;
        }

        ISM.deviceReset();

        Serial.println("Reset ISM");
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
    };

    void initializeMag(SFE_MMC5983MA &Mag, TwoWire &Wire)
    {
        if (!Mag.begin(Wire))
        {
            Serial.println("MMC5983MA did not respond - check your wiring. Freezing.");
            while (true)
                ;
        }
        Mag.softReset();
    };

    void initializeGNSS(SFE_UBLOX_GNSS &GNSS, TwoWire &Wire)
    {
        if (!GNSS.begin(Wire)) // Connect to the u-blox module using Wire port
        {
            Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
            while (1)
                ;
        }

        GNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
        GNSS.setNavigationFrequency(2);  // Produce two solutions per second
        GNSS.setAutoPVT(true);           // Tell the GNSS to "send" each solution
        GNSS.saveConfiguration();
    };

    void readIMUData(SparkFun_ISM330DHCX &ISM, IMUData &dataIMU)
    {
        ISM.getAccel(&dataIMU.accelData);
        ISM.getGyro(&dataIMU.gyroData);
    };

    void readMAGData(SFE_MMC5983MA &mag, IMUData &dataIMU)
    {
        uint32_t rawValueX = 0;
        uint32_t rawValueY = 0;
        uint32_t rawValueZ = 0;
        double scaledX = 0;
        double scaledY = 0;
        double scaledZ = 0;
        double heading = 0;

        mag.getMeasurementXYZ(&rawValueX, &rawValueY, &rawValueZ);

        scaledX = (double)rawValueX - 131072.0;
        scaledX /= 131072.0;

        scaledY = (double)rawValueY - 131072.0;
        scaledY /= 131072.0;

        scaledZ = (double)rawValueZ - 131072.0;
        scaledZ /= 131072.0;

        heading = atan2(scaledX, 0 - scaledY);

        heading /= PI;
        heading *= 180;
        heading += 180;

        dataIMU.heading = heading;
    };

    void readGNSSData(SFE_UBLOX_GNSS &GNSS, SensorData &sensorData)
    {
        sensorData.latitude = GNSS.getLatitude();

        sensorData.longitude = GNSS.getLongitude();

        sensorData.altitude = GNSS.getAltitude();

        sensorData.numSatelites = GNSS.getSIV();
    };

public:
    Sensors(double updateRateIMU = 100.0, double updateRateMag = 100.0, double updateRateGNSS = 1.0, double publishRate = 100.0)
    {
        updateRateIMU_ = updateRateIMU;
        updateRateMag_ = updateRateMag;
        updateRateGNSS_ = updateRateGNSS;
        publishRate_ = publishRate;
    };

    void initSensors(TwoWire &Wire, TwoWire &Wire1)
    {
        initializeIMU(ISM1_, Wire);
        initializeIMU(ISM2_, Wire1);
        Serial.println("ISMs Initialized");

        initializeMag(Mag1_, Wire);
        initializeMag(Mag2_, Wire1);
        Serial.println("Mags Initialized");

        initializeGNSS(GNSS_, Wire);
        Serial.println("GNSS Initialized");

        Serial.println("The data format will be pulished in the following order");
        Serial.println("IMU1.accelDataX, IMU1.accelDataY, IMU1.accelDataZ, IMU1.gyroDataX, IMU1.gyroDataY, IMU1.gyroDataZ, Mag1.heading, GNSS.Latitude, GNSS.Longitude, GNSS.Altitude, GNSS.numSatelites");
    };

    void readSensors()
    {
        if ((1000 / double((millis() - lastIMUupdate_))) < updateRateIMU_)
        {
            lastIMUupdate_ = millis();
            readIMUData(ISM1_, sensorData_.dataIMU1);
            readIMUData(ISM2_, sensorData_.dataIMU2);
        }

        if ((1000 / double((millis() - lastMagUpdate_))) < updateRateMag_)
        {
            lastMagUpdate_ = millis();
            readMAGData(Mag1_, sensorData_.dataIMU1);
            readMAGData(Mag2_, sensorData_.dataIMU2);
        }

        if ((1000 / double((millis() - lastGNSSUpdate_))) < updateRateGNSS_)
        {
            if (GNSS_.getPVT() && (GNSS_.getInvalidLlh() == false))
            {
                lastGNSSUpdate_ = millis();
                readGNSSData(GNSS_, sensorData_);
            }
        }
    };

    void publishSensorData()
    {

        if ((1000 / double((millis() - lastPublish_))) < publishRate_)
        {
            lastPublish_ = millis();

            // IMU and Mag Sensor 1
            Serial.print(sensorData_.dataIMU1.accelData.xData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU1.accelData.yData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU1.accelData.zData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU1.gyroData.xData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU1.gyroData.yData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU1.gyroData.zData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU1.heading);
            Serial.print(",");

            // IMU and Mag Sensor 2
            Serial.print(sensorData_.dataIMU2.accelData.xData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU2.accelData.yData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU2.accelData.zData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU2.gyroData.xData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU2.gyroData.yData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU2.gyroData.zData);
            Serial.print(",");
            Serial.print(sensorData_.dataIMU2.heading);
            Serial.print(",");

            // GNSS Sensor
            Serial.print(sensorData_.latitude);
            Serial.print(",");
            Serial.print(sensorData_.longitude);
            Serial.print(",");
            Serial.print(sensorData_.altitude);
            Serial.print(",");
            Serial.println(sensorData_.numSatelites);
        }
    };
};

#endif