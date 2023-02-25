/**
 * imu.cpp
 * Author: Ethan Garnier
 * BBST IMU software for use on Raspberry Pi
 * sun tracking system. This software
 * grabs the absolute heading, pitch, and roll
 * of the IMU connected to the Raspberry Pi
 * over serial and determined correction factors
 * to point the IMU at the sun.
 */
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

#include "sun_pos.h"
#include "cmps14.hpp"

int main()
{
    auto imu = new cmps14(false);

    if (imu->begin() == -1)
    {
        return 0;
    }

    std::cout << "IMU Software Version: " << imu->getSoftwareVersion() << std::endl;
    std::vector<int> calStatus = imu->getCalibrationStatus();
    std::cout << "IMU Calibration State: " << std::endl
              << "\tSystem: " << calStatus[3] << std::endl
              << "\tGyroscope: " << calStatus[2] << std::endl
              << "\tAccelerometer: " << calStatus[1] << std::endl
              << "\tMagnotometer: " << calStatus[0] << std::endl;

    // TODO: Do we want to enable automatic calibration
    // or just use the factory calibration?

    float heading = 0.0f, pitch = 0.0f;
    float azimuth = 0.0f, elevation = 0.0f;
    float latitude = 45.945045;
    float longitude = -66.648227;
    float headingCorrection = 0.0f, pitchCorrection = 0.0f;

    while (1)
    {
        heading = imu->getHeading();
        pitch = imu->getPitch();

        calcSunPos(elevation, azimuth, longitude, latitude);

        headingCorrection = azimuth - heading;
        pitchCorrection = elevation - pitch;

        std::cout << "\nHeading correction: " << headingCorrection << " degrees" << std::endl;
        std::cout << "\nPitch correction: " << pitchCorrection << " degrees" << std::endl;

        //std::cout << "\nHeading: " << heading << std::endl;
        //std::cout << "Pitch: " << pitch << std::endl;
        //std::cout << "Roll: " << roll << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 1;
}