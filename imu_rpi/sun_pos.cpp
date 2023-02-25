/*
 * sun_pos.cpp
 * Author: Ethan Garnier
 * Date Modified: February 23, 2022
 */
#include "sun_pos.h"

#include <iostream>

#include <chrono>
#include <cmath>
#include <time.h>

/*
 julianDate calculates the integer portion of the current Julia Date.
 This is the number of days ellapsed since noon on January 1, 4713 B. C.
 This code is modified from "Arduino Uno and Solar Position Calculations"
 authored by David Brooks.
*/
long getJulianDate(int year, int month, int day)
{
    long jd;
    int A, B;
    if (month <= 2)
    {
        year--;
        month += 12;
    }
    A = year / 100;
    B = 2 - A + A / 4;
    jd = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
    return jd;
}

/*
 calcSunPos calculates the Sun's elevation and azimuth angles as
 a function of location, date, and time. This code is modified from
 "Arduino Uno and Solar Position Calculations" authored by David Brooks.
 The disclamer given by Mr. Brooks:

 This program calculates solar positions as a function of location, date, and time.
 The equations are from Jean Meeus, Astronomical Algorithms, Willmann-Bell, Inc., Richmond, VA
 (C) 2015, David Brooks, Institute for Earth Science Research and Education.

 calcSunPos returns the Sun's elevation and azimuth angles in degrees through the two
 float references float &elevation and float &azimuth. The actual values returned is 1
 if all calculations were successful, and 0 if an error occured.
*/
int calcSunPos(float &elevation, float &azimuth, float longitude, float latitude)
{
    auto tp = std::chrono::system_clock::now();
    time_t tt = std::chrono::system_clock::to_time_t(tp);
    tm utc_tm = *gmtime(&tt);
    auto year = utc_tm.tm_year + 1900;
    auto month = utc_tm.tm_mon + 1;
    auto day = utc_tm.tm_mday;
    auto hour = utc_tm.tm_hour;
    auto minute = utc_tm.tm_min;
    auto second = utc_tm.tm_sec;

    // Multiply by 4 since we are GMT-4:00
    /*float LSTM = 15 * 4;

    // Determine number of days since start of year
    const auto day0 = std::chrono::sys_days{std::chrono::January/0/year};
    const auto delta = (dp - day0).count();

    // Calculate equation of time
    float B = (360 / 365) * (delta - 81);
    float EoT = 9.87 * sin(2 * B) - 7.53 * cos(B) - 1.5 * sin(B);

    // Time Correction Factor
    float TC = 4 * (longitude - LSTM) + EoT;*/

    /*long jd_whole = getJulianDate(static_cast<int>(year), static_cast<unsigned>(month), static_cast<unsigned>(day));
    //float jd_frac = (hour.count() + minute.count() / 60.0 + second.count() / 3600.0) / 24.0 - 0.5;

    float t = (jd_whole + hour.count() / 24 - 2451545.0) / 36525;

    float T = t + floor(-3.36 + 1.35 * pow(t + 2.33, 2)) * pow(10, -8);

    // The Greensich mean sidereal time
    float ST = 100.4606 - 36000.77005 * t + 0.000388 * t * t - 3 * pow(10, -8) * t * t * t;

    // The geometric mean ecliptic longitude of date
    float L = 280.46607 + 36000.76980 * T + 0.0003025 * T * T;

    // The mean anomaly
    float G = 357.528 + 35999.0503 * T;

    // The mean obliquity of the ecliptic
    float epsilon = 23.4393 - 0.01300 * T - 0.0000002 * T * T + 0.0000005 * T * T * T;

    // The equation of the centre
    float C = (1.9146 - 0.00484 * T - 0.000014 * T * T) * sin(G) + (0.01999 - 0.00008 * T) * sin(2 * G);

    // The ecliptic longitude of date
    float L0 = L + C - 0.0057;

    // Right ascension
    float y = tan(epsilon / 2) * tan(epsilon / 2);
    float f = 180 / PI;
    float alpha = L0 - y * f * sin(2 * L0) + 0.2 * y * y * f * sin(4 * L0);

    // Equation of time
    float E = (ST - alpha) - (15 * hour.count() - 180);
    E = (E > 10) ? E - 360 : E;

    std::cout << "Equation of time: " << E << std::endl;*/

    float longitudeRad = longitude * DEG_TO_RAD;
    float latitudeRad = latitude * DEG_TO_RAD;

    // From Arduino Uno and Solar Position Calculations:
    // "T is the number of Julian centuries (36525 days)
    // since 12h:00m:00s Universal Time, Jan 1, 2000"
    float T;

    long JD_whole = getJulianDate(year, month, day);
    float JD_frac = (hour + minute / 60. + second / 3600.) / 24. - .5;

    // Calculate T
    T = JD_whole - 2451545;
    T = (T + JD_frac) / 36525.;

    // Calculate solar longitude L0 and
    // solar mean anomaly M. Convert to radians
    float L0 = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * T, 360);
    float M = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * T, 360);

    // Eccentricity of Earth’s orbit e`
    float e = 0.016708617 - 0.000042037 * T;

    // Sun’s equation of center C in radians
    float C = DEG_TO_RAD * ((1.9146 - 0.004847 * T) * sin(M) + (0.019993 - 0.000101 * T) * sin(2 * M) + 0.00029 * sin(3 * M));

    // Solar true anomaly f in radians
    float f = M + C;

    // Earth's distance from the Sun R in astronomical units
    float R = 1.000001018 * (1 - e * e) / (1 + e * cos(f));

    // Greenwich hour angle in degrees
    // The following math removes the issue of Arduino
    // math's precision
    long JDx = JD_whole - 2451545;
    float GrHrAngle = 280.46061837 + (360 * JDx) % 360 + .98564736629 * JDx + 360.98564736629 * JD_frac;
    GrHrAngle = fmod(GrHrAngle, 360.);

    // Obliquity of the equator in radians
    float Obl = DEG_TO_RAD * (23 + 26 / 60. + 21.448 / 3600. - 46.815 / 3600 * T);

    // Solar true longitude
    float L_true = fmod(C + L0, TWOPI);

    // Right ascension
    float RA = atan2(sin(L_true) * cos(Obl), cos(L_true));

    // Declination
    float Decl = asin(sin(Obl) * sin(L_true));

    // Hour angle
    float HrAngle = DEG_TO_RAD * GrHrAngle + longitudeRad - RA;

    // Values returned:
    *elevation = (asin(sin(latitudeRad) * sin(Decl) + cos(latitudeRad) * (cos(Decl) * cos(HrAngle)))) / DEG_TO_RAD;
    // Azimuth measured eastward from north.
    *azimuth = (PI + atan2(sin(HrAngle), cos(HrAngle) * sin(latitudeRad) - tan(Decl) * cos(latitudeRad))) / DEG_TO_RAD;

    /*Serial.print(cYear);
    Serial.print(",");
    Serial.print(cMonth);
    Serial.print(",");
    Serial.print(cDay);
    Serial.print(",");
    Serial.print(cHour);
    Serial.print(",");
    Serial.print(cMinute);
    Serial.print(",");
    Serial.print(cSecond);
    Serial.println();*/

    return 1;
}

/*
 testSunPosCalc tests the calcSunPos function above by running it over the time
 interval 10:00am to 4:30pm on June 21st 2022 in 30 minute increments. Please note that
 calcSunPos works off of UTC, however I am testing it against UTC-03:00, thus the values
 in set time start at hour 13 not 10.

 This function will print the calculated azimuth and elevation angles for further processing.
*/
/*int testSunPosCalc()
{
    float azimuth = 0.000f;
    float elevation = 0.000f;
    float latitude = 45.94505;
    float longitude = -66.64798;

    setTime(13, 0, 0, 21, 6, 2022);
    time_t t = now();

    while (hour(t) <= 19)
    {
        calcSunPos(&elevation, &azimuth, longitude, latitude, t);

        Serial.println();
        Serial.print("Time: ");
        Serial.print((hour(t) - 3));
        Serial.print(":");
        Serial.print(minute(t));
        Serial.print(":");
        Serial.print(second(t));
        Serial.println();
        Serial.print("Elevation angle (deg): ");
        Serial.print(elevation);
        Serial.println();
        Serial.print("Azimuth angle (deg): ");
        Serial.print(azimuth);
        Serial.println();

        // increment time by 1800 seconds
        adjustTime((long)1800);
        t = now();
    }
}*/
