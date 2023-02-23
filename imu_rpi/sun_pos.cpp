/*
 * sun_pos.cpp
 * Author: Ethan Garnier
 * Date Modified: February 23, 2022
 */
#include <iostream>

#include <chrono>
#include <ctime>

#include "sun_pos.h"

/*
 julianDate calculates the integer portion of the current Julia Date.
 This is the number of days ellapsed since noon on January 1, 4713 B. C.
 This code is modified from "Arduino Uno and Solar Position Calculations"
 authored by David Brooks.
*/
long julianDate(int year, int month, int day)
{
    long JD_whole;
    int A, B;
    if (month <= 2)
    {
        year--;
        month += 12;
    }
    A = year / 100;
    B = 2 - A + A / 4;
    JD_whole = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
    return JD_whole;
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
int calcSunPos(float *elevation, float *azimuth, float longitude, float latitude)
{
    //auto wallTime = std::chrono::system_clock::now();
    //time_t time = std::chrono::system_clock::to_time_t(wallTime);
    //tm utc_time = *gmtime(&time);

    auto tp = std::chrono::system_clock::now();
    auto dp = floor<std::chrono::days>(tp);
    std::chrono::year_month_day ymd{dp};
    std::chrono::hh_mm_ss time{floor<std::chrono::milliseconds>(tp-dp)};
    auto y = ymd.year();
    auto m = ymd.month();
    auto d = ymd.day();
    auto h = time.hours();
    auto M = time.minutes();
    auto s = time.seconds();
    auto ms = time.subseconds();

    std::cout << "Year:" << static_cast<int>(y) << std::endl;
    std::cout << "Month:" << static_cast<unsigned>(m) << std::endl;
    std::cout << "Day:" << static_cast<unsigned>(d) << std::endl;
    std::cout << "Hour:" << h.count() << std::endl;
    std::cout << "Minute:" << M.count() << std::endl;
    std::cout << "Second:" << s.count() << std::endl;
    std::cout << "Millisecond:" << ms.count() << std::endl;

    /*int cSecond = second(currentTime);
    int cMinute = minute(currentTime);
    int cHour = hour(currentTime);
    int cDay = day(currentTime);
    int cMonth = month(currentTime);
    int cYear = year(currentTime); // todo: we can hardcode this yes?

    float longitudeRad = longitude * DEG_TO_RAD;
    float latitudeRad = latitude * DEG_TO_RAD;

    // From Arduino Uno and Solar Position Calculations:
    // "T is the number of Julian centuries (36525 days)
    // since 12h:00m:00s Universal Time, Jan 1, 2000"
    float T;

    long JD_whole = julianDate(cYear, cMonth, cDay);
    float JD_frac = (cHour + cMinute / 60. + cSecond / 3600.) / 24. - .5;

    // Calculate T
    T = JD_whole - 2451545;
    T = (T + JD_frac) / 36525.;

    // Calculate solar longitude L0 and
    // solar mean anomaly M. Convert to radians
    float L0 = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * T, 360);
    float M = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * T, 360);

    // Eccentricity of Earth’s orbit e
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

    */

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
