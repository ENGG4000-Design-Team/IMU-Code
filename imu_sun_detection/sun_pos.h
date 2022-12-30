/*
 * sun_pos.h
 * Author: Ethan Garnier
 * Date Modified: December 30, 2022
 */
#ifndef H_SUN_POS
#define H_SUN_POS

#include <Arduino.h>
#include <TimeLib.h>

#define DEG_TO_RAD 0.01745329
#define PI 3.141592654
#define TWOPI 6.28318531

int calcSunPos(float *elevation, float *azimuth, float longitude, float latitude, time_t currentTime);

#endif