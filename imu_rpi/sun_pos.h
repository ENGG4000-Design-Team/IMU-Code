/*
 * sun_pos.h
 * Author: Ethan Garnier
 * Date Modified: February 22, 2022
 */
#pragma once

#define DEG_TO_RAD 0.01745329
#define PI 3.141592654
#define TWOPI 6.28318531

int calcSunPos(float *elevation, float *azimuth, float longitude, float latitude, time_t currentTime);
int testSunPosCalc();
