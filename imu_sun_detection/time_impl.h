/*
 * time_impl.h
 * Author: Ethan Garnier
 * Date Modified: January 3, 2022
 */
#ifndef H_TIME_IMPL
#define H_TIME_IMPL

#include <Arduino.h>
#include <TimeLib.h>

#define UTC_OFFSET -4

time_t compileTime();
time_t toUTC(time_t local);

#endif