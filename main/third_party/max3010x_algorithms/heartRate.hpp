#pragma once

/*
 Optical Heart Rate Detection (PBA Algorithm)

 Original algorithm:
 - Nathan Seidle, SparkFun Electronics
 - Maxim Integrated Products, Inc.

 Local ESP-IDF-compatible port:
 - Arduino.h / WProgram.h dependency removed
 - C/C++ standard integer types used
 - function names preserved for compatibility with the original API

 This code is for experimental/prototyping use only.
 Do not use it for medical diagnosis.
*/

#include <cstdint>

bool checkForBeat(int32_t sample);

int16_t averageDCEstimator(int32_t* p, uint16_t x);
int16_t lowPassFIRFilter(int16_t din);
int32_t mul16(int16_t x, int16_t y);

void resetBeatDetector();