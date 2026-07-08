This directory contains a local ESP-IDF-compatible port of the optical heart-rate beat detection algorithm originally distributed with the SparkFun MAX3010x Sensor Library / Maxim Integrated sample code.

Original algorithm:
- Optical Heart Rate Detection (PBA Algorithm)
- SparkFun Electronics, Nathan Seidle
- Maxim Integrated Products, Inc.

Changes in this port:
- Arduino.h / WProgram.h dependencies removed.
- Standard C/C++ integer headers used.
- Boolean expressions using bitwise '&' were changed to logical '&&'.
- Static state initialization was made explicit.
- Function names were preserved for compatibility with the original API.

This code is intended for experimental/prototyping use only and must not be used for medical diagnosis.

Original license notice from Maxim Integrated should be preserved with this project.