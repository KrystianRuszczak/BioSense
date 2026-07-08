#include "heartRate.hpp"

/*
 Optical Heart Rate Detection (PBA Algorithm)

 Original algorithm:
 - Nathan Seidle, SparkFun Electronics
 - Maxim Integrated Products, Inc.

 Local ESP-IDF-compatible port:
 - Arduino-specific dependencies removed
 - bitwise '&' in boolean conditions replaced with logical '&&'
 - static initialization made explicit
*/

static int16_t IR_AC_Max = 20;
static int16_t IR_AC_Min = -20;

static int16_t IR_AC_Signal_Current = 0;
static int16_t IR_AC_Signal_Previous = 0;
static int16_t IR_AC_Signal_min = 0;
static int16_t IR_AC_Signal_max = 0;
static int16_t IR_Average_Estimated = 0;

static int16_t positiveEdge = 0;
static int16_t negativeEdge = 0;
static int32_t ir_avg_reg = 0;

static int16_t cbuf[32] = {0};
static uint8_t offset = 0;

static const uint16_t FIRCoeffs[12] = {
    172, 321, 579, 927, 1360, 1858,
    2390, 2916, 3391, 3768, 4012, 4096
};

void resetBeatDetector()
{
    IR_AC_Max = 20;
    IR_AC_Min = -20;

    IR_AC_Signal_Current = 0;
    IR_AC_Signal_Previous = 0;
    IR_AC_Signal_min = 0;
    IR_AC_Signal_max = 0;
    IR_Average_Estimated = 0;

    positiveEdge = 0;
    negativeEdge = 0;
    ir_avg_reg = 0;

    for (uint8_t i = 0; i < 32; i++) {
        cbuf[i] = 0;
    }

    offset = 0;
}

bool checkForBeat(int32_t sample)
{
    bool beatDetected = false;

    IR_AC_Signal_Previous = IR_AC_Signal_Current;

    IR_Average_Estimated =
        averageDCEstimator(&ir_avg_reg, static_cast<uint16_t>(sample));

    IR_AC_Signal_Current =
        lowPassFIRFilter(static_cast<int16_t>(sample - IR_Average_Estimated));

    // Positive zero crossing, rising edge.
    if ((IR_AC_Signal_Previous < 0) && (IR_AC_Signal_Current >= 0))
    {
        IR_AC_Max = IR_AC_Signal_max;
        IR_AC_Min = IR_AC_Signal_min;

        positiveEdge = 1;
        negativeEdge = 0;
        IR_AC_Signal_max = 0;

        const int16_t amplitude = IR_AC_Max - IR_AC_Min;

        if ((amplitude > 20) && (amplitude < 1000))
        {
            beatDetected = true;
        }
    }

    // Negative zero crossing, falling edge.
    if ((IR_AC_Signal_Previous > 0) && (IR_AC_Signal_Current <= 0))
    {
        positiveEdge = 0;
        negativeEdge = 1;
        IR_AC_Signal_min = 0;
    }

    // Find maximum value in positive cycle.
    if ((positiveEdge != 0) && (IR_AC_Signal_Current > IR_AC_Signal_Previous))
    {
        IR_AC_Signal_max = IR_AC_Signal_Current;
    }

    // Find minimum value in negative cycle.
    if ((negativeEdge != 0) && (IR_AC_Signal_Current < IR_AC_Signal_Previous))
    {
        IR_AC_Signal_min = IR_AC_Signal_Current;
    }

    return beatDetected;
}

int16_t averageDCEstimator(int32_t* p, uint16_t x)
{
    *p += ((((int32_t)x << 15) - *p) >> 4);
    return static_cast<int16_t>(*p >> 15);
}

int16_t lowPassFIRFilter(int16_t din)
{
    cbuf[offset] = din;

    int32_t z = mul16(
        static_cast<int16_t>(FIRCoeffs[11]),
        cbuf[(offset - 11) & 0x1F]
    );

    for (uint8_t i = 0; i < 11; i++)
    {
        z += mul16(
            static_cast<int16_t>(FIRCoeffs[i]),
            static_cast<int16_t>(
                cbuf[(offset - i) & 0x1F] +
                cbuf[(offset - 22 + i) & 0x1F]
            )
        );
    }

    offset++;
    offset %= 32;

    return static_cast<int16_t>(z >> 15);
}

int32_t mul16(int16_t x, int16_t y)
{
    return static_cast<int32_t>(x) * static_cast<int32_t>(y);
}