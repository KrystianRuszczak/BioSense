#include "battery_soc.hpp"

namespace biosense
{

struct battery_point
{
    float voltage;
    uint8_t percent;
};

static constexpr battery_point BATT_TABLE[] = {
    {4.20f, 100},
    {4.15f, 95},
    {4.10f, 90},
    {4.05f, 85},
    {4.00f, 80},
    {3.95f, 70},
    {3.90f, 62},
    {3.85f, 52},
    {3.80f, 42},
    {3.75f, 30},
    {3.70f, 20},
    {3.65f, 12},
    {3.60f, 7},
    {3.50f, 3},
    {3.40f, 1},
    {3.30f, 0},
};

static constexpr uint8_t BATT_TABLE_SIZE = sizeof(BATT_TABLE) / sizeof(BATT_TABLE[0]);

uint8_t battery_voltage_to_percent(float batt_v)
{
    if (batt_v >= BATT_TABLE[0].voltage)
    {
        return BATT_TABLE[0].percent;
    }
    
    if (batt_v <= BATT_TABLE[BATT_TABLE_SIZE - 1].voltage)
    {
        return BATT_TABLE[BATT_TABLE_SIZE - 1].percent;
    }
    
    for (uint8_t i = 0; i < BATT_TABLE_SIZE - 1; i++)
    {
        const battery_point& high = BATT_TABLE[i];
        const battery_point& low = BATT_TABLE[i + 1];

        if (batt_v <= high.voltage && batt_v >= low.voltage)
        {
            const float position = (batt_v - low.voltage) / (high.voltage - low.voltage);
            const float percent = static_cast<float>(low.percent) + position * static_cast<float>(high.percent - low.percent);

            return static_cast<uint8_t>(percent + 0.5f);
        }
    }
    
    return 0;
}
    
} // namespace biosense