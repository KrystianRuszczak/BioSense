#pragma once

/*
 MAXREFDES117 / MAX3010x SpO2 algorithm

 Local ESP-IDF-compatible port:
 - Arduino.h dependency removed
 - AVR branch removed
 - global scratch buffers moved to algorithm.cpp
 - function names preserved

 This code is for experimental/prototyping use only.
 Do not use it for medical diagnosis.
*/

#include <cstdint>

static constexpr int32_t MAXIM_SPO2_FREQ_HZ = 25;
static constexpr int32_t MAXIM_SPO2_BUFFER_SIZE = MAXIM_SPO2_FREQ_HZ * 4;
static constexpr int32_t MAXIM_SPO2_MA4_SIZE = 4;

void maxim_heart_rate_and_oxygen_saturation(
    uint32_t* pun_ir_buffer,
    int32_t n_ir_buffer_length,
    uint32_t* pun_red_buffer,
    int32_t* pn_spo2,
    int8_t* pch_spo2_valid,
    int32_t* pn_heart_rate,
    int8_t* pch_hr_valid
);

void maxim_find_peaks(
    int32_t* pn_locs,
    int32_t* n_npks,
    int32_t* pn_x,
    int32_t n_size,
    int32_t n_min_height,
    int32_t n_min_distance,
    int32_t n_max_num
);

void maxim_peaks_above_min_height(
    int32_t* pn_locs,
    int32_t* n_npks,
    int32_t* pn_x,
    int32_t n_size,
    int32_t n_min_height
);

void maxim_remove_close_peaks(
    int32_t* pn_locs,
    int32_t* pn_npks,
    int32_t* pn_x,
    int32_t n_min_distance
);

void maxim_sort_ascend(int32_t* pn_x, int32_t n_size);

void maxim_sort_indices_descend(
    int32_t* pn_x,
    int32_t* pn_indx,
    int32_t n_size
);