/*
 MAXREFDES117 / MAX3010x SpO2 algorithm

 Local ESP-IDF-compatible port:
 - Arduino.h dependency removed
 - AVR branch removed
 - global scratch buffers moved to this .cpp file
 - function names preserved

 This code is for experimental/prototyping use only.
 Do not use it for medical diagnosis.
*/

#include "spo2.hpp"

#include <cstdint>

static const uint8_t uch_spo2_table[184] = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97,
    97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
    99, 99, 99, 99, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 99, 99, 99, 99, 99, 99,
    99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
    97, 97, 96, 96, 96, 96, 95, 95, 95, 94,
    94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
    90, 90, 89, 89, 89, 88, 88, 87, 87, 86,
    86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
    80, 80, 79, 78, 78, 77, 76, 76, 75, 74,
    74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
    66, 66, 65, 64, 63, 62, 62, 61, 60, 59,
    58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
    49, 48, 47, 46, 45, 44, 43, 42, 41, 40,
    39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
    28, 27, 26, 25, 23, 22, 21, 20, 19, 17,
    16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
    3, 2, 1
};

static int32_t an_x[MAXIM_SPO2_BUFFER_SIZE] = {0}; // IR
static int32_t an_y[MAXIM_SPO2_BUFFER_SIZE] = {0}; // RED

static int32_t maxim_min_i32(int32_t a, int32_t b)
{
    return (a < b) ? a : b;
}

void maxim_heart_rate_and_oxygen_saturation(
    uint32_t* pun_ir_buffer,
    int32_t n_ir_buffer_length,
    uint32_t* pun_red_buffer,
    int32_t* pn_spo2,
    int8_t* pch_spo2_valid,
    int32_t* pn_heart_rate,
    int8_t* pch_hr_valid
)
{
    if (pun_ir_buffer == nullptr ||
        pun_red_buffer == nullptr ||
        pn_spo2 == nullptr ||
        pch_spo2_valid == nullptr ||
        pn_heart_rate == nullptr ||
        pch_hr_valid == nullptr ||
        n_ir_buffer_length != MAXIM_SPO2_BUFFER_SIZE)
    {
        if (pn_spo2 != nullptr) {
            *pn_spo2 = -999;
        }

        if (pch_spo2_valid != nullptr) {
            *pch_spo2_valid = 0;
        }

        if (pn_heart_rate != nullptr) {
            *pn_heart_rate = -999;
        }

        if (pch_hr_valid != nullptr) {
            *pch_hr_valid = 0;
        }

        return;
    }

    uint32_t un_ir_mean = 0;
    int32_t k = 0;
    int32_t n_i_ratio_count = 0;
    int32_t i = 0;
    int32_t n_exact_ir_valley_locs_count = 0;
    int32_t n_middle_idx = 0;
    int32_t n_th1 = 0;
    int32_t n_npks = 0;
    int32_t an_ir_valley_locs[15] = {0};
    int32_t n_peak_interval_sum = 0;

    int32_t n_y_ac = 0;
    int32_t n_x_ac = 0;
    int32_t n_spo2_calc = 0;
    int32_t n_y_dc_max = 0;
    int32_t n_x_dc_max = 0;
    int32_t n_y_dc_max_idx = 0;
    int32_t n_x_dc_max_idx = 0;
    int32_t an_ratio[5] = {0};
    int32_t n_ratio_average = 0;
    int32_t n_nume = 0;
    int32_t n_denom = 0;

    // Calculate DC mean.
    for (k = 0; k < n_ir_buffer_length; k++) {
        un_ir_mean += pun_ir_buffer[k];
    }

    un_ir_mean = un_ir_mean / static_cast<uint32_t>(n_ir_buffer_length);

    // Remove DC and invert IR signal so that peak detector works as valley detector.
    for (k = 0; k < n_ir_buffer_length; k++) {
        an_x[k] = -1 * (
            static_cast<int32_t>(pun_ir_buffer[k]) -
            static_cast<int32_t>(un_ir_mean)
        );
    }

    // 4-point moving average.
    for (k = 0; k < MAXIM_SPO2_BUFFER_SIZE - MAXIM_SPO2_MA4_SIZE; k++) {
        an_x[k] = (
            an_x[k] +
            an_x[k + 1] +
            an_x[k + 2] +
            an_x[k + 3]
        ) / 4;
    }

    // Calculate threshold.
    n_th1 = 0;

    for (k = 0; k < MAXIM_SPO2_BUFFER_SIZE; k++) {
        n_th1 += an_x[k];
    }

    n_th1 = n_th1 / MAXIM_SPO2_BUFFER_SIZE;

    if (n_th1 < 30) {
        n_th1 = 30;
    }

    if (n_th1 > 60) {
        n_th1 = 60;
    }

    for (k = 0; k < 15; k++) {
        an_ir_valley_locs[k] = 0;
    }

    // Since we flipped signal, we use peak detector as valley detector.
    maxim_find_peaks(
        an_ir_valley_locs,
        &n_npks,
        an_x,
        MAXIM_SPO2_BUFFER_SIZE,
        n_th1,
        4,
        15
    );

    n_peak_interval_sum = 0;

    if (n_npks >= 2) {
        for (k = 1; k < n_npks; k++) {
            n_peak_interval_sum += (
                an_ir_valley_locs[k] -
                an_ir_valley_locs[k - 1]
            );
        }

        n_peak_interval_sum = n_peak_interval_sum / (n_npks - 1);

        if (n_peak_interval_sum > 0) {
            *pn_heart_rate =
                static_cast<int32_t>((MAXIM_SPO2_FREQ_HZ * 60) / n_peak_interval_sum);
            *pch_hr_valid = 1;
        } else {
            *pn_heart_rate = -999;
            *pch_hr_valid = 0;
        }
    } else {
        *pn_heart_rate = -999;
        *pch_hr_valid = 0;
    }

    // Load raw value again for SpO2 calculation: RED = y, IR = x.
    for (k = 0; k < n_ir_buffer_length; k++) {
        an_x[k] = static_cast<int32_t>(pun_ir_buffer[k]);
        an_y[k] = static_cast<int32_t>(pun_red_buffer[k]);
    }

    n_exact_ir_valley_locs_count = n_npks;

    n_ratio_average = 0;
    n_i_ratio_count = 0;

    for (k = 0; k < 5; k++) {
        an_ratio[k] = 0;
    }

    for (k = 0; k < n_exact_ir_valley_locs_count; k++) {
        if (an_ir_valley_locs[k] >= MAXIM_SPO2_BUFFER_SIZE) {
            *pn_spo2 = -999;
            *pch_spo2_valid = 0;
            return;
        }
    }

    // Find max between two valley locations and calculate RED/IR AC/DC ratio.
    for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++) {
        n_y_dc_max = -16777216;
        n_x_dc_max = -16777216;

        if (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k] > 3) {
            for (i = an_ir_valley_locs[k]; i < an_ir_valley_locs[k + 1]; i++) {
                if (an_x[i] > n_x_dc_max) {
                    n_x_dc_max = an_x[i];
                    n_x_dc_max_idx = i;
                }

                if (an_y[i] > n_y_dc_max) {
                    n_y_dc_max = an_y[i];
                    n_y_dc_max_idx = i;
                }
            }

            // RED AC.
            n_y_ac =
                (an_y[an_ir_valley_locs[k + 1]] - an_y[an_ir_valley_locs[k]]) *
                (n_y_dc_max_idx - an_ir_valley_locs[k]);

            n_y_ac =
                an_y[an_ir_valley_locs[k]] +
                n_y_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);

            n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;

            // IR AC.
            n_x_ac =
                (an_x[an_ir_valley_locs[k + 1]] - an_x[an_ir_valley_locs[k]]) *
                (n_x_dc_max_idx - an_ir_valley_locs[k]);

            n_x_ac =
                an_x[an_ir_valley_locs[k]] +
                n_x_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);

            n_x_ac = an_x[n_x_dc_max_idx] - n_x_ac;

            n_nume = (n_y_ac * n_x_dc_max) >> 7;
            n_denom = (n_x_ac * n_y_dc_max) >> 7;

            if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0) {
                an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom;
                n_i_ratio_count++;
            }
        }
    }

    if (n_i_ratio_count <= 0) {
        *pn_spo2 = -999;
        *pch_spo2_valid = 0;
        return;
    }

    maxim_sort_ascend(an_ratio, n_i_ratio_count);

    n_middle_idx = n_i_ratio_count / 2;

    if (n_middle_idx > 1) {
        n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2;
    } else {
        n_ratio_average = an_ratio[n_middle_idx];
    }

    if (n_ratio_average > 2 && n_ratio_average < 184) {
        n_spo2_calc = uch_spo2_table[n_ratio_average];
        *pn_spo2 = n_spo2_calc;
        *pch_spo2_valid = 1;
    } else {
        *pn_spo2 = -999;
        *pch_spo2_valid = 0;
    }
}

void maxim_find_peaks(
    int32_t* pn_locs,
    int32_t* n_npks,
    int32_t* pn_x,
    int32_t n_size,
    int32_t n_min_height,
    int32_t n_min_distance,
    int32_t n_max_num
)
{
    maxim_peaks_above_min_height(
        pn_locs,
        n_npks,
        pn_x,
        n_size,
        n_min_height
    );

    maxim_remove_close_peaks(
        pn_locs,
        n_npks,
        pn_x,
        n_min_distance
    );

    *n_npks = maxim_min_i32(*n_npks, n_max_num);
}

void maxim_peaks_above_min_height(
    int32_t* pn_locs,
    int32_t* n_npks,
    int32_t* pn_x,
    int32_t n_size,
    int32_t n_min_height
)
{
    int32_t i = 1;
    int32_t n_width = 0;

    *n_npks = 0;

    while (i < n_size - 1) {
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1]) {
            n_width = 1;

            while ((i + n_width < n_size) && (pn_x[i] == pn_x[i + n_width])) {
                n_width++;
            }

            if ((i + n_width < n_size) &&
                pn_x[i] > pn_x[i + n_width] &&
                (*n_npks) < 15)
            {
                pn_locs[(*n_npks)++] = i;
                i += n_width + 1;
            } else {
                i += n_width;
            }
        } else {
            i++;
        }
    }
}

void maxim_remove_close_peaks(
    int32_t* pn_locs,
    int32_t* pn_npks,
    int32_t* pn_x,
    int32_t n_min_distance
)
{
    int32_t i = 0;
    int32_t j = 0;
    int32_t n_old_npks = 0;
    int32_t n_dist = 0;

    maxim_sort_indices_descend(pn_x, pn_locs, *pn_npks);

    for (i = -1; i < *pn_npks; i++) {
        n_old_npks = *pn_npks;
        *pn_npks = i + 1;

        for (j = i + 1; j < n_old_npks; j++) {
            n_dist = pn_locs[j] - ((i == -1) ? -1 : pn_locs[i]);

            if (n_dist > n_min_distance || n_dist < -n_min_distance) {
                pn_locs[(*pn_npks)++] = pn_locs[j];
            }
        }
    }

    maxim_sort_ascend(pn_locs, *pn_npks);
}

void maxim_sort_ascend(int32_t* pn_x, int32_t n_size)
{
    int32_t i = 0;
    int32_t j = 0;
    int32_t n_temp = 0;

    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];

        for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--) {
            pn_x[j] = pn_x[j - 1];
        }

        pn_x[j] = n_temp;
    }
}

void maxim_sort_indices_descend(
    int32_t* pn_x,
    int32_t* pn_indx,
    int32_t n_size
)
{
    int32_t i = 0;
    int32_t j = 0;
    int32_t n_temp = 0;

    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];

        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--) {
            pn_indx[j] = pn_indx[j - 1];
        }

        pn_indx[j] = n_temp;
    }
}