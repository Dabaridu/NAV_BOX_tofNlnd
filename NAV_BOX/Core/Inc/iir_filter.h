#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#include <stdint.h>
#include <math.h>

/**
 * @file iir_filter.h
 * @brief IIR (Infinite Impulse Response) Low-Pass Filter Library
 * @details Provides first and second-order low-pass filters optimized for accelerometer filtering
 */

/* ======================== First-Order IIR Filter ======================== */

/**
 * @struct IIR_LPF_1st
 * @brief First-order IIR low-pass filter structure
 */
typedef struct {
    float alpha;              /**< Filter coefficient (0 to 1) */
    float filtered_value;     /**< Current filtered output */
} IIR_LPF_1st;

/**
 * @brief Initialize first-order IIR low-pass filter
 * @param filter Pointer to filter structure
 * @param cutoff_freq Cutoff frequency (Hz)
 * @param sample_rate Sampling frequency (Hz)
 */
void IIR_LPF_1st_Init(IIR_LPF_1st *filter, float cutoff_freq, float sample_rate);

/**
 * @brief Apply first-order IIR low-pass filter
 * @param filter Pointer to filter structure
 * @param raw_value Raw input value
 * @return Filtered output value
 */
float IIR_LPF_1st_Apply(IIR_LPF_1st *filter, float raw_value);

/**
 * @brief Reset filter to initial state
 * @param filter Pointer to filter structure
 */
void IIR_LPF_1st_Reset(IIR_LPF_1st *filter);

/* ======================== Second-Order IIR Filter ======================== */

/**
 * @struct IIR_LPF_2nd
 * @brief Second-order IIR low-pass filter structure (Butterworth)
 */
typedef struct {
    float alpha;              /**< Primary filter coefficient */
    float beta;               /**< Secondary filter coefficient */
    float y1;                 /**< Previous output value */
    float y2;                 /**< Previous previous output value */
    float x1;                 /**< Previous input value */
    float x2;                 /**< Previous previous input value */
} IIR_LPF_2nd;

/**
 * @brief Initialize second-order IIR low-pass filter (Butterworth)
 * @param filter Pointer to filter structure
 * @param cutoff_freq Cutoff frequency (Hz)
 * @param sample_rate Sampling frequency (Hz)
 */
void IIR_LPF_2nd_Init(IIR_LPF_2nd *filter, float cutoff_freq, float sample_rate);

/**
 * @brief Apply second-order IIR low-pass filter
 * @param filter Pointer to filter structure
 * @param raw_value Raw input value
 * @return Filtered output value
 */
float IIR_LPF_2nd_Apply(IIR_LPF_2nd *filter, float raw_value);

/**
 * @brief Reset filter to initial state
 * @param filter Pointer to filter structure
 */
void IIR_LPF_2nd_Reset(IIR_LPF_2nd *filter);

/* ======================== Utilities ======================== */

/**
 * @brief Calculate alpha coefficient from cutoff and sample frequency
 * @param cutoff_freq Cutoff frequency (Hz)
 * @param sample_rate Sampling frequency (Hz)
 * @return Alpha coefficient value
 */
float IIR_CalculateAlpha(float cutoff_freq, float sample_rate);

#endif // IIR_FILTER_H
