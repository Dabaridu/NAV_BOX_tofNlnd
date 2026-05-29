/**
 * @file iir_filter_example.c
 * @brief Examples of how to use the IIR filter library for accelerometer filtering
 */

#include "iir_filter.h"
#include "bno055.h"
#include <stdio.h>

/* ======================== Example 1: Simple First-Order Filter ======================== */

// Filter for each accelerometer axis
IIR_LPF_1st accel_x_filter;
IIR_LPF_1st accel_y_filter;
IIR_LPF_1st accel_z_filter;

/**
 * @brief Initialize accelerometer filters (first-order)
 * 
 * For a 100 Hz accelerometer with 10 Hz cutoff frequency:
 * - Removes high-frequency noise
 * - Minimal latency
 * - Good for real-time applications
 */
void Example1_InitFilters(void) {
    float cutoff_freq = 10.0f;    // Hz
    float sample_rate = 100.0f;   // Hz (BNO055 typical rate)
    
    IIR_LPF_1st_Init(&accel_x_filter, cutoff_freq, sample_rate);
    IIR_LPF_1st_Init(&accel_y_filter, cutoff_freq, sample_rate);
    IIR_LPF_1st_Init(&accel_z_filter, cutoff_freq, sample_rate);
}

/**
 * @brief Read and filter accelerometer data (first-order)
 */
void Example1_ReadFilteredAccel(float *accel_x, float *accel_y, float *accel_z) {
    // Read raw accelerometer data from BNO055
    float raw_x, raw_y, raw_z;
    BNO055_ReadAccel(&raw_x, &raw_y, &raw_z);  // Your BNO055 function
    
    // Apply first-order low-pass filter to each axis
    *accel_x = IIR_LPF_1st_Apply(&accel_x_filter, raw_x);
    *accel_y = IIR_LPF_1st_Apply(&accel_y_filter, raw_y);
    *accel_z = IIR_LPF_1st_Apply(&accel_z_filter, raw_z);
}

/* ======================== Example 2: Second-Order Filter (Butterworth) ======================== */

// Second-order filters for smoother response
IIR_LPF_2nd accel_x_filter_2nd;
IIR_LPF_2nd accel_y_filter_2nd;
IIR_LPF_2nd accel_z_filter_2nd;

/**
 * @brief Initialize accelerometer filters (second-order Butterworth)
 * 
 * For steeper rolloff (-40dB/decade instead of -20dB/decade):
 * - Better noise attenuation
 * - Slightly more latency than first-order
 * - Still suitable for real-time navigation
 */
void Example2_InitFilters(void) {
    float cutoff_freq = 15.0f;    // Hz
    float sample_rate = 100.0f;   // Hz
    
    IIR_LPF_2nd_Init(&accel_x_filter_2nd, cutoff_freq, sample_rate);
    IIR_LPF_2nd_Init(&accel_y_filter_2nd, cutoff_freq, sample_rate);
    IIR_LPF_2nd_Init(&accel_z_filter_2nd, cutoff_freq, sample_rate);
}

/**
 * @brief Read and filter accelerometer data (second-order)
 */
void Example2_ReadFilteredAccel(float *accel_x, float *accel_y, float *accel_z) {
    float raw_x, raw_y, raw_z;
    BNO055_ReadAccel(&raw_x, &raw_y, &raw_z);
    
    // Apply second-order low-pass filter to each axis
    *accel_x = IIR_LPF_2nd_Apply(&accel_x_filter_2nd, raw_x);
    *accel_y = IIR_LPF_2nd_Apply(&accel_y_filter_2nd, raw_y);
    *accel_z = IIR_LPF_2nd_Apply(&accel_z_filter_2nd, raw_z);
}

/* ======================== Example 3: Multi-Rate Filtering ======================== */

// Use different cutoff frequencies for different purposes
IIR_LPF_1st accel_vibration_filter;   // High cutoff: 30Hz - Pass vibrations
IIR_LPF_1st accel_gravity_filter;     // Low cutoff: 5Hz - Only gravity component

/**
 * @brief Initialize multi-rate filters
 * 
 * Use case:
 * - Vibration filter: Fast response for acceleration changes
 * - Gravity filter: Slow response for DC bias/gravity measurement
 */
void Example3_InitFilters(void) {
    float sample_rate = 100.0f;
    
    // High-pass characteristics (through subtraction)
    IIR_LPF_1st_Init(&accel_vibration_filter, 30.0f, sample_rate);
    
    // Low-pass for gravity
    IIR_LPF_1st_Init(&accel_gravity_filter, 5.0f, sample_rate);
}

/**
 * @brief Separate vibration from gravity
 */
void Example3_SeparateComponents(float raw_accel, float *vibration, float *gravity) {
    float filtered_vibration = IIR_LPF_1st_Apply(&accel_vibration_filter, raw_accel);
    float filtered_gravity = IIR_LPF_1st_Apply(&accel_gravity_filter, raw_accel);
    
    // Vibration = high-pass = raw - low-pass
    *vibration = raw_accel - filtered_gravity;
    *gravity = filtered_gravity;
}

/* ======================== Example 4: Integration in Main Loop ======================== */

void Example4_MainLoop(void) {
    // Initialize
    Example1_InitFilters();
    
    // Main loop
    while(1) {
        float filtered_x, filtered_y, filtered_z;
        
        // Read filtered accelerometer data
        Example1_ReadFilteredAccel(&filtered_x, &filtered_y, &filtered_z);
        
        // Calculate magnitude for activity detection
        float accel_magnitude = sqrtf(filtered_x * filtered_x + 
                                      filtered_y * filtered_y + 
                                      filtered_z * filtered_z);
        
        // Use in sensor fusion or navigation
        // ekfNavINS_Update(filtered_x, filtered_y, filtered_z);
        
        // Log or transmit
        // printf("Accel: X=%.2f Y=%.2f Z=%.2f Mag=%.2f\n", 
        //        filtered_x, filtered_y, filtered_z, accel_magnitude);
        
        HAL_Delay(10);  // 100 Hz sampling
    }
}

/* ======================== Example 5: Dynamic Filter Adjustment ======================== */

IIR_LPF_1st adaptive_filter;
uint32_t motion_counter = 0;

/**
 * @brief Adapt filter cutoff based on motion level
 * 
 * Strategy:
 * - High motion (walking): Use higher cutoff (20Hz) for responsiveness
 * - Low motion (stationary): Use lower cutoff (5Hz) for stability
 */
void Example5_AdaptiveFilter(float raw_accel, float *filtered) {
    static float prev_accel = 0.0f;
    
    // Detect motion using differential
    float diff = fabsf(raw_accel - prev_accel);
    
    if (diff > 0.5f) {
        motion_counter = 50;  // 500ms motion window
    }
    
    // Adjust cutoff based on motion state
    float cutoff = (motion_counter > 0) ? 20.0f : 5.0f;
    
    // Reinitialize filter with new cutoff (optional: only do if cutoff changed significantly)
    static float last_cutoff = 10.0f;
    if (fabsf(cutoff - last_cutoff) > 2.0f) {
        IIR_LPF_1st_Init(&adaptive_filter, cutoff, 100.0f);
        last_cutoff = cutoff;
    }
    
    *filtered = IIR_LPF_1st_Apply(&adaptive_filter, raw_accel);
    
    if (motion_counter > 0) {
        motion_counter--;
    }
    
    prev_accel = raw_accel;
}

/* ======================== Example 6: Cascaded Filters ======================== */

// Use two filters in series for steeper rolloff without second-order complexity
IIR_LPF_1st cascade_filter_1;
IIR_LPF_1st cascade_filter_2;

/**
 * @brief Initialize cascaded first-order filters
 * 
 * Two first-order filters = ~second-order response
 * - Easier to tune than second-order
 * - More flexible for embedded systems
 */
void Example6_InitCascadedFilters(void) {
    float cutoff_freq = 10.0f;
    float sample_rate = 100.0f;
    
    IIR_LPF_1st_Init(&cascade_filter_1, cutoff_freq, sample_rate);
    IIR_LPF_1st_Init(&cascade_filter_2, cutoff_freq, sample_rate);
}

/**
 * @brief Apply cascaded filters
 */
float Example6_ApplyCascaded(float raw_value) {
    float stage1 = IIR_LPF_1st_Apply(&cascade_filter_1, raw_value);
    float stage2 = IIR_LPF_1st_Apply(&cascade_filter_2, stage1);
    return stage2;
}

/* ======================== Example 7: Filter Performance Metrics ======================== */

/**
 * @brief Calculate filter delay (group delay at DC)
 * 
 * First-order IIR delay: approximately 1/(2*pi*fc) seconds
 * Second-order IIR delay: approximately 2/(2*pi*fc) seconds
 */
float CalculateFilterDelay(float cutoff_freq, int order) {
    return (float)order / (2.0f * 3.14159f * cutoff_freq);
}

/**
 * @brief Example: Check latency for 10Hz cutoff
 */
void Example7_LatencyCheck(void) {
    float delay_1st = CalculateFilterDelay(10.0f, 1);   // ~0.016 seconds = 16ms
    float delay_2nd = CalculateFilterDelay(10.0f, 2);   // ~0.032 seconds = 32ms
    
    // printf("1st-order delay: %.2f ms\n", delay_1st * 1000.0f);
    // printf("2nd-order delay: %.2f ms\n", delay_2nd * 1000.0f);
}

/* ======================== Quick Reference ======================== */

/**
 * RECOMMENDED SETTINGS FOR ACCELEROMETERS:
 * 
 * Application                  Cutoff Freq    Order   Sample Rate
 * =====================================================================
 * Activity Recognition         5-10 Hz        1st     50-100 Hz
 * Vehicle Navigation          10-20 Hz        1st     100-200 Hz
 * Structural Monitoring        1-5 Hz         2nd     50 Hz
 * Vibration Analysis          15-50 Hz        1st     1000+ Hz
 * Step Detection               5-15 Hz        1st     50-100 Hz
 * 
 * QUICK START:
 * 1. Include: #include "iir_filter.h"
 * 2. Declare: IIR_LPF_1st accel_filter;
 * 3. Initialize: IIR_LPF_1st_Init(&accel_filter, 10.0f, 100.0f);
 * 4. Apply: filtered = IIR_LPF_1st_Apply(&accel_filter, raw_value);
 * 5. Reset if needed: IIR_LPF_1st_Reset(&accel_filter);
 */
