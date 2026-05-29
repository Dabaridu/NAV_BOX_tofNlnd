#include "iir_filter.h"
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ======================== First-Order IIR Filter Implementation ======================== */

float IIR_CalculateAlpha(float cutoff_freq, float sample_rate) {
    if (cutoff_freq <= 0 || sample_rate <= 0) {
        return 0.5f;  // Default safe value
    }
    
    float dt = 1.0f / sample_rate;
    float wc = 2.0f * M_PI * cutoff_freq;
    float alpha = (wc * dt) / (wc * dt + 1.0f);
    
    return alpha;
}

void IIR_LPF_1st_Init(IIR_LPF_1st *filter, float cutoff_freq, float sample_rate) {
    if (filter == NULL) return;
    
    filter->alpha = IIR_CalculateAlpha(cutoff_freq, sample_rate);
    filter->filtered_value = 0.0f;
}

float IIR_LPF_1st_Apply(IIR_LPF_1st *filter, float raw_value) {
    if (filter == NULL) {
        return raw_value;
    }
    
    // y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
    filter->filtered_value = filter->alpha * raw_value + 
                             (1.0f - filter->alpha) * filter->filtered_value;
    
    return filter->filtered_value;
}

void IIR_LPF_1st_Reset(IIR_LPF_1st *filter) {
    if (filter == NULL) return;
    
    filter->filtered_value = 0.0f;
}

/* ======================== Second-Order IIR Filter Implementation ======================== */

void IIR_LPF_2nd_Init(IIR_LPF_2nd *filter, float cutoff_freq, float sample_rate) {
    if (filter == NULL) return;
    
    if (cutoff_freq <= 0 || sample_rate <= 0) {
        memset(filter, 0, sizeof(IIR_LPF_2nd));
        return;
    }
    
    // Butterworth second-order low-pass filter coefficients
    float dt = 1.0f / sample_rate;
    float wc = 2.0f * M_PI * cutoff_freq;
    float wc_dt = wc * dt;
    
    // Normalized coefficients for Butterworth response
    float sqrt2 = 1.41421356237f;
    float denominator = wc_dt * wc_dt + sqrt2 * wc_dt + 1.0f;
    
    filter->alpha = (wc_dt * wc_dt) / denominator;
    filter->beta = (2.0f * wc_dt * wc_dt) / denominator;
    
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
}

float IIR_LPF_2nd_Apply(IIR_LPF_2nd *filter, float raw_value) {
    if (filter == NULL) {
        return raw_value;
    }
    
    // Second-order difference equation
    // y[n] = 2*y[n-1] - y[n-2] + alpha*(x[n] + 2*x[n-1] + x[n-2])
    float output = 2.0f * filter->y1 - filter->y2 + 
                   filter->alpha * (raw_value + 2.0f * filter->x1 + filter->x2);
    
    // Update history
    filter->y2 = filter->y1;
    filter->y1 = output;
    filter->x2 = filter->x1;
    filter->x1 = raw_value;
    
    return output;
}

void IIR_LPF_2nd_Reset(IIR_LPF_2nd *filter) {
    if (filter == NULL) return;
    
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
}
