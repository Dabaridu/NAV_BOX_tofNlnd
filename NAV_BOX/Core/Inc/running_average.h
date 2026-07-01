#ifndef RUNNING_AVERAGE_H
#define RUNNING_AVERAGE_H

#include "main.h"

#define MAX_WINDOW_SIZE 256

typedef struct {
	double *data_input_address;
	double data_output;
	uint32_t window_size;
	double data_buffer[MAX_WINDOW_SIZE];
} RunningAverage;

void RA_set(double *data_in, uint32_t window_size, RunningAverage *filter);
void RA_update(RunningAverage *filter);

#endif // RUNNING_AVERAGE_H
