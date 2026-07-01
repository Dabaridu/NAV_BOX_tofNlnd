#include "running_average.h"
#include "main.h"

/*
	* running_average.c
	*
	*  Created on: Jun 6, 2024

	this is a...
*/

void RA_set(double *data_in, uint32_t window_size, RunningAverage *filter){
	if(window_size > MAX_WINDOW_SIZE){
		return;
	}
	filter->data_input_address = data_in;
	filter->window_size = window_size;
}

void RA_update(RunningAverage *filter)
{
	// Shift the data buffer to the left by one position
	for (uint32_t i = 0; i < filter->window_size - 1; i++) {
		filter->data_buffer[i] = filter->data_buffer[i + 1];
	}

	// Add the new data point to the end of the buffer
	filter->data_buffer[filter->window_size - 1] = *(filter->data_input_address);

	// Calculate the running average
	double sum = 0.0;
	for (uint32_t i = 0; i < filter->window_size; i++) {
		sum += filter->data_buffer[i];
	}
	filter->data_output = sum / filter->window_size;
}

