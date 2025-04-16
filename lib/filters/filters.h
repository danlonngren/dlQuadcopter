#ifndef FILTERS_H
#define FILTERS_H

// Low-pass filter function
static inline float LowPassFilter(float input, float *output, float alpha) {
    *output += alpha * (input - *output);
    return *output;
}

// Moving average filter function
static inline float MovingAverageFilter(float input, float *buffer, int bufferSize, int *index, float *sum) {
    // Subtract the oldest value from the sum
    *sum -= buffer[*index];

    // Add the new input value to the buffer and sum
    buffer[*index] = input;
    *sum += input;

    // Increment the index and wrap around if necessary
    *index = (*index + 1) % bufferSize;

    // Return the average
    return *sum / bufferSize;
}

#endif