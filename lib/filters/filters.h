#ifndef FILTERS_H
#define FILTERS_H

// Low-pass filter function
static inline float lowPassFilter(float current, float previous, float alpha) {
    return alpha * current + (1 - alpha) * previous;
}

static inline float highPassFilter(float x, float x_prev, float y_prev, float alpha) {
    return alpha * (y_prev + x - x_prev);
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