#include "accel/compute_unit.hh"
#include "compute_unit.hh"
#include "debug/ComputeUnit.hh"
#include <cstdint>
#include <cmath>
#include <stdint.h>

/** BaseComputeUnit **/
BaseComputeUnit::BaseComputeUnit(const Params *p)
    : SimObject(p)
{
    input = nullptr;
    output = nullptr;
    input_count = 0;
    output_count = 0;
    cb = nullptr;
}

/** SimpleComputeUnit **/
SimpleComputeUnit::SimpleComputeUnit(const Params *p)
    : BaseComputeUnit(p),
      latency(p->latency),
      event_compute(this, false, Event::Accelerator_Compute_Done_Pri)
{
}

HAR_Accelerator::HAR_Accelerator(const Params *p)
    : SimpleComputeUnit(p) {}

ImageProcessingUnit::ImageProcessingUnit(const Params *p)
    : SimpleComputeUnit(p) {}

void SimpleComputeUnit::init()
{
    // Implement init logic here
}

void SimpleComputeUnit::start(uint8_t* input, uint8_t* output, uint32_t input_count, uint32_t output_count, ComputeCallBack* cb)
{
    // Implement start logic here
    this->input = input;
    this->output = output;
    this->input_count = input_count;
    this->output_count = output_count;
    this->cb = cb;

    DPRINTF(ComputeUnit, "[ComputeUnit] scheduling computation.Need LAT = %i\n", latency);
    schedule(event_compute, curTick() + latency);
}

void SimpleComputeUnit::compute()
{
    DPRINTF(ComputeUnit, "[ComputeUnit] Performing computation...\n");
    // Implement compute logic here
    for (uint32_t i = 0; i < input_count; i++) {
        uint32_t x = input[i];

        for (uint32_t j = 0; j < 100; j++)
            x = (x * 17 + j) % 256;

        output[i] = x;
    }

    if (cb) {
        DPRINTF(ComputeUnit, "[ComputeUnit] Calling callback...\n");
        cb->onComputeDone();
    }
}

void SimpleComputeUnit::abort()
{
    DPRINTF(ComputeUnit, "[ComputeUnit] aborting computation...\n");
    // Implement abort logic here
    if (event_compute.scheduled()) {
        deschedule(event_compute);
        if (cb)
            cb->onComputeAbort();
    }
}


void HAR_Accelerator::compute() {
    DPRINTF(ComputeUnit, "[ComputeUnit] Performing HAR computation...\n");

    float magnitude_history[MOVING_AVG_WINDOW];
    int step_count = 0;
    int history_index = 0;

    // Initialize magnitude history buffer
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
        magnitude_history[i] = 0.0;
    }

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // Calculate magnitude of acceleration vector
        float x = input[i*ACCEL_DATA_DIM+0] / 1000.0;
        float y = input[i*ACCEL_DATA_DIM+1] / 1000.0;
        float z = input[i*ACCEL_DATA_DIM+2] / 1000.0;

        float acceleration_magnitude = sqrt(x*x + y*y + z*z);

        // Update moving average filter
        magnitude_history[history_index] = acceleration_magnitude;
        history_index = (history_index + 1) % MOVING_AVG_WINDOW;

        // Calculate moving average
        float moving_average = 0.0;
        for (int j = 0; j < MOVING_AVG_WINDOW; j++) {
            moving_average += magnitude_history[j];
        }
        moving_average /= MOVING_AVG_WINDOW;

        // Threshold-based step detection
        if (acceleration_magnitude > moving_average * STEP_THRESHOLD &&
            acceleration_magnitude > MIN_STEP_MAGNITUDE) {
            output[i] = 1;  // Step detected

            // Debouncing: only count if previous few samples were low
            if (i > 3) {
                int recent_steps = 0;
                for (int k = 1; k <= 3; k++) {
                    if (output[i - k] == 1) recent_steps++;
                }
                if (recent_steps == 0) {  // No recent steps
                    step_count++;
                }
            } else {
                step_count++;
            }
        } else {
            output[i] = 0;  // No step
        }
    }

    if (cb) {
        DPRINTF(ComputeUnit, "[ComputeUnit] Calling callback...\n");
        cb->onComputeDone();
    }
}
void ImageProcessingUnit::compute()
{
    // Validate
    if (input_count != 64 || output_count != 64 || !input || !output) return;

    // Process inner 6x6 region of 8x8 image
    // EXACT loop structure as convolution_kernel
    for (int y = 1; y < 7; y++) {
        for (int x = 1; x < 7; x++) {
            // EXACT index calculation as convolution_kernel
            int idx[9] = {
                (y-1)*8 + (x-1), (y-1)*8 + x, (y-1)*8 + (x+1),
                y*8 + (x-1),     y*8 + x,     y*8 + (x+1),
                (y+1)*8 + (x-1), (y+1)*8 + x, (y+1)*8 + (x+1)
            };

            // EXACT weights as convolution_kernel
            int weights[9] = {1, 2, 1, 2, 4, 2, 1, 2, 1};

            // EXACT MAC operations as convolution_kernel
            int sum = 0;
            for (int i = 0; i < 9; i++) {
                sum += input[idx[i]] * weights[i];
            }

            // EXACT normalization as convolution_kernel
            output[y*8 + x] = sum / 16;
        }
    }

    // Set border pixels (implicit in convolution_kernel, explicit here)
    for (int i = 0; i < 8; i++) {
        output[i] = 0;               // Top row
        output[56 + i] = 0;          // Bottom row
        output[i * 8] = 0;           // Left column
        output[i * 8 + 7] = 0;       // Right column
    }

    if (cb) {
           DPRINTF(ComputeUnit, "[ComputeUnit] Calling callback...\n");
           cb->onComputeDone();
       }
}

BaseComputeUnit*
BaseComputeUnitParams::create()
{
    return new BaseComputeUnit(this);
}

SimpleComputeUnit*
SimpleComputeUnitParams::create()
{
    return new SimpleComputeUnit(this);
}

HAR_Accelerator*
HAR_AcceleratorParams::create()
{
    return new HAR_Accelerator(this);
}

ImageProcessingUnit*
ImageProcessingUnitParams::create()
{
    return new ImageProcessingUnit(this);
}
