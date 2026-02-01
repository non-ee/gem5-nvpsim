#pragma once
#include "sim/sim_object.hh"
#include "params/BaseComputeUnit.hh"
#include "params/SimpleComputeUnit.hh"
#include "params/HAR_Accelerator.hh"
#include "params/ImageProcessingUnit.hh"
#include "sim/eventq.hh"
#include "base/types.hh"
#include <cstdint>

// HAR macros
#define SAMPLE_COUNT 20          // 2 seconds at 10Hz sampling (since SAMPLE_COUNT=20)
#define ACCEL_DATA_DIM 3         // x, y, z axes
#define MOVING_AVG_WINDOW 5      // For moving average filter
#define STEP_THRESHOLD 1.5       // Step detection threshold multiplier
#define MIN_STEP_MAGNITUDE 1.2   // Minimum acceleration magnitude for step


// Filter macros
#define IMAGE_WIDTH         8    // Image width in pixels
#define IMAGE_HEIGHT        8    // Image height in pixels
#define IMAGE_SIZE          (IMAGE_WIDTH * IMAGE_HEIGHT)
#define KERNEL_SIZE         3      // Convolution kernel size (3x3)
#define EDGE_THRESHOLD      50     // Edge detection threshold
#define NOISE_THRESHOLD     20     // Noise filtering threshold

struct ComputeCallBack {
    virtual void onComputeDone() = 0;
    virtual void onComputeAbort() = 0;
    virtual ~ComputeCallBack() = default;
};

class BaseComputeUnit : public SimObject {
    public:
        typedef BaseComputeUnitParams Params;
        const Params *params() const {
            return reinterpret_cast<const Params*>(_params);
        }

        BaseComputeUnit(const Params *p);
        virtual ~BaseComputeUnit() = default;
        virtual void init() {}
        virtual void start(uint8_t* input, uint8_t* output, uint32_t input_count, uint32_t output_count, ComputeCallBack* cb) {}
        virtual void compute() {}
        virtual void abort() {}

    protected:
        uint8_t* input;
        uint8_t* output;
        uint32_t input_count;
        uint32_t output_count;
        ComputeCallBack* cb;
};

class SimpleComputeUnit : public BaseComputeUnit {
    public:
        typedef SimpleComputeUnitParams Params;
        const Params *params() const {
            return reinterpret_cast<const Params*>(_params);
        }

        SimpleComputeUnit(const Params *p);
        virtual ~SimpleComputeUnit() = default;
        virtual void init();
        virtual void start(uint8_t* input, uint8_t* output, uint32_t input_count, uint32_t output_count, ComputeCallBack* cb);
        virtual void compute();
        virtual void abort();

    private:
        Tick latency;
        EventWrapper<SimpleComputeUnit, &SimpleComputeUnit::compute> event_compute;
};

class HAR_Accelerator : public SimpleComputeUnit {
    public:
    typedef HAR_AcceleratorParams Params;
    const Params *params() const {
        return reinterpret_cast<const Params*>(_params);
    }

    HAR_Accelerator(const Params *p);
    virtual void compute() override;
};


class ImageProcessingUnit : public SimpleComputeUnit {
    public:
    typedef ImageProcessingUnitParams Params;
    const Params *params() const {
        return reinterpret_cast<const Params*>(_params);
    }

    ImageProcessingUnit(const Params *p);
    virtual void compute() override;
};
