#pragma once
#include "sim/sim_object.hh"
#include "params/BaseComputeUnit.hh"
#include "params/SimpleComputeUnit.hh"
#include "sim/eventq.hh"
#include "base/types.hh"
#include <cstdint>

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
