#pragma once
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "base/types.hh"
#include <cstdint>

struct ComputeCallBack {
    virtual void onComputeDone() = 0;
    virtual void onComputeAbort() {};
    virtual ~ComputeCallBack() = default;
};

struct ComputeTask {
    uint8_t* input;
    uint8_t* output;
    uint32_t size;
    ComputeCallBack* cb;

    virtual void compute() = 0;

    ComputeTask() : input(nullptr), output(nullptr), size(0), cb(nullptr) {}
    ComputeTask(uint8_t* input, uint8_t* output, uint32_t size, ComputeCallBack* cb)
        : input(input), output(output), size(size), cb(cb) {}
};

class ComputeUnit : public EventManager {
    private:
        ComputeTask task;
        Tick latency;

    public:
        ComputeUnit(EventQueue* eq, Tick latency)
            : EventManager(eq),
              task(),
              latency(latency),
              event_compute(this, false, Event::Accelerator_Compute_Done_Pri)
            {}

        const std::string name() const {
            return "ComputeUnit";
        }

        void compute() {
            auto& t = task;

            // Implement compute logic here
            for (uint32_t i = 0; i < t.size; i++) {
                uint32_t x = t.input[i];

                for (uint32_t j = 0; j < 100; j++)
                    x = (x * 17 + j) % 256;

                t.output[i] = x;
            }

            if (t.cb)
                t.cb->onComputeDone();
        }
        void abort() {
            if (event_compute.scheduled())
                deschedule(event_compute);
        }

        void startCompute(uint8_t* input, uint8_t* output, uint32_t size, ComputeCallBack* cb) {
            task.input = input;
            task.output = output;
            task.size = size;
            task.cb = cb;

            schedule(event_compute, curTick() + latency);
        }

    private:
        EventWrapper<ComputeUnit, &ComputeUnit::compute> event_compute;
};
