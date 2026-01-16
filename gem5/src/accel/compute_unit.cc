#include "accel/compute_unit.hh"
#include "compute_unit.hh"
#include "debug/ComputeUnit.hh"

/** BaseComputeUnit **/
BaseComputeUnit::BaseComputeUnit(const Params *p)
    : SimObject(p)
{
    input = nullptr;
    output = nullptr;
    size = 0;
    cb = nullptr;
}

/** SimpleComputeUnit **/
SimpleComputeUnit::SimpleComputeUnit(const Params *p)
    : BaseComputeUnit(p),
      latency(p->latency),
      event_compute(this, false, Event::Accelerator_Compute_Done_Pri)
{
}

void SimpleComputeUnit::init()
{
    // Implement init logic here
}

void SimpleComputeUnit::start(uint8_t* input, uint8_t* output, uint32_t size, ComputeCallBack* cb)
{
    // Implement start logic here
    this->input = input;
    this->output = output;
    this->size = size;
    this->cb = cb;

    DPRINTF(ComputeUnit, "[ComputeUnit] scheduling computation.Need LAT = %i\n", latency);
    schedule(event_compute, curTick() + latency);
}

void SimpleComputeUnit::compute()
{
    DPRINTF(ComputeUnit, "[ComputeUnit] Performing computation...\n");
    // Implement compute logic here
    for (uint32_t i = 0; i < size; i++) {
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
