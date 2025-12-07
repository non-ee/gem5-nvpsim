#include "dma_ctrl.hh"
#include <cstdint>


DmaCtrl::TickEvent::TickEvent(DmaCtrl *c) : ctrl(c) {}

DmaCtrl::TickEvent::process() {
    assert(ctrl);
    ctrl->tick();
}

const char *DmaCtrl::TickEvent::description() const {
    return "DMA Tick Event";
}

void DmaCtrl::tick() {
    Tick latency = clockPeriod();
    double EngyConsume = 0;

    if (active)
        EngyConsume = energy_per_tx;

    EnergyObject::consumeEnergy(ctrl->name(), EngyConsume);
    DPRINTF(EnergyMgmt, "DmaCtrl consumed %f energy\n", EngyConsume);
    schedule(tickEvent, curTick() + latency);
}

void DmaCtrl::handleMsg(const EnergyMsg& msg) {
    if (msg.type == SimpleEnergySM::MsgType::POWER_OFF) {
        active = false;
        if (readEvent.scheduled())
            deschedule(readEvent);
        if (writeEvent.scheduled())
            deschedule(writeEvent);
    }
    else if (msg.type == SimpleEnergySM::MsgType::POWER_ON) {
        active = false;
        if (!readEvent.scheduled())
            schedule(readEvent, clockEdge(Cycle(1)));
        if (!writeEvent.scheduled())
            schedule(writeEvent, clockEdge(Cycle(1)));
    }
}

DmaCtrl::DmaCtrl(const DmaCtrlParams *p)
    : SimObject(p),
      mem(nullptr),
      bandwidth(p->bandwidth),
      energy_per_tx(p->energy_per_tx),
      readEvent(this, false, Accelerator_DMA_Pri),
      writeEvent(this, false, Accelerator_DMA_Pri),
      active(false)
{
    readTask = {};
    writeTask = {};
}

DmaCtrl::~DmaCtrl() {
    if (mem)
        delete mem;
    if (tickEvent.scheduled())
        deschedule(tickEvent);
}

void DmaCtrl::init() {
    if (!tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycle(0)));
}

void DmaCtrl::setMemoryInterface(MemoryInterface* m) {
    mem = m;
}

void DmaCtrl::startRead(Addr addr, uint8_t* buf, size_t size, DmaCallback* cb)
{
    active = true;
    readTask = {addr, buf, size, cb};
    schedule(readEvent, clockEdge(Cycle(1)));
}

void DmaCtrl::startWrite(Addr addr, const uint8_t* buf, size_t size, DmaCallback* cb)
{
    active = true;
    writeTask = {addr, buf, size, cb};
    schedule(writeEvent, clockEdge(Cycle(1)));
}

void DmaCtrl::doRead() {
    auto &t = readTask;

    if (t.sizeLeft == 0) {
        if (t.cb) {
            active = false;
            t.cb->onDmaReadDone();
        }
        return;
    }

    size_t chunk = std::min(bandwidth, t.sizeLeft);
    mem->read(t.addr, t.buf, chunk);

    t.addr += chunk;
    t.buf += chunk;
    t.sizeLeft -= chunk;

    schedule(readEvent, clockEdge(Cycle(1)));
}

void DmaCtrl::doWrite() {
    auto &t = writeTask;

    if (t.sizeLeft == 0) {
        if (t.cb) {
            active = false;
            writeTask.cb->onDmaWriteDone();
        }
        return;
    }

    size_t chunk = std::min(bandwidth, t.sizeLeft);
    mem->write(t.addr, t.buf, chunk);

    t.addr += chunk;
    t.buf += chunk;
    t.sizeLeft -= chunk;

    schedule(writeEvent, clockEdge(Cycle(1)));
}

DmaCtrl* DmaCtrl::create(const DmaCtrlParams &p)
{
    return new DmaCtrl(p);
}
