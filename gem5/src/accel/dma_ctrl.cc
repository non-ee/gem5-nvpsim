#include "accel/dma_ctrl.hh"
#include "accel/mem_if.hh"
#include "mem/se_translating_port_proxy.hh"
#include "cpu/thread_context.hh"
#include "engy/state_machine.hh"
#include "debug/EnergyMgmt.hh"
#include "debug/DmaCtrl.hh"
#include <cstdint>


DmaCtrl::TickEvent::TickEvent(DmaCtrl *c)
    : Event(Accelerator_Tick_Pri), ctrl(c) {}

void DmaCtrl::TickEvent::process() {
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

    char devname[100] = "DmaCtrl";
    EnergyObject::consumeEnergy(devname, EngyConsume);
    DPRINTF(EnergyMgmt, "DmaCtrl consumed %f energy\n", EngyConsume);
    schedule(tickEvent, curTick() + latency);
}

int DmaCtrl::handleMsg(const EnergyMsg& msg) {
    if (msg.type == SimpleEnergySM::MsgType::POWER_OFF) {
        if (!active) return 1;

        DPRINTF(DmaCtrl, "Powering off DMA controller\n");
        active = false;
        if (readEvent.scheduled())
            deschedule(readEvent);
        if (writeEvent.scheduled())
            deschedule(writeEvent);
    }
    else if (msg.type == SimpleEnergySM::MsgType::POWER_ON) {
        if (active) return 1;

        DPRINTF(DmaCtrl, "Powering on DMA controller\n");
        active = true;
        if (!readEvent.scheduled())
            schedule(readEvent, clockEdge(Cycles(1)));
        if (!writeEvent.scheduled())
            schedule(writeEvent, clockEdge(Cycles(1)));
    }
    else {
        DPRINTF(EnergyMgmt, "Unrecognized MsgType!\n");
        return 0;
    }

    return 1;
}

DmaCtrl::DmaCtrl(const DmaCtrlParams *p)
    : ClockedObject(p),
      tickEvent(this),
      cpu(p->cpu),
      portProxy(nullptr),
      mem(nullptr),
      bandwidth(p->bandwidth),
      energy_per_tx(p->energy_per_tx),
      active(false),
      readEvent(this, false, Event::Accelerator_DMA_Pri),
      writeEvent(this, false, Event::Accelerator_DMA_Pri),
      debug_io(p->debug_io)
{
    portProxy = new SETranslatingPortProxy(
        cpu->getDataPort(),
        cpu->getContext(0)->getProcessPtr(),
        SETranslatingPortProxy::AllocType::Never
    );
    mem = new AccelMemInterface(portProxy);
}

DmaCtrl::~DmaCtrl() {
    if (portProxy)
        delete portProxy;
    if (mem)
        delete mem;
    if (tickEvent.scheduled())
        deschedule(tickEvent);
}

void DmaCtrl::init() {
    DPRINTF(DmaCtrl, "Initializing DmaCtrl\n");

    if (portProxy) {
        DPRINTF(DmaCtrl, "Port proxy created successfully\n");
    } else {
        panic("DmaCtrl::init(): failed to create port proxy\n");
    }

    if (mem) {
        DPRINTF(DmaCtrl, "Memory interface created successfully\n");
    } else {
        panic("DmaCtrl::init(): failed to create memory interface\n");
    }

    if (!tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(0)));
}

void DmaCtrl::startRead(Addr addr, uint8_t* buf, size_t size, DmaCallBack* cb)
{
    DPRINTF(DmaCtrl, "Starting read from address %lx\n", addr);

    active = true;
    readTask = DmaTask(addr, buf, size, cb);
    schedule(readEvent, clockEdge(Cycles(1)));
}

void DmaCtrl::startWrite(Addr addr, uint8_t* buf, size_t size, DmaCallBack* cb)
{
    DPRINTF(DmaCtrl, "Starting read from address %lx\n", addr);
    active = true;
    writeTask = DmaTask(addr, buf, size, cb);
    schedule(writeEvent, clockEdge(Cycles(1)));
}

void DmaCtrl::doRead() {
    auto &t = readTask;

    if (!mem) {
        panic("DmaCtrl::doRead(): memory interface not set (mem == nullptr)\n");
    }

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

    if (debug_io) {
        DPRINTF(DmaCtrl, "Read %lu bytes from address %lx\n", chunk, t.addr);
    }

    schedule(readEvent, clockEdge(Cycles(1)));
}

void DmaCtrl::doWrite() {
    auto &t = writeTask;

    if (!mem) {
        panic("DmaCtrl::doWrite(): memory interface not set (mem == nullptr)\n");
    }

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

    schedule(writeEvent, clockEdge(Cycles(1)));
}

DmaCtrl* DmaCtrlParams::create()
{
    return new DmaCtrl(this);
}
