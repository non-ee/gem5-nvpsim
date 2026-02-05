#include "accel/dma_ctrl.hh"
#include "accel.hh"
#include "accel/mem_if.hh"
#include "dma_ctrl.hh"
#include "mem/se_translating_port_proxy.hh"
#include "cpu/thread_context.hh"
#include "engy/state_machine.hh"
#include "debug/EnergyMgmt.hh"
#include "debug/DmaCtrl.hh"
#include "debug/Accelerator.hh"
#include <cstdint>
#include <stdio.h>


DmaCtrl::DmaCtrl(const DmaCtrlParams *p)
    : ClockedObject(p),
      cpu(p->cpu),
      portProxy(nullptr),
      mem(nullptr),
      latency_access_per_byte(p->latency_access_per_byte),
      energy_access_per_byte(p->energy_access_per_byte),
      dmaTask(),
      dmaEvent(this, false, Event::Accelerator_DMA_Pri)
{
    inTask = false;
    access_latency = 0;

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
}

void DmaCtrl::startRead(Addr addr, uint8_t* buf, size_t size, std::function<void()> cb)
{
    DPRINTF(DmaCtrl, "Start reading from address %lx\n", addr);
    inTask = true;
    dmaTask = DmaTask(addr, buf, size, cb, READ);

    access_latency = size * latency_access_per_byte;
    schedule(dmaEvent, curTick() + access_latency);
    DPRINTF(DmaCtrl, "Memory access need LAT=%i\n", access_latency);
}

void DmaCtrl::startWrite(Addr addr, uint8_t* buf, size_t size, std::function<void()> cb)
{
    DPRINTF(DmaCtrl, "Start writing from address %lx\n", addr);
    inTask = true;
    dmaTask = DmaTask(addr, buf, size, cb, WRITE);

    access_latency = size * latency_access_per_byte;
    schedule(dmaEvent, curTick() + access_latency);
    DPRINTF(DmaCtrl, "Memory access need LAT=%i\n", access_latency);
}

void DmaCtrl::abortDma() {
    DPRINTF(DmaCtrl, "Aborting memory access ...\n");
    deschedule(dmaEvent);
}

void DmaCtrl::doDma() {
    auto &t = dmaTask;

    if (t.op == READ)
        mem->read(t.addr, t.buf, t.size);
    else if (t.op == WRITE)
        mem->write(t.addr, t.buf, t.size);
    else
        panic("Invalid state");

    if (t.cb)
        t.cb();
    inTask = false;

    DPRINTF(DmaCtrl, "Access to %lu bytes from address %lx\n", t.size, t.addr);

    char devname[100] = "DmaCtrl";
    double energy_access = t.size * energy_access_per_byte;
    EnergyObject::consumeEnergy(devname, energy_access);
    DPRINTF(Accelerator, "Memory access consumed %f energy\n", energy_access);
}

int DmaCtrl::handleMsg(const EnergyMsg& msg) {
    // if (msg.type == SimpleEnergySM::MsgType::POWER_OFF) {
    //     DPRINTF(DmaCtrl, "Powering off ...\n");
    //     if (dmaEvent.scheduled()) {
    //         DPRINTF(DmaCtrl, "Aborting memory access..\n");
    //         deschedule(dmaEvent);
    //     }
    // }
    // else if (msg.type == SimpleEnergySM::MsgType::POWER_ON) {
    //     DPRINTF(DmaCtrl, "Powering on ...\n");
    //     if (inTask) {
    //         DPRINTF(DmaCtrl, "Rescheduling memory access...\n");
    //         schedule(dmaEvent, curTick() + access_latency);
    //     }
    // }
    // else {
    //     DPRINTF(EnergyMgmt, "Unrecognized MsgType!\n");
    //     return 0;
    // }

    return 1;
}

DmaCtrl* DmaCtrlParams::create()
{
    return new DmaCtrl(this);
}
