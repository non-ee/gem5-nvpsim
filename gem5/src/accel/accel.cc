#include "accel/accel.hh"
#include "accel.hh"
#include "debug/Accelerator.hh"
#include "debug/EnergyMgmt.hh"
#include "debug/MemoryAccess.hh"
#include "engy/state_machine.hh"
#include "base/trace.hh"
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <stdint.h>
#include <unistd.h>

/* -- TickEvent --- */
Accelerator::TickEvent::TickEvent(Accelerator *accel)
    : Event(Accelerator_Tick_Pri), owner(accel)
{
}

void Accelerator::TickEvent::process()
{
    assert(owner);
    owner->tick();
}

const char *Accelerator::TickEvent::description() const
{
    assert(owner);
    return "Accelerator Tick Event";
}

void Accelerator::tick()
{
    Tick latency = clockPeriod();
    double EngyConsume = 0;
    EngyConsume = energy_per_cycle[energy_state] * ticksToCycles(latency);

    char dev_name[100] = "Accelerator";
    EnergyObject::consumeEnergy(dev_name, EngyConsume);
    DPRINTF(EnergyMgmt, "Accelerator consumed %f energy\n", EngyConsume);
    schedule(tickEvent, curTick() + latency);
}

/* ---------------- CtrlPort implementation ---------------- */

Accelerator::CtrlPort::CtrlPort(const std::string &name, Accelerator *accel)
    : SlavePort(name, accel), owner(accel)
{
}

/* CtrlPort::recvAtomic handles every MMIO access to the accelerator’s control register range. */
Tick Accelerator::CtrlPort::recvAtomic(PacketPtr pkt)
{
    assert(owner);
    return owner->recvAtomic(pkt);
}

void Accelerator::CtrlPort::recvFunctional(PacketPtr pkt)
{
    assert(owner);
    owner->recvFunctional(pkt);
}

bool Accelerator::CtrlPort::recvTimingReq(PacketPtr pkt)
{
    assert(owner);
    return owner->recvTimingReq(pkt);
}

void Accelerator::CtrlPort::recvRespRetry()
{
    assert(owner);
    owner->recvRespRetry();
}

AddrRangeList
Accelerator::CtrlPort::getAddrRanges() const
{
    assert(owner);
    AddrRangeList list;
    list.push_back(owner->controlRange);
    return list;
}

/* ---------------- Accelerator implementation ---------------- */
Accelerator::Accelerator(const Params *p) :
    MemObject(p),
    ctrlPort(name() + ".ctrlPort", this),
    tickEvent(this),

    cpu(p->cpu),
    dmaCtrl(p->dmaCtrl),
    controlRange(p->controlRange),

    delay_init(p->delay_init),
    delay_compute_per_count(p->delay_compute_per_count),
    delay_cpu_interrupt(p->delay_cpu_interrupt),

    energy_state(AccelEnergyState::STATE_OFF),
    event_init(this, false, Event::Accelerator_Interrupt)
{
    /* configure compute unit */
    computeUnit = new ComputeUnit(getEventQueue(0), delay_compute);

    energy_per_cycle[0] = p->energy_per_cycle[0];
    energy_per_cycle[1] = p->energy_per_cycle[1];
    energy_per_cycle[2] = p->energy_per_cycle[2];

    src_addr = 0;
    dst_addr = 0;
    cmd_reg = 0;

    busy = false;

    input_buffer = nullptr;
    output_buffer = nullptr;
}

Accelerator::~Accelerator()
{
    if (computeUnit)
    {
        delete computeUnit;
        computeUnit = nullptr;
    }

    if (input_buffer)
    {
        delete[] input_buffer;
        input_buffer = nullptr;
    }
    if (output_buffer)
    {
        delete[] output_buffer;
        output_buffer = nullptr;
    }
}

void Accelerator::init()
{
    // register port ranges
    if (ctrlPort.isConnected())
        ctrlPort.sendRangeChange();

    DPRINTF(Accelerator, "%s initialized: controlRange: %#llx - %#llx\n",
            name(), controlRange.start(), controlRange.end());

    // set default energy state
    energy_state = STATE_OFF;
    if (!tickEvent.scheduled())
    {
        schedule(tickEvent, clockEdge(Cycles(0)));
    }
}

BaseSlavePort &
Accelerator::getSlavePort(const std::string &if_name, PortID idx)
{
    if (if_name == "ctrlPort" || if_name == "ctrl")
    {
        return ctrlPort;
    }
    return MemObject::getSlavePort(if_name, idx);
}

void Accelerator::onDmaReadDone()
{
    // Handle DMA read completion
    cmd_reg &= ~CMD_DMA_READ;
}

void Accelerator::onDmaWriteDone()
{
    // Handle DMA write completion
    cmd_reg &= ~CMD_DMA_WRITE;
    cmd_reg |= CMD_CPU_INTERRUPT;
}

/* ComputeTask interfaces implementation */
void Accelerator::onComputeDone()
{
    DPRINTF(Accelerator, "Compute done...\n");
    cmd_reg &= ~CMD_COMPUTE;
    cmd_reg |= CMD_DMA_WRITE;
}

void Accelerator::onComputeAbort()
{
    DPRINTF(Accelerator, "Compute failed...\n");
    cmd_reg &= ~CMD_COMPUTE;
    energy_state = AccelEnergyState::STATE_IDLE;
}

/** Initialize the accelerator */
void Accelerator::initEvent()
{
    /* Initialize any necessary resources or state */
    busy = false;
    input_buffer = new uint8_t[count];
    output_buffer = new uint8_t[count];

    DPRINTF(Accelerator, "Initialization done\n");

    cmd_reg &= ~CMD_START;
    cmd_reg |= CMD_DMA_READ;
}

void Accelerator::doInit()
{
    DPRINTF(Accelerator, "Scheduling initialization event\n");
    energy_state = STATE_ON;
    schedule(initEvent, curTick() + delay_init);
}

void Accelerator::doDmaRead()
{
    DPRINTF(Accelerator, "Scheduling DMA read ...\n");
    energy_state = STATE_IDLE;

    if (!dmaCtrl) {
        panic("DMA controller not initialized");
    }

    dmaCtrl->startRead(
        src_addr,
        input_buffer,
        count,
        [this]() {
            onDmaReadDone();
        }
    );
}

void Accelerator::doDmaWrite()
{
    DPRINTF(Accelerator, "Scheduling DMA write ...\n");
    energy_state = STATE_IDLE;

    if (!dmaCtrl) {
        panic("DMA controller not initialized");
    }

    dmaCtrl->startWrite(
        dst_addr,
        output_buffer,
        count,
        [this]() {
            onDmaWriteDone();
        }
    );
}

void Accelerator::doCompute()
{
    DPRINTF(Accelerator, "Compute started...\n");
    energy_state = STATE_ON;
    computeUnit->startCompute(
        input_buffer,
        output_buffer,
        count,
        this
    );
}

void Accelerator::abortCompute()
{
    DPRINTF(Accelerator, "Compute aborted...\n");
    computeUnit->abort();

    cmd_reg &= ~CMD_COMPUTE;
    energy_state = AccelEnergyState::STATE_OFF;
}

/** Receive atomic request **/
Tick Accelerator::recvAtomic(PacketPtr pkt)
{
    // DPRINTF(Accelerator, "Received atomic request at %s\n", name());
    Addr offset = pkt->getAddr() - controlRange.start();

    // assume 32-bit aligned register accesses
    if (pkt->isWrite())
    {
        switch (offset)
        {
        case 0x00: // CMD
            cmd_reg = *(pkt->getConstPtr<uint8_t>());
            if (cmd_reg & CMD_START)
            { // START bit
                if (!busy)
                {
                    DPRINTF(Accelerator, "CMD_START received: scheduling initialization\n");
                }
                else
                {
                    DPRINTF(Accelerator, "%s: START requested but busy\n", name());
                }
            }
            break;

        case 0x08: // SRC_ADDR
            src_addr = *(pkt->getConstPtr<Addr>());
            break;

        case 0x10: // DST_ADDR
            dst_addr = *(pkt->getConstPtr<Addr>());
            break;

        case 0x18: // COUNT
            count = *(pkt->getConstPtr<uint32_t>());
            break;

        default:
            DPRINTF(Accelerator, "%s: Unknown write offset %#x val %#x\n", name(), offset);
            break;
        }
    }
    else if (pkt->isRead())
    {
        uint32_t ret = 0;
        switch (offset)
        {
        case 0x00:
            ret = cmd_reg;
            break;

        case 0x20:
            ret = busy ? 1 : 0;
            break;

        default:
            ret = 0;
            break;
        }
        *(pkt->getPtr<uint32_t>()) = ret;
    }
    pkt->makeResponse();
    return 0;
}

void Accelerator::recvFunctional(PacketPtr pkt)
{
    fatal("Functional request not supported");
}

bool Accelerator::recvTimingReq(PacketPtr pkt)
{
    fatal("Timing request not supported");
}

void Accelerator::recvRespRetry()
{
    fatal("Response retry not supported");
}

AddrRange Accelerator::getAddrRanges() const
{
    return controlRange;
}

/** handle energy manager messages (optional) **/
int Accelerator::handleMsg(const EnergyMsg &msg)
{
    if (!busy)
        return 1;

    if (msg.type == SimpleEnergySM::MsgType::POWER_OFF)
    {
        if (energy_state == AccelEnergyState::STATE_OFF) return 1;

        DPRINTF(Accelerator, "Powering off ...\n");
        energy_state = AccelEnergyState::STATE_OFF;
        handleInterrupt();
    }
    else if (msg.type == SimpleEnergySM::MsgType::POWER_ON)
    {
        if (energy_state == AccelEnergyState::STATE_ON) {
            // Progress execution
            if (cmd_reg & CMD_START)
                doInit();
            else if (cmd_reg & CMD_DMA_READ)
                doDmaRead();
            else if (cmd_reg & CMD_DMA_WRITE)
                doDmaWrite();
            else if (cmd_reg & CMD_COMPUTE)
                doCompute();
            else if (cmd_reg & CMD_CPU_INTERRUPT)
                triggerInterrupt();
            else if (cmd_reg & CMD_DONE)
                finishSuccess();
        }

        else {
            DPRINTF(Accelerator, "Powering on ...\n");
            energy_state = STATE_ON;
            handleRecovery();
        }

    }
    else
    {
        DPRINTF(EnergyMgmt, "Unknown message type received!\n");
        return 0;
    }

    return 1;
}

void Accelerator::handleInterrupt()
{
    if (cmd_reg & CMD_START) {
        if (event_init.scheduled())
            event_init.deschedule();

    else if (cmd_reg & CMD_DMA_READ || cmd_reg & CMD_DMA_WRITE)
        dmaCtrl->pauseDma();

    else if (cmd_reg & CMD_COMPUTE)
        abortCompute();

    else if (cmd_reg & CMD_CPU_INTERRUPT)
        triggerInterrupt();

}

void Accelerator::handleRecovery()
{
    if (cmd_reg & CMD_COMPUTE)
        doCompute();

    else if (cmd_reg & CMD_DMA_READ || cmd_reg & CMD_DMA_WRITE)
    {
        DPRINTF(Accelerator, "DMA resumed\n");
    }
}

/** triggerInterrupt: stub to notify CPU - adjust to your system's API */
void Accelerator::triggerInterrupt()
{
    DPRINTF(Accelerator, "Accelerator: triggers an interrupt to CPU\n");
    busy = false;
    energy_state = STATE_IDLE;
    cpu->accelInterrupt(delay_cpu_interrupt);
}

Accelerator *AcceleratorParams::create() {
    return new Accelerator(this);
}
