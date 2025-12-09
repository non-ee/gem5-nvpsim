#include "accel/accel.hh"
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

    switch (energy_state)
    {
    case AccelEnergyState::STATE_OFF:
        EngyConsume = 0;
        break;
    case AccelEnergyState::STATE_IDLE:
        EngyConsume = energy_idle_per_tick * ticksToCycles(latency);
        break;
    case AccelEnergyState::STATE_ON:
        EngyConsume = energy_compute_per_tick * ticksToCycles(latency);
        break;
    default:
        panic("Invalid energy state");
    }

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

    count(p->count),
    delay_init(p->delay_init),
    delay_compute(p->delay_compute),
    delay_cpu_interrupt(p->delay_cpu_interrupt),

    energy_compute_per_tick(p->energy_compute_per_tick),
    energy_idle_per_tick(p->energy_idle_per_tick),
    energy_state(AccelEnergyState::STATE_OFF),
    event_init(this, false, Event::Accelerator_Interrupt)
{
    /* configure compute unit */
    computeUnit = new ComputeUnit(getEventQueue(0), delay_compute);

    src_addr = 0;
    dst_addr = 0;
    cmd_reg = 0;
    busy = false;

    /* configure buffers */
    input_buffer = new uint8_t[count];
    output_buffer = new uint8_t[count];
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
    doCompute();
}

void Accelerator::onDmaWriteDone()
{
    // Handle DMA write completion
    cmd_reg &= ~CMD_DMA_WRITE;
    triggerInterrupt();
}

/* ComputeTask interfaces implementation */
void Accelerator::onComputeDone()
{
    DPRINTF(Accelerator, "Compute done...\n");
    cmd_reg &= ~CMD_COMPUTE;
    energy_state = AccelEnergyState::STATE_IDLE;
    doDmaWrite();
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
    // Initialize any necessary resources or state
    DPRINTF(Accelerator, "Initialization done\n");
    doDmaRead();
}

void Accelerator::doDmaRead()
{
    DPRINTF(Accelerator, "DMA read started...\n");
    cmd_reg |= CMD_DMA_READ;
    energy_state = AccelEnergyState::STATE_IDLE;

    if (!dmaCtrl) {
        panic("DMA controller not initialized");
    }

    dmaCtrl->startRead(
        src_addr,
        input_buffer,
        count,
        this
    );
}

void Accelerator::doDmaWrite()
{
    DPRINTF(Accelerator, "DMA write started...\n");
    cmd_reg |= CMD_DMA_WRITE;
    energy_state = AccelEnergyState::STATE_IDLE;

    if (!dmaCtrl) {
        panic("DMA controller not initialized");
    }

    dmaCtrl->startWrite(
        dst_addr,
        output_buffer,
        count,
        this
    );
}

void Accelerator::doCompute()
{
    DPRINTF(Accelerator, "Compute started...\n");
    cmd_reg |= CMD_COMPUTE;
    energy_state = AccelEnergyState::STATE_ON;
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
    cmd_reg &= ~CMD_COMPUTE;
    energy_state = AccelEnergyState::STATE_OFF;
    computeUnit->abort();
}

/** Receive atomic request **/
Tick Accelerator::recvAtomic(PacketPtr pkt)
{
    // DPRINTF(Accelerator, "Received atomic request at %s\n", name());
    Addr offset = pkt->getAddr() - controlRange.start();

    // assume 32-bit aligned register accesses
    if (pkt->isWrite())
    {
        // printf("Accelerator: received atomic write from address %lx at offset %lx\n", pkt->getAddr(), offset);
        switch (offset)
        {
        case 0x00: // CMD
            cmd_reg = *(pkt->getConstPtr<uint8_t>());
            if (cmd_reg & CMD_START)
            { // START bit
                if (!busy)
                {
                    busy = true;
                    DPRINTF(Accelerator, "CMD_START received: scheduling initialization\n");
                    schedule(event_init, curTick() + delay_init);
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

        case 0x14: // STATUS
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
    if (msg.type == SimpleEnergySM::MsgType::POWER_OFF)
    {
        if (energy_state == AccelEnergyState::STATE_OFF)
            return 1;

        DPRINTF(Accelerator, "Powering off accelerator\n");
        energy_state = AccelEnergyState::STATE_OFF;
        handleInterrupt();
    }
    else if (msg.type == SimpleEnergySM::MsgType::POWER_ON)
    {
        if (energy_state == AccelEnergyState::STATE_ON)
            return 1;

        DPRINTF(Accelerator, "Powering on accelerator\n");
        energy_state = AccelEnergyState::STATE_ON;
        handleRecovery();
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
    if (cmd_reg & CMD_COMPUTE)
        abortCompute();

    else if (cmd_reg & CMD_DMA_READ || cmd_reg & CMD_DMA_WRITE)
    {
        DPRINTF(Accelerator, "MemPort: DMA paused\n");
    }
}

void Accelerator::handleRecovery()
{
    if (cmd_reg & CMD_COMPUTE)
        doCompute();

    else if (cmd_reg & CMD_DMA_READ || cmd_reg & CMD_DMA_WRITE)
    {
        DPRINTF(Accelerator, "MemPort: DMA resumed\n");
    }
}

/** triggerInterrupt: stub to notify CPU - adjust to your system's API */
void Accelerator::triggerInterrupt()
{
    DPRINTF(Accelerator, "Accelerator: triggers an interrupt to CPU\n");

    cmd_reg |= CMD_DONE;
    energy_state = AccelEnergyState::STATE_OFF;

    cpu->accelInterrupt(delay_cpu_interrupt);
}

Accelerator *AcceleratorParams::create() {
    return new Accelerator(this);
}
