#include "accel/accel.hh"
#include "accel/compute_unit.hh"
#include "debug/Accelerator.hh"
#include "debug/EnergyMgmt.hh"
#include "debug/MemoryAccess.hh"
#include "engy/state_machine.hh"
#include "params/Accelerator.hh"
#include "base/trace.hh"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <stdint.h>

/** -- ComputeUnit **/
ComputeUnit::ComputeUnit(Accelerator *_owner, Tick latency)
    : owner(_owner),
      busy(false),
      computeLatency(latency),
      remainingLatency(0),
      paused(false),
      computeDoneEvent(this)
{
}

void ComputeUnit::start()
{
    if (busy)
        return;

    DPRINTF(Accelerator, "ComputeUnit: start compute (latency=%lu)\n", computeLatency);
    busy = true;
    paused = false;
    computeStartTick = curTick();

    // Schedule completion event after computeLatency ticks
    // Tick latency = remainingLatency ? remainingLatency : computeLatency;
    owner->schedule(&computeDoneEvent, curTick() + computeLatency);
}

void ComputeUnit::compute()
{
    uint8_t *input = owner->input_buffer;
    uint8_t *output = owner->output_buffer;
    uint32_t N = owner->count;

    // Implement computation logic here
    for (int i = 0; i < N; i++) {
        uint32_t x = input[i];

        for (int j = 0; j < 100; j++) {
            x = (x * 17 + j) % 256;
        }

        output[i] = x;

        DPRINTF(Accelerator, "Input[%d]: %d\n", i, input[i]);
    }
}

void ComputeUnit::finish()
{
    // compute();
    owner->writeOutputBuffer(0x1f);
    busy = false;
    DPRINTF(Accelerator, "ComputeUnit: computation finished\n");

    // Fill accelerator's output buffer with a predefined value

    owner->energy_state = Accelerator::AccelEnergyState::STATE_IDLE;
    owner->cmd_reg &= ~Accelerator::CMD_COMPUTE;

    owner->doDmaWrite();
}

void ComputeUnit::abort()
{
    // Handle interrupt logic here
    DPRINTF(Accelerator, "ComputeUnit: Aborting computation\n");

    busy = false;
    if (computeDoneEvent.scheduled())
        owner->deschedule(&computeDoneEvent);
}

void ComputeUnit::pause()
{
    if (!busy || paused)
        return;

    paused = true;

    Tick elapsed = curTick() - computeStartTick;
    remainingLatency = (elapsed < computeLatency) ? (computeLatency - elapsed) : 0;

    DPRINTF(Accelerator, "ComputeUnit: paused (remaining=%lu)\n", remainingLatency);

    // Cancel the scheduled done event
    if (computeDoneEvent.scheduled())
        owner->deschedule(&computeDoneEvent);

    busy = false;
    owner->cmd_reg &= ~Accelerator::CMD_COMPUTE;
    owner->energy_state = Accelerator::AccelEnergyState::STATE_OFF;
}

void ComputeUnit::resume()
{
    if (!paused || busy)
        return;

    DPRINTF(Accelerator, "ComputeUnit: resume (remaining=%lu)\n", remainingLatency);

    paused = false;
    busy = true;
    computeStartTick = curTick();

    owner->cmd_reg |= Accelerator::CMD_COMPUTE;
    owner->energy_state = Accelerator::AccelEnergyState::STATE_ON;
    owner->schedule(&computeDoneEvent, curTick() + remainingLatency);
}

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

CtrlPort::CtrlPort(const std::string &name, Accelerator *accel)
    : SlavePort(name, accel), owner(accel)
{
}

/* CtrlPort::recvAtomic handles every MMIO access to the accelerator’s control register range. */
Tick CtrlPort::recvAtomic(PacketPtr pkt)
{
    assert(owner);
    return owner->recvAtomic(pkt);
}

void CtrlPort::recvFunctional(PacketPtr pkt)
{
    assert(owner);
    owner->recvFunctional(pkt);
}

bool CtrlPort::recvTimingReq(PacketPtr pkt)
{
    assert(owner);
    return owner->recvTimingReq(pkt);
}

void CtrlPort::recvRespRetry()
{
    assert(owner);
    owner->recvRespRetry();
}

AddrRangeList
CtrlPort::getAddrRanges() const
{
    assert(owner);
    AddrRangeList list;
    list.push_back(owner->controlRange);
    return list;
}

/* ---------------- MemPort implementation ---------------- */

MemPort::MemPort(const std::string &name, Accelerator *accel)
    : MasterPort(name, accel),
      owner(accel),
      dmaEvent(this, false, Event::Accelerator_DMA_Pri)
{
    dmaIndex = 0;
    dmaCount = 0;
    dmaReadMode = false;
    dmaWriteMode = false;
}

uint8_t
MemPort::readAtomic(Addr addr)
{
    Request* req = new Request(
        addr, sizeof(uint8_t), 0, Request::funcMasterId
    );
    PacketPtr pkt = Packet::createRead(req);
    pkt->allocate();
    sendAtomic(pkt);

    uint8_t data = pkt->get<uint8_t>();
    DPRINTF(Accelerator, "readAtomic: read value %#x from addr=%#lx\n", data, pkt->getPtr<uint8_t>());
    delete req;
    delete pkt;

    return data;
}

void MemPort::writeAtomic(Addr addr, uint8_t val)
{
    Request* req = new Request(
        addr, sizeof(uint8_t), 0, Request::funcMasterId
    );
    PacketPtr pkt = Packet::createWrite(req);

    pkt->allocate();
    pkt->set<uint8_t>(val);
    sendAtomic(pkt);

    DPRINTF(Accelerator, "MemPort::writeAtomic: wrote value %#x to addr=%#lx\n", val, addr);
}

void MemPort::startDmaRead(Addr src, uint8_t *buf, uint32_t count)
{
    dmaAddr = src;
    dmaBuffer = buf;
    dmaCount = count;
    dmaIndex = 0;
    dmaReadMode = true;

    DPRINTF(Accelerator, "MemPort::startDmaRead: src=%#lx, count=%u\n", src, count);

    // Schedule first DMA step immediately
    if (!dmaEvent.scheduled())
        owner->schedule(dmaEvent, owner->clockEdge(Cycles(1)));
}

void MemPort::startDmaWrite(Addr dst, uint8_t *buf, uint32_t count)
{
    dmaAddr = dst;
    dmaBuffer = buf;
    dmaCount = count;
    dmaIndex = 0;
    dmaWriteMode = true;

    DPRINTF(Accelerator, "MemPort::startDmaWrite: dst=%#lx, count=%u\n", dst, count);

    if (!dmaEvent.scheduled())
        owner->schedule(dmaEvent, owner->clockEdge(Cycles(1)));
}


void MemPort::dmaStep()
{
    if (dmaIndex < dmaCount)
    {
        if (dmaReadMode)
        {
            uint8_t data = readAtomic(dmaAddr + dmaIndex);
            dmaBuffer[dmaIndex] = data;
        }
        else if (dmaWriteMode)
        {
            uint8_t data = dmaBuffer[dmaIndex];
            writeAtomic(dmaAddr + dmaIndex, data);
        }

        dmaIndex++;

        if (dmaIndex % 16 == 0)
        {
            DPRINTF(Accelerator, "MemPort::dmaStep: transferred %u/%u bytes\n",
                    dmaIndex, dmaCount);
        }

        // Continue DMA next tick
        owner->schedule(dmaEvent, owner->clockEdge(Cycles(1)));
    }
    else
    {
        // DMA complete
        if (dmaReadMode)
        {
            owner->cmd_reg &= ~Accelerator::CMD_DMA_READ;
            dmaReadMode = false;

            DPRINTF(Accelerator, "MemPort::dmaStep: DMA READ complete (%u bytes)\n", dmaCount);

            // trigger compute after read
            owner->doCompute();
        }
        else if (dmaWriteMode)
        {
            owner->cmd_reg &= ~Accelerator::CMD_DMA_WRITE;
            dmaWriteMode = false;

            DPRINTF(Accelerator, "MemPort::dmaStep: DMA WRITE complete (%u bytes)\n", dmaCount);

            // optionally signal CPU or interrupt
            owner->triggerInterrupt();
        }

        owner->energy_state = Accelerator::STATE_IDLE;
    }
}

void MemPort::pauseDma()
{
    if ((!dmaReadMode && !dmaWriteMode) || paused)
        return;

    paused = true;

    DPRINTF(Accelerator, "MemPort::pauseDma: Pausing DMA at %u/%u bytes\n",
            dmaIndex, dmaCount);

    // Cancel pending DMA event
    if (dmaEvent.scheduled())
        owner->deschedule(&dmaEvent);

    // Clear status flags temporarily
    if (dmaReadMode)
        owner->cmd_reg &= ~Accelerator::CMD_DMA_READ;

    else if (dmaWriteMode)
        owner->cmd_reg &= ~Accelerator::CMD_DMA_WRITE;

    // Set accelerator to OFF state
    owner->energy_state = Accelerator::AccelEnergyState::STATE_OFF;
}

void MemPort::resumeDma()
{
    if (!paused)
        return;

    paused = false;

    DPRINTF(Accelerator, "MemPort::resumeDma: Resuming DMA from %u/%u bytes\n",
            dmaIndex, dmaCount);

    // Restore correct flags
    if (dmaReadMode)
        owner->cmd_reg |= Accelerator::CMD_DMA_READ;

    else if (dmaWriteMode)
        owner->cmd_reg |= Accelerator::CMD_DMA_WRITE;


    owner->energy_state = Accelerator::AccelEnergyState::STATE_IDLE;

    // Resume next DMA step
    if (!dmaEvent.scheduled())
        owner->schedule(dmaEvent, owner->clockEdge(Cycles(1)));
}
/* ---------------- Accelerator implementation ---------------- */

Accelerator::Accelerator(const Params *p) : MemObject(p),
                                            ctrlPort(name() + ".ctrlPort", this),
                                            memPort(name() + ".memPort", this),
                                            computeUnit(this, p->delay_compute),
                                            tickEvent(this),

                                            cpu(p->cpu),
                                            controlRange(p->controlRange),

                                            cmd_reg(0),
                                            src_addr(0),
                                            dst_addr(0),
                                            busy(false),

                                            count(p->count),
                                            delay_init(p->delay_init),
                                            delay_compute(p->delay_compute),
                                            delay_cpu_interrupt(p->delay_cpu_interrupt),

                                            energy_compute_per_tick(p->energy_compute_per_tick),
                                            energy_idle_per_tick(p->energy_idle_per_tick),
                                            energy_state(AccelEnergyState::STATE_OFF),

                                            event_init(this, false, Event::Accelerator_Interrupt)
{
    /* configure buffers */
    input_buffer = new uint8_t[count];
    output_buffer = new uint8_t[count];
}

Accelerator::~Accelerator()
{
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
    {
        ctrlPort.sendRangeChange();
    }

    DPRINTF(Accelerator, "%s initialized: controlRange: %#llx - %#llx\n",
            name(), controlRange.start(), controlRange.end());

    printf("Accelerator's controlRange: %#lx - %#lx\n", controlRange.start(), controlRange.end());

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

BaseMasterPort &
Accelerator::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name == "memPort" || if_name == "mem")
    {
        return memPort;
    }
    return MemObject::getMasterPort(if_name, idx);
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
        DPRINTF(Accelerator, "Powering off accelerator\n");
        energy_state = AccelEnergyState::STATE_OFF;
        handleInterrupt();
    }
    else if (msg.type == SimpleEnergySM::MsgType::POWER_ON)
    {
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
    {
        computeUnit.abort();
        DPRINTF(Accelerator, "ComputeUnit: aborted\n");
    }
    else if (cmd_reg & CMD_DMA_READ || cmd_reg & CMD_DMA_WRITE)
    {
        memPort.pauseDma();
        DPRINTF(Accelerator, "MemPort: DMA paused\n");
    }
}

void Accelerator::handleRecovery()
{
    if (cmd_reg & CMD_COMPUTE)
    {
        computeUnit.start();
        DPRINTF(Accelerator, "ComputeUnit: started\n");
    }
    else if (cmd_reg & CMD_DMA_READ || cmd_reg & CMD_DMA_WRITE)
    {
        memPort.resumeDma();
        DPRINTF(Accelerator, "MemPort: DMA resumed\n");
    }
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
    memPort.startDmaRead(src_addr, input_buffer, count);
}

void Accelerator::doDmaWrite()
{
    DPRINTF(Accelerator, "DMA write started...\n");
    cmd_reg |= CMD_DMA_WRITE;
    energy_state = AccelEnergyState::STATE_IDLE;
    memPort.startDmaWrite(dst_addr, output_buffer, count);
}

void Accelerator::doCompute()
{
    DPRINTF(Accelerator, "Compute started...\n");
    cmd_reg |= CMD_COMPUTE;
    energy_state = AccelEnergyState::STATE_ON;
    computeUnit.start();
}

/** Write output buffer **/
void Accelerator::writeOutputBuffer(uint8_t value)
{
    DPRINTF(Accelerator, "Accelerator: writing result 0x%x to output buffer\n", value);

    for (uint32_t i = 0; i < count; i++)
    {
        output_buffer[i] = value;
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
