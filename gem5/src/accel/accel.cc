#include "accel/accel.hh"
#include "accel.hh"
#include "debug/Accelerator.hh"
#include "debug/EnergyMgmt.hh"
#include "debug/MemoryAccess.hh"
#include "engy/state_machine.hh"
#include "base/trace.hh"
#include "base/callback.hh"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
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

void Accelerator::fsmStep()
{
    uint8_t accel_op = cmd & CMD_MASK;

    if (accel_op == ACCEL_IDLE)
        return;

    cmd |= BUSY_BIT;
    switch (accel_op)
    {
        case ACCEL_INIT:
            doInit();
            break;
        case ACCEL_DMA_READ:
            doDmaRead();
            break;
        case ACCEL_DMA_WRITE:
            doDmaWrite();
            break;
        case ACCEL_COMPUTE:
            doCompute();
            break;
        case ACCEL_INTERRUPT:
            doInterrupt();
            break;
    }
}

void Accelerator::tick()
{
    Tick latency = clockPeriod();
    double EngyConsume = energy_per_cycle[energy_state] * ticksToCycles(latency);
    EnergyObject::consumeEnergy(accel_name, EngyConsume);
    total_energy_consumed += EngyConsume;

    DPRINTF(EnergyMgmt, "Accelerator consumed %f energy\n", EngyConsume);

    schedule(tickEvent, curTick() + latency);

    // DPRINTF(Accelerator, "Tick: cmd = %x\n", cmd);
    if (!(cmd & BUSY_BIT)) {
        fsmStep();
    }

    if (energy_state != STATE_OFF) {
        total_tick += latency;
    }
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
    ctrlPort(name() + ".ctrl_port", this),
    tickEvent(this),

    cpu(p->cpu),
    computeUnit(p->compute_unit),
    dmaCtrl(p->dma_ctrl),
    controlRange(p->control_range),

    delay_init(p->delay_init),
    delay_cpu_interrupt(p->delay_cpu_interrupt),
    delay_recover(p->delay_recover),

    energy_state(AccelEnergyState::STATE_OFF),
    event_interrupt(this, false, Event::Accelerator_Interrupt)
{
    strcpy(accel_name, "Accelerator");

    energy_per_cycle[0] = p->energy_per_cycle[0];
    energy_per_cycle[1] = p->energy_per_cycle[1];
    energy_per_cycle[2] = p->energy_per_cycle[2];
    energy_per_cycle[3] = p->energy_per_cycle[3];

    total_energy_consumed = 0;

    src_addr = 0;
    dst_addr = 0;
    input_count = 0;
    output_count = 0;

    cmd = 0;
    need_recover = false;

    input_buffer = nullptr;
    output_buffer = nullptr;

    total_tick = 0;

    /* register end-of simulation callback */
    registerExitCallback(
        new MakeCallback<Accelerator, &Accelerator::onSimulationExit>(this)
    );
}

Accelerator::~Accelerator()
{
    computeUnit = nullptr;

    delete[] input_buffer;
    delete[] output_buffer;
}

void Accelerator::onSimulationExit()
{
    std::ofstream fout("m5out/energy_consumed.txt", std::ios::app);
    assert(fout);
    fout << "Accelerator: " << total_energy_consumed << std::endl;
    fout.close();

    fout.open("m5out/ticks_output.txt", std::ios::app);
    fout << "Accelerator: " << total_tick << std::endl;
    fout.close();
}

void Accelerator::init()
{
    // register port ranges
    if (ctrlPort.isConnected())
        ctrlPort.sendRangeChange();

    DPRINTF(Accelerator, "%s initialized: controlRange: %#llx - %#llx\n",
            name(), controlRange.start(), controlRange.end());
    DPRINTF(Accelerator, "%s connected master energy port: %s\n",
        name(), getMasterEnergyPort().owner->name());


    cmd &= ~INIT_BIT;
    cmd &= ~BUSY_BIT;
    cmd &= ~DONE_BIT;

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
    if (if_name == "ctrl_port" || if_name == "ctrl")
    {
        return ctrlPort;
    }
    return MemObject::getSlavePort(if_name, idx);
}

void Accelerator::onDmaReadDone()
{
    // Handle DMA read completion
    DPRINTF(Accelerator, "DMA read done...\n");
    cmd &= ~BUSY_BIT;
    setCmd(ACCEL_COMPUTE);
}

void Accelerator::onDmaWriteDone()
{
    // Handle DMA write completion
    DPRINTF(Accelerator, "DMA write done...\n");
    cmd &= ~BUSY_BIT;
    setCmd(ACCEL_INTERRUPT);
}

/* ComputeTask interfaces implementation */
void Accelerator::onComputeDone()
{
    DPRINTF(Accelerator, "Compute done...\n");
    cmd &= ~BUSY_BIT;
    setCmd(ACCEL_DMA_WRITE);
}

void Accelerator::onComputeAbort()
{
    DPRINTF(Accelerator, "Compute failed...\n");
}


void Accelerator::doInit()
{
    DPRINTF(Accelerator, "Scheduling initialization event\n");
    energy_state = STATE_INIT;

    /* Initialize any necessary resources or state */
    input_buffer = new uint8_t[input_count];
    output_buffer = new uint8_t[output_count];
    schedule(event_interrupt, curTick() + delay_init);
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
        input_count,
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
        output_count,
        [this]() {
            onDmaWriteDone();
        }
    );
}

void Accelerator::doCompute()
{
    DPRINTF(Accelerator, "Compute started...\n");
    energy_state = STATE_ON;
    computeUnit->start(
        input_buffer,
        output_buffer,
        input_count,
        output_count,
        this
    );
}

/** triggerInterrupt: stub to notify CPU - adjust to your system's API */
void Accelerator::doInterrupt()
{
    DPRINTF(Accelerator, "Accelerator: triggers an interrupt to CPU\n");
    energy_state = STATE_IDLE;
    cpu->accelInterrupt(delay_cpu_interrupt);
    schedule(event_interrupt, curTick() + delay_cpu_interrupt);
}

void Accelerator::abortCompute()
{
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
        switch (offset)
        {
            case 0x00: // CMD
            {
                uint8_t cmd_val = *(pkt->getConstPtr<uint8_t>());
                cmd_val = cmd_val & CMD_MASK;
                if (cmd_val == ACCEL_INIT) {
                    // START bit
                    if (cmd & BUSY_BIT)
                    {
                        DPRINTF(Accelerator, "%s: START requested but busy\n", name());
                    }
                    else
                    {
                        DPRINTF(Accelerator, "INIT received: scheduling initialization\n");
                        cmd &= ~DONE_BIT;

                        if (cmd & INIT_BIT)
                            setCmd(ACCEL_DMA_READ);
                        else
                            setCmd(ACCEL_INIT);
                    }
                }
                else {
                    DPRINTF(Accelerator, "Unknown command. Command No: %d\n", cmd_val);
                }
                break;
            }

            case 0x08: // SRC_ADDR
                src_addr = *(pkt->getConstPtr<Addr>());
                break;

            case 0x10: // DST_ADDR
                dst_addr = *(pkt->getConstPtr<Addr>());
                break;

            case 0x18: // COUNT
                input_count = *(pkt->getConstPtr<uint32_t>());
                break;

            case 0x20: // Output count
                output_count = *(pkt->getConstPtr<uint32_t>());
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
            ret = cmd;
            // DPRINTF(Accelerator, "CPU read. cmd = %x\n", cmd);
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
    if (!(cmd & BUSY_BIT))
        return 1;

    if (msg.type == SimpleEnergySM::MsgType::POWER_OFF)
    {
        DPRINTF(Accelerator, "Powering off ...\n");
        energy_state = STATE_OFF;
        handleInterrupt();
    }
    else if (msg.type == SimpleEnergySM::MsgType::POWER_ON)
    {
        DPRINTF(Accelerator, "Powering on ...\n");
        energy_state = STATE_IDLE;
        handleRecovery();
    }
    else {
        DPRINTF(EnergyMgmt, "Unknown message type received!\n");
        return 0;
    }

    return 1;
}

void Accelerator::handleInterrupt()
{
    if ((cmd & CMD_MASK) == ACCEL_INIT) {
        // if (event_interrupt.scheduled()) {
        //     DPRINTF(Accelerator, "Accelerator: deschedule event_init\n");
        //     deschedule(event_interrupt);
        // }
        DPRINTF(Accelerator, "Accelerator: deschedule event_init\n");
        deschedule(event_interrupt);
    }
    else {
        need_recover = true;
        if ((cmd & CMD_MASK) == ACCEL_COMPUTE) {
            DPRINTF(Accelerator, "Accelerator: abort compute\n");
            abortCompute();
        }
    }
}

void Accelerator::handleRecovery()
{
    DPRINTF(Accelerator, "Accelerator: handles recovery\n");

    if ((cmd & CMD_MASK) == ACCEL_INIT) {
        DPRINTF(Accelerator, "Accelerator: reschedule event_init\n");
        // doInit();
    }
    else {
        /* Recovery */
        energy_state = STATE_ON;
        schedule(event_interrupt, curTick() + delay_recover);
    }
}

/** triggerInterrupt: stub to notify CPU - adjust to your system's API */
void Accelerator::triggerInterrupt()
{
    if (need_recover) {
        DPRINTF(Accelerator, "Power recovery done\n");
        need_recover = false;
        return;
    }

    uint8_t accel_op = cmd & CMD_MASK;
    if (accel_op == ACCEL_INIT) {
        DPRINTF(Accelerator, "Initialization done\n");
        cmd |= INIT_BIT;
        cmd &= ~BUSY_BIT;
        setCmd(ACCEL_DMA_READ);
    }
    else if (accel_op == ACCEL_INTERRUPT) {
        DPRINTF(Accelerator, "CPU Interruption done\n");
        cmd |= DONE_BIT;
        cmd &= ~BUSY_BIT;
        finishSuccess();
    }
}

void Accelerator::finishSuccess()
{
    DPRINTF(Accelerator, "Computing task finished successfully\n");
    cmd |= DONE_BIT;
    setCmd(ACCEL_IDLE);
    energy_state = STATE_OFF;
    DPRINTF(Accelerator, "cmd = %x\n", cmd);
}

Accelerator *AcceleratorParams::create() {
    return new Accelerator(this);
}

void Accelerator::setCmd(uint8_t accel_cmd)
{
    cmd = (cmd & ~CMD_MASK) | accel_cmd;
}
