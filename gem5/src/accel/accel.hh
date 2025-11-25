#ifndef GEM5_ACCEL_HH
#define GEM5_ACCEL_HH

#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "sim/eventq.hh"
#include "params/Accelerator.hh"
#include "cpu/base.hh"
#include <cstdint>
#include <string>
#include <sys/types.h>

class Accelerator;

class ComputeUnit
{
  private:
    Accelerator* owner;
    bool busy;
    Tick computeLatency;

    Tick computeStartTick;
    Tick remainingLatency;
    bool paused;


    class ComputeDoneEvent : public Event
    {
      private:
        ComputeUnit* parent;
      public:
        ComputeDoneEvent(ComputeUnit* p)
            : Event(Default_Pri, AutoDelete), parent(p) {}
        void process() override { parent->finish(); }
        const char* description() const override {
            return "ComputeUnit::ComputeDoneEvent";
        }
    };

    ComputeDoneEvent computeDoneEvent;

  public:
    ComputeUnit(Accelerator* _owner, Tick latency);

    void start();
    void finish();
    void abort();

    void pause();
    void resume();
    bool isBusy() const { return busy; }
};

/** Memory port: Accelerator -> Memory (DMA) **/
class MemPort : public MasterPort {
    private:
    Accelerator *owner;

    // DMA State
    Addr dmaAddr;
    uint8_t *dmaBuffer;
    uint32_t dmaCount;
    uint32_t dmaIndex;

    bool paused;
    bool dmaReadMode;
    bool dmaWriteMode;

    public:
    MemPort(const std::string &name, Accelerator *accel);

    /* Basic atomic access */
    uint8_t readAtomic(Addr addr);
    void writeAtomic(Addr addr, uint8_t val);

    /* DMA entry point */
    void startDmaRead(Addr addr, uint8_t *buf, uint32_t count);
    void startDmaWrite(Addr addr, uint8_t *buf, uint32_t count);

    void pauseDma();
    void resumeDma();
    void dmaStep();

    // Must implement these!
    bool recvTimingResp(PacketPtr pkt) override {
        panic("Accelerator MemPort does not support timing responses!\n");
        return false;
    }

    void recvReqRetry() override {
        panic("Accelerator MemPort does not use reqRetry!\n");
    }

    private:
    // DMA Event
    EventWrapper<MemPort, &MemPort::dmaStep> dmaEvent;
};

/** Control port: CPU -> Accelerator (MMIO) **/
class CtrlPort : public SlavePort {
    private:
    Accelerator *owner;
    public:
    CtrlPort(const std::string &name, Accelerator *accel);
    protected:
    Tick recvAtomic(PacketPtr pkt) override;
    void recvFunctional(PacketPtr pkt) override;
    bool recvTimingReq(PacketPtr pkt) override;
    void recvRespRetry() override;
    AddrRangeList getAddrRanges() const override;
};

class Accelerator : public MemObject
{
    friend class ComputeUnit;
    friend class CtrlPort;
    friend class MemPort;

private:
    /* TickEvent for handling periodic energy consumption */
    struct TickEvent : public Event {
        Accelerator *owner;
        TickEvent(Accelerator *owner_this);
        void process();
        const char *description() const override;
    };

    CtrlPort ctrlPort;
    MemPort memPort;
    ComputeUnit computeUnit;

    TickEvent tickEvent;

    void tick();

public:
    typedef AcceleratorParams Params;
    const Params *params() const {
        return reinterpret_cast<const Params *>(_params);
    }

    Accelerator(const Params *p);
    virtual ~Accelerator();

    virtual void init() override;


    /** Gem5 port accessors */
    BaseSlavePort &getSlavePort(const std::string &if_name, PortID idx = InvalidPortID) override;
    BaseMasterPort &getMasterPort(const std::string &if_name, PortID idx = InvalidPortID) override;

    /** Methods to handle packets **/
    Tick recvAtomic(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);
    bool recvTimingReq(PacketPtr pkt);
    void recvRespRetry();
    AddrRange getAddrRanges() const;

    /** Called by EnergyMgr (optional). Return 1 on handled. */
    int handleMsg(const EnergyMsg &msg);

    void triggerInterrupt();

    void handleInterrupt();
    void handleRecovery();

    /** Operation routines */
    void initEvent();
    void doDmaRead();
    void doDmaWrite();
    void doCompute();
    void writeOutputBuffer(uint8_t val);

    /** Energy state (simple enum) */
    enum AccelEnergyState {
        STATE_OFF = 0,
        STATE_IDLE = 2,
        STATE_ON = 4
    };

protected:
    /** CPU / system references */
    BaseCPU *cpu;
    AddrRange controlRange;

    /** I/O Buffers **/
    uint8_t *input_buffer;
    uint8_t *output_buffer;

    /** Control registers (MMIO) */
    uint8_t cmd_reg;   // register to interact with cpu
    Addr src_addr;      // source buffer in system memory
    Addr dst_addr;      // destination buffer in system memory
    bool busy;

    /* cmd_reg bit */
    static const uint8_t CMD_START = (1 << 0);
    static const uint8_t CMD_ABORT = (1 << 1);
    static const uint8_t CMD_DMA_READ = (1 << 2);
    static const uint8_t CMD_DMA_WRITE = (1 << 3);
    static const uint8_t CMD_COMPUTE = (1 << 4);
    static const uint8_t CMD_DONE = (1 << 5);

    /** Status */
    uint32_t count; // number of elements / bytes
    Tick delay_init;
    Tick delay_compute;
    Tick delay_cpu_interrupt;

    double energy_compute_per_tick;
    double energy_idle_per_tick;

    AccelEnergyState energy_state;

    /** Event scheduled when computation finishes */
    EventWrapper<Accelerator, &Accelerator::initEvent> event_init;
};

#endif // GEM5_ACCEL_HH
