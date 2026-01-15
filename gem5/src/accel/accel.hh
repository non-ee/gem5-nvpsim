#ifndef GEM5_ACCEL_HH
#define GEM5_ACCEL_HH

#include "accel/dma_ctrl.hh"
#include "accel/compute_unit.hh"
#include "mem/se_translating_port_proxy.hh"
#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "sim/eventq.hh"
#include "params/Accelerator.hh"
#include "cpu/base.hh"
#include <cstdint>
#include <string>
#include "base/types.hh"

/** Energy state (simple enum) */
enum AccelEnergyState {
    STATE_OFF = 0,
    STATE_IDLE = 1,
    STATE_ON = 2
};

/** Accelerator state **/
enum AccelState {
    IDLE = 0,
    START = 1,
    INIT = 2,
    DMA_READ = 3,
    DMA_WRITE = 4,
    COMPUTE = 5,
    CPU_INT = 6,
    DONE = 7,
    NOP = 8,
    RECOVERY = 9
};

class Accelerator : public MemObject, public DmaCallBack, public ComputeCallBack
{
    private:
    char accel_name[100];

    /* TickEvent for handling periodic energy consumption */
    struct TickEvent : public Event {
        Accelerator *owner;
        TickEvent(Accelerator *owner_this);
        void process();
        const char *description() const override;
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

    CtrlPort ctrlPort;
    TickEvent tickEvent;

    void tick();
    void fsmStep();

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

    /** Methods to handle packets **/
    Tick recvAtomic(PacketPtr pkt);
    void recvFunctional(PacketPtr pkt);
    bool recvTimingReq(PacketPtr pkt);
    void recvRespRetry();
    AddrRange getAddrRanges() const;

    /** DMA Callback methods **/
    void onDmaReadDone() override;
    void onDmaWriteDone() override;
    /** Compute Callback methods **/
    void onComputeDone() override;
    void onComputeAbort() override;

    /** Called by EnergyMgr (optional). Return 1 on handled. */
    int handleMsg(const EnergyMsg &msg);

protected:
    /** CPU / system references */
    BaseCPU* cpu;
    BaseComputeUnit* computeUnit;
    DmaCtrl* dmaCtrl;
    AddrRange controlRange;

    /** I/O Buffers **/
    uint8_t *input_buffer;
    uint8_t *output_buffer;

    /** Control registers (MMIO) */
    Addr src_addr;      // source buffer in system memory
    Addr dst_addr;      // destination buffer in system memory
    uint32_t count;     // number of elements
    AccelState state;
    bool busy;

    /** Energy consumption report**/
    double total_energy_consumed;
    Tick total_ticks;
    Tick total_poweroff_ticks;

    void onSimulationExit();

    /** Status */
    Tick delay_init;
    Tick delay_cpu_interrupt;

    double energy_per_cycle[3] = {0.0, 0.2, 2.0};
    AccelEnergyState energy_state;

    /** Operation routines */
    void initDone();

    void doInit();
    void doDmaRead();
    void doDmaWrite();
    void doCompute();
    void triggerInterrupt();
    void finishSuccess();
    void abortCompute();

    void handleInterrupt();
    void handleRecovery();

    /** Event scheduled when computation finishes */
    EventWrapper<Accelerator, &Accelerator::initDone> event_init;
};

#endif // GEM5_ACCEL_HH
