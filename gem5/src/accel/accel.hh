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

class Accelerator : public MemObject, public DmaCallBack, public ComputeCallBack
{

    private:
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


    /* cmd_reg bit */
    static const uint8_t CMD_START = (1 << 0);
    static const uint8_t CMD_INIT = (1 << 1);
    static const uint8_t CMD_ABORT = (1 << 2);
    static const uint8_t CMD_DMA_READ = (1 << 3);
    static const uint8_t CMD_DMA_WRITE = (1 << 4);
    static const uint8_t CMD_COMPUTE = (1 << 5);
    static const uint8_t CMD_CPU_INTERRUPT = (1 << 6);
    static const uint8_t CMD_DONE = (1 << 7);

protected:
    /** CPU / system references */
    BaseCPU* cpu;
    DmaCtrl* dmaCtrl;
    ComputeUnit* computeUnit;
    AddrRange controlRange;

    /** I/O Buffers **/
    uint8_t *input_buffer;
    uint8_t *output_buffer;

    /** Control registers (MMIO) */
    Addr src_addr;      // source buffer in system memory
    Addr dst_addr;      // destination buffer in system memory
    uint32_t count;     // number of elements
    uint8_t cmd_reg;   // register to interact with cpu
    bool busy;
    bool need_recover;

    /** Status */
    Tick delay_init;
    Tick delay_compute;
    Tick delay_cpu_interrupt;

    double energy_per_cycle[3] = {0.0, 0.2, 2.0};
    AccelEnergyState energy_state;

    /** Operation routines */
    void initEvent();

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
    EventWrapper<Accelerator, &Accelerator::initEvent> event_init;
};

#endif // GEM5_ACCEL_HH
