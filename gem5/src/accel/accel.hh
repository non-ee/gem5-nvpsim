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
#include <stdint.h>
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

    static const uint8_t ACCEL_IDLE = 0;
    static const uint8_t ACCEL_INIT = 1;
    static const uint8_t ACCEL_DMA_READ = 2;
    static const uint8_t ACCEL_DMA_WRITE = 3;
    static const uint8_t ACCEL_COMPUTE = 4;
    static const uint8_t ACCEL_INTERRUPT = 5;
    static const uint8_t CMD_MASK = 0x07;

    static const uint8_t INIT_BIT = (1 << 4);
    static const uint8_t BUSY_BIT = (1 << 5);
    static const uint8_t DONE_BIT = (1 << 6);


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

    uint8_t cmd;

    /** Energy consumption report**/
    double total_energy_consumed;
    Tick total_tick;

    void onSimulationExit();

    /** Status */
    Tick delay_init;
    Tick delay_cpu_interrupt;

    double energy_per_cycle[3] = {0.0, 0.2, 2.0};
    AccelEnergyState energy_state;

    /** Cmd manipulation **/
    void setCmd(uint8_t accel_cmd);

    /** Operation routines */
    void triggerInterrupt();
    void doInit();
    void doDmaRead();
    void doDmaWrite();
    void doCompute();
    void doInterrupt();
    void finishSuccess();
    void abortCompute();

    void handleInterrupt();
    void handleRecovery();

    /** Event scheduled when computation finishes */
    EventWrapper<Accelerator, &Accelerator::triggerInterrupt> event_interrupt;
};

#endif // GEM5_ACCEL_HH
