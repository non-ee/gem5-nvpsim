/*
 * Head file for gem5 accelerator
 * created by non-ee, 10/24/2025
 */

 #ifndef GEM5_ACCELERATOR_HH
 #define GEM5_ACCELERATOR_HH

 #include "mem/mem_object.hh"
 #include "sim/eventq.hh"
 #include "cpu/base.hh"
 #include "params/Accelerator.hh"
 #include "energy/energy_msg.hh"

 class Accelerator : public MemObject
 {
   public:
     typedef AcceleratorParams Params;
     const Params *params() const {
         return reinterpret_cast<const Params *>(_params);
     }

     Accelerator(const Params *p);
     virtual ~Accelerator();

     void init() override;

     /** Port to connect with CPU (MMIO access) **/
     class CtrlPort : public SlavePort
     {
       private:
         Accelerator *accel;

       public:
         CtrlPort(const std::string &name, Accelerator *_accel);

       protected:
         Tick recvAtomic(PacketPtr pkt) override;
         void recvFunctional(PacketPtr pkt) override;
         bool recvTimingReq(PacketPtr pkt) override;
         void recvRespRetry() override;
         AddrRangeList getAddrRanges() const override;
     };

     CtrlPort ctrlPort;

     /** Event to handle computation done **/
     EventFunctionWrapper doneEvent;

     /** CPU interrupt trigger **/
     void triggerInterrupt();

     /** Communication with energy manager **/
     virtual int handleMsg(const EnergyMsg &msg);

     /** gem5 required overrides **/
     BaseSlavePort &getSlavePort(const std::string &if_name, PortID idx = InvalidPortID) override;
     AddrRange getAddrRange() const;

   protected:
     /** Internal state **/
     BaseCPU *cpu;
     AddrRange range;

     /** Control registers (accessible by CPU) **/
     uint32_t cmd_reg;
     uint32_t src_addr;
     uint32_t dst_addr;
     uint32_t size;
     uint32_t status_reg;

     /** Accelerator state **/
     bool busy;
     Tick compute_latency;
     double energy_per_cycle;

     /** Energy management **/
     enum AccelEngyState {
         STATE_OFF = 0,
         STATE_SLEEP,
         STATE_IDLE,
         STATE_ACTIVE
     };
     AccelEngyState energy_state;

     /** Internal operation handlers **/
     void startCompute();
     void finishCompute();
 };

 #endif // GEM5_ACCELERATOR_HH
