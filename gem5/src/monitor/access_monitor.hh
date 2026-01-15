#ifndef __MONITOR_ACCESS_MONITOR_HH__
#define __MONITOR_ACCESS_MONITOR_HH__

#include "mem/mem_object.hh"
#include "mem/packet.hh"
#include "base/types.hh"
#include "params/AccessMonitor.hh"

class AccessMonitor : public MemObject
{
  public:
    typedef AccessMonitorParams Params;
    AccessMonitor(const Params *p);

    /* gem5-nvp style port access */
    BaseMasterPort& getMasterPort(const std::string &if_name,
                                  PortID idx = InvalidPortID) override;

    BaseSlavePort& getSlavePort(const std::string &if_name,
                                PortID idx = InvalidPortID) override;

    void observe(PacketPtr pkt);

  protected:
    /* ================= SlavePort ================= */
    class MonitorSlavePort : public SlavePort
    {
      public:
        MonitorSlavePort(const std::string &name, AccessMonitor &owner);

        AddrRangeList getAddrRanges() const override;

        Tick recvAtomic(PacketPtr pkt) override;
        void recvFunctional(PacketPtr pkt) override;
        bool recvTimingReq(PacketPtr pkt) override;
        void recvRespRetry() override;

      private:
        AccessMonitor &owner;
    };

    /* ================= MasterPort ================= */
    class MonitorMasterPort : public MasterPort
    {
      public:
        MonitorMasterPort(const std::string &name, AccessMonitor &owner);

        bool recvTimingResp(PacketPtr pkt) override;
        void recvReqRetry() override;

      private:
        AccessMonitor &owner;
    };

  private:
    MonitorSlavePort slavePort;
    MonitorMasterPort masterPort;

    bool measuring;
};

#endif
