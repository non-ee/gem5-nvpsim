#ifndef __MEASURE_UNIT_HH__
#define __MEASURE_UNIT_HH__

#include "vdev/vdev.hh"
#include "params/MeasureUnit.hh"

class MeasureUnit : public VirtualDevice
{
  public:
    MeasureUnit(const MeasureUnitParams *p);
    ~MeasureUnit() override = default;

  protected:
    // Override VirtualDevice behaviors
    void tick() override;
    Tick access(PacketPtr pkt) override;
    int handleMsg(const EnergyMsg &msg) override;

    void onSimulationExit() override;

  private:
    bool measuring;
    Tick start_tick;
    Tick latency;
    double start_energy;
};

#endif
