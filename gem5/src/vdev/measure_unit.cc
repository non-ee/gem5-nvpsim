#include "vdev/measure_unit.hh"
#include "engy/state_machine.hh"
#include "debug/MeasureUnit.hh"

MeasureUnit::MeasureUnit(const MeasureUnitParams *p)
    : VirtualDevice(p),
      measuring(false),
      start_tick(0),
      latency(0),
      start_energy(0.0)
{
    DPRINTF(MeasureUnit, "MeasureUnit created: %s\n", name());

    /* Disable inherited active behavior */
    // if (tickEvent.scheduled())
    //     deschedule(tickEvent);
}

void
MeasureUnit::tick()
{
    /* Override: do nothing */
}

void
MeasureUnit::onSimulationExit()
{
    /* Override: do nothing */
}

/* MMIO access override */
Tick
MeasureUnit::access(PacketPtr pkt)
{
    Addr offset = pkt->getAddr() - range.start();

    if (pkt->isWrite()) {
        if (offset == 0) {
            const uint8_t *cmd = pkt->getConstPtr<uint8_t>();

            if (*cmd & 0x1) {   // MEASURE_START
                measuring = true;
                start_tick = curTick();
                latency = 0;

                // start_energy = EnergyObject::getEnergyRemained();

                DPRINTF(MeasureUnit,
                    "[MeasureUnit] START tick=%llu energy=%lf\n",
                    (unsigned long long)start_tick,
                    start_energy);
            }

            else if (*cmd & 0x2) {   // MEASURE_END
                if (measuring) {
                    Tick end_tick = curTick();
                    // double end_energy = EnergyObject::getEnergyRemained();

                    latency += end_tick - start_tick;
                    double energy = 0;

                    measuring = false;

                    DPRINTF(MeasureUnit,
                        "[MeasureUnit] END latency=%llu ticks energy=%lf\n",
                        (unsigned long long)latency,
                        energy);
                }
            }
        }
        else if (offset == 0x00100000) {
            DPRINTF(MeasureUnit, "[MeasureUnit] Temp sensor accessed!");
        }
        else if (offset == 0x00200000) {
            DPRINTF(MeasureUnit, "[MeasureUnit] Voltage sensor accessed!");
        }


        pkt->makeResponse();
        return 0;
    }

    return 1;
    /* Fallback to base behavior if needed */
    // return MeasureUnit::access(pkt);
}

int
MeasureUnit::handleMsg(const EnergyMsg &msg)
{
    if (!measuring)
        return 0;

    if (msg.type == SimpleEnergySM::MsgType::POWER_ON) {
        Tick end_tick = curTick();
        latency += end_tick - start_tick;
    }
    else if (msg.type == SimpleEnergySM::MsgType::POWER_OFF) {
        start_tick = curTick();
    }

    return 1;
}

MeasureUnit *
MeasureUnitParams::create()
{
	return new MeasureUnit(this);
}
