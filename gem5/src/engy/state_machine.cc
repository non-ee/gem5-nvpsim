//
// Created by lf-z on 3/13/17.
// Update by tongda on 3/14/18.
//

#include "engy/state_machine.hh"
#include "debug/EnergyMgmt.hh"
#include "debug/SimpleEnergySM.hh"
#include "debug/MeasureUnit.hh"
#include <fstream>

/******* BaseEnergySM *******/
BaseEnergySM::BaseEnergySM(const Params *p) : SimObject(p), mgmt(NULL)
{
	energy_consume_lower_bound = 0;
}

void
BaseEnergySM::broadcastMsg(const EnergyMsg &msg)
{
	mgmt->broadcastMsgAsEvent(msg);
}

/******* SimpleEnergySM *******/
SimpleEnergySM::SimpleEnergySM(const Params *p) :
	BaseEnergySM(p),
	state(SimpleEnergySM::State::STATE_POWER_OFF),
	thres_1_to_off(p->thres_1_to_off),
	thres_off_to_1(p->thres_off_to_1)
{
	// when the system cannot consume energy
	energy_consume_lower_bound = thres_1_to_off;

	// Initialize variables for outage latency tracking
	outage_start_tick = 0;
	total_charging_time = 0;
	in_outage = false;

	/* register end-of simulation callback */
    registerExitCallback(
        new MakeCallback<SimpleEnergySM, &SimpleEnergySM::onSimulationExit>(this)
    );

}

void
SimpleEnergySM::onSimulationExit(){
    // Write total simulation ticks to file
    std::ofstream fout("m5out/ticks_output.txt", std::ios::app);
    fout << "Total charging ticks: " << total_charging_time << std::endl;
    fout.close();
}


void
SimpleEnergySM::init()
{
	EnergyMsg msg;
	msg.val = 0;
	state = State::STATE_POWER_OFF;
	msg.type = MsgType::POWER_OFF;
	broadcastMsg(msg);

	//
	std::ofstream fout;
	fout.open("m5out/power_failure", std::ios::app);
	assert(fout);
	fout << outage_times << std::endl;
	fout.close();

	//
	in_outage = true;
	outage_start_tick = curTick();
	total_charging_time = 0;
	fout.open("m5out/powerfailure_report", std::ios::app);
	assert(fout);
	fout << "Start tick: " << outage_start_tick << std::endl;
	fout.close();

	DPRINTF(MeasureUnit, "[MeasureUnit] Initialized SimpleEnergySM. Charging time: %lu\n", total_charging_time);
}

void SimpleEnergySM::update(double _energy)
{
	EnergyMsg msg;
	msg.val = 0;

	// power failure
	if (state == STATE_POWER_ON && _energy <= thres_1_to_off)
	{
		DPRINTF(EnergyMgmt, "[SimpleEnergySM] State change: POWER_ON->POWER_OFF, energy=%lf, thres=%lf.\n", _energy, thres_1_to_off);
		DPRINTF(MeasureUnit, "[MeasureUnit] Power failure detected.\n");
		state = State::STATE_POWER_OFF;
		msg.type = MsgType::POWER_OFF;

		// record outage start time
		outage_start_tick = curTick();

		// Calculate Power failure times
		outage_times++;
		std::ofstream fout("m5out/power_failure");
		//fout.open("m5out/power_failure", std::ios::app);
		assert(fout);
		fout << outage_times << std::endl;
		fout.close();

		broadcastMsg(msg);
	}

	// power recovery
	else if (state == State::STATE_POWER_OFF && _energy >= thres_off_to_1)
	{
		DPRINTF(EnergyMgmt, "[SimpleEnergySM] State change: POWER_OFF->POWER_ON, energy=%lf, thres=%lf.\n", _energy, thres_off_to_1);
		state = State::STATE_POWER_ON;
		msg.type = MsgType::POWER_ON;

		Tick outage_latency = curTick() - outage_start_tick;
  		total_charging_time += outage_latency;
        DPRINTF(MeasureUnit, "[MeasureUnit] Power recovery detected. Charging time: %lu\n", outage_latency);

		broadcastMsg(msg);
	}
}

BaseEnergySM *
BaseEnergySMParams::create()
{
	return new BaseEnergySM(this);
}

SimpleEnergySM *
SimpleEnergySMParams::create()
{
	return new SimpleEnergySM(this);
}
