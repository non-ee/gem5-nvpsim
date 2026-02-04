#pragma once
#include <cstdint>
#include "accel/mem_if.hh"
#include "cpu/base.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "params/DmaCtrl.hh"
#include <functional>

enum DmaCtrlOp {
    OFF = 0,
    READ = 1,
    WRITE = 2
};

struct DmaCallBack {
    virtual void onDmaReadDone() = 0;
    virtual void onDmaWriteDone() = 0;
    virtual ~DmaCallBack() = default;
};

struct DmaTask {
    Addr addr;
    uint8_t* buf;
    size_t size;
    std::function<void()> cb;

    DmaCtrlOp op = OFF;

    DmaTask()
        : addr(0), buf(nullptr), size(0), cb(nullptr), op(OFF) {}
    DmaTask(Addr a, uint8_t* b, size_t s, std::function<void()> c, DmaCtrlOp o = OFF)
        : addr(a), buf(b), size(s), cb(c), op(o) {}
};

class DmaCtrl : public ClockedObject
{
    public:
        typedef DmaCtrlParams Params;
        const Params *params() const {
            return reinterpret_cast<const Params*>(_params);
        }
        DmaCtrl(const Params* p);
        virtual ~DmaCtrl();
        virtual void init();
        virtual int handleMsg(const EnergyMsg& msg);

        // Simple async read/write
        void startRead(Addr addr, uint8_t* buf, size_t size, std::function<void()> cb);
        void startWrite(Addr addr, uint8_t* buf, size_t size, std::function<void()> cb);

        bool active() const {
            return dmaTask.op == READ || dmaTask.op == WRITE;
        }

    private:
        BaseCPU *cpu;
        PortProxy* portProxy;
        MemoryInterface* mem;

        bool inTask;

        Tick latency_access_per_byte;
        double energy_access_per_byte;

        Tick access_latency;

        // Tasks
        DmaTask dmaTask;

        void doDma();

        // events
        EventWrapper<DmaCtrl, &DmaCtrl::doDma> dmaEvent;
};
