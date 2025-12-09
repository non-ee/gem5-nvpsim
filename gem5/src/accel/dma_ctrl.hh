#pragma once
#include <cstdint>
#include "accel/mem_if.hh"
#include "cpu/base.hh"
#include "sim/clocked_object.hh"
#include "sim/eventq.hh"
#include "params/DmaCtrl.hh"

struct DmaCallBack {
    virtual void onDmaReadDone() = 0;
    virtual void onDmaWriteDone() = 0;
    virtual ~DmaCallBack() = default;
};

struct DmaTask {
    Addr addr;
    uint8_t* buf;
    size_t sizeLeft;
    DmaCallBack* cb;

    DmaTask() : addr(0), buf(nullptr), sizeLeft(0), cb(nullptr) {}
    DmaTask(Addr a, uint8_t* b, size_t s, DmaCallBack* c)
        : addr(a), buf(b), sizeLeft(s), cb(c) {}
};

class DmaCtrl : public ClockedObject
{
    private:
        struct TickEvent : public Event {
            DmaCtrl* ctrl;
            TickEvent(DmaCtrl* c);
            void process();
            const char *description() const;
        };

        TickEvent tickEvent;
        void tick();


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
        void startRead(Addr addr, uint8_t* buf, size_t size, DmaCallBack* cb);

        void startWrite(Addr addr, uint8_t* buf, size_t size, DmaCallBack* cb);

        void doRead();
        void doWrite();

    private:
        BaseCPU *cpu;
        PortProxy* portProxy;
        MemoryInterface* mem;
        size_t bandwidth;
        double energy_per_tx;

        bool active;

        // current task
        DmaTask readTask;
        DmaTask writeTask;

        // events
        EventWrapper<DmaCtrl, &DmaCtrl::doRead> readEvent;
        EventWrapper<DmaCtrl, &DmaCtrl::doWrite> writeEvent;

        // debug
        bool debug_io;
};
