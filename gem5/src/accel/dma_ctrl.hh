#pragma once
#include <cstdint>
#include "accel/mem_if.hh"
#include "sim/clocked_object.hh"
#include "params/DmaCtrl.hh"

struct DmaTask {
    Addr addr;
    uint8_t* buf;
    size_t sizeLeft;
    DmaCallBack* cb;
};

struct DmaCallBack {
    virtual void onDmaReadDone() = 0;
    virtual void onDmaWriteDone() = 0;
    virtual ~DmaCallBack() = default;
};

class DmaCtrl : public ClockedObject
{
    private:
        struct TickEvent : public Event {
            DmaCtrl* ctrl;
            TickEvent(DmaCtrl* c) : ctrl(c) {}
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
        virtual int handleMessage(const EnergyMsg& msg);

        void setMemoryInterface(MemoryInterface* m);

        // Simple async read/write
        void startRead(Addr addr, uint8_t* buf,
                       size_t size,
                       DmaCallBack* cb);

        void startWrite(Addr addr, const uint8_t* buf,
                        size_t size,
                        DmaCallBack* cb);

        void doRead();
        void doWrite();

    private:
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
};
