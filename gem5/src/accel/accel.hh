/*
 * Head file for gem5 accelerator
 * created by non-ee, 10/24/2025
 */


#include <cstdint>
#include <stdint.h>

class Accelerator : public MemObject
{
    /* Identity of the accelerator */
    protected:
        uint32_t id;
        char dev_names[100];

    private:
        /* Whether the state needs reset */
        bool need_reset;
        /* Definition of TickEvent for Accelerator */
        struct TickEvent : public Event {
            Accelerator *accel;

            TickEvent(Accelerator *accel);
            void process();
        }

        TickEvent tickEvent;
        /* Record the execution state and energy consumption */
        void tick();

        /* Definition of device port for Accelerator */
        class DevicePort : public SlavePort {
            private:
                Accelerator *accel;

            public:
                DevicePort(Accelerator *accel, const std::string &name);
                void recvPacket(PacketPtr pkt);
        }


    public:
        typedef AcceleratorParams Params;
        const Params *params() const {
            return reinterpret_cast<const Param*>(_params);
        }
        Accelerator(const Params *p);
        virtual ~Accelerator();
        virtual void init();

        /* Energy state for accelerator */
        enum AccelEngyState {
            STATE_POWER_OFF = 0,
            STATE_SLEEP = 1,
            STATE_ACTIVE = 2,
        };

        /* Control registers */
        static const uint8_t ACCEL_INIT = 0x80;
        static const uint8_t ACCEL_ACTIVATE = 0x80;
        static const uint8_t ACCEL_INIT = 0x80;

        uint8_t *controlRegister;
        uint8_t *status;
};
