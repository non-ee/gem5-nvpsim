#pragma once
#include <cstdint>
#include "mem/port_proxy.hh"

struct MemoryInterface {
    virtual bool read(Addr addr, uint8_t *buf, size_t len) = 0;
    virtual bool write(Addr addr, const uint8_t *buf, size_t len) = 0;
    virtual ~MemoryInterface() = default;
};

class AccelMemInterface : public MemoryInterface {
    private:
        PortProxy& proxy;

    public:
        AccelMemInterface(PortProxy& port_proxy) : proxy(port_proxy) {}

        bool read(Addr addr, uint8_t *buf, size_t len) override {
            proxy.readBlob(addr, buf, len);
            return true;
        }

        bool write(Addr addr, const uint8_t *buf, size_t len) override {
            proxy.writeBlob(addr, buf, len);
            return true;
        }
};
