#pragma once


#include <CommonAPI/CommonAPI.hpp>
#include "v1/com/marelli/vehicleStubDefault.hpp"

class VehicleStubImpl: public v1::com::marelli::vehicleStubDefault {

public:
    VehicleStubImpl();
    virtual ~VehicleStubImpl();
    virtual void incCounter();

private:
    int cnt;
    bool add;
};