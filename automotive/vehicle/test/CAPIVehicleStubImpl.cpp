#include "CAPIVehicleStubImpl.hpp"

VehicleStubImpl::VehicleStubImpl() {
    cnt = 0;
    add = true;
}

VehicleStubImpl::~VehicleStubImpl() {
}

void VehicleStubImpl::incCounter() {
    add?cnt++:cnt--;
    if(cnt == 0)add=true;
    if(cnt > 50)add=false;
    float l_speed = 1.2;
    l_speed += 0.1 * cnt;
    setINFO_VINAttribute("lkhlkhlkh");
    setINFO_FUEL_CAPACITYAttribute(10.2);
    setFUEL_DOOR_OPENAttribute(cnt%2?true:false);
    setPERF_VEHICLE_SPEEDAttribute(l_speed);
    //std::cout <<  "New counter value = " << cnt << "!" << std::endl;
}