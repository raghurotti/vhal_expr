#include <thread>
#include <iostream>

#include <CommonAPI/CommonAPI.hpp>
#include "CAPIVehicleStubImpl.hpp"


int main() {
    //CommonAPI::Runtime::setProperty("LogContext", "E02S");
    //CommonAPI::Runtime::setProperty("LogApplication", "E02S");
    //CommonAPI::Runtime::setProperty("LibraryBase", "E02Attributes");

    std::shared_ptr<CommonAPI::Runtime> runtime = CommonAPI::Runtime::get();

    std::string domain = "local";
    std::string instance = "com.marelli.vehicle";
    std::string connection = "service-sample";

    std::shared_ptr<VehicleStubImpl> myService = std::make_shared<VehicleStubImpl>();
    while (!runtime->registerService(domain, instance, myService, connection)) {
//    while (!runtime->registerService(domain, instance, myService)) {
        std::cout << "Register Service failed, trying again in 100 milliseconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Successfully Registered Service!" << std::endl;

    while (true) {
        myService->incCounter(); // Change value of attribute, see stub implementation
        //std::cout << "Waiting for calls... (Abort with CTRL+C)" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    return 0;
}