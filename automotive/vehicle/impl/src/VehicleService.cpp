#define LOG_TAG __FILE_NAME__
#include <log/log_main.h>

#include <chrono>
#include <thread>

#include <hidl/HidlTransportSupport.h>
#include <vhal_v2_0/VehicleHalManager.h>
#include "../../../../../../../external/automotive-source/dlt-daemon-2.18.10/include/dlt/dlt.h"
#include "VehicleHalImpl.h"
#include "hw-clients/Simulator.h"

using namespace std::chrono_literals;

using namespace vendor::marelli::vehicle::V1_0;

using namespace android;
using namespace android::hardware;
using namespace android::hardware::automotive::vehicle::V2_0;

DltContext SYIN;
DltContext STLA;

int main(int argc, char const* argv[])
{
    static_cast<void>(argc);
    static_cast<void>(argv);

    struct timespec ts;

    //DLT_REGISTER_APP(APPID,DESCRIPTION)
    DLT_REGISTER_APP("VHAL", "register app");

    //DLT_REGISTER_CONTEXT(CONTEXTNAME, CONTEXTID, DESCRIPTION)
    DLT_REGISTER_CONTEXT(SYIN, "SYIN", "register context");
    
   	//Registering another context id for printing stla imp logs
	  DLT_REGISTER_CONTEXT(STLA, "STLA_LOGS", "register context");

    ALOGI("%s(): Hello World form Marelli VHAL!\n", __func__);
	std::this_thread::sleep_for(std::chrono::seconds(10));

    auto store = std::make_unique<vhal_v2_0::VehiclePropertyStore>();

    // HAL client where car signals (e.g. CAN) are transformed to VHAL properties and vice versa.
    // Now Simulator is used, where VENDOR properties are simulated only,
    // Google's SYSTEM properties are dummy and have only init values from DefaultConfig.h.
    auto hw_client = std::make_unique<impl::Simulator>();
    hw_client->start();

    auto hal = std::make_unique<impl::VehicleHalImpl>(store.get(), hw_client.get());
    auto service = std::make_unique<VehicleHalManager>(hal.get());

    configureRpcThreadpool(1, true); // TODO: to understand

    ALOGI("%s(): Registering as service...", __func__);
    status_t status = service->registerAsService();

    if (status != OK)
    {
        ALOGE("%s(): Unable to register vehicle service (%d)", __func__, status);
        return 1;
    }

    ALOGI("%s(): Vehicle service ready", __func__);
    joinRpcThreadpool();

    ts.tv_sec = 0;
    ts.tv_nsec = 1000000;
    nanosleep(&ts, NULL);

    //DLT_UNREGISTER_CONTEXT(CONTEXTID)
    DLT_UNREGISTER_CONTEXT(SYIN);
    DLT_UNREGISTER_CONTEXT(STLA);
  
    //DLT_UNREGISTER_APP()
    DLT_UNREGISTER_APP();

    return 0;
}
