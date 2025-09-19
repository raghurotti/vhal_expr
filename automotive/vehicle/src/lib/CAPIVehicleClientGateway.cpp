#include <android-base/logging.h>
#include <log/log_main.h>
#include "android/log.h"
#include "CAPIVehicleClientGateway.h"
#include "CalibrationDataStruct.h"
#include <v0/iMicro/SOC_LIFECYCLEProxy.hpp>
#include <v0/iMicro/SOC_Channel_CANProxy.hpp>
#include <v0/iMicro/SOC_Channel_DIAGProxy.hpp>
#include <v0/iMicro/SOC_Channel_CALIBProxy.hpp>
#include <v0/iMicro/SOC_IMUProxy.hpp>
#include <v0/iMicro/SOC_DIAGProxy.hpp>
#include <CommonAPI/CommonAPI.hpp>
#include <CommonAPI/Extensions/AttributeCacheExtension.hpp>
#include <iMicro/pmStructs.hpp>
#include <v0/iMicro/SOC_DEMProxy.hpp>
#include <v0/iMicro/SOC_FIRSTProxy.hpp>
#include <utility>
#include <iomanip>
#include <iostream>
#include <vector>
#include <atomic>
#include <fstream>
#include "tcuStubImpl.hpp"
#include <v0/VHAL/VHAL_LIFECYCLEProxy.hpp> 
#include <sys/stat.h>
#include <mutex>
#include <v1/VHAL/LOGR/LOGR_VHALInterfaceProxy.hpp>
#include <ctime>
#include <VHAL/LOGR/LOGR_Types.hpp>
#include <sys/system_properties.h>
#include <string.h>
#include <sstream>
uint32_t TCU_Data[TCU_InternalSignals_Index];
uint32_t SLI_Data[SLI_data_length];
//uint16_t FOTA_HMI_INTDATA[FOTA_HMIIntData_lenth];
uint32_t flag = 0;
std::string packagePath_SW;
std::string FotaHmiCheckforUpdateResult_S;
std::string FotaHmiUpdateAvailable_S;
std::string FotaHmiConditionsNotMet_S;
std::string FotaHmiInstallationStatus_S;
std::string FotaHmiUpdateFinished_S;
std::string FotaHmiWhatsNewDetails_S;
std::string FotaHmiUpdateHistory_S;
std::string FotaHmiPendingUpdates_S;
std::string FotaHmiScheduleUpdate_S;
std::string AOSPRBUATrigger_S;
std::string FOTAHMI_InfoResult_S;
std::string USBUpdate_Details_S;
//std::vector<uint32_t> srecFile_calibData(5233, 0); 

uint32_t log_extraction = 0;
uint32_t Dote_data[117];
#if defined ANDROID || defined __ANDROID__
#include "android/log.h"
#define LOG_TAG "vsomeip_client"
#define LOG_INF(...)  (void)__android_log_print(ANDROID_LOG_INFO, LOG_TAG, ##__VA_ARGS__)
#define LOG_ERR(...)  (void)__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, ##__VA_ARGS__)
#endif

// //IDS
using vendor::securethings::hardware::ids::ids_client::V1_0::IIdsClient;
using vendor::securethings::hardware::ids::ids_client::V1_0::IDSReturnValue;
using android::hardware::Return;
using android::hardware::Void;
using android::hardware::configureRpcThreadpool;
using android::hardware::joinRpcThreadpool;
using android::sp;
// //IDS_END

namespace android {
namespace hardware {
namespace automotive {
namespace vehicle {
namespace V2_0 {

static uint16_t kx =0;
static uint16_t lx =0;


class CAPIVehicleClientGateway::VehicleClientImpl {
public:
    VehicleClientImpl() : m_vehicle_proxy(nullptr) , m_vehicle_proxy_CANChanel(nullptr) ,m_vehicle_calibChannelProxy(nullptr),
	m_diagChannelProxy(nullptr),m_vehicle_IMUProxy(nullptr),m_DIAGProxy(nullptr),m_DEMProxy(nullptr),m_FIRSTProxy(nullptr),m_TCU_proxy(nullptr){ }
		
	    void initialize() {
	static uint16_t count_calib = 0U;
			int cnt =0;
        std::shared_ptr < CommonAPI::Runtime > runtime = CommonAPI::Runtime::get();
 
 	    /*lifecycle CHANNEL*/
        std::string domain = "local";
        std::string instance = "iMicro.SOC_LIFECYCLE"; 
        std::string connection = "client-lifecycle";
        LOG(INFO) << "CAPIVehicleClientGateway :Registering one proxy...";
        m_vehicle_proxy = runtime->buildProxy<v0::iMicro::SOC_LIFECYCLEProxy>(domain, instance, connection);
	if(m_vehicle_proxy != nullptr){
	while (!m_vehicle_proxy->isAvailable()) {
		cnt++;
		if(cnt >=200){
			break;
		}
        std::this_thread::sleep_for(std::chrono::microseconds(10));
		}}
	
	    /*DIAG CHANNEL*/
         domain = "local";
        instance = "iMicro.SOC_Channel_DIAG"; 
        connection = "client-diag-channel";
        m_diagChannelProxy = runtime->buildProxy<v0::iMicro::SOC_Channel_DIAGProxy>(domain, instance, connection);
        LOG(INFO) << "Waiting for diag-channel service to become available.";
     	cnt =0;
	 if(m_diagChannelProxy != nullptr){
	while (!m_diagChannelProxy->isAvailable()) {
			cnt++;
		if(cnt >=200){
			break;
		}
        std::this_thread::sleep_for(std::chrono::microseconds(10));
	}}
	/*DIAG Proxy*/
	    domain = "local";
    instance = "iMicro.SOC_DIAG"; 
    connection = "client-diag-soc";
    m_DIAGProxy = runtime->buildProxy<v0::iMicro::SOC_DIAGProxy>(domain, instance, connection);
    LOG_INF("Waiting for SOC_DIAG service to become available.\n");
   cnt =0;
	 if(m_DIAGProxy != nullptr){
	while (!m_DIAGProxy->isAvailable()) {
			cnt++;
		if(cnt >=200){
			break;
		}
        std::this_thread::sleep_for(std::chrono::microseconds(10));
	}}
		  /* DEM DTC Proxy*/
		  domain = "local";
          instance = "iMicro.SOC_DEM"; 
          connection = "client-dem-soc";
          m_DEMProxy = runtime->buildProxy<v0::iMicro::SOC_DEMProxy>(domain, instance, connection);
		  LOG_INF("Waiting for SOC_DEM service to become available.\n");
          cnt =0;
	    if(m_DEMProxy != nullptr){
	    while (!m_DEMProxy->isAvailable()) {
			cnt++;
		if(cnt >=200){
			break;
		}
        std::this_thread::sleep_for(std::chrono::microseconds(10));
	}}
	   
		/*CAN CHANNEL*/
		domain = "local";
        instance = "iMicro.SOC_Channel_CAN"; 
        connection = "client-can-channel";
        m_vehicle_proxy_CANChanel = runtime->buildProxy<v0::iMicro::SOC_Channel_CANProxy>(domain, instance, connection);
		LOG(INFO) << "Waiting for CAN-channel service to become available.";
		cnt =0;
     if(m_vehicle_proxy_CANChanel != nullptr){
		while (!m_vehicle_proxy_CANChanel->isAvailable()) {
		cnt++;
		if(cnt >=200){
			break;
		}
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }}
		    /*CALIBRATION CHANNEL*/
        domain = "local";
        instance = "iMicro.SOC_Channel_CALIB"; 
        connection = "client-calib-channel";
        m_vehicle_calibChannelProxy = runtime->buildProxy<v0::iMicro::SOC_Channel_CALIBProxy>(domain, instance, connection);
        LOG(INFO) << "Waiting for CALIBRATION-channel service to become available.";
		cnt =0;
     if(m_vehicle_calibChannelProxy != nullptr){
		while (!m_vehicle_calibChannelProxy->isAvailable()) {
		cnt++;
		if(cnt >=200){
			break;
		}
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }}
			    /*IMU DATA CHANNEL*/
	    domain = "local";
        instance = "iMicro.SOC_IMU"; 
        connection = "client-imu-soc";
        m_vehicle_IMUProxy = runtime->buildProxy<v0::iMicro::SOC_IMUProxy>(domain, instance, connection);
        LOG_INF("Waiting for SOC_IMU service to become available.\n");
		cnt =0;
     if(m_vehicle_IMUProxy != nullptr){
       while (!m_vehicle_IMUProxy->isAvailable()) {
		 cnt++;
		if(cnt >=200){
			break;
		}
		std::this_thread::sleep_for(std::chrono::microseconds(10));
	   }
	 }
     /* FIRST Proxy */
		  domain = "local";
          instance = "iMicro.SOC_FIRST"; 
          connection = "client-first-soc";
          m_FIRSTProxy = runtime->buildProxy<v0::iMicro::SOC_FIRSTProxy>(domain, instance, connection);
		  LOG_INF("Waiting for SOC_FIRST service to become available.\n");
          cnt =0;
	    if(m_FIRSTProxy != nullptr){
	    while (!m_FIRSTProxy->isAvailable()) {
			cnt++;
		if(cnt >=200){
			break;
		}
        std::this_thread::sleep_for(std::chrono::microseconds(10));
	}}

    	    /*TCU CLIENT PROXY*/
            domain = "local";
            instance = "VHAL.LOGR.LOGR_VHALInterface"; 
            connection = "client-logretrievr";
            m_TCU_proxy = runtime->buildProxy<v1::VHAL::LOGR::LOGR_VHALInterfaceProxy>(domain, instance, connection);
            LOG_INF("Waiting for LOGR_Interface service to become available.\n");
            cnt =0;
         if(m_TCU_proxy != nullptr){
           while (!m_TCU_proxy->isAvailable()) {
             cnt++;
            if(cnt >=200){
                break;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(10));
           }
         }

	   LOG(INFO) << "CAPIVehicleClientGateway :Registering  proxy... end "; 
    }

	
    bool isAvailable() const {
        bool proxyStatus = false;
		if(m_vehicle_proxy_CANChanel != nullptr)
			proxyStatus = m_vehicle_proxy_CANChanel->isAvailable();
        return proxyStatus;
    }
	   bool isAvailable_dotteProxy() const {
       bool proxyStatus = false;
		if(m_vehicle_proxy != nullptr)
		{
			proxyStatus = m_vehicle_proxy->isAvailable();		
		}
        return proxyStatus;
    }
	
	void setIVI_TX_Signals(const uint16_t IVI_TX_Signal , uint32_t tx_enum_number){
    std::stringstream logStream;
	CommonAPI::CallStatus callStatus;
    unsigned j = 0; 
	unsigned i = tx_enum_number;
	uint16_t IVI_TX_Signals[]{IVI_TX_Signal};
	uint8_t TX_signals_arr[TX_Buffer_size] = {0};
	uint16_t Signal_index = i;
	uint32_t _result;
	unsigned k = 0U;
	TX_signals_arr[k++] = (Signal_index >> 8) & 0xFF;
	TX_signals_arr[k++] = (Signal_index >> 0) & 0xFF;
	TX_signals_arr[k++] = 0U;
	TX_signals_arr[k++] = 0U;
	TX_signals_arr[k++] = (IVI_TX_Signals[j] >> 0) & 0xFF;
	TX_signals_arr[k++] = (IVI_TX_Signals[j] >> 8) & 0xFF;

    CommonAPI::ByteBuffer bufferCAN(TX_Buffer_size);
    std::memcpy(bufferCAN.data(), &TX_signals_arr, TX_Buffer_size);
	if(m_vehicle_proxy_CANChanel != nullptr)
	m_vehicle_proxy_CANChanel->SendRaw(bufferCAN,callStatus,_result,nullptr);
    else{
	LOG(INFO) << "CAPIVehicleClientGateway :IVI_TX_Signals_data proxy not available"; }
    /*for(unsigned k = 0; k < TX_Buffer_size; k++)
    ALOGD("CAPIVehicleClientGateway :IVI_TX_Signals_data %d %d",k,TX_signals_arr[k]);*/
 	
}
 void setIVI_CalibrationTriggerStatus(uint16_t& IVICalibTriggerStatus){
	CommonAPI::CallStatus callStatus;
	CommonAPI::ByteBuffer bufferCAN(1);
	uint32_t _result;
	int8_t calib_arr[1] = {0};
	calib_arr[0] = IVICalibTriggerStatus;
    std::memcpy(bufferCAN.data(), &calib_arr, 1);
	LOG(INFO) << "CAPIVehicleClientGateway :IVI CALIB  Trigger send successfully"<< (uint32_t)IVICalibTriggerStatus;
	if(m_vehicle_calibChannelProxy != nullptr)
	m_vehicle_calibChannelProxy->SendRaw(bufferCAN,callStatus,_result,nullptr);
    else{
	  LOG(INFO) << "CAPIVehicleClientGateway :IVI CALIB  proxy not available"; }
}

 void setIVI_LogExtractionTriggerStatus(uint16_t& IVILogDTCExtractionStatus){
    log_extraction ++;
	CommonAPI::CallStatus callStatus;
	if (m_vehicle_proxy != nullptr) {
	m_vehicle_proxy->SendEvent_LogExtractionTrigger_SOC(IVILogDTCExtractionStatus, callStatus, nullptr );
	LOG(INFO) << "CAPIVehicleClientGateway : setIVI_LogExtractionTriggerStatus" << IVILogDTCExtractionStatus;
	}
    else {
	LOG(INFO) << "CAPIVehicleClientGateway : setIVILogExtractionTrigger *** Proxy not available";}
    if(log_extraction > 1)
    {
	LOG(INFO) << "log_extraction:" << log_extraction;
    notifydLTVectorParamUpdated(McpuVcpuSwVersion);
    for (unsigned i = 0; i < McpuVcpuSwVersion.size(); i++) {
        LOG(INFO) << "[McpuVcpuSwVersion] data[" << i << "] = " << static_cast<int>(McpuVcpuSwVersion[i]);
    }

    notifydLTVectorParamUpdated(EcuSerialNumber);
    for (unsigned i = 0; i < EcuSerialNumber.size(); i++) {
        LOG(INFO) << "[EcuSerialNumber] data[" << i << "] = " << static_cast<int>(EcuSerialNumber[i]);
    }

    notifydLTVectorParamUpdated(EcuHwVersionNumber);
    for (unsigned i = 0; i < EcuHwVersionNumber.size(); i++) {
        LOG(INFO) << "[EcuHwVersionNumber] data[" << i << "] = " << static_cast<int>(EcuHwVersionNumber[i]);
    }

    notifydLTVectorParamUpdated(EcuHwPartNumber);
    for (unsigned i = 0; i < EcuHwPartNumber.size(); i++) {
        LOG(INFO) << "[EcuHwPartNumber] data[" << i << "] = " << static_cast<int>(EcuHwPartNumber[i]);
    }

    notifydLTVectorParamUpdated(EcuHwNumbers);
    for (unsigned i = 0; i < EcuHwNumbers.size(); i++) {
        LOG(INFO) << "[EcuHwNumbers] data[" << i << "] = " << static_cast<int>(EcuHwNumbers[i]);
    }

    notifydLTVectorParamUpdated(EcuSwVersionNumber);
    for (unsigned i = 0; i < EcuSwVersionNumber.size(); i++) {
        LOG(INFO) << "[EcuSwVersionNumber] data[" << i << "] = " << static_cast<int>(EcuSwVersionNumber[i]);
    }

    notifydLTVectorParamUpdated(EcuSwPartNumber);
    for (unsigned i = 0; i < EcuSwPartNumber.size(); i++) {
        LOG(INFO) << "[EcuSwPartNumber] data[" << i << "] = " << static_cast<int>(EcuSwPartNumber[i]);
    }

    notifydLTVectorParamUpdated(EcuSwNumbers);
    for (unsigned i = 0; i < EcuSwNumbers.size(); i++) {
        LOG(INFO) << "[EcuSwNumbers] data[" << i << "] = " << static_cast<int>(EcuSwNumbers[i]);
    }

    notifydLTVectorParamUpdated(EcuSparePartNumber);
    for (unsigned i = 0; i < EcuSparePartNumber.size(); i++) {
        LOG(INFO) << "[EcuSparePartNumber] data[" << i << "] = " << static_cast<int>(EcuSparePartNumber[i]);
    }

    notifydLTVectorParamUpdated(Read_Cyber_UIN);
    for (unsigned i = 0; i < Read_Cyber_UIN.size(); i++) {
        LOG(INFO) << "[Read_Cyber_UIN] data[" << i << "] = " << static_cast<int>(Read_Cyber_UIN[i]);
    }

    notifydLTVectorParamUpdated(ECUManufacturingDateData);
    for (unsigned i = 0; i < ECUManufacturingDateData.size(); i++) {
        LOG(INFO) << "[ECUManufacturingDateData] data[" << i << "] = " << static_cast<int>(ECUManufacturingDateData[i]);
    }

    notifydLTVectorParamUpdated(SupplierManufacturerECUSoftwareVersionNum);
    for (unsigned i = 0; i < SupplierManufacturerECUSoftwareVersionNum.size(); i++) {
        LOG(INFO) << "[SupplierManufacturerECUSoftwareVersionNum] data[" << i << "] = " << static_cast<int>(SupplierManufacturerECUSoftwareVersionNum[i]);
    }

    notifydLTVectorParamUpdated(InternalMarellibuildVersion);
    for (unsigned i = 0; i < InternalMarellibuildVersion.size(); i++) {
        LOG(INFO) << "[InternalMarellibuildVersion] data[" << i << "] = " << static_cast<int>(InternalMarellibuildVersion[i]);
    }

    notifydLTVectorParamUpdated(Manufacturer_Name);
    for (unsigned i = 0; i < Manufacturer_Name.size(); i++) {
        LOG(INFO) << "[Manufacturer_Name] data[" << i << "] = " << static_cast<int>(Manufacturer_Name[i]);
    }

    notifydLTVectorParamUpdated(BootLoaderVersionNumber);
    for (unsigned i = 0; i < BootLoaderVersionNumber.size(); i++) {
        LOG(INFO) << "[BootLoaderVersionNumber] data[" << i << "] = " << static_cast<int>(BootLoaderVersionNumber[i]);
    }

    notifydLTVectorParamUpdated(Generic_ID1);
    for (unsigned i = 0; i < Generic_ID1.size(); i++) {
        LOG(INFO) << "[Generic_ID1] data[" << i << "] = " << static_cast<int>(Generic_ID1[i]);
    }

    notifydLTVectorParamUpdated(Generic_ID2);
    for (unsigned i = 0; i < Generic_ID2.size(); i++) {
        LOG(INFO) << "[Generic_ID2] data[" << i << "] = " << static_cast<int>(Generic_ID2[i]);
    }

    notifydLTVectorParamUpdated(Generic_ID3);
    for (unsigned i = 0; i < Generic_ID3.size(); i++) {
        LOG(INFO) << "[Generic_ID3] data[" << i << "] = " << static_cast<int>(Generic_ID3[i]);
    }
  
    }
	else{
		LOG(INFO) << "log_extraction less than one" << log_extraction;
	}
}
void setIVI_DLTExtractionTriggerStatus(uint16_t& IVIDLTExtractionTrigger){
	CommonAPI::CallStatus callStatus;
	if (m_vehicle_proxy != nullptr) {
	m_vehicle_proxy->SendEvent_DLTDataExtractionTrigger_SOC(IVIDLTExtractionTrigger, callStatus, nullptr );
	LOG(INFO) << "CAPIVehicleClientGateway : setIVIDLTExtracTrigger" << IVIDLTExtractionTrigger;
	}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : setIVIDLTDataExtractionTrigger *** Proxy not available";}
}
void setIVI_Usb_Reset_TriggerStatus(uint16_t& IVIUsbResetTrigger){
	CommonAPI::CallStatus callStatus;
	if (m_vehicle_proxy != nullptr) {
	m_vehicle_proxy->SendEvent_USB_trigger_Request(IVIUsbResetTrigger, callStatus, nullptr );
	LOG(INFO) << "CAPIVehicleClientGateway : IVIUsbResetTrigger" << IVIUsbResetTrigger;
	}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : setIVI_Usb_Reset_TriggerStatus *** Proxy not available";}
}
void setSOC_Ready_Status(){
	CommonAPI::CallStatus callStatus;
    uint8_t SOC_Ready_Status = 1 ;
	if (m_vehicle_proxy != nullptr) {
	m_vehicle_proxy->SendEvent_SOC_Ready(SOC_Ready_Status, callStatus, nullptr );
	LOG(INFO) << "CAPIVehicleClientGateway : SendEvent_SOC_Ready" << SOC_Ready_Status;
	}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : m_vehicle_proxy *** Proxy not available";}
}


void setIVIHMI_iviData(std::vector<uint16_t>& IVIHMI_iviData){
	CommonAPI::CallStatus callStatus;
	ivi_param.setSCREEN_TOUCH(IVIHMI_iviData[0]);

	ivi_param.setX_CALL_ACTIVE(IVIHMI_iviData[1]);

	ivi_param.setAUDIO_AUTO_STATUS(IVIHMI_iviData[2]);

	ivi_param.setPARK_ASSIST_APP_STATUS(IVIHMI_iviData[3]);

	ivi_param.setPARTIAL_OP_SCREEN_MODE(IVIHMI_iviData[4]);

	ivi_param.setIDLE_SCREEN_MODE(IVIHMI_iviData[5]);

	ivi_param.setSTANDBY_SCREEN_MODE(IVIHMI_iviData[6]);

    ivi_param.setANTI_THEFT_STATUS(IVIHMI_iviData[7]);
	
//	ivi_param.setFORCE_RESET(IVIHMI_iviData[8]);
    m_vehicle_proxy->SendEvent_SystemReset(IVIHMI_iviData[8],callStatus);
 
    for(auto i:IVIHMI_iviData)
     LOG(INFO) << "CAPIVehicleClientGateway :setIVIHMI_TouchbuttonData " << IVIHMI_iviData[i];
	if (m_vehicle_proxy != nullptr) {
    m_vehicle_proxy->update_IVI_Parameters(ivi_param, callStatus, nullptr);}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : setIVIHMI_iviData *** Proxy not available";}
}
 void setIVIPhoneCallStatus(uint16_t& IVIPhoneCallStatus){
	CommonAPI::CallStatus callStatus;
	LOG(INFO) << "CAPIVehicleClientGateway :IVIPhoneCallStatus " << (uint32_t)IVIPhoneCallStatus;
  switch(IVIPhoneCallStatus){

 	case 0:
	ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_NEW);
	break;
	case 1:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_CONNECTING);
	break;
	case 2:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_SELECT_PHONE_ACCOUNT);
	break;
	case 3:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_DIALING);
	break;
	case 4:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_ACTIVE);
	break;
	case 5:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_RINGING);
	break;
	case 6:
	ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_ON_HOLD);
	break;
 	case 7:
	ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_DISCONNECTED);
	break;
	case 8:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_ABORTED);
	break;
	case 9:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_DISCONNECTING);
	break;
	case 10:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_PULLING);
	break;
	case 11:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_ANSWERED);
	break;
	case 12:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_AUDIO_PROCESSING);
	break;
	case 13:
    ivi_param.setPHONECALL_STATUS(::iMicro::pmStructs::phonecallstatus::PHONE_STATE_SIMULATED_RINGING);
	break;
     default:
	 break;
 }
	LOG(INFO) << "CAPIVehicleClientGateway :IVIPhoneCallStatus " << IVIPhoneCallStatus;
	if (m_vehicle_proxy != nullptr) {
    m_vehicle_proxy->update_IVI_Parameters(ivi_param, callStatus, nullptr);}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : setIVIPhoneCallStatus *** Proxy not available";}
}

  void setIVIHMISoftKeyStatus(std::vector<uint16_t>& softkeystatus){
	CommonAPI::CallStatus callStatus;
	ivi_param.setAUTO_SWITCH_ON(softkeystatus[0]);

    ivi_param.setRADIO_OFF_DELAY(softkeystatus[1]);

    ivi_param.setSWITCH_OFF_WITH_DOOR(softkeystatus[2]);

    ivi_param.setDOOR_CONDITION_2(softkeystatus[3]);

    ivi_param.setRADIO_OFF_WITH_DOOR(softkeystatus[4]);

    ivi_param.setPOWER_SOFT_BUTTON_PRESSED(softkeystatus[5]);

     for(auto i:softkeystatus)
    LOG(INFO) << "CAPIVehicleClientGateway :IVIHMISoftKeyStatus " << softkeystatus[i];
	if (m_vehicle_proxy != nullptr) {
    m_vehicle_proxy->update_IVI_Parameters(ivi_param, callStatus, nullptr);}
	else{ 
	LOG(INFO) << "CAPIVehicleClientGateway : softkeystatus *** Proxy not available";}

}
 

 void setIVISmsStatus(uint16_t& IVISmsStatus){
	CommonAPI::CallStatus callStatus;
  switch(IVISmsStatus){
 	case 0:
	ivi_param.setSMS_STATUS(iMicro::pmStructs::smsStatus::SMS_RECEIVED);
	case 1:
    ivi_param.setSMS_STATUS(iMicro::pmStructs::smsStatus::MMS_RECEIVED);
	case 2:
    ivi_param.setSMS_STATUS(iMicro::pmStructs::smsStatus::WAP_PUSH_RECEIVED);
     default:
	 break;
 }
	LOG(INFO) << "CAPIVehicleClientGateway :IVISmsStatus " << IVISmsStatus;
	if (m_vehicle_proxy != nullptr) {
    m_vehicle_proxy->update_IVI_Parameters(ivi_param, callStatus, nullptr);}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : setIVISMSStatus *** Proxy not available";}
}
void setIVI_S2RStatus(uint8_t& str_status){
	CommonAPI::CallStatus callStatus;
	if (m_vehicle_proxy != nullptr) {
	m_vehicle_proxy->SendEvent_Suspend_Ready( str_status, callStatus, nullptr );
	LOG(INFO) << "CAPIVehicleClientGateway : setIVIS2R_status" << str_status;
	}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : etIVIS2R_statusTrigger *** Proxy not available";}
}
void setIVI_USB_Update(std::vector<uint16_t>& p1)
{
    for(unsigned i = 0 ; i < p1.size() ; i++)
    {
        ALOGD("setIVI_USB_Update %d ",p1[i]);
    }
    CommonAPI::CallStatus callStatus;
  ::iMicro::iMicroStructs::USBUPDATE_VENDOR_type send_ivi_usb_update;
    send_ivi_usb_update.setVENDOR_USERCONSENT_NEWUPDATE(p1[0]);
    send_ivi_usb_update.setVENDOR_CONTINUE_UPDATE(p1[1]);
    if (m_vehicle_proxy != nullptr) {
        ALOGD("Sending USB Update data to IMC");
        m_vehicle_proxy->GetEvent_USBUPDATE_VENDOR_SOC(send_ivi_usb_update, callStatus, nullptr);
        ALOGD(" USB Update data sent to IMC");
	}
    else {
	LOG(INFO) << "CAPIVehicleClientGateway : GetEvent_USBUPDATE_VENDOR_SOC *** Proxy not available";
    }
}
void setDEM_DTCStatus(std::vector<uint8_t>& dtc_value){
	LOG(INFO) << "CAPIVehicleClientGateway : inside setDEM_DTCStatus function ";
	 bool test_result = dtc_value[2];
	 ::iMicro::iMicroStructs::dem_eventstatus_type dtcData;
	 ::iMicro::iMicroTypes::dem_eventstatus_enum testStatus;
	CommonAPI::CallStatus call_status;
	uint32_t dtc_code = 0;
	if (dtc_value.size() == sizeof(dtc_code)){
	dtc_code |= static_cast<uint32_t>(dtc_value[0]) << 24;
    dtc_code |= static_cast<uint32_t>(dtc_value[1]) << 16;
    dtc_code |= static_cast<uint32_t>(dtc_value[2]) << 8;
    dtc_code |= static_cast<uint32_t>(dtc_value[3]);
	}
	else{
		LOG(INFO) << "Vector size must be exactly 4 bytes to convert to uint32_t." << std::endl;
	}
	LOG(INFO) << "CAPIVehicleClientGateway : dtc_code-> "<<(uint32_t)dtc_code;
	if(test_result)
    {
        testStatus = iMicro::iMicroTypes::dem_eventstatus_enum::Literal::IMICRO_DEM_EVENT_STATUS_FAILED;
    }
    else
    {
        testStatus = iMicro::iMicroTypes::dem_eventstatus_enum::Literal::IMICRO_DEM_EVENT_STATUS_PASSED;
    }
	dtcData.setP_DtcCode(dtc_code);
	dtcData.setP_EventStatus(testStatus);
	LOG(INFO) << "CAPIVehicleClientGateway : testStatus-> "<<(uint32_t)testStatus;
	if (m_DEMProxy != nullptr) {
	m_DEMProxy->SendEvent_DEM_SetEventStatusIMicro(dtcData, call_status, nullptr);
	LOG(INFO) << "CAPIVehicleClientGateway : SendEvent_DEM_SetEvent_SOC from VHAL ";}
	else{
		LOG(INFO) << "CAPIVehicleClientGateway : SendEvent_DEM_SetEventStatus_SOC *** Proxy not available";
	}
}
void setIVI_DIDdata_request(std::vector<uint16_t>& p1){
    
		CommonAPI::CallStatus callStatus;
  ::iMicro::iMicroStructs::diag_message_type _DIAG_SendtoVuc;
    _DIAG_SendtoVuc.setP_MsgID(p1[0]);
    _DIAG_SendtoVuc.setP_AppID(p1[1]);
    _DIAG_SendtoVuc.setP_Handle(p1[2]);
    _DIAG_SendtoVuc.setP_length(p1[3]); 
	
    std::vector<uint8_t> vectorData;
    for(unsigned i = 4 ; i < p1.size(); i++)
    {

       vectorData.push_back(p1[i]); 
    }
    ALOGD("p1.size():%d",p1.size());
    _DIAG_SendtoVuc.setP_DiagMessage(vectorData);
    m_DIAGProxy->SendEvent_DIAG_SendtoVuc( _DIAG_SendtoVuc, callStatus, nullptr );
	for(auto N:vectorData)
    LOG(INFO) <<"diagMsg sent to MCU.\n"<<uint32_t(N);
}

void setIVI_CallActiveSTR_HMIPopup_Status(uint8_t& userInputFromHMIPopup){
    LOG(INFO) << "setIVI_CallActiveSTR_HMIPopup_Status()";
	CommonAPI::CallStatus callStatus;
	if(m_vehicle_proxy != nullptr) {
        LOG(INFO) << "setIVI_CallActiveSTR_HMIPopup_Status() userInputFromHMIPopup = " << userInputFromHMIPopup;
	    m_vehicle_proxy->SendEvent_HmiPopUpForCallUserInput(userInputFromHMIPopup, callStatus, nullptr);
	}
	else {
	    LOG(INFO) << "setIVI_CallActiveSTR_HMIPopup_Status() Proxy not available";
    }
}

void SetInternal_Fan_Speed(uint32_t& Fan_speed){
	CommonAPI::CallStatus callStatus;
	if (m_FIRSTProxy != nullptr) {
	m_FIRSTProxy->SendEvent_Internal_Fan_Speed(Fan_speed, callStatus, nullptr );  
	LOG(INFO) << "CAPIVehicleClientGateway : Internal_Fan_speed" << Fan_speed;
	}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : m_FIRSTProxyStatus *** Proxy not available";}
    }
void setIVIResettrigger(uint32_t& IVI_Reset){
	CommonAPI::CallStatus callStatus = CommonAPI::CallStatus::UNKNOWN;
    iMicro::iMicroTypes::Factory_Reset_t reset(
        static_cast<iMicro::iMicroTypes::Factory_Reset_t::Literal>(IVI_Reset)
    );
	if (m_vehicle_proxy != nullptr) {
	m_vehicle_proxy->SendEvent_Factory_Reset(reset, callStatus, nullptr );  
	LOG(INFO) << "CAPIVehicleClientGateway : IVI_Reset" << IVI_Reset;
	}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : m_vehicle_proxy *** Proxy not available";}
    if (callStatus == CommonAPI::CallStatus::SUCCESS) {
        LOG(INFO) << "CAPIVehicleClientGateway: IVI_Reset triggered successfully for reset type: " << reset.toString();
    } else {
        LOG(INFO) << "CAPIVehicleClientGateway: IVI_Reset call failed. CallStatus: " << static_cast<int>(callStatus);
    }
    }
    void SetSystemTimeZone(int32_t& TimeZone){
	CommonAPI::CallStatus callStatus = CommonAPI::CallStatus::UNKNOWN;
	if (m_TCU_proxy != nullptr) {
	m_TCU_proxy->system_timezone_offset(TimeZone,callStatus,nullptr);  
	LOG(INFO) << "CAPIVehicleClientGateway : SetSystemTimeZone" << TimeZone;
	}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : m_TCU_proxy *** Proxy not available";}
    if (callStatus == CommonAPI::CallStatus::SUCCESS) {
        LOG(INFO) << "CAPIVehicleClientGateway: SetSystemTimeZone is success:";
    } else {
        LOG(INFO) << "CAPIVehicleClientGateway: SetSystemTimeZone is unsuccessful:";
    }
    }
        
void SendFilepathtoTCU(std::string& p1){
	CommonAPI::CallStatus callStatus = CommonAPI::CallStatus::UNKNOWN;
    ::VHAL::LOGR::LOGR_Types::LOGR_status status;
	if (m_TCU_proxy != nullptr) {
	m_TCU_proxy->free(p1, callStatus, status);  
	LOG(INFO) << "CAPIVehicleClientGateway : Filepath_free" << p1.c_str();
	}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : m_FIRSTProxyStatus *** Proxy not available";
    }
    if (callStatus == CommonAPI::CallStatus::SUCCESS) {
    LOG(INFO) << " free call succeeded. Status: " << status.toString();
    LOG(INFO) << "Status_free" << status;
    notifyLog_retriver_ack(status);
    } else {
    std::cerr << "Retrieve call failed. CallStatus: " << static_cast<int>(callStatus);
    }
    }

void getlogretrieverstatus(uint32_t& lagretriever_status){
	CommonAPI::CallStatus callStatus = CommonAPI::CallStatus::UNKNOWN;
    ::VHAL::LOGR::LOGR_Types::LOGR_status status;
	if (m_TCU_proxy != nullptr) {
	m_TCU_proxy->status(callStatus, status);  
	LOG(INFO) << "CAPIVehicleClientGateway : getlogretrieverstatus";
	}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : m_FIRSTProxyStatus *** Proxy not available";
    }
    if (callStatus == CommonAPI::CallStatus::SUCCESS) {
    LOG(INFO) << " getlogretrieverstatus call succeeded. Status: " << status.toString();
    LOG(INFO) << "getlogretrieverstatus::status" << status;
    notifylogretriever_status(status);
    } else {
    std::cerr << "Retrieve call failed. CallStatus: " << static_cast<int>(callStatus);
    }
    }
        

    void PrintTimeUTC(const VHAL::LOGR::LOGR_Types::t_time_UTC& timeUTC) {
        LOG(INFO) << "Year: " << timeUTC.getYear()
                  << ", Month: " << timeUTC.getMonth()
                  << ", Day: " << timeUTC.getDay()
                  << ", Hours: " << timeUTC.getHours()
                  << ", Minutes: " << timeUTC.getMinutes()
                  << ", Seconds: " << timeUTC.getSeconds()
                  << std::endl;
    }

    VHAL::LOGR::LOGR_Types::t_time_UTC convertEpochToTimeUTC(uint32_t epochTime) {
        std::time_t epochTimeT = static_cast<std::time_t>(epochTime);
        std::tm *timeInfo = std::gmtime(&epochTimeT);
    
        if (!timeInfo) {
            std::cerr << "Failed to convert the epoch time!" << std::endl;
            return VHAL::LOGR::LOGR_Types::t_time_UTC();
        }
        
        uint16_t year = static_cast<uint16_t>(timeInfo->tm_year + 1900);
        uint16_t month = static_cast<uint16_t>(timeInfo->tm_mon + 1);
        uint16_t day = static_cast<uint16_t>(timeInfo->tm_mday);
        uint16_t hours = static_cast<uint16_t>(timeInfo->tm_hour);
        uint16_t minutes = static_cast<uint16_t>(timeInfo->tm_min);
        uint16_t seconds = static_cast<uint16_t>(timeInfo->tm_sec);
    
        return VHAL::LOGR::LOGR_Types::t_time_UTC(year, month, day, hours, minutes, seconds);
    }

    void SetLoggerdata(std::vector<uint32_t>& Logger_Data) {
        ALOGD("Inside SetLoggerdata");
        CommonAPI::CallStatus callStatus = CommonAPI::CallStatus::UNKNOWN;
        VHAL::LOGR::LOGR_Types::LOGR_status status;
        if (Logger_Data.size() < 4) {
            std::cerr << "Logger_Data vector is too small!" << std::endl;
            return;
        }
    
        VHAL::LOGR::LOGR_Types::LOGR_verbosity verbosity(
            static_cast<VHAL::LOGR::LOGR_Types::LOGR_verbosity::Literal>(Logger_Data[0])
        );
        std::vector<VHAL::LOGR::LOGR_Types::LOGR_source> sources = {
            VHAL::LOGR::LOGR_Types::LOGR_source(
                static_cast<VHAL::LOGR::LOGR_Types::LOGR_source::Literal>(Logger_Data[1])
            )
        };
        VHAL::LOGR::LOGR_Types::t_time_UTC startTime = convertEpochToTimeUTC(Logger_Data[2]);
        VHAL::LOGR::LOGR_Types::t_time_UTC endTime = convertEpochToTimeUTC(Logger_Data[3]);
        LOG(INFO) << "Start Time: ";
        PrintTimeUTC(startTime);
        LOG(INFO) << "End Time: ";
        PrintTimeUTC(endTime);


        if(m_TCU_proxy != nullptr) {
            m_TCU_proxy->retrieve(verbosity, sources, startTime, endTime, callStatus, status);
            LOG(INFO) << "CAPIVehicleClientGateway : Send_retrieve from VHAL ";
        }
        else {
            LOG(INFO) << "CAPIVehicleClientGateway :  m_TCU_proxy not available";
        }
        if (callStatus == CommonAPI::CallStatus::SUCCESS) {
            LOG(INFO) << "Retrieve call succeeded. Status: " << status.toString();
            LOG(INFO) << "Status" << status;
            notifyLog_retriver_ack(status);
        } else {
            std::cerr << "Retrieve call failed. CallStatus: " << static_cast<int>(callStatus);
        }
        
    }
      
void setIVI_SoCVersion_Send_MCU(){
	char SoC_Version_array[SOC_VERSION_SIZE];
    CommonAPI::CallStatus callStatus;
	
    if (__system_property_get("ro.system.build.version.incremental", SoC_Version_array)) {
        LOG(INFO) <<"CAPIVehicleClientGateway :SoC_Version successfully Property found: ";
    } else {
       LOG(INFO) << "CAPIVehicleClientGateway :ro.system.build.version.incremental ->Property not found!\n";
    }
	std::string SoC_Version_string = std::string(SoC_Version_array);
	size_t spacePos = SoC_Version_string.find(' ');
    if (spacePos != std::string::npos) {
        SoC_Version_string = SoC_Version_string.substr(0, spacePos);
    }
	std::vector<uint8_t> SoC_Version(SoC_Version_string.begin(), SoC_Version_string.end());
	m_vehicle_proxy->SendEvent_IVI_SW_Release_version_NO(SoC_Version, callStatus, nullptr );
	LOG(INFO) <<"CAPIVehicleClientGateway :SoC_Version     "<<SoC_Version_string;
}
std::string getValueFromProcCmdline(const std::string& key) {
    std::ifstream file("/proc/cmdline");
    if (!file) {
        LOG(INFO) << "Error: Unable to open /proc/cmdline file" << std::endl;
        return "";
    }
 
    std::string line;
    if (std::getline(file, line)) {
        std::istringstream stream(line);
        std::string token;
        while (stream >> token) {
            size_t pos = token.find('=');
            if (pos != std::string::npos) {
                std::string currentKey = token.substr(0, pos);
                if (currentKey == key) {
                    return token.substr(pos + 1);
                }
            }
        }
    }
 
    return "";
}
void setIVI_hw_variant_id_Send_MCU() 
{
    CommonAPI::CallStatus callStatus = CommonAPI::CallStatus::UNKNOWN;
    ::iMicro::iMicroStructs::IM_Generic_message_t _IM_Generic_message_SOC;
    std::vector<uint8_t> vectorData;
    std::string hw_variant_id = getValueFromProcCmdline("hw_variant_id");
    if (!hw_variant_id.empty()) {
    LOG(INFO) << "hw_variant_id:" << hw_variant_id;
    for (char c : hw_variant_id) {
        LOG(INFO) << "Character: " << c << " ASCII: " << static_cast<int>(c) << std::endl;
        vectorData.push_back(static_cast<int>(c));
    }
    _IM_Generic_message_SOC.setP_ContainerID(ContainerID);
    _IM_Generic_message_SOC.setP_MsgID(MsgID);
    _IM_Generic_message_SOC.setP_AppID(AppID);
    _IM_Generic_message_SOC.setP_Length(vectorData.size());
    _IM_Generic_message_SOC.setP_GenericMessage(vectorData);
    if (m_vehicle_proxy != nullptr) {
    m_vehicle_proxy->SendEvent_IM_Generic_message_SOC(_IM_Generic_message_SOC, callStatus, nullptr );  
    LOG(INFO) << "CAPIVehicleClientGateway : Sent IM_Generic_message_SOC";
    }
    else {
    LOG(INFO) << "CAPIVehicleClientGateway : m_vehicle_proxy *** Proxy not available";}
	for(auto N:vectorData)
    LOG(INFO) <<"hw_variant_id sent to MCU.\n"<<uint32_t(N);
    } else {
        LOG(INFO) << "Key not found or value is empty";
    }
    if (callStatus == CommonAPI::CallStatus::SUCCESS) {
        LOG(INFO) << " callStatus_SendEvent_IM_Generic_message_SOC: ";
        } else {
        LOG(INFO) << "Retrieve call failed. CallStatus: " << static_cast<int>(callStatus);
        }
    
   
}
void setDCSD_DIAG_Res(std::vector<uint8_t>& DCSD_DIAG_Res) {
    for (unsigned i = 0; i < DCSD_DIAG_Res.size(); i++) {
        LOG(INFO) << "Initial DCSD_DIAG_Res::" << (uint32_t)DCSD_DIAG_Res[i];
    }

    uint8_t length = DCSD_DIAG_Res.size();
    LOG(INFO) << "DCSD_DIAG_Res_size()::" << (uint32_t)DCSD_DIAG_Res.size();
    uint8_t dcsd_dig_res_length = length + 6;

    CommonAPI::CallStatus callStatus;
    std::vector<uint8_t> dcsd_dig_res(dcsd_dig_res_length, 0);  
    LOG(INFO) << "dcsd_dig_res_size()::" << (uint32_t)dcsd_dig_res.size();

    uint16_t Signal_index = iviCanDatasend::NTW_TX_DCSD_DIAG_RES_Com_DCSD_Signal_Tx;
    uint32_t _result;
    unsigned k = 0U;
    dcsd_dig_res[k++] = (Signal_index >> 8) & 0xFF;
    dcsd_dig_res[k++] = (Signal_index >> 0) & 0xFF;
    dcsd_dig_res[k++] = 0U;
    dcsd_dig_res[k++] = 0U;
    dcsd_dig_res[k++] = (length >> 0) & 0xFF;
    dcsd_dig_res[k++] = (length >> 8) & 0xFF;

    for (unsigned i = 0; i < DCSD_DIAG_Res.size(); i++) {
        dcsd_dig_res[k++] = DCSD_DIAG_Res[i];
    }
    LOG(INFO) << "final dcsd_dig_res_size()::" << (uint32_t)dcsd_dig_res.size();

    CommonAPI::ByteBuffer bufferCAN(dcsd_dig_res.size());
    if (bufferCAN.size() < dcsd_dig_res.size() || dcsd_dig_res.size() < dcsd_dig_res_length) {
        LOG(ERROR) << "Buffer size mismatch. Copy operation aborted.";
        return;
    }

    std::memcpy(bufferCAN.data(), dcsd_dig_res.data(), dcsd_dig_res.size());
    for (unsigned i = 0; i < bufferCAN.size(); i++) {
        LOG(INFO) << "BufferCAN[" << i << "]: " << (uint32_t)bufferCAN[i];
    }
    
    if (m_vehicle_proxy_CANChanel != nullptr) {
        m_vehicle_proxy_CANChanel->SendRaw(bufferCAN, callStatus, _result, nullptr);
    } else {
        LOG(INFO) << "CAPIVehicleClientGateway :DCSD_DIAG_Res proxy not available";
    }

    for (unsigned k = 0; k < dcsd_dig_res.size(); k++) {
        LOG(INFO) << "CAPIVehicleClientGateway :final DCSD_DIAG_Res " << (uint32_t)dcsd_dig_res[k];  
    }
}
void send_arvc_status_mcu(uint8_t& arvc_status){
    CommonAPI::CallStatus callStatus;
	if (m_vehicle_proxy != nullptr) {
	m_vehicle_proxy->SendEvent_RVC(arvc_status, callStatus, nullptr);  
	LOG(INFO) << "CAPIVehicleClientGateway : arvc_status" << (uint32_t)arvc_status;
	}
	else {
	LOG(INFO) << "CAPIVehicleClientGateway : arvc_status *** Proxy not available";}
}
	void notifySpeedUpdated(uint32_t speed) 
	{
		if (speedCallback_) {
			speedCallback_(speed);
		}
	}
	void notifyIgnitionSts(uint32_t IgnitionSts) 
	{
		if (IgnitionStsCallback_) {
			IgnitionStsCallback_(IgnitionSts);
		}
	}
	
	void notifyEcallCrash(uint32_t EcallCrash) 
	{
		if (EcallCrashCallback_) {
			EcallCrashCallback_(EcallCrash);
		}
	}
	void notifyParkLamp(uint32_t ParkLamp) 
	{
		if (ParkLampCallback) {
			ParkLampCallback(ParkLamp);
		}
	}
	void notifyDayNightMode(uint32_t DayNightMode) 
	{
		if (DayNightModeCallback) {
			DayNightModeCallback(DayNightMode);
		}
	}
	void notifyFuellevel(uint32_t Fuellevel) 
	{
		if (FuellevelCallback) {
			FuellevelCallback(Fuellevel);
		}
	}
	void notifyFuelType(uint32_t FuelType) 
	{
		if (FuelTypeCallback) {
			FuelTypeCallback(FuelType);
		}
	}
	void notifyparkbreakstatus(uint32_t parkbreakstatus) 
	{
		if (ParkBreakStatusCallback) {
			ParkBreakStatusCallback(parkbreakstatus);
		}
	}
	void notifyOutTemp(uint32_t OutTemp) 
	{
		if (OutTempCallback) {
			OutTempCallback(OutTemp);
		}
	}
	void notifyHVBatterySOHData(uint32_t HVBatterySOHData) 
	{
		if (HVBatterySOHDataCallback) {
			HVBatterySOHDataCallback(HVBatterySOHData);
		}
	}
	void notifyHVBatteryPercentage(uint32_t HVBatteryPercentage) 
	{
		if (HVBatteryPercentageCallback) {
			HVBatteryPercentageCallback(HVBatteryPercentage);
		}
	}
	void notifyHVBatteryMinCharge(uint32_t HVBatteryMinCharge) 
	{
		if (HVBatteryMinChargeCallback) {
			HVBatteryMinChargeCallback(HVBatteryMinCharge);
		}
	}
	void notifyHVBatteryActiveConnector(uint32_t HVBatteryActiveConnector) 
	{
		if (HVBatteryActiveConnectorCallback) {
			HVBatteryActiveConnectorCallback(HVBatteryActiveConnector);
		}
	}
	void notifyHVBatteryMaxRange(uint32_t HVBatteryMaxRange) 
	{
		if (HVBatteryMaxRangeCallback) {
			HVBatteryMaxRangeCallback(HVBatteryMaxRange);
		}
	}
	void notifyHVBatteryChargeLevel(uint32_t HVBatteryChargeLevel) 
	{
		if (HVBatteryChargeLevelCallback) {
			HVBatteryChargeLevelCallback(HVBatteryChargeLevel);
		}
	}
	void notifyHVBatteryIsCharging(uint32_t HVBatteryIsCharging) 
	{
		if (HVBatteryIsChargingCallback) {
			HVBatteryIsChargingCallback(HVBatteryIsCharging);
		}
	}
	void notifyAutonomy(uint32_t Autonomy) 
	{
		if (AutonomyCallback) {
			AutonomyCallback(Autonomy);
		}
	}
	void notifyBrakePedalrSts(uint32_t BrakePedalrSts) 
	{
		if (BrakePedalrStsCallback) {
			BrakePedalrStsCallback(BrakePedalrSts);
		}
	}
	void notifyCanDistanceUnit(uint32_t CanDistanceUnit) 
	{
		if (CanDistanceUnitCallback) {
			CanDistanceUnitCallback(CanDistanceUnit);
		}
	}
	void notifyInstantFuelCons(uint32_t InstantFuelCons) 
	{
		if (InstantFuelConsCallback) {
			InstantFuelConsCallback(InstantFuelCons);
		}
	}
	void notifyReverseGearEngaged(uint32_t ReverseGearEngaged) 
	{
		if (ReverseGearEngagedCallback) {
			ReverseGearEngagedCallback(ReverseGearEngaged);
		}
	}
	void notifyRoadSlope(uint32_t RoadSlope) 
	{
		if (RoadSlopeCallback) {
			RoadSlopeCallback(RoadSlope);
		}
	}
	void notifySteeringAngle(uint32_t SteeringAngle) 
	{
		if (SteeringAngleCallback) {
			SteeringAngleCallback(SteeringAngle);
		}
	}
	void notifyTotalKM(uint32_t TotalKM) 
	{
		if (TotalKMCallback) {
			TotalKMCallback(TotalKM);
		}
	}
	void notifyTransmGearDisplay(uint32_t TransmGearDisplay) 
	{
		if (TransmGearDisplayCallback) {
			TransmGearDisplayCallback(TransmGearDisplay);
		}
	}
	void notifyTripAvgBattPowerCons(uint32_t TripAvgBattPowerCons) 
	{
		if (TripAvgBattPowerConsCallback) {
			TripAvgBattPowerConsCallback(TripAvgBattPowerCons);
		}
	}
	void notifyYawSpeed(uint32_t YawSpeed) 
	{
		if (YawSpeedCallback) {
			YawSpeedCallback(YawSpeed);
		}
	}
	void notifyGrossYawRate(uint32_t GrossYawRate) 
	{
		if (GrossYawRateCallback) {
			GrossYawRateCallback(GrossYawRate);
		}
	}
	void notifyFLWheelTickCounterFault(uint32_t FLWheelTickCounterFault) 
	{
		if (FLWheelTickCounterFaultCallback) {
			FLWheelTickCounterFaultCallback(FLWheelTickCounterFault);
		}
	}
	void notifyFRWheelTickCounterFault(uint32_t FRWheelTickCounterFault) 
	{
		if (FRWheelTickCounterFaultCallback) {
			FRWheelTickCounterFaultCallback(FRWheelTickCounterFault);
		}
	}
	void notifyRLWheelTickCounterFault(uint32_t RLWheelTickCounterFault) 
	{
		if (RLWheelTickCounterFaultCallback) {
			RLWheelTickCounterFaultCallback(RLWheelTickCounterFault);
		}
	}
	void notifyRRWheelTickCounterFault(uint32_t RRWheelTickCounterFault) 
	{
		if (RRWheelTickCounterFaultCallback) {
			RRWheelTickCounterFaultCallback(RRWheelTickCounterFault);
		}
	}
	void notifyPulseCountFLWheel(uint32_t PulseCountFLWheel) 
	{
		if (PulseCountFLWheelCallback) {
			PulseCountFLWheelCallback(PulseCountFLWheel);
		}
	}
    void notifyPulseCountFRWheel(uint32_t PulseCountFRWheel) 
	{
		if (PulseCountFRWheelCallback) {
			PulseCountFRWheelCallback(PulseCountFRWheel);
		}
	}
	void notifyPulseCountRLWheel(uint32_t PulseCountRLWheel) 
	{
		if (PulseCountRLWheelCallback) {
			PulseCountRLWheelCallback(PulseCountRLWheel);
		}
	}
	void notifyPulseCountRRWheel(uint32_t PulseCountRRWheel) 
	{
		if (PulseCountRRWheelCallback) {
			PulseCountRRWheelCallback(PulseCountRRWheel);
		}
	}
	void notifyFPASSts(uint32_t FPASSts) 
	{
		if (FPASStsCallback) {
			FPASStsCallback(FPASSts);
		}
	}
	void notifyDrivingDirection(uint32_t DrivingDirection) 
	{
		if (DrivingDirectionCallback) {
			DrivingDirectionCallback(DrivingDirection);
		}
	}
	void notifyCapacityUnit(uint32_t CapacityUnit) 
	{
		if (CapacityUnitCallback) {
			CapacityUnitCallback(CapacityUnit);
		}
	}
	void notifyCombustFuelLv(uint32_t CombustFuelLv) 
	{
		if (CombustFuelLvCallback) {
			CombustFuelLvCallback(CombustFuelLv);
		}
	}
	void notifyCompressorSts(uint32_t CompressorSts) 
	{
		if (CompressorStsCallback) {
			CompressorStsCallback(CompressorSts);
		}
	}
	void notifyElectricMotorPower(int32_t ElectricMotorPower) 
	{
		if (ElectricMotorPowerCallback) {
			ElectricMotorPowerCallback(ElectricMotorPower);
		}
	}
	void notifyEngineCoolantTemp(uint32_t EngineCoolantTemp) 
	{
		if (EngineCoolantTempCallback) {
			EngineCoolantTempCallback(EngineCoolantTemp);
		}
	}
	void notifyEngineRPM(uint32_t EngineRPM) 
	{
		if (EngineRPMCallback) {
			EngineRPMCallback(EngineRPM);
		}
	}
	void notifyFuelLevelMinimumSts(uint32_t FuelLevelMinimumSts) 
	{
		if (FuelLevelMinimumStsCallback) {
			FuelLevelMinimumStsCallback(FuelLevelMinimumSts);
		}
	}
	void notifyAbsSteeringWheelAngleAcu(uint32_t AbsSteeringWheelAngleAcu) 
	{
		if (AbsSteeringWheelAngleAcuCallback) {
			AbsSteeringWheelAngleAcuCallback(AbsSteeringWheelAngleAcu);
		}
	}
	void notifyInitSteeringWheenAngleFlag(uint32_t InitSteeringWheenAngleFlag) 
	{
		if (InitSteeringWheenAngleFlagCallback) {
			InitSteeringWheenAngleFlagCallback(InitSteeringWheenAngleFlag);
		}
	}
	void notifyNetworkMNGT(uint32_t NetworkMNGT) 
	{
		if (NetworkMNGTCallback) {
			NetworkMNGTCallback(NetworkMNGT);
		}
	}
	void notifyFPASCenterLeftBarSts(uint32_t FPASCenterLeftBarSts) 
	{
		if (FPASCenterLeftBarStsCallback) {
			FPASCenterLeftBarStsCallback(FPASCenterLeftBarSts);
		}
	}
	
	void notifyFPASCenterRightBarSts(uint32_t FPASCenterRightBarSts) 
	{
		if (FPASCenterRightBarStsCallback) {
			FPASCenterRightBarStsCallback(FPASCenterRightBarSts);
		}
	}
	void notifyFPASLeftBarSts(uint32_t FPASLeftBarSts) 
	{
		if (FPASLeftBarStsCallback) {
			FPASLeftBarStsCallback(FPASLeftBarSts);
		}
	}
	void notifyFPASRightBarSts(uint32_t FPASRightBarSts) 
	{
		if (FPASRightBarStsCallback) {
			FPASRightBarStsCallback(FPASRightBarSts);
		}
	}
	void notifyRPASCenterLeftBarSts(uint32_t RPASCenterLeftBarSts) 
	{
		if (RPASCenterLeftBarStsCallback) {
			RPASCenterLeftBarStsCallback(RPASCenterLeftBarSts);
		}
	}
	void notifyRPASLeftBarSts(uint32_t RPASLeftBarSts) 
	{
		if (RPASLeftBarStsCallback) {
			RPASLeftBarStsCallback(RPASLeftBarSts);
		}
	}
	void notifyRPASCenterRightBarSts(uint32_t RPASCenterRightBarSts) 
	{
		if (RPASCenterRightBarStsCallback) {
			RPASCenterRightBarStsCallback(RPASCenterRightBarSts);
		}
	}
	void notifyTrailerPresent(uint32_t TrailerPresent) 
	{
		if (TrailerPresentCallback) {
			TrailerPresentCallback(TrailerPresent);
		}
	}
	void notifyTrunkDoorSts(uint32_t TrunkDoorSts) 
	{
		if (TrunkDoorStsCallback) {
			TrunkDoorStsCallback(TrunkDoorSts);
		}
	}
	void notifyRPASSts(uint32_t RPASSts) 
	{
		if (RPASStsCallback) {
			RPASStsCallback(RPASSts);
		}
	}
	void notifyEngineTorque(uint32_t EngineTorque) 
	{
		if (EngineTorqueCallback) {
			EngineTorqueCallback(EngineTorque);
		}
	}
	void notifyEconShftRq(uint32_t EconShftRq) 
	{
		if (EconShftRqCallback) {
			EconShftRqCallback(EconShftRq);
		}
	}
	void notifyFuelConsUnit(uint32_t FuelConsUnit) 
	{
		if (FuelConsUnitCallback) {
			FuelConsUnitCallback(FuelConsUnit);
		}
	}
	void notifyRegeneratedEnergy(uint32_t RegeneratedEnergy) 
	{
		if (RegeneratedEnergyCallback) {
			RegeneratedEnergyCallback(RegeneratedEnergy);
		}
	}
	void notifyShiftLeverPosition(uint32_t ShiftLeverPosition) 
	{
		if (ShiftLeverPositionCallback) {
			ShiftLeverPositionCallback(ShiftLeverPosition);
		}
	}
	void notifyTripAverageFuelConsumption(uint32_t TripAverageFuelConsumption) 
	{
		if (TripAverageFuelConsumptionCallback) {
			TripAverageFuelConsumptionCallback(TripAverageFuelConsumption);
		}
	}
	void notifyReqVolCtrl(uint32_t ReqVolCtrl) 
	{
		if (ReqVolCtrlCallback) {
			ReqVolCtrlCallback(ReqVolCtrl);
		}
	}
	void notifyVehPrivacyMode(uint32_t VehPrivacyMode) 
	{
		if (VehPrivacyModeCallback) {
			VehPrivacyModeCallback(VehPrivacyMode);
		}
	}
	
	void notifyVinNumberUpdated(std::vector<uint32_t> vinNumber) 
	{  
	    if (vinNumberCallback) {
		vinNumberCallback(vinNumber);
		}
	}
	void notifyBCMHmiTimeUpdated(uint32_t bcmHMItime[]) 
	{
	    if (bcmHMItimeCallback)
		{
		bcmHMItimeCallback(bcmHMItime);
		}
	}
	void notifyBCMSecureTimeUpdated(uint32_t bcmSECURtime[]) 
	{	    
        if (bcmSECURtimeCallback) 
		{
		bcmSECURtimeCallback(bcmSECURtime);
		}
	}
	void notifydotteParamUpdated(std::vector<uint32_t> dotteParam) 
	{	    
        if (dotteParamCallback) 
		{
		dotteParamCallback(dotteParam);
		}
	}
void notifyEVSignalUpdated(int32_t EVSignal[]) 
	{	    
        if (EVSignalsCallback) 
		{
		EVSignalsCallback(EVSignal);
		}
	}
	void notifyCanCommonSignalUpdated(int32_t CanCommonSignal[]) 
	{	    
        if (CanCommonSignalsCallback) 
		{
		CanCommonSignalsCallback(CanCommonSignal);
		}
	}
    void notifyCAN_RX_SignalUpdated(uint32_t CAN_RX_Signal[]) 
	{	    
        if (CAN_RX_SignalCallbacK) 
		{
        CAN_RX_SignalCallbacK(CAN_RX_Signal);
		}
	}
    void notifySigned_CAN_RX_SignalUpdated(int32_t Signed_CAN_RX_Signal[]) 
	{	    
        if (Signed_CAN_RX_SignalCallbacK) 
		{
        Signed_CAN_RX_SignalCallbacK(Signed_CAN_RX_Signal);
		}
	}
	void notifyIMUParamUpdated(std::vector<int32_t> imuParam) 
	{	    
        if (imuParamCallback) 
		{
		imuParamCallback(imuParam);
		}
	}

	void notifyCameraSignalUpdated(uint32_t CameraSignal[]) 
	{	    
        if (CameraSignalsCallback) 
		{
		CameraSignalsCallback(CameraSignal);
		}
	}
	void notifyBCMSignalsUpdated(uint32_t BCMSignal[]) 
	{	    
        if (BCMSignalsCallback) 
		{
		BCMSignalsCallback(BCMSignal);
		}
     } 
	void notifyInternalSignalUpdated(uint32_t internalSignal[]) 
	{	    
        if (internalSignalCallback) 
		{
		internalSignalCallback(internalSignal);
		}
	}
	void notifydtcParamUpdated(std::vector<uint32_t> dtcParam) 
	{	    
        if (dtcParamCallback) 
		{
		dtcParamCallback(dtcParam);
		}
	}
	void notifydLTParamUpdated(std::vector<uint32_t> dLTParam) 
	{	    
        if (dLTParamCallback) 
		{
		dLTParamCallback(dLTParam);
		}
	}
	void notifydLTVectorParamUpdated(std::vector<uint8_t> dLTVectorParam ) 
	{	
        if (dLT_VectorParamCallback) 
		{
		dLT_VectorParamCallback(dLTVectorParam);
		}
	}
	void notifyCalDataUpdated(uint32_t calData[] ) 
	{	
        if (calibrationDataCallback) 
		{
		calibrationDataCallback(calData);
		}
	}
		void notifyCalDataUpdated_Audio(uint32_t calData[] ) 
	{	
        if (calibrationDataCallback_Audio) 
		{
		calibrationDataCallback_Audio(calData);
		}
	}
		void notifyCalDataUpdated_Camera(uint32_t calData[] ) 
	{	
        if (calibrationDataCallback_Camera) 
		{
		calibrationDataCallback_Camera(calData);
		}
	}
		void notifyCalDataUpdated_Array(std::vector<uint8_t> calData ) 
	{	
        if (calibrationDataCallback_Array) 
		{
		calibrationDataCallback_Array(calData);
		}
	}
        void notifyCalDataforSrec_Calibration(std::vector<uint32_t> calSrecDATA ) 
    {    
        if (calibrationDataforSrecCallback) 
        {
        calibrationDataforSrecCallback(calSrecDATA);
        }
    }
		void notifyTemperatureMode(uint32_t Temperature) 
	{
		if (TemperatureCallback) {
			TemperatureCallback(Temperature);
		}
	}
    void notifySupplyVoltage(uint32_t SupplyVoltage) 
	{
		if (SupplyVoltageCallback) {
			SupplyVoltageCallback(SupplyVoltage);
		}
	}
    void notifyMarelliPN(std::vector<uint8_t> MarelliPN) 
	{	
        if (MarelliPNCallback) 
		{
		MarelliPNCallback(MarelliPN);
		}
	}
    void notifyEOLHWID(std::vector<uint8_t> EOLHWID) 
	{	
        if (EOLHWIDCallback) 
		{
		EOLHWIDCallback(EOLHWID);
		}
	}
	    void notifyAuthenticationZone(std::vector<uint8_t> AuthenticationZone) 
	{	
        if (AuthenticationZoneCallback) 
		{
		AuthenticationZoneCallback(AuthenticationZone);
		}
	}
	    void notifyApplicationSoftwareIdentification(std::vector<uint8_t> ApplicationSoftwareIdentification) 
	{	
        if (ApplicationSoftwareIdentificationCallback) 
		{
		ApplicationSoftwareIdentificationCallback(ApplicationSoftwareIdentification);
		}
	}
	    void notifyApplicationDataIdentification(std::vector<uint8_t> ApplicationDataIdentification) 
	{	
        if (ApplicationDataIdentificationCallback) 
		{
		ApplicationDataIdentificationCallback(ApplicationDataIdentification);
		}
	}
	    void notifyDataLibraryidentifier(std::vector<uint8_t> DataLibraryidentifier) 
	{	
        if (DataLibraryidentifierCallback) 
		{
		DataLibraryidentifierCallback(DataLibraryidentifier);
		}
	}
	    void notifyVehicleAppsIdentifier(std::vector<uint8_t> VehicleAppsIdentifier) 
	{	
        if (VehicleAppsIdentifierCallback) 
		{
		VehicleAppsIdentifierCallback(VehicleAppsIdentifier);
		}
	}
    
    void notifyCallActiveSTRDisplayPopupStatus(uint32_t displayPopupInputFromMCU){
        LOG(INFO) << "notifyCallActiveSTRDisplayPopupStatus()";
        if(CallActiveSTRDisplayPopupStatusCallback){
            CallActiveSTRDisplayPopupStatusCallback(displayPopupInputFromMCU);
        }
    }

    void notifySTRCancelShutdownStatus(uint32_t cancelShutdownSTRInputFromMCU){
        LOG(INFO) << "notifySTRCancelShutdownStatus()";
        if(STRCancelShutdownStatusCallback){
            STRCancelShutdownStatusCallback(cancelShutdownSTRInputFromMCU);
        }
    }
	void notifyUSB_Update(std::vector<uint16_t> USB_Update) 
	{
		if (USB_UpdateCallback) {
			USB_UpdateCallback(USB_Update);
		}
	}
	void notifyUSB_Update_Details(std::vector<uint32_t> USB_Update_Details) 
	{
		if (USB_Update_DetailsCallback) {
			USB_Update_DetailsCallback(USB_Update_Details);
		}
	}
		void notifyDID_RID_DataUpdate(std::vector<uint16_t> m_dID_message_type) 
	{
		if (dIDrIDParamCallback) {
			dIDrIDParamCallback(m_dID_message_type);
		}
	}
	void notifyDEM_DTCstatusUpdate(std::vector<uint16_t> m_DEM_DTC_data){
		
		if (DEM_DTCStatusCallback) {
			DEM_DTCStatusCallback(m_DEM_DTC_data);
		}
	}
	
	void notifypowermodedata(uint32_t Powermode) 
	{
		if (PowermodeCallback) {
			PowermodeCallback(Powermode);
		}
	}
	void notifyDriverdoorStatusUpdate(uint8_t driverdoorStatus) 
	{
		if (driverdoorStatusCallback) {
			driverdoorStatusCallback(driverdoorStatus);
		}
	}
	void notifyPassengerDoorStatusUpdate(uint8_t passengerDoorStatus) 
	{
		if (passengerDoorStatusCallback) {
			passengerDoorStatusCallback(passengerDoorStatus);
		}
	}
    void notifyLog_retriver_ack(uint32_t Log_retriver_ack) {
        if (Log_retriver_ackCallback) {
            Log_retriver_ackCallback(Log_retriver_ack);
        }
    }
    void notifyloggerfilepath(std::string filepath) {
        if (LoggerfilepathCallback) {
            LoggerfilepathCallback(filepath);
        }
    }
    void notifylogretriever_status(uint32_t Log_retriever_status) {
        if (Log_retriver_statusCallback) {
            Log_retriver_statusCallback(Log_retriever_status);
        }
    }
    void notifyWakeupReason(uint32_t WakeupReason) {
        if (WakeupReasonCallback) {
            WakeupReasonCallback(WakeupReason);
        }
    }
	void notifyRXSignals_Float(float RXSignals_Float[]){
        if(RXSignals_FloatCallback){
            RXSignals_FloatCallback(RXSignals_Float);
        }
    }
    void notifyDCSD_Req(std::vector<uint8_t> DCSD_Req) 
	{	
        if(DCSD_ReqCallback) 
		{
		    DCSD_ReqCallback(DCSD_Req);
		}
	}
	
	void getCANData()
	{
     uint8_t count = 0;
	 uint8_t count_secure = 0;
		if (m_vehicle_proxy_CANChanel != nullptr) {
		    m_vehicle_proxy_CANChanel->getGetRawEvent().subscribe([&](const CommonAPI::ByteBuffer& buffer){ 
			std::lock_guard<std::mutex> lock(mutex_);
		uint32_t size = 1024;
		uint8_t l_rx_buf[size];
            std::memcpy(l_rx_buf, buffer.data(), buffer.size());   
            uint32_t dataNo = static_cast<uint32_t>((l_rx_buf[0]<<8)|(l_rx_buf[1]));
			uint32_t payload = 0;

			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_VITV)
			{
				payload = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.01 ); 
				notifySpeedUpdated(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_KEY_POS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyIgnitionSts(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ECALL_CRASH)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyEcallCrash(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FEUX_POS)
			{
				payload = static_cast<uint32_t>(l_rx_buf[4]);
				notifyParkLamp(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_JOUR_NUIT)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyDayNightMode(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BCCM_FUEL_TYPE)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFuelType(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MINC)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFuellevel(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONTACT_FREIN_PRK)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyparkbreakstatus(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_T_EXT)
			{
				floatrxvalues[0]= static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.5 ) - 40 ;
				LOG(INFO) << "NTW_RX_SIGNAL_T_EXT : " << floatrxvalues[0];
                notifyRXSignals_Float(floatrxvalues);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HV_BATT_SOH_CAPA_HD)
			{
				floatrxvalues[1] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.03 );
				notifyRXSignals_Float(floatrxvalues);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HV_BATT_SOC)
			{
				floatrxvalues[2] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.1 ) ;
				notifyRXSignals_Float(floatrxvalues);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HV_BATT_ALERT_LOW_SOC)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyHVBatteryMinCharge(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_OBC_PLUG_STATE_rx)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyHVBatteryActiveConnector(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_AUTONOMIE_ZEV)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				payload = static_cast<uint32_t>( ((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 2 );
				notifyHVBatteryMaxRange(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_TOTALBATTENERGY)
			{
				payload = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 32 );
				notifyHVBatteryChargeLevel(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MODE_CHARGE_IN_PROGRESS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyHVBatteryIsCharging(payload);
			}
      		if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MODE_HEURE_CLIENT){
				payload = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
				m_BCMsecurtime[0] = payload;
				notifyBCMSecureTimeUpdated(m_BCMsecurtime);
				}
		    if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TISTAMPSECURECLKHR){
				payload = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
			   m_BCMsecurtime[1] = payload;
				notifyBCMSecureTimeUpdated(m_BCMsecurtime);
				}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TISTAMPSECURECLKMINS){
				payload = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
			   m_BCMsecurtime[2] = payload;
				notifyBCMSecureTimeUpdated(m_BCMsecurtime);
				}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TISTAMPSECURECLKSEC1){
				payload = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
				m_BCMsecurtime[3] = payload;
				notifyBCMSecureTimeUpdated(m_BCMsecurtime);	
				}
		    if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TISTAMPSECURECLKDAY){
				payload = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
				m_BCMsecurtime[4] = payload;
				notifyBCMSecureTimeUpdated(m_BCMsecurtime);
				}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TISTAMPSECURECLKMTH){
				payload = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
			    m_BCMsecurtime[5] = payload;
                notifyBCMSecureTimeUpdated(m_BCMsecurtime);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TISTAMPSECURECLKYR){
				payload = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
			    m_BCMsecurtime[6] = (payload + OFFSET_YEAR);
                notifyBCMSecureTimeUpdated(m_BCMsecurtime);
				}

      		if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MODE_HEURE_CLIENT){
			m_BCMHMItime[0] = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
			notifyBCMHmiTimeUpdated(m_BCMHMItime);
			}
		    if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HEURE_HORLOGE){
			m_BCMHMItime[1] = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
			notifyBCMHmiTimeUpdated(m_BCMHMItime);
				}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MINUTE_HORLOGE){
			m_BCMHMItime[2] = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
			notifyBCMHmiTimeUpdated(m_BCMHMItime);
				}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_JOUR_HORLOGE){
			m_BCMHMItime[3] = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
			notifyBCMHmiTimeUpdated(m_BCMHMItime);
				}
		    if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MOIS_HORLOGE){
			m_BCMHMItime[4] = (static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])));
			notifyBCMHmiTimeUpdated(m_BCMHMItime);
			}
		    if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ANNEE_HORLOGE){
			m_BCMHMItime[5] = (static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) + OFFSET_YEAR));
			notifyBCMHmiTimeUpdated(m_BCMHMItime);
			} 			

			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_AUTONOMIE)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyAutonomy(payload);
				internalSignal_value[12] = payload;
                notifyInternalSignalUpdated(internalSignal_value);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONTACT_FREIN1)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyBrakePedalrSts(payload);
			}	
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_UNITE_DISTANCE)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyCanDistanceUnit(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_INSTANTANEE)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				floatrxvalues[3] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.1  );
				notifyRXSignals_Float(floatrxvalues);
                LOG(INFO) << "NTW_RX_SIGNAL_CONSO_INSTANTANEE : " << floatrxvalues[3];
				internalSignal_value[10] =static_cast<uint32_t>(floatrxvalues[3]);
				notifyInternalSignalUpdated(internalSignal_value);

			}	
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MARCHE_AR_TCUM)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
                if(Dote_data[18] == 0 && Dote_data[28] == 0)
                {
                    notifyTransmGearDisplay(payload); // for ICE engine type
                    LOG(INFO) << "Sending Reverse gear signal for ICE engine type in Manual transmission";
                }
				//notifyReverseGearEngaged(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DYN_SLOPE_VALUE)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				payload = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) - 30 );
				notifyRoadSlope(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ANGLE_VOLANT)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
                int16_t signedValue = static_cast<int16_t>(payload);
                floatrxvalues[4] = signedValue * 0.1;
                LOG(INFO) << "NTW_RX_SIGNAL_ANGLE_VOLANT : " <<  floatrxvalues[4];
                LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
                notifyRXSignals_Float(floatrxvalues);
                notifySteeringAngle(signedValue);
            }  
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_KM_TOTAL)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				floatrxvalues[5] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.1 ;
				notifyRXSignals_Float(floatrxvalues);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_POS_LEVIER_BV_AFFICH)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyTransmGearDisplay(payload); // for default case
                if(payload == 1 && Dote_data[18] == 3)
                {
                    notifyTransmGearDisplay(1); // for MHEV engine type
                    LOG(INFO) << "Sending Reverse gear for MHEV engine type";
                }
                else if(payload == 1 && Dote_data[18] == 0 && (Dote_data[28] == 1 || Dote_data[28] == 2))
                {
                    notifyTransmGearDisplay(1); // for ICE engine type
                    LOG(INFO) << "Sending Reverse gear for ICE engine type";
                }
			}	
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_MOY_ZEV_TRAJ)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				floatrxvalues[6] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.1 ;
				 notifyRXSignals_Float(floatrxvalues);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_BSRF_3__VITESSE_LACET)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				payload  = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])) ;
                int16_t signedValue = static_cast<int16_t>( payload);
                floatrxvalues[7] = signedValue * 0.1;
                LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
				notifyYawSpeed(signedValue);
                notifyRXSignals_Float(floatrxvalues);
			}	
			
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FPAS_ACTIVATION_STATUS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFPASSts(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_VITESSE_LACET_BRUTE)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				 floatrxvalues[8] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.08 ) - 163.84 ;
				 notifyRXSignals_Float(floatrxvalues);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FAULT_PULS_CNT_FL)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFLWheelTickCounterFault(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FAULT_PULS_CNT_FR)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFRWheelTickCounterFault(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FAULT_PULS_CNT_RL)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyRLWheelTickCounterFault(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FAULT_PULS_CNT_RR)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyRRWheelTickCounterFault(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PULS_CNT_WHEEL_FL)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyPulseCountFLWheel(payload);
			}	
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PULS_CNT_WHEEL_FR)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyPulseCountFRWheel(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PULS_CNT_WHEEL_RL)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyPulseCountRLWheel(payload);
			}	
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PULS_CNT_WHEEL_RR)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyPulseCountRRWheel(payload);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_COMMANDES_BSI_36__SENS_ROULAGE)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyDrivingDirection(payload);

			}	

			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_UNITE_VOLUME)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyCapacityUnit(payload);

			}	
            if(dataNo == rawCanSinal::NTW_RX_SIGNAL_NIV_CRB)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyCombustFuelLv(payload);
				internalSignal_value[11] = payload;
				notifyInternalSignalUpdated(internalSignal_value);

			}	
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_EMB_COMP)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyCompressorSts(payload);

			}	
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PEM_ESTIM_ELEC_POWER)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				payload = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 100 );
                int16_t signedValue = static_cast<int16_t>(payload);
                LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
				notifyElectricMotorPower(signedValue);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TEMP_EAU_MOT)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyEngineCoolantTemp(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_REGIME_MOTEUR)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				 floatrxvalues[9] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.125 );
				 notifyRXSignals_Float(floatrxvalues);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MINI_CARB)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFuelLevelMinimumSts(payload);
				internalSignal_value[13] = payload;
				notifyInternalSignalUpdated(internalSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FLG_AVOL_ICN)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyAbsSteeringWheelAngleAcu(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FLG_INIT_AVOL)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyInitSteeringWheenAngleFlag(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PHASE_VIE)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyNetworkMNGT(payload);
                if(payload == 1 || payload == 4){
                    const char* hmi_time_path = "/dev/hmi_time";
                    const char* reference_time_path = "/dev/reference_time";
                    if (chmod(hmi_time_path, 0777) != 0) {
                        LOG(INFO) << "Failed to change permissions 777 for  " << hmi_time_path;
                    }
                    else{
                        LOG(INFO) << "successfully to change permissions 777 for " << hmi_time_path;
                    }
                    if (chmod(reference_time_path, 0777) != 0) {
                        LOG(INFO) << "Failed to change permissions 777 for " << reference_time_path;
                    }
                    else{
                        LOG(INFO) << "successfully to change permissions 777 for " << reference_time_path;
                    }
                }
                else{
                    const char* hmi_time_path = "/dev/hmi_time";
                    const char* reference_time_path = "/dev/reference_time";
                    if (chmod(hmi_time_path, 0000) != 0) {
                        LOG(INFO) << "Failed to change permissions 000 for " << hmi_time_path;
                    }
                    else{
                        LOG(INFO) << "successfully to change permissions 000 for " << hmi_time_path;
                    }
                     if (chmod(reference_time_path, 0000) != 0) {
                        LOG(INFO) << "Failed to change permissions 000 for " << reference_time_path;
                    }
                    else{
                        LOG(INFO) << "successfully to change permissions 000 for " << reference_time_path;
                    }
                }

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BAR_SECT_CENTER_LEFT_FPAS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFPASCenterLeftBarSts(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BAR_SECT_CENTER_RIGHT_FPAS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFPASCenterRightBarSts(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BAR_SECT_LEFT_FPAS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFPASLeftBarSts(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BAR_SECT_RIGHT_FPAS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFPASRightBarSts(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BAR_SECT_CENTER_LEFT_RPAS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyRPASCenterLeftBarSts(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BAR_SECT_CENTER_RIGHT_RPAS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyRPASCenterRightBarSts(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BAR_SECT_LEFT_RPAS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyRPASLeftBarSts(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DRV_INFO_BOOT)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyTrunkDoorSts(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PRESENCE_RQUE)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyTrailerPresent(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_RPAS_ACTIVATION_STATUS)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyRPASSts(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ALLUM_FLECHE)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyEconShftRq(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_SIGN_IVC)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyEngineTorque(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_UNITE_CONSO)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyFuelConsUnit(payload);
				internalSignal_value[16] = payload;
				notifyInternalSignalUpdated(internalSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TOT_ENERGY_ACCUMULATION)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				floatrxvalues[10] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4]))*0.1);
				 notifyRXSignals_Float(floatrxvalues);
				internalSignal_value[14] =static_cast<uint32_t>(floatrxvalues[10]);
				notifyInternalSignalUpdated(internalSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_POS_LEVIER_BV)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyShiftLeverPosition(payload);
                if(Dote_data[18] == 0 && (Dote_data[28] == 1 || Dote_data[28] == 2))
                {
                    notifyTransmGearDisplay(payload); // for ICE engine type
                    LOG(INFO) << "Sending Reverse gear for ICE engine type in automatic transmission";
                }

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_MOY_TRAJ)
			{
				LOG(INFO) << "CAPIVehicleClientGateway : signal_enum : " << dataNo;
				floatrxvalues[11] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4]))*0.1);
				notifyRXSignals_Float(floatrxvalues);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_REQ_VOL_CTRL)
			{
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyReqVolCtrl(payload);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FCT_PRIVACY_MODE)
			{
				
				payload = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyVehPrivacyMode(payload);
				internalSignal_value[15] = payload;
				notifyInternalSignalUpdated(internalSignal_value);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_CPO_COND) {
              payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
              notifyDriverdoorStatusUpdate(payload);
             }
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_CPO_PASS) {
				payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
				notifyPassengerDoorStatusUpdate(payload);
				}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BAR_SECT_RIGHT_RPAS)
			{
				
				CameraSignals_value[0] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyCameraSignalUpdated(CameraSignals_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CHIME_MUTE_RES)
			{
				
				CameraSignals_value[1] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyCameraSignalUpdated(CameraSignals_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_INST_PCT_PWT_ELEC)
			{
				
				EVSignal_value[0] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyEVSignalUpdated(EVSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_INST_PCT_CFT)
			{
				
				EVSignal_value[1] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyEVSignalUpdated(EVSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_TRIP_ELEC_PWT)
			{
				floatrxvalues[13]= static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.01 ;
                notifyRXSignals_Float(floatrxvalues);
			}
            if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_TRIP_CFT)
			{
				floatrxvalues[28] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.01 ;
                notifyRXSignals_Float(floatrxvalues);

			}
			
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_TRIP_OTHER)
			{
                floatrxvalues[14]=static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.01 ;
                notifyRXSignals_Float(floatrxvalues);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_TRIP_ELEC_PWT_REGE)
			{
				floatrxvalues[15]=static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.01 ;
                 notifyRXSignals_Float(floatrxvalues);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_MELAV)
			{
				EVSignal_value[6] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyEVSignalUpdated(EVSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_MEL)
			{
				EVSignal_value[7] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyEVSignalUpdated(EVSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_SOC_BATT)
			{
				EVSignal_value[8] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyEVSignalUpdated(EVSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MODE_CHARGE_IN_PROGRESS)
			{
				EVSignal_value[9] = static_cast<uint32_t>((l_rx_buf[5]<<8)|(l_rx_buf[4]));
				notifyEVSignalUpdated(EVSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_AUTONOMIE_ZEV)
			{
				EVSignal_value[10] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) *2 );
				notifyEVSignalUpdated(EVSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HV_BATT_SOH_CAPA_HD)
			{
                floatrxvalues[1]= static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.03 );
                notifyRXSignals_Float(floatrxvalues);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_RESTBATTENERGY)
			{
				EVSignal_value[12] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 32 );
				notifyEVSignalUpdated(EVSignal_value);

			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_AUXILIARY_POWER)
			{
				floatrxvalues[16]=static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.1 );
                notifyRXSignals_Float(floatrxvalues);
			}
			if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_REMAININGBATTCAPACITY)
			{
                floatrxvalues[17]=static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.001 );
                notifyRXSignals_Float(floatrxvalues);
			}
			

if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP1)
{
    EVSignal_value[15] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40 ;
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP2)
{
    EVSignal_value[16] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40 ;
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP3)
{
    EVSignal_value[17] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP4)
{
    EVSignal_value[18] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]))- 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP5)
{
    EVSignal_value[19] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP6)
{
    EVSignal_value[20] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40;
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP7)
{
    EVSignal_value[21] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP8)
{
    EVSignal_value[22] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP9)
{
    EVSignal_value[23] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP10)
{
    EVSignal_value[24] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP11)
{
    EVSignal_value[25] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP12)
{
    EVSignal_value[26] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP13)
{
    EVSignal_value[27] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP14)
{
    EVSignal_value[28] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP15)
{
    EVSignal_value[29] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP16)
{
    EVSignal_value[30] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP17)
{
    EVSignal_value[31] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP18)
{
    EVSignal_value[32] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP19)
{
    EVSignal_value[33] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP20)
{
    EVSignal_value[34] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP21)
{
    EVSignal_value[35] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP22)
{
    EVSignal_value[36] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP23)
{
    EVSignal_value[37] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP24)
{
    EVSignal_value[38] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP25)
{
    EVSignal_value[39] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP26)
{
    EVSignal_value[40] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP27)
{
    EVSignal_value[41] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP28)
{
    EVSignal_value[42] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP29)
{
    EVSignal_value[43] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP30)
{
    EVSignal_value[44] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP31)
{
    EVSignal_value[45] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP32) {
    EVSignal_value[46] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP33) {
    EVSignal_value[47] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP34) {
    EVSignal_value[48] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP35) {
    EVSignal_value[49] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP36) {
    EVSignal_value[50] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP37) {
    EVSignal_value[51] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40;
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP38) {
    EVSignal_value[52] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP39) {
    EVSignal_value[53] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP40) {
    EVSignal_value[54] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP41) {
    EVSignal_value[55] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP42) {
    EVSignal_value[56] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP43) {
    EVSignal_value[57] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP44) {
    EVSignal_value[58] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP45) {
    EVSignal_value[59] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP46) {
    EVSignal_value[60] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP47) {
    EVSignal_value[61] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP48) {
    EVSignal_value[62] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40;
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP49) {
    EVSignal_value[63] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP50) {
    EVSignal_value[64] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP51) {
    EVSignal_value[65] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP52) {
    EVSignal_value[66] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40; 
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP53) { 
    EVSignal_value[67] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40 );
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_PROBETEMP54) {
    EVSignal_value[68] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40 );
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HV_BATT_TEMP_AVG) { 
    EVSignal_value[69] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40 );
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_VCU_CTRL_INPUT_201h__HVBAT_CHARGING_STATUS) { 
    EVSignal_value[70] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyEVSignalUpdated(EVSignal_value);
}

if(dataNo == rawCanSinal::NTW_RX_SIGNAL_OBC_PLUG_STATE_rx) {
    EVSignal_value[75] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_MOY_ZEV_TRAJ) { 
   floatrxvalues[6]=static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ;
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PEM_ESTIM_ELEC_POWER) { 
    payload = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 100 );
    int16_t signedValue = static_cast<int16_t>(payload);
    EVSignal_value[77] = signedValue;
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HV_BATT_SOC) {
    floatrxvalues[2]=static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 );
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TOT_ENERGY_ACCUMULATION) {
    floatrxvalues[10] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4]))*0.1);
	 notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_SC_INFO_BCM_442h_VCU_CHARG_TYPE) {
    EVSignal_value[80] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_DAT_TBMU_494_TOTAL_HV_BATT_NEGATIVE_ENERGY) { 
    EVSignal_value[81] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 35 );
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_BSRF_14_HV_BATT_SOE_HD) {
    EVSignal_value[82] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 10 );
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_BSRF_16_HV_BATT_SOH_RES_HD) {
    floatrxvalues[27] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 );
    notifyRXSignals_Float(floatrxvalues); 
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_TOTALBATTENERGY) { 
    EVSignal_value[84] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 32 ) ;
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FEUX_POS) {
    EVSignal_value[85] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_P_TEM_LED_BAS_NIV_BATT) {
    EVSignal_value[86] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyEVSignalUpdated(EVSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CPT_TEMPOREL) {

    floatrxvalues[24] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 );
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PRESENCE_RQUE) {
    BCMSignals_value[1] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DRIVER_DETECT_CHANGE) {
    BCMSignals_value[2] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_BSRF_7__ETAT_ESSUYAGE) {
    BCMSignals_value[3] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_BSRF_8__CDE_ESV_AR) {
    BCMSignals_value[4] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_BSRF_8__CDE_ESV) {
    BCMSignals_value[5] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_BSRF_7__NUM_VITESSE) {
    BCMSignals_value[6] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_T_EXT) { 
   floatrxvalues[0]= static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4])) * 0.5 ) - 40 ;
   notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_CMB_SIGNALISATION__FEUX_ROUTE) {
    BCMSignals_value[8] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_CMB_SIGNALISATION__FEUX_CROIS) {
    BCMSignals_value[9] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_JOUR_NUIT) {
    BCMSignals_value[10] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ANNEE_HORLOGE) { 
    BCMSignals_value[11] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) + 2000 );
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HEURE_HORLOGE) {
    BCMSignals_value[12] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_JOUR_HORLOGE) {
    BCMSignals_value[13] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MINUTE_HORLOGE) {
    BCMSignals_value[14] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MOIS_HORLOGE) {
    BCMSignals_value[15] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MODE_HEURE_CLIENT) {
    BCMSignals_value[16] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_UNITE_DISTANCE) {
    BCMSignals_value[17] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_BSI_INF_PROFILS__UNITE_TEMPERATURE) {
    BCMSignals_value[18] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MODE_HEURE_CLIENT) {
    BCMSignals_value[19] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}

if(dataNo == rawCanSinal::NTW_RX_SIGNAL_UNITE_VOLUME) {
    BCMSignals_value[20] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DISPO_UNITES_LANGUE) {
    BCMSignals_value[21] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_PRINCIP_SEV) {
    BCMSignals_value[22] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_UNITE_CONSO) {
    BCMSignals_value[23] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_KM_TOTAL) { 
   floatrxvalues[5]=static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ;
   notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO_MOY_TRAJ) {
floatrxvalues[11] = static_cast<uint32_t>(((l_rx_buf[5]<<8)|(l_rx_buf[4]))*0.1);
notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_COMMANDES_BSI_36_ETAT_JN) {
BCMSignals_value[26] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_1) {
BCMSignals_value[27] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_2) {
BCMSignals_value[28] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_3) {
BCMSignals_value[29] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_4) {
BCMSignals_value[30] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_5) {
BCMSignals_value[31] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_6) {
BCMSignals_value[32] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_7) {
BCMSignals_value[33] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_8) {
BCMSignals_value[34] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FCT_PRIVACY_MODE) {
BCMSignals_value[35] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyBCMSignalsUpdated(BCMSignals_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ECALL_CRASH) {
CanCommonSignal_value[0] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BOUC_CEINT_PASS) {
CanCommonSignal_value[1] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BOUC_CEINT_AVMIL) {
CanCommonSignal_value[2] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BOUC_CEINT_AR2MIL) {
CanCommonSignal_value[3] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BOUC_CEINT_AR2D) {
CanCommonSignal_value[4] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BOUC_CEINT_AR2G) {
CanCommonSignal_value[5] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BOUC_CEINT_AR3G) {
CanCommonSignal_value[6] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BOUC_CEINT_AR3D) {
CanCommonSignal_value[7] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BOUC_CEINT_COND) {
CanCommonSignal_value[8] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FRONT_F_VIT) {
CanCommonSignal_value[9] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FRONT_G_VIT) {
CanCommonSignal_value[10] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FRONT_M_VIT_1) {
CanCommonSignal_value[11] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FRONT_M_VIT_2) {
CanCommonSignal_value[12] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FRONT_REPAR) {
CanCommonSignal_value[13] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ARR_F_VIT) {
CanCommonSignal_value[14] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ARR_G_VIT) {
CanCommonSignal_value[15] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ARR_M_VIT) {
CanCommonSignal_value[16] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ARR_REPAR) {
CanCommonSignal_value[17] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_LAT_F_VIT) {
CanCommonSignal_value[18] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_LAT_G_VIT) {
CanCommonSignal_value[19] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_LAT_M_VIT) {
CanCommonSignal_value[20] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_LAT_REPAR) {
CanCommonSignal_value[21] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TYPE_DIR) {
CanCommonSignal_value[22] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_SIDE_OF_LATERAL_CRASH) {
CanCommonSignal_value[23] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TEAU) { 
CanCommonSignal_value[27] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40 );
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_T_HUILE) {  
CanCommonSignal_value[28] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) - 40 );
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_NIV_HUILE) {
CanCommonSignal_value[29] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_K_MAINT) { 
CanCommonSignal_value[30] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 20 );
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_NB_JOUR_MAINTENANCE) {
CanCommonSignal_value[31] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_SIGN_MAINT) {
CanCommonSignal_value[32] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_SIGN_ECHEANCE) {
CanCommonSignal_value[33] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_GMP) {
CanCommonSignal_value[34] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_INFO_CRASH) {
CanCommonSignal_value[35] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PIETON) {
CanCommonSignal_value[36] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FATC_CABIN_TEMPERATURE) { 
    floatrxvalues[25] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.5 ;
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_VCU_CTRL_GEAR_POS_INFO) {
CanCommonSignal_value[38] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
    if(CanCommonSignal_value[38] == 5 && Dote_data[18] == 1)
    {
        notifyTransmGearDisplay(1);
        LOG(INFO) << "In GearPosition signal Sending Reverse gear for EV engine type "; // for EV engine type
    }
    else
    {
        notifyTransmGearDisplay(0);
        LOG(INFO) << "In GearPosition signal Sending non Reverse gear for EV engine type";
    }
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BCCM_FUEL_TYPE) {
CanCommonSignal_value[39] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CONSO) { 
CanCommonSignal_value[40] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 80 );
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_VITESSE_VEHICULE_ROUES) { 
    floatrxvalues[20] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.01 );
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PULS_CNT_WHEEL_FL) {
CanCommonSignal_value[42] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PULS_CNT_WHEEL_FR) {
CanCommonSignal_value[43] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PULS_CNT_WHEEL_RL) {
CanCommonSignal_value[44] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PULS_CNT_WHEEL_RR) {
CanCommonSignal_value[45] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DISTANCE_ROUES) {  
    floatrxvalues[21] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 );
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ACCEL_LONGI_ROUES) {    
 floatrxvalues[18]= static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.08 ;
 notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_EFCMNT_PDLE_ACCEL) { 

    floatrxvalues[19]= static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.5 );
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ACCEL_LAT) {
payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
int16_t signedValue = static_cast<int16_t>(payload);
floatrxvalues[23] = signedValue * 0.05;
LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_INFO_CMDM_MODE_VHL) {
CanCommonSignal_value[50] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CLIGNO_G) {
CanCommonSignal_value[51] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CLIGNO_D) {
CanCommonSignal_value[52] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_CLIGNOTANTS) {
CanCommonSignal_value[53] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_IND_ROUE_ARD) {
CanCommonSignal_value[54] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_UC_FREIN_5ED__ETAT_IND_ROUE_ARG) {
CanCommonSignal_value[55] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_UC_FREIN_5ED__ETAT_IND_ROUE_AVD) {
CanCommonSignal_value[56] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_UC_FREIN_5ED__ETAT_IND_ROUE_AVG) {
CanCommonSignal_value[57] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_COMMANDES_BSI_36__SENS_ROULAGE) {
CanCommonSignal_value[58] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_BSRF_3__VITESSE_LACET) {
payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
int16_t signedValue = static_cast<int16_t>(payload);
floatrxvalues[7]=signedValue*0.1;
notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DIST_PARCOURUE_TRAJET) {
CanCommonSignal_value[60] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FLG_AVOL_ICN) {
CanCommonSignal_value[62] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_FLG_INIT_AVOL) {
CanCommonSignal_value[63] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ACCEL_LONGI_CALIB) { 
    floatrxvalues[22] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.02 ) - 40.96 ;
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_STATUS_SOC) {
CanCommonSignal_value[65] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MINC) {
CanCommonSignal_value[66] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HV_BATT_ALERT_LOW_SOC) {
CanCommonSignal_value[67] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_DONNEES_BSI_RAPIDES__VITM) { 
floatrxvalues[26] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.125 );
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DIST_TRAJINST_TOT_ZEVPCT) {
CanCommonSignal_value[69] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ALERTES) {
CanCommonSignal_value[70] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_NUMERO_BLOC) {
CanCommonSignal_value[71] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ACCES_JDA) {
CanCommonSignal_value[72] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_NB_BLOC) {
CanCommonSignal_value[73] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_RTAB_CHRG_REMAINING_TIME) { 
CanCommonSignal_value[74] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 5 ) ;
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_CPO_COND) {
CanCommonSignal_value[77] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_CPO_PASS) {
CanCommonSignal_value[78] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_CPO_ARD) {
CanCommonSignal_value[79] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_CPO_ARG) {
CanCommonSignal_value[80] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_ETAT_COFFRE) {
CanCommonSignal_value[81] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_REMOTE_IMMO_STATUS) {
CanCommonSignal_value[82] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_AFFICHAGE_HORLOGE) {
CanCommonSignal_value[83] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_MODE_CONFIG_VHL) {
CanCommonSignal_value[84] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCanCommonSignalUpdated(CanCommonSignal_value);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_VIS_1) {
    CAN_RX_Signals[0] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    notifyCAN_RX_SignalUpdated(CAN_RX_Signals);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CRASH_MAX_DELTA_VX_oETAT_INFO_CRASH_19) {
    payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    int16_t signedValue = static_cast<int16_t>(payload);
    Signed_CAN_RX_Signals[0] = signedValue;
    LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
    notifySigned_CAN_RX_SignalUpdated(Signed_CAN_RX_Signals);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_CRASH_MAX_DELTA_VY_oETAT_INFO_CRASH_19) {
    payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    int16_t signedValue = static_cast<int16_t>(payload);
    Signed_CAN_RX_Signals[1] = signedValue;
    LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
    notifySigned_CAN_RX_SignalUpdated(Signed_CAN_RX_Signals);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DIST_VHL_L_LINE_EXT) {
    payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    int16_t signedValue = static_cast<int16_t>(payload);
    floatrxvalues[29] = signedValue * 0.016;
    LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DIST_VHL_R_LINE_EXT) {
    payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    int16_t signedValue = static_cast<int16_t>(payload);
    floatrxvalues[30] = signedValue * 0.016;
    LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_SEC_DIST_VHL_L_LINE_EXT) { 
    payload= static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    int16_t signedValue = (static_cast<int16_t>(payload));
    floatrxvalues[31]=signedValue*0.016;
    LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HV_BATT_REAL_CURR_HD) { 
    payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    int16_t signedValue = (static_cast<int16_t>(payload));
    floatrxvalues[12]= signedValue * 0.1 ;
    notifyRXSignals_Float(floatrxvalues);
    LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PEM_SPEED_HD) {
    payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    int16_t signedValue = static_cast<int16_t>(payload);
    Signed_CAN_RX_Signals[6] = signedValue;
    LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
    notifySigned_CAN_RX_SignalUpdated(Signed_CAN_RX_Signals);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_REM_REAL_TRQ) {
    payload = static_cast<uint32_t>((l_rx_buf[4]));
    int8_t signedValue = static_cast<int8_t>(payload);
    Signed_CAN_RX_Signals[7] = signedValue;
    LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
    notifySigned_CAN_RX_SignalUpdated(Signed_CAN_RX_Signals);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PEM_ENGINE_RPM) { 
    payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 10 ;
    int16_t signedValue = static_cast<int16_t>(payload);
    Signed_CAN_RX_Signals[8] = signedValue;
    LOG(INFO) << "CAPIVehicleClientGateway : signedValue : " << signedValue;
    notifySigned_CAN_RX_SignalUpdated(Signed_CAN_RX_Signals);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_HS3_CDE_BSRF_16_HV_BATT_REAL_VOLT_HD) {   
    floatrxvalues[32] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ; 
    LOG(INFO) << "NTW_RX_SIGNAL_HS3_CDE_BSRF_16_HV_BATT_REAL_VOLT_HD : " << floatrxvalues[32];
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_SEC_DIST_VHL_R_LINE_EXT) {               
    payload = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) ;     
    int16_t signedValue = static_cast<int16_t>(payload);
    floatrxvalues[33] = signedValue * 0.016; 
    LOG(INFO) << "NTW_RX_SIGNAL_SEC_DIST_VHL_R_LINE_EXT : " << floatrxvalues[33];
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT1_14B) {           
    floatrxvalues[34] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ; 
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT10_14B) { 
    floatrxvalues[35] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001; 
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT100_14B) { 
    floatrxvalues[36] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ; 
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT101_14B) { 
    floatrxvalues[37] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ; 
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT102_14B) { 
    floatrxvalues[38] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT103_14B) { 
    floatrxvalues[39] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT104_14B) { 
    floatrxvalues[40] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT105_14B) { 
    floatrxvalues[41] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT106_14B) { 
    floatrxvalues[42] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT107_14B) { 
    floatrxvalues[43] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT108_14B) { 
    floatrxvalues[44] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT11_14B) { 
    floatrxvalues[45] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT12_14B) { 
    floatrxvalues[46] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT13_14B) { 
    floatrxvalues[47] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT14_14B) { 
    floatrxvalues[48] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT15_14B) { 
    floatrxvalues[49] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT16_14B) { 
    floatrxvalues[50] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT17_14B) { 
    floatrxvalues[51] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT18_14B) { 
    floatrxvalues[52] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT19_14B) { 
    floatrxvalues[53] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT2_14B) { 
    floatrxvalues[54] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT20_14B) { 
    floatrxvalues[55] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT21_14B) { 
    floatrxvalues[56] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT22_14B) { 
    floatrxvalues[57] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT23_14B) { 
    floatrxvalues[58] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT24_14B) { 
    floatrxvalues[59] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT25_14B) { 
    floatrxvalues[60] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT26_14B) { 
    floatrxvalues[61] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT27_14B) { 
    floatrxvalues[62] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT28_14B) { 
    floatrxvalues[63] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT29_14B) { 
    floatrxvalues[64] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT3_14B) { 
    floatrxvalues[65] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT30_14B) { 
    floatrxvalues[66] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT31_14B) { 
    floatrxvalues[67] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT32_14B) { 
    floatrxvalues[68] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT33_14B) { 
    floatrxvalues[69] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT34_14B) { 
    floatrxvalues[70] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT35_14B) { 
    floatrxvalues[71] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT36_14B) { 
    floatrxvalues[72] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT37_14B) { 
    floatrxvalues[73] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT38_14B) { 
    floatrxvalues[74] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT39_14B) { 
    floatrxvalues[75] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT4_14B) { 
    floatrxvalues[76] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT40_14B) { 
    floatrxvalues[77] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT41_14B) { 
    floatrxvalues[78] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;       
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT42_14B) { 
    floatrxvalues[79] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT43_14B) { 
    floatrxvalues[80] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT44_14B) { 
    floatrxvalues[81] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT45_14B) { 
    floatrxvalues[82] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT46_14B) { 
    floatrxvalues[83] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT47_14B) { 
    floatrxvalues[84] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT48_14B) { 
    floatrxvalues[85] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;       
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT49_14B) { 
    floatrxvalues[86] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT5_14B) { 
    floatrxvalues[87] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT50_14B) { 
    floatrxvalues[88] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT51_14B) { 
    floatrxvalues[89] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT52_14B) { 
    floatrxvalues[90] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT53_14B) { 
    floatrxvalues[91] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT54_14B) { 
    floatrxvalues[92] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT55_14B) { 
    floatrxvalues[93] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT56_14B) { 
    floatrxvalues[94] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT57_14B) { 
    floatrxvalues[95] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT58_14B) { 
    floatrxvalues[96] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT59_14B) { 
    floatrxvalues[97] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;     
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT6_14B) { 
    floatrxvalues[98] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT60_14B) { 
    floatrxvalues[99] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT61_14B) { 
    floatrxvalues[100] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT62_14B) { 
    floatrxvalues[101] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT63_14B) { 
    floatrxvalues[102] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT64_14B) { 
    floatrxvalues[103] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT65_14B) { 
    floatrxvalues[104] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT66_14B) { 
    floatrxvalues[105] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT67_14B) { 
    floatrxvalues[106] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT68_14B) { 
    floatrxvalues[107] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT69_14B) { 
    floatrxvalues[108] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT7_14B) { 
    floatrxvalues[109] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT70_14B) { 
    floatrxvalues[110] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT71_14B) { 
    floatrxvalues[111] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT72_14B) { 
    floatrxvalues[112] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT73_14B) { 
    floatrxvalues[113] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT74_14B) { 
    floatrxvalues[114] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;          
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT75_14B) { 
    floatrxvalues[115] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT76_14B) { 
    floatrxvalues[116] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT77_14B) { 
    floatrxvalues[117] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT78_14B) { 
    floatrxvalues[118] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT79_14B) { 
    floatrxvalues[119] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT8_14B) { 
    floatrxvalues[120] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT80_14B) { 
    floatrxvalues[121] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT81_14B) { 
    floatrxvalues[122] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT82_14B) { 
    floatrxvalues[123] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT83_14B) { 
    floatrxvalues[124] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT84_14B) { 
    floatrxvalues[125] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT85_14B) { 
    floatrxvalues[126] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT86_14B) { 
    floatrxvalues[127] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT87_14B) { 
    floatrxvalues[128] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT88_14B) { 
    floatrxvalues[129] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT89_14B) { 
    floatrxvalues[130] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT9_14B) { 
    floatrxvalues[131] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT90_14B) { 
    floatrxvalues[132] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT91_14B) { 
    floatrxvalues[133] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT92_14B) { 
    floatrxvalues[134] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT93_14B) { 
    floatrxvalues[135] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT94_14B) { 
    floatrxvalues[136] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT95_14B) { 
    floatrxvalues[137] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT96_14B) { 
    floatrxvalues[138] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT97_14B) { 
    floatrxvalues[139] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT98_14B) { 
    floatrxvalues[140] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_BMS_BATTCELLVOLT99_14B) { 
    floatrxvalues[141] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.001 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DIST) { 
    floatrxvalues[142] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_SECU_VITV) { 
    floatrxvalues[143] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_PRESSION_MAITRE_CYL) { 
    floatrxvalues[144] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ) - 55 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_U_BATT_BECB) { 
    floatrxvalues[145] = static_cast<uint32_t>(((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.01 ) + 5 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_VIT_CONS_LVV_RVV) { 
    floatrxvalues[146] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.01 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_DIST_VHL_TOTAL) { 
    floatrxvalues[147] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_VOL_FUEL_CONSO_TOTAL) { 
    floatrxvalues[148] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.01 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TIME_HV_BATT_OVER_CH_PWR) { 
    floatrxvalues[149] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TIME_HV_BATT_OVER_DCH_PWR) { 
    floatrxvalues[150] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_VOLONTE_COND) { 
    floatrxvalues[151] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.5 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TIME_HV_BATT_HIGH_TEMP_HIGH_SOC) { 
    floatrxvalues[152] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TIME_HV_BATT_HIGH_TEMP_LOW_SOC) { 
    floatrxvalues[153] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ;      
    LOG(INFO) << "NTW_RX_SIGNAL_TIME_HV_BATT_HIGH_TEMP_LOW_SOC : " << floatrxvalues[153];
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TIME_HV_BATT_HIGH_VOLTAGE) { 
    floatrxvalues[154] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_SIGNAL_TIME_HV_BATT_OVER_TEMPERATURE) { 
    floatrxvalues[155] = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4])) * 0.1 ;      
    notifyRXSignals_Float(floatrxvalues);
}
if(dataNo == rawCanSinal::NTW_RX_DCSD_DIAG_REQ_Com_DCSD_Signal_Rx)
{
    uint32_t DCSD_Req_length = static_cast<uint32_t>((l_rx_buf[5] << 8) | (l_rx_buf[4]));
    std::vector<uint8_t> DCSD_Req;
    LOG(INFO) << "DCSD_Req_length:" << DCSD_Req_length;
    for(unsigned i = 6 ; i < DCSD_Req_length+6 ; i++)
    {
        DCSD_Req.push_back(l_rx_buf[i]);
    }
    for(unsigned i = 0 ; i < DCSD_Req.size() ; i++)
    {
        LOG(INFO) << "NTW_RX_DCSD_DIAG_REQ_Com_DCSD_Signal_Rx:" << DCSD_Req[i];
    }
    notifyDCSD_Req(DCSD_Req);
    DCSD_Req.clear();
}
            LOG(INFO) << "CAPIVehicleClientGateway :dataNo:"<<dataNo <<"payload:"<<payload;			



	});
}	
		else{
			LOG(INFO) << "CAPIVehicleClientGateway : Proxy Null";
		}
}
	
	
	void registerSpeedCallback(std::function<void(uint32_t)> callback) {
		speedCallback_ = callback;
	}
	void registerIgnitionStsCallback(std::function<void(uint32_t)> IgnitionSts) {
		IgnitionStsCallback_ = IgnitionSts;
	}
	void registerParkLampCallback(std::function<void(uint32_t)> ParkLamp) {
		ParkLampCallback = ParkLamp;
	}
	void registerEcallCrashCallback(std::function<void(uint32_t)> EcallCrash) {
		EcallCrashCallback_ = EcallCrash;
	}
	void registerDayNightModeCallback(std::function<void(uint32_t)> DayNightMode) {
		DayNightModeCallback = DayNightMode;
		
	}
	void registerFuellevelCallback(std::function<void(uint32_t)> Fuellevel) {
		FuellevelCallback = Fuellevel;
	}
	void registerFuelTypeCallback(std::function<void(uint32_t)> FuelType) {
		FuelTypeCallback = FuelType;
	}
	void registerParkBreakStatusCallback(std::function<void(uint32_t)> ParkBreakStatus) {
		ParkBreakStatusCallback = ParkBreakStatus;
	}
	void registerOutTempCallback(std::function<void(uint32_t)> OutTemp) {
		OutTempCallback = OutTemp;
	}
	void registerHVBatterySOHDataCallback(std::function<void(uint32_t)> HVBatterySOHData) {
		HVBatterySOHDataCallback = HVBatterySOHData;
	}
	void registerHVBatteryPercentageCallback(std::function<void(uint32_t)> HVBatteryPercentage) {
		HVBatteryPercentageCallback = HVBatteryPercentage;
	}
	void registerHVBatteryMinChargeCallback(std::function<void(uint32_t)> HVBatteryMinCharge) {
		HVBatteryMinChargeCallback = HVBatteryMinCharge;
	}
	void registerHVBatteryActiveConnectorCallback(std::function<void(uint32_t)> HVBatteryActiveConnector) {
		HVBatteryActiveConnectorCallback = HVBatteryActiveConnector;
	}
	void registerHVBatteryMaxRangeCallback(std::function<void(uint32_t)> HVBatteryMaxRange) {
		HVBatteryMaxRangeCallback = HVBatteryMaxRange;
	}
	void registerHVBatteryChargeLevelCallback(std::function<void(uint32_t)> HVBatteryChargeLevel) {
		HVBatteryChargeLevelCallback = HVBatteryChargeLevel;
	}
	void registerHVBatteryIsChargingCallback(std::function<void(uint32_t)> HVBatteryIsCharging) {
		HVBatteryIsChargingCallback = HVBatteryIsCharging;
	}
	void registerAutonomyCallback(std::function<void(uint32_t)> Autonomy) {
		AutonomyCallback = Autonomy;
	}
	void registerBrakePedalrStsCallback(std::function<void(uint32_t)> BrakePedalrSts) {
		BrakePedalrStsCallback = BrakePedalrSts;
	}
	void registerCanDistanceUnitCallback(std::function<void(uint32_t)> CanDistanceUnit) {
		CanDistanceUnitCallback = CanDistanceUnit;
	}
	void registerInstantFuelConsCallback(std::function<void(uint32_t)> InstantFuelCons) {
		InstantFuelConsCallback = InstantFuelCons;
	}
	void registerReverseGearEngagedConsCallback(std::function<void(uint32_t)> ReverseGearEngaged) {
		ReverseGearEngagedCallback = ReverseGearEngaged;
	}
	void registerRoadSlopeCallback(std::function<void(uint32_t)> RoadSlope) {
		RoadSlopeCallback = RoadSlope;
	}
	void registerSteeringAngleCallback(std::function<void(uint32_t)> SteeringAngle) {
		SteeringAngleCallback = SteeringAngle;
	}
	void registerTotalKMCallback(std::function<void(uint32_t)> TotalKM) {
		TotalKMCallback = TotalKM;
	}
	void registerTransmGearDisplayCallback(std::function<void(uint32_t)> TransmGearDisplay) {
		TransmGearDisplayCallback = TransmGearDisplay;
	}
	void registerTripAvgBattPowerConsCallback(std::function<void(uint32_t)> TripAvgBattPowerCons) {
		TripAvgBattPowerConsCallback = TripAvgBattPowerCons;
	}
	void registerYawSpeedCallback(std::function<void(uint32_t)> TripAvgBattPowerCons) {
		TripAvgBattPowerConsCallback = TripAvgBattPowerCons;
	}
	void registerGrossYawRateCallback(std::function<void(uint32_t)> GrossYawRate) {
		GrossYawRateCallback = GrossYawRate;
	}
	void registerFLWheelTickCounterFaultCallback(std::function<void(uint32_t)> FLWheelTickCounterFault) {
		FLWheelTickCounterFaultCallback = FLWheelTickCounterFault;
	}
	void registerFRWheelTickCounterFaultCallback(std::function<void(uint32_t)> FRWheelTickCounterFault) {
		FRWheelTickCounterFaultCallback = FRWheelTickCounterFault;
	}
	void registerRLWheelTickCounterFaultCallback(std::function<void(uint32_t)> RLWheelTickCounterFault) {
		RLWheelTickCounterFaultCallback = RLWheelTickCounterFault;
	}
	void registerRRWheelTickCounterFaultCallback(std::function<void(uint32_t)> RRWheelTickCounterFault) {
		RRWheelTickCounterFaultCallback = RRWheelTickCounterFault;
	}
	void registerPulseCountFLWheelCallback(std::function<void(uint32_t)> PulseCountFLWheel) {
		PulseCountFLWheelCallback = PulseCountFLWheel;
	}
	void registerPulseCountFRWheelCallback(std::function<void(uint32_t)> PulseCountFRWheel) {
		PulseCountFRWheelCallback = PulseCountFRWheel;
	}
	void registerPulseCountRLWheelCallback(std::function<void(uint32_t)> PulseCountRLWheel) {
		PulseCountRLWheelCallback = PulseCountRLWheel;
	}
	void registerPulseCountRRWheelCallback(std::function<void(uint32_t)> PulseCountRRWheel) {
		PulseCountRRWheelCallback = PulseCountRRWheel;
	}
	void registerFPASStsCallback(std::function<void(uint32_t)> FPASSts) {
		FPASStsCallback = FPASSts;
	}
	void registerDrivingDirectionCallback(std::function<void(uint32_t)> DrivingDirection) {
		DrivingDirectionCallback = DrivingDirection;
	}
	void registerCapacityUnitCallback(std::function<void(uint32_t)> CapacityUnit) {
		CapacityUnitCallback = CapacityUnit;
	}
	void registerCombustFuelLvCallback(std::function<void(uint32_t)> CombustFuelLv) {
		CombustFuelLvCallback = CombustFuelLv;
	}
	void registerCompressorStsCallback(std::function<void(uint32_t)> CompressorSts) {
		CompressorStsCallback = CompressorSts;
	}
	void registerElectricMotorPowerCallback(std::function<void(int32_t)> ElectricMotorPower) {
		ElectricMotorPowerCallback = ElectricMotorPower;
	}
	void registerEngineCoolantTempCallback(std::function<void(uint32_t)> EngineCoolantTemp) {
		EngineCoolantTempCallback = EngineCoolantTemp;
	}
	void registerEngineRPMCallback(std::function<void(uint32_t)> EngineRPM) {
		EngineRPMCallback = EngineRPM;
	}
	void registerFuelLevelMinimumStsCallback(std::function<void(uint32_t)> FuelLevelMinimumSts) {
		FuelLevelMinimumStsCallback = FuelLevelMinimumSts;
	}
	void registerAbsSteeringWheelAngleAcuCallback(std::function<void(uint32_t)> AbsSteeringWheelAngleAcu) {
		AbsSteeringWheelAngleAcuCallback = AbsSteeringWheelAngleAcu;
	}
	void registerInitSteeringWheenAngleFlagCallback(std::function<void(uint32_t)> InitSteeringWheenAngleFlag) {
		InitSteeringWheenAngleFlagCallback = InitSteeringWheenAngleFlag;
	}
	void registerNetworkMNGTCallback(std::function<void(uint32_t)> NetworkMNGT) {
		NetworkMNGTCallback = NetworkMNGT;
	}
	void registerFPASCenterLeftBarStsCallback(std::function<void(uint32_t)> FPASCenterLeftBarSts) {
		FPASCenterLeftBarStsCallback = FPASCenterLeftBarSts;
	}
	void registerFPASCenterRightBarStsCallback(std::function<void(uint32_t)> FPASCenterRightBarSts) {
		FPASCenterRightBarStsCallback = FPASCenterRightBarSts;
	}
	void registerFPASLeftBarStsCallback(std::function<void(uint32_t)> FPASLeftBarSts) {
		FPASLeftBarStsCallback = FPASLeftBarSts;
	}
	void registerFPASRightBarStsCallback(std::function<void(uint32_t)> FPASRightBarSts) {
		FPASRightBarStsCallback = FPASRightBarSts;
	}
	void registerRPASCenterLeftBarStsCallback(std::function<void(uint32_t)> RPASCenterLeftBarSts) {
		RPASCenterLeftBarStsCallback = RPASCenterLeftBarSts;
	}
	void registerRPASCenterRightBarStsCallback(std::function<void(uint32_t)> RPASCenterRightBarSts) {
		RPASCenterRightBarStsCallback = RPASCenterRightBarSts;
	}
	void registerTrunkDoorStsCallback(std::function<void(uint32_t)> TrunkDoorSts) {
		TrunkDoorStsCallback = TrunkDoorSts;
	}
	void registerRPASLeftBarStsCallback(std::function<void(uint32_t)> RPASLeftBarSts) {
		RPASLeftBarStsCallback = RPASLeftBarSts;
	}
	void registerTrailerPresentCallback(std::function<void(uint32_t)> TrailerPresent) {
		TrailerPresentCallback = TrailerPresent;
	}
	void registerRPASStsCallback(std::function<void(uint32_t)> RPASSts) {
		RPASStsCallback = RPASSts;
	}
	void registerEconShftRqCallback(std::function<void(uint32_t)> EconShftRq) {
		EconShftRqCallback = EconShftRq;
	}
	void registerEngineTorqueCallback(std::function<void(uint32_t)> EngineTorque) {
		EngineTorqueCallback = EngineTorque;
	}
	void registerFuelConsUnitCallback(std::function<void(uint32_t)> FuelConsUnit) {
		FuelConsUnitCallback = FuelConsUnit;
	}
	void registerRegeneratedEnergyCallback(std::function<void(uint32_t)> RegeneratedEnergy) {
		RegeneratedEnergyCallback = RegeneratedEnergy;
	}
	void registerShiftLeverPositionCallback(std::function<void(uint32_t)> ShiftLeverPosition) {
		ShiftLeverPositionCallback = ShiftLeverPosition;
	}
	void registerTripAverageFuelConsumptionCallback(std::function<void(uint32_t)> TripAverageFuelConsumption) {
		TripAverageFuelConsumptionCallback = TripAverageFuelConsumption;
	}
	void registerReqVolCtrlCallback(std::function<void(uint32_t)> ReqVolCtrl) {
		ReqVolCtrlCallback = ReqVolCtrl;
	}
	void registerVehPrivacyModeCallback(std::function<void(uint32_t)> VehPrivacyMode) {
		VehPrivacyModeCallback = VehPrivacyMode;
	}
	void registerVinNumberCallback(std::function<void(std::vector<uint32_t>)> vinNumber){
        vinNumberCallback = vinNumber;
	}
	void registerBCMHMItimeCallback(std::function<void(uint32_t [])> bcmHMItime){
       bcmHMItimeCallback = bcmHMItime;
	}
	void registerBCMSECURtimeCallback(std::function<void(uint32_t [])> bcmSECURtime){
       bcmSECURtimeCallback = bcmSECURtime;
	}
	void registerIMUdata_callback(std::function<void(std::vector<int32_t>)> imuDataParam){
       imuParamCallback = imuDataParam;
	}
	void registerDotteParamCallback(std::function<void(std::vector<uint32_t>)> dotteParam){
       dotteParamCallback = dotteParam;
	}
	    void registerInternalSignalCallback(std::function<void(uint32_t [])>  internalSignal){
		internalSignalCallback = internalSignal;
	}
	void registerEVSignalsCallback(std::function<void(int32_t [])>  EVSignal){
		EVSignalsCallback = EVSignal;
	}
	void registerCanCommonSignalsCallback(std::function<void(int32_t [])>  CanCommonSignal){
		CanCommonSignalsCallback = CanCommonSignal;
	}
	void registerCameraSignalsCallback(std::function<void(uint32_t [])>  CameraSignal){
		CameraSignalsCallback = CameraSignal;
	}
	void registerBCMSignalsCallback(std::function<void(uint32_t [])>  BCMSignal){
		BCMSignalsCallback = BCMSignal;
	}
	void registerDTCParamCallback(std::function<void(std::vector<uint32_t>)> dtcParam){
       dtcParamCallback = dtcParam;
	}
	void registerDLTParamCallback(std::function<void(std::vector<uint32_t>)> dLTParam){
       dLTParamCallback = dLTParam;
	}
 	void registerDLT_VectorParamCallback(std::function<void(std::vector<uint8_t>)> dLT_VectorValueParam){
	         dLT_VectorParamCallback = dLT_VectorValueParam;
	}
	void registerCalibrationDataCallback(std::function<void(uint32_t [])> calDATA){
	         calibrationDataCallback = calDATA;
	}
		void registerCalibrationDataCallback_Audio(std::function<void(uint32_t [])> calDATA){
	         calibrationDataCallback_Audio = calDATA;
	}
		void registerCalibrationDataCallback_Camera(std::function<void(uint32_t [])> calDATA){
	         calibrationDataCallback_Camera = calDATA;
	}
		void registerCalibrationDataCallback_Array(std::function<void(std::vector<uint8_t>)> calDATA){
	         calibrationDataCallback_Array = calDATA;
	}
        void registercalibrationDataforSrecCallback(std::function<void(std::vector<uint32_t>)> calSrecDATA){
             calibrationDataforSrecCallback = calSrecDATA;      
    }         
		void registerTemperatureCallback(std::function<void(uint32_t)> Temperature) {
		TemperatureCallback = Temperature;
	}
    void registerSupplyVoltageCallback(std::function<void(uint32_t)> SupplyVoltage){
        SupplyVoltageCallback = SupplyVoltage;
    }
    void registerCallActiveSTRDisplayPopupStatusCallback(std::function<void(uint32_t)> callActiveSTRDisplayPopupStatus){
        CallActiveSTRDisplayPopupStatusCallback = callActiveSTRDisplayPopupStatus;
    }
    void registerSTRCancelShutdownStatusCallback(std::function<void(uint32_t)> STRCancelShutdownStatus){
        STRCancelShutdownStatusCallback = STRCancelShutdownStatus;
    }
	void registerUSB_UpdateCallback(std::function<void(std::vector<uint16_t>)> USB_Update) {
		USB_UpdateCallback = USB_Update;
	}
	void registerDID_RID_callback(std::function<void(std::vector<uint16_t>)> m_dID_message_type){
	         dIDrIDParamCallback = m_dID_message_type;
	}
	 void registerPowermodeCallback(std::function<void(uint32_t)> Powermode) {
		PowermodeCallback = Powermode;
	}
	void registerCAN_RX_SignalsCallback(std::function<void(uint32_t [])>  CAN_RX_Signals){
		CAN_RX_SignalCallbacK = CAN_RX_Signals;
	}
	void registerDEM_DTCstatuscallback(std::function<void(std::vector<uint16_t>)> m_DEM_DTC_message_type){
		DEM_DTCStatusCallback = m_DEM_DTC_message_type;
	}
	void register_DriverdoorStatuscallback(std::function<void(uint8_t)> driverdoorStatus){
		driverdoorStatusCallback = driverdoorStatus;
	}
	void register_PassengerDoorStatuscallback(std::function<void(uint8_t)> passengerDoorStatus){
		passengerDoorStatusCallback = passengerDoorStatus;
	}
    void registerSigned_CAN_RX_SignalsCallback(std::function<void(int32_t [])>  Signed_CAN_RX_Signals){
		Signed_CAN_RX_SignalCallbacK = Signed_CAN_RX_Signals;
	}
    void registerLog_RetriverCallback(std::function<void(uint32_t)> Log_Retriver_data) {
        Log_retriver_ackCallback = Log_Retriver_data;
    }
    void registerlogerfilepathCallback(std::function<void(std::string)> logger_filepath) {
        LoggerfilepathCallback = logger_filepath;
    }
    void registerLog_RetriverStatusCallback(std::function<void(uint32_t)> Log_Retriver_status) {
        Log_retriver_statusCallback = Log_Retriver_status;
    }
    void registerSOC_WakeupReasonCallback(std::function<void(uint32_t)> WakeUpReason) {
        WakeupReasonCallback = WakeUpReason;
    }
    void registerRXSignals_FloatCallback(std::function<void(float [])> RXSignals_Float) {
        RXSignals_FloatCallback = RXSignals_Float;
    }
    void registerMarelliPNCallback(std::function<void(std::vector<uint8_t>)> MarelliPN){
	    MarelliPNCallback = MarelliPN;
	}
    void registerEOLHWIDCallback(std::function<void(std::vector<uint8_t>)> EOLHWID){
	    EOLHWIDCallback = EOLHWID;
	}
	void registerAuthenticationZoneCallback(std::function<void(std::vector<uint8_t>)> AuthenticationZone){
	    AuthenticationZoneCallback = AuthenticationZone;
	}
	void registerApplicationSoftwareIdentificationCallback(std::function<void(std::vector<uint8_t>)> ApplicationSoftwareIdentification){
	    ApplicationSoftwareIdentificationCallback = ApplicationSoftwareIdentification;
	}
	    void registerApplicationDataIdentificationCallback(std::function<void(std::vector<uint8_t>)> ApplicationDataIdentification){
	    ApplicationDataIdentificationCallback = ApplicationDataIdentification;
	}
	    void registerDataLibraryidentifierCallback(std::function<void(std::vector<uint8_t>)> DataLibraryidentifier){
	    DataLibraryidentifierCallback = DataLibraryidentifier;
	}
	    void registerVehicleAppsIdentifierCallback(std::function<void(std::vector<uint8_t>)> VehicleAppsIdentifier){
	    VehicleAppsIdentifierCallback = VehicleAppsIdentifier;
	}
    void registerDCSD_DIAG_ReqCallback(std::function<void(std::vector<uint8_t>)> DCSD_DIAG_Req){
	    DCSD_ReqCallback = DCSD_DIAG_Req;
	}

void onPowerModeAttributeChanged(uint32_t newValue) {
    LOG(INFO) << "newValue_powermode" << newValue ;
    notifypowermodedata(newValue);
    

}
	void getPowerMode()
{

        if (m_vehicle_proxy != nullptr) {
            m_vehicle_proxy->getPowerModeStatus_VHALAttribute().getChangedEvent().subscribe(std::bind(&VehicleClientImpl::onPowerModeAttributeChanged, this, std::placeholders::_1));
        
        }
        
		else {
            LOG(INFO) << "CAPIVehicleClientGateway : Power Mode *** Proxy not available";
        }
}

void onModemAttributeChanged(iMicro::iMicroStructs::diag_message_type data){
	std::vector <uint16_t> dID_RID_arr;
    std::uint8_t diag_message_array;
    LOG(INFO) <<"The onModemAttributeChanged has changed\n"; 
    LOG(INFO) <<"getP_MsgID:"<< data.getP_MsgID();
    LOG(INFO) <<"getP_AppID:"<< data.getP_AppID(); 
    LOG(INFO) <<"getP_Handle:"<< data.getP_Handle();
    LOG(INFO) <<"getP_length:"<< data.getP_length();
    auto DIAGVector = data.getP_DiagMessage();

	dID_message_type m_dID_message_type;
	m_dID_message_type.dID_MsgID = data.getP_MsgID();
	m_dID_message_type.dID_AppID = data.getP_AppID();
	m_dID_message_type.dID_Handle = data.getP_Handle();
	m_dID_message_type.dID_length = data.getP_length();
	m_dID_message_type.dID_DiagMessage = data.getP_DiagMessage();

	dID_RID_arr.push_back(data.getP_MsgID());
	dID_RID_arr.push_back(data.getP_AppID());
	dID_RID_arr.push_back(data.getP_Handle());
	dID_RID_arr.push_back(data.getP_length());

		for(auto N :m_dID_message_type.dID_DiagMessage)
    dID_RID_arr.push_back(N);

     if(dID_RID_arr[4] == 34)
     {
        notifyDID_RID_DataUpdate(dID_RID_arr);
     }
     else if (dID_RID_arr[4] == 46)
     {
         notifyDID_RID_DataUpdate(dID_RID_arr);
     }
     else if (dID_RID_arr[4] == 49)
     {
        notifyDID_RID_DataUpdate(dID_RID_arr);
     }
	ALOGD("dID_RID_arr[5] :%d",dID_RID_arr[5]);
    ALOGD("dID_RID_arr[6] :%d",dID_RID_arr[6]);
    ALOGD("dID_RID_arr[7] :%d",dID_RID_arr[7]);
    ALOGD("dID_RID_arr[4] :%d",dID_RID_arr[4]);
    dID_RID_arr.clear();
}		
	void getDIDsubscribe() {
	if (m_DIAGProxy != nullptr) {			 
		 	m_DIAGProxy->getSignalEvent_DIAG_SendtoModemEvent().subscribe(std::bind(&VehicleClientImpl::onModemAttributeChanged,this ,std::placeholders::_1));
		 }
			else {
			LOG(INFO) << "CAPIVehicleClientGateway : DID Param *** Proxy not available";
        }
 } 
 void onIMUDataManagerAttributeChanged(iMicro::iMicroStructs::IMUDataManager_MCU_t IMUVal) {
	 std::vector<int32_t> imuDATAParam;

	imuDATAParam.push_back(IMUVal.getX_Acceleration());
	imuDATAParam.push_back(IMUVal.getY_Acceleration());
	imuDATAParam.push_back(IMUVal.getZ_Acceleration());
	imuDATAParam.push_back(IMUVal.getAlpha_Angle());
	imuDATAParam.push_back(IMUVal.getBeta_Angle());
	imuDATAParam.push_back(IMUVal.getGamma_Angle());
	imuDATAParam.push_back(IMUVal.getHeading());
    imuDATAParam.push_back(IMUVal.getTimeStamp());
     notifyIMUParamUpdated(imuDATAParam);
	 imuDATAParam.clear();
}
 	void getIMUsubscribe() {
	if (m_vehicle_IMUProxy != nullptr) {			 
		 m_vehicle_IMUProxy->getIMUDataManagerAttribute().getChangedEvent().subscribe(std::bind(&VehicleClientImpl::onIMUDataManagerAttributeChanged,this ,std::placeholders::_1));
		 }
			else {
			LOG(INFO) << "CAPIVehicleClientGateway : IMU DATA Param *** Proxy not available";
        }
 }
 
void onDotteNVMParametersAttributeChanged(iMicro::iMicroStructs::DotteNVMParameters_MCU_t dotteVal) 
{
LOG(INFO) << " Inside CAPIVehicleClientGateway onDotteNVMParametersAttributeChanged";
	std::vector<uint32_t> dotteParam;
    std::vector<uint32_t> readData;
	
     dotteParam.push_back(dotteVal.getVEHICLE_BRAND());
     dotteParam.push_back(dotteVal.getDISPLAY_TYPE());
     dotteParam.push_back(dotteVal.getCOUNTRY_CODE());
     dotteParam.push_back(dotteVal.getSTART_ANM_MODE());
     dotteParam.push_back(dotteVal.getCONT_START_ANM());
     dotteParam.push_back(dotteVal.getWELCOME_SOUND());
     dotteParam.push_back(dotteVal.getVEH_LINE());
     dotteParam.push_back(dotteVal.getVEH_FUEL_TYPE());
     dotteParam.push_back(dotteVal.getDAYNYT_THM_EN());
     dotteParam.push_back(dotteVal.getEV_CONN_TYPE());
     dotteParam.push_back(dotteVal.getAMP_PRESENT());
     dotteParam.push_back(dotteVal.getTHEMED_SOUNDS());
     dotteParam.push_back(dotteVal.getSGN_SRC_ENABLED());
     dotteParam.push_back(dotteVal.getPAM_CHIME_EN());
     dotteParam.push_back(dotteVal.getPAM_CONFIG());
     dotteParam.push_back(dotteVal.getPAS_VOL_STRAT());
     dotteParam.push_back(dotteVal.getPAM_CHIME_MODE());
     dotteParam.push_back(dotteVal.getSYS_TYPE());
     dotteParam.push_back(dotteVal.getSIGNAL_TYPE());
     dotteParam.push_back(dotteVal.getRPAS_PRESENT());
     dotteParam.push_back(dotteVal.getFPAS_PRESENT());
     dotteParam.push_back(dotteVal.getRVC_TYPE());
     dotteParam.push_back(dotteVal.getVP1_ENABLED());
     dotteParam.push_back(dotteVal.getDRIVER_SIDE());
     dotteParam.push_back(dotteVal.getSTART_AUD_VOL());
     dotteParam.push_back(dotteVal.getREV_MUTE_EN());
     dotteParam.push_back(dotteVal.getEQL_SETTING());
     dotteParam.push_back(dotteVal.getSPK_CONFIG());
     dotteParam.push_back(dotteVal.getSIVI_GEAR_BOX_TYPE());
     dotteParam.push_back(dotteVal.getSIVI_HVAC_TYPE());
     dotteParam.push_back(dotteVal.getVEH_THEME());
     dotteParam.push_back(dotteVal.getDAB_PRESENT());
     dotteParam.push_back(dotteVal.getAM_TUNER_EN());
     dotteParam.push_back(dotteVal.getHW_IGN_PRESENT());
     dotteParam.push_back(dotteVal.getHW_ACC_PRESENT());
     dotteParam.push_back(dotteVal.getACC_PRESENT());
     dotteParam.push_back(dotteVal.getCONN_PRESENT());
     dotteParam.push_back(dotteVal.getNAV_PRESENT());
     dotteParam.push_back(0);
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getPWR_BTN_TYPE());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getREGULATION_TYPE());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getTRF_CONN_SVC());
     dotteParam.push_back(dotteVal.getDISCL_CTRLS());
     dotteParam.push_back(dotteVal.getDISCL_REG_SPEC());
     dotteParam.push_back(dotteVal.getDISCL_SCR_MODE());
     dotteParam.push_back(dotteVal.getTYPE_XCALL());
     dotteParam.push_back(0);
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getAUDIO_BRAND());
     dotteParam.push_back(0);
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getSWL_THM_EN());
     dotteParam.push_back(dotteVal.getDRV_DIST_MODE());
     dotteParam.push_back(dotteVal.getCLIM_HMI_EN());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getMHU_TYPE());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getICS_PRESENT());
     dotteParam.push_back(dotteVal.getXCALL_SPK_PRES());
     dotteParam.push_back(dotteVal.getSURROUND());
     dotteParam.push_back(dotteVal.getSDW_CONFIG());
     dotteParam.push_back(dotteVal.getSDW_CHIME_MODE());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getBSS_PRESENT());
     dotteParam.push_back(dotteVal.getBSS_CONFIG());
     dotteParam.push_back(dotteVal.getSRRP_PRESENCE());
     dotteParam.push_back(dotteVal.getSRRP2_PRESENCE());
     dotteParam.push_back(dotteVal.getHALF_PRESENT());
     dotteParam.push_back(0);
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getACALLED_PRES());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getVEHICLE_TYPE());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getECALL_BTN_PRESS());
     dotteParam.push_back(dotteVal.getACALL_BTN_PRESS());
     dotteParam.push_back(dotteVal.getECALLED_PRESS());
     dotteParam.push_back(dotteVal.getBUB_PRESENCE());
     dotteParam.push_back(dotteVal.getHTD_WHL_PRESENT());
     dotteParam.push_back(dotteVal.getHTDS_DRV_PRES());
     dotteParam.push_back(dotteVal.getHTDS_PAS_PRES());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getFRONT_DEFROSTER());
     dotteParam.push_back(dotteVal.getREAR_DEFROSTER());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getSWS_TYPE());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getPROF_DET_EN());
     dotteParam.push_back(dotteVal.getRVC_INST_LOC());
     dotteParam.push_back(dotteVal.getDYN_GRID_EN());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getRPAS_CHM_ENT());
     dotteParam.push_back(dotteVal.getFPAS_CHM_ENT());
     dotteParam.push_back(dotteVal.getMEM_DYN_GRID());
     dotteParam.push_back(dotteVal.getPRV_MODE_MENU());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getIDLE_SCR_MODE());
     dotteParam.push_back(0);
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getSIVI_UNIT_SYNC_EN());
     dotteParam.push_back(dotteVal.getSIVI_SHOW_UNITS());
     dotteParam.push_back(dotteVal.getSIVI_AIRBAG_INT());
     dotteParam.push_back(0);
     dotteParam.push_back(dotteVal.getSUSPEND_RAM_DURATION());
     dotteParam.push_back(dotteVal.getLISTEN_MODE1_DURATION());
     dotteParam.push_back(dotteVal.getLISTEN_MODE2_DURATION());
     dotteParam.push_back(dotteVal.getLISTEN_MODE3_DURATION());
     dotteParam.push_back(dotteVal.getLISTEN_MODE_1_CONN_RETRY_TIME());
     dotteParam.push_back(dotteVal.getLISTEN_MODE_3_WAKE_UP_TIME());
     dotteParam.push_back(dotteVal.getPOWER_BUTTON_TYPE());
     dotteParam.push_back(dotteVal.getSANITY_REBOOT_PERIOD());
     dotteParam.push_back(dotteVal.getMAX_CALL_DURATION());	

      dotteParam.push_back( (dotteVal.getSIVI_MAX_AUTONOMY().at(0)) << 8 | (dotteVal.getSIVI_MAX_AUTONOMY().at(1)));
      dotteParam.push_back((dotteVal.getSIVI_HV_BATT_CAP().at(0)) << 8| (dotteVal.getSIVI_HV_BATT_CAP().at(1)));
      LOG(INFO) << "CAPIVehicleClientGateway: dotteParam size " << (uint32_t)dotteParam.size();	
	        notifydotteParamUpdated(dotteParam);
			
	for(unsigned i = 0; i < dotteParam.size(); i++){
        Dote_data[i] = dotteParam[i];
        LOG(INFO) << "CAPIVehicleClientGateway: dotteParam :   " << i<< " ->"<<(uint32_t)dotteParam[i];
        LOG(INFO) << "CAPIVehicleClientGateway: Dote_data :   " << i<< " ->"<<(uint32_t)Dote_data[i];

	}
            std::string filePath = "/mnt/anim/dote_config.txt";
            {    
            std::ofstream outFile(filePath);  
              if (outFile.is_open()) {
            for (size_t i = 0; i < dotteParam.size(); ++i) {
                outFile << parameterNames[i] << " " << dotteParam[i] << std::endl;  
            }
            outFile.close();
            ALOGD("Data written successfully in config.txt.");
            if (chmod(filePath.c_str(), 0777) != 0) {
                ALOGD("Failed to change permissions of the file:");
            } else {
                ALOGD("File permissions set to 777 successfully.");
            }
            } else {
            ALOGD("Could not open the file for writing.");
            }
            }
            {
            std::ifstream inFile(filePath);
            if (inFile.is_open()) {
            uint32_t number;
            while (inFile >> number) {
                readData.push_back(number);  
            }
            inFile.close();
            } else {
            }
            }
            for(uint32_t i = 0; i<readData.size(); i++)
            {
                ALOGD("readData %d %d ",i,readData[i]);
            }
            for (const auto& number : readData) {
            ALOGD("number:%d",number);
            }
            readData.clear();
            dotteParam.clear();
	
            

        
}
    void getDottesubscribe(){
        ALOGD("Inside getDottesubscribe");
		 if (m_vehicle_proxy != nullptr) {
			 LOG(INFO) << "CAPIVehicleClientGateway inside getDottesubscribe() call ";
			 m_vehicle_proxy->getDotteNVMParameters_MCUAttribute().getChangedEvent().subscribe(
           std::bind(&VehicleClientImpl::onDotteNVMParametersAttributeChanged, this, std::placeholders::_1));
		 }
	   		else {
				
            LOG(INFO) << "CAPIVehicleClientGateway : dotte Param *** Proxy not available";
        }
 }

 void onDTCSignalAttributeChanged(iMicro::iMicroStructs::DTCSignals_MCU_t dtc) {
     LOG(INFO) <<"CAPIVehicleClientGateway : The onDTCSignalAttributeChanged has changed";
	std::vector<uint32_t> dtcParam;
    dtcParam.push_back(dtc.getDTC_0x900311());
    dtcParam.push_back(dtc.getDTC_0x900312());
    dtcParam.push_back(dtc.getDTC_0x900313());
	dtcParam.push_back(dtc.getDTC_0x900371());
	dtcParam.push_back(dtc.getDTC_0x900411());
	dtcParam.push_back(dtc.getDTC_0x900412());
	dtcParam.push_back(dtc.getDTC_0x900413());
	dtcParam.push_back(dtc.getDTC_0x900471());
	dtcParam.push_back(dtc.getDTC_0x9E0116());
	dtcParam.push_back(dtc.getDTC_0x9E0117());
	dtcParam.push_back(dtc.getDTC_0xAE0551());
	dtcParam.push_back(dtc.getDTC_0xAE0E51());
	dtcParam.push_back(dtc.getDTC_0xAE0E62());
	dtcParam.push_back(dtc.getDTC_0xC03013());
	dtcParam.push_back(dtc.getDTC_0xC03111());
	dtcParam.push_back(dtc.getDTC_0xC03212());
	dtcParam.push_back(dtc.getDTC_0xC03313());
	dtcParam.push_back(dtc.getDTC_0xC03411());
	dtcParam.push_back(dtc.getDTC_0xC03512());
	dtcParam.push_back(dtc.getDTC_0xC0362B());
	dtcParam.push_back(dtc.getDTC_0xC04013());
	dtcParam.push_back(dtc.getDTC_0xC04111());
	dtcParam.push_back(dtc.getDTC_0xC04212());
	dtcParam.push_back(dtc.getDTC_0xC04313());
	dtcParam.push_back(dtc.getDTC_0xC04411());
	dtcParam.push_back(dtc.getDTC_0xC04512());
	dtcParam.push_back(dtc.getDTC_0xC1400());
	dtcParam.push_back(dtc.getDTC_0xC4220());
	dtcParam.push_back(dtc.getDTC_0xD9017C());
	dtcParam.push_back(dtc.getDTC_0xD90187());
	dtcParam.push_back(dtc.getDTC_0xD90188());
	dtcParam.push_back(dtc.getDTC_0xD90287());
	dtcParam.push_back(dtc.getDTC_0xD90387());

	//notifydtcParamUpdated(dtcParam);

   for(unsigned i = 0U;i<dtcParam.size();++i)
   LOG(INFO) << "CAPIVehicleClientGateway :dtcParam: "<<i<<"->  "<<dtcParam[i];

	dtcParam.clear();
}

 void getDTCsubscribe(){
	if (m_vehicle_proxy != nullptr) {
			 LOG(INFO) << "CAPIVehicleClientGateway inside getDTCsubscribe() call ";
			 m_vehicle_proxy->getDTCSignals_MCUAttribute().getChangedEvent().subscribe(
           std::bind(&VehicleClientImpl::onDTCSignalAttributeChanged, this, std::placeholders::_1));
		 }
	   	else {
		        LOG(INFO) << "CAPIVehicleClientGateway : DTC Param *** Proxy not available";
        }
 }
 void onDEMEventStatusEventChanged(::iMicro::iMicroStructs::dem_eventstatus_type val)
{
	std::vector<uint16_t> m_DEM_DTC_data;
 ::iMicro::iMicroTypes::dem_eventstatus_enum payload = val.getP_EventStatus();
    LOG_INF("getP_EventStatus::%d\n",static_cast<int>(payload));
     LOG_INF("getP_DtcCode::%d\n",val.getP_DtcCode());
	m_DEM_DTC_data.push_back(val.getP_EventStatus());
	m_DEM_DTC_data.push_back(val.getP_DtcCode());
	
	notifyDEM_DTCstatusUpdate(m_DEM_DTC_data);
	m_DEM_DTC_data.clear();
}
   void getDEM_DTCStatussubscribe(){
	if (m_DEMProxy != nullptr) {
			 m_DEMProxy->getSignalResponse_DEM_EventStatusEvent().subscribe(
           std::bind(&VehicleClientImpl::onDEMEventStatusEventChanged, this, std::placeholders::_1)); 
		 		 }
	   	else {
		        LOG(INFO) << "CAPIVehicleClientGateway : DTC Param *** Proxy not available";
        }
 }
 void getfilepathdata(const ::VHAL::LOGR::LOGR_Types::LOGR_status &_status, const std::string &_file_path)
 {
    ALOGD("Inside getfilepathdata");
    notifyloggerfilepath(_file_path);
    notifyLog_retriver_ack(_status);


 }
 void getLoggerdatasubscribe(){
    LOG(INFO)<< ("Inside getLoggerdatasubscribe");
    if (m_TCU_proxy != nullptr)
    {
        m_TCU_proxy->getRequest_finishedEvent().subscribe(
		[this](const ::VHAL::LOGR::LOGR_Types::LOGR_status &_status,
			   const std::string &_file_path) {
			this->getfilepathdata(_status, _file_path);
		});
    }
    else{
        LOG(INFO) << "CAPIVehicleClientGateway : m_TCU_proxy *** Proxy not available";
    }
    
 }

void onDLTDataAttributeChanged(iMicro::iMicroStructs::DLT_Var_MCU_t dltVal){
	std::vector<uint32_t> dLTParam;	 
	 std::vector<uint8_t> dLTVector;
    LOG(INFO) << "The onDLTDataAttributeChanged has changed"; 
    LOG(INFO) << "getHandbrakeStatus::"<< (uint32_t)dltVal.getHandbrakeStatus();
	
   dLTVector = dltVal.getMcpuVcpuSwVersion();
	dLTVector.insert(dLTVector.begin(),1);
    notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getMcpuVcpuSwVersion: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();
	
 	dLTVector = dltVal.getEcuSerialNumber();
	dLTVector.insert(dLTVector.begin(),2);
    notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getEcuSerialNumber: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();
	
 	dLTVector = dltVal.getEcuHwVersionNumber();
	dLTVector.insert(dLTVector.begin(),3);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getEcuHwVersionNumber: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();
	
	dLTVector = dltVal.getEcuHwPartNumber();
	dLTVector.insert(dLTVector.begin(),4);
    notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getEcuHwPartNumber: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();
	
	dLTVector = dltVal.getEcuHwNumbers();
	dLTVector.insert(dLTVector.begin(),5);
    notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getEcuHwNumbers: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();
	
    dLTVector = dltVal.getEcuSwVersionNumber();
	dLTVector.insert(dLTVector.begin(),6);
    notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getEcuSwVersionNumber: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();
	
    dLTVector = dltVal.getEcuSwPartNumber();
	dLTVector.insert(dLTVector.begin(),7);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getEcuSwPartNumber: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();
	
    dLTVector = dltVal.getEcuSwNumbers();
	dLTVector.insert(dLTVector.begin(),8);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getEcuSwNumbers: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();
	
    dLTVector = dltVal.getEcuSparePartNumber();
	dLTVector.insert(dLTVector.begin(),9);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getEcuSparePartNumber: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();

    dLTVector = dltVal.getRead_Cyber_UIN();
	dLTVector.insert(dLTVector.begin(),10);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getRead_Cyber_UIN: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();

    dLTVector = dltVal.getECUManufacturingDateData();
	dLTVector.insert(dLTVector.begin(),11);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getECUManufacturingDateData: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();

    dLTVector = dltVal.getSupplierManufacturerECUSoftwareVersionNum();
	dLTVector.insert(dLTVector.begin(),12);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getSupplierManufacturerECUSoftwareVersionNum: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();

    dLTVector = dltVal.getInternalMarellibuildVersion();
	dLTVector.insert(dLTVector.begin(),13);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getInternalMarellibuildVersion: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();

    dLTVector = dltVal.getManufacturer_Name();
	dLTVector.insert(dLTVector.begin(),14);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getManufacturer_Name: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();

    dLTVector = dltVal.getBootLoaderVersionNumber();
	dLTVector.insert(dLTVector.begin(),15);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getBootLoaderVersionNumber: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();

    dLTVector = dltVal.getGeneric_ID1();
	dLTVector.insert(dLTVector.begin(),16);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getGeneric_ID1: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();

    dLTVector = dltVal.getGeneric_ID2();
	dLTVector.insert(dLTVector.begin(),17);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getGeneric_ID2: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();

    dLTVector = dltVal.getGeneric_ID3();
	dLTVector.insert(dLTVector.begin(),18);
	notifydLTVectorParamUpdated(dLTVector);
	for(unsigned i = 0U;i<dLTVector.size();++i)
	LOG(INFO) << "CAPIVehicleClientGateway :getGeneric_ID3: "<<"   -> "<<(uint32_t)dLTVector[i];
	dLTVector.clear();
	
	McpuVcpuSwVersion = dltVal.getMcpuVcpuSwVersion();
	McpuVcpuSwVersion.insert(McpuVcpuSwVersion.begin(),1);
	EcuSerialNumber = dltVal.getEcuSerialNumber();
	EcuSerialNumber.insert(EcuSerialNumber.begin(),2);
    EcuHwVersionNumber = dltVal.getEcuHwVersionNumber();
	EcuHwVersionNumber.insert(EcuHwVersionNumber.begin(),3);
	EcuHwPartNumber = dltVal.getEcuHwPartNumber();
	EcuHwPartNumber.insert(EcuHwPartNumber.begin(),4);
	EcuHwNumbers = dltVal.getEcuHwNumbers();
	EcuHwNumbers.insert(EcuHwNumbers.begin(),5);
	EcuSwVersionNumber = dltVal.getEcuSwVersionNumber();
	EcuSwVersionNumber.insert(EcuSwVersionNumber.begin(),6);
	EcuSwPartNumber = dltVal.getEcuSwPartNumber();
	EcuSwPartNumber.insert(EcuSwPartNumber.begin(),7);
	EcuSwNumbers = dltVal.getEcuSwNumbers();
	EcuSwNumbers.insert(EcuSwNumbers.begin(),8);
	EcuSparePartNumber = dltVal.getEcuSparePartNumber();
	EcuSparePartNumber.insert(EcuSparePartNumber.begin(),9);
	Read_Cyber_UIN = dltVal.getRead_Cyber_UIN();
	Read_Cyber_UIN.insert(Read_Cyber_UIN.begin(),10);
	ECUManufacturingDateData = dltVal.getECUManufacturingDateData();
	ECUManufacturingDateData.insert(ECUManufacturingDateData.begin(),11);
	SupplierManufacturerECUSoftwareVersionNum = dltVal.getSupplierManufacturerECUSoftwareVersionNum();
	SupplierManufacturerECUSoftwareVersionNum.insert(SupplierManufacturerECUSoftwareVersionNum.begin(),12);
	InternalMarellibuildVersion = dltVal.getInternalMarellibuildVersion();
	InternalMarellibuildVersion.insert(InternalMarellibuildVersion.begin(),13);
	Manufacturer_Name = dltVal.getManufacturer_Name();
	Manufacturer_Name.insert(Manufacturer_Name.begin(),14);
	BootLoaderVersionNumber = dltVal.getBootLoaderVersionNumber();
	BootLoaderVersionNumber.insert(BootLoaderVersionNumber.begin(),15);
	Generic_ID1 = dltVal.getGeneric_ID1();
	Generic_ID1.insert(Generic_ID1.begin(),16);
	Generic_ID2 = dltVal.getGeneric_ID2();
	Generic_ID2.insert(Generic_ID2.begin(),17);
	Generic_ID3 = dltVal.getGeneric_ID3();
	Generic_ID3.insert(Generic_ID3.begin(),18);
	
    dLTParam.push_back(dltVal.getHandbrakeStatus());
	dLTParam.push_back(dltVal.getHeadlightsStatus());
	dLTParam.push_back(dltVal.getSteeringWheelCtrlChange());
	dLTParam.push_back(dltVal.getIviTempSensorOutputs());
	dLTParam.push_back(dltVal.getSystemInterfaceFailures());
	dLTParam.push_back(dltVal.getLoudspeakerStatus());
	dLTParam.push_back(dltVal.getMicrophoneStatus());
	dLTParam.push_back(dltVal.getEcuGyroAcceleroCompassSt());
	dLTParam.push_back(dltVal.getActiveEcuDTCs());
	dLTParam.push_back(dltVal.getBubChargePercentage());
	dLTParam.push_back(dltVal.getBubOverITempChargeDischargeEvent());
	dLTParam.push_back(dltVal.getBubDisconnectionEvent());
	
     notifydLTParamUpdated(dLTParam);
	for(unsigned i = 0U;i<dLTParam.size();++i)
    LOG(INFO) << "CAPIVehicleClientGateway :DLTParam: "<<i<<"->  "<<(uint32_t)dLTParam[i];
    dLTParam.clear();

}

void onTemperatureDataAttributeChanged(uint16_t newValue) {
    LOG(INFO) << "The onTemperatureDataAttributeChanged has changed. New value:" << newValue;
	notifyTemperatureMode(newValue);
}
void onSupplyVoltageDataAttributeChanged(uint32_t newValue){
    LOG(INFO) << "The onSupplyVoltageDataAttributeChanged has changed. New value:" << newValue;
    notifySupplyVoltage(newValue);
}
void onCallActiveSTRDisplayPopupStatusAttributeChanged(uint8_t displayPopupInputFromMCU){
    LOG(INFO) << "onCallActiveSTRDisplayPopupStatusAttributeChanged() displayPopupInputFromMCU : "<<displayPopupInputFromMCU;
    uint32_t displayPopupInputFromMCUValue = static_cast<uint32_t>(displayPopupInputFromMCU);
    notifyCallActiveSTRDisplayPopupStatus(displayPopupInputFromMCUValue);
}

void onSTRCancelShutdownAttributeChanged(uint8_t cancelShutdownSTRInputFromMCU){
    LOG(INFO) << "onSTRCancelShutdownAttributeChanged() cancelShutdownSTRInputFromMCU : "<<cancelShutdownSTRInputFromMCU;
    uint32_t cancelShutdownSTRInputFromMCUValue = static_cast<uint32_t>(cancelShutdownSTRInputFromMCU);
    notifySTRCancelShutdownStatus(cancelShutdownSTRInputFromMCUValue);
}

 void getDLTsubscribe(){
	if (m_vehicle_proxy != nullptr) {
			 LOG(INFO) << "CAPIVehicleClientGateway inside getDLTsubscribe() call ";
			 m_vehicle_proxy->getDLT_Var_MCUAttribute().getChangedEvent().subscribe(
           std::bind(&VehicleClientImpl::onDLTDataAttributeChanged, this, std::placeholders::_1));
		 }
	   	else {
		        LOG(INFO) << "CAPIVehicleClientGateway : DLT Param *** Proxy not available";
        }
 }

void getTemperaturesubscribe()
{
	if (m_vehicle_proxy != nullptr) {
			 LOG(INFO) << "CAPIVehicleClientGateway inside getTemperaturesubscribe() call ";
			 m_vehicle_proxy->getTemperatureData_MCUAttribute().getChangedEvent().subscribe(
             std::bind(&VehicleClientImpl::onTemperatureDataAttributeChanged, this, std::placeholders::_1));
		 }
	   	else {
		        LOG(INFO) << "CAPIVehicleClientGateway : Temperature Param *** Proxy not available";
        }
} 
void getsupplyvgesubscribe(){
    if (m_vehicle_proxy != nullptr) {
		LOG(INFO) << "CAPIVehicleClientGateway inside getsupplyvgesubscribe() call ";
		m_FIRSTProxy->getIM_VUC_ID_VOLTAGE_VALUEAttribute().getChangedEvent().subscribe(
        std::bind(&VehicleClientImpl::onSupplyVoltageDataAttributeChanged, this, std::placeholders::_1));
	}
	else {
		LOG(INFO) << "CAPIVehicleClientGateway : m_vehicle_proxy *** Proxy not available";
    }
}
void getCallActiveSTRDisplayPopupStatusSubscribe(){
    if(m_vehicle_proxy != nullptr) {
        LOG(INFO) << "getCallActiveSTRDisplayPopupStatusSubscribe subscribing the call ";
        m_vehicle_proxy->getShowHmiPopUpForCallAttribute().getChangedEvent().subscribe(
        std::bind(&VehicleClientImpl::onCallActiveSTRDisplayPopupStatusAttributeChanged, this, std::placeholders::_1));
    }
    else{
        LOG(INFO) << "getCallActiveSTRDisplayPopupStatusSubscribe m_vehicle_proxy not available";
    }
}

void getSTRCancelShutdownSubscribe(){
    if(m_vehicle_proxy != nullptr) {
        LOG(INFO) << "getSTRCancelShutdownSubscribe subscribing the call ";
        m_vehicle_proxy->getSuspendProgressCommandAttribute().getChangedEvent().subscribe(
        std::bind(&VehicleClientImpl::onSTRCancelShutdownAttributeChanged, this, std::placeholders::_1));
    }
    else{
        LOG(INFO) << "getSTRCancelShutdownSubscribe m_vehicle_proxy not available";
    }
}

void getUSB_Updatesubscribe()
{
	if (m_vehicle_proxy != nullptr) {
			 LOG(INFO) << "CAPIVehicleClientGateway inside get USB_Updatesubscribe() call ";
			 m_vehicle_proxy->getSignalEvent_USBUPDATE_VENDOR_SOCEvent().subscribe(
             std::bind(&VehicleClientImpl::OnUSBUPDATEVENDORSOCEventChanged, this, std::placeholders::_1));
		 }
	   	else {
		        LOG(INFO) << "CAPIVehicleClientGateway : USB_Update Param *** Proxy not available";
        }
} 
void getUSB_Update_detailssubscribe()
{
	if (m_vehicle_proxy != nullptr) {
			 LOG(INFO) << "CAPIVehicleClientGateway inside get USB_Update_details subscribe() call ";
			 m_vehicle_proxy->getSignalEvent_USBUPDATE_DETAILS_VENDOR_SOCEvent().subscribe(
             std::bind(&VehicleClientImpl::OnUSBUPDATEDETAILSVENDORSOCEventChanged, this, std::placeholders::_1));
		 }
	   	else {
		        LOG(INFO) << "CAPIVehicleClientGateway : USB_Update_details subscribe Param *** Proxy not available";
        }
} 
void OnUSBUPDATEVENDORSOCEventChanged(iMicro::iMicroStructs::USBUPDATE_VENDOR_type _USBUPDATE_VENDOR_SOC) 
{
  LOG(INFO) << "CAPIVehicleClientGateway inside USB_Update attribute call ";	
	std::vector<uint16_t> USB_Update;
	
			USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_USB_VENDOR_PACKAGE());
			USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_DISK_SPACE());
			USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_INVALID_SOFTWARE());
			USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_UPDATE_COUNTDOWN_TIMER());
			USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_ABORT_USB_SOFTWARE_UPDATE());
			USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_DOWNGRADE_SOFTWARE());
			USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_USB_UPDATE_PROGRESS());
			USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_USB_UPDATE_REMAINING_TIME());
			USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_USB_UPDATE_FAILURE());
			USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_AUTOMATIC_RESTART());
            USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_USERCONSENT_NEWUPDATE());
            USB_Update.push_back(_USBUPDATE_VENDOR_SOC.getVENDOR_CONTINUE_UPDATE());
            
			notifyUSB_Update(USB_Update);
			USB_Update.clear();
}
void OnUSBUPDATEDETAILSVENDORSOCEventChanged(std::vector< uint8_t > USB_Update_Details_array) 
{
  LOG(INFO) << "CAPIVehicleClientGateway inside USB_UpdateDetails attribute call ";	
	std::vector<uint32_t> USB_Update_details;
	        for(uint32_t i=0 ; i<USB_Update_Details_array.size(); i++)
			{
			USB_Update_details.push_back(USB_Update_Details_array[i]);
			}
            notifyUSB_Update_Details(USB_Update_details);
			USB_Update_details.clear();
}
			


			
void onInternalVariablesAttributeChanged(iMicro::iMicroStructs::InternalVariables_MCU_t Val) {
    LOG(INFO) << "The onInternalVariablesAttributeChanged has changed\n";

  	 if(internalSignal::INTERNAL_VoiceRecognitionReq == Val.getInternaVariable_Enum()){
	internalSignal_value[0] = Val.getInternaVariable_Data();
	LOG(INFO) << "CAPIVehicleClientGateway :INTERNAL_VoiceRecognitionReq " << (uint32_t)Val.getInternaVariable_Data();
	notifyInternalSignalUpdated(internalSignal_value);
	 }

	  if(internalSignal::INTERNAL_SourceReq == Val.getInternaVariable_Enum()){
	   internalSignal_value[1] = Val.getInternaVariable_Data();
	   notifyInternalSignalUpdated(internalSignal_value);
	   }

	  if(internalSignal::INTERNAL_SeekUpAcceptCallReq == Val.getInternaVariable_Enum()){
	 internalSignal_value[2] = Val.getInternaVariable_Data();
	 notifyInternalSignalUpdated(internalSignal_value);
	  }

	  if(internalSignal::INTERNAL_SeekDownRejectCallReq == Val.getInternaVariable_Enum()){
	 internalSignal_value[3] = Val.getInternaVariable_Data();
	 notifyInternalSignalUpdated(internalSignal_value);
	  }

	  if(internalSignal::INTERNAL_VolUpReq == Val.getInternaVariable_Enum()){
	internalSignal_value[4] = Val.getInternaVariable_Data();
	notifyInternalSignalUpdated(internalSignal_value);
}

	  if(internalSignal::INTERNAL_VolDownReq == Val.getInternaVariable_Enum()){
	internalSignal_value[5] = Val.getInternaVariable_Data();
	notifyInternalSignalUpdated(internalSignal_value);
	}

	  if(internalSignal::INTERNAL_MuteReq == Val.getInternaVariable_Enum()){
	internalSignal_value[6] = Val.getInternaVariable_Data();
	notifyInternalSignalUpdated(internalSignal_value);
	  }

	 if(internalSignal::INTERNAL_Ignition_Status == Val.getInternaVariable_Enum()){
	internalSignal_value[7] = Val.getInternaVariable_Data();
	notifyInternalSignalUpdated(internalSignal_value);
	  }

	  if(internalSignal::INTERNAL_CurrentPower_Mode == Val.getInternaVariable_Enum()){
	internalSignal_value[8] = Val.getInternaVariable_Data();
	notifyInternalSignalUpdated(internalSignal_value);}

	  if(internalSignal::INTERNAL_PrevPower_Mode == Val.getInternaVariable_Enum()){
	internalSignal_value[9] = Val.getInternaVariable_Data();
	notifyInternalSignalUpdated(internalSignal_value);}
}

		
void getInternalSigsubscribe(){

		 if (m_vehicle_proxy != nullptr) {
			 LOG(INFO) << "CAPIVehicleClientGateway inside Internal attribute call ";
			 m_vehicle_proxy->getInternalVariables_MCUAttribute().getChangedEvent().subscribe(
           std::bind(&VehicleClientImpl::onInternalVariablesAttributeChanged, this, std::placeholders::_1));
		 }
	   		else {
            LOG(INFO) << "CAPIVehicleClientGateway : Internal Data *** Proxy not available";
        }
 }
 void onSOCWAKEUPREASONAttributeChanged(iMicro::iMicroTypes::WAKEUP_REASON_enum val){
    LOG(INFO) << "The onSOCWAKEUPREASONAttributeChanged has changed";
    ::iMicro::iMicroTypes::WAKEUP_REASON_enum payload = val;
    LOG(INFO) << "Wakeup_reason:" << static_cast<int>(payload);
    notifyWakeupReason(payload);
   }

 void getSOC_Wakeup_reasonsubscribe(){
    if (m_vehicle_proxy != nullptr) {
        LOG(INFO) << "CAPIVehicleClientGateway inside getSOC_Wakeup_reasonsubscribe call ";
        m_vehicle_proxy->getSOC_WAKEUP_REASONAttribute().getChangedEvent().subscribe(std::bind(&VehicleClientImpl::onSOCWAKEUPREASONAttributeChanged, this, std::placeholders::_1));
    }
          else {
       LOG(INFO) << "CAPIVehicleClientGateway : getSOC_Wakeup_reasonsubscribe *** Proxy not available";
   }
}
void onEOLMarelliPNAttributeChanged(std::vector<uint8_t> MarelliPNVal)
{
    LOG(INFO) << "Inside onEOLMarelliPNAttributeChanged";
    for(auto i = 0u ; i < MarelliPNVal.size(); i++)
    {
        LOG(INFO) << "MarelliPNVal[" << i << "] : " << (uint32_t)MarelliPNVal[i];
    }
    notifyMarelliPN(MarelliPNVal);
}
void getMarelliPartNumbersubscribe(){
    if (m_vehicle_proxy != nullptr) {
        LOG(INFO) << "CAPIVehicleClientGateway inside getMarelliPartNumbersubscribe call ";
        m_vehicle_proxy->getE_EOL_Marelli_PNAttribute().getChangedEvent().subscribe(std::bind(&VehicleClientImpl::onEOLMarelliPNAttributeChanged, this, std::placeholders::_1));
    }
          else {
       LOG(INFO) << "CAPIVehicleClientGateway : getMarelliPartNumbersubscribe *** Proxy not available";
   }
}
void onEOLHWIDAttributeChanged(std::vector<uint8_t> EOLHWID)
{
    LOG(INFO) << "Inside onEOLHWIDAttributeChanged";
    for(auto i = 0u ; i < EOLHWID.size(); i++)
    {
        LOG(INFO) << "EOLHWID[" << i << "] : " << EOLHWID[i];
    }
    notifyEOLHWID(EOLHWID);
}
void getEOL_hw_variantsubscribe(){
    if (m_vehicle_proxy != nullptr) {
        LOG(INFO) << "CAPIVehicleClientGateway inside getEOL_hw_variantsubscribe call ";
        m_vehicle_proxy->getE_EOL_Variant_IDAttribute().getChangedEvent().subscribe(std::bind(&VehicleClientImpl::onEOLHWIDAttributeChanged, this, std::placeholders::_1));
    }
          else {
       LOG(INFO) << "CAPIVehicleClientGateway : getEOL_hw_variantsubscribe *** Proxy not available";
   }
}
 
 Cal_Union Cal_Union_Parameters;
 Learning_Data_Union Learn_Union;
 

 void oncalibGetRawEventReceived(CommonAPI::ByteBuffer buffer) {
	int32_t m_Calibration_data[256];
    std::vector <uint8_t> calData_arr_IOT;
    std::vector <uint8_t> calData_arr_SBS_URI;
    std::vector <uint8_t> calData_arr_ECU_BT;
    std::vector <uint8_t> calData_arr_ECALL1;
    std::vector <uint8_t> calData_arr_ECALL2;



   std::memcpy(m_Calibration_data, buffer.data(), buffer.size());
   uint8_t buf_count_val = (buffer[0] | buffer[1]);
    LOG(INFO) << "CAPIVehicleClientGateway : buffer counter value" << (uint32_t)buf_count_val;
    LOG(INFO) << "CAPIVehicleClientGateway : buffer size()->" << (uint32_t)buffer.size();

    if (buf_count_val == 0 && flag == 0) {
        flag = 1;
    }

    if (flag == 1) {
        if (kx < 5239) {
            for (unsigned j = 4; j < buffer.size(); j++) {
                Cal_Union_Parameters.customer_calibration_data[kx] = buffer[j];
                //srecFile_calibData[kx] = buffer[j];
                /*LOG(INFO) << "CAPIVehicleClientGateway :in loop Cal_Union_Parameters.customer_calibration_data-> "
                          << std::dec << (uint32_t)j
                          << " kx " << (uint32_t)kx
                          << " 0x" << std::hex << (uint32_t)buffer[j] << " 0x" << std::hex << (uint32_t)Cal_Union_Parameters.customer_calibration_data[kx];*/
                ++kx;
                if (kx == 5239)
                    break;
            }
        }
        // Notify function call
      //  notifyCalDataforSrec_Calibration(srecFile_calibData);
 if(buf_count_val == 21 && lx < 123){
    LOG(INFO) << "Filling Learning Data";
  for(unsigned m=4;m < buffer.size();m++){
    Learn_Union.Learned_Data[lx] = buffer[m];
	/*LOG(INFO) << "CAPIVehicleClientGateway :in loop Learn_Union.Learned_Data-> " << (uint32_t)m <<"lx  "<< (uint32_t)lx <<" "<<(uint32_t)buffer[m];*/
	++lx;
	if(lx==123)
	break;
 }
 }
   LOG(INFO) << "CAPIVehicleClientGateway : static value K->" <<(uint32_t)kx;
   LOG(INFO) << "CAPIVehicleClientGateway : static value K->" <<(uint32_t)lx;

   /*for(unsigned i =0;i < kx;i++){
  LOG(INFO) << "CAPIVehicleClientGateway : Calibration Data Array" << (uint32_t)i <<" " <<(uint32_t)Cal_Union_Parameters.customer_calibration_data[i]<<" ";
}*/

/*for(unsigned i =0;i < lx;i++){
  LOG(INFO) << "CAPIVehicleClientGateway : Learning data Array" << (uint32_t)i <<" " <<(uint32_t)Learn_Union.Learned_Data[i]<<" ";
}*/
/*  for(unsigned i=0;i < 249;i++)
  {
    ALOGD("IOT_HUB_ENDPOINT: %d %d ",i,Cal_Union_Parameters.Cal_Parameter.IOT_HUB_ENDPOINT[i]);
  } */

int i = 0;
for(auto variable : Cal_Union_Parameters.Cal_Parameter.IOT_HUB_ENDPOINT){
    calData_arr_IOT.push_back(variable);
    LOG(INFO) << "IOT_HUB_ENDPOINT : calData_arr_IOT data index " << i << " " << static_cast<int>(variable);
    i++;
}

  calData_arr_IOT.insert(calData_arr_IOT.begin(), 1);
  notifyCalDataUpdated_Array(calData_arr_IOT);
  calData_arr_IOT.clear();
  
  int k = 0;
  for(auto variable1 : Cal_Union_Parameters.Cal_Parameter.SBS_URI)
  {
    calData_arr_SBS_URI.push_back(variable1);
    LOG(INFO) << "SBS_URI:calData_arr data index " << k << " " << static_cast<int>(variable1);
    k++;
  }
  LOG(INFO) << "calData_arr_SBS_URI.size() = " <<calData_arr_SBS_URI.size();
  calData_arr_SBS_URI.insert(calData_arr_SBS_URI.begin(),2);
  notifyCalDataUpdated_Array(calData_arr_SBS_URI);
  calData_arr_SBS_URI.clear();

  int l = 0;
  for(auto variable2 : Cal_Union_Parameters.Cal_Parameter.ECU_BT_FRIENDLY_NAME)
  {
    calData_arr_ECU_BT.push_back(variable2);
    LOG(INFO) << "ECU_BT_FRIENDLY_NAME:calData_arr data index " << l << " " << static_cast<int>(variable2);
    l++;
  }
  LOG(INFO) << "calData_arr_ECU_BT.size() = " <<calData_arr_ECU_BT.size();
  calData_arr_ECU_BT.insert(calData_arr_ECU_BT.begin(),3);
  notifyCalDataUpdated_Array(calData_arr_ECU_BT);
  calData_arr_ECU_BT.clear();

  int m = 0;
  for(auto variable3 : Cal_Union_Parameters.Cal_Parameter.ECALL_TEST_NUMBER)
  {
    calData_arr_ECALL1.push_back(variable3);
    LOG(INFO) << "calData_arr_ECALL1:calData_arr data index " << m << " " << static_cast<int>(variable3);
    m++;
  }
  LOG(INFO) << "calData_arr_ECALL1.size() = " <<calData_arr_ECALL1.size();
  calData_arr_ECALL1.insert(calData_arr_ECALL1.begin(),4);
  notifyCalDataUpdated_Array(calData_arr_ECALL1);
  calData_arr_ECALL1.clear();

  int n = 0;
  for(auto variable4 : Cal_Union_Parameters.Cal_Parameter.ECALL_TEST_NUMBER_2)
  {
    calData_arr_ECALL2.push_back(variable4);
    LOG(INFO) << "calData_arr_ECALL1:calData_arr data index " << n << " " << static_cast<int>(variable4);
    n++;
  }
  LOG(INFO) << "calData_arr_ECALL2.size() = " <<calData_arr_ECALL2.size();
  calData_arr_ECALL2.insert(calData_arr_ECALL2.begin(),5);
  notifyCalDataUpdated_Array(calData_arr_ECALL2);
  calData_arr_ECALL2.clear();
  

  calData_display_All[0] = Cal_Union_Parameters.Cal_Parameter.MAX_RETRY_COUNT;
  calData_display_All[1] = Cal_Union_Parameters.Cal_Parameter.BASE_CONNECTION_RETRY_TIME;
  calData_display_All[2] = Cal_Union_Parameters.Cal_Parameter.MQTT_KEEP_ALIVE_TIME;
  calData_display_All[3] = Cal_Union_Parameters.Cal_Parameter.COAP_EXCHANGE_LIFETIME;
  calData_display_All[4] = Cal_Union_Parameters.Cal_Parameter.CRASH_DETECTION_TIMER;
  calData_display_All[5] = Cal_Union_Parameters.Cal_Parameter.CLIMATE_T_SEND;
  calData_display_All[6] = Cal_Union_Parameters.Cal_Parameter.CLIMATE_T_DISPLAY;
  calData_display_All[7] = Cal_Union_Parameters.Cal_Parameter.SWS1_TH_PRESS_NONE;
  calData_display_All[8] = Cal_Union_Parameters.Cal_Parameter.SWS1_TH_PRESS_B1;
  calData_display_All[9] = Cal_Union_Parameters.Cal_Parameter.SWS1_TH_PRESS_B2;
  calData_display_All[10] = Cal_Union_Parameters.Cal_Parameter.SWS1_TH_PRESS_B3;
  calData_display_All[11] = Cal_Union_Parameters.Cal_Parameter.SWS1_TH_PRESS_B4;
  calData_display_All[12] = Cal_Union_Parameters.Cal_Parameter.SWS1_TH_PRESS_B5;
  calData_display_All[13] = Cal_Union_Parameters.Cal_Parameter.SWS1_TH_PRESS_B6;
  calData_display_All[14] = Cal_Union_Parameters.Cal_Parameter.SWS1_TH_PRESS_B7;
  calData_display_All[15] = Cal_Union_Parameters.Cal_Parameter.SWS2_TH_PRESS_NONE;
  calData_display_All[16] = Cal_Union_Parameters.Cal_Parameter.SWS2_TH_PRESS_B1;
  calData_display_All[17] = Cal_Union_Parameters.Cal_Parameter.SWS2_TH_PRESS_B2;
  calData_display_All[18] = Cal_Union_Parameters.Cal_Parameter.SWS2_TH_PRESS_B3;
  calData_display_All[19] = Cal_Union_Parameters.Cal_Parameter.SWS2_TH_PRESS_B4;
  calData_display_All[20] = Cal_Union_Parameters.Cal_Parameter.SWS2_TH_PRESS_B5;
  calData_display_All[21] = Cal_Union_Parameters.Cal_Parameter.SWS2_TH_PRESS_B6;
  calData_display_All[22] = Cal_Union_Parameters.Cal_Parameter.SWS2_TH_PRESS_B7;
  calData_display_All[23] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_0;
  calData_display_All[24] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_1;
  calData_display_All[25] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_2;
  calData_display_All[26] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_3;
  calData_display_All[27] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_4;
  calData_display_All[28] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_5;
  calData_display_All[29] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_6;
  calData_display_All[30] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_7;
  calData_display_All[31] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_8;
  calData_display_All[32] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_9;
  calData_display_All[33] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_10;
  calData_display_All[34] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_11;
  calData_display_All[35] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_12;
  calData_display_All[36] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_13;
  calData_display_All[37] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_14;
  calData_display_All[38] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_DAY_BRIGHTNESS_STEP_15;
  calData_display_All[39] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_0;
  calData_display_All[40] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_1;
  calData_display_All[41] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_2;
  calData_display_All[42] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_3;
  calData_display_All[43] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_4;
  calData_display_All[44] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_5;
  calData_display_All[45] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_6;
  calData_display_All[46] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_7;
  calData_display_All[47] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_8;
  calData_display_All[48] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_9;
  calData_display_All[49] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_10;
  calData_display_All[50] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_11;
  calData_display_All[51] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_12;
  calData_display_All[52] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_13;
  calData_display_All[53] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_14;
  calData_display_All[54] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_NIGHT_BRIGHTNESS_STEP_15;
  calData_display_All[55] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_0;
  calData_display_All[56] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_1;
  calData_display_All[57] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_2;
  calData_display_All[58] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_3;
  calData_display_All[59] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_4;
  calData_display_All[60] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_5;
  calData_display_All[61] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_6;
  calData_display_All[62] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_7;
  calData_display_All[63] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_8;
  calData_display_All[64] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_9;
  calData_display_All[65] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_10;
  calData_display_All[66] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_11;
  calData_display_All[67] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_12;
  calData_display_All[68] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_13;
  calData_display_All[69] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_14;
  calData_display_All[70] = Cal_Union_Parameters.Cal_Parameter.DISPLAY_SCREEN_LOCK_BRIGHTNESS_STEP_15;
  calData_display_All[71] = Cal_Union_Parameters.Cal_Parameter.MQTT_CLEAN_SESSION;
  calData_display_All[72] = Cal_Union_Parameters.Cal_Parameter.PRESSURE_UNITS_ENABLED;
  calData_display_All[73] = Cal_Union_Parameters.Cal_Parameter.VOLUME_UNITS_ENABLED;
  calData_display_All[74] = Cal_Union_Parameters.Cal_Parameter.POWER_UNITS_ENABLED;
  calData_display_All[75] = Cal_Union_Parameters.Cal_Parameter.TORQUE_UNITS_ENABLED;
  calData_display_All[76] = Cal_Union_Parameters.Cal_Parameter.LVDS_TOUCH_NO_VALUE_PULL_TIME;
  calData_display_All[77] = Cal_Union_Parameters.Cal_Parameter.LVDS_TOUCH_VALUE_PULL_TIME;
  calData_display_All[78] = Cal_Union_Parameters.Cal_Parameter.SWC_LONG_PRESS_TH;
  calData_display_All[79] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_0;
  calData_display_All[80] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_1;
  calData_display_All[81] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_2;
  calData_display_All[82] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_3;
  calData_display_All[83] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_4;
  calData_display_All[84] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_5;
  calData_display_All[85] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_6;
  calData_display_All[86] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_7;
  calData_display_All[87] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_8;
  calData_display_All[88] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_9;
  calData_display_All[89] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_10;
  calData_display_All[90] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_11;
  calData_display_All[91] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_12;
  calData_display_All[92] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_13;
  calData_display_All[93] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_14;
  calData_display_All[94] = Cal_Union_Parameters.Cal_Parameter.DISP_BRIGHTNESS_DELAY_STEP_15;
  calData_display_All[95] = Cal_Union_Parameters.Cal_Parameter.FUEL_TANK_CAPACITY_SIVI;
  calData_display_All[96] = Cal_Union_Parameters.Cal_Parameter.TUNER_FM_SIGNAL_STRENGTH_THRESHOLD;
  calData_display_All[97] = Cal_Union_Parameters.Cal_Parameter.TUNER_AM_SIGNAL_STRENGTH_THRESHOLD;
  calData_display_All[98] = Cal_Union_Parameters.Cal_Parameter.SLI_CLIENT_STOP;
  calData_display_All[99] = Cal_Union_Parameters.Cal_Parameter.SUSPEND_RAM_DURATION;
  calData_display_All[100] = Cal_Union_Parameters.Cal_Parameter.LISTEN_MODE1_DURATION;
  calData_display_All[101] = Cal_Union_Parameters.Cal_Parameter.LISTEN_MODE2_DURATION;
  calData_display_All[102] = Cal_Union_Parameters.Cal_Parameter.LISTEN_MODE3_DURATION;
  calData_display_All[103] = Cal_Union_Parameters.Cal_Parameter.LISTEN_MODE_1_CONN_RETRY_TIME;
  calData_display_All[104] = Cal_Union_Parameters.Cal_Parameter.LISTEN_MODE_3_WAKE_UP_TIME;
  calData_display_All[105] = Cal_Union_Parameters.Cal_Parameter.SANITY_REBOOT_PERIOD;
  calData_display_All[106] = Cal_Union_Parameters.Cal_Parameter.MAX_CAL_DURATION;
  calData_display_All[107] = Cal_Union_Parameters.Cal_Parameter.TIMED_INACTIVITY_TIMEOUT;
  calData_display_All[108] = Cal_Union_Parameters.Cal_Parameter.SLEEP_TIMERS_RESET_DURATION;
  calData_display_All[109] = Cal_Union_Parameters.Cal_Parameter.STANDBY_SCREEN_MODE;
  calData_display_All[110] = Cal_Union_Parameters.Cal_Parameter.PARTIAL_OP_SCREEN_MODE;
  calData_display_All[111] = Cal_Union_Parameters.Cal_Parameter.BUB_DURATION;
  calData_display_All[112] = Cal_Union_Parameters.Cal_Parameter.MAINTENANCE_VEL_THRESHOLD;
  calData_display_All[113] = Cal_Union_Parameters.Cal_Parameter.VEHICLE_WEIGHT;
  calData_display_All[114] = Cal_Union_Parameters.Cal_Parameter.WHEEL_DIAMETER;
  calData_display_All[115] = Cal_Union_Parameters.Cal_Parameter.VEHICLE_WIDTH;
  calData_display_All[116] = Cal_Union_Parameters.Cal_Parameter.TICKS_REVOLUTION;
  calData_display_All[117] = Cal_Union_Parameters.Cal_Parameter.BUB_BATTERY_THRESHOLD;
  calData_display_All[118] = Cal_Union_Parameters.Cal_Parameter.HV_BATTERY_INITIAL_CAPACITY;
  calData_display_All[119] = Cal_Union_Parameters.Cal_Parameter.CAN2_ENABLE;
  calData_display_All[120] = Cal_Union_Parameters.Cal_Parameter.SANITY_REBOOT_DELAY;
  calData_display_All[121] = Cal_Union_Parameters.Cal_Parameter.MIRRORING_EV_EPISLON;
  calData_display_All[122] = Cal_Union_Parameters.Cal_Parameter.EV_ROUTING_CARPLAY;
  calData_display_All[123] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_1_WH_PER_KM;
  calData_display_All[124] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_2_WH_PER_KM;
  calData_display_All[125] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_3_WH_PER_KM;
  calData_display_All[126] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_4_WH_PER_KM;
  calData_display_All[127] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_5_WH_PER_KM;
  calData_display_All[128] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_6_WH_PER_KM;
  calData_display_All[129] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_7_WH_PER_KM;
  calData_display_All[130] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_8_WH_PER_KM;
  calData_display_All[131] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_9_WH_PER_KM;
  calData_display_All[132] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_10_WH_PER_KM;
  calData_display_All[133] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_11_WH_PER_KM;
  calData_display_All[134] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_12_WH_PER_KM;
  calData_display_All[135] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_13_WH_PER_KM;
  calData_display_All[136] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_14_WH_PER_KM;
  calData_display_All[137] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_15_WH_PER_KM;
  calData_display_All[138] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_16_WH_PER_KM;
  calData_display_All[139] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_17_WH_PER_KM;
  calData_display_All[140] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_18_WH_PER_KM;
  calData_display_All[141] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_19_WH_PER_KM;
  calData_display_All[142] = Cal_Union_Parameters.Cal_Parameter.SPEED_CONS_20_WH_PER_KM;

    calData_display_All[143] = Cal_Union_Parameters.Cal_Parameter.SEARCH_FORWARD_ACCELERATION_MULTIPLIER;
    calData_display_All[144] = Cal_Union_Parameters.Cal_Parameter.SEARCH_BACKWARD_ACCELERATION_MULTIPLIER;
    calData_display_All[145] = Cal_Union_Parameters.Cal_Parameter.HMI_UPDATE_CHARGE_INFO;
    calData_display_All[146] = Cal_Union_Parameters.Cal_Parameter.BUB_VEOC;
    calData_display_All[147] = Cal_Union_Parameters.Cal_Parameter.BUB_VEC;
    calData_display_All[148] = Cal_Union_Parameters.Cal_Parameter.BUB_VC_MIN;
    calData_display_All[149] = Cal_Union_Parameters.Cal_Parameter.BUB_VC_MIN_U_REG;
    calData_display_All[150] = Cal_Union_Parameters.Cal_Parameter.BUB_VC_MIN_U_SMT;
    calData_display_All[151] = Cal_Union_Parameters.Cal_Parameter.BUB_T_CHRG_MIN;
    calData_display_All[152] = Cal_Union_Parameters.Cal_Parameter.BUB_T_CHRG_MAX;
    calData_display_All[153] = Cal_Union_Parameters.Cal_Parameter.BUB_MAX_CHRG_TIME;
    calData_display_All[154] = Cal_Union_Parameters.Cal_Parameter.BUB_CS_VSC;
    calData_display_All[155] = Cal_Union_Parameters.Cal_Parameter.BUB_CS_VLC;
    calData_display_All[156] = Cal_Union_Parameters.Cal_Parameter.BUB_CS_VEC;
    calData_display_All[157] = Cal_Union_Parameters.Cal_Parameter.TEMP_VUC_TU;
    calData_display_All[158] = Cal_Union_Parameters.Cal_Parameter.TEMP_VUC_TC;
    calData_display_All[159] = Cal_Union_Parameters.Cal_Parameter.TEMP_VUC_T0;
    calData_display_All[160] = Cal_Union_Parameters.Cal_Parameter.TEMP_VUC_T1;
    calData_display_All[161] = Cal_Union_Parameters.Cal_Parameter.TEMP_VUC_T2;
    calData_display_All[162] = Cal_Union_Parameters.Cal_Parameter.TEMP_VUC_T3;
    calData_display_All[163] = Cal_Union_Parameters.Cal_Parameter.TEMP_MIF_TU;
    calData_display_All[164] = Cal_Union_Parameters.Cal_Parameter.TEMP_MIF_TC;
    calData_display_All[165] = Cal_Union_Parameters.Cal_Parameter.TEMP_MIF_T0;
    calData_display_All[166] = Cal_Union_Parameters.Cal_Parameter.TEMP_MIF_T1;
    calData_display_All[167] = Cal_Union_Parameters.Cal_Parameter.TEMP_MIF_T2;
    calData_display_All[168] = Cal_Union_Parameters.Cal_Parameter.TEMP_MIF_T3;
    calData_display_All[169] = Cal_Union_Parameters.Cal_Parameter.TEMP_BUB_TU;
    calData_display_All[170] = Cal_Union_Parameters.Cal_Parameter.TEMP_BUB_TC;
    calData_display_All[171] = Cal_Union_Parameters.Cal_Parameter.TEMP_BUB_T0;
    calData_display_All[172] = Cal_Union_Parameters.Cal_Parameter.TEMP_BUB_T1;
    calData_display_All[173] = Cal_Union_Parameters.Cal_Parameter.TEMP_BUB_T2;
    calData_display_All[174] = Cal_Union_Parameters.Cal_Parameter.TEMP_BUB_T3;
    calData_display_All[175] = Cal_Union_Parameters.Cal_Parameter.TEMP_HYSTERESIS;
    calData_display_All[176] = Cal_Union_Parameters.Cal_Parameter.HW_IGN_MODE;
    calData_display_All[177] = Cal_Union_Parameters.Cal_Parameter.VEHICLE_TRACK;
    calData_display_All[178] = Cal_Union_Parameters.Cal_Parameter.GNSS_ANTENNA_POSE_X; // Please note: GNSS parameters are often uint32, so ensure proper size handling
    calData_display_All[179] = Cal_Union_Parameters.Cal_Parameter.GNSS_ANTENNA_POSE_Y;
    calData_display_All[180] = Cal_Union_Parameters.Cal_Parameter.GNSS_ANTENNA_POSE_Z;
    calData_display_All[181] = Cal_Union_Parameters.Cal_Parameter.GNSS_ANTENNA_POSE_ROLL;
    calData_display_All[182] = Cal_Union_Parameters.Cal_Parameter.GNSS_ANTENNA_POSE_PITCH;
    calData_display_All[183] = Cal_Union_Parameters.Cal_Parameter.GNSS_ANTENNA_POSE_YAW;
    calData_display_All[184] = Cal_Union_Parameters.Cal_Parameter.REDUCER_GEAR_CONFIG;
    calData_display_All[185] = Cal_Union_Parameters.Cal_Parameter.CVT_GEAR_CONFIG;
    calData_display_All[186] = Cal_Union_Parameters.Cal_Parameter.MANUAL_AUTO_GEAR_CONFIG;
    calData_display_All[187] = Cal_Union_Parameters.Cal_Parameter.AT6_GEAR_CONFIG;
    calData_display_All[188] = Cal_Union_Parameters.Cal_Parameter.AT8_GEAR_CONFIG;
    calData_display_All[189] = Cal_Union_Parameters.Cal_Parameter.AT_GEAR_SIM_MON;
    calData_display_All[190] = Cal_Union_Parameters.Cal_Parameter.VCU_GEAR_DEF_VALUE;
    calData_display_All[191] = Cal_Union_Parameters.Cal_Parameter.VEH_GEAR_SNA_COUNTER;
    calData_display_All[192] = Cal_Union_Parameters.Cal_Parameter.AT_INTER_REV_GEAR_POS;
    calData_display_All[193] = Cal_Union_Parameters.Cal_Parameter.AT_INTER_PARK_GEAR_POS;
    calData_display_All[194] = Cal_Union_Parameters.Cal_Parameter.INIT_STATE_REVERSE_GEAR_ENGAGED;
    calData_display_All[195] = Cal_Union_Parameters.Cal_Parameter.INIT_STATE_SHIFT_LEVER_POSITION;
    calData_display_All[196] = Cal_Union_Parameters.Cal_Parameter.INIT_STATE_TRANSM_GEAR_DISPLAY;
    calData_display_All[197] = Cal_Union_Parameters.Cal_Parameter.INIT_STATE_GEAR_POSITION;
    calData_display_All[198] = Cal_Union_Parameters.Cal_Parameter.INIT_STATE_STATUS_PARK_BREAK;
    calData_display_All[199] = Cal_Union_Parameters.Cal_Parameter.PARK_BREAK_SNA_COUNTER;
    calData_display_All[200] = Cal_Union_Parameters.Cal_Parameter.RPAS_REQUIRE_REV_GEAR;
    calData_display_All[201] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_MUTE_BTN_TIMEOUT;
    calData_display_All[202] = Cal_Union_Parameters.Cal_Parameter.FPAS_REQUIRE_ACTIVE_BAR;
    calData_display_All[203] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIS_INHIBIT_SHOWROOM;
    calData_display_All[204] = Cal_Union_Parameters.Cal_Parameter.REVERSE_GEAR_SIGNAL_TYPE;
    calData_display_All[205] = Cal_Union_Parameters.Cal_Parameter.COUNTRY_OVERRIDE_KM;
    calData_display_All[206] = Cal_Union_Parameters.Cal_Parameter.UPDATE_WATCHDOG_TIMEOUT;
    calData_display_All[207] = Cal_Union_Parameters.Cal_Parameter.MAX_INSTALLATION_DURATION;
    calData_display_All[208] = Cal_Union_Parameters.Cal_Parameter.PRIVACY_SETTING_DEFAULT_EUROPE;
    calData_display_All[209] = Cal_Union_Parameters.Cal_Parameter.PRIVACY_SETTING_DEFAULT_EURASIA;
    calData_display_All[210] = Cal_Union_Parameters.Cal_Parameter.PRIVACY_SETTING_DEFAULT_SA;
    calData_display_All[211] = Cal_Union_Parameters.Cal_Parameter.PRIVACY_SETTING_DEFAULT_MEA;
    calData_display_All[212] = Cal_Union_Parameters.Cal_Parameter.PRIVACY_SETTING_DEFAULT_IAP;
    calData_display_All[213] = Cal_Union_Parameters.Cal_Parameter.PRIVACY_SETTING_DEFAULT_CH;

  notifyCalDataUpdated(calData_display_All);
  
    calData_Audio[0] = Cal_Union_Parameters.Cal_Parameter.EQ_NAV_PROMPT_ATT;
    calData_Audio[1] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_PERCENT;
    calData_Audio[2] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_DET_TIME;
    calData_Audio[3] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_ATT_RATE;
    calData_Audio[4] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_REL_RATE;
    calData_Audio[5] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_FL_ATT_TIME;
    calData_Audio[6] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_FL_REL_TIME;
    calData_Audio[7] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_FR_ATT_TIME;
    calData_Audio[8] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_FR_REL_TIME;
    calData_Audio[9] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_RL_ATT_TIME;
    calData_Audio[10] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_RL_REL_TIME;
    calData_Audio[11] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_RR_ATT_TIME;
    calData_Audio[12] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_RR_REL_TIME;
    calData_Audio[13] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_XCALL_ATT_TIME;
    calData_Audio[14] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_XCALL_REL_TIME;
    calData_Audio[15] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_MIC1_ENABLE;
    calData_Audio[16] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_MIC2_ENABLE;
    calData_Audio[17] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_MIC1_ATT_TIME;
    calData_Audio[18] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_MIC1_REL_TIME;
    calData_Audio[19] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_MIC2_ATT_TIME;
    calData_Audio[20] = Cal_Union_Parameters.Cal_Parameter.EQ_DIST_LIM_MIC2_REL_TIME;
    calData_Audio[21] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_1_TYPE;
    calData_Audio[22] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_1_FREQ;
    calData_Audio[23] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_1_GAIN;
    calData_Audio[24] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_1_Q_FACTOR;
    calData_Audio[25] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_2_TYPE;
    calData_Audio[26] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_2_FREQ;
    calData_Audio[27] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_2_GAIN;
    calData_Audio[28] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_2_Q_FACTOR;
    calData_Audio[29] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_3_TYPE;
    calData_Audio[30] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_3_FREQ;
    calData_Audio[31] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_3_GAIN;
    calData_Audio[32] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_3_Q_FACTOR;
    calData_Audio[33] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_4_TYPE;
    calData_Audio[34] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_4_FREQ;
    calData_Audio[35] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_4_GAIN;
    calData_Audio[36] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_4_Q_FACTOR;
    calData_Audio[37] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_5_TYPE;
    calData_Audio[38] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_5_FREQ;
    calData_Audio[39] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_5_GAIN;
    calData_Audio[40] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_5_Q_FACTOR;
    calData_Audio[41] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_6_TYPE;
    calData_Audio[42] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_6_FREQ;
    calData_Audio[43] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_6_GAIN;
    calData_Audio[44] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_PSU_6_Q_FACTOR;
    calData_Audio[45] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_1_TYPE;
    calData_Audio[46] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_1_FREQ;
    calData_Audio[47] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_1_GAIN;
    calData_Audio[48] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_1_Q_FACTOR;
    calData_Audio[49] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_2_TYPE;
    calData_Audio[50] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_2_FREQ;
    calData_Audio[51] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_2_GAIN;
    calData_Audio[52] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_2_Q_FACTOR;
    calData_Audio[53] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_3_TYPE;
    calData_Audio[54] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_3_FREQ;
    calData_Audio[55] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_3_GAIN;
    calData_Audio[56] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_3_Q_FACTOR;
    calData_Audio[57] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_4_TYPE;
    calData_Audio[58] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_4_FREQ;
    calData_Audio[59] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_4_GAIN;
    calData_Audio[60] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_4_Q_FACTOR;
    calData_Audio[61] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_5_TYPE;
    calData_Audio[62] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_5_FREQ;
    calData_Audio[63] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_5_GAIN;
    calData_Audio[64] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_5_Q_FACTOR;
    calData_Audio[65] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_6_TYPE;
    calData_Audio[66] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_6_FREQ;
    calData_Audio[67] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_6_GAIN;
    calData_Audio[68] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_PSU_6_Q_FACTOR;
    calData_Audio[69] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_1_TYPE;
    calData_Audio[70] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_1_FREQ;
    calData_Audio[71] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_1_GAIN;
    calData_Audio[72] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_1_Q_FACTOR;
    calData_Audio[73] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_2_TYPE;
    calData_Audio[74] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_2_FREQ;
    calData_Audio[75] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_2_GAIN;
    calData_Audio[76] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_2_Q_FACTOR;
    calData_Audio[77] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_3_TYPE;
    calData_Audio[78] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_3_FREQ;
    calData_Audio[79] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_3_GAIN;
    calData_Audio[80] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_3_Q_FACTOR;
    calData_Audio[81] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_4_TYPE;
    calData_Audio[82] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_4_FREQ;
    calData_Audio[83] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_4_GAIN;
    calData_Audio[84] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_4_Q_FACTOR;
    calData_Audio[85] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_5_TYPE;
    calData_Audio[86] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_5_FREQ;
    calData_Audio[87] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_5_GAIN;
    calData_Audio[88] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_5_Q_FACTOR;
    calData_Audio[89] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_6_TYPE;
    calData_Audio[90] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_6_FREQ;
    calData_Audio[91] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_6_GAIN;
    calData_Audio[92] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_VEH_BKP_6_Q_FACTOR;
    calData_Audio[93] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_1_TYPE;
    calData_Audio[94] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_1_FREQ;
    calData_Audio[95] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_1_GAIN;
    calData_Audio[96] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_1_Q_FACTOR;
    calData_Audio[97] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_2_TYPE;
    calData_Audio[98] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_2_FREQ;
    calData_Audio[99] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_2_GAIN;
    calData_Audio[100] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_2_Q_FACTOR;
    calData_Audio[101] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_3_TYPE;
    calData_Audio[102] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_3_FREQ;
    calData_Audio[103] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_3_GAIN;
    calData_Audio[104] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_3_Q_FACTOR;
    calData_Audio[105] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_4_TYPE;
    calData_Audio[106] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_4_FREQ;
    calData_Audio[107] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_4_GAIN;
    calData_Audio[108] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_4_Q_FACTOR;
    calData_Audio[109] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_5_TYPE;
    calData_Audio[110] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_5_FREQ;
    calData_Audio[111] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_5_GAIN;
    calData_Audio[112] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_5_Q_FACTOR;
    calData_Audio[113] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_6_TYPE;
    calData_Audio[114] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_6_FREQ;
    calData_Audio[115] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_6_GAIN;
    calData_Audio[116] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_XCALL_DED_BKP_6_Q_FACTOR;
    calData_Audio[117] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_TYPE;
    calData_Audio[118] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_FREQ;
    calData_Audio[119] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_Q_FACTOR;
    calData_Audio[120] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_00_GAIN;
    calData_Audio[121] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_01_GAIN;
    calData_Audio[122] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_02_GAIN;
    calData_Audio[123] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_03_GAIN;
    calData_Audio[124] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_04_GAIN;
    calData_Audio[125] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_05_GAIN;
    calData_Audio[126] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_06_GAIN;
    calData_Audio[127] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_07_GAIN;
    calData_Audio[128] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_08_GAIN;
    calData_Audio[129] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_09_GAIN;
    calData_Audio[130] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_10_GAIN;
    calData_Audio[131] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_11_GAIN;
    calData_Audio[132] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_12_GAIN;
    calData_Audio[133] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_13_GAIN;
    calData_Audio[134] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_14_GAIN;
    calData_Audio[135] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_15_GAIN;
    calData_Audio[136] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_16_GAIN;
    calData_Audio[137] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_17_GAIN;
    calData_Audio[138] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_18_GAIN;
    calData_Audio[139] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_19_GAIN;
    calData_Audio[140] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_20_GAIN;
    calData_Audio[141] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_21_GAIN;
    calData_Audio[142] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_22_GAIN;
    calData_Audio[143] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_23_GAIN;
    calData_Audio[144] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_24_GAIN;
    calData_Audio[145] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_25_GAIN;
    calData_Audio[146] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_26_GAIN;
    calData_Audio[147] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_27_GAIN;
    calData_Audio[148] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_28_GAIN;
    calData_Audio[149] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_29_GAIN;
    calData_Audio[150] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_30_GAIN;
    calData_Audio[151] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_31_GAIN;
    calData_Audio[152] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_32_GAIN;
    calData_Audio[153] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_33_GAIN;
    calData_Audio[154] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_34_GAIN;
    calData_Audio[155] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_35_GAIN;
    calData_Audio[156] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_36_GAIN;
    calData_Audio[157] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_37_GAIN;
    calData_Audio[158] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_1_STEP_38_GAIN;
    calData_Audio[159] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_TYPE;
    calData_Audio[160] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_FREQ;
    calData_Audio[161] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_Q_FACTOR;
    calData_Audio[162] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_00_GAIN;
    calData_Audio[163] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_01_GAIN;
    calData_Audio[164] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_02_GAIN;
    calData_Audio[165] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_03_GAIN;
    calData_Audio[166] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_04_GAIN;
    calData_Audio[167] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_05_GAIN;
    calData_Audio[168] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_06_GAIN;
    calData_Audio[169] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_07_GAIN;
    calData_Audio[170] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_08_GAIN;
    calData_Audio[171] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_09_GAIN;
    calData_Audio[172] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_10_GAIN;
    calData_Audio[173] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_11_GAIN;
    calData_Audio[174] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_12_GAIN;
    calData_Audio[175] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_13_GAIN;
    calData_Audio[176] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_14_GAIN;
    calData_Audio[177] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_15_GAIN;
    calData_Audio[178] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_16_GAIN;
    calData_Audio[179] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_17_GAIN;
    calData_Audio[180] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_18_GAIN;
    calData_Audio[181] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_19_GAIN;
    calData_Audio[182] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_20_GAIN;
    calData_Audio[183] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_21_GAIN;
    calData_Audio[184] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_22_GAIN;
    calData_Audio[185] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_23_GAIN;
    calData_Audio[186] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_24_GAIN;
    calData_Audio[187] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_25_GAIN;
    calData_Audio[188] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_26_GAIN;
    calData_Audio[189] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_27_GAIN;
    calData_Audio[190] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_28_GAIN;
    calData_Audio[191] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_29_GAIN;
    calData_Audio[192] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_30_GAIN;
    calData_Audio[193] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_31_GAIN;
    calData_Audio[194] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_32_GAIN;
    calData_Audio[195] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_33_GAIN;
    calData_Audio[196] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_34_GAIN;
    calData_Audio[197] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_35_GAIN;
    calData_Audio[198] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_36_GAIN;
    calData_Audio[199] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_37_GAIN;
    calData_Audio[200] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_LOUD_2_STEP_38_GAIN;
    calData_Audio[201] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_TYPE;
    calData_Audio[202] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_FREQ;
    calData_Audio[203] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_Q_FACTOR;
    calData_Audio[204] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_N09_GAIN;
    calData_Audio[205] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_N08_GAIN;
    calData_Audio[206] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_N07_GAIN;
    calData_Audio[207] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_N06_GAIN;
    calData_Audio[208] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_N05_GAIN;
    calData_Audio[209] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_N04_GAIN;
    calData_Audio[210] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_N03_GAIN;
    calData_Audio[211] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_N02_GAIN;
    calData_Audio[212] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_N01_GAIN;
    calData_Audio[213] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_00_GAIN;
    calData_Audio[214] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_P01_GAIN;
    calData_Audio[215] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_P02_GAIN;
    calData_Audio[216] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_P03_GAIN;
    calData_Audio[217] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_P04_GAIN;
    calData_Audio[218] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_P05_GAIN;
    calData_Audio[219] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_P06_GAIN;
    calData_Audio[220] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_P07_GAIN;
    calData_Audio[221] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_P08_GAIN;
    calData_Audio[222] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_1_STEP_P09_GAIN;
    calData_Audio[223] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_TYPE;
    calData_Audio[224] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_FREQ;
    calData_Audio[225] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_Q_FACTOR;
    calData_Audio[226] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_N09_GAIN;
    calData_Audio[227] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_N08_GAIN;
    calData_Audio[228] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_N07_GAIN;
    calData_Audio[229] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_N06_GAIN;
    calData_Audio[230] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_N05_GAIN;
    calData_Audio[231] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_N04_GAIN;
    calData_Audio[232] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_N03_GAIN;
    calData_Audio[233] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_N02_GAIN;
    calData_Audio[234] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_N01_GAIN;
    calData_Audio[235] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_00_GAIN;
    calData_Audio[236] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_P01_GAIN;
    calData_Audio[237] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_P02_GAIN;
    calData_Audio[238] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_P03_GAIN;
    calData_Audio[239] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_P04_GAIN;
    calData_Audio[240] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_P05_GAIN;
    calData_Audio[241] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_P06_GAIN;
    calData_Audio[242] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_P07_GAIN;
    calData_Audio[243] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_P08_GAIN;
    calData_Audio[244] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_2_STEP_P09_GAIN;
    calData_Audio[245] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_TYPE;
    calData_Audio[246] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_FREQ;
    calData_Audio[247] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_Q_FACTOR;
    calData_Audio[248] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_N09_GAIN;
    calData_Audio[249] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_N08_GAIN;
    calData_Audio[250] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_N07_GAIN;
    calData_Audio[251] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_N06_GAIN;
    calData_Audio[252] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_N05_GAIN;
    calData_Audio[253] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_N04_GAIN;
    calData_Audio[254] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_N03_GAIN;
    calData_Audio[255] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_N02_GAIN;
    calData_Audio[256] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_N01_GAIN;
    calData_Audio[257] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_00_GAIN;
    calData_Audio[258] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_P01_GAIN;
    calData_Audio[259] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_P02_GAIN;
    calData_Audio[260] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_P03_GAIN;
    calData_Audio[261] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_P04_GAIN;
    calData_Audio[262] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_P05_GAIN;
    calData_Audio[263] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_P06_GAIN;
    calData_Audio[264] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_P07_GAIN;
    calData_Audio[265] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_P08_GAIN;
    calData_Audio[266] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_1_STEP_P09_GAIN;
    calData_Audio[267] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_TYPE;
    calData_Audio[268] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_FREQ;
    calData_Audio[269] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_Q_FACTOR;
    calData_Audio[270] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_N09_GAIN;
    calData_Audio[271] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_N08_GAIN;
    calData_Audio[272] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_N07_GAIN;
    calData_Audio[273] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_N06_GAIN;
    calData_Audio[274] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_N05_GAIN;
    calData_Audio[275] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_N04_GAIN;
    calData_Audio[276] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_N03_GAIN;
    calData_Audio[277] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_N02_GAIN;
    calData_Audio[278] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_N01_GAIN;
    calData_Audio[279] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_00_GAIN;
    calData_Audio[280] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_P01_GAIN;
    calData_Audio[281] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_P02_GAIN;
    calData_Audio[282] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_P03_GAIN;
    calData_Audio[283] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_P04_GAIN;
    calData_Audio[284] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_P05_GAIN;
    calData_Audio[285] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_P06_GAIN;
    calData_Audio[286] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_P07_GAIN;
    calData_Audio[287] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_P08_GAIN;
    calData_Audio[288] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_2_STEP_P09_GAIN;
    calData_Audio[289] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_TYPE;
    calData_Audio[290] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_FREQ;
    calData_Audio[291] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_Q_FACTOR;
    calData_Audio[292] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_N09_GAIN;
    calData_Audio[293] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_N08_GAIN;
    calData_Audio[294] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_N07_GAIN;
    calData_Audio[295] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_N06_GAIN;
    calData_Audio[296] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_N05_GAIN;
    calData_Audio[297] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_N04_GAIN;
    calData_Audio[298] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_N03_GAIN;
    calData_Audio[299] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_N02_GAIN;
    calData_Audio[300] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_N01_GAIN;
    calData_Audio[301] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_00_GAIN;
    calData_Audio[302] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_P01_GAIN;
    calData_Audio[303] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_P02_GAIN;
    calData_Audio[304] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_P03_GAIN;
    calData_Audio[305] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_P04_GAIN;
    calData_Audio[306] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_P05_GAIN;
    calData_Audio[307] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_P06_GAIN;
    calData_Audio[308] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_P07_GAIN;
    calData_Audio[309] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_P08_GAIN;
    calData_Audio[310] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_1_STEP_P09_GAIN;
    calData_Audio[311] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_TYPE;
    calData_Audio[312] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_FREQ;
    calData_Audio[313] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_Q_FACTOR;
    calData_Audio[314] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_N09_GAIN;
    calData_Audio[315] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_N08_GAIN;
    calData_Audio[316] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_N07_GAIN;
    calData_Audio[317] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_N06_GAIN;
    calData_Audio[318] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_N05_GAIN;
    calData_Audio[319] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_N04_GAIN;
    calData_Audio[320] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_N03_GAIN;
    calData_Audio[321] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_N02_GAIN;
    calData_Audio[322] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_N01_GAIN;
    calData_Audio[323] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_00_GAIN;
    calData_Audio[324] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_P01_GAIN;
    calData_Audio[325] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_P02_GAIN;
    calData_Audio[326] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_P03_GAIN;
    calData_Audio[327] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_P04_GAIN;
    calData_Audio[328] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_P05_GAIN;
    calData_Audio[329] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_P06_GAIN;
    calData_Audio[330] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_P07_GAIN;
    calData_Audio[331] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_P08_GAIN;
    calData_Audio[332] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_2_STEP_P09_GAIN;
    calData_Audio[333] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_TYPE;
    calData_Audio[334] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_FREQ;
    calData_Audio[335] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_Q_FACTOR;
    calData_Audio[336] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_N09_GAIN;
    calData_Audio[337] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_N08_GAIN;
    calData_Audio[338] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_N07_GAIN;
    calData_Audio[339] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_N06_GAIN;
    calData_Audio[340] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_N05_GAIN;
    calData_Audio[341] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_N04_GAIN;
    calData_Audio[342] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_N03_GAIN;
    calData_Audio[343] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_N02_GAIN;
    calData_Audio[344] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_N01_GAIN;
    calData_Audio[345] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_00_GAIN;
    calData_Audio[346] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_P01_GAIN;
    calData_Audio[347] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_P02_GAIN;
    calData_Audio[348] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_P03_GAIN;
    calData_Audio[349] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_P04_GAIN;
    calData_Audio[350] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_P05_GAIN;
    calData_Audio[351] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_P06_GAIN;
    calData_Audio[352] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_P07_GAIN;
    calData_Audio[353] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_P08_GAIN;
    calData_Audio[354] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_1_STEP_P09_GAIN;
    calData_Audio[355] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_TYPE;
    calData_Audio[356] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_FREQ;
    calData_Audio[357] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_Q_FACTOR;
    calData_Audio[358] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_N09_GAIN;
    calData_Audio[359] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_N08_GAIN;
    calData_Audio[360] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_N07_GAIN;
    calData_Audio[361] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_N06_GAIN;
    calData_Audio[362] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_N05_GAIN;
    calData_Audio[363] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_N04_GAIN;
    calData_Audio[364] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_N03_GAIN;
    calData_Audio[365] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_N02_GAIN;
    calData_Audio[366] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_N01_GAIN;
    calData_Audio[367] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_00_GAIN;
    calData_Audio[368] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_P01_GAIN;
    calData_Audio[369] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_P02_GAIN;
    calData_Audio[370] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_P03_GAIN;
    calData_Audio[371] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_P04_GAIN;
    calData_Audio[372] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_P05_GAIN;
    calData_Audio[373] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_P06_GAIN;
    calData_Audio[374] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_P07_GAIN;
    calData_Audio[375] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_P08_GAIN;
    calData_Audio[376] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_SUB_2_STEP_P09_GAIN;
    calData_Audio[377] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_1_TYPE;
    calData_Audio[378] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_1_FREQ;
    calData_Audio[379] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_1_GAIN;
    calData_Audio[380] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_1_Q_FACTOR;
    calData_Audio[381] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_2_TYPE;
    calData_Audio[382] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_2_FREQ;
    calData_Audio[383] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_2_GAIN;
    calData_Audio[384] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_2_Q_FACTOR;
    calData_Audio[385] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_3_TYPE;
    calData_Audio[386] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_3_FREQ;
    calData_Audio[387] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_3_GAIN;
    calData_Audio[388] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_3_Q_FACTOR;
    calData_Audio[389] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_4_TYPE;
    calData_Audio[390] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_4_FREQ;
    calData_Audio[391] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_4_GAIN;
    calData_Audio[392] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_NAV_4_Q_FACTOR;
    calData_Audio[393] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_1_TYPE;
    calData_Audio[394] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_1_FREQ;
    calData_Audio[395] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_1_GAIN;
    calData_Audio[396] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_1_Q_FACTOR;
    calData_Audio[397] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_2_TYPE;
    calData_Audio[398] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_2_FREQ;
    calData_Audio[399] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_2_GAIN;
    calData_Audio[400] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_2_Q_FACTOR;
    calData_Audio[401] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_3_TYPE;
    calData_Audio[402] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_3_FREQ;
    calData_Audio[403] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_3_GAIN;
    calData_Audio[404] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_3_Q_FACTOR;
    calData_Audio[405] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_4_TYPE;
    calData_Audio[406] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_4_FREQ;
    calData_Audio[407] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_4_GAIN;
    calData_Audio[408] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_TTS_4_Q_FACTOR;
    calData_Audio[409] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_1_TYPE;
    calData_Audio[410] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_1_FREQ;
    calData_Audio[411] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_1_GAIN;
    calData_Audio[412] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_1_Q_FACTOR;
    calData_Audio[413] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_2_TYPE;
    calData_Audio[414] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_2_FREQ;
    calData_Audio[415] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_2_GAIN;
    calData_Audio[416] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_2_Q_FACTOR;
    calData_Audio[417] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_3_TYPE;
    calData_Audio[418] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_3_FREQ;
    calData_Audio[419] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_3_GAIN;
    calData_Audio[420] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_3_Q_FACTOR;
    calData_Audio[421] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_4_TYPE;
    calData_Audio[422] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_4_FREQ;
    calData_Audio[423] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_4_GAIN;
    calData_Audio[424] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO1_RES_4_Q_FACTOR;
    calData_Audio[425] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_1_TYPE;
    calData_Audio[426] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_1_FREQ;
    calData_Audio[427] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_1_GAIN;
    calData_Audio[428] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_1_Q_FACTOR;
    calData_Audio[429] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_2_TYPE;
    calData_Audio[430] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_2_FREQ;
    calData_Audio[431] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_2_GAIN;
    calData_Audio[432] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_2_Q_FACTOR;
    calData_Audio[433] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_3_TYPE;
    calData_Audio[434] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_3_FREQ;
    calData_Audio[435] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_3_GAIN;
    calData_Audio[436] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_3_Q_FACTOR;
    calData_Audio[437] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_4_TYPE;
    calData_Audio[438] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_4_FREQ;
    calData_Audio[439] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_4_GAIN;
    calData_Audio[440] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_INFO2_HFP_CALL_4_Q_FACTOR;
    calData_Audio[441] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_FL_POLARITY;
    calData_Audio[442] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_FL_OFFSET_GAIN;
    calData_Audio[443] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_FL_DELAY;
    calData_Audio[444] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_FR_POLARITY;
    calData_Audio[445] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_FR_OFFSET_GAIN;
    calData_Audio[446] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_FR_DELAY;
    calData_Audio[447] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_RL_POLARITY;
    calData_Audio[448] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_RL_OFFSET_GAIN;
    calData_Audio[449] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_RL_DELAY;
    calData_Audio[450] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_RR_POLARITY;
    calData_Audio[451] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_RR_OFFSET_GAIN;
    calData_Audio[452] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_RR_DELAY;
    calData_Audio[453] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_XCALL_POLARITY;
    calData_Audio[454] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_XCALL_OFFSET_GAIN;
    calData_Audio[455] = Cal_Union_Parameters.Cal_Parameter.EQ_CHN_XCALL_DELAY;
    calData_Audio[456] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_AM_OFFSET_GAIN;
    calData_Audio[457] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_FM_OFFSET_GAIN;
    calData_Audio[458] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_DAB_OFFSET_GAIN;
    calData_Audio[459] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_PROJ_OFFSET_GAIN;
    calData_Audio[460] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_BTSA_OFFSET_GAIN;
    calData_Audio[461] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_USB_OFFSET_GAIN;
    calData_Audio[462] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_NAV_OFFSET_GAIN;
    calData_Audio[463] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_TTS_OFFSET_GAIN;
    calData_Audio[464] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_XCALL_OFFSET_GAIN;
    calData_Audio[465] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_HFP_CALL_OFFSET_GAIN;
    calData_Audio[466] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_HFP_RING_OFFSET_GAIN;
    calData_Audio[467] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_CHM_OFFSET_GAIN;
    calData_Audio[468] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_ALERT_OFFSET_GAIN;
    calData_Audio[469] = Cal_Union_Parameters.Cal_Parameter.EQ_SRC_CONF_OFFSET_GAIN;
    calData_Audio[470] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_L1_GAIN;
    calData_Audio[471] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_L2_GAIN;
    calData_Audio[472] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_L3_GAIN;
    calData_Audio[473] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_L4_GAIN;
    calData_Audio[474] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_L5_GAIN;
    calData_Audio[475] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_L6_GAIN;
    calData_Audio[476] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_L7_GAIN;
    calData_Audio[477] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_L8_GAIN;
    calData_Audio[478] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_L9_GAIN;
    calData_Audio[479] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_L1_ATT;
    calData_Audio[480] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_L2_ATT;
    calData_Audio[481] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_L3_ATT;
    calData_Audio[482] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_L4_ATT;
    calData_Audio[483] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_L5_ATT;
    calData_Audio[484] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_L6_ATT;
    calData_Audio[485] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_L7_ATT;
    calData_Audio[486] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_L8_ATT;
    calData_Audio[487] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_L9_ATT;
    calData_Audio[488] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_R1_GAIN;
    calData_Audio[489] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_R2_GAIN;
    calData_Audio[490] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_R3_GAIN;
    calData_Audio[491] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_R4_GAIN;
    calData_Audio[492] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_R5_GAIN;
    calData_Audio[493] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_R6_GAIN;
    calData_Audio[494] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_R7_GAIN;
    calData_Audio[495] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_R8_GAIN;
    calData_Audio[496] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_R_R9_GAIN;
    calData_Audio[497] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_R1_ATT;
    calData_Audio[498] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_R2_ATT;
    calData_Audio[499] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_R3_ATT;
    calData_Audio[500] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_R4_ATT;
    calData_Audio[501] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_R5_ATT;
    calData_Audio[502] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_R6_ATT;
    calData_Audio[503] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_R7_ATT;
    calData_Audio[504] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_R8_ATT;
    calData_Audio[505] = Cal_Union_Parameters.Cal_Parameter.EQ_BAL_STEP_L_R9_ATT;
    calData_Audio[506] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_F1_GAIN;
    calData_Audio[507] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_F2_GAIN;
    calData_Audio[508] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_F3_GAIN;
    calData_Audio[509] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_F4_GAIN;
    calData_Audio[510] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_F5_GAIN;
    calData_Audio[511] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_F6_GAIN;
    calData_Audio[512] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_F7_GAIN;
    calData_Audio[513] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_F8_GAIN;
    calData_Audio[514] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_F9_GAIN;
    calData_Audio[515] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_F1_ATT;
    calData_Audio[516] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_F2_ATT;
    calData_Audio[517] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_F3_ATT;
    calData_Audio[518] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_F4_ATT;
    calData_Audio[519] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_F5_ATT;
    calData_Audio[520] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_F6_ATT;
    calData_Audio[521] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_F7_ATT;
    calData_Audio[522] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_F8_ATT;
    calData_Audio[523] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_F9_ATT;
    calData_Audio[524] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_R1_GAIN;
    calData_Audio[525] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_R2_GAIN;
    calData_Audio[526] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_R3_GAIN;
    calData_Audio[527] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_R4_GAIN;
    calData_Audio[528] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_R5_GAIN;
    calData_Audio[529] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_R6_GAIN;
    calData_Audio[530] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_R7_GAIN;
    calData_Audio[531] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_R8_GAIN;
    calData_Audio[532] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_R_R9_GAIN;
    calData_Audio[533] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_R1_ATT;
    calData_Audio[534] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_R2_ATT;
    calData_Audio[535] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_R3_ATT;
    calData_Audio[536] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_R4_ATT;
    calData_Audio[537] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_R5_ATT;
    calData_Audio[538] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_R6_ATT;
    calData_Audio[539] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_R7_ATT;
    calData_Audio[540] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_R8_ATT;
    calData_Audio[541] = Cal_Union_Parameters.Cal_Parameter.EQ_FADE_STEP_F_R9_ATT;
    calData_Audio[542] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_000_KMH_GAIN;
    calData_Audio[543] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_010_KMH_GAIN;
    calData_Audio[544] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_020_KMH_GAIN;
    calData_Audio[545] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_030_KMH_GAIN;
    calData_Audio[546] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_040_KMH_GAIN;
    calData_Audio[547] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_050_KMH_GAIN;
    calData_Audio[548] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_060_KMH_GAIN;
    calData_Audio[549] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_070_KMH_GAIN;
    calData_Audio[550] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_080_KMH_GAIN;
    calData_Audio[551] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_090_KMH_GAIN;
    calData_Audio[552] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_100_KMH_GAIN;
    calData_Audio[553] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_0_110_KMH_GAIN;
    calData_Audio[554] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_000_KMH_GAIN;
    calData_Audio[555] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_010_KMH_GAIN;
    calData_Audio[556] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_020_KMH_GAIN;
    calData_Audio[557] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_030_KMH_GAIN;
    calData_Audio[558] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_040_KMH_GAIN;
    calData_Audio[559] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_050_KMH_GAIN;
    calData_Audio[560] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_060_KMH_GAIN;
    calData_Audio[561] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_070_KMH_GAIN;
    calData_Audio[562] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_080_KMH_GAIN;
    calData_Audio[563] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_090_KMH_GAIN;
    calData_Audio[564] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_100_KMH_GAIN;
    calData_Audio[565] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_1_110_KMH_GAIN;
    calData_Audio[566] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_000_KMH_GAIN;
    calData_Audio[567] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_010_KMH_GAIN;
    calData_Audio[568] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_020_KMH_GAIN;
    calData_Audio[569] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_030_KMH_GAIN;
    calData_Audio[570] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_040_KMH_GAIN;
    calData_Audio[571] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_050_KMH_GAIN;
    calData_Audio[572] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_060_KMH_GAIN;
    calData_Audio[573] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_070_KMH_GAIN;
    calData_Audio[574] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_080_KMH_GAIN;
    calData_Audio[575] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_090_KMH_GAIN;
    calData_Audio[576] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_100_KMH_GAIN;
    calData_Audio[577] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_2_110_KMH_GAIN;
    calData_Audio[578] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_000_KMH_GAIN;
    calData_Audio[579] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_010_KMH_GAIN;
    calData_Audio[580] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_020_KMH_GAIN;
    calData_Audio[581] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_030_KMH_GAIN;
    calData_Audio[582] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_040_KMH_GAIN;
    calData_Audio[583] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_050_KMH_GAIN;
    calData_Audio[584] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_060_KMH_GAIN;
    calData_Audio[585] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_070_KMH_GAIN;
    calData_Audio[586] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_080_KMH_GAIN;
    calData_Audio[587] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_090_KMH_GAIN;
    calData_Audio[588] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_100_KMH_GAIN;
    calData_Audio[589] = Cal_Union_Parameters.Cal_Parameter.SVC_CURVE_3_110_KMH_GAIN;
    calData_Audio[590] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_MEDIA_DEF_STEP;
    calData_Audio[591] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_PHONE_DEF_STEP;
    calData_Audio[592] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_PHONE_RING_DEF_STEP;
    calData_Audio[593] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_NAV_DEF_STEP;
    calData_Audio[594] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_VR_DEF_STEP;
    calData_Audio[595] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_00_ATT;
    calData_Audio[596] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_01_ATT;
    calData_Audio[597] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_02_ATT;
    calData_Audio[598] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_03_ATT;
    calData_Audio[599] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_04_ATT;
    calData_Audio[600] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_05_ATT;
    calData_Audio[601] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_06_ATT;
    calData_Audio[602] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_07_ATT;
    calData_Audio[603] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_08_ATT;
    calData_Audio[604] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_09_ATT;
    calData_Audio[605] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_10_ATT;
    calData_Audio[606] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_11_ATT;
    calData_Audio[607] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_12_ATT;
    calData_Audio[608] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_13_ATT;
    calData_Audio[609] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_14_ATT;
    calData_Audio[610] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_15_ATT;
    calData_Audio[611] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_16_ATT;
    calData_Audio[612] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_17_ATT;
    calData_Audio[613] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_18_ATT;
    calData_Audio[614] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_19_ATT;
    calData_Audio[615] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_20_ATT;
    calData_Audio[616] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_21_ATT;
    calData_Audio[617] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_22_ATT;
    calData_Audio[618] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_23_ATT;
    calData_Audio[619] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_24_ATT;
    calData_Audio[620] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_25_ATT;
    calData_Audio[621] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_26_ATT;
    calData_Audio[622] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_27_ATT;
    calData_Audio[623] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_28_ATT;
    calData_Audio[624] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_29_ATT;
    calData_Audio[625] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_30_ATT;
    calData_Audio[626] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_31_ATT;
    calData_Audio[627] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_32_ATT;
    calData_Audio[628] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_33_ATT;
    calData_Audio[629] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_34_ATT;
    calData_Audio[630] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_35_ATT;
    calData_Audio[631] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_36_ATT;
    calData_Audio[632] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_37_ATT;
    calData_Audio[633] = Cal_Union_Parameters.Cal_Parameter.EQ_VOL_STEP_38_ATT;
    calData_Audio[634] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_WAVEFORM;
    calData_Audio[635] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_FREQ;
    calData_Audio[636] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_VOL_STEP;
    calData_Audio[637] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_ATT_TYPE;
    calData_Audio[638] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_ATT_TIME;
    calData_Audio[639] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_TON;
    calData_Audio[640] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_DEC_TYPE;
    calData_Audio[641] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_DEC_TIME;
    calData_Audio[642] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_TOFF;
    calData_Audio[643] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_NPULSES;
    calData_Audio[644] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE1_TDWELL;
    calData_Audio[645] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_WAVEFORM;
    calData_Audio[646] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_FREQ;
    calData_Audio[647] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_VOL_STEP;
    calData_Audio[648] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_ATT_TYPE;
    calData_Audio[649] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_ATT_TIME;
    calData_Audio[650] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_TON;
    calData_Audio[651] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_DEC_TYPE;
    calData_Audio[652] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_DEC_TIME;
    calData_Audio[653] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_TOFF;
    calData_Audio[654] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_NPULSES;
    calData_Audio[655] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE2_TDWELL;
    calData_Audio[656] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_WAVEFORM;
    calData_Audio[657] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_FREQ;
    calData_Audio[658] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_VOL_STEP;
    calData_Audio[659] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_ATT_TYPE;
    calData_Audio[660] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_ATT_TIME;
    calData_Audio[661] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_TON;
    calData_Audio[662] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_DEC_TYPE;
    calData_Audio[663] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_DEC_TIME;
    calData_Audio[664] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_TOFF;
    calData_Audio[665] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_NPULSES;
    calData_Audio[666] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE3_TDWELL;
    calData_Audio[667] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE4_WAVEFORM;
    calData_Audio[668] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE4_FREQ;
    calData_Audio[669] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE4_VOL_STEP;
    calData_Audio[670] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE4_ATT_TYPE;
    calData_Audio[671] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE4_ATT_TIME;
    calData_Audio[672] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE4_TON;
    calData_Audio[673] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE4_DEC_TYPE;
    calData_Audio[674] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE4_DEC_TIME;
    calData_Audio[675] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE5_WAVEFORM;
    calData_Audio[676] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE5_FREQ;
    calData_Audio[677] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE5_VOL_STEP;
    calData_Audio[678] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE5_ATT_TYPE;
    calData_Audio[679] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE5_ATT_TIME;
    calData_Audio[680] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE5_TON;
    calData_Audio[681] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE5_DEC_TYPE;
    calData_Audio[682] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE5_DEC_TIME;
    calData_Audio[683] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE6_WAVEFORM;
    calData_Audio[684] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE6_FREQ;
    calData_Audio[685] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE6_VOL_STEP;
    calData_Audio[686] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE6_ATT_TYPE;
    calData_Audio[687] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE6_ATT_TIME;
    calData_Audio[688] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE6_TON;
    calData_Audio[689] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE6_DEC_TYPE;
    calData_Audio[690] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE6_DEC_TIME;
    calData_Audio[691] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_WAVEFORM;
    calData_Audio[692] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_FREQ;
    calData_Audio[693] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_VOL_STEP;
    calData_Audio[694] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_ATT_TYPE;
    calData_Audio[695] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_ATT_TIME;
    calData_Audio[696] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_TON;
    calData_Audio[697] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_DEC_TYPE;
    calData_Audio[698] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_DEC_TIME;
    calData_Audio[699] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_TOFF;
    calData_Audio[700] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_NPULSES;
    calData_Audio[701] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE7_TDWELL;
    calData_Audio[702] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE8_WAVEFORM;
    calData_Audio[703] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE8_FREQ;
    calData_Audio[704] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE8_VOL_STEP;
    calData_Audio[705] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE8_ATT_TYPE;
    calData_Audio[706] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE8_ATT_TIME;
    calData_Audio[707] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE8_TON;
    calData_Audio[708] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE8_DEC_TYPE;
    calData_Audio[709] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE8_DEC_TIME;
    calData_Audio[710] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_WAVEFORM;
    calData_Audio[711] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_FREQ;
    calData_Audio[712] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_VOL_STEP;
    calData_Audio[713] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_ATT_TYPE;
    calData_Audio[714] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_ATT_TIME;
    calData_Audio[715] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_TON;
    calData_Audio[716] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_DEC_TYPE;
    calData_Audio[717] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_DEC_TIME;
    calData_Audio[718] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_TOFF;
    calData_Audio[719] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_NPULSES;
    calData_Audio[720] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE9_TDWELL;
    calData_Audio[721] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_WAVEFORM;
    calData_Audio[722] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_FREQ;
    calData_Audio[723] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_VOL_STEP;
    calData_Audio[724] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_ATT_TYPE;
    calData_Audio[725] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_ATT_TIME;
    calData_Audio[726] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_TON;
    calData_Audio[727] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_DEC_TYPE;
    calData_Audio[728] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_DEC_TIME;
    calData_Audio[729] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_TOFF;
    calData_Audio[730] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_NPULSES;
    calData_Audio[731] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE10_TDWELL;
    calData_Audio[732] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_WAVEFORM;
    calData_Audio[733] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_FREQ;
    calData_Audio[734] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_VOL_STEP;
    calData_Audio[735] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_ATT_TYPE;
    calData_Audio[736] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_ATT_TIME;
    calData_Audio[737] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_TON;
    calData_Audio[738] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_DEC_TYPE;
    calData_Audio[739] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_DEC_TIME;
    calData_Audio[740] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_TOFF;
    calData_Audio[741] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_NPULSES;
    calData_Audio[742] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE11_TDWELL;
    calData_Audio[743] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_WAVEFORM;
    calData_Audio[744] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_FREQ;
    calData_Audio[745] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_VOL_STEP;
    calData_Audio[746] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_ATT_TYPE;
    calData_Audio[747] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_ATT_TIME;
    calData_Audio[748] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_TON;
    calData_Audio[749] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_DEC_TYPE;
    calData_Audio[750] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_DEC_TIME;
    calData_Audio[751] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_TOFF;
    calData_Audio[752] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_NPULSES;
    calData_Audio[753] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_TYPE12_TDWELL;
    calData_Audio[754] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF1_WAVEFORM;
    calData_Audio[755] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF1_FREQ;
    calData_Audio[756] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF1_ATT_TYPE;
    calData_Audio[757] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF1_ATT_TIME;
    calData_Audio[758] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF1_TON;
    calData_Audio[759] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF1_DEC_TYPE;
    calData_Audio[760] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF1_DEC_TIME;
    calData_Audio[761] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF1_TOFF;
    calData_Audio[762] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF1_NPULSES;
    calData_Audio[763] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF2_WAVEFORM;
    calData_Audio[764] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF2_FREQ;
    calData_Audio[765] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF2_ATT_TYPE;
    calData_Audio[766] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF2_ATT_TIME;
    calData_Audio[767] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF2_TON;
    calData_Audio[768] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF2_DEC_TYPE;
    calData_Audio[769] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF2_DEC_TIME;
    calData_Audio[770] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF2_TOFF;
    calData_Audio[771] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF2_NPULSES;
    calData_Audio[772] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3A_WAVEFORM;
    calData_Audio[773] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3A_FREQ;
    calData_Audio[774] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3A_ATT_TYPE;
    calData_Audio[775] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3A_ATT_TIME;
    calData_Audio[776] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3A_TON;
    calData_Audio[777] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3A_DEC_TYPE;
    calData_Audio[778] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3A_DEC_TIME;
    calData_Audio[779] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3A_TOFF;
    calData_Audio[780] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3A_NPULSES;
    calData_Audio[781] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3B_WAVEFORM;
    calData_Audio[782] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3B_FREQ;
    calData_Audio[783] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3B_ATT_TYPE;
    calData_Audio[784] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3B_ATT_TIME;
    calData_Audio[785] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3B_TON;
    calData_Audio[786] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3B_DEC_TYPE;
    calData_Audio[787] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3B_DEC_TIME;
    calData_Audio[788] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3B_TOFF;
    calData_Audio[789] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF3B_NPULSES;
    calData_Audio[790] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF4_WAVEFORM;
    calData_Audio[791] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF4_FREQ;
    calData_Audio[792] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF4_ATT_TYPE;
    calData_Audio[793] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF4_ATT_TIME;
    calData_Audio[794] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF4_TON;
    calData_Audio[795] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF4_DEC_TYPE;
    calData_Audio[796] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF4_DEC_TIME;
    calData_Audio[797] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF4_TOFF;
    calData_Audio[798] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF4_NPULSES;
    calData_Audio[799] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF5_WAVEFORM;
    calData_Audio[800] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF5_FREQ;
    calData_Audio[801] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF5_ATT_TYPE;
    calData_Audio[802] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF5_ATT_TIME;
    calData_Audio[803] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF5_TON;
    calData_Audio[804] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF5_DEC_TYPE;
    calData_Audio[805] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF5_DEC_TIME;
    calData_Audio[806] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF5_TOFF;
    calData_Audio[807] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_CONF5_NPULSES;
    calData_Audio[808] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT1_WAVEFORM;
    calData_Audio[809] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT1_FREQ;
    calData_Audio[810] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT1_ATT_TYPE;
    calData_Audio[811] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT1_ATT_TIME;
    calData_Audio[812] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT1_TON;
    calData_Audio[813] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT1_DEC_TYPE;
    calData_Audio[814] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT1_DEC_TIME;
    calData_Audio[815] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT1_TOFF;
    calData_Audio[816] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT1_NPULSES;
    calData_Audio[817] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT2_WAVEFORM;
    calData_Audio[818] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT2_FREQ;
    calData_Audio[819] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT2_ATT_TYPE;
    calData_Audio[820] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT2_ATT_TIME;
    calData_Audio[821] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT2_TON;
    calData_Audio[822] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT2_DEC_TYPE;
    calData_Audio[823] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT2_DEC_TIME;
    calData_Audio[824] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT2_TOFF;
    calData_Audio[825] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT2_NPULSES;
    calData_Audio[826] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT3_WAVEFORM;
    calData_Audio[827] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT3_FREQ;
    calData_Audio[828] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT3_ATT_TYPE;
    calData_Audio[829] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT3_ATT_TIME;
    calData_Audio[830] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT3_TON;
    calData_Audio[831] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT3_DEC_TYPE;
    calData_Audio[832] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT3_DEC_TIME;
    calData_Audio[833] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT3_TOFF;
    calData_Audio[834] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT3_NPULSES;
    calData_Audio[835] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT4_WAVEFORM;
    calData_Audio[836] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT4_FREQ;
    calData_Audio[837] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT4_ATT_TYPE;
    calData_Audio[838] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT4_ATT_TIME;
    calData_Audio[839] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT4_TON;
    calData_Audio[840] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT4_DEC_TYPE;
    calData_Audio[841] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT4_DEC_TIME;
    calData_Audio[842] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT4_TOFF;
    calData_Audio[843] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT4_NPULSES;
    calData_Audio[844] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT5_WAVEFORM;
    calData_Audio[845] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT5_FREQ;
    calData_Audio[846] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT5_ATT_TYPE;
    calData_Audio[847] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT5_ATT_TIME;
    calData_Audio[848] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT5_TON;
    calData_Audio[849] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT5_DEC_TYPE;
    calData_Audio[850] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT5_DEC_TIME;
    calData_Audio[851] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT5_TOFF;
    calData_Audio[852] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT5_NPULSES;
    calData_Audio[853] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT6_WAVEFORM;
    calData_Audio[854] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT6_FREQ;
    calData_Audio[855] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT6_ATT_TYPE;
    calData_Audio[856] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT6_ATT_TIME;
    calData_Audio[857] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT6_TON;
    calData_Audio[858] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT6_DEC_TYPE;
    calData_Audio[859] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT6_DEC_TIME;
    calData_Audio[860] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT6_TOFF;
    calData_Audio[861] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT6_NPULSES;
    calData_Audio[862] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT7_WAVEFORM;
    calData_Audio[863] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT7_FREQ;
    calData_Audio[864] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT7_ATT_TYPE;
    calData_Audio[865] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT7_ATT_TIME;
    calData_Audio[866] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT7_TON;
    calData_Audio[867] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT7_DEC_TYPE;
    calData_Audio[868] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT7_DEC_TIME;
    calData_Audio[869] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT7_TOFF;
    calData_Audio[870] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT7_NPULSES;
    calData_Audio[871] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT8_WAVEFORM;
    calData_Audio[872] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT8_FREQ;
    calData_Audio[873] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT8_ATT_TYPE;
    calData_Audio[874] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT8_ATT_TIME;
    calData_Audio[875] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT8_TON;
    calData_Audio[876] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT8_DEC_TYPE;
    calData_Audio[877] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT8_DEC_TIME;
    calData_Audio[878] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT8_TOFF;
    calData_Audio[879] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT8_NPULSES;
    calData_Audio[880] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT9_WAVEFORM;
    calData_Audio[881] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT9_FREQ;
    calData_Audio[882] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT9_ATT_TYPE;
    calData_Audio[883] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT9_ATT_TIME;
    calData_Audio[884] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT9_TON;
    calData_Audio[885] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT9_DEC_TYPE;
    calData_Audio[886] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT9_DEC_TIME;
    calData_Audio[887] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT9_TOFF;
    calData_Audio[888] = Cal_Union_Parameters.Cal_Parameter.EQ_CHIME_ALERT9_NPULSES;
    calData_Audio[889] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_1_TYPE;
    calData_Audio[890] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_1_FREQ;
    calData_Audio[891] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_1_GAIN;
    calData_Audio[892] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_1_Q_FACTOR;
    calData_Audio[893] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_2_TYPE;
    calData_Audio[894] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_2_FREQ;
    calData_Audio[895] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_2_GAIN;
    calData_Audio[896] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_2_Q_FACTOR;
    calData_Audio[897] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_3_TYPE;
    calData_Audio[898] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_3_FREQ;
    calData_Audio[899] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_3_GAIN;
    calData_Audio[900] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_3_Q_FACTOR;
    calData_Audio[901] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_4_TYPE;
    calData_Audio[902] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_4_FREQ;
    calData_Audio[903] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_4_GAIN;
    calData_Audio[904] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_4_Q_FACTOR;
    calData_Audio[905] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_5_TYPE;
    calData_Audio[906] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_5_FREQ;
    calData_Audio[907] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_5_GAIN;
    calData_Audio[908] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_5_Q_FACTOR;
    calData_Audio[909] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_6_TYPE;
    calData_Audio[910] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_6_FREQ;
    calData_Audio[911] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_6_GAIN;
    calData_Audio[912] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_6_Q_FACTOR;
    calData_Audio[913] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_7_TYPE;
    calData_Audio[914] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_7_FREQ;
    calData_Audio[915] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_7_GAIN;
    calData_Audio[916] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_7_Q_FACTOR;
    calData_Audio[917] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_8_TYPE;
    calData_Audio[918] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_8_FREQ;
    calData_Audio[919] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_8_GAIN;
    calData_Audio[920] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC1_8_Q_FACTOR;
    calData_Audio[921] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_1_TYPE;
    calData_Audio[922] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_1_FREQ;
    calData_Audio[923] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_1_GAIN;
    calData_Audio[924] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_1_Q_FACTOR;
    calData_Audio[925] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_2_TYPE;
    calData_Audio[926] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_2_FREQ;
    calData_Audio[927] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_2_GAIN;
    calData_Audio[928] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_2_Q_FACTOR;
    calData_Audio[929] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_3_TYPE;
    calData_Audio[930] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_3_FREQ;
    calData_Audio[931] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_3_GAIN;
    calData_Audio[932] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_3_Q_FACTOR;
    calData_Audio[933] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_4_TYPE;
    calData_Audio[934] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_4_FREQ;
    calData_Audio[935] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_4_GAIN;
    calData_Audio[936] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_4_Q_FACTOR;
    calData_Audio[937] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_5_TYPE;
    calData_Audio[938] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_5_FREQ;
    calData_Audio[939] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_5_GAIN;
    calData_Audio[940] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_5_Q_FACTOR;
    calData_Audio[941] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_6_TYPE;
    calData_Audio[942] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_6_FREQ;
    calData_Audio[943] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_6_GAIN;
    calData_Audio[944] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_6_Q_FACTOR;
    calData_Audio[945] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_7_TYPE;
    calData_Audio[946] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_7_FREQ;
    calData_Audio[947] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_7_GAIN;
    calData_Audio[948] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_7_Q_FACTOR;
    calData_Audio[949] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_8_TYPE;
    calData_Audio[950] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_8_FREQ;
    calData_Audio[951] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_8_GAIN;
    calData_Audio[952] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_MIC2_8_Q_FACTOR;
    calData_Audio[953] = Cal_Union_Parameters.Cal_Parameter.EQ_MIC1_GAIN;
    calData_Audio[954] = Cal_Union_Parameters.Cal_Parameter.EQ_MIC2_GAIN;
    calData_Audio[955] = Cal_Union_Parameters.Cal_Parameter.EQ_XCALL_PROMPT_ATT;
    calData_Audio[956] = Cal_Union_Parameters.Cal_Parameter.EQ_AGC_PHONE_ENABLE;
    calData_Audio[957] = Cal_Union_Parameters.Cal_Parameter.EQ_AGC_IN_MIN_GAIN;
    calData_Audio[958] = Cal_Union_Parameters.Cal_Parameter.EQ_AGC_OUT_MIN_GAIN;
    calData_Audio[959] = Cal_Union_Parameters.Cal_Parameter.EQ_AGC_IN_MAX_GAIN;
    calData_Audio[960] = Cal_Union_Parameters.Cal_Parameter.EQ_AGC_OUT_MAX_GAIN;
    calData_Audio[961] = Cal_Union_Parameters.Cal_Parameter.EQ_MIC_NR_INT;
    calData_Audio[962] = Cal_Union_Parameters.Cal_Parameter.EQ_NR_PHONE_ENABLE;
    calData_Audio[963] = Cal_Union_Parameters.Cal_Parameter.EQ_COMF_NOISE_INT;
    calData_Audio[964] = Cal_Union_Parameters.Cal_Parameter.EQ_AEC_INT;
    calData_Audio[965] = Cal_Union_Parameters.Cal_Parameter.EQ_AEC_OFFSET_DELAY;
    calData_Audio[966] = Cal_Union_Parameters.Cal_Parameter.EQ_COMP_ENABLED;
    calData_Audio[967] = Cal_Union_Parameters.Cal_Parameter.EQ_COMP_THRESHOLD;
    calData_Audio[968] = Cal_Union_Parameters.Cal_Parameter.EQ_COMP_RATIO;
    calData_Audio[969] = Cal_Union_Parameters.Cal_Parameter.EQ_COMP_ATT_TIME;
    calData_Audio[970] = Cal_Union_Parameters.Cal_Parameter.EQ_COMP_REL_TIME;
    calData_Audio[971] = Cal_Union_Parameters.Cal_Parameter.EQ_COMP_KNEE_TYPE;
    calData_Audio[972] = Cal_Union_Parameters.Cal_Parameter.EQ_COMP_GAIN;
    calData_Audio[973] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_TYPE;
    calData_Audio[974] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_FREQ;
    calData_Audio[975] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_Q_FACTOR;
    calData_Audio[976] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_N09_GAIN;
    calData_Audio[977] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_N08_GAIN;
    calData_Audio[978] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_N07_GAIN;
    calData_Audio[979] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_N06_GAIN;
    calData_Audio[980] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_N05_GAIN;
    calData_Audio[981] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_N04_GAIN;
    calData_Audio[982] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_N03_GAIN;
    calData_Audio[983] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_N02_GAIN;
    calData_Audio[984] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_N01_GAIN;
    calData_Audio[985] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_00_GAIN;
    calData_Audio[986] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_P01_GAIN;
    calData_Audio[987] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_P02_GAIN;
    calData_Audio[988] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_P03_GAIN;
    calData_Audio[989] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_P04_GAIN;
    calData_Audio[990] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_P05_GAIN;
    calData_Audio[991] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_P06_GAIN;
    calData_Audio[992] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_P07_GAIN;
    calData_Audio[993] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_P08_GAIN;
    calData_Audio[994] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_BASS_3_STEP_P09_GAIN;
    calData_Audio[995] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_TYPE;
    calData_Audio[996] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_FREQ;
    calData_Audio[997] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_Q_FACTOR;
    calData_Audio[998] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_N09_GAIN;
    calData_Audio[999] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_N08_GAIN;
    calData_Audio[1000] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_N07_GAIN;
    calData_Audio[1001] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_N06_GAIN;
    calData_Audio[1002] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_N05_GAIN;
    calData_Audio[1003] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_N04_GAIN;
    calData_Audio[1004] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_N03_GAIN;
    calData_Audio[1005] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_N02_GAIN;
    calData_Audio[1006] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_N01_GAIN;
    calData_Audio[1007] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_00_GAIN;
    calData_Audio[1008] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_P01_GAIN;
    calData_Audio[1009] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_P02_GAIN;
    calData_Audio[1010] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_P03_GAIN;
    calData_Audio[1011] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_P04_GAIN;
    calData_Audio[1012] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_P05_GAIN;
    calData_Audio[1013] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_P06_GAIN;
    calData_Audio[1014] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_P07_GAIN;
    calData_Audio[1015] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_P08_GAIN;
    calData_Audio[1016] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_MID_3_STEP_P09_GAIN;
    calData_Audio[1017] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_TYPE;
    calData_Audio[1018] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_FREQ;
    calData_Audio[1019] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_Q_FACTOR;
    calData_Audio[1020] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_N09_GAIN;
    calData_Audio[1021] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_N08_GAIN;
    calData_Audio[1022] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_N07_GAIN;
    calData_Audio[1023] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_N06_GAIN;
    calData_Audio[1024] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_N05_GAIN;
    calData_Audio[1025] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_N04_GAIN;
    calData_Audio[1026] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_N03_GAIN;
    calData_Audio[1027] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_N02_GAIN;
    calData_Audio[1028] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_N01_GAIN;
    calData_Audio[1029] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_00_GAIN;
    calData_Audio[1030] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_P01_GAIN;
    calData_Audio[1031] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_P02_GAIN;
    calData_Audio[1032] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_P03_GAIN;
    calData_Audio[1033] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_P04_GAIN;
    calData_Audio[1034] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_P05_GAIN;
    calData_Audio[1035] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_P06_GAIN;
    calData_Audio[1036] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_P07_GAIN;
    calData_Audio[1037] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_P08_GAIN;
    calData_Audio[1038] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_USER_EQ_TREB_3_STEP_P09_GAIN;
    calData_Audio[1039] = Cal_Union_Parameters.Cal_Parameter.SVC_HYSTERESIS_VAL;
    calData_Audio[1040] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_01_TYPE;
    calData_Audio[1041] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_01_FREQ;
    calData_Audio[1042] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_01_GAIN;
    calData_Audio[1043] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_01_Q_FACTOR;
    calData_Audio[1044] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_02_TYPE;
    calData_Audio[1045] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_02_FREQ;
    calData_Audio[1046] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_02_GAIN;
    calData_Audio[1047] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_02_Q_FACTOR;
    calData_Audio[1048] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_03_TYPE;
    calData_Audio[1049] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_03_FREQ;
    calData_Audio[1050] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_03_GAIN;
    calData_Audio[1051] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_03_Q_FACTOR;
    calData_Audio[1052] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_04_TYPE;
    calData_Audio[1053] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_04_FREQ;
    calData_Audio[1054] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_04_GAIN;
    calData_Audio[1055] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_04_Q_FACTOR;
    calData_Audio[1056] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_05_TYPE;
    calData_Audio[1057] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_05_FREQ;
    calData_Audio[1058] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_05_GAIN;
    calData_Audio[1059] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_05_Q_FACTOR;
    calData_Audio[1060] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_06_TYPE;
    calData_Audio[1061] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_06_FREQ;
    calData_Audio[1062] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_06_GAIN;
    calData_Audio[1063] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_06_Q_FACTOR;
    calData_Audio[1064] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_07_TYPE;
    calData_Audio[1065] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_07_FREQ;
    calData_Audio[1066] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_07_GAIN;
    calData_Audio[1067] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_07_Q_FACTOR;
    calData_Audio[1068] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_08_TYPE;
    calData_Audio[1069] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_08_FREQ;
    calData_Audio[1070] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_08_GAIN;
    calData_Audio[1071] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_08_Q_FACTOR;
    calData_Audio[1072] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_09_TYPE;
    calData_Audio[1073] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_09_FREQ;
    calData_Audio[1074] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_09_GAIN;
    calData_Audio[1075] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_09_Q_FACTOR;
    calData_Audio[1076] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_10_TYPE;
    calData_Audio[1077] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_10_FREQ;
    calData_Audio[1078] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_10_GAIN;
    calData_Audio[1079] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_10_Q_FACTOR;
    calData_Audio[1080] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_11_TYPE;
    calData_Audio[1081] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_11_FREQ;
    calData_Audio[1082] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_11_GAIN;
    calData_Audio[1083] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_11_Q_FACTOR;
    calData_Audio[1084] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_12_TYPE;
    calData_Audio[1085] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_12_FREQ;
    calData_Audio[1086] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_12_GAIN;
    calData_Audio[1087] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DEF_12_Q_FACTOR;
    calData_Audio[1088] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_01_TYPE;
    calData_Audio[1089] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_01_FREQ;
    calData_Audio[1090] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_01_GAIN;
    calData_Audio[1091] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_01_Q_FACTOR;
    calData_Audio[1092] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_02_TYPE;
    calData_Audio[1093] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_02_FREQ;
    calData_Audio[1094] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_02_GAIN;
    calData_Audio[1095] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_02_Q_FACTOR;
    calData_Audio[1096] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_03_TYPE;
    calData_Audio[1097] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_03_FREQ;
    calData_Audio[1098] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_03_GAIN;
    calData_Audio[1099] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_03_Q_FACTOR;
    calData_Audio[1100] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_04_TYPE;
    calData_Audio[1101] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_04_FREQ;
    calData_Audio[1102] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_04_GAIN;
    calData_Audio[1103] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_04_Q_FACTOR;
    calData_Audio[1104] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_05_TYPE;
    calData_Audio[1105] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_05_FREQ;
    calData_Audio[1106] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_05_GAIN;
    calData_Audio[1107] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_05_Q_FACTOR;
    calData_Audio[1108] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_06_TYPE;
    calData_Audio[1109] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_06_FREQ;
    calData_Audio[1110] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_06_GAIN;
    calData_Audio[1111] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_06_Q_FACTOR;
    calData_Audio[1112] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_07_TYPE;
    calData_Audio[1113] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_07_FREQ;
    calData_Audio[1114] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_07_GAIN;
    calData_Audio[1115] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_07_Q_FACTOR;
    calData_Audio[1116] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_08_TYPE;
    calData_Audio[1117] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_08_FREQ;
    calData_Audio[1118] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_08_GAIN;
    calData_Audio[1119] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_08_Q_FACTOR;
    calData_Audio[1120] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_09_TYPE;
    calData_Audio[1121] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_09_FREQ;
    calData_Audio[1122] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_09_GAIN;
    calData_Audio[1123] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_09_Q_FACTOR;
    calData_Audio[1124] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_10_TYPE;
    calData_Audio[1125] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_10_FREQ;
    calData_Audio[1126] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_10_GAIN;
    calData_Audio[1127] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_10_Q_FACTOR;
    calData_Audio[1128] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_11_TYPE;
    calData_Audio[1129] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_11_FREQ;
    calData_Audio[1130] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_11_GAIN;
    calData_Audio[1131] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_11_Q_FACTOR;
    calData_Audio[1132] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_12_TYPE;
    calData_Audio[1133] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_12_FREQ;
    calData_Audio[1134] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_12_GAIN;
    calData_Audio[1135] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_AM_12_Q_FACTOR;
    calData_Audio[1136] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_01_TYPE;
    calData_Audio[1137] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_01_FREQ;
    calData_Audio[1138] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_01_GAIN;
    calData_Audio[1139] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_01_Q_FACTOR;
    calData_Audio[1140] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_02_TYPE;
    calData_Audio[1141] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_02_FREQ;
    calData_Audio[1142] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_02_GAIN;
    calData_Audio[1143] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_02_Q_FACTOR;
    calData_Audio[1144] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_03_TYPE;
    calData_Audio[1145] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_03_FREQ;
    calData_Audio[1146] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_03_GAIN;
    calData_Audio[1147] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_03_Q_FACTOR;
    calData_Audio[1148] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_04_TYPE;
    calData_Audio[1149] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_04_FREQ;
    calData_Audio[1150] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_04_GAIN;
    calData_Audio[1151] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_04_Q_FACTOR;
    calData_Audio[1152] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_05_TYPE;
    calData_Audio[1153] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_05_FREQ;
    calData_Audio[1154] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_05_GAIN;
    calData_Audio[1155] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_05_Q_FACTOR;
    calData_Audio[1156] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_06_TYPE;
    calData_Audio[1157] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_06_FREQ;
    calData_Audio[1158] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_06_GAIN;
    calData_Audio[1159] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_06_Q_FACTOR;
    calData_Audio[1160] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_07_TYPE;
    calData_Audio[1161] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_07_FREQ;
    calData_Audio[1162] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_07_GAIN;
    calData_Audio[1163] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_07_Q_FACTOR;
    calData_Audio[1164] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_08_TYPE;
    calData_Audio[1165] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_08_FREQ;
    calData_Audio[1166] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_08_GAIN;
    calData_Audio[1167] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_08_Q_FACTOR;
    calData_Audio[1168] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_09_TYPE;
    calData_Audio[1169] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_09_FREQ;
    calData_Audio[1170] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_09_GAIN;
    calData_Audio[1171] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_09_Q_FACTOR;
    calData_Audio[1172] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_10_TYPE;
    calData_Audio[1173] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_10_FREQ;
    calData_Audio[1174] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_10_GAIN;
    calData_Audio[1175] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_10_Q_FACTOR;
    calData_Audio[1176] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_11_TYPE;
    calData_Audio[1177] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_11_FREQ;
    calData_Audio[1178] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_11_GAIN;
    calData_Audio[1179] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_11_Q_FACTOR;
    calData_Audio[1180] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_12_TYPE;
    calData_Audio[1181] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_12_FREQ;
    calData_Audio[1182] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_12_GAIN;
    calData_Audio[1183] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_FM_12_Q_FACTOR;
    calData_Audio[1184] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_01_TYPE;
    calData_Audio[1185] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_01_FREQ;
    calData_Audio[1186] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_01_GAIN;
    calData_Audio[1187] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_01_Q_FACTOR;
    calData_Audio[1188] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_02_TYPE;
    calData_Audio[1189] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_02_FREQ;
    calData_Audio[1190] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_02_GAIN;
    calData_Audio[1191] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_02_Q_FACTOR;
    calData_Audio[1192] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_03_TYPE;
    calData_Audio[1193] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_03_FREQ;
    calData_Audio[1194] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_03_GAIN;
    calData_Audio[1195] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_03_Q_FACTOR;
    calData_Audio[1196] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_04_TYPE;
    calData_Audio[1197] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_04_FREQ;
    calData_Audio[1198] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_04_GAIN;
    calData_Audio[1199] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_04_Q_FACTOR;
    calData_Audio[1200] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_05_TYPE;
    calData_Audio[1201] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_05_FREQ;
    calData_Audio[1202] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_05_GAIN;
    calData_Audio[1203] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_05_Q_FACTOR;
    calData_Audio[1204] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_06_TYPE;
    calData_Audio[1205] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_06_FREQ;
    calData_Audio[1206] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_06_GAIN;
    calData_Audio[1207] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_06_Q_FACTOR;
    calData_Audio[1208] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_07_TYPE;
    calData_Audio[1209] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_07_FREQ;
    calData_Audio[1210] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_07_GAIN;
    calData_Audio[1211] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_07_Q_FACTOR;
    calData_Audio[1212] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_08_TYPE;
    calData_Audio[1213] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_08_FREQ;
    calData_Audio[1214] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_08_GAIN;
    calData_Audio[1215] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_08_Q_FACTOR;
    calData_Audio[1216] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_09_TYPE;
    calData_Audio[1217] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_09_FREQ;
    calData_Audio[1218] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_09_GAIN;
    calData_Audio[1219] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_09_Q_FACTOR;
    calData_Audio[1220] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_10_TYPE;
    calData_Audio[1221] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_10_FREQ;
    calData_Audio[1222] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_10_GAIN;
    calData_Audio[1223] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_10_Q_FACTOR;
    calData_Audio[1224] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_11_TYPE;
    calData_Audio[1225] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_11_FREQ;
    calData_Audio[1226] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_11_GAIN;
    calData_Audio[1227] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_11_Q_FACTOR;
    calData_Audio[1228] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_12_TYPE;
    calData_Audio[1229] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_12_FREQ;
    calData_Audio[1230] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_12_GAIN;
    calData_Audio[1231] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_DAB_12_Q_FACTOR;
    calData_Audio[1232] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_01_TYPE;
    calData_Audio[1233] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_01_FREQ;
    calData_Audio[1234] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_01_GAIN;
    calData_Audio[1235] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_01_Q_FACTOR;
    calData_Audio[1236] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_02_TYPE;
    calData_Audio[1237] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_02_FREQ;
    calData_Audio[1238] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_02_GAIN;
    calData_Audio[1239] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_02_Q_FACTOR;
    calData_Audio[1240] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_03_TYPE;
    calData_Audio[1241] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_03_FREQ;
    calData_Audio[1242] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_03_GAIN;
    calData_Audio[1243] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_03_Q_FACTOR;
    calData_Audio[1244] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_04_TYPE;
    calData_Audio[1245] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_04_FREQ;
    calData_Audio[1246] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_04_GAIN;
    calData_Audio[1247] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_04_Q_FACTOR;
    calData_Audio[1248] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_05_TYPE;
    calData_Audio[1249] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_05_FREQ;
    calData_Audio[1250] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_05_GAIN;
    calData_Audio[1251] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_05_Q_FACTOR;
    calData_Audio[1252] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_06_TYPE;
    calData_Audio[1253] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_06_FREQ;
    calData_Audio[1254] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_06_GAIN;
    calData_Audio[1255] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_06_Q_FACTOR;
    calData_Audio[1256] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_07_TYPE;
    calData_Audio[1257] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_07_FREQ;
    calData_Audio[1258] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_07_GAIN;
    calData_Audio[1259] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_07_Q_FACTOR;
    calData_Audio[1260] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_08_TYPE;
    calData_Audio[1261] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_08_FREQ;
    calData_Audio[1262] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_08_GAIN;
    calData_Audio[1263] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_08_Q_FACTOR;
    calData_Audio[1264] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_09_TYPE;
    calData_Audio[1265] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_09_FREQ;
    calData_Audio[1266] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_09_GAIN;
    calData_Audio[1267] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_09_Q_FACTOR;
    calData_Audio[1268] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_10_TYPE;
    calData_Audio[1269] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_10_FREQ;
    calData_Audio[1270] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_10_GAIN;
    calData_Audio[1271] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_10_Q_FACTOR;
    calData_Audio[1272] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_11_TYPE;
    calData_Audio[1273] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_11_FREQ;
    calData_Audio[1274] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_11_GAIN;
    calData_Audio[1275] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_11_Q_FACTOR;
    calData_Audio[1276] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_12_TYPE;
    calData_Audio[1277] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_12_FREQ;
    calData_Audio[1278] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_12_GAIN;
    calData_Audio[1279] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_PROJ_12_Q_FACTOR;
    calData_Audio[1280] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_01_TYPE;
    calData_Audio[1281] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_01_FREQ;
    calData_Audio[1282] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_01_GAIN;
    calData_Audio[1283] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_01_Q_FACTOR;
    calData_Audio[1284] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_02_TYPE;
    calData_Audio[1285] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_02_FREQ;
    calData_Audio[1286] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_02_GAIN;
    calData_Audio[1287] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_02_Q_FACTOR;
    calData_Audio[1288] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_03_TYPE;
    calData_Audio[1289] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_03_FREQ;
    calData_Audio[1290] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_03_GAIN;
    calData_Audio[1291] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_03_Q_FACTOR;
    calData_Audio[1292] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_04_TYPE;
    calData_Audio[1293] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_04_FREQ;
    calData_Audio[1294] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_04_GAIN;
    calData_Audio[1295] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_04_Q_FACTOR;
    calData_Audio[1296] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_05_TYPE;
    calData_Audio[1297] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_05_FREQ;
    calData_Audio[1298] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_05_GAIN;
    calData_Audio[1299] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_05_Q_FACTOR;
    calData_Audio[1300] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_06_TYPE;
    calData_Audio[1301] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_06_FREQ;
    calData_Audio[1302] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_06_GAIN;
    calData_Audio[1303] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_06_Q_FACTOR;
    calData_Audio[1304] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_07_TYPE;
    calData_Audio[1305] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_07_FREQ;
    calData_Audio[1306] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_07_GAIN;
    calData_Audio[1307] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_07_Q_FACTOR;
    calData_Audio[1308] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_08_TYPE;
    calData_Audio[1309] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_08_FREQ;
    calData_Audio[1310] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_08_GAIN;
    calData_Audio[1311] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_08_Q_FACTOR;
    calData_Audio[1312] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_09_TYPE;
    calData_Audio[1313] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_09_FREQ;
    calData_Audio[1314] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_09_GAIN;
    calData_Audio[1315] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_09_Q_FACTOR;
    calData_Audio[1316] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_10_TYPE;
    calData_Audio[1317] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_10_FREQ;
    calData_Audio[1318] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_10_GAIN;
    calData_Audio[1319] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_10_Q_FACTOR;
    calData_Audio[1320] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_11_TYPE;
    calData_Audio[1321] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_11_FREQ;
    calData_Audio[1322] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_11_GAIN;
    calData_Audio[1323] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_11_Q_FACTOR;
    calData_Audio[1324] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_12_TYPE;
    calData_Audio[1325] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_12_FREQ;
    calData_Audio[1326] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_12_GAIN;
    calData_Audio[1327] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_BTSA_12_Q_FACTOR;
    calData_Audio[1328] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_01_TYPE;
    calData_Audio[1329] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_01_FREQ;
    calData_Audio[1330] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_01_GAIN;
    calData_Audio[1331] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_01_Q_FACTOR;
    calData_Audio[1332] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_02_TYPE;
    calData_Audio[1333] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_02_FREQ;
    calData_Audio[1334] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_02_GAIN;
    calData_Audio[1335] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_02_Q_FACTOR;
    calData_Audio[1336] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_03_TYPE;
    calData_Audio[1337] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_03_FREQ;
    calData_Audio[1338] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_03_GAIN;
    calData_Audio[1339] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_03_Q_FACTOR;
    calData_Audio[1340] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_04_TYPE;
    calData_Audio[1341] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_04_FREQ;
    calData_Audio[1342] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_04_GAIN;
    calData_Audio[1343] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_04_Q_FACTOR;
    calData_Audio[1344] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_05_TYPE;
    calData_Audio[1345] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_05_FREQ;
    calData_Audio[1346] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_05_GAIN;
    calData_Audio[1347] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_05_Q_FACTOR;
    calData_Audio[1348] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_06_TYPE;
    calData_Audio[1349] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_06_FREQ;
    calData_Audio[1350] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_06_GAIN;
    calData_Audio[1351] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_06_Q_FACTOR;
    calData_Audio[1352] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_07_TYPE;
    calData_Audio[1353] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_07_FREQ;
    calData_Audio[1354] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_07_GAIN;
    calData_Audio[1355] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_07_Q_FACTOR;
    calData_Audio[1356] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_08_TYPE;
    calData_Audio[1357] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_08_FREQ;
    calData_Audio[1358] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_08_GAIN;
    calData_Audio[1359] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_08_Q_FACTOR;
    calData_Audio[1360] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_09_TYPE;
    calData_Audio[1361] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_09_FREQ;
    calData_Audio[1362] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_09_GAIN;
    calData_Audio[1363] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_09_Q_FACTOR;
    calData_Audio[1364] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_10_TYPE;
    calData_Audio[1365] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_10_FREQ;
    calData_Audio[1366] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_10_GAIN;
    calData_Audio[1367] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_10_Q_FACTOR;
    calData_Audio[1368] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_11_TYPE;
    calData_Audio[1369] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_11_FREQ;
    calData_Audio[1370] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_11_GAIN;
    calData_Audio[1371] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_11_Q_FACTOR;
    calData_Audio[1372] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_12_TYPE;
    calData_Audio[1373] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_12_FREQ;
    calData_Audio[1374] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_12_GAIN;
    calData_Audio[1375] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FL_USB_12_Q_FACTOR;
    calData_Audio[1376] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_01_TYPE;
    calData_Audio[1377] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_01_FREQ;
    calData_Audio[1378] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_01_GAIN;
    calData_Audio[1379] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_01_Q_FACTOR;
    calData_Audio[1380] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_02_TYPE;
    calData_Audio[1381] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_02_FREQ;
    calData_Audio[1382] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_02_GAIN;
    calData_Audio[1383] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_02_Q_FACTOR;
    calData_Audio[1384] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_03_TYPE;
    calData_Audio[1385] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_03_FREQ;
    calData_Audio[1386] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_03_GAIN;
    calData_Audio[1387] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_03_Q_FACTOR;
    calData_Audio[1388] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_04_TYPE;
    calData_Audio[1389] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_04_FREQ;
    calData_Audio[1390] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_04_GAIN;
    calData_Audio[1391] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_04_Q_FACTOR;
    calData_Audio[1392] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_05_TYPE;
    calData_Audio[1393] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_05_FREQ;
    calData_Audio[1394] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_05_GAIN;
    calData_Audio[1395] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_05_Q_FACTOR;
    calData_Audio[1396] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_06_TYPE;
    calData_Audio[1397] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_06_FREQ;
    calData_Audio[1398] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_06_GAIN;
    calData_Audio[1399] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_06_Q_FACTOR;
    calData_Audio[1400] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_07_TYPE;
    calData_Audio[1401] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_07_FREQ;
    calData_Audio[1402] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_07_GAIN;
    calData_Audio[1403] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_07_Q_FACTOR;
    calData_Audio[1404] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_08_TYPE;
    calData_Audio[1405] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_08_FREQ;
    calData_Audio[1406] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_08_GAIN;
    calData_Audio[1407] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_08_Q_FACTOR;
    calData_Audio[1408] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_09_TYPE;
    calData_Audio[1409] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_09_FREQ;
    calData_Audio[1410] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_09_GAIN;
    calData_Audio[1411] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_09_Q_FACTOR;
    calData_Audio[1412] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_10_TYPE;
    calData_Audio[1413] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_10_FREQ;
    calData_Audio[1414] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_10_GAIN;
    calData_Audio[1415] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_10_Q_FACTOR;
    calData_Audio[1416] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_11_TYPE;
    calData_Audio[1417] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_11_FREQ;
    calData_Audio[1418] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_11_GAIN;
    calData_Audio[1419] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_11_Q_FACTOR;
    calData_Audio[1420] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_12_TYPE;
    calData_Audio[1421] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_12_FREQ;
    calData_Audio[1422] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_12_GAIN;
    calData_Audio[1423] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DEF_12_Q_FACTOR;
    calData_Audio[1424] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_01_TYPE;
    calData_Audio[1425] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_01_FREQ;
    calData_Audio[1426] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_01_GAIN;
    calData_Audio[1427] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_01_Q_FACTOR;
    calData_Audio[1428] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_02_TYPE;
    calData_Audio[1429] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_02_FREQ;
    calData_Audio[1430] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_02_GAIN;
    calData_Audio[1431] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_02_Q_FACTOR;
    calData_Audio[1432] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_03_TYPE;
    calData_Audio[1433] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_03_FREQ;
    calData_Audio[1434] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_03_GAIN;
    calData_Audio[1435] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_03_Q_FACTOR;
    calData_Audio[1436] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_04_TYPE;
    calData_Audio[1437] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_04_FREQ;
    calData_Audio[1438] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_04_GAIN;
    calData_Audio[1439] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_04_Q_FACTOR;
    calData_Audio[1440] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_05_TYPE;
    calData_Audio[1441] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_05_FREQ;
    calData_Audio[1442] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_05_GAIN;
    calData_Audio[1443] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_05_Q_FACTOR;
    calData_Audio[1444] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_06_TYPE;
    calData_Audio[1445] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_06_FREQ;
    calData_Audio[1446] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_06_GAIN;
    calData_Audio[1447] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_06_Q_FACTOR;
    calData_Audio[1448] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_07_TYPE;
    calData_Audio[1449] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_07_FREQ;
    calData_Audio[1450] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_07_GAIN;
    calData_Audio[1451] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_07_Q_FACTOR;
    calData_Audio[1452] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_08_TYPE;
    calData_Audio[1453] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_08_FREQ;
    calData_Audio[1454] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_08_GAIN;
    calData_Audio[1455] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_08_Q_FACTOR;
    calData_Audio[1456] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_09_TYPE;
    calData_Audio[1457] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_09_FREQ;
    calData_Audio[1458] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_09_GAIN;
    calData_Audio[1459] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_09_Q_FACTOR;
    calData_Audio[1460] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_10_TYPE;
    calData_Audio[1461] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_10_FREQ;
    calData_Audio[1462] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_10_GAIN;
    calData_Audio[1463] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_10_Q_FACTOR;
    calData_Audio[1464] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_11_TYPE;
    calData_Audio[1465] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_11_FREQ;
    calData_Audio[1466] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_11_GAIN;
    calData_Audio[1467] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_11_Q_FACTOR;
    calData_Audio[1468] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_12_TYPE;
    calData_Audio[1469] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_12_FREQ;
    calData_Audio[1470] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_12_GAIN;
    calData_Audio[1471] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_AM_12_Q_FACTOR;
    calData_Audio[1472] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_01_TYPE;
    calData_Audio[1473] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_01_FREQ;
    calData_Audio[1474] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_01_GAIN;
    calData_Audio[1475] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_01_Q_FACTOR;
    calData_Audio[1476] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_02_TYPE;
    calData_Audio[1477] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_02_FREQ;
    calData_Audio[1478] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_02_GAIN;
    calData_Audio[1479] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_02_Q_FACTOR;
    calData_Audio[1480] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_03_TYPE;
    calData_Audio[1481] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_03_FREQ;
    calData_Audio[1482] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_03_GAIN;
    calData_Audio[1483] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_03_Q_FACTOR;
    calData_Audio[1484] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_04_TYPE;
    calData_Audio[1485] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_04_FREQ;
    calData_Audio[1486] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_04_GAIN;
    calData_Audio[1487] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_04_Q_FACTOR;
    calData_Audio[1488] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_05_TYPE;
    calData_Audio[1489] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_05_FREQ;
    calData_Audio[1490] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_05_GAIN;
    calData_Audio[1491] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_05_Q_FACTOR;
    calData_Audio[1492] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_06_TYPE;
    calData_Audio[1493] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_06_FREQ;
    calData_Audio[1494] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_06_GAIN;
    calData_Audio[1495] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_06_Q_FACTOR;
    calData_Audio[1496] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_07_TYPE;
    calData_Audio[1497] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_07_FREQ;
    calData_Audio[1498] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_07_GAIN;
    calData_Audio[1499] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_07_Q_FACTOR;
    calData_Audio[1500] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_08_TYPE;
    calData_Audio[1501] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_08_FREQ;
    calData_Audio[1502] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_08_GAIN;
    calData_Audio[1503] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_08_Q_FACTOR;
    calData_Audio[1504] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_09_TYPE;
    calData_Audio[1505] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_09_FREQ;
    calData_Audio[1506] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_09_GAIN;
    calData_Audio[1507] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_09_Q_FACTOR;
    calData_Audio[1508] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_10_TYPE;
    calData_Audio[1509] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_10_FREQ;
    calData_Audio[1510] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_10_GAIN;
    calData_Audio[1511] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_10_Q_FACTOR;
    calData_Audio[1512] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_11_TYPE;
    calData_Audio[1513] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_11_FREQ;
    calData_Audio[1514] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_11_GAIN;
    calData_Audio[1515] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_11_Q_FACTOR;
    calData_Audio[1516] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_12_TYPE;
    calData_Audio[1517] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_12_FREQ;
    calData_Audio[1518] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_12_GAIN;
    calData_Audio[1519] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_FM_12_Q_FACTOR;
    calData_Audio[1520] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_01_TYPE;
    calData_Audio[1521] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_01_FREQ;
    calData_Audio[1522] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_01_GAIN;
    calData_Audio[1523] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_01_Q_FACTOR;
    calData_Audio[1524] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_02_TYPE;
    calData_Audio[1525] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_02_FREQ;
    calData_Audio[1526] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_02_GAIN;
    calData_Audio[1527] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_02_Q_FACTOR;
    calData_Audio[1528] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_03_TYPE;
    calData_Audio[1529] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_03_FREQ;
    calData_Audio[1530] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_03_GAIN;
    calData_Audio[1531] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_03_Q_FACTOR;
    calData_Audio[1532] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_04_TYPE;
    calData_Audio[1533] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_04_FREQ;
    calData_Audio[1534] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_04_GAIN;
    calData_Audio[1535] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_04_Q_FACTOR;
    calData_Audio[1536] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_05_TYPE;
    calData_Audio[1537] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_05_FREQ;
    calData_Audio[1538] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_05_GAIN;
    calData_Audio[1539] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_05_Q_FACTOR;
    calData_Audio[1540] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_06_TYPE;
    calData_Audio[1541] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_06_FREQ;
    calData_Audio[1542] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_06_GAIN;
    calData_Audio[1543] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_06_Q_FACTOR;
    calData_Audio[1544] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_07_TYPE;
    calData_Audio[1545] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_07_FREQ;
    calData_Audio[1546] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_07_GAIN;
    calData_Audio[1547] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_07_Q_FACTOR;
    calData_Audio[1548] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_08_TYPE;
    calData_Audio[1549] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_08_FREQ;
    calData_Audio[1550] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_08_GAIN;
    calData_Audio[1551] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_08_Q_FACTOR;
    calData_Audio[1552] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_09_TYPE;
    calData_Audio[1553] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_09_FREQ;
    calData_Audio[1554] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_09_GAIN;
    calData_Audio[1555] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_09_Q_FACTOR;
    calData_Audio[1556] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_10_TYPE;
    calData_Audio[1557] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_10_FREQ;
    calData_Audio[1558] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_10_GAIN;
    calData_Audio[1559] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_10_Q_FACTOR;
    calData_Audio[1560] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_11_TYPE;
    calData_Audio[1561] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_11_FREQ;
    calData_Audio[1562] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_11_GAIN;
    calData_Audio[1563] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_11_Q_FACTOR;
    calData_Audio[1564] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_12_TYPE;
    calData_Audio[1565] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_12_FREQ;
    calData_Audio[1566] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_12_GAIN;
    calData_Audio[1567] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_DAB_12_Q_FACTOR;
    calData_Audio[1568] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_01_TYPE;
    calData_Audio[1569] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_01_FREQ;
    calData_Audio[1570] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_01_GAIN;
    calData_Audio[1571] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_01_Q_FACTOR;
    calData_Audio[1572] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_02_TYPE;
    calData_Audio[1573] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_02_FREQ;
    calData_Audio[1574] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_02_GAIN;
    calData_Audio[1575] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_02_Q_FACTOR;
    calData_Audio[1576] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_03_TYPE;
    calData_Audio[1577] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_03_FREQ;
    calData_Audio[1578] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_03_GAIN;
    calData_Audio[1579] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_03_Q_FACTOR;
    calData_Audio[1580] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_04_TYPE;
    calData_Audio[1581] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_04_FREQ;
    calData_Audio[1582] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_04_GAIN;
    calData_Audio[1583] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_04_Q_FACTOR;
    calData_Audio[1584] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_05_TYPE;
    calData_Audio[1585] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_05_FREQ;
    calData_Audio[1586] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_05_GAIN;
    calData_Audio[1587] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_05_Q_FACTOR;
    calData_Audio[1588] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_06_TYPE;
    calData_Audio[1589] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_06_FREQ;
    calData_Audio[1590] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_06_GAIN;
    calData_Audio[1591] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_06_Q_FACTOR;
    calData_Audio[1592] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_07_TYPE;
    calData_Audio[1593] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_07_FREQ;
    calData_Audio[1594] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_07_GAIN;
    calData_Audio[1595] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_07_Q_FACTOR;
    calData_Audio[1596] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_08_TYPE;
    calData_Audio[1597] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_08_FREQ;
    calData_Audio[1598] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_08_GAIN;
    calData_Audio[1599] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_08_Q_FACTOR;
    calData_Audio[1600] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_09_TYPE;
    calData_Audio[1601] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_09_FREQ;
    calData_Audio[1602] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_09_GAIN;
    calData_Audio[1603] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_09_Q_FACTOR;
    calData_Audio[1604] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_10_TYPE;
    calData_Audio[1605] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_10_FREQ;
    calData_Audio[1606] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_10_GAIN;
    calData_Audio[1607] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_10_Q_FACTOR;
    calData_Audio[1608] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_11_TYPE;
    calData_Audio[1609] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_11_FREQ;
    calData_Audio[1610] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_11_GAIN;
    calData_Audio[1611] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_11_Q_FACTOR;
    calData_Audio[1612] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_12_TYPE;
    calData_Audio[1613] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_12_FREQ;
    calData_Audio[1614] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_12_GAIN;
    calData_Audio[1615] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_PROJ_12_Q_FACTOR;
    calData_Audio[1616] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_01_TYPE;
    calData_Audio[1617] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_01_FREQ;
    calData_Audio[1618] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_01_GAIN;
    calData_Audio[1619] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_01_Q_FACTOR;
    calData_Audio[1620] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_02_TYPE;
    calData_Audio[1621] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_02_FREQ;
    calData_Audio[1622] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_02_GAIN;
    calData_Audio[1623] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_02_Q_FACTOR;
    calData_Audio[1624] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_03_TYPE;
    calData_Audio[1625] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_03_FREQ;
    calData_Audio[1626] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_03_GAIN;
    calData_Audio[1627] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_03_Q_FACTOR;
    calData_Audio[1628] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_04_TYPE;
    calData_Audio[1629] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_04_FREQ;
    calData_Audio[1630] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_04_GAIN;
    calData_Audio[1631] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_04_Q_FACTOR;
    calData_Audio[1632] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_05_TYPE;
    calData_Audio[1633] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_05_FREQ;
    calData_Audio[1634] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_05_GAIN;
    calData_Audio[1635] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_05_Q_FACTOR;
    calData_Audio[1636] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_06_TYPE;
    calData_Audio[1637] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_06_FREQ;
    calData_Audio[1638] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_06_GAIN;
    calData_Audio[1639] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_06_Q_FACTOR;
    calData_Audio[1640] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_07_TYPE;
    calData_Audio[1641] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_07_FREQ;
    calData_Audio[1642] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_07_GAIN;
    calData_Audio[1643] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_07_Q_FACTOR;
    calData_Audio[1644] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_08_TYPE;
    calData_Audio[1645] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_08_FREQ;
    calData_Audio[1646] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_08_GAIN;
    calData_Audio[1647] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_08_Q_FACTOR;
    calData_Audio[1648] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_09_TYPE;
    calData_Audio[1649] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_09_FREQ;
    calData_Audio[1650] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_09_GAIN;
    calData_Audio[1651] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_09_Q_FACTOR;
    calData_Audio[1652] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_10_TYPE;
    calData_Audio[1653] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_10_FREQ;
    calData_Audio[1654] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_10_GAIN;
    calData_Audio[1655] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_10_Q_FACTOR;
    calData_Audio[1656] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_11_TYPE;
    calData_Audio[1657] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_11_FREQ;
    calData_Audio[1658] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_11_GAIN;
    calData_Audio[1659] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_11_Q_FACTOR;
    calData_Audio[1660] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_12_TYPE;
    calData_Audio[1661] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_12_FREQ;
    calData_Audio[1662] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_12_GAIN;
    calData_Audio[1663] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_BTSA_12_Q_FACTOR;
    calData_Audio[1664] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_01_TYPE;
    calData_Audio[1665] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_01_FREQ;
    calData_Audio[1666] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_01_GAIN;
    calData_Audio[1667] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_01_Q_FACTOR;
    calData_Audio[1668] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_02_TYPE;
    calData_Audio[1669] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_02_FREQ;
    calData_Audio[1670] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_02_GAIN;
    calData_Audio[1671] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_02_Q_FACTOR;
    calData_Audio[1672] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_03_TYPE;
    calData_Audio[1673] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_03_FREQ;
    calData_Audio[1674] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_03_GAIN;
    calData_Audio[1675] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_03_Q_FACTOR;
    calData_Audio[1676] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_04_TYPE;
    calData_Audio[1677] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_04_FREQ;
    calData_Audio[1678] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_04_GAIN;
    calData_Audio[1679] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_04_Q_FACTOR;
    calData_Audio[1680] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_05_TYPE;
    calData_Audio[1681] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_05_FREQ;
    calData_Audio[1682] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_05_GAIN;
    calData_Audio[1683] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_05_Q_FACTOR;
    calData_Audio[1684] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_06_TYPE;
    calData_Audio[1685] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_06_FREQ;
    calData_Audio[1686] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_06_GAIN;
    calData_Audio[1687] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_06_Q_FACTOR;
    calData_Audio[1688] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_07_TYPE;
    calData_Audio[1689] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_07_FREQ;
    calData_Audio[1690] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_07_GAIN;
    calData_Audio[1691] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_07_Q_FACTOR;
    calData_Audio[1692] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_08_TYPE;
    calData_Audio[1693] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_08_FREQ;
    calData_Audio[1694] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_08_GAIN;
    calData_Audio[1695] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_08_Q_FACTOR;
    calData_Audio[1696] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_09_TYPE;
    calData_Audio[1697] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_09_FREQ;
    calData_Audio[1698] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_09_GAIN;
    calData_Audio[1699] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_09_Q_FACTOR;
    calData_Audio[1700] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_10_TYPE;
    calData_Audio[1701] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_10_FREQ;
    calData_Audio[1702] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_10_GAIN;
    calData_Audio[1703] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_10_Q_FACTOR;
    calData_Audio[1704] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_11_TYPE;
    calData_Audio[1705] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_11_FREQ;
    calData_Audio[1706] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_11_GAIN;
    calData_Audio[1707] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_11_Q_FACTOR;
    calData_Audio[1708] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_12_TYPE;
    calData_Audio[1709] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_12_FREQ;
    calData_Audio[1710] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_12_GAIN;
    calData_Audio[1711] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_FR_USB_12_Q_FACTOR;
    calData_Audio[1712] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_01_TYPE;
    calData_Audio[1713] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_01_FREQ;
    calData_Audio[1714] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_01_GAIN;
    calData_Audio[1715] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_01_Q_FACTOR;
    calData_Audio[1716] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_02_TYPE;
    calData_Audio[1717] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_02_FREQ;
    calData_Audio[1718] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_02_GAIN;
    calData_Audio[1719] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_02_Q_FACTOR;
    calData_Audio[1720] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_03_TYPE;
    calData_Audio[1721] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_03_FREQ;
    calData_Audio[1722] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_03_GAIN;
    calData_Audio[1723] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_03_Q_FACTOR;
    calData_Audio[1724] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_04_TYPE;
    calData_Audio[1725] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_04_FREQ;
    calData_Audio[1726] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_04_GAIN;
    calData_Audio[1727] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_04_Q_FACTOR;
    calData_Audio[1728] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_05_TYPE;
    calData_Audio[1729] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_05_FREQ;
    calData_Audio[1730] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_05_GAIN;
    calData_Audio[1731] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_05_Q_FACTOR;
    calData_Audio[1732] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_06_TYPE;
    calData_Audio[1733] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_06_FREQ;
    calData_Audio[1734] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_06_GAIN;
    calData_Audio[1735] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_06_Q_FACTOR;
    calData_Audio[1736] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_07_TYPE;
    calData_Audio[1737] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_07_FREQ;
    calData_Audio[1738] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_07_GAIN;
    calData_Audio[1739] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_07_Q_FACTOR;
    calData_Audio[1740] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_08_TYPE;
    calData_Audio[1741] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_08_FREQ;
    calData_Audio[1742] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_08_GAIN;
    calData_Audio[1743] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_08_Q_FACTOR;
    calData_Audio[1744] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_09_TYPE;
    calData_Audio[1745] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_09_FREQ;
    calData_Audio[1746] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_09_GAIN;
    calData_Audio[1747] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_09_Q_FACTOR;
    calData_Audio[1748] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_10_TYPE;
    calData_Audio[1749] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_10_FREQ;
    calData_Audio[1750] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_10_GAIN;
    calData_Audio[1751] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_10_Q_FACTOR;
    calData_Audio[1752] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_11_TYPE;
    calData_Audio[1753] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_11_FREQ;
    calData_Audio[1754] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_11_GAIN;
    calData_Audio[1755] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_11_Q_FACTOR;
    calData_Audio[1756] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_12_TYPE;
    calData_Audio[1757] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_12_FREQ;
    calData_Audio[1758] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_12_GAIN;
    calData_Audio[1759] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DEF_12_Q_FACTOR;
    calData_Audio[1760] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_01_TYPE;
    calData_Audio[1761] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_01_FREQ;
    calData_Audio[1762] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_01_GAIN;
    calData_Audio[1763] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_01_Q_FACTOR;
    calData_Audio[1764] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_02_TYPE;
    calData_Audio[1765] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_02_FREQ;
    calData_Audio[1766] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_02_GAIN;
    calData_Audio[1767] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_02_Q_FACTOR;
    calData_Audio[1768] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_03_TYPE;
    calData_Audio[1769] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_03_FREQ;
    calData_Audio[1770] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_03_GAIN;
    calData_Audio[1771] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_03_Q_FACTOR;
    calData_Audio[1772] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_04_TYPE;
    calData_Audio[1773] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_04_FREQ;
    calData_Audio[1774] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_04_GAIN;
    calData_Audio[1775] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_04_Q_FACTOR;
    calData_Audio[1776] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_05_TYPE;
    calData_Audio[1777] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_05_FREQ;
    calData_Audio[1778] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_05_GAIN;
    calData_Audio[1779] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_05_Q_FACTOR;
    calData_Audio[1780] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_06_TYPE;
    calData_Audio[1781] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_06_FREQ;
    calData_Audio[1782] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_06_GAIN;
    calData_Audio[1783] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_06_Q_FACTOR;
    calData_Audio[1784] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_07_TYPE;
    calData_Audio[1785] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_07_FREQ;
    calData_Audio[1786] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_07_GAIN;
    calData_Audio[1787] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_07_Q_FACTOR;
    calData_Audio[1788] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_08_TYPE;
    calData_Audio[1789] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_08_FREQ;
    calData_Audio[1790] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_08_GAIN;
    calData_Audio[1791] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_08_Q_FACTOR;
    calData_Audio[1792] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_09_TYPE;
    calData_Audio[1793] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_09_FREQ;
    calData_Audio[1794] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_09_GAIN;
    calData_Audio[1795] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_09_Q_FACTOR;
    calData_Audio[1796] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_10_TYPE;
    calData_Audio[1797] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_10_FREQ;
    calData_Audio[1798] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_10_GAIN;
    calData_Audio[1799] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_10_Q_FACTOR;
    calData_Audio[1800] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_11_TYPE;
    calData_Audio[1801] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_11_FREQ;
    calData_Audio[1802] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_11_GAIN;
    calData_Audio[1803] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_11_Q_FACTOR;
    calData_Audio[1804] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_12_TYPE;
    calData_Audio[1805] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_12_FREQ;
    calData_Audio[1806] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_12_GAIN;
    calData_Audio[1807] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_AM_12_Q_FACTOR;
    calData_Audio[1808] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_01_TYPE;
    calData_Audio[1809] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_01_FREQ;
    calData_Audio[1810] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_01_GAIN;
    calData_Audio[1811] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_01_Q_FACTOR;
    calData_Audio[1812] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_02_TYPE;
    calData_Audio[1813] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_02_FREQ;
    calData_Audio[1814] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_02_GAIN;
    calData_Audio[1815] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_02_Q_FACTOR;
    calData_Audio[1816] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_03_TYPE;
    calData_Audio[1817] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_03_FREQ;
    calData_Audio[1818] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_03_GAIN;
    calData_Audio[1819] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_03_Q_FACTOR;
    calData_Audio[1820] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_04_TYPE;
    calData_Audio[1821] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_04_FREQ;
    calData_Audio[1822] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_04_GAIN;
    calData_Audio[1823] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_04_Q_FACTOR;
    calData_Audio[1824] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_05_TYPE;
    calData_Audio[1825] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_05_FREQ;
    calData_Audio[1826] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_05_GAIN;
    calData_Audio[1827] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_05_Q_FACTOR;
    calData_Audio[1828] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_06_TYPE;
    calData_Audio[1829] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_06_FREQ;
    calData_Audio[1830] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_06_GAIN;
    calData_Audio[1831] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_06_Q_FACTOR;
    calData_Audio[1832] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_07_TYPE;
    calData_Audio[1833] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_07_FREQ;
    calData_Audio[1834] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_07_GAIN;
    calData_Audio[1835] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_07_Q_FACTOR;
    calData_Audio[1836] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_08_TYPE;
    calData_Audio[1837] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_08_FREQ;
    calData_Audio[1838] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_08_GAIN;
    calData_Audio[1839] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_08_Q_FACTOR;
    calData_Audio[1840] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_09_TYPE;
    calData_Audio[1841] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_09_FREQ;
    calData_Audio[1842] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_09_GAIN;
    calData_Audio[1843] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_09_Q_FACTOR;
    calData_Audio[1844] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_10_TYPE;
    calData_Audio[1845] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_10_FREQ;
    calData_Audio[1846] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_10_GAIN;
    calData_Audio[1847] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_10_Q_FACTOR;
    calData_Audio[1848] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_11_TYPE;
    calData_Audio[1849] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_11_FREQ;
    calData_Audio[1850] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_11_GAIN;
    calData_Audio[1851] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_11_Q_FACTOR;
    calData_Audio[1852] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_12_TYPE;
    calData_Audio[1853] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_12_FREQ;
    calData_Audio[1854] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_12_GAIN;
    calData_Audio[1855] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_FM_12_Q_FACTOR;
    calData_Audio[1856] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_01_TYPE;
    calData_Audio[1857] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_01_FREQ;
    calData_Audio[1858] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_01_GAIN;
    calData_Audio[1859] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_01_Q_FACTOR;
    calData_Audio[1860] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_02_TYPE;
    calData_Audio[1861] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_02_FREQ;
    calData_Audio[1862] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_02_GAIN;
    calData_Audio[1863] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_02_Q_FACTOR;
    calData_Audio[1864] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_03_TYPE;
    calData_Audio[1865] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_03_FREQ;
    calData_Audio[1866] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_03_GAIN;
    calData_Audio[1867] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_03_Q_FACTOR;
    calData_Audio[1868] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_04_TYPE;
    calData_Audio[1869] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_04_FREQ;
    calData_Audio[1870] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_04_GAIN;
    calData_Audio[1871] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_04_Q_FACTOR;
    calData_Audio[1872] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_05_TYPE;
    calData_Audio[1873] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_05_FREQ;
    calData_Audio[1874] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_05_GAIN;
    calData_Audio[1875] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_05_Q_FACTOR;
    calData_Audio[1876] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_06_TYPE;
    calData_Audio[1877] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_06_FREQ;
    calData_Audio[1878] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_06_GAIN;
    calData_Audio[1879] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_06_Q_FACTOR;
    calData_Audio[1880] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_07_TYPE;
    calData_Audio[1881] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_07_FREQ;
    calData_Audio[1882] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_07_GAIN;
    calData_Audio[1883] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_07_Q_FACTOR;
    calData_Audio[1884] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_08_TYPE;
    calData_Audio[1885] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_08_FREQ;
    calData_Audio[1886] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_08_GAIN;
    calData_Audio[1887] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_08_Q_FACTOR;
    calData_Audio[1888] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_09_TYPE;
    calData_Audio[1889] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_09_FREQ;
    calData_Audio[1890] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_09_GAIN;
    calData_Audio[1891] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_09_Q_FACTOR;
    calData_Audio[1892] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_10_TYPE;
    calData_Audio[1893] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_10_FREQ;
    calData_Audio[1894] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_10_GAIN;
    calData_Audio[1895] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_10_Q_FACTOR;
    calData_Audio[1896] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_11_TYPE;
    calData_Audio[1897] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_11_FREQ;
    calData_Audio[1898] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_11_GAIN;
    calData_Audio[1899] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_11_Q_FACTOR;
    calData_Audio[1900] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_12_TYPE;
    calData_Audio[1901] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_12_FREQ;
    calData_Audio[1902] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_12_GAIN;
    calData_Audio[1903] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_DAB_12_Q_FACTOR;
    calData_Audio[1904] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_01_TYPE;
    calData_Audio[1905] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_01_FREQ;
    calData_Audio[1906] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_01_GAIN;
    calData_Audio[1907] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_01_Q_FACTOR;
    calData_Audio[1908] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_02_TYPE;
    calData_Audio[1909] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_02_FREQ;
    calData_Audio[1910] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_02_GAIN;
    calData_Audio[1911] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_02_Q_FACTOR;
    calData_Audio[1912] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_03_TYPE;
    calData_Audio[1913] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_03_FREQ;
    calData_Audio[1914] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_03_GAIN;
    calData_Audio[1915] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_03_Q_FACTOR;
    calData_Audio[1916] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_04_TYPE;
    calData_Audio[1917] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_04_FREQ;
    calData_Audio[1918] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_04_GAIN;
    calData_Audio[1919] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_04_Q_FACTOR;
    calData_Audio[1920] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_05_TYPE;
    calData_Audio[1921] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_05_FREQ;
    calData_Audio[1922] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_05_GAIN;
    calData_Audio[1923] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_05_Q_FACTOR;
    calData_Audio[1924] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_06_TYPE;
    calData_Audio[1925] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_06_FREQ;
    calData_Audio[1926] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_06_GAIN;
    calData_Audio[1927] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_06_Q_FACTOR;
    calData_Audio[1928] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_07_TYPE;
    calData_Audio[1929] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_07_FREQ;
    calData_Audio[1930] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_07_GAIN;
    calData_Audio[1931] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_07_Q_FACTOR;
    calData_Audio[1932] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_08_TYPE;
    calData_Audio[1933] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_08_FREQ;
    calData_Audio[1934] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_08_GAIN;
    calData_Audio[1935] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_08_Q_FACTOR;
    calData_Audio[1936] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_09_TYPE;
    calData_Audio[1937] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_09_FREQ;
    calData_Audio[1938] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_09_GAIN;
    calData_Audio[1939] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_09_Q_FACTOR;
    calData_Audio[1940] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_10_TYPE;
    calData_Audio[1941] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_10_FREQ;
    calData_Audio[1942] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_10_GAIN;
    calData_Audio[1943] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_10_Q_FACTOR;
    calData_Audio[1944] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_11_TYPE;
    calData_Audio[1945] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_11_FREQ;
    calData_Audio[1946] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_11_GAIN;
    calData_Audio[1947] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_11_Q_FACTOR;
    calData_Audio[1948] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_12_TYPE;
    calData_Audio[1949] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_12_FREQ;
    calData_Audio[1950] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_12_GAIN;
    calData_Audio[1951] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_PROJ_12_Q_FACTOR;
    calData_Audio[1952] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_01_TYPE;
    calData_Audio[1953] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_01_FREQ;
    calData_Audio[1954] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_01_GAIN;
    calData_Audio[1955] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_01_Q_FACTOR;
    calData_Audio[1956] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_02_TYPE;
    calData_Audio[1957] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_02_FREQ;
    calData_Audio[1958] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_02_GAIN;
    calData_Audio[1959] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_02_Q_FACTOR;
    calData_Audio[1960] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_03_TYPE;
    calData_Audio[1961] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_03_FREQ;
    calData_Audio[1962] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_03_GAIN;
    calData_Audio[1963] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_03_Q_FACTOR;
    calData_Audio[1964] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_04_TYPE;
    calData_Audio[1965] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_04_FREQ;
    calData_Audio[1966] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_04_GAIN;
    calData_Audio[1967] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_04_Q_FACTOR;
    calData_Audio[1968] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_05_TYPE;
    calData_Audio[1969] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_05_FREQ;
    calData_Audio[1970] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_05_GAIN;
    calData_Audio[1971] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_05_Q_FACTOR;
    calData_Audio[1972] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_06_TYPE;
    calData_Audio[1973] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_06_FREQ;
    calData_Audio[1974] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_06_GAIN;
    calData_Audio[1975] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_06_Q_FACTOR;
    calData_Audio[1976] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_07_TYPE;
    calData_Audio[1977] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_07_FREQ;
    calData_Audio[1978] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_07_GAIN;
    calData_Audio[1979] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_07_Q_FACTOR;
    calData_Audio[1980] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_08_TYPE;
    calData_Audio[1981] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_08_FREQ;
    calData_Audio[1982] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_08_GAIN;
    calData_Audio[1983] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_08_Q_FACTOR;
    calData_Audio[1984] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_09_TYPE;
    calData_Audio[1985] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_09_FREQ;
    calData_Audio[1986] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_09_GAIN;
    calData_Audio[1987] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_09_Q_FACTOR;
    calData_Audio[1988] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_10_TYPE;
    calData_Audio[1989] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_10_FREQ;
    calData_Audio[1990] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_10_GAIN;
    calData_Audio[1991] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_10_Q_FACTOR;
    calData_Audio[1992] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_11_TYPE;
    calData_Audio[1993] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_11_FREQ;
    calData_Audio[1994] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_11_GAIN;
    calData_Audio[1995] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_11_Q_FACTOR;
    calData_Audio[1996] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_12_TYPE;
    calData_Audio[1997] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_12_FREQ;
    calData_Audio[1998] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_12_GAIN;
    calData_Audio[1999] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_BTSA_12_Q_FACTOR;
    calData_Audio[2000] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_01_TYPE;
    calData_Audio[2001] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_01_FREQ;
    calData_Audio[2002] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_01_GAIN;
    calData_Audio[2003] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_01_Q_FACTOR;
    calData_Audio[2004] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_02_TYPE;
    calData_Audio[2005] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_02_FREQ;
    calData_Audio[2006] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_02_GAIN;
    calData_Audio[2007] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_02_Q_FACTOR;
    calData_Audio[2008] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_03_TYPE;
    calData_Audio[2009] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_03_FREQ;
    calData_Audio[2010] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_03_GAIN;
    calData_Audio[2011] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_03_Q_FACTOR;
    calData_Audio[2012] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_04_TYPE;
    calData_Audio[2013] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_04_FREQ;
    calData_Audio[2014] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_04_GAIN;
    calData_Audio[2015] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_04_Q_FACTOR;
    calData_Audio[2016] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_05_TYPE;
    calData_Audio[2017] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_05_FREQ;
    calData_Audio[2018] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_05_GAIN;
    calData_Audio[2019] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_05_Q_FACTOR;
    calData_Audio[2020] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_06_TYPE;
    calData_Audio[2021] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_06_FREQ;
    calData_Audio[2022] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_06_GAIN;
    calData_Audio[2023] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_06_Q_FACTOR;
    calData_Audio[2024] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_07_TYPE;
    calData_Audio[2025] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_07_FREQ;
    calData_Audio[2026] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_07_GAIN;
    calData_Audio[2027] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_07_Q_FACTOR;
    calData_Audio[2028] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_08_TYPE;
    calData_Audio[2029] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_08_FREQ;
    calData_Audio[2030] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_08_GAIN;
    calData_Audio[2031] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_08_Q_FACTOR;
    calData_Audio[2032] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_09_TYPE;
    calData_Audio[2033] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_09_FREQ;
    calData_Audio[2034] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_09_GAIN;
    calData_Audio[2035] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_09_Q_FACTOR;
    calData_Audio[2036] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_10_TYPE;
    calData_Audio[2037] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_10_FREQ;
    calData_Audio[2038] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_10_GAIN;
    calData_Audio[2039] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_10_Q_FACTOR;
    calData_Audio[2040] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_11_TYPE;
    calData_Audio[2041] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_11_FREQ;
    calData_Audio[2042] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_11_GAIN;
    calData_Audio[2043] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_11_Q_FACTOR;
    calData_Audio[2044] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_12_TYPE;
    calData_Audio[2045] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_12_FREQ;
    calData_Audio[2046] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_12_GAIN;
    calData_Audio[2047] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RR_USB_12_Q_FACTOR;
    calData_Audio[2048] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_01_TYPE;
    calData_Audio[2049] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_01_FREQ;
    calData_Audio[2050] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_01_GAIN;
    calData_Audio[2051] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_01_Q_FACTOR;
    calData_Audio[2052] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_02_TYPE;
    calData_Audio[2053] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_02_FREQ;
    calData_Audio[2054] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_02_GAIN;
    calData_Audio[2055] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_02_Q_FACTOR;
    calData_Audio[2056] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_03_TYPE;
    calData_Audio[2057] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_03_FREQ;
    calData_Audio[2058] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_03_GAIN;
    calData_Audio[2059] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_03_Q_FACTOR;
    calData_Audio[2060] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_04_TYPE;
    calData_Audio[2061] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_04_FREQ;
    calData_Audio[2062] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_04_GAIN;
    calData_Audio[2063] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_04_Q_FACTOR;
    calData_Audio[2064] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_05_TYPE;
    calData_Audio[2065] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_05_FREQ;
    calData_Audio[2066] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_05_GAIN;
    calData_Audio[2067] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_05_Q_FACTOR;
    calData_Audio[2068] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_06_TYPE;
    calData_Audio[2069] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_06_FREQ;
    calData_Audio[2070] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_06_GAIN;
    calData_Audio[2071] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_06_Q_FACTOR;
    calData_Audio[2072] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_07_TYPE;
    calData_Audio[2073] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_07_FREQ;
    calData_Audio[2074] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_07_GAIN;
    calData_Audio[2075] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_07_Q_FACTOR;
    calData_Audio[2076] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_08_TYPE;
    calData_Audio[2077] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_08_FREQ;
    calData_Audio[2078] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_08_GAIN;
    calData_Audio[2079] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_08_Q_FACTOR;
    calData_Audio[2080] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_09_TYPE;
    calData_Audio[2081] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_09_FREQ;
    calData_Audio[2082] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_09_GAIN;
    calData_Audio[2083] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_09_Q_FACTOR;
    calData_Audio[2084] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_10_TYPE;
    calData_Audio[2085] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_10_FREQ;
    calData_Audio[2086] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_10_GAIN;
    calData_Audio[2087] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_10_Q_FACTOR;
    calData_Audio[2088] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_11_TYPE;
    calData_Audio[2089] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_11_FREQ;
    calData_Audio[2090] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_11_GAIN;
    calData_Audio[2091] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_11_Q_FACTOR;
    calData_Audio[2092] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_12_TYPE;
    calData_Audio[2093] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_12_FREQ;
    calData_Audio[2094] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_12_GAIN;
    calData_Audio[2095] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DEF_12_Q_FACTOR;
    calData_Audio[2096] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_01_TYPE;
    calData_Audio[2097] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_01_FREQ;
    calData_Audio[2098] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_01_GAIN;
    calData_Audio[2099] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_01_Q_FACTOR;
    calData_Audio[2100] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_02_TYPE;
    calData_Audio[2101] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_02_FREQ;
    calData_Audio[2102] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_02_GAIN;
    calData_Audio[2103] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_02_Q_FACTOR;
    calData_Audio[2104] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_03_TYPE;
    calData_Audio[2105] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_03_FREQ;
    calData_Audio[2106] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_03_GAIN;
    calData_Audio[2107] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_03_Q_FACTOR;
    calData_Audio[2108] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_04_TYPE;
    calData_Audio[2109] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_04_FREQ;
    calData_Audio[2110] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_04_GAIN;
    calData_Audio[2111] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_04_Q_FACTOR;
    calData_Audio[2112] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_05_TYPE;
    calData_Audio[2113] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_05_FREQ;
    calData_Audio[2114] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_05_GAIN;
    calData_Audio[2115] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_05_Q_FACTOR;
    calData_Audio[2116] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_06_TYPE;
    calData_Audio[2117] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_06_FREQ;
    calData_Audio[2118] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_06_GAIN;
    calData_Audio[2119] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_06_Q_FACTOR;
    calData_Audio[2120] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_07_TYPE;
    calData_Audio[2121] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_07_FREQ;
    calData_Audio[2122] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_07_GAIN;
    calData_Audio[2123] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_07_Q_FACTOR;
    calData_Audio[2124] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_08_TYPE;
    calData_Audio[2125] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_08_FREQ;
    calData_Audio[2126] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_08_GAIN;
    calData_Audio[2127] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_08_Q_FACTOR;
    calData_Audio[2128] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_09_TYPE;
    calData_Audio[2129] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_09_FREQ;
    calData_Audio[2130] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_09_GAIN;
    calData_Audio[2131] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_09_Q_FACTOR;
    calData_Audio[2132] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_10_TYPE;
    calData_Audio[2133] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_10_FREQ;
    calData_Audio[2134] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_10_GAIN;
    calData_Audio[2135] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_10_Q_FACTOR;
    calData_Audio[2136] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_11_TYPE;
    calData_Audio[2137] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_11_FREQ;
    calData_Audio[2138] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_11_GAIN;
    calData_Audio[2139] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_11_Q_FACTOR;
    calData_Audio[2140] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_12_TYPE;
    calData_Audio[2141] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_12_FREQ;
    calData_Audio[2142] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_12_GAIN;
    calData_Audio[2143] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_AM_12_Q_FACTOR;
    calData_Audio[2144] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_01_TYPE;
    calData_Audio[2145] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_01_FREQ;
    calData_Audio[2146] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_01_GAIN;
    calData_Audio[2147] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_01_Q_FACTOR;
    calData_Audio[2148] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_02_TYPE;
    calData_Audio[2149] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_02_FREQ;
    calData_Audio[2150] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_02_GAIN;
    calData_Audio[2151] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_02_Q_FACTOR;
    calData_Audio[2152] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_03_TYPE;
    calData_Audio[2153] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_03_FREQ;
    calData_Audio[2154] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_03_GAIN;
    calData_Audio[2155] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_03_Q_FACTOR;
    calData_Audio[2156] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_04_TYPE;
    calData_Audio[2157] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_04_FREQ;
    calData_Audio[2158] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_04_GAIN;
    calData_Audio[2159] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_04_Q_FACTOR;
    calData_Audio[2160] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_05_TYPE;
    calData_Audio[2161] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_05_FREQ;
    calData_Audio[2162] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_05_GAIN;
    calData_Audio[2163] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_05_Q_FACTOR;
    calData_Audio[2164] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_06_TYPE;
    calData_Audio[2165] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_06_FREQ;
    calData_Audio[2166] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_06_GAIN;
    calData_Audio[2167] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_06_Q_FACTOR;
    calData_Audio[2168] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_07_TYPE;
    calData_Audio[2169] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_07_FREQ;
    calData_Audio[2170] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_07_GAIN;
    calData_Audio[2171] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_07_Q_FACTOR;
    calData_Audio[2172] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_08_TYPE;
    calData_Audio[2173] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_08_FREQ;
    calData_Audio[2174] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_08_GAIN;
    calData_Audio[2175] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_08_Q_FACTOR;
    calData_Audio[2176] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_09_TYPE;
    calData_Audio[2177] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_09_FREQ;
    calData_Audio[2178] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_09_GAIN;
    calData_Audio[2179] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_09_Q_FACTOR;
    calData_Audio[2180] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_10_TYPE;
    calData_Audio[2181] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_10_FREQ;
    calData_Audio[2182] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_10_GAIN;
    calData_Audio[2183] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_10_Q_FACTOR;
    calData_Audio[2184] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_11_TYPE;
    calData_Audio[2185] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_11_FREQ;
    calData_Audio[2186] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_11_GAIN;
    calData_Audio[2187] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_11_Q_FACTOR;
    calData_Audio[2188] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_12_TYPE;
    calData_Audio[2189] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_12_FREQ;
    calData_Audio[2190] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_12_GAIN;
    calData_Audio[2191] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_FM_12_Q_FACTOR;
    calData_Audio[2192] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_01_TYPE;
    calData_Audio[2193] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_01_FREQ;
    calData_Audio[2194] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_01_GAIN;
    calData_Audio[2195] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_01_Q_FACTOR;
    calData_Audio[2196] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_02_TYPE;
    calData_Audio[2197] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_02_FREQ;
    calData_Audio[2198] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_02_GAIN;
    calData_Audio[2199] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_02_Q_FACTOR;
    calData_Audio[2200] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_03_TYPE;
    calData_Audio[2201] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_03_FREQ;
    calData_Audio[2202] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_03_GAIN;
    calData_Audio[2203] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_03_Q_FACTOR;
    calData_Audio[2204] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_04_TYPE;
    calData_Audio[2205] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_04_FREQ;
    calData_Audio[2206] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_04_GAIN;
    calData_Audio[2207] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_04_Q_FACTOR;
    calData_Audio[2208] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_05_TYPE;
    calData_Audio[2209] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_05_FREQ;
    calData_Audio[2210] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_05_GAIN;
    calData_Audio[2211] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_05_Q_FACTOR;
    calData_Audio[2212] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_06_TYPE;
    calData_Audio[2213] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_06_FREQ;
    calData_Audio[2214] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_06_GAIN;
    calData_Audio[2215] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_06_Q_FACTOR;
    calData_Audio[2216] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_07_TYPE;
    calData_Audio[2217] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_07_FREQ;
    calData_Audio[2218] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_07_GAIN;
    calData_Audio[2219] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_07_Q_FACTOR;
    calData_Audio[2220] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_08_TYPE;
    calData_Audio[2221] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_08_FREQ;
    calData_Audio[2222] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_08_GAIN;
    calData_Audio[2223] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_08_Q_FACTOR;
    calData_Audio[2224] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_09_TYPE;
    calData_Audio[2225] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_09_FREQ;
    calData_Audio[2226] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_09_GAIN;
    calData_Audio[2227] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_09_Q_FACTOR;
    calData_Audio[2228] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_10_TYPE;
    calData_Audio[2229] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_10_FREQ;
    calData_Audio[2230] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_10_GAIN;
    calData_Audio[2231] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_10_Q_FACTOR;
    calData_Audio[2232] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_11_TYPE;
    calData_Audio[2233] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_11_FREQ;
    calData_Audio[2234] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_11_GAIN;
    calData_Audio[2235] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_11_Q_FACTOR;
    calData_Audio[2236] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_12_TYPE;
    calData_Audio[2237] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_12_FREQ;
    calData_Audio[2238] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_12_GAIN;
    calData_Audio[2239] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_DAB_12_Q_FACTOR;
    calData_Audio[2240] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_01_TYPE;
    calData_Audio[2241] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_01_FREQ;
    calData_Audio[2242] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_01_GAIN;
    calData_Audio[2243] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_01_Q_FACTOR;
    calData_Audio[2244] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_02_TYPE;
    calData_Audio[2245] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_02_FREQ;
    calData_Audio[2246] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_02_GAIN;
    calData_Audio[2247] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_02_Q_FACTOR;
    calData_Audio[2248] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_03_TYPE;
    calData_Audio[2249] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_03_FREQ;
    calData_Audio[2250] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_03_GAIN;
    calData_Audio[2251] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_03_Q_FACTOR;
    calData_Audio[2252] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_04_TYPE;
    calData_Audio[2253] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_04_FREQ;
    calData_Audio[2254] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_04_GAIN;
    calData_Audio[2255] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_04_Q_FACTOR;
    calData_Audio[2256] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_05_TYPE;
    calData_Audio[2257] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_05_FREQ;
    calData_Audio[2258] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_05_GAIN;
    calData_Audio[2259] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_05_Q_FACTOR;
    calData_Audio[2260] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_06_TYPE;
    calData_Audio[2261] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_06_FREQ;
    calData_Audio[2262] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_06_GAIN;
    calData_Audio[2263] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_06_Q_FACTOR;
    calData_Audio[2264] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_07_TYPE;
    calData_Audio[2265] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_07_FREQ;
    calData_Audio[2266] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_07_GAIN;
    calData_Audio[2267] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_07_Q_FACTOR;
    calData_Audio[2268] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_08_TYPE;
    calData_Audio[2269] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_08_FREQ;
    calData_Audio[2270] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_08_GAIN;
    calData_Audio[2271] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_08_Q_FACTOR;
    calData_Audio[2272] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_09_TYPE;
    calData_Audio[2273] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_09_FREQ;
    calData_Audio[2274] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_09_GAIN;
    calData_Audio[2275] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_09_Q_FACTOR;
    calData_Audio[2276] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_10_TYPE;
    calData_Audio[2277] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_10_FREQ;
    calData_Audio[2278] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_10_GAIN;
    calData_Audio[2279] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_10_Q_FACTOR;
    calData_Audio[2280] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_11_TYPE;
    calData_Audio[2281] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_11_FREQ;
    calData_Audio[2282] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_11_GAIN;
    calData_Audio[2283] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_11_Q_FACTOR;
    calData_Audio[2284] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_12_TYPE;
    calData_Audio[2285] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_12_FREQ;
    calData_Audio[2286] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_12_GAIN;
    calData_Audio[2287] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_PROJ_12_Q_FACTOR;
    calData_Audio[2288] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_01_TYPE;
    calData_Audio[2289] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_01_FREQ;
    calData_Audio[2290] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_01_GAIN;
    calData_Audio[2291] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_01_Q_FACTOR;
    calData_Audio[2292] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_02_TYPE;
    calData_Audio[2293] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_02_FREQ;
    calData_Audio[2294] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_02_GAIN;
    calData_Audio[2295] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_02_Q_FACTOR;
    calData_Audio[2296] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_03_TYPE;
    calData_Audio[2297] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_03_FREQ;
    calData_Audio[2298] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_03_GAIN;
    calData_Audio[2299] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_03_Q_FACTOR;
    calData_Audio[2300] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_04_TYPE;
    calData_Audio[2301] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_04_FREQ;
    calData_Audio[2302] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_04_GAIN;
    calData_Audio[2303] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_04_Q_FACTOR;
    calData_Audio[2304] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_05_TYPE;
    calData_Audio[2305] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_05_FREQ;
    calData_Audio[2306] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_05_GAIN;
    calData_Audio[2307] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_05_Q_FACTOR;
    calData_Audio[2308] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_06_TYPE;
    calData_Audio[2309] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_06_FREQ;
    calData_Audio[2310] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_06_GAIN;
    calData_Audio[2311] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_06_Q_FACTOR;
    calData_Audio[2312] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_07_TYPE;
    calData_Audio[2313] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_07_FREQ;
    calData_Audio[2314] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_07_GAIN;
    calData_Audio[2315] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_07_Q_FACTOR;
    calData_Audio[2316] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_08_TYPE;
    calData_Audio[2317] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_08_FREQ;
    calData_Audio[2318] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_08_GAIN;
    calData_Audio[2319] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_08_Q_FACTOR;
    calData_Audio[2320] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_09_TYPE;
    calData_Audio[2321] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_09_FREQ;
    calData_Audio[2322] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_09_GAIN;
    calData_Audio[2323] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_09_Q_FACTOR;
    calData_Audio[2324] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_10_TYPE;
    calData_Audio[2325] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_10_FREQ;
    calData_Audio[2326] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_10_GAIN;
    calData_Audio[2327] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_10_Q_FACTOR;
    calData_Audio[2328] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_11_TYPE;
    calData_Audio[2329] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_11_FREQ;
    calData_Audio[2330] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_11_GAIN;
    calData_Audio[2331] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_11_Q_FACTOR;
    calData_Audio[2332] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_12_TYPE;
    calData_Audio[2333] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_12_FREQ;
    calData_Audio[2334] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_12_GAIN;
    calData_Audio[2335] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_BTSA_12_Q_FACTOR;
    calData_Audio[2336] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_01_TYPE;
    calData_Audio[2337] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_01_FREQ;
    calData_Audio[2338] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_01_GAIN;
    calData_Audio[2339] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_01_Q_FACTOR;
    calData_Audio[2340] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_02_TYPE;
    calData_Audio[2341] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_02_FREQ;
    calData_Audio[2342] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_02_GAIN;
    calData_Audio[2343] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_02_Q_FACTOR;
    calData_Audio[2344] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_03_TYPE;
    calData_Audio[2345] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_03_FREQ;
    calData_Audio[2346] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_03_GAIN;
    calData_Audio[2347] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_03_Q_FACTOR;
    calData_Audio[2348] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_04_TYPE;
    calData_Audio[2349] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_04_FREQ;
    calData_Audio[2350] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_04_GAIN;
    calData_Audio[2351] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_04_Q_FACTOR;
    calData_Audio[2352] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_05_TYPE;
    calData_Audio[2353] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_05_FREQ;
    calData_Audio[2354] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_05_GAIN;
    calData_Audio[2355] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_05_Q_FACTOR;
    calData_Audio[2356] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_06_TYPE;
    calData_Audio[2357] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_06_FREQ;
    calData_Audio[2358] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_06_GAIN;
    calData_Audio[2359] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_06_Q_FACTOR;
    calData_Audio[2360] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_07_TYPE;
    calData_Audio[2361] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_07_FREQ;
    calData_Audio[2362] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_07_GAIN;
    calData_Audio[2363] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_07_Q_FACTOR;
    calData_Audio[2364] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_08_TYPE;
    calData_Audio[2365] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_08_FREQ;
    calData_Audio[2366] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_08_GAIN;
    calData_Audio[2367] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_08_Q_FACTOR;
    calData_Audio[2368] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_09_TYPE;
    calData_Audio[2369] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_09_FREQ;
    calData_Audio[2370] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_09_GAIN;
    calData_Audio[2371] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_09_Q_FACTOR;
    calData_Audio[2372] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_10_TYPE;
    calData_Audio[2373] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_10_FREQ;
    calData_Audio[2374] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_10_GAIN;
    calData_Audio[2375] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_10_Q_FACTOR;
    calData_Audio[2376] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_11_TYPE;
    calData_Audio[2377] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_11_FREQ;
    calData_Audio[2378] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_11_GAIN;
    calData_Audio[2379] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_11_Q_FACTOR;
    calData_Audio[2380] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_12_TYPE;
    calData_Audio[2381] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_12_FREQ;
    calData_Audio[2382] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_12_GAIN;
    calData_Audio[2383] = Cal_Union_Parameters.Cal_Parameter.EQ_FILT_RL_USB_12_Q_FACTOR;

   notifyCalDataUpdated_Audio(calData_Audio);

  calData_Camera[0] = Cal_Union_Parameters.Cal_Parameter.CAM_OFF_DELAY_DURATION;
  calData_Camera[1] = Cal_Union_Parameters.Cal_Parameter.CAM_OFF_DELAY_VEL_LIM;
  calData_Camera[2] = Cal_Union_Parameters.Cal_Parameter.FAST_MANEUVER_THRESHOLD;
  calData_Camera[3] = Cal_Union_Parameters.Cal_Parameter.FAST_MANEUVER_DELAY;
  calData_Camera[4] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_APP_IGN_OFF;
  calData_Camera[5] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_APP_STANDBY_DURATION;
  calData_Camera[6] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_APP_MIN_RUN_TIME;
  calData_Camera[7] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_APP_MAX_VEL;
  calData_Camera[8] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_APP_VEL_HIST; 
  calData_Camera[9] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_IGN_OFF_MAX_TIMER;
  calData_Camera[10] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_MOTION_ESTM_ENABLED;
  calData_Camera[11] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_USER_CLOSE_ALLOWED;
  calData_Camera[12] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_MUTE_ENABLED;
  calData_Camera[13] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_MUTE_SWS;
  calData_Camera[14] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_INTERP_DURATION;
  calData_Camera[15] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_VIEW_2_MODE;
  calData_Camera[16] = Cal_Union_Parameters.Cal_Parameter.PARK_ASSIST_AUTO_ZOOM_PUSH;
  calData_Camera[17] = Cal_Union_Parameters.Cal_Parameter.ARVC_ACE_ENABLED;
  calData_Camera[18] = Cal_Union_Parameters.Cal_Parameter.ARVC_ACE_RESPONSE_SPEED;
  calData_Camera[19] = Cal_Union_Parameters.Cal_Parameter.ARVC_ACE_CHROMA_MAX;
  calData_Camera[20] = Cal_Union_Parameters.Cal_Parameter.ARVC_ACE_CHROMA_GAIN;
  calData_Camera[21] = Cal_Union_Parameters.Cal_Parameter.ARVC_ACE_LUMA_GAIN;
  calData_Camera[22] = Cal_Union_Parameters.Cal_Parameter.ARVC_ACE_GAMMA_GAIN;
  calData_Camera[23] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_CONTRAST_ADJ;
  calData_Camera[24] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_BRIGHTNESS_ADJ;
  calData_Camera[25] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_HUE_ADJ;
  calData_Camera[26] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_CR_SAT_ADJ;
  calData_Camera[27] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_CR_OFFSET_ADJ;
  calData_Camera[28] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_CB_SAT_ADJ;
  calData_Camera[29] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_CB_OFFSET_ADJ;
  calData_Camera[30] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_LUMA_GAIN_MODE;
  calData_Camera[31] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_LUMA_GAIN_SPEED;
  calData_Camera[32] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_LUMA_MANUAL_GAIN;
  calData_Camera[33] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_CHROMA_GAIN_MODE;
  calData_Camera[34] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_CHROMA_GAIN_SPEED;
  calData_Camera[35] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_CHROMA_MANUAL_GAIN;
  calData_Camera[36] = Cal_Union_Parameters.Cal_Parameter.ARVC_DEC_LUMA_PEAKING_GAIN;
  calData_Camera[37] = Cal_Union_Parameters.Cal_Parameter.DRVC_MAX_TRY_COUNT;
  calData_Camera[38] = Cal_Union_Parameters.Cal_Parameter.DRVC_TRY_DELAY;
  calData_Camera[39] = Cal_Union_Parameters.Cal_Parameter.RVC_MIRROR_HOR_IMG;
  calData_Camera[40] = Cal_Union_Parameters.Cal_Parameter.RVC_MIRROR_VER_IMG;
  calData_Camera[41] = Cal_Union_Parameters.Cal_Parameter.RVC_GAMMA_CORRECTION_R;
  calData_Camera[42] = Cal_Union_Parameters.Cal_Parameter.RVC_GAMMA_CORRECTION_G;
  calData_Camera[43] = Cal_Union_Parameters.Cal_Parameter.RVC_GAMMA_CORRECTION_B;
  calData_Camera[44] = Cal_Union_Parameters.Cal_Parameter.RVC_HUE_CORRECTION;
  calData_Camera[45] = Cal_Union_Parameters.Cal_Parameter.RVC_SATURATION_CORRECTION;
  calData_Camera[46] = Cal_Union_Parameters.Cal_Parameter.RVC_VALUE_CORRECTION;
  calData_Camera[47] = Cal_Union_Parameters.Cal_Parameter.RVC_RED_CH_GAIN;
  calData_Camera[48] = Cal_Union_Parameters.Cal_Parameter.RVC_RED_CH_OFFSET;
  calData_Camera[49] = Cal_Union_Parameters.Cal_Parameter.RVC_GREEN_CH_GAIN;
  calData_Camera[50] = Cal_Union_Parameters.Cal_Parameter.RVC_GREEN_CH_OFFSET;
  calData_Camera[51] = Cal_Union_Parameters.Cal_Parameter.RVC_BLUE_CH_GAIN;
  calData_Camera[52] = Cal_Union_Parameters.Cal_Parameter.RVC_BLUE_CH_OFFSET;
  calData_Camera[53] = Cal_Union_Parameters.Cal_Parameter.RVC_NOISE_FILTER_ENABLED;
  calData_Camera[54] = Cal_Union_Parameters.Cal_Parameter.RVC_CONTRAST_EQUALIZATION_ENABLED;
  calData_Camera[55] = Cal_Union_Parameters.Cal_Parameter.RVC_CONTRAST_EQUALIZATION_THRESHOLD;
  calData_Camera[56] = Cal_Union_Parameters.Cal_Parameter.RVC_CONTRAST_EQUALIZATION_GAMMA_CORR; 
  calData_Camera[57] = Cal_Union_Parameters.Cal_Parameter.RVC_STEP_N2_BRIGHTNESS;
  calData_Camera[58] = Cal_Union_Parameters.Cal_Parameter.RVC_STEP_N1_BRIGHTNESS;
  calData_Camera[59] = Cal_Union_Parameters.Cal_Parameter.RVC_STEP_0_BRIGHTNESS;
  calData_Camera[60] = Cal_Union_Parameters.Cal_Parameter.RVC_STEP_P1_BRIGHTNESS;
  calData_Camera[61] = Cal_Union_Parameters.Cal_Parameter.RVC_STEP_P2_BRIGHTNESS;
  calData_Camera[62] = Cal_Union_Parameters.Cal_Parameter.RVC_STEP_N2_CONTRAST;
  calData_Camera[63] = Cal_Union_Parameters.Cal_Parameter.RVC_STEP_N1_CONTRAST;
  calData_Camera[64] = Cal_Union_Parameters.Cal_Parameter.RVC_STEP_0_CONTRAST;
  calData_Camera[65] = Cal_Union_Parameters.Cal_Parameter.RVC_STEP_P1_CONTRAST;
  calData_Camera[66] = Cal_Union_Parameters.Cal_Parameter.RVC_STEP_P2_CONTRAST;
  calData_Camera[67] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_BRIGHTNESS;
  calData_Camera[68] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_CONTRAST;
  calData_Camera[69] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_ZOOM;
  calData_Camera[70] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_TOP_CROP;
  calData_Camera[71] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_BOTTOM_CROP;
  calData_Camera[72] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_LEFT_CROP;
  calData_Camera[73] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_RIGHT_CROP;
  calData_Camera[74] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_TOP_LEFT_X_DELTA;
  calData_Camera[75] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_TOP_LEFT_Y_DELTA;
  calData_Camera[76] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_TOP_RIGHT_X_DELTA;
  calData_Camera[77] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_TOP_RIGHT_Y_DELTA;
  calData_Camera[78] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_BOTTOM_LEFT_X_DELTA;
  calData_Camera[79] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_BOTTOM_LEFT_Y_DELTA;
  calData_Camera[80] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_BOTTOM_RIGHT_X_DELTA;
  calData_Camera[81] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_1_BOTTOM_RIGHT_Y_DELTA;
  calData_Camera[82] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_BRIGHTNESS;
  calData_Camera[83] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_CONTRAST;
  calData_Camera[84] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_ZOOM;
  calData_Camera[85] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_TOP_CROP;
  calData_Camera[86] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_BOTTOM_CROP;
  calData_Camera[87] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_LEFT_CROP;
  calData_Camera[88] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_RIGHT_CROP;
  calData_Camera[89] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_TOP_LEFT_X_DELTA;
  calData_Camera[90] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_TOP_LEFT_Y_DELTA;
  calData_Camera[91] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_TOP_RIGHT_X_DELTA;
  calData_Camera[92] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_TOP_RIGHT_Y_DELTA;
  calData_Camera[93] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_BOTTOM_LEFT_X_DELTA;
  calData_Camera[94] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_BOTTOM_LEFT_Y_DELTA;
  calData_Camera[95] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_BOTTOM_RIGHT_X_DELTA;
  calData_Camera[96] = Cal_Union_Parameters.Cal_Parameter.RVC_VIEW_2_BOTTOM_RIGHT_Y_DELTA;
  calData_Camera[97] = Cal_Union_Parameters.Cal_Parameter.STEERING_COEFF;
  calData_Camera[98] = Cal_Union_Parameters.Cal_Parameter.WHEEL_BASE;
  calData_Camera[99] = Cal_Union_Parameters.Cal_Parameter.OVERHANG_SCALE_FACTOR;
  calData_Camera[100] = Cal_Union_Parameters.Cal_Parameter.VEHICLE_FRONT_OVERHANG;
  calData_Camera[101] = Cal_Union_Parameters.Cal_Parameter.VEH_MIRROR_TO_MIRROR_WIDTH;
  calData_Camera[102] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_REG1_RGBA_COLOR;
  calData_Camera[103] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_REG2_RGBA_COLOR;
  calData_Camera[104] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_REG3_RGBA_COLOR;
  calData_Camera[105] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_CENTERLINE_FILL_COLOR;
  calData_Camera[106] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_CENTERLINE_BORDER_COLOR;
  calData_Camera[107] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_SHOW_CENTERLINE;
  calData_Camera[108] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_NORMAL_OPACITY;
  calData_Camera[109] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_UNCERTAIN_OPACITY;
  calData_Camera[110] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_SNA_OPACITY;
  calData_Camera[111] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_REG1_LENGTH;
  calData_Camera[112] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_REG2_LENGTH;
  calData_Camera[113] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_REG3_LENGTH;
  calData_Camera[114] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_CENTERLINE_DIST;
  calData_Camera[115] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_RIGHT_ANGLE;
  calData_Camera[116] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_LEFT_ANGLE;
  calData_Camera[117] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_START_DIST;
  calData_Camera[118] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_BUMPER_X_POS;
  calData_Camera[119] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_THICKNESS;
  calData_Camera[120] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_REG1_NOTCH_WIDTH;
  calData_Camera[121] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_REG2_NOTCH_WIDTH;
  calData_Camera[122] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_REG3_NOTCH_WIDTH;
  calData_Camera[123] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_CENTERLINE_START;
  calData_Camera[124] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_CENTERLINE_THICKNESS;
  calData_Camera[125] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_CENTERLINE_BORDER_THICKNESS;
  calData_Camera[126] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_WIDTH_OFFSET;
  calData_Camera[127] = Cal_Union_Parameters.Cal_Parameter.RPAS_BAR_BLINK_RATE_1;
  calData_Camera[128] = Cal_Union_Parameters.Cal_Parameter.RPAS_BAR_BLINK_RATE_2;
  calData_Camera[129] = Cal_Union_Parameters.Cal_Parameter.RPAS_BAR_BLINK_RATE_3;
  calData_Camera[130] = Cal_Union_Parameters.Cal_Parameter.FPAS_BAR_BLINK_RATE_1;
  calData_Camera[131] = Cal_Union_Parameters.Cal_Parameter.FPAS_BAR_BLINK_RATE_2;
  calData_Camera[132] = Cal_Union_Parameters.Cal_Parameter.FPAS_BAR_BLINK_RATE_3;
  calData_Camera[133] = Cal_Union_Parameters.Cal_Parameter.PAS_CHIME_TYPE;
  calData_Camera[134] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE1_CHIME_ENABLED;
  calData_Camera[135] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE1_CHIME_RATE;
  calData_Camera[136] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE1_CHIME_VOL;
  calData_Camera[137] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE2_CHIME_ENABLED;
  calData_Camera[138] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE2_CHIME_RATE;
  calData_Camera[139] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE2_CHIME_VOL;
  calData_Camera[140] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE3_CHIME_ENABLED;
  calData_Camera[141] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE3_CHIME_RATE;
  calData_Camera[142] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE3_CHIME_VOL;
  calData_Camera[143] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE4_CHIME_ENABLED;
  calData_Camera[144] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE4_CHIME_RATE;
  calData_Camera[145] = Cal_Union_Parameters.Cal_Parameter.RPAS_ZONE4_CHIME_VOL;
  calData_Camera[146] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE1_CHIME_ENABLED;
  calData_Camera[147] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE1_CHIME_RATE;
  calData_Camera[148] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE1_CHIME_VOL;
  calData_Camera[149] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE2_CHIME_ENABLED;
  calData_Camera[150] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE2_CHIME_RATE;
  calData_Camera[151] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE2_CHIME_VOL;
  calData_Camera[152] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE3_CHIME_ENABLED;
  calData_Camera[153] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE3_CHIME_RATE;
  calData_Camera[154] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE3_CHIME_VOL;
  calData_Camera[155] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE4_CHIME_ENABLED;
  calData_Camera[156] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE4_CHIME_RATE;
  calData_Camera[157] = Cal_Union_Parameters.Cal_Parameter.FPAS_ZONE4_CHIME_VOL;
  calData_Camera[158] = Cal_Union_Parameters.Cal_Parameter.RVC_ENABLE_LENS_CORRECTION;
  calData_Camera[159] = Learn_Union.Learning_Data.CAM_CALIBRATION_STATUS;
  calData_Camera[160] = Cal_Union_Parameters.Cal_Parameter.CAM_INITIAL_POSE_X;
  calData_Camera[161] = Cal_Union_Parameters.Cal_Parameter.CAM_INITIAL_POSE_Y;
  calData_Camera[162] = Cal_Union_Parameters.Cal_Parameter.CAM_INITIAL_POSE_Z;
  calData_Camera[163] = Cal_Union_Parameters.Cal_Parameter.CAM_INITIAL_POSE_ROLL;
  calData_Camera[164] = Cal_Union_Parameters.Cal_Parameter.CAM_INITIAL_POSE_PITCH;
  calData_Camera[165] = Cal_Union_Parameters.Cal_Parameter.CAM_INITIAL_POSE_YAW;
  calData_Camera[166] = Cal_Union_Parameters.Cal_Parameter.CAM_INITIAL_FX;
  calData_Camera[167] = Cal_Union_Parameters.Cal_Parameter.CAM_INITIAL_FY;
  calData_Camera[168] = Cal_Union_Parameters.Cal_Parameter.CAM_INITIAL_CX;
  calData_Camera[169] = Cal_Union_Parameters.Cal_Parameter.CAM_INITIAL_CY;
  calData_Camera[170] = Learn_Union.Learning_Data.CAM_CALIBRATED_POSE_X;
  calData_Camera[171] = Learn_Union.Learning_Data.CAM_CALIBRATED_POSE_Y;
  calData_Camera[172] = Learn_Union.Learning_Data.CAM_CALIBRATED_POSE_Z;
  calData_Camera[173] = Learn_Union.Learning_Data.CAM_CALIBRATED_POSE_ROLL;
  calData_Camera[174] = Learn_Union.Learning_Data.CAM_CALIBRATED_POSE_PITCH;
  calData_Camera[175] = Learn_Union.Learning_Data.CAM_CALIBRATED_POSE_YAW;
  calData_Camera[176] = Learn_Union.Learning_Data.CAM_CALIBRATED_FX;
  calData_Camera[177] = Learn_Union.Learning_Data.CAM_CALIBRATED_FY;
  calData_Camera[178] = Learn_Union.Learning_Data.CAM_CALIBRATED_CX;
  calData_Camera[179] = Learn_Union.Learning_Data.CAM_CALIBRATED_CY;
  calData_Camera[180] = Learn_Union.Learning_Data.CAM_REFINED_FX;
  calData_Camera[181] = Learn_Union.Learning_Data.CAM_REFINED_FY;
  calData_Camera[182] = Learn_Union.Learning_Data.CAM_REFINED_CX;
  calData_Camera[183] = Learn_Union.Learning_Data.CAM_REFINED_CY;
  calData_Camera[184] = Learn_Union.Learning_Data.CAM_CALIBRATED_K1;
  calData_Camera[185] = Learn_Union.Learning_Data.CAM_CALIBRATED_K2;
  calData_Camera[186] = Learn_Union.Learning_Data.CAM_CALIBRATED_K3;
  calData_Camera[187] = Learn_Union.Learning_Data.CAM_CALIBRATED_K4;
  calData_Camera[188] = Learn_Union.Learning_Data.CAM_CALIBRATED_K5;
  calData_Camera[189] = Learn_Union.Learning_Data.CAM_CALIBRATED_K6;
  calData_Camera[190] = Learn_Union.Learning_Data.CAM_CALIBRATED_P1;
  calData_Camera[191] = Learn_Union.Learning_Data.CAM_CALIBRATED_P2;
  calData_Camera[192] = Learn_Union.Learning_Data.CAM_CALIBRATED_S1;
  calData_Camera[193] = Learn_Union.Learning_Data.CAM_CALIBRATED_S2;
  calData_Camera[194] = Learn_Union.Learning_Data.CAM_CALIBRATED_S3;
  calData_Camera[195] = Learn_Union.Learning_Data.CAM_CALIBRATED_S4;
  calData_Camera[196] = Learn_Union.Learning_Data.CAM_CALIBRATED_TAU_X;
  calData_Camera[197] = Learn_Union.Learning_Data.CAM_CALIBRATED_TAU_Y;
  calData_Camera[198] = Cal_Union_Parameters.Cal_Parameter.DYN_GRID_LENS_DISTORTION_CFG;
  calData_Camera[199] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_K1;
  calData_Camera[200] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_K2;
  calData_Camera[201] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_K3;
  calData_Camera[202] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_K4;
  calData_Camera[203] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_K5;
  calData_Camera[204] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_K6;
  calData_Camera[205] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_P1;
  calData_Camera[206] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_P2;
  calData_Camera[207] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_S1;
  calData_Camera[208] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_S2;
  calData_Camera[209] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_S3;
  calData_Camera[210] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_S4;
  calData_Camera[211] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_TAU_X;
  calData_Camera[212] = Learn_Union.Learning_Data.DYN_GRID_CALIBRATED_TAU_Y;
  calData_Camera[213] = Cal_Union_Parameters.Cal_Parameter.SKIP_TO;
  calData_Camera[214] = Cal_Union_Parameters.Cal_Parameter.SIVI_EULER_ALPHA;
  calData_Camera[215] = Cal_Union_Parameters.Cal_Parameter.SIVI_EULER_BETA;
  calData_Camera[216] = Cal_Union_Parameters.Cal_Parameter.SIVI_EULER_GAMA;
  calData_Camera[217] = Cal_Union_Parameters.Cal_Parameter.DISCLAIMER_KEY_CYCLES;
  calData_Camera[218] = Cal_Union_Parameters.Cal_Parameter.WELCOME_SCREEN_TIMEOUT;
  calData_Camera[219] = Cal_Union_Parameters.Cal_Parameter.SHUTDOWN_ANIMATION_ENABLED;
  calData_Camera[220] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_MAX_ATTEMPTS;
  calData_Camera[221] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_ATTEMPT_DELAY;
  calData_Camera[222] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_GRAYSCALE_R;
  calData_Camera[223] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_GRAYSCALE_G;
  calData_Camera[224] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_GRAYSCALE_B;
  calData_Camera[225] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_THRESHOLD_METHOD;
  calData_Camera[226] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_THRESHOLD_RADIUS;
  calData_Camera[227] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_THRESHOLD_VALUE;
  calData_Camera[228] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_THRESHOLD_FILTER;
  calData_Camera[229] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_THRESHOLD_FILTER_SIZE;
  calData_Camera[230] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_ROW_1_ACTIVE;
  calData_Camera[231] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_ROW_2_ACTIVE;
  calData_Camera[232] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_ROW_3_ACTIVE;
  calData_Camera[233] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_SUBPIXEL_ENABLED;
  calData_Camera[234] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_SUBPIXEL_WINDOW;
  calData_Camera[235] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_SUBPIXEL_DEADZONE;
  calData_Camera[236] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_SUBPIXEL_MAX_ITER;
  calData_Camera[237] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_SUBPIXEL_EPSILON;
  calData_Camera[238] = Cal_Union_Parameters.Cal_Parameter.RVC_CALIBRATION_CALIBRATE_LENS;

  notifyCalDataUpdated_Camera(calData_Camera);
  
}
else{
        setIVI_CalibrationTriggerStatus(Calib_trigger);
        LOG(INFO) << "CAPIVehicleClientGateway::buf_count_val is not zero" ;
}
}


 void getCalibrationDatasubscribe(){

		 if (m_vehicle_calibChannelProxy != nullptr) {
			 LOG(INFO) << "CAPIVehicleClientGateway inside Calibration attribute call ";
			 m_vehicle_calibChannelProxy->getGetRawEvent().subscribe(std::bind(&VehicleClientImpl::oncalibGetRawEventReceived, this, std::placeholders::_1));
		 }
	   		else {
            LOG(INFO) << "CAPIVehicleClientGateway : Calibration *** Proxy not available";
        }
 }
  void onAuthenticationZoneAttributeChanged(std::vector< uint8_t >AuthenticationZoneval){
	notifyAuthenticationZone(AuthenticationZoneval);
}
	void onApplicationSoftwareIdentificationAttributeChanged(std::vector< uint8_t > SwIDval){
 /*    LOG_INF("CAPI:The onApplicationSoftwareIdentificationAttributeChanged has changed, size: %zu\n", SwIDval.size());
       for (size_t i = 0; i < SwIDval.size(); ++i) {
        LOG_INF("Element %zu: %d\n", i, SwIDval[i]);
    } */
     notifyApplicationSoftwareIdentification(SwIDval);	
}


void onApplicationDataIdentificationAttributeChanged(std::vector< uint8_t > AppDataIDval){

       notifyApplicationDataIdentification(AppDataIDval);
}
void onDataLibraryidentifierAttributeChanged(std::vector< uint8_t > DataLibIDval){

       notifyDataLibraryidentifier(DataLibIDval);
}

void onVehicleAppsIdentifierAttributeChanged(std::vector< uint8_t > VehiAppIDval){

      notifyVehicleAppsIdentifier(VehiAppIDval);
}
 void getApplicationSoftwareIdentification(){
	 
	 LOG(INFO) << "CAPIVehicleClientGateway : Application_Software_Identification";
  
     m_vehicle_proxy->getE_Authentication_ZoneAttribute().getChangedEvent().subscribe(std::bind(&VehicleClientImpl::onAuthenticationZoneAttributeChanged,this,std::placeholders::_1));
  
     m_vehicle_proxy->getE_Application_Software_IdentificationAttribute().getChangedEvent().subscribe(std::bind(&VehicleClientImpl::onApplicationSoftwareIdentificationAttributeChanged,this,std::placeholders::_1));
   
     m_vehicle_proxy->getE_Application_Data_IdentificationAttribute().getChangedEvent().subscribe(std::bind(&VehicleClientImpl::onApplicationDataIdentificationAttributeChanged,this,std::placeholders::_1));
   
    m_vehicle_proxy->getE_Data_Library_identifierAttribute().getChangedEvent().subscribe(std::bind(&VehicleClientImpl::onDataLibraryidentifierAttributeChanged,this,std::placeholders::_1));
   
     m_vehicle_proxy->getE_Vehicle_Apps_IdentifierAttribute().getChangedEvent().subscribe(std::bind(&VehicleClientImpl::onVehicleAppsIdentifierAttributeChanged,this,std::placeholders::_1));
    
	
 }	
void onDiagGetRawEventReceived(CommonAPI::ByteBuffer buffer) {
	LOG(INFO) << "CAPIVehicleClientGateway : getVinNumber onDiagGetRawEventReceived";
		uint32_t size = 1024;
		uint8_t l_rx_buf[size];	
        std::memcpy(l_rx_buf, buffer.data(), buffer.size());   
        uint32_t dataNo = static_cast<uint32_t>((l_rx_buf[0]<<8)|(l_rx_buf[1]));
	    uint32_t payload = 0;
		if(dataNo == vinNumber_ID)
		{
 		for(unsigned i =0;i < vinNumber_size;i++)
		{ 
		 m_VinNumber.push_back(buffer[i]);	
		}
		notifyVinNumberUpdated(m_VinNumber);
	    }
           m_VinNumber.clear();
        if(dataNo == DTC_ID){
           for(unsigned i =4;i < buffer.size();i++){ 
		      m_DTCstructure.push_back(buffer[i]);	
		 }
        notifydtcParamUpdated(m_DTCstructure);
         }
        m_DTCstructure.clear();

}


void getVinNumbersubscribe(){

		 if (m_diagChannelProxy != nullptr) {
			 LOG(INFO) << "CAPIVehicleClientGateway inside VinNumber attribute call ";
       m_diagChannelProxy->getGetRawEvent().subscribe(std::bind(&VehicleClientImpl::onDiagGetRawEventReceived,this,
        std::placeholders::_1));
		 }
	   		else {
				
            LOG(INFO) << "CAPIVehicleClientGateway : VinNumber *** Proxy not available";
        }
 }
  void getVinNumber(std::vector<uint32_t>& p1 ,bool& updatestatus)
	{ 
	    for(unsigned i = 0; i < m_VinNumber.size(); i++){
			p1.push_back(m_VinNumber[i]);			
		}
		updatestatus = update_ret_status;
}
 	
	private:
    std::shared_ptr<v0::iMicro::SOC_LIFECYCLEProxy<>> m_vehicle_proxy; 
    std::shared_ptr<v0::iMicro::SOC_Channel_CANProxy<>> m_vehicle_proxy_CANChanel;
	std::shared_ptr<v0::iMicro::SOC_Channel_CALIBProxy<>> m_vehicle_calibChannelProxy;
	std::shared_ptr<v0::iMicro::SOC_Channel_DIAGProxy<>> m_diagChannelProxy;
	std::shared_ptr<v0::iMicro::SOC_IMUProxy<>> m_vehicle_IMUProxy;
	std::shared_ptr<v0::iMicro::SOC_DIAGProxy<>> m_DIAGProxy;
	std::shared_ptr<v0::iMicro::SOC_DEMProxy<>> m_DEMProxy;
    std::shared_ptr<v0::iMicro::SOC_FIRSTProxy<>> m_FIRSTProxy;
    std::shared_ptr<v1::VHAL::LOGR::LOGR_VHALInterfaceProxy<>> m_TCU_proxy;
    
	
	::iMicro::pmStructs::ivi_parameters_type ivi_param;
	
	std::function<void(uint32_t)> speedCallback_;
	std::function<void(uint32_t)> IgnitionStsCallback_;
	std::function<void(std::vector<uint32_t>)> vinNumberCallback;
	std::function<void(uint32_t[])> bcmHMItimeCallback;

	std::function<void(uint32_t[])>  internalSignalCallback;
	std::function<void(uint32_t[])> bcmSECURtimeCallback;
	std::function<void(std::vector<int32_t>)> imuParamCallback;
	std::function<void(std::vector<uint16_t>)> dIDrIDParamCallback;
	std::function<void(std::vector<uint16_t>)> DEM_DTCStatusCallback;
	std::function<void(uint8_t)> driverdoorStatusCallback;
	std::function<void(uint8_t)> passengerDoorStatusCallback;
	std::function<void(std::vector<uint32_t>)> dotteParamCallback;
	std::function<void(std::vector<uint32_t>)> dotteArrayParamCallback;
	std::function<void(std::vector<uint32_t>)> dtcParamCallback;
	std::function<void(std::vector<uint32_t>)> dLTParamCallback;
	std::function<void(std::vector<uint8_t>)> dLT_VectorParamCallback;
	std::function<void(uint32_t[])>  calibrationDataCallback;
	std::function<void(uint32_t[])>  calibrationDataCallback_Audio;
	std::function<void(uint32_t[])>  calibrationDataCallback_Camera;
	std::function<void(std::vector<uint8_t>)> calibrationDataCallback_Array;
    std::function<void(std::vector<uint32_t>)> calibrationDataforSrecCallback;
    std::function<void(uint32_t)> EcallCrashCallback_;
	std::function<void(uint32_t)> ParkLampCallback;
	std::function<void(uint32_t)> DayNightModeCallback;
	std::function<void(uint32_t)> FuellevelCallback;
	std::function<void(uint32_t)> FuelTypeCallback;
	std::function<void(uint32_t)> ParkBreakStatusCallback;
	std::function<void(uint32_t)> OutTempCallback;
	std::function<void(uint32_t)> HVBatterySOHDataCallback;
	std::function<void(uint32_t)> HVBatteryPercentageCallback;
	std::function<void(uint32_t)> HVBatteryMinChargeCallback;
	std::function<void(uint32_t)> HVBatteryActiveConnectorCallback;
    std::function<void(uint32_t)> HVBatteryMaxRangeCallback;
	std::function<void(uint32_t)> HVBatteryChargeLevelCallback;
	std::function<void(uint32_t)> HVBatteryIsChargingCallback;
	std::function<void(uint32_t)> AutonomyCallback;
    std::function<void(uint32_t)> BrakePedalrStsCallback;
	std::function<void(uint32_t)> CanDistanceUnitCallback;
	std::function<void(uint32_t)> InstantFuelConsCallback;
    std::function<void(uint32_t)> ReverseGearEngagedCallback;
	std::function<void(uint32_t)> RoadSlopeCallback;
	std::function<void(uint32_t)> SteeringAngleCallback;
	std::function<void(uint32_t)> TotalKMCallback;
	std::function<void(uint32_t)> TransmGearDisplayCallback;
	std::function<void(uint32_t)> TripAvgBattPowerConsCallback;
	std::function<void(uint32_t)> YawSpeedCallback;
	std::function<void(uint32_t)> GrossYawRateCallback;
	std::function<void(uint32_t)> FLWheelTickCounterFaultCallback;
	std::function<void(uint32_t)> FRWheelTickCounterFaultCallback;
	std::function<void(uint32_t)> RLWheelTickCounterFaultCallback;
	std::function<void(uint32_t)> RRWheelTickCounterFaultCallback;
	std::function<void(uint32_t)> PulseCountFLWheelCallback;
	std::function<void(uint32_t)> PulseCountFRWheelCallback;
	std::function<void(uint32_t)> PulseCountRLWheelCallback;
	std::function<void(uint32_t)> PulseCountRRWheelCallback;
	std::function<void(uint32_t)> FPASStsCallback;
	std::function<void(uint32_t)> DrivingDirectionCallback;
	std::function<void(uint32_t)> CapacityUnitCallback;
	std::function<void(uint32_t)> CombustFuelLvCallback;
	std::function<void(uint32_t)> CompressorStsCallback;
	std::function<void(int32_t)> ElectricMotorPowerCallback;
	std::function<void(uint32_t)> EngineCoolantTempCallback;
	std::function<void(uint32_t)> EngineRPMCallback;
	std::function<void(uint32_t)> FuelLevelMinimumStsCallback;
	std::function<void(uint32_t)> AbsSteeringWheelAngleAcuCallback;
	std::function<void(uint32_t)> InitSteeringWheenAngleFlagCallback;
	std::function<void(uint32_t)> NetworkMNGTCallback;
	std::function<void(uint32_t)> FPASCenterLeftBarStsCallback;
	std::function<void(uint32_t)> FPASCenterRightBarStsCallback;
    std::function<void(uint32_t)> FPASLeftBarStsCallback;
	std::function<void(uint32_t)> FPASRightBarStsCallback;
	std::function<void(uint32_t)> RPASCenterLeftBarStsCallback;
	std::function<void(uint32_t)> RPASCenterRightBarStsCallback;
	std::function<void(uint32_t)> RPASLeftBarStsCallback;
	std::function<void(uint32_t)> TrailerPresentCallback;
	std::function<void(uint32_t)> TrunkDoorStsCallback;
	std::function<void(uint32_t)> RPASStsCallback;
	std::function<void(uint32_t)> EconShftRqCallback;
	std::function<void(uint32_t)> EngineTorqueCallback;
	std::function<void(uint32_t)> FuelConsUnitCallback;
	std::function<void(uint32_t)> RegeneratedEnergyCallback;
	std::function<void(uint32_t)> ShiftLeverPositionCallback;
	std::function<void(uint32_t)> TripAverageFuelConsumptionCallback;
	std::function<void(uint32_t)> ReqVolCtrlCallback;
    std::function<void(uint32_t)> VehPrivacyModeCallback;
	std::function<void(int32_t[])>  EVSignalsCallback;
	std::function<void(uint32_t[])>  BCMSignalsCallback;
	std::function<void(uint32_t[])>  CameraSignalsCallback;
	std::function<void(uint32_t)>  TemperatureCallback;
    std::function<void(uint32_t)>  SupplyVoltageCallback;
	std::function<void(int32_t[])>  CanCommonSignalsCallback;
	std::function<void(std::vector<uint16_t>)> USB_UpdateCallback;
	std::function<void(std::vector<uint32_t>)> USB_Update_DetailsCallback;
    std::function<void(uint32_t)> CallActiveSTRDisplayPopupStatusCallback;
    std::function<void(uint32_t)> STRCancelShutdownStatusCallback;
	std::function<void(uint32_t)>  PowermodeCallback;
    std::function<void(uint32_t[])>  CAN_RX_SignalCallbacK;
    std::function<void(int32_t[])>  Signed_CAN_RX_SignalCallbacK;
    std::function<void(uint32_t)> Log_retriver_ackCallback;
    std::function<void(std::string)> LoggerfilepathCallback;
    std::function<void(uint32_t)> Log_retriver_statusCallback;
    std::function<void(uint32_t)> WakeupReasonCallback;
    std::function<void(float[])> RXSignals_FloatCallback;
    std::function<void(std::vector<uint8_t>)> MarelliPNCallback;
    std::function<void(std::vector<uint8_t>)> EOLHWIDCallback;
	std::function<void(std::vector<uint8_t>)> AuthenticationZoneCallback;
	std::function<void(std::vector<uint8_t>)> ApplicationSoftwareIdentificationCallback;
	std::function<void(std::vector<uint8_t>)> ApplicationDataIdentificationCallback;
	std::function<void(std::vector<uint8_t>)> DataLibraryidentifierCallback;
	std::function<void(std::vector<uint8_t>)> VehicleAppsIdentifierCallback;
    std::function<void(std::vector<uint8_t>)> DCSD_ReqCallback;
    std::vector<uint8_t> McpuVcpuSwVersion;
	std::vector<uint8_t> EcuSerialNumber;
	std::vector<uint8_t> EcuHwVersionNumber;
	std::vector<uint8_t> EcuHwPartNumber;
	std::vector<uint8_t> EcuHwNumbers;
	std::vector<uint8_t> EcuSwVersionNumber;
	std::vector<uint8_t> EcuSwPartNumber;
	std::vector<uint8_t> EcuSwNumbers;
	std::vector<uint8_t> EcuSparePartNumber;
	std::vector<uint8_t> Read_Cyber_UIN;
	std::vector<uint8_t> ECUManufacturingDateData;
	std::vector<uint8_t> SupplierManufacturerECUSoftwareVersionNum;
	std::vector<uint8_t> InternalMarellibuildVersion;
	std::vector<uint8_t> Manufacturer_Name;
	std::vector<uint8_t> BootLoaderVersionNumber;
	std::vector<uint8_t> Generic_ID1;
	std::vector<uint8_t> Generic_ID2;
	std::vector<uint8_t> Generic_ID3;
		
	std::mutex mutex_;
	uint32_t m_BCMsecurtime[SECUREtime_index]{0,0,0,0,0,0};
	uint32_t m_BCMHMItime[HMItime_index]{0,0,0,0,0};
	uint32_t internalSignal_value[internalSignal::InternalSig_Lenth]{0};
    int32_t EVSignal_value[ev_signals::EVSignal_Length]{0};
	uint32_t BCMSignals_value[bcm_signals::BCMSignal_Length]{0};
	uint32_t CameraSignals_value[camera_signals::CameraSignal_Length]{0};
	int32_t CanCommonSignal_value[CanCommonSignal::CanCommonSignal_Length]{0};	
    uint32_t CAN_RX_Signals[CAN_RX_Signal_Index]{0};
    int32_t Signed_CAN_RX_Signals[Signed_CAN_RX_signals_Index]{0};
	std::vector<uint32_t>  m_VinNumber;
    std::vector<uint32_t> m_DTCstructure;
	uint32_t calData_display_All[213]{0};
	uint32_t calData_Audio[2384]{0};
	uint32_t calData_Camera[239]{0};
    uint16_t Calib_trigger = 1U;
    float floatrxvalues[156]{0};
    std::vector<std::string> parameterNames = {
    "VEHICLE_BRAND", "DISPLAY_TYPE", "COUNTRY_CODE", "STARTUP_ANIMATION_MODE", "CONTEXTUAL_STARTUP_ANIMATION", "WELCOME_ONBOARD_SOUND", "VEH_LINE",
    "VEHICLE_PROPULSION_STORAGE_TYPE", "DAY_NIGHT_THEME_ENABLED", "EV_CONNECTOR_TYPE", "AMP_PRESENT", "THEMED_SOUNDS", "SIGNAL_SOURCE_ENABLE",
    "PAM_CHIME_ENABLE", "PAM_CONFIGURATION", "PARK_ASSIST_VOLUME_STRATEGY", "PAM_CHIME_MODE", "AUDIO_SYSTEM_TYPE", "SIGNAL_TYPE", "RPAS_PRESENT",
    "FPAS_PRESENT", "RVC_TYPE", "VISIO_PARK_ENABLED", "DRIVER_SIDE", "STARTUP_AUDIO_VOL_CFG", "REVERSE_MUTE_ENABLED", "AUDIO_EQUALIZER_SETTING",
    "SPEAKER_CONFIGURATION", "GEAR_BOX_TYPE", "HVAC_TYPE", "DEFAULT_VEH_THEME", "DAB_PRESENT", "AM_TUNER_ENABLED", "HW_IGN_PRESENT", "HW_ACC_PRESENT",
    "ACC_PRESENT", "CONN_PRESENT", "NAV_PRESENT", "Reserve0", "Reserve2", "POWER_BUTTON_TYPE", "Reserve3", "REGULATION_TYPE", "Reserve5",
    "TRAFFIC_SIGN_CONNECTED_SERVICE", "DISCLAIMER_CLIMATE_CONTROLS", "REGION_SPECIFIC_DISCLAIMER", "DISCLAIMER_SCREEN_MODE", "TYPE_EMERGENCY_CALL",
    "Reserve6", "Reserve7", "PREMIUM_AUDIO_BRAND", "Reserve8", "Reserve9", "SELECTABLE_THEME_ENABLED", "DRIVER_DISTRACTION_MODE", "CLIMATE_HMI_ENABLED",
    "Reserve10", "MHU_TYPE", "Reserve11", "ICS_PRESENT", "LOUDSPEAKER_PRESENT", "SURROUND", "SDW_CONFIGURATION", "SDW_CHIME_MODE", "Reserve12",
    "BSS_PRESENT", "BSS_CONFIGURATION", "SRRP_PRESENCE", "SRRP2_PRESENCE", "HALF_PRESENT", "Reserve13", "Reserve14", "ACALL_LED_PRESENCE", "Reserve15",
    "VEHICLE_TYPE", "Reserve16", "ECALL_PUSH_PRESENCE", "ACALL_PUSH_PRESENCE", "ECALL_LED_PRESENCE", "BUB_PRESENCE", "HEATED_STEERING_WHEEL_PRESENCE",
    "HEATED_STEATS_DRIVER_PRESENCE", "HEATED_STEATS_PASSENGER_PRESENCE", "Reserve20", "FRONT_DEFROSTER", "REAR_DEFROSTER", "Reserve21", "SWS_TYPE",
    "Reserve22", "PROFILE_DETECTOR_ENABLED", "RVC_INSTALLATION_LOC", "DYNAMIC_GRIDLINES_ENABLED", "Reserve23", "RPAS_CHIMES_ENABLED", "FPAS_CHIMES_ENABLED",
    "MEM_DYN_GRID_SETTING", "PRIVACY_MODE_MENU", "Reserve24", "IDLE_SCREEN_MODE", "Reserve25", "Reserve26", "MEASUREMENT_UNIT_SYNC_ENABLED",
    "SHOW_MEASUREMENT_UNITS", "AIRBAG_INTERFACE_TYPE", "Reserved27", "SUSPEND_RAM_DURATION", "LISTEN_MODE1_DURATION", "LISTEN_MODE2_DURATION",
    "LISTEN_MODE3_DURATION", "LISTEN_MODE_1_CONN_RETRY_TIME", "LISTEN_MODE_3_WAKE_UP_TIME", "POWER_BUTTON_TYPE", "SANITY_REBOOT_PERIOD", "MAX_CALL_DURATION", "MAX_AUTONOMY", "HV_BATT_CAP"
    };
	bool update_ret_status;
	iMicro::iMicroStructs::DotteNVMParameters_MCU_t m_dotteVal;
	iMicro::iMicroStructs::InternalVariables_MCU_t m_internalsignal;
	CommonAPI::ByteBuffer canbuffer;
	CommonAPI::ByteBuffer buffer;

};

CAPIVehicleClientGateway::CAPIVehicleClientGateway() : m_pImpl{std::make_unique<VehicleClientImpl>()} {}

CAPIVehicleClientGateway::~CAPIVehicleClientGateway() {
};


class CAPIVehicleTCUServerGateway::VehicleClientImpl {
    public:
        void initialize_TCU() {

            /**
            *TCU Server Initialization and Registration
            */
            CommonAPI::Runtime::setProperty("LogContext", "T02S");
            CommonAPI::Runtime::setProperty("LogApplication", "T02S");
            CommonAPI::Runtime::setProperty("LibraryBase", "tcuService");
            /**
            *Persist the runtime to ensure registration remains valid
            */
            runtime_tcu = CommonAPI::Runtime::get();
    
            std::string tcu_domain = "local";
            std::string tcu_instance = "VHAL.VHAL_LIFECYCLE";
            std::string tcu_connection = "tcu-lifecycle";
            /**
            *Create and store the stub service
            */
            tcuStubService = std::make_shared<tcuStubImpl>();
    
            while (!runtime_tcu->registerService(tcu_domain, tcu_instance, tcuStubService, tcu_connection)) {
                LOG(INFO) << "Register Service failed, trying again in 100 milliseconds...\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
    
            uint8_t data = 1;
            uint8_t count_tcu = 0U;
            while (count_tcu <= 5) {
                tcuStubService->setEcallStatusAttribute(data);
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                data++;
                count_tcu++;
            }
        }
    
            void getTCUData(uint32_t* p1) {
                for (uint32_t i = 0; i < TCU_InternalSignals_Index; ++i) {
                    p1[i] = TCU_Data[i];
                }
            }

            void getSLIData(uint32_t* p1) {
                for (uint32_t i = 0; i < SLI_data_length; ++i) {
                    p1[i] = SLI_Data[i];
                }
            }

            void getPackagepath(std::string& packagePath) {
                packagePath = packagePath_SW;
            }
            void getFOTAHMICheckforUpdateResultsUpdated(std::string& CheckforUpdateResult ){
                CheckforUpdateResult = FotaHmiCheckforUpdateResult_S;
            }
            void getFOTAHMIUpdateAvailable(std::string& UpdateAvailable){
                UpdateAvailable = FotaHmiUpdateAvailable_S;
            }
            void getFOTAHMIConditionsNotMet(std::string& ConditionsNotMet){
                ConditionsNotMet = FotaHmiConditionsNotMet_S;
            }
            void getFOTAHMIInstallationStatus(std::string& InstallationStatus){
                InstallationStatus = FotaHmiInstallationStatus_S;
            }
            void getFOTAHMIUpdateFinished(std::string& UpdateFinished){
                UpdateFinished = FotaHmiUpdateFinished_S;
            }
            void getFOTAHMIWhatsNewDetails(std::string& WhatsnewDetails){
                WhatsnewDetails = FotaHmiWhatsNewDetails_S;
            }
            void getFOTAHMIUpdateHistory(std::string& UpdateHistory){
                UpdateHistory= FotaHmiUpdateHistory_S;
            }
            void getFOTAHMIPendingUpdates(std::string& PendingUpdates){
                PendingUpdates = FotaHmiPendingUpdates_S;
            }
            void getFOTAHMIScheduleUpdate(std::string& ScheduleUpdate){
                ScheduleUpdate = FotaHmiScheduleUpdate_S;
            }
            void getAOSPRBUATrigger(std::string& AOSPRBUATrigger){
                AOSPRBUATrigger = AOSPRBUATrigger_S;
            }
            void getFotaHmiInfoResult(std::string& FotaHmiInfoResult){
                FotaHmiInfoResult = FOTAHMI_InfoResult_S;
            }
            void getUsbUpdateDetails(std::string& USBUpdatedetails){
                USBUpdatedetails = USBUpdate_Details_S;
            }
            void notifyTCU_InternalSignalUpdated(uint32_t TCU_internalSignal[]) {
                if (TCU_internalSignalCallback) {
                    TCU_internalSignalCallback(TCU_internalSignal);
                }
            }
            void notifySLIdataUpdated(uint32_t SLIData[]) {
                if (SLI_DataCallback) {
                    SLI_DataCallback(SLIData);
                }
            }
     
            void notifyFlashUnitId(uint32_t FlashUnitId){
                if(Flash_UnitIdCallback){
                    Flash_UnitIdCallback(FlashUnitId);
                }
            }
			void notifyfoataDataUpdated(uint16_t FOTAHMI_intData[]) {	    
                if (FotaHmiDataCallback) {
		           FotaHmiDataCallback(FOTAHMI_intData);
		        }
	        }
            void notifyPackagePath(std::string PackagePath){
                if(Package_PathCallback){
                    Package_PathCallback(PackagePath);
                }
            }

            void notifyFOTAHMICheckforResultsUpdated(std::string FOTA_Check_for_Updates_result){
                if(FotaDataCheckforResultsCallback){
                    FotaDataCheckforResultsCallback(FOTA_Check_for_Updates_result);
                }
            }
            void notifyFOTAHMIUpdateAvailableUpdated(std::string FOTA_Update_Available){
                if(FotaDataUpdateAvailableCallback){
                    FotaDataUpdateAvailableCallback(FOTA_Update_Available);
                }
            }
            void notifyFOTAHMIConditionsnotmet(std::string FOTA_Conditions_not_Met){
                if(FotaDataConditionsnotmetCallback){
                    FotaDataConditionsnotmetCallback(FOTA_Conditions_not_Met);
                }
            }
            void notifyFOTAHMIInstallationStatus(std::string FOTA_Installation_Status){
                if(FotaDataInstallationStatusCallback){
                    FotaDataInstallationStatusCallback(FOTA_Installation_Status);
                }
            }
            void notifyFOTAHMIWhatsnewDetailsResponse(std::string FOTAWhatsNew_Details_Response){
                if(FotaDataWhatNewDetailsCallback){
                    FotaDataWhatNewDetailsCallback(FOTAWhatsNew_Details_Response);
                }
            }
            void notifyFOTAHMIUpdateHistoryResponse(std::string FOTAUpdate_History_Response){
                if(FotaDataUpdateHistoryResponseCallback){
                   FotaDataUpdateHistoryResponseCallback(FOTAUpdate_History_Response);
                }
            }
            void notifyFOTAHMIPendingUpdateHistoryResponse(std::string FOTAPending_Updates_Response){
                if(FotaDataPendingUpdateHistoryCallback){
                    FotaDataPendingUpdateHistoryCallback(FOTAPending_Updates_Response);
                }
            }
            void notifyFOTAHMIUpdateFinished(std::string FOTA_Update_Finished){
                if(FotaDataUpdateFinishedCallback){
                    FotaDataUpdateFinishedCallback(FOTA_Update_Finished);
                }
            }
            void notifyFOTAHMIScheduleUpdateResponse(std::string FOTASchedule_Update_Response){
                if(FotaDataUpdateScheduleCallback){
                    FotaDataUpdateScheduleCallback(FOTASchedule_Update_Response);
                }
            }
			void notifyUINTcuToSendHMI(std::string UIN_TCU_toHMI){
                if(UIN_ToSend_HMICallback){
                    UIN_ToSend_HMICallback(UIN_TCU_toHMI);
                }
            }

            void notifyTCUVersiontoHMI(std::string TCU_version_number){
                if(TCUVersionnumbertoHMICallback){
                    TCUVersionnumbertoHMICallback(TCU_version_number);
                }
            }

            void notifyAOSPRBUATrigger(std::string AOSPRBUATrigger) {
                if (AOSPRBUATriggerCallback) {
                    AOSPRBUATriggerCallback(AOSPRBUATrigger);
                }
            }
            void notifyFotaHmiInfoResult(std::string FotaHmiInfoResult) {
                if (FotaHmiInfoResultCallback) {
                    FotaHmiInfoResultCallback(FotaHmiInfoResult);
                }
            }
            void notifyUsbUpdateDetails(std::string USBUpdatedetails){
                if (USBUpdateDetailsCallback){
                    USBUpdateDetailsCallback(USBUpdatedetails);
                }
            }

            void registerTCUSignalCallback(std::function<void(uint32_t[])> TCU_internalSignal) {
                TCU_internalSignalCallback = TCU_internalSignal;
            }

            void registerSLIDataCallback(std::function<void(uint32_t[])> SLIData) {
                SLI_DataCallback = SLIData;
            }
            
            void registerFlashUnitIdCallback(std::function<void(uint32_t)> Flash_UnitId){
                Flash_UnitIdCallback = Flash_UnitId;
            }
            void registerFotaHMICheckforResultsCallback(std::function<void(std::string)> m_FOTAHMI_CheckforResults_DATA){
                FotaDataCheckforResultsCallback = m_FOTAHMI_CheckforResults_DATA;
            }
            void registerFotaHMIUpdateAvailableCallback(std::function<void(std::string)> m_FOTAHMI_UpdateAvailable_DATA){
                FotaDataUpdateAvailableCallback = m_FOTAHMI_UpdateAvailable_DATA; 
            }
            void registerFotaHMIConditionsnotmetCallback(std::function<void(std::string)> m_FOTAHMI_ConditionsnotMet_DATA){
                FotaDataConditionsnotmetCallback = m_FOTAHMI_ConditionsnotMet_DATA;
            }
            void registerFotaHMIInstallationStatusCallback(std::function<void(std::string)> m_FOTAHMI_InstallationStatus_DATA){
                FotaDataInstallationStatusCallback = m_FOTAHMI_InstallationStatus_DATA;
            }
            void registerFotaHMIWhatNewDetailsCallback(std::function<void(std::string)> m_FOTAWhatsNew_Details_Response){
                    FotaDataWhatNewDetailsCallback = m_FOTAWhatsNew_Details_Response;
            }
            void registerFotaHMIUpdateHistoryResponseCallback(std::function<void(std::string)> m_FOTAUpdate_History_Response){
                    FotaDataUpdateHistoryResponseCallback = m_FOTAUpdate_History_Response;
            }
            void registerFotaHMIPendingUpdateHistoryCallback(std::function<void(std::string)> m_FOTAPending_Updates_Response){
                    FotaDataPendingUpdateHistoryCallback = m_FOTAPending_Updates_Response;
            }
            void registerFotaHMIUpdateFinishedCallback(std::function<void(std::string)> m_FOTA_Update_Finished){
                    FotaDataUpdateFinishedCallback = m_FOTA_Update_Finished;
            }
            void registerFotaHMIUpdateScheduleCallback(std::function<void(std::string)> m_FOTASchedule_Update_Response){
                    FotaDataUpdateScheduleCallback = m_FOTASchedule_Update_Response;
            }
            void registerUIN_TCUToHMICallback(std::function<void(std::string)> UIN_TCU_ToHMI_Data){
                 UIN_ToSend_HMICallback = UIN_TCU_ToHMI_Data;
	        }
            void registerTCUVersiontoHMICallback(std::function<void(std::string)> TCU_version_to_HMI){
                TCUVersionnumbertoHMICallback = TCU_version_to_HMI;
            }
            void registerPackagePathCallback(std::function<void(std::string)> package_filepath){
                Package_PathCallback = package_filepath;
            }
            void registerAOSPRBUATriggerCallback(std::function<void(string)> m_AOSPRBUATrigger) {
                AOSPRBUATriggerCallback = m_AOSPRBUATrigger;
            }
            void registerFotaHmiInfoResultCallback(std::function<void(std::string)> m_FOTAHMI_Info_Result){
                FotaHmiInfoResultCallback = m_FOTAHMI_Info_Result;
            }
            void registerUSB_Update_detailsCallback(std::function<void(std::string)> m_USBUpdate_details){
                USBUpdateDetailsCallback = m_USBUpdate_details;
            }
            void SetUserXcall_TCU(uint32_t UserXcall_Data) {
                LOG(INFO) << "Address of tcuStubService SetUserXcall_TCU: " << tcuStubService.get() << "\n";
                if (tcuStubService) {
                    tcuStubService->fireSend_UserXCallInput_TCUEvent(UserXcall_Data);
                    ALOGD("Called fireSend_UserXCallInput_TCUEvent successfully");
                } else {
                    ALOGD("Failed to call fireSend_UserXCallInput_TCUEvent: tcuStubService is null");
                }
            }
    
            void SetLanguage_Selection_TCU(uint32_t Language_Selection) {
                LOG(INFO) << "Address of tcuStubService SetLanguage_Selection: " << tcuStubService.get() << "\n";
                if (tcuStubService) {
                    tcuStubService->fireSend_Language_selectionEvent(Language_Selection);
                    sleep(1);
                    tcuStubService->SetLanguageSelection(Language_Selection);
                    ALOGD("Called fireSend_Language_selectionEvent successfully");
                } else {
                    ALOGD("Failed to call fireSend_Language_selectionEvent: tcuStubService is null");
                }
            }
            void SendEvent_HMI_Trigger_For_UIN(uint32_t HMI_Trigger_for_UIN) {
                if (tcuStubService) {
                    tcuStubService->setUIN_Trigger_from_HMIAttribute(HMI_Trigger_for_UIN);
                    ALOGD("Called setUIN_Trigger_from_HMIAttribute successfully");
                } else {
                    ALOGD("Failed to call setUIN_Trigger_from_HMIAttribute: tcuStubService is null");
                }
            }
            void Send_Privacy_mode_sts_TCU(uint32_t privacy_mode) {
                if (tcuStubService) {
                    tcuStubService->setPrivacy_ModeAttribute(privacy_mode);
                    ALOGD("Called setPrivacy_ModeAttribute successfully");
                } else {
                    ALOGD("Failed to call setPrivacy_ModeAttribute: tcuStubService is null");
                }
            }
            void HMI_FOTA_Update_history(std::string Update_History) {
                if (tcuStubService) {
                    tcuStubService->setHMI_FOTA_Update_historyAttribute(Update_History);
                    ALOGD("Called setHMI_FOTA_Update_historyAttribute successfully");
                } else {
                    ALOGD("Failed to call setHMI_FOTA_Update_historyAttribute: tcuStubService is null");
                }  
            }
            void HMI_FOTA_pending_updates(std::string Pending_updates) {
                if (tcuStubService) {
                    tcuStubService->setHMI_FOTA_pending_updatesAttribute(Pending_updates);
                    ALOGD("Called setHMI_FOTA_pending_updatesAttribute successfully");
                } else {
                    ALOGD("Failed to call setHMI_FOTA_pending_updatesAttribute: tcuStubService is null");
                }    
            }
            void FOTA_WhatsNew_Details(std::string Whatsnewdetails){
                if (tcuStubService) {
                    tcuStubService->setFOTA_WhatsNew_DetailsAttribute(Whatsnewdetails);
                    ALOGD("Called setFOTA_WhatsNew_DetailsAttribute successfully");
                } else {
                    ALOGD("Failed to call setFOTA_WhatsNew_DetailsAttribute: tcuStubService is null");
                }
            }
            void FOTA_Schedule_Update(std::string ScheduleUpdate){
                if (tcuStubService) {
                    tcuStubService->setFOTA_Schedule_UpdateAttribute(ScheduleUpdate);
                    ALOGD("Called setFOTA_Schedule_UpdateAttribute successfully");
                } else {
                    ALOGD("Failed to call setFOTA_Schedule_UpdateAttribute: tcuStubService is null");
                }
            }
            void FOTA_Update_Dismissed(std::string UpdateDismissed){
                if (tcuStubService) {
                    tcuStubService->setFOTA_Update_DismissedAttribute(UpdateDismissed);
                    ALOGD("Called setFOTA_Update_DismissedAttribute successfully");
                } else {
                    ALOGD("Failed to call setFOTA_Update_DismissedAttribute: tcuStubService is null");
                }
            }
            void FOTA_Installation_Finished_with_User_Conformation(std::string Installation_finished){
                if (tcuStubService) {
                    tcuStubService->setFOTA_Installation_Finished_with_User_ConformationAttribute(Installation_finished);
                    ALOGD("Called setFOTA_Installation_Finished_with_User_ConformationAttribute successfully");
                } else {
                    ALOGD("Failed to call setFOTA_Installation_Finished_with_User_ConformationAttribute: tcuStubService is null");
                }
            }
            void FOTA_Start_Installation(std::string start_installation){
                if (tcuStubService) {
                    tcuStubService->setFOTA_Start_InstallationAttribute(start_installation);
                    ALOGD("Called setFOTA_Start_InstallationAttribute successfully");
                } else {
                    ALOGD("Failed to call setFOTA_Start_InstallationAttribute: tcuStubService is null");
                }
            }
            void FOTA_Check_for_Updates(std::string checkfor_updates){
                if (tcuStubService) {
                    tcuStubService->setFOTA_Check_for_UpdatesAttribute(checkfor_updates);
                    ALOGD("Called setFOTA_Check_for_UpdatesAttribute successfully");
                } else {
                    ALOGD("Failed to call setFOTA_Check_for_UpdatesAttribute: tcuStubService is null");
                }
            }
            void AOSP_Flash_Result(std::string AOSP_Flash_Result){
                if (tcuStubService) {
                    tcuStubService->setAOSP_Flash_ResultAttribute(AOSP_Flash_Result);
                    ALOGD("Called setAOSP_Flash_ResultAttribute successfully");
                } else {
                    ALOGD("Failed to call setAOSP_Flash_ResultAttribute: tcuStubService is null");
                }
            }
            void HMI_Status_SWUpdate(uint32_t HMI_Status_SWUpdate) {
                if (tcuStubService) {
                    tcuStubService->setHMI_Status_SWUpdateAttribute(HMI_Status_SWUpdate);
                    ALOGD("Called setHMI_Status_SWUpdateAttribute successfully");
                } else {
                    ALOGD("Failed to call setHMI_Status_SWUpdateAttribute: tcuStubService is null");
                }
            }
            void HMIFOTA_Info(std::string hmifota_info) {
                if (tcuStubService) {
                    tcuStubService->setHMIFOTA_InfoAttribute(hmifota_info);
                    ALOGD("Called setHMIFOTA_InfoAttribute successfully");
                } else {
                    ALOGD("Failed to call setHMIFOTA_InfoAttribute: tcuStubService is null");
                }    
            }
            void FOTAHMI_CheckforUpdates_Result(std::string FOTA_Check_for_Updates_result){
                notifyFOTAHMICheckforResultsUpdated(FOTA_Check_for_Updates_result);
            }
            void FOTAHMI_Update_Available(std::string FOTA_Update_Available){
                notifyFOTAHMIUpdateAvailableUpdated(FOTA_Update_Available);
            }
            void FOTAHMI_Conditions_notMet(std::string FOTA_Conditions_not_Met){
                notifyFOTAHMIConditionsnotmet(FOTA_Conditions_not_Met);
            }
            void FOTAHMI_InstallationStatus(std::string FOTA_Installation_Status){
                notifyFOTAHMIInstallationStatus(FOTA_Installation_Status);
            }
            void FOTAHMI_WhatsNew_Details_Response(std::string FOTAWhatsNew_Details_Response){
                notifyFOTAHMIWhatsnewDetailsResponse(FOTAWhatsNew_Details_Response);
            }
            void FOTAHMI_Update_History_Response(std::string FOTAUpdate_History_Response){
                notifyFOTAHMIUpdateHistoryResponse(FOTAUpdate_History_Response);
            }
            void FOTAHMI_Pending_Update_History_Response(std::string FOTAPending_Updates_Response){
                notifyFOTAHMIPendingUpdateHistoryResponse(FOTAPending_Updates_Response);
            }
            void FOTAHMI_Update_Finished(std::string FOTA_Update_Finished){
                notifyFOTAHMIUpdateFinished(FOTA_Update_Finished);
            }
            void FOTAHMI_Schedule_Update_Responsee(std::string FOTASchedule_Update_Response){
                notifyFOTAHMIScheduleUpdateResponse(FOTASchedule_Update_Response);
            }
			void UIN_TCU_ToSend_HMI(std::string UIN_TCU_toHMI) {
                  notifyUINTcuToSendHMI(UIN_TCU_toHMI);
            }
            void AOSP_RBUA_Trigger_Response(std::string AOSPRBUATrigger) {
                notifyAOSPRBUATrigger(AOSPRBUATrigger);
            }
            void FotaHmi_InfoResult(std::string FOTAHMI_Info_Result){
                notifyFotaHmiInfoResult(FOTAHMI_Info_Result);
            }
            void USB_Update_Details(std::string USBUpdate_details){
                notifyUsbUpdateDetails(USBUpdate_details);
            }
			void TCU_version_Number_to_HMI(std::string TCU_version_number){
                const std::string TCU_VersionfilePath = "/data/misc/update_engine/prefs/TCU-Version.txt";
                // const std::string TCU_version_number = "R2EX_TCU_L1_25244A_D";  // Replace with the actual version string
                std::ofstream outFile(TCU_VersionfilePath);
                // Check if the file stream is successfully opened
                if (outFile.is_open()) {
                // Write the version string to the file
                outFile << TCU_version_number ;
                notifyTCUVersiontoHMI(TCU_version_number) ;

                }
                else{
                ALOGD("File Not open /data/misc/update_engine/prefs/TCU-Version.txt.");
                }
               outFile.close();
            
                ALOGD("TCU-Version successfully written to the file.");
            }
			

            void SetXcall_HMI_Control_TCU(uint32_t XCallHmiControl) {
                if (tcuStubService) {
                    tcuStubService->setXCallHmiControlAttribute(XCallHmiControl);
                    ALOGD("Called setXCallHmiControlAttribute successfully");
                } else {
                    ALOGD("Failed to call setXCallHmiControlAttribute: tcuStubService is null");
                }
            }

            void SetFlashResult_TCU(uint32_t FlashResult)
            {
                if (tcuStubService) {
                    tcuStubService->setFlash_ResultAttribute(FlashResult);
                    ALOGD("Called setFlash_ResultAttribute successfully");
                } else {
                    ALOGD("Failed to call setFlash_ResultAttribute: tcuStubService is null");
                }
            }
            
            void SetFlash_UnitId_TCU(uint32_t FlashUnitId) {
                notifyFlashUnitId(FlashUnitId);
            }

            void SetPackage_Path_TCU(std::string PackagePath) {
                notifyPackagePath(PackagePath);
            }

            void SetActive_phone_call_status_TCU(uint32_t active_phone_call_status)
            {
                if (tcuStubService) {
                    tcuStubService->setActive_phone_callAttribute(active_phone_call_status);
                    ALOGD("Called setActive_phone_callAttribute successfully");
                } else {
                    ALOGD("Failed to call setActive_phone_callAttribute: tcuStubService is null");
                }
            }
            void SetBTCallHMI_Status_TCU(uint32_t BtCallHmi_status)
            {
                if (tcuStubService) {
                    tcuStubService->setBTCallHMI_StatusAttribute(BtCallHmi_status);
                    ALOGD("Called setBTCallHMI_StatusAttribute successfully");
                } else {
                    ALOGD("Failed to call setBTCallHMI_StatusAttribute: tcuStubService is null");
                }
            }

            void SetBTDeviceAttached_Status_TCU(uint32_t BtDeviceAttached_status)
            {
                if (tcuStubService) {
                    tcuStubService->setBTDeviceAttached_StatusAttribute(BtDeviceAttached_status);
                    ALOGD("Called setBTDeviceAttached_StatusAttribute successfully");
                } else {
                    ALOGD("Failed to call setBTDeviceAttached_StatusAttribute: tcuStubService is null");
                }
            }

            void SetBTTransferTime_Duration_TCU(uint32_t BtTransfertime_duration)
            {
                if (tcuStubService) {
                    tcuStubService->setBTTransferTime_DurationAttribute(BtTransfertime_duration);
                    ALOGD("Called setBTTransferTime_DurationAttribute successfully");
                } else {
                    ALOGD("Failed to call setBTTransferTime_DurationAttribute: tcuStubService is null");
                }
            }
            void SetIncomingPhoneCall_Status_TCU(uint32_t IncomingPhoneCall_status){
                if (tcuStubService) {
                    tcuStubService->setIncomingPhoneCallAttribute(IncomingPhoneCall_status);
                    ALOGD("Called setIncomingPhoneCallAttribute successfully");
                } else {
                    ALOGD("Failed to call setIncomingPhoneCallAttribute: tcuStubService is null");
                }
            }
            void SetOutgoingPhoneCall_Status_TCU(uint32_t OutgoingPhoneCall_status){
                if (tcuStubService) {
                    tcuStubService->setOutgoingPhoneCallAttribute(OutgoingPhoneCall_status);
                    ALOGD("Called setOutgoingPhoneCallAttribute successfully");
                } else {
                    ALOGD("Failed to call setOutgoingPhoneCallAttribute: tcuStubService is null");
                }
            }
            void SetBTTransferAcceptance_Status_TCU(uint32_t BTTransferAcceptance_status){
                if (tcuStubService) {
                    tcuStubService->setBtTransferAcceptanceAttribute(BTTransferAcceptance_status);
                    ALOGD("Called setBtTransferAcceptanceAttribute successfully");
                } else {
                    ALOGD("Failed to call setBtTransferAcceptanceAttribute: tcuStubService is null");
                }
            }


        private:
            std::shared_ptr<CommonAPI::Runtime> runtime_tcu;
            std::shared_ptr<tcuStubImpl> tcuStubService;
            std::function<void(uint32_t[])> TCU_internalSignalCallback;
            std::function<void(uint32_t[])> SLI_DataCallback;
            std::function<void(uint32_t)> Flash_UnitIdCallback;
            std::function<void(std::string)> Package_PathCallback;
			std::function<void(uint16_t[])> FotaHmiDataCallback;
            std::function<void(std::string)> FotaDataCheckforResultsCallback;
            std::function<void(std::string)> FotaDataUpdateAvailableCallback;
            std::function<void(std::string)> FotaDataConditionsnotmetCallback;
            std::function<void(std::string)> FotaDataInstallationStatusCallback;
            std::function<void(std::string)> FotaDataWhatNewDetailsCallback;
            std::function<void(std::string)> FotaDataUpdateHistoryResponseCallback;
            std::function<void(std::string)> FotaDataPendingUpdateHistoryCallback;
            std::function<void(std::string)> FotaDataUpdateFinishedCallback;
            std::function<void(std::string)> FotaDataUpdateScheduleCallback;
            std::function<void(std::string)> TCUVersionnumbertoHMICallback;
            std::function<void(std::string)> UIN_ToSend_HMICallback;
            std::function<void(std::string)> AOSPRBUATriggerCallback;
            std::function<void(std::string)> FotaHmiInfoResultCallback;
            std::function<void(std::string)> USBUpdateDetailsCallback;

        };
    
  CAPIVehicleTCUServerGateway::CAPIVehicleTCUServerGateway(): m_pImpl_TCU{std::make_unique<VehicleClientImpl>()} {}

  CAPIVehicleTCUServerGateway::~CAPIVehicleTCUServerGateway() {
  };


  void CAPIVehicleTCUServerGateway::getTCUData(uint32_t* p1) {
    m_pImpl_TCU->getTCUData(p1);
}
void CAPIVehicleTCUServerGateway::getSLIData(uint32_t* p1) {
    m_pImpl_TCU->getSLIData(p1);
}
void CAPIVehicleTCUServerGateway::getPackagepath(std::string& packagePath) {
    m_pImpl_TCU->getPackagepath(packagePath);
}
void CAPIVehicleTCUServerGateway::getFOTAHMICheckforUpdateResultsUpdated(std::string& CheckforUpdateResult){
    m_pImpl_TCU->getFOTAHMICheckforUpdateResultsUpdated(CheckforUpdateResult);
}
void CAPIVehicleTCUServerGateway::getFOTAHMIUpdateAvailable(std::string& UpdateAvailable){
    m_pImpl_TCU->getFOTAHMIUpdateAvailable(UpdateAvailable);
}
void CAPIVehicleTCUServerGateway::getFOTAHMIConditionsNotMet(std::string& ConditionsNotMet){
    m_pImpl_TCU->getFOTAHMIConditionsNotMet(ConditionsNotMet);
}
void CAPIVehicleTCUServerGateway::getFOTAHMIInstallationStatus(std::string& InstallationStatus){
    m_pImpl_TCU->getFOTAHMIInstallationStatus(InstallationStatus);
}
void CAPIVehicleTCUServerGateway::getFOTAHMIUpdateFinished(std::string& UpdateFinished){
    m_pImpl_TCU->getFOTAHMIUpdateFinished(UpdateFinished);
}
void CAPIVehicleTCUServerGateway::getFOTAHMIWhatsNewDetails(std::string& WhatsnewDetails){
    m_pImpl_TCU->getFOTAHMIWhatsNewDetails(WhatsnewDetails);
}
void CAPIVehicleTCUServerGateway::getFOTAHMIUpdateHistory(std::string& UpdateHistory){
    m_pImpl_TCU->getFOTAHMIWhatsNewDetails(UpdateHistory);
}
void CAPIVehicleTCUServerGateway::getFOTAHMIPendingUpdates(std::string& PendingUpdates){
    m_pImpl_TCU->getFOTAHMIPendingUpdates(PendingUpdates);
}
void CAPIVehicleTCUServerGateway::getFOTAHMIScheduleUpdate(std::string& ScheduleUpdate){
    m_pImpl_TCU->getFOTAHMIScheduleUpdate(ScheduleUpdate);
}
void CAPIVehicleTCUServerGateway::getAOSPRBUATrigger(std::string& AOSPRBUATrigger){
    m_pImpl_TCU->getAOSPRBUATrigger(AOSPRBUATrigger);
}
void CAPIVehicleTCUServerGateway::getFotaHmiInfoResult(std::string& FotaHmiInfoResult){
    m_pImpl_TCU->getFotaHmiInfoResult(FotaHmiInfoResult);
}
void CAPIVehicleTCUServerGateway::getUsbUpdateDetails(std::string& USBUpdatedetails){
    m_pImpl_TCU->getUsbUpdateDetails(USBUpdatedetails);
}
 void CAPIVehicleTCUServerGateway::init_TCU()
{
    LOG(INFO) << "CAPIVehicleTCUServerGateway : init_TCU .";
    m_pImpl_TCU->initialize_TCU();
    LOG(INFO) << "CAPIVehicleTCUServerGateway : end init_TCU ";
}

void CAPIVehicleTCUServerGateway::SendEvent_MsdTransmissionTime_TCU_To_IVI(uint32_t msdTransmissionTime_TCU){
    TCU_Data[0] = msdTransmissionTime_TCU;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
}
void CAPIVehicleTCUServerGateway::SendEvent_EcallState_TCU_To_IVI(uint32_t EcallState_TCU){
	TCU_Data[1] = EcallState_TCU;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_AcallState_TCU_To_IVI(uint32_t AcallState_TCU){
	TCU_Data[2] = AcallState_TCU;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);
	
}
void CAPIVehicleTCUServerGateway::SendEvent_UserXcallFeedback_TCU_To_IVI(uint32_t UserXcallFeedback_TCU){
	TCU_Data[3] = UserXcallFeedback_TCU;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_EcallDurationTimer_TCU_To_IVI(uint32_t EcallDurationTimer_TCU){
	TCU_Data[4] = EcallDurationTimer_TCU;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_EcallTpsState_TCU_To_IVI(uint32_t EcallTpsState_TCU){
	TCU_Data[5] = EcallTpsState_TCU;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_XcallStatus_TCU_To_IVI(uint32_t XcallStatus_TCU){
	TCU_Data[6] = XcallStatus_TCU;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);
	
}
void CAPIVehicleTCUServerGateway::SendEvent_EcallWcbTimer_TCU_To_IVI(uint32_t _EcallWcbTimer_TCU){
	TCU_Data[7] = _EcallWcbTimer_TCU;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_DmdMuteSual_TCU_To_IVI(uint32_t _DmdMuteSual){
	TCU_Data[8] = _DmdMuteSual;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	

	
}
void CAPIVehicleTCUServerGateway::SendEvent_AppUrgMds_TCU_To_IVI(uint32_t _AppUrgMds){
	TCU_Data[9] = _AppUrgMds;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_SignalVoyantDysfunction_TCU_To_IVI(uint32_t _SignalVoyantDysfunction){
	TCU_Data[10] = _SignalVoyantDysfunction;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_SignalVoyantEtat_TCU_To_IVI(uint32_t _SignalVoyantEtat){
	TCU_Data[11] = _SignalVoyantEtat;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_BoostrapRetryCount_TCU_To_IVI(uint32_t _BoostrapRetryCount){
	TCU_Data[12] = _BoostrapRetryCount;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_CloudConnectionStatus_CConnState_connectionGateID_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_connectionGateID){
	TCU_Data[13] = _CloudConnectionStatus_CConnState_connectionGateID;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_CloudConnectionStatus_CConnState_isConnected_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_isConnected){
	TCU_Data[14] = _CloudConnectionStatus_CConnState_isConnected;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_CloudConnectionStatus_CConnState_connType_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_connType){
	TCU_Data[15] = _CloudConnectionStatus_CConnState_connType;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_CloudConnectionStatus_CConnState_isRoaming_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_isRoaming){
	TCU_Data[16] = _CloudConnectionStatus_CConnState_isRoaming;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_CloudConnectionStatus_CConnState_sigStrength_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_sigStrength){
	TCU_Data[17] = _CloudConnectionStatus_CConnState_sigStrength;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_CloudConnectionStatus_CConnState_ipVersion_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_ipVersion){
	TCU_Data[18] = _CloudConnectionStatus_CConnState_ipVersion;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_CellularNetworkType_TCU_To_IVI(uint32_t _CellularNetworkType){
	TCU_Data[19] = _CellularNetworkType;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_NetConnection_TCU_To_IVI(uint32_t _NetConnection){
	TCU_Data[20] = _NetConnection;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_Cellsignal_TCU_To_IVI(uint32_t _Cellsignal){
	TCU_Data[21] = _Cellsignal;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_CloudConnectionStatus_CConnCtrl_reqDisable_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnCtrl_reqDisable){
	TCU_Data[22] = _CloudConnectionStatus_CConnCtrl_reqDisable;
    m_pImpl_TCU->notifyTCU_InternalSignalUpdated(TCU_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_SLIRoadType_TCU_To_IVI(uint32_t _SLIRoadType){
	SLI_Data[0] = _SLIRoadType;
    m_pImpl_TCU->notifySLIdataUpdated(SLI_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_SLISpeedLimit_TCU_To_IVI(uint32_t _SLISpeedLimit){
	SLI_Data[1] = _SLISpeedLimit;
    m_pImpl_TCU->notifySLIdataUpdated(SLI_Data);	
	
}
void CAPIVehicleTCUServerGateway::SendEvent_SLICountryCode_TCU_To_IVI(uint32_t _SLICountryCode){
	SLI_Data[2] = _SLICountryCode;
    m_pImpl_TCU->notifySLIdataUpdated(SLI_Data);	
	
}
void CAPIVehicleTCUServerGateway::Send_Register_App_TCU_To_IVI(std::string appId, std::string appName) {

    IDSReturnValue ret = IDSReturnValue::IDS_ERROR;
    pIdsClientService = IIdsClient::getService();

    if (pIdsClientService == nullptr) {

        ALOGD("CAPIVehcileClientGateway:: Failed to get IDS client service");
        return;
    }
    else {
        ALOGD("CAPIVehcileClientGateway:: Success to get IDS client service");
    }

    ret = pIdsClientService->IdsRegisterApp(appId, appName); 

    if (ret != IDSReturnValue::IDS_OK) {
        ALOGD("CAPIVehcileClientGateway:: Failed to Register Application with IdsClient");
        return;
    }
   
    return;
}

void CAPIVehicleTCUServerGateway::Send_UnRegister_App_TCU_To_IVI() {

    IDSReturnValue ret = IDSReturnValue::IDS_ERROR;
    pIdsClientService = IIdsClient::getService();

    if (pIdsClientService == nullptr) {

        ALOGD("CAPIVehcileClientGateway:: Failed to get IDS client service");
        return;
    }
    else {
        ALOGD("CAPIVehcileClientGateway:: Success to get IDS client service");
    }

    ret = pIdsClientService->IdsUnregisterApp(); 

    if (ret != IDSReturnValue::IDS_OK) {
        ALOGD("CAPIVehcileClientGateway:: Failed to UnRegister Application with IdsClient");
        return;
    }
   
    return;
}

void CAPIVehicleTCUServerGateway::Send_WriteSev_App_TCU_To_IVI(StSecurityEvent &security_event, StLogIssuer &log_issuer, StEventSrc &event_src, StAction &action) {

    IDSReturnValue ret = IDSReturnValue::IDS_ERROR;
    pIdsClientService = IIdsClient::getService();

    if (pIdsClientService == nullptr) {

        ALOGD("CAPIVehcileClientGateway:: Failed to get IDS client service");
        return;
    }
    else {
        ALOGD("CAPIVehcileClientGateway:: Success to get IDS client service");
    }

    ret = pIdsClientService->IdsWriteSev(security_event, log_issuer, event_src, action); 

    if (ret != IDSReturnValue::IDS_OK) {
        ALOGD("CAPIVehcileClientGateway:: Failed to UnRegister Application with IdsClient");
         return;
    }
   
    return;
 }


void CAPIVehicleTCUServerGateway::Send_ExportSecurityEvents_TCU_To_IVI() {

    IDSReturnValue ret = IDSReturnValue::IDS_ERROR;
    pIdsManagerService = IIdsManager::getService();

    if (pIdsManagerService == nullptr) {

        ALOGD("CAPIVehcileClientGateway:: Failed to get IDS Manager service");
        return;
    }
    else {
        ALOGD("CAPIVehcileClientGateway:: Success to get IDS Manager service");
    }

    // Add the call to MGR Here.
    ret = pIdsManagerService->exportIDSSecurityEventLogs();

    if (ret != IDSReturnValue::IDS_OK) {
        ALOGD("CAPIVehcileClientGateway:: Failed to call UnRegister Application with IdsManager");
        return;
    }
   
    return;
    
}

void CAPIVehicleTCUServerGateway::Send_Vehicle_Phase_TCU_To_IVI(std::string vehiclePhase) {

    IDSReturnValue ret = IDSReturnValue::IDS_ERROR;
    pIdsClientService = IIdsClient::getService();

    if (pIdsClientService == nullptr) {

        ALOGD("CAPIVehcileClientGateway:: Failed to get IDS Client service");
        return;
    }
    else {
        ALOGD("CAPIVehcileClientGateway:: Success to get IDS Client service");
    }

    // Add the call to MGR Here.

    //int32_t phase = static_cast<int32_t>(std::stoi(vehiclePhase));
    ret = pIdsClientService->SetVehiclePhase(vehiclePhase);

    if (ret != IDSReturnValue::IDS_OK) {
        ALOGD("CAPIVehcileClientGateway:: Failed to call SetVehiclePhase with Client");
        return;
    }
   
    return;
}

void CAPIVehicleTCUServerGateway::SetUserXcall(uint32_t UserXcall_Data) {
    m_pImpl_TCU->SetUserXcall_TCU(UserXcall_Data);
}
void CAPIVehicleTCUServerGateway::SetLanguage_Selection(uint32_t Language_Selection) {
    m_pImpl_TCU->SetLanguage_Selection_TCU(Language_Selection);
}
void CAPIVehicleTCUServerGateway::Send_Privacy_mode_sts(uint32_t privacy_mode) {
    m_pImpl_TCU->Send_Privacy_mode_sts_TCU(privacy_mode);
}

void CAPIVehicleTCUServerGateway:: SendEvent_UIN_TCUToHMI(std::string UIN_TCU_toHMI) {
	m_pImpl_TCU->UIN_TCU_ToSend_HMI(UIN_TCU_toHMI);
}

void CAPIVehicleTCUServerGateway:: SendEvent_HMI_Trigger_For_UIN(uint32_t HMI_Trigger_for_UIN) {
    m_pImpl_TCU->SendEvent_HMI_Trigger_For_UIN(HMI_Trigger_for_UIN);
}

void CAPIVehicleTCUServerGateway::SendEvent_ReceiveFOTA_Check_for_Updates_Result(std::string FOTA_Check_for_Updates_result){
    FotaHmiCheckforUpdateResult_S = FOTA_Check_for_Updates_result;
    m_pImpl_TCU->FOTAHMI_CheckforUpdates_Result(FOTA_Check_for_Updates_result); 
}
void CAPIVehicleTCUServerGateway::SendEvent_ReceiveFOTA_Update_Available(std::string FOTA_Update_Available){
    FotaHmiUpdateAvailable_S = FOTA_Update_Available;
    m_pImpl_TCU->FOTAHMI_Update_Available(FOTA_Update_Available);
}
void CAPIVehicleTCUServerGateway::SendEvent_ReceiveFOTA_Conditions_not_Met(std::string FOTA_Conditions_not_Met){
    FotaHmiConditionsNotMet_S = FOTA_Conditions_not_Met;
    m_pImpl_TCU->FOTAHMI_Conditions_notMet(FOTA_Conditions_not_Met);
}
void CAPIVehicleTCUServerGateway::SendEvent_ReceiveFOTA_Installation_Status(std::string FOTA_Installation_Status){
    FotaHmiInstallationStatus_S = FOTA_Installation_Status;
    m_pImpl_TCU->FOTAHMI_InstallationStatus(FOTA_Installation_Status);
}
void CAPIVehicleTCUServerGateway::SendEvent_ReceiveFOTAWhatsNew_Details_Response(std::string FOTAWhatsNew_Details_Response){
    FotaHmiWhatsNewDetails_S = FOTAWhatsNew_Details_Response;
    m_pImpl_TCU->FOTAHMI_WhatsNew_Details_Response(FOTAWhatsNew_Details_Response);
}
void CAPIVehicleTCUServerGateway::SendEvent_ReceiveFOTAUpdate_History_Response(std::string FOTAUpdate_History_Response){
    FotaHmiUpdateHistory_S = FOTAUpdate_History_Response;
    m_pImpl_TCU->FOTAHMI_Update_History_Response(FOTAUpdate_History_Response);
}
void CAPIVehicleTCUServerGateway::SendEvent_ReceiveFOTAPending_Updates_Response(std::string FOTAPending_Updates_Response ){
    FotaHmiPendingUpdates_S = FOTAPending_Updates_Response;
    m_pImpl_TCU->FOTAHMI_Pending_Update_History_Response(FOTAPending_Updates_Response);
}
void CAPIVehicleTCUServerGateway::SendEvent_ReceiveFOTA_Update_Finished(std::string FOTA_Update_Finished){
    FotaHmiUpdateFinished_S = FOTA_Update_Finished;
    m_pImpl_TCU->FOTAHMI_Update_Finished(FOTA_Update_Finished);
}
void CAPIVehicleTCUServerGateway::SendEvent_ReceiveFOTASchedule_Update_Response(std::string FOTASchedule_Update_Response){
    FotaHmiScheduleUpdate_S = FOTASchedule_Update_Response;
    m_pImpl_TCU->FOTAHMI_Schedule_Update_Responsee(FOTASchedule_Update_Response);
}
void CAPIVehicleTCUServerGateway::SendEvent_ReceiveAOSP_RBUA_Trigger_Response(std::string AOSPRBUATrigger) {
    AOSPRBUATrigger_S = AOSPRBUATrigger;
    m_pImpl_TCU->AOSP_RBUA_Trigger_Response(AOSPRBUATrigger);
}
void CAPIVehicleTCUServerGateway::SendEvent_FOTAHMI_Info_Result(std::string FOTAHMI_Info_Result) {
    FOTAHMI_InfoResult_S = FOTAHMI_Info_Result;
    m_pImpl_TCU->FotaHmi_InfoResult(FOTAHMI_Info_Result);
}
void CAPIVehicleTCUServerGateway::SendEvent_USBUpdate_Details(std::string USBUpdate_details) {
    USBUpdate_Details_S = USBUpdate_details;
    m_pImpl_TCU->USB_Update_Details(USBUpdate_details);
}
void CAPIVehicleTCUServerGateway:: SendEvent_TCU_Version_Number_toHMI(std::string TCU_version_number) {
	m_pImpl_TCU->TCU_version_Number_to_HMI(TCU_version_number);
}

void CAPIVehicleTCUServerGateway:: Send_HMIFOTA_Update_history(std::string Update_History) {
    m_pImpl_TCU->HMI_FOTA_Update_history(Update_History);
}
void CAPIVehicleTCUServerGateway:: Send_HMIFOTA_pending_updates(std::string Pending_updates) {
    m_pImpl_TCU->HMI_FOTA_pending_updates(Pending_updates);
}
void CAPIVehicleTCUServerGateway::Send_FOTA_WhatsNew_Details(std::string Whatsnewdetails){
    m_pImpl_TCU->FOTA_WhatsNew_Details(Whatsnewdetails);
}
void CAPIVehicleTCUServerGateway::Send_FOTA_Schedule_Update(std::string ScheduleUpdate){
    m_pImpl_TCU->FOTA_Schedule_Update(ScheduleUpdate);
}
void CAPIVehicleTCUServerGateway::Send_FOTA_Update_Dismissed(std::string UpdateDismissed){
    m_pImpl_TCU->FOTA_Update_Dismissed(UpdateDismissed);
}
void CAPIVehicleTCUServerGateway::send_AOSP_Flash_Result(std::string AOSP_Flash_Result){
    m_pImpl_TCU->AOSP_Flash_Result(AOSP_Flash_Result);
}
void CAPIVehicleTCUServerGateway::Send_HMI_Status_SWUpdate(uint32_t HMI_Status_SWUpdate){
    m_pImpl_TCU->HMI_Status_SWUpdate(HMI_Status_SWUpdate);
}

void CAPIVehicleTCUServerGateway:: Send_FOTA_Installation_Finished_with_User_Conformation(std::string Installation_finished){
    m_pImpl_TCU->FOTA_Installation_Finished_with_User_Conformation(Installation_finished);
}
void CAPIVehicleTCUServerGateway:: Send_FOTA_Start_Installation(std::string start_installation){
    m_pImpl_TCU->FOTA_Start_Installation(start_installation);
}
void CAPIVehicleTCUServerGateway:: Send_FOTA_Check_for_Updates(std::string checkfor_updates){
    m_pImpl_TCU->FOTA_Check_for_Updates(checkfor_updates);
}

void CAPIVehicleTCUServerGateway::SetXcall_HMI_Control(uint32_t XCallHmiControl) {
    m_pImpl_TCU->SetXcall_HMI_Control_TCU(XCallHmiControl);
}

void CAPIVehicleTCUServerGateway::SetFlashUnitId(uint32_t FlashUnitId) {
    m_pImpl_TCU->SetFlash_UnitId_TCU(FlashUnitId);
}
void CAPIVehicleTCUServerGateway::SetPackagePath(std::string PackagePath) {
    packagePath_SW = PackagePath;
    m_pImpl_TCU->SetPackage_Path_TCU(PackagePath);
}
void CAPIVehicleTCUServerGateway::SetFlashResult(uint32_t FlashResult) {
        m_pImpl_TCU->SetFlashResult_TCU(FlashResult);
}
void CAPIVehicleTCUServerGateway:: Send_HMIFOTA_Info(std::string hmifota_info) {
    m_pImpl_TCU->HMIFOTA_Info(hmifota_info);
}

void CAPIVehicleTCUServerGateway::SetActive_phone_call_status(uint32_t active_phone_call_status){
	m_pImpl_TCU->SetActive_phone_call_status_TCU(active_phone_call_status);
} 
void CAPIVehicleTCUServerGateway::SetBTCallHMI_Status(uint32_t BtCallHmi_status){
    m_pImpl_TCU->SetBTCallHMI_Status_TCU(BtCallHmi_status);
}
void CAPIVehicleTCUServerGateway::SetBTDeviceAttached_Status(uint32_t BtDeviceAttached_status){
    m_pImpl_TCU->SetBTDeviceAttached_Status_TCU(BtDeviceAttached_status);
}
void CAPIVehicleTCUServerGateway::SetBTTransferTime_Duration(uint32_t BtTransfertime_duration){
    m_pImpl_TCU->SetBTTransferTime_Duration_TCU(BtTransfertime_duration);
}
void CAPIVehicleTCUServerGateway::SetIncomingPhoneCall_Status(uint32_t IncomingPhoneCall_status){
    m_pImpl_TCU->SetIncomingPhoneCall_Status_TCU(IncomingPhoneCall_status);
}
void CAPIVehicleTCUServerGateway::SetOutgoingPhoneCall_Status(uint32_t OutgoingPhoneCall_status){
    m_pImpl_TCU->SetOutgoingPhoneCall_Status_TCU(OutgoingPhoneCall_status);
}
void CAPIVehicleTCUServerGateway::SetBTTransferAcceptance_Status(uint32_t BTTransferAcceptance_status){
    m_pImpl_TCU->SetBTTransferAcceptance_Status_TCU(BTTransferAcceptance_status);
}

void CAPIVehicleClientGateway::init()
{
    LOG(INFO) << "CAPIVehicleClientGateway : Init .";
    m_pImpl->initialize();
    LOG(INFO) << "CAPIVehicleClientGateway : end Init ";
}

bool CAPIVehicleClientGateway::isAvailable() const
{
    return m_pImpl->isAvailable();
}
bool CAPIVehicleClientGateway::isAvailable_dotteProxy() const
{
    return m_pImpl->isAvailable_dotteProxy();
}

void CAPIVehicleClientGateway::getPowerMode(){
    m_pImpl->getPowerMode();
}
void CAPIVehicleClientGateway::getCANData()
{
	 m_pImpl->getCANData();
}
void CAPIVehicleClientGateway::getDIDsubscribe()
{
	m_pImpl->getDIDsubscribe();
}
void CAPIVehicleClientGateway::getIMUsubscribe(){
	m_pImpl->getIMUsubscribe();
}
void CAPIVehicleClientGateway::getDottesubscribe()
{
	 m_pImpl->getDottesubscribe();
}
void CAPIVehicleClientGateway::getDTCsubscribe()
{
	 m_pImpl->getDTCsubscribe();
}

void CAPIVehicleClientGateway::getDEM_DTCStatussubscribe()
{
	 m_pImpl->getDEM_DTCStatussubscribe();
}
void CAPIVehicleClientGateway::getDLTsubscribe()
{
	 m_pImpl->getDLTsubscribe();
}
void CAPIVehicleClientGateway::getLoggerdatasubscribe()
{
	 m_pImpl->getLoggerdatasubscribe();
}
void CAPIVehicleClientGateway::getInternalSigsubscribe()
{
	 m_pImpl->getInternalSigsubscribe();
}
void CAPIVehicleClientGateway::getSOC_Wakeup_reasonsubscribe()
{
	 m_pImpl->getSOC_Wakeup_reasonsubscribe();
}
void CAPIVehicleClientGateway::getMarelliPartNumbersubscribe()
{
	 m_pImpl->getMarelliPartNumbersubscribe();
}
void CAPIVehicleClientGateway::getEOL_hw_variantsubscribe()
{
	 m_pImpl->getEOL_hw_variantsubscribe();
}
void CAPIVehicleClientGateway::getApplicationSoftwareIdentification()
{
	 m_pImpl->getApplicationSoftwareIdentification();
}

void CAPIVehicleClientGateway::getCalibrationDatasubscribe()
{
	 m_pImpl->getCalibrationDatasubscribe();
}
void CAPIVehicleClientGateway::getVinNumbersubscribe()
{
	 m_pImpl->getVinNumbersubscribe();
}
void CAPIVehicleClientGateway::setIVIHMI_iviData(std::vector<uint16_t>& p1)
{
	m_pImpl->setIVIHMI_iviData(p1);
}

void CAPIVehicleClientGateway::setIVIPhoneCallStatus(uint16_t& p1)
{
	m_pImpl->setIVIPhoneCallStatus(p1);
}
 void CAPIVehicleClientGateway::setIVIHMISoftKeyStatus(std::vector<uint16_t>& softkeystatus)
{
	m_pImpl->setIVIHMISoftKeyStatus(softkeystatus);
} 

void CAPIVehicleClientGateway::setIVISmsStatus(uint16_t& p1)
{
	m_pImpl->setIVISmsStatus(p1);
}
void CAPIVehicleClientGateway::setIVI_LogExtractionTriggerStatus(uint16_t& p1)
{
	m_pImpl->setIVI_LogExtractionTriggerStatus(p1);
}
void CAPIVehicleClientGateway::setIVI_DLTExtractionTriggerStatus(uint16_t& p1)
{
	m_pImpl->setIVI_DLTExtractionTriggerStatus(p1);
}
void CAPIVehicleClientGateway::setIVI_TX_Signals(uint16_t IVI_TX_Signals , uint16_t tx_enum_number)
{
	m_pImpl->setIVI_TX_Signals(IVI_TX_Signals,tx_enum_number);
}
void CAPIVehicleClientGateway::getTemperaturesubscribe()
{
	 m_pImpl->getTemperaturesubscribe();
}
void CAPIVehicleClientGateway::getsupplyvgesubscribe()
{
    m_pImpl->getsupplyvgesubscribe();
}
void CAPIVehicleClientGateway::setDCSD_DIAG_Res(std::vector<uint8_t>& DCSD_DIAG_Res)
{
	m_pImpl->setDCSD_DIAG_Res(DCSD_DIAG_Res);
}
void CAPIVehicleClientGateway::send_arvc_status_mcu(uint8_t& arvc_status)
{
	m_pImpl->send_arvc_status_mcu(arvc_status);
}
void CAPIVehicleClientGateway::getCallActiveSTRDisplayPopupStatusSubscribe(){
    LOG(INFO) << "CAPIVehicleClientGateway::getCallActiveSTRDisplayPopupStatusSubscribe()";
    m_pImpl->getCallActiveSTRDisplayPopupStatusSubscribe();
}
void CAPIVehicleClientGateway::getSTRCancelShutdownSubscribe(){
    LOG(INFO) << "CAPIVehicleClientGateway::getSTRCancelShutdownSubscribe()";
    m_pImpl->getSTRCancelShutdownSubscribe();
}
void CAPIVehicleClientGateway::getUSB_Update_detailssubscribe()
{
	 m_pImpl->getUSB_Update_detailssubscribe();
}
void CAPIVehicleClientGateway::getUSB_Updatesubscribe()
{
	 m_pImpl->getUSB_Updatesubscribe();
}
void CAPIVehicleClientGateway::setIVI_CalibrationTriggerStatus(uint16_t& p1)
{
	m_pImpl->setIVI_CalibrationTriggerStatus(p1);
}

void CAPIVehicleClientGateway::setIVI_S2RStatus(uint8_t& p1)
{
	m_pImpl->setIVI_S2RStatus(p1);
}
void CAPIVehicleClientGateway::setIVI_DIDdata_request(std::vector<uint16_t>& p1) 
{
	m_pImpl->setIVI_DIDdata_request(p1);
}
void CAPIVehicleClientGateway::setIVI_USB_Update(std::vector<uint16_t>& p1)
{
	m_pImpl->setIVI_USB_Update(p1);
}
void CAPIVehicleClientGateway::setIVI_Usb_Reset_TriggerStatus(uint16_t& p1)
{
	m_pImpl->setIVI_Usb_Reset_TriggerStatus(p1);
}
void CAPIVehicleClientGateway::setSOC_Ready_Status()
{
	m_pImpl->setSOC_Ready_Status();
}
void CAPIVehicleClientGateway::setIVI_CallActiveSTR_HMIPopup_Status(uint8_t& userInputFromHMIPopup)
{
	m_pImpl->setIVI_CallActiveSTR_HMIPopup_Status(userInputFromHMIPopup);
}
void CAPIVehicleClientGateway::setDEM_DTCStatus(std::vector<uint8_t>& p1)
{
	m_pImpl->setDEM_DTCStatus(p1);
}
void CAPIVehicleClientGateway::SetLoggerdata(std::vector<uint32_t>& Logger_Data) {
    m_pImpl->SetLoggerdata(Logger_Data);
}
void CAPIVehicleClientGateway::SetInternal_Fan_Speed(uint32_t& p1)
{
	m_pImpl->SetInternal_Fan_Speed(p1);
}
void CAPIVehicleClientGateway::setIVIResettrigger(uint32_t& p1)
{
	m_pImpl->setIVIResettrigger(p1);
}
void CAPIVehicleClientGateway::SetSystemTimeZone(int32_t& p1)
{
	m_pImpl->SetSystemTimeZone(p1);
}
void CAPIVehicleClientGateway::SendFilepathtoTCU(std::string& p1)
{
	m_pImpl->SendFilepathtoTCU(p1);
}
void CAPIVehicleClientGateway::getlogretrieverstatus(uint32_t& p1)
{
	m_pImpl->getlogretrieverstatus(p1);
}
void CAPIVehicleClientGateway::setIVI_SoCVersion_Send_MCU()
{
	m_pImpl->setIVI_SoCVersion_Send_MCU();
}
void CAPIVehicleClientGateway::setIVI_hw_variant_id_Send_MCU()
{
	m_pImpl->setIVI_hw_variant_id_Send_MCU();
}

void CAPIVehicleClientGateway::registerSpeedCallback(std::function<void(uint32_t)> callback){
     m_pImpl->registerSpeedCallback(callback);
}
void CAPIVehicleClientGateway::registerIgnitionStsCallback(std::function<void(uint32_t)> IgnitionSts){
     m_pImpl->registerIgnitionStsCallback(IgnitionSts);
}
void CAPIVehicleClientGateway::registerParkLampCallback(std::function<void(uint32_t)> ParkLamp){
     m_pImpl->registerParkLampCallback(ParkLamp);
}
void CAPIVehicleClientGateway::registerEcallCrashCallback(std::function<void(uint32_t)> EcallCrash){
     m_pImpl->registerEcallCrashCallback(EcallCrash);
}
void CAPIVehicleClientGateway::registerDayNightModeCallback(std::function<void(uint32_t)> DayNightMode){
     m_pImpl->registerDayNightModeCallback(DayNightMode);
}
void CAPIVehicleClientGateway::registerFuellevelCallback(std::function<void(uint32_t)> Fuellevel){
     m_pImpl->registerFuellevelCallback(Fuellevel);
}
void CAPIVehicleClientGateway::registerFuelTypeCallback(std::function<void(uint32_t)> FuelType){
     m_pImpl->registerFuelTypeCallback(FuelType);
}
void CAPIVehicleClientGateway::registerParkBreakStatusCallback(std::function<void(uint32_t)> ParkBreakStatus){
     m_pImpl->registerParkBreakStatusCallback(ParkBreakStatus);
}
void CAPIVehicleClientGateway::registerOutTempCallback(std::function<void(uint32_t)> OutTemp){
     m_pImpl->registerOutTempCallback(OutTemp);
}
void CAPIVehicleClientGateway::registerHVBatterySOHDataCallback(std::function<void(uint32_t)> HVBatterySOHData){
     m_pImpl->registerHVBatterySOHDataCallback(HVBatterySOHData);
}
void CAPIVehicleClientGateway::registerHVBatteryPercentageCallback(std::function<void(uint32_t)> HVBatteryPercentage){
     m_pImpl->registerHVBatteryPercentageCallback(HVBatteryPercentage);
}
void CAPIVehicleClientGateway::registerHVBatteryMinChargeCallback(std::function<void(uint32_t)> HVBatteryMinCharge){
     m_pImpl->registerHVBatteryMinChargeCallback(HVBatteryMinCharge);
}
void CAPIVehicleClientGateway::registerHVBatteryActiveConnectorCallback(std::function<void(uint32_t)> HVBatteryActiveConnector){
     m_pImpl->registerHVBatteryActiveConnectorCallback(HVBatteryActiveConnector);
}
void CAPIVehicleClientGateway::registerHVBatteryMaxRangeCallback(std::function<void(uint32_t)> HVBatteryMaxRange){
     m_pImpl->registerHVBatteryMaxRangeCallback(HVBatteryMaxRange);
}
void CAPIVehicleClientGateway::registerHVBatteryChargeLevelCallback(std::function<void(uint32_t)> HVBatteryChargeLevel){
     m_pImpl->registerHVBatteryChargeLevelCallback(HVBatteryChargeLevel);
}
void CAPIVehicleClientGateway::registerHVBatteryIsChargingCallback(std::function<void(uint32_t)> HVBatteryIsCharging){
     m_pImpl->registerHVBatteryIsChargingCallback(HVBatteryIsCharging);
}
void CAPIVehicleClientGateway::registerAutonomyCallback(std::function<void(uint32_t)> Autonomy){
     m_pImpl->registerAutonomyCallback(Autonomy);
}
void CAPIVehicleClientGateway::registerBrakePedalrStsCallback(std::function<void(uint32_t)> BrakePedalrSts){
     m_pImpl->registerBrakePedalrStsCallback(BrakePedalrSts);
}
void CAPIVehicleClientGateway::registerCanDistanceUnitCallback(std::function<void(uint32_t)> CanDistanceUnit){
     m_pImpl->registerCanDistanceUnitCallback(CanDistanceUnit);
}
void CAPIVehicleClientGateway::registerInstantFuelConsCallback(std::function<void(uint32_t)> InstantFuelCons){
     m_pImpl->registerInstantFuelConsCallback(InstantFuelCons);
}
void CAPIVehicleClientGateway::registerReverseGearEngagedConsCallback(std::function<void(uint32_t)> ReverseGearEngaged){
     m_pImpl->registerReverseGearEngagedConsCallback(ReverseGearEngaged);
}
void CAPIVehicleClientGateway::registerRoadSlopeCallback(std::function<void(uint32_t)> RoadSlope){
     m_pImpl->registerRoadSlopeCallback(RoadSlope);
}
void CAPIVehicleClientGateway::registerSteeringAngleCallback(std::function<void(uint32_t)> SteeringAngle){
     m_pImpl->registerSteeringAngleCallback(SteeringAngle);
}
void CAPIVehicleClientGateway::registerTotalKMCallback(std::function<void(uint32_t)> TotalKM){
     m_pImpl->registerTotalKMCallback(TotalKM);
}
void CAPIVehicleClientGateway::registerTransmGearDisplayCallback(std::function<void(uint32_t)> TransmGearDisplay){
     m_pImpl->registerTransmGearDisplayCallback(TransmGearDisplay);
}
void CAPIVehicleClientGateway::registerTripAvgBattPowerConsCallback(std::function<void(uint32_t)> TripAvgBattPowerCons){
     m_pImpl->registerTripAvgBattPowerConsCallback(TripAvgBattPowerCons);
}
void CAPIVehicleClientGateway::registerYawSpeedCallback(std::function<void(uint32_t)> YawSpeed){
     m_pImpl->registerYawSpeedCallback(YawSpeed);
}
void CAPIVehicleClientGateway::registerGrossYawRateCallback(std::function<void(uint32_t)> GrossYawRate){
     m_pImpl->registerGrossYawRateCallback(GrossYawRate);
}
void CAPIVehicleClientGateway::registerFLWheelTickCounterFaultCallback(std::function<void(uint32_t)> FLWheelTickCounterFault){
     m_pImpl->registerFLWheelTickCounterFaultCallback(FLWheelTickCounterFault);
}
void CAPIVehicleClientGateway::registerFRWheelTickCounterFaultCallback(std::function<void(uint32_t)> FRWheelTickCounterFault){
     m_pImpl->registerFRWheelTickCounterFaultCallback(FRWheelTickCounterFault);
}
void CAPIVehicleClientGateway::registerRLWheelTickCounterFaultCallback(std::function<void(uint32_t)> RLWheelTickCounterFault){
     m_pImpl->registerRLWheelTickCounterFaultCallback(RLWheelTickCounterFault);
}
void CAPIVehicleClientGateway::registerRRWheelTickCounterFaultCallback(std::function<void(uint32_t)> RRWheelTickCounterFault){
     m_pImpl->registerRRWheelTickCounterFaultCallback(RRWheelTickCounterFault);
}
void CAPIVehicleClientGateway::registerPulseCountFLWheelCallback(std::function<void(uint32_t)> PulseCountFLWheel){
     m_pImpl->registerPulseCountFLWheelCallback(PulseCountFLWheel);
}
void CAPIVehicleClientGateway::registerPulseCountFRWheelCallback(std::function<void(uint32_t)> PulseCountFRWheel){
     m_pImpl->registerPulseCountFRWheelCallback(PulseCountFRWheel);
}
void CAPIVehicleClientGateway::registerPulseCountRLWheelCallback(std::function<void(uint32_t)> PulseCountRLWheel){
     m_pImpl->registerPulseCountRLWheelCallback(PulseCountRLWheel);
}
void CAPIVehicleClientGateway::registerPulseCountRRWheelCallback(std::function<void(uint32_t)> PulseCountRRWheel){
     m_pImpl->registerPulseCountRRWheelCallback(PulseCountRRWheel);
}
void CAPIVehicleClientGateway::registerFPASStsCallback(std::function<void(uint32_t)> FPASSts){
     m_pImpl->registerFPASStsCallback(FPASSts);
}
void CAPIVehicleClientGateway::registerDrivingDirectionCallback(std::function<void(uint32_t)> DrivingDirection){
     m_pImpl->registerDrivingDirectionCallback(DrivingDirection);
}
void CAPIVehicleClientGateway::registerCapacityUnitCallback(std::function<void(uint32_t)> CapacityUnit){
     m_pImpl->registerCapacityUnitCallback(CapacityUnit);
}
void CAPIVehicleClientGateway::registerCombustFuelLvCallback(std::function<void(uint32_t)> CombustFuelLv){
     m_pImpl->registerCombustFuelLvCallback(CombustFuelLv);
}
void CAPIVehicleClientGateway::registerCompressorStsCallback(std::function<void(uint32_t)> CompressorSts){
     m_pImpl->registerCompressorStsCallback(CompressorSts);
}
void CAPIVehicleClientGateway::registerElectricMotorPowerCallback(std::function<void(int32_t)> ElectricMotorPower){
     m_pImpl->registerElectricMotorPowerCallback(ElectricMotorPower);
}
void CAPIVehicleClientGateway::registerEngineCoolantTempCallback(std::function<void(uint32_t)> EngineCoolantTemp){
     m_pImpl->registerEngineCoolantTempCallback(EngineCoolantTemp);
}
void CAPIVehicleClientGateway::registerEngineRPMCallback(std::function<void(uint32_t)> EngineRPM){
     m_pImpl->registerEngineRPMCallback(EngineRPM);
}
void CAPIVehicleClientGateway::registerFuelLevelMinimumStsCallback(std::function<void(uint32_t)> FuelLevelMinimumSts){
     m_pImpl->registerFuelLevelMinimumStsCallback(FuelLevelMinimumSts);
}
void CAPIVehicleClientGateway::registerAbsSteeringWheelAngleAcuCallback(std::function<void(uint32_t)> AbsSteeringWheelAngleAcu){
     m_pImpl->registerAbsSteeringWheelAngleAcuCallback(AbsSteeringWheelAngleAcu);
}
void CAPIVehicleClientGateway::registerInitSteeringWheenAngleFlagCallback(std::function<void(uint32_t)> InitSteeringWheenAngleFlag){
     m_pImpl->registerInitSteeringWheenAngleFlagCallback(InitSteeringWheenAngleFlag);
}
void CAPIVehicleClientGateway::registerNetworkMNGTCallback(std::function<void(uint32_t)> NetworkMNGT){
     m_pImpl->registerNetworkMNGTCallback(NetworkMNGT);
}
void CAPIVehicleClientGateway::registerFPASCenterLeftBarStsCallback(std::function<void(uint32_t)> FPASCenterLeftBarSts){
     m_pImpl->registerFPASCenterLeftBarStsCallback(FPASCenterLeftBarSts);
}
void CAPIVehicleClientGateway::registerFPASCenterRightBarStsCallback(std::function<void(uint32_t)> FPASCenterRightBarSts){
     m_pImpl->registerFPASCenterRightBarStsCallback(FPASCenterRightBarSts);
}
void CAPIVehicleClientGateway::registerFPASLeftBarStsCallback(std::function<void(uint32_t)> FPASLeftBarSts){
     m_pImpl->registerFPASLeftBarStsCallback(FPASLeftBarSts);
}
void CAPIVehicleClientGateway::registerRPASCenterLeftBarStsCallback(std::function<void(uint32_t)> RPASCenterLeftBarSts){
     m_pImpl->registerRPASCenterLeftBarStsCallback(RPASCenterLeftBarSts);
}
void CAPIVehicleClientGateway::registerRPASCenterRightBarStsCallback(std::function<void(uint32_t)> RPASCenterRightBarSts){
     m_pImpl->registerRPASCenterRightBarStsCallback(RPASCenterRightBarSts);
}
void CAPIVehicleClientGateway::registerFPASRightBarStsCallback(std::function<void(uint32_t)> FPASRightBarSts){
     m_pImpl->registerFPASRightBarStsCallback(FPASRightBarSts);
}
void CAPIVehicleClientGateway::registerRPASLeftBarStsCallback(std::function<void(uint32_t)> RPASLeftBarSts){
     m_pImpl->registerRPASLeftBarStsCallback(RPASLeftBarSts);
}
void CAPIVehicleClientGateway::registerTrailerPresentCallback(std::function<void(uint32_t)> TrailerPresent){
     m_pImpl->registerTrailerPresentCallback(TrailerPresent);
}
void CAPIVehicleClientGateway::registerTrunkDoorStsCallback(std::function<void(uint32_t)> TrunkDoorSts){
     m_pImpl->registerTrunkDoorStsCallback(TrunkDoorSts);
}
void CAPIVehicleClientGateway::registerRPASStsCallback(std::function<void(uint32_t)> RPASSts){
     m_pImpl->registerRPASStsCallback(RPASSts);
}
void CAPIVehicleClientGateway::registerEconShftRqCallback(std::function<void(uint32_t)> EconShftRq){
     m_pImpl->registerEconShftRqCallback(EconShftRq);
}
void CAPIVehicleClientGateway::registerEngineTorqueCallback(std::function<void(uint32_t)> EngineTorque){
     m_pImpl->registerEngineTorqueCallback(EngineTorque);
}
void CAPIVehicleClientGateway::registerFuelConsUnitCallback(std::function<void(uint32_t)> FuelConsUnit){
     m_pImpl->registerFuelConsUnitCallback(FuelConsUnit);
}
void CAPIVehicleClientGateway::registerRegeneratedEnergyCallback(std::function<void(uint32_t)> RegeneratedEnergy){
     m_pImpl->registerRegeneratedEnergyCallback(RegeneratedEnergy);
}
void CAPIVehicleClientGateway::registerShiftLeverPositionCallback(std::function<void(uint32_t)> ShiftLeverPosition){
     m_pImpl->registerShiftLeverPositionCallback(ShiftLeverPosition);
}
void CAPIVehicleClientGateway::registerTripAverageFuelConsumptionCallback(std::function<void(uint32_t)> TripAverageFuelConsumption){
     m_pImpl->registerTripAverageFuelConsumptionCallback(TripAverageFuelConsumption);
}
void CAPIVehicleClientGateway::registerReqVolCtrlCallback(std::function<void(uint32_t)> ReqVolCtrl){
     m_pImpl->registerReqVolCtrlCallback(ReqVolCtrl);
}
void CAPIVehicleClientGateway::registerVinNumberCallback(std::function<void(std::vector<uint32_t>)> vinNumber){
     m_pImpl->registerVinNumberCallback(vinNumber);
}
void CAPIVehicleClientGateway::registerBCMHMItimeCallback(std::function<void(uint32_t [])> bcmHMItime){
     m_pImpl->registerBCMHMItimeCallback(bcmHMItime);
}
void CAPIVehicleClientGateway::registerBCMSECURtimeCallback(std::function<void(uint32_t [])> bcmSECURtime){
     m_pImpl->registerBCMSECURtimeCallback(bcmSECURtime);
}
void CAPIVehicleClientGateway::registerIMUdata_callback(std::function<void(std::vector<int32_t>)> imuDataParam){
	m_pImpl->registerIMUdata_callback(imuDataParam);
}
void CAPIVehicleClientGateway::registerDotteParamCallback(std::function<void(std::vector<uint32_t>)> dotteParam){
	m_pImpl->registerDotteParamCallback(dotteParam);
}
void CAPIVehicleClientGateway::registerInternalSignalCallback(std::function<void(uint32_t [])>  internalSignal){
	 m_pImpl->registerInternalSignalCallback(internalSignal);
}
void CAPIVehicleClientGateway::registerDTCParamCallback(std::function<void(std::vector<uint32_t>)> dtcParam){
	m_pImpl->registerDTCParamCallback(dtcParam);
}

void CAPIVehicleTCUServerGateway::registerFotaHMICheckforResultsCallback(std::function<void(std::string)> m_FOTAHMI_CheckforResults_DATA){
    m_pImpl_TCU->registerFotaHMICheckforResultsCallback(m_FOTAHMI_CheckforResults_DATA);
}
void CAPIVehicleTCUServerGateway::registerFotaHMIUpdateAvailableCallback(std::function<void(std::string)> m_FOTAHMI_UpdateAvailable_DATA){
    m_pImpl_TCU->registerFotaHMIUpdateAvailableCallback(m_FOTAHMI_UpdateAvailable_DATA);
}
void CAPIVehicleTCUServerGateway::registerFotaHMIConditionsnotmetCallback(std::function<void(std::string)> m_FOTAHMI_ConditionsnotMet_DATA){
    m_pImpl_TCU->registerFotaHMIConditionsnotmetCallback(m_FOTAHMI_ConditionsnotMet_DATA);
}
void CAPIVehicleTCUServerGateway::registerFotaHMIInstallationStatusCallback(std::function<void(std::string)> m_FOTAHMI_InstallationStatus_DATA){
    m_pImpl_TCU->registerFotaHMIInstallationStatusCallback(m_FOTAHMI_InstallationStatus_DATA);
}
void CAPIVehicleTCUServerGateway::registerFotaHMIWhatNewDetailsCallback(std::function<void(std::string)> m_FOTAWhatsNew_Details_Response){
    m_pImpl_TCU->registerFotaHMIWhatNewDetailsCallback(m_FOTAWhatsNew_Details_Response);
}
void CAPIVehicleTCUServerGateway::registerFotaHMIUpdateHistoryResponseCallback(std::function<void(std::string)> m_FOTAUpdate_History_Response){
    m_pImpl_TCU->registerFotaHMIUpdateHistoryResponseCallback(m_FOTAUpdate_History_Response);
}
void CAPIVehicleTCUServerGateway::registerFotaHMIPendingUpdateHistoryCallback(std::function<void(std::string)> m_FOTAPending_Updates_Response){
    m_pImpl_TCU->registerFotaHMIPendingUpdateHistoryCallback(m_FOTAPending_Updates_Response);
}
void CAPIVehicleTCUServerGateway::registerFotaHMIUpdateFinishedCallback(std::function<void(std::string)> m_FOTA_Update_Finished){
    m_pImpl_TCU->registerFotaHMIUpdateFinishedCallback(m_FOTA_Update_Finished);
}
void CAPIVehicleTCUServerGateway::registerFotaHMIUpdateScheduleCallback(std::function<void(std::string)> m_FOTASchedule_Update_Response){
    m_pImpl_TCU->registerFotaHMIUpdateScheduleCallback(m_FOTASchedule_Update_Response);
}
void CAPIVehicleTCUServerGateway::registerAOSPRBUATriggerCallback(std::function<void(std::string)> m_AOSPRBUATrigger){
    m_pImpl_TCU->registerAOSPRBUATriggerCallback(m_AOSPRBUATrigger);
}
void CAPIVehicleTCUServerGateway::registerFotaHmiInfoResultCallback(std::function<void(std::string)> m_FOTAHMI_Info_Result){
    m_pImpl_TCU->registerFotaHmiInfoResultCallback(m_FOTAHMI_Info_Result);
}
void CAPIVehicleTCUServerGateway::registerUSB_Update_detailsCallback(std::function<void(std::string)> m_USBUpdate_details){
    m_pImpl_TCU->registerUSB_Update_detailsCallback(m_USBUpdate_details);
}
void CAPIVehicleTCUServerGateway::registerUIN_TCUToHMICallback(std::function<void(std::string)> UIN_TCU_ToHMI_Data){
	m_pImpl_TCU->registerUIN_TCUToHMICallback(UIN_TCU_ToHMI_Data);
}
void CAPIVehicleTCUServerGateway::registerTCUVersiontoHMICallback(std::function<void(std::string)> TCU_version_to_HMI){
    m_pImpl_TCU->registerTCUVersiontoHMICallback(TCU_version_to_HMI);
}
void CAPIVehicleClientGateway::registerDLTParamCallback(std::function<void(std::vector<uint32_t>)> dLTParam){
	m_pImpl->registerDLTParamCallback(dLTParam);
}
void CAPIVehicleClientGateway::registerDLT_VectorParamCallback(std::function<void(std::vector<uint8_t>)> dLT_VectorValueParam) {
	m_pImpl->registerDLT_VectorParamCallback(dLT_VectorValueParam);
}
void CAPIVehicleClientGateway::registerCalibrationDataCallback(std::function<void(uint32_t [])> calDATA){
     m_pImpl->registerCalibrationDataCallback(calDATA);
}
void CAPIVehicleClientGateway::registerCalibrationDataCallback_Audio(std::function<void(uint32_t [])> calDATA){
     m_pImpl->registerCalibrationDataCallback_Audio(calDATA);
}
void CAPIVehicleClientGateway::registerCalibrationDataCallback_Camera(std::function<void(uint32_t [])> calDATA){
     m_pImpl->registerCalibrationDataCallback_Camera(calDATA);
}
void CAPIVehicleClientGateway::registerCalibrationDataCallback_Array(std::function<void(std::vector<uint8_t>)> calDATA){
     m_pImpl->registerCalibrationDataCallback_Array(calDATA);
}
void CAPIVehicleClientGateway::registercalibrationDataforSrecCallback(std::function<void(std::vector<uint32_t>)> calSrecDATA){
     m_pImpl->registercalibrationDataforSrecCallback(calSrecDATA);
}
void CAPIVehicleClientGateway::registerTemperatureCallback(std::function<void(uint32_t)> Temperature){
     m_pImpl->registerTemperatureCallback(Temperature);
}
void CAPIVehicleClientGateway::registerSupplyVoltageCallback(std::function<void(uint32_t)> SupplyVoltage){
    m_pImpl->registerSupplyVoltageCallback(SupplyVoltage);
}
void CAPIVehicleClientGateway::registerCallActiveSTRDisplayPopupStatusCallback(std::function<void(uint32_t)> callActiveSTRDisplayPopupStatus){
    m_pImpl->registerCallActiveSTRDisplayPopupStatusCallback(callActiveSTRDisplayPopupStatus);
}
void CAPIVehicleClientGateway::registerSTRCancelShutdownStatusCallback(std::function<void(uint32_t)> STRCancelShutdownStatus){
    m_pImpl->registerSTRCancelShutdownStatusCallback(STRCancelShutdownStatus);
}
void CAPIVehicleClientGateway::registerUSB_UpdateCallback(std::function<void(std::vector<uint16_t>)> USB_Update){
	m_pImpl->registerUSB_UpdateCallback(USB_Update);
}
void CAPIVehicleClientGateway::registerEVSignalsCallback(std::function<void(int32_t [])>  EVSignal){
	 m_pImpl->registerEVSignalsCallback(EVSignal);
}
void CAPIVehicleClientGateway::registerCanCommonSignalsCallback(std::function<void(int32_t [])>  CanCommonSignal){
	 m_pImpl->registerCanCommonSignalsCallback(CanCommonSignal);
}
void CAPIVehicleClientGateway::registerCameraSignalsCallback(std::function<void(uint32_t [])>  CameraSignal){
	 m_pImpl->registerCameraSignalsCallback(CameraSignal);
}
void CAPIVehicleClientGateway::registerBCMSignalsCallback(std::function<void(uint32_t [])>  BCMSignal){
	 m_pImpl->registerBCMSignalsCallback(BCMSignal);
}
void CAPIVehicleClientGateway::registerVehPrivacyModeCallback(std::function<void(uint32_t)> VehPrivacyMode){
     m_pImpl->registerVehPrivacyModeCallback(VehPrivacyMode);
}
void CAPIVehicleClientGateway::registerDID_RID_callback(std::function<void(std::vector<uint16_t>)> m_dID_message_type){
     m_pImpl->registerDID_RID_callback(m_dID_message_type);
}
void CAPIVehicleTCUServerGateway::registerTCUSignalCallback(std::function<void(uint32_t [])>  TCU_internalSignal){
	 m_pImpl_TCU->registerTCUSignalCallback(TCU_internalSignal);
}
void CAPIVehicleClientGateway::registerPowermodeCallback(std::function<void(uint32_t)> Powermode){
    m_pImpl->registerPowermodeCallback(Powermode);
}
void CAPIVehicleClientGateway::registerCAN_RX_SignalsCallback(std::function<void(uint32_t [])>  CAN_RX_Signals){
    m_pImpl->registerCAN_RX_SignalsCallback(CAN_RX_Signals);
}
void CAPIVehicleClientGateway::registerSigned_CAN_RX_SignalsCallback(std::function<void(int32_t [])>  Signed_CAN_RX_Signals){
    m_pImpl->registerSigned_CAN_RX_SignalsCallback(Signed_CAN_RX_Signals);
}
void CAPIVehicleClientGateway::registerDEM_DTCstatuscallback(std::function<void(std::vector<uint16_t>)> m_diag_wakeup_message_type){
	 m_pImpl->registerDEM_DTCstatuscallback(m_diag_wakeup_message_type);
}
void CAPIVehicleClientGateway::register_DriverdoorStatuscallback(std::function<void(uint8_t)> driverdoorStatus){
	 m_pImpl->register_DriverdoorStatuscallback(driverdoorStatus);
}
void CAPIVehicleClientGateway::register_PassengerDoorStatuscallback(std::function<void(uint8_t)> passengerDoorStatus){
	 m_pImpl->register_PassengerDoorStatuscallback(passengerDoorStatus);
}
void CAPIVehicleClientGateway::registerLog_RetriverCallback(std::function<void(uint32_t)> Log_Retriver_data){
    m_pImpl->registerLog_RetriverCallback(Log_Retriver_data);
}
void CAPIVehicleClientGateway::registerlogerfilepathCallback(std::function<void(std::string)> logger_filepath){
    m_pImpl->registerlogerfilepathCallback(logger_filepath);
}
void CAPIVehicleTCUServerGateway::registerSLIDataCallback(std::function<void(uint32_t [])>  SLI_Data){
    m_pImpl_TCU->registerSLIDataCallback(SLI_Data);
}
void CAPIVehicleClientGateway::registerLog_RetriverStatusCallback(std::function<void(uint32_t)> Log_Retriver_status){
    m_pImpl->registerLog_RetriverStatusCallback(Log_Retriver_status);
}
void CAPIVehicleClientGateway::registerSOC_WakeupReasonCallback(std::function<void(uint32_t)> WakeupReason){
    m_pImpl->registerSOC_WakeupReasonCallback(WakeupReason);
}
void CAPIVehicleTCUServerGateway::registerFlashUnitIdCallback(std::function<void(uint32_t)> Flash_UnitId){
    m_pImpl_TCU->registerFlashUnitIdCallback(Flash_UnitId);
}
void CAPIVehicleTCUServerGateway::registerPackagePathCallback(std::function<void(std::string)> package_filepath){
    m_pImpl_TCU->registerPackagePathCallback(package_filepath);
}
void CAPIVehicleClientGateway::registerRXSignals_FloatCallback(std::function<void(float [])> RXSignals_Float){
    m_pImpl->registerRXSignals_FloatCallback(RXSignals_Float);
}
void CAPIVehicleClientGateway::registerMarelliPNCallback(std::function<void(std::vector<uint8_t>)> MarelliPN) {
	m_pImpl->registerMarelliPNCallback(MarelliPN);
}
void CAPIVehicleClientGateway::registerEOLHWIDCallback(std::function<void(std::vector<uint8_t>)> EOLHWID) {
	m_pImpl->registerEOLHWIDCallback(EOLHWID);
}
void CAPIVehicleClientGateway::registerAuthenticationZoneCallback(std::function<void(std::vector<uint8_t>)> AuthenticationZone) {
	m_pImpl->registerAuthenticationZoneCallback(AuthenticationZone);
}
void CAPIVehicleClientGateway::registerApplicationSoftwareIdentificationCallback(std::function<void(std::vector<uint8_t>)> ApplicationSoftwareIdentification) {
	m_pImpl->registerApplicationSoftwareIdentificationCallback(ApplicationSoftwareIdentification);
}
void CAPIVehicleClientGateway::registerApplicationDataIdentificationCallback(std::function<void(std::vector<uint8_t>)> ApplicationDataIdentification) {
	m_pImpl->registerApplicationDataIdentificationCallback(ApplicationDataIdentification);
}
void CAPIVehicleClientGateway::registerDataLibraryidentifierCallback(std::function<void(std::vector<uint8_t>)> DataLibraryidentifier) {
	m_pImpl->registerDataLibraryidentifierCallback(DataLibraryidentifier);
}
void CAPIVehicleClientGateway::registerVehicleAppsIdentifierCallback(std::function<void(std::vector<uint8_t>)> VehicleAppsIdentifier) {
	m_pImpl->registerVehicleAppsIdentifierCallback(VehicleAppsIdentifier);
}
void CAPIVehicleClientGateway::registerDCSD_DIAG_ReqCallback(std::function<void(std::vector<uint8_t>)> DCSD_DIAG_Req) {
	m_pImpl->registerDCSD_DIAG_ReqCallback(DCSD_DIAG_Req);
}

}
}
}
}
}
