#pragma once

#include <memory>
#include <inttypes.h>
#include <stdint.h>
#include <sys/types.h>

#include <list>
#include <map>
#include <set>

#include <android/hardware/automotive/vehicle/2.0/IVehicle.h>
//IDS
#include <hidl/LegacySupport.h>
#include <hidl/HidlSupport.h>
#include <hidl/HidlTransportSupport.h>
#include <vendor/securethings/hardware/ids/ids_client/1.0/IIdsClient.h>
#include <vendor/securethings/hardware/ids/ids_manager/1.0/IIdsManager.h>
#include <vendor/securethings/hardware/ids/ids_client/1.0/types.h>
#include <hidl/MQDescriptor.h>
#include <hidl/Status.h>
#include <unistd.h>
#include <ids_client.h>
//IDS_END

#define Dotte_strut_size 108 
#define vinNumber_ID 0x354
#define DTC_ID 0x355
#define vinNumber_size 21
#define SECUREtime_index 7
#define HMItime_index 6
#define HMI_OLDDATA 7
#define SECURE_OLDDATA 6
#define EXTGPS_OLDDATA 6
#define BATTERYPRECONDITION_OLDDATA 4
#define INTERNALSignal_index 17
#define EVSignals_index 86 
#define BCMSignals_index 36
#define OFFSET_YEAR 2000
#define DLTVectorValue_index 9
#define CameraSignals_index 2
#define CanCommonSignal_index 85
#define IVIBatteryPreconditioning_index 4
#define IVI_EXTGPS_Index 6
#define internal_audio_length 18
#define internal_camera_length 20
#define internal_hmi_length 6
#define internal_projection_length 7
#define internal_second_party_App_length 13
#define CALDATA 213
#define CALDATA_AUDIO 2383
#define CALDATA_CAMERA 239
#define TCU_InternalSignals_Index 23
#define Internal_SystemTime_Index 23
#define Softkeystatus_index 6
#define HMITouchData_index 9
#define CAN_RX_Signal_Index 207
#define Signed_CAN_RX_signals_Index 9
#define IVI_TX_Signals_size 142
#define SLI_data_length 3
#define FOTA_data_length 5
#define TX_Buffer_size 8
#define SOC_VERSION_SIZE 30
// #define FOTA_HMIIntData_lenth 9
#define ContainerID 1
#define MsgID 1
#define AppID 1
#define RXSignals_Float_Index 156




typedef uint8_t uint8 ;
typedef uint16_t uint16 ;
typedef uint32_t uint32 ;
typedef bool boolean ;


namespace android {
namespace hardware {
namespace automotive {
namespace vehicle {
namespace V2_0 {

// //IDS
using ::android::hardware::hidl_array;
using ::android::hardware::hidl_memory;
using ::android::hardware::hidl_string;
using ::android::hardware::hidl_vec;
using ::android::hardware::Return;
using ::android::hardware::Void;
using ::android::sp;
using android::hidl::manager::V1_0::IServiceNotification;
using vendor::securethings::hardware::ids::ids_client::V1_0::IIdsClient;
using vendor::securethings::hardware::ids::ids_manager::V1_0::IIdsManager;
using vendor::securethings::hardware::ids::ids_client::V1_0::IDSReturnValue;
// using VHAL::sIdsFramework_Types;
// //IDS_END

class CAPIVehicleClientGateway{

    public:
        CAPIVehicleClientGateway();
        ~CAPIVehicleClientGateway();
   	void registerSpeedCallback(std::function<void(uint32_t)> callback); 
	void registerVinNumberCallback(std::function<void(std::vector<uint32_t>)> vinNumber);
    void registerIgnitionStsCallback(std::function<void(uint32_t)> IgnitionSts); 
	void registerEcallCrashCallback(std::function<void(uint32_t)> EcallCrash); 
	void registerParkLampCallback(std::function<void(uint32_t)> ParkLamp); 
	void registerDayNightModeCallback(std::function<void(uint32_t)> DayNightMode); 
	void registerFuellevelCallback(std::function<void(uint32_t)> Fuellevel); 
	void registerFuelTypeCallback(std::function<void(uint32_t)> FuelType); 
	void registerParkBreakStatusCallback(std::function<void(uint32_t)> FuelType); 
	void registerOutTempCallback(std::function<void(uint32_t)> OutTemp);
	void registerHVBatterySOHDataCallback(std::function<void(uint32_t)> HVBatterySOHData);
	void registerHVBatteryPercentageCallback(std::function<void(uint32_t)> HVBatteryPercentage);
	void registerHVBatteryMinChargeCallback(std::function<void(uint32_t)> HVBatteryMinCharge);
	void registerHVBatteryActiveConnectorCallback(std::function<void(uint32_t)> HVBatteryActiveConnector);
	void registerHVBatteryMaxRangeCallback(std::function<void(uint32_t)> HVBatteryMaxRange);
	void registerHVBatteryChargeLevelCallback(std::function<void(uint32_t)> HVBatteryChargeLevel);
	void registerHVBatteryIsChargingCallback(std::function<void(uint32_t)> HVBatteryIsCharging);
	void registerAutonomyCallback(std::function<void(uint32_t)> Autonomy);
	void registerBrakePedalrStsCallback(std::function<void(uint32_t)> BrakePedalrSts);
	void registerCanDistanceUnitCallback(std::function<void(uint32_t)> CanDistanceUnit);
	void registerInstantFuelConsCallback(std::function<void(uint32_t)> InstantFuelCons);
	void registerReverseGearEngagedConsCallback(std::function<void(uint32_t)> ReverseGearEngaged);
	void registerRoadSlopeCallback(std::function<void(uint32_t)> RoadSlope);
	void registerSteeringAngleCallback(std::function<void(uint32_t)> SteeringAngle);
	void registerTotalKMCallback(std::function<void(uint32_t)> TotalKM);
	void registerTransmGearDisplayCallback(std::function<void(uint32_t)> TransmGearDisplay);
	void registerTripAvgBattPowerConsCallback(std::function<void(uint32_t)> TripAvgBattPowerCons);
	void registerYawSpeedCallback(std::function<void(uint32_t)> YawSpeed);
	void registerGrossYawRateCallback(std::function<void(uint32_t)> GrossYawRate);
	void registerFLWheelTickCounterFaultCallback(std::function<void(uint32_t)> FLWheelTickCounterFault);
	void registerFRWheelTickCounterFaultCallback(std::function<void(uint32_t)> FRWheelTickCounterFault);
	void registerRLWheelTickCounterFaultCallback(std::function<void(uint32_t)> RLWheelTickCounterFault);
	void registerRRWheelTickCounterFaultCallback(std::function<void(uint32_t)> RRWheelTickCounterFault);
	void registerPulseCountFLWheelCallback(std::function<void(uint32_t)> PulseCountFLWheel);
    void registerPulseCountFRWheelCallback(std::function<void(uint32_t)> PulseCountFRWheel);
	void registerPulseCountRLWheelCallback(std::function<void(uint32_t)> PulseCountRLWheel);
	void registerPulseCountRRWheelCallback(std::function<void(uint32_t)> PulseCountRRWheel);
	void registerFPASStsCallback(std::function<void(uint32_t)> FPASSts);
	void registerDrivingDirectionCallback(std::function<void(uint32_t)> DrivingDirection);
	void registerCapacityUnitCallback(std::function<void(uint32_t)> CapacityUnit);
	void registerCombustFuelLvCallback(std::function<void(uint32_t)> CombustFuelLv);
	void registerCompressorStsCallback(std::function<void(uint32_t)> CompressorSts);
	void registerElectricMotorPowerCallback(std::function<void(int32_t)> ElectricMotorPower);
	void registerEngineCoolantTempCallback(std::function<void(uint32_t)> EngineCoolantTemp);
	void registerEngineRPMCallback(std::function<void(uint32_t)> EngineRPM);
	void registerFuelLevelMinimumStsCallback(std::function<void(uint32_t)> FuelLevelMinimumSts);
	void registerAbsSteeringWheelAngleAcuCallback(std::function<void(uint32_t)> AbsSteeringWheelAngleAcu);
	void registerInitSteeringWheenAngleFlagCallback(std::function<void(uint32_t)> InitSteeringWheenAngleFlag);
	void registerNetworkMNGTCallback(std::function<void(uint32_t)> NetworkMNGT);
	void registerFPASCenterLeftBarStsCallback(std::function<void(uint32_t)> FPASCenterLeftBarSts);
	void registerFPASCenterRightBarStsCallback(std::function<void(uint32_t)> FPASCenterRightBarSts);
    void registerFPASLeftBarStsCallback(std::function<void(uint32_t)> FPASLeftBarSts);
	void registerFPASRightBarStsCallback(std::function<void(uint32_t)> FPASRightBarSts);
	void registerRPASCenterLeftBarStsCallback(std::function<void(uint32_t)> RPASCenterLeftBarSts);
	void registerRPASCenterRightBarStsCallback(std::function<void(uint32_t)> RPASCenterRightBarSts);
	void registerRPASLeftBarStsCallback(std::function<void(uint32_t)> RPASLeftBarSts);
	void registerTrailerPresentCallback(std::function<void(uint32_t)> TrailerPresent);
	void registerTrunkDoorStsCallback(std::function<void(uint32_t)> TrunkDoorSts);
	void registerRPASStsCallback(std::function<void(uint32_t)> RPASSts);
	void registerEconShftRqCallback(std::function<void(uint32_t)> EconShftRq);
	void registerEngineTorqueCallback(std::function<void(uint32_t)> EngineTorque);
	void registerFuelConsUnitCallback(std::function<void(uint32_t)> FuelConsUnit);
	void registerRegeneratedEnergyCallback(std::function<void(uint32_t)> RegeneratedEnergy);
	void registerShiftLeverPositionCallback(std::function<void(uint32_t)> ShiftLeverPosition);
	void registerTripAverageFuelConsumptionCallback(std::function<void(uint32_t)> TripAverageFuelConsumption);
	void registerReqVolCtrlCallback(std::function<void(uint32_t)> ReqVolCtrl);
	void registerVehPrivacyModeCallback(std::function<void(uint32_t)> VehPrivacyMode);

	void registerBCMSECURtimeCallback(std::function<void(uint32_t [])> bcmSECURtime);
	void registerIMUdata_callback(std::function<void(std::vector<int32_t>)> imuDataParam);
	void registerDotteParamCallback(std::function<void(std::vector<uint32_t>)> dotteParam);
	void registerBCMHMItimeCallback(std::function<void(uint32_t [])> bcmHMItime);
	void registerInternalSignalCallback(std::function<void(uint32_t [])> internalSignal);
	void registerEVSignalsCallback(std::function<void(int32_t [])> EVSignal);
	void registerCanCommonSignalsCallback(std::function<void(int32_t [])> CanCommonSignal);
	void registerCameraSignalsCallback(std::function<void(uint32_t [])> CameraSignal);
	void registerBCMSignalsCallback(std::function<void(uint32_t [])> BCMSignal);
	void registerDTCParamCallback(std::function<void(std::vector<uint32_t>)> dtcParam);
	void registerDLTParamCallback(std::function<void(std::vector<uint32_t>)> dLTParam);
	void registerDLT_VectorParamCallback(std::function<void(std::vector<uint8_t>)> dLT_VectorValueParam);
	void registerCalibrationDataCallback(std::function<void(std::vector<uint32_t>)> calDATA);
	void registerTemperatureCallback(std::function<void(uint32_t)> Temperature);
	void registerSupplyVoltageCallback(std::function<void(uint32_t)> SupplyVoltage);
	void registerUSB_UpdateCallback(std::function<void(std::vector<uint16_t>)> USB_Update);
	void registerCalibrationDataCallback(std::function<void(uint32_t [])> calDATA);
	void registerCalibrationDataCallback_Audio(std::function<void(uint32_t [])> calDATA);
	void registerCalibrationDataCallback_Camera(std::function<void(uint32_t [])> calDATA);
	void registerCalibrationDataCallback_Array(std::function<void(std::vector<uint8_t>)> calDATA);
	void registerDID_RID_callback(std::function<void(std::vector<uint16_t>)> m_dID_message_type);
	void registerCallActiveSTRDisplayPopupStatusCallback(std::function<void(uint32_t)> callActiveSTRDisplayPopupStatus);
	void registerSTRCancelShutdownStatusCallback(std::function<void(uint32_t)> STRCancelShutdownStatus);
    void registerPowermodeCallback(std::function<void(uint32_t)> powermode);
    void registerCAN_RX_SignalsCallback(std::function<void(uint32_t [])> CAN_RX_Signals);
    void registerDEM_DTCstatuscallback(std::function<void(std::vector<uint16_t>)> m_diag_wakeup_message_type);
	void register_DriverdoorStatuscallback(std::function<void(uint8_t)> driverdoorStatus);
	void register_PassengerDoorStatuscallback(std::function<void(uint8_t)> passengerDoorStatus);
	void registerSigned_CAN_RX_SignalsCallback(std::function<void(int32_t [])> Signed_CAN_RX_Signals);
	void registerLog_RetriverCallback(std::function<void(uint32_t)> Log_Retriver_data);
	void registerlogerfilepathCallback(std::function<void(std::string)> Loggerfilepath);
	void registerLog_RetriverStatusCallback(std::function<void(uint32_t)> Log_Retriver_status);
	void registerSOC_WakeupReasonCallback(std::function<void(uint32_t)> WakeupReason);
	void registerRXSignals_FloatCallback(std::function<void(float [])> RXSignals_Float);
	void registerMarelliPNCallback(std::function<void(std::vector<uint8_t>)> MarelliPN);
	void registerEOLHWIDCallback(std::function<void(std::vector<uint8_t>)> EOLhwid);
	void registerAuthenticationZoneCallback(std::function<void(std::vector<uint8_t>)> AuthenticationZone);
	void registerApplicationSoftwareIdentificationCallback(std::function<void(std::vector<uint8_t>)> ApplicationSoftwareIdentification);
	void registerApplicationDataIdentificationCallback(std::function<void(std::vector<uint8_t>)> ApplicationDataIdentification);
	void registerDataLibraryidentifierCallback(std::function<void(std::vector<uint8_t>)> DataLibraryidentifier);
	void registerVehicleAppsIdentifierCallback(std::function<void(std::vector<uint8_t>)> VehicleAppsIdentifier);
	void registerDCSD_DIAG_ReqCallback(std::function<void(std::vector<uint8_t>)> DCSD_DIAG_Req);
    void registercalibrationDataforSrecCallback(std::function<void(std::vector<uint32_t>)> calSrecDATA);
    void init();
	void getPowerMode();
	void getIMUsubscribe();
	void getDIDsubscribe();
	void getDottesubscribe();
	void getDTCsubscribe();
	void getDLTsubscribe();
	void getVinNumbersubscribe();
	void getInternalSigsubscribe();
	void getInternalSignal(std::vector<uint32_t>& p1);
	void getCalibrationDatasubscribe();
	void getTemperaturesubscribe();
	void getsupplyvgesubscribe();
	void getUSB_Updatesubscribe();
	void getUSB_Update_detailssubscribe();
	void getCallActiveSTRDisplayPopupStatusSubscribe();
	void getSTRCancelShutdownSubscribe();
	void getDEM_DTCStatussubscribe();
	void getCANData();
	void getLoggerdatasubscribe();
	void getSOC_Wakeup_reasonsubscribe();
	void getMarelliPartNumbersubscribe();
	void getEOL_hw_variantsubscribe();
	void getApplicationSoftwareIdentification();
	void setIVIPhoneCallStatus(uint16_t& p1);
	void setIVISmsStatus(uint16_t& p1);
	void setIVI_LogExtractionTriggerStatus(uint16_t& p1);
	void setIVI_DLTExtractionTriggerStatus(uint16_t& p1);
	void setIVI_CalibrationTriggerStatus(uint16_t& p1);
    void setIVI_S2RStatus(uint8_t& str_status);
	void onPowerModeAttributeChanged(uint32_t newValue);
	void setIVI_DIDdata_request(std::vector<uint16_t>& p1);
	void setIVIHMISoftKeyStatus(std::vector<uint16_t>& p1);
	void setIVIHMI_iviData(std::vector<uint16_t>& p1);
	void setIVI_USB_Update(std::vector<uint16_t>& p1);
	void setIVI_Usb_Reset_TriggerStatus(uint16_t& p1);
    void setSOC_Ready_Status();
	void setIVI_CallActiveSTR_HMIPopup_Status(uint8_t& userInputFromHMIPopup);
	void setDEM_DTCStatus(std::vector<uint8_t>& p1);
	void setIVI_TX_Signals(uint16_t IVI_TX_Signals , uint16_t tx_enum_number);
	void SetInternal_Fan_Speed(uint32_t& Fan_speed);
	void SetLoggerdata(std::vector<uint32_t>& p1);
	void SendFilepathtoTCU(std::string& filepath);
	void getlogretrieverstatus(uint32_t& lagretriever_status);
	void setIVIResettrigger(uint32_t& IVI_Reset);
	void setIVI_SoCVersion_Send_MCU();
	void setIVI_hw_variant_id_Send_MCU();
	std::string getValueFromProcCmdline(const std::string& key);
	void SetSystemTimeZone(int32_t& TimeZone);	
	void setDCSD_DIAG_Res(std::vector<uint8_t>& DCSD_DIAG_Res);
    void send_arvc_status_mcu(uint8_t& arvc_status);
struct dID_message_type{
	uint16_t dID_MsgID;
	uint16_t dID_AppID;
	uint16_t dID_Handle;
	uint16_t dID_length;
	std::vector<uint8_t> dID_DiagMessage;
}m_dID_message_type;

enum dlt_signal{
    NTW_TX_DLT_handbrakeStatus = 0,
    NTW_TX_DLT_headlightsStatus,
	NTW_TX_DLT_steeringWheelCtrlChange,
	NTW_TX_DLT_iviTempSensorOutputs,
	NTW_TX_DLT_systemInterfaceFailures,
	NTW_TX_DLT_loudspeakerStatus,
	NTW_TX_DLT_microphoneStatus,
	NTW_TX_DLT_ecuGyroAcceleroCompassSt,
	NTW_TX_DLT_activeEcuDTCs,
	NTW_TX_DLT_bubChargePercentage,
	NTW_TX_DLT_bubOverITempChargeDischargeEvent,
	NTW_TX_DLT_bubDisconnectionEvent,
	NTW_TX_DLT_mcpuVcpuSwVersion,
	NTW_TX_DLT_ecuSerialNumber,
	NTW_TX_DLT_ecuHwVersionNumber,
	NTW_TX_DLT_ecuHwPartNumber,
	NTW_TX_DLT_ecuHwNumbers,
	NTW_TX_DLT_ecuSwVersionNumber,
	NTW_TX_DLT_ecuSwPartNumber,
	NTW_TX_DLT_ecuSwNumbers,
	NTW_TX_DLT_ecuSparePartNumber,
	NTW_TX_DLT_Number,
} NTW_TX_DLT_Key;

enum VENDOR_USB_SOFTWARE_UPDATE{
VENDOR_USB_VENDOR_PACKAGE = 0,
VENDOR_DISK_SPACE ,
VENDOR_INVALID_SOFTWARE ,
VENDOR_UPDATE_COUNTDOWN_TIMER ,
VENDOR_ABORT_USB_SOFTWARE_UPDATE ,
VENDOR_DOWNGRADE_SOFTWARE ,
VENDOR_USB_UPDATE_PROGRESS  ,
VENDOR_USB_UPDATE_REMAINING_TIME ,
VENDOR_USB_UPDATE_FAILURE ,
VENDOR_AUTOMATIC_RESTART,
VENDOR_USERCONSENT_NEWUPDATE,
VENDOR_CONTINUE_UPDATE,
USB_Update_Number,
} USB_Update_Key;

enum dtc_signal{
    NTW_TX_DTC_0x900311 = 0,
    NTW_TX_DTC_0x900312,
    NTW_TX_DTC_0x900313,
    NTW_TX_DTC_0x900371,
    NTW_TX_DTC_0x900411,
    NTW_TX_DTC_0x900412,
    NTW_TX_DTC_0x900413,
    NTW_TX_DTC_0x900471,
    NTW_TX_DTC_0x9E0116,
    NTW_TX_DTC_0x9E0117,
    NTW_TX_DTC_0xAE0551,
    NTW_TX_DTC_0xAE0E51,
    NTW_TX_DTC_0xAE0E62,
    NTW_TX_DTC_0xC03013,
    NTW_TX_DTC_0xC03111,
    NTW_TX_DTC_0xC03212,
    NTW_TX_DTC_0xC03313,
    NTW_TX_DTC_0xC03411,
    NTW_TX_DTC_0xC03512,
    NTW_TX_DTC_0xC0362B,
    NTW_TX_DTC_0xC04013,
    NTW_TX_DTC_0xC04111,
    NTW_TX_DTC_0xC04212,
    NTW_TX_DTC_0xC04313,
    NTW_TX_DTC_0xC04411,
    NTW_TX_DTC_0xC04512,
    NTW_TX_DTC_0xC1400,
    NTW_TX_DTC_0xC4220,
    NTW_TX_DTC_0xD9017C,
    NTW_TX_DTC_0xD90187,
    NTW_TX_DTC_0xD90188,
    NTW_TX_DTC_0xD90287,
    NTW_TX_DTC_0xD90387,
    NTW_TX_DTC_NUMBER, 
} NTW_TX_DTC_Key;

enum dID_MsgID{
	
DIDSERVICES_0xD409_READDATA,
DIDSERVICES_0xD446_READDATA,
DIDSERVICES_0xD46E_READDATA,
DIDSERVICES_0x165F_READDATA,
DIDSERVICES_0x1660_READDATA,
DIDSERVICES_0x1661_READDATA,
DIDSERVICES_0x1662_READDATA,
DIDSERVICES_0x1663_READDATA,
DIDSERVICES_0x2206_READDATA,
DIDSERVICES_0x1646_READDATA,
DIDSERVICES_0xD44B_READDATA,
DIDSERVICES_0xD44D_READDATA,
DIDSERVICES_0xD482_READDATA,
DIDSERVICES_0xD483_READDATA,
DIDSERVICES_0xD484_READDATA,
DIDSERVICES_0xD486_READDATA,
DIDSERVICES_0xD4E1_READDATA,
DIDSERVICES_0xD4E2_READDATA,
DIDSERVICES_0x2228_READDATA,
DIDSERVICES_0xD450_READDATA,
DIDSERVICES_0xD473_READDATA,
DIDSERVICES_0xD474_READDATA,
DIDSERVICES_0xD475_READDATA,
DIDSERVICES_0xD476_READDATA,
DIDSERVICES_0xF020_READDATA,
DIDSERVICES_0xF021_READDATA,
DIDSERVICES_0xF022_READDATA,
DIDSERVICES_0x164D_READDATA,
DIDSERVICES_0x164E_READDATA,
DIDSERVICES_0x164F_READDATA,
DIDSERVICES_0x1650_READDATA,
DIDSERVICES_0xDADE_READDATA,
DIDSERVICES_0xF028_READDATA,
DIDSERVICES_0x16AB_READDATA,
DIDSERVICES_0xD48E_READDATA,
DIDSERVICES_0xD48F_READDATA,
DIDSERVICES_0xD490_READDATA,
DIDSERVICES_0xD492_READDATA,
DIDSERVICES_0xD491_READDATA,
DIDSERVICES_0xD49B_READDATA,
DIDSERVICES_0xD4A6_READDATA,
DIDSERVICES_0xD4A7_READDATA,
DIDSERVICES_0xD4A8_READDATA,
DIDSERVICES_0xD4A9_READDATA,
DIDSERVICES_0x2224_READDATA,
DIDSERVICES_0xD437_READDATA,
DIDSERVICES_0xD4AA_READDATA,
DIDSERVICES_0xD4B5_READDATA,
DIDSERVICES_0xD4B9_READDATA,
DIDSERVICES_0xD4BF_READDATA,
DIDSERVICES_0xD44C_READDATA,
DIDSERVICES_0x1628_READDATA,
DIDSERVICES_0x1623_READDATA,
DIDSERVICES_0x1629_READDATA,
DIDSERVICES_0xD4D7_READDATA,
DIDSERVICES_0x160B_READDATA,
DIDSERVICES_0x1669_READDATA,
DIDSERVICES_0x166A_READDATA,
DIDSERVICES_0x166B_READDATA,
DIDSERVICES_0x166C_READDATA,
DIDSERVICES_0x166D_READDATA,
DIDSERVICES_0x166E_READDATA,
DIDSERVICES_0x166F_READDATA,
DIDSERVICES_0x1670_READDATA,
DIDSERVICES_0x1671_READDATA,
DIDSERVICES_0xD455_READDATA,
DIDSERVICES_0xD456_READDATA,
DIDSERVICES_0xD458_READDATA,
DIDSERVICES_0xD459_READDATA,
DIDSERVICES_0xD45F_READDATA,
DIDSERVICES_0xD460_READDATA,
DIDSERVICES_0xD461_READDATA,
DIDSERVICES_0xD464_READDATA,
/*******************write******************************/
DIDSERVICES_0xD446_WRITEDATA,
DIDSERVICES_0x165F_WRITEDATA,
DIDSERVICES_0x1660_WRITEDATA,
DIDSERVICES_0x1661_WRITEDATA,
DIDSERVICES_0x1662_WRITEDATA,
DIDSERVICES_0x1663_WRITEDATA,
DIDSERVICES_0x2206_WRITEDATA,
DIDSERVICES_0x1646_WRITEDATA,
DIDSERVICES_0xF020_WRITEDATA,
DIDSERVICES_0x164D_WRITEDATA,
DIDSERVICES_0xD4A9_WRITEDATA,
DIDSERVICES_0x2224_WRITEDATA,
DIDSERVICES_0xD4B9_WRITEDATA,
DIDSERVICES_0xD4BF_WRITEDATA,
DIDSERVICES_0x1628_WRITEDATA,
DIDSERVICES_0x1623_WRITEDATA,
DIDSERVICES_0x1629_WRITEDATA,
DIDSERVICES_0x160B_WRITEDATA,
DIDSERVICES_0x1669_WRITEDATA,
DIDSERVICES_0x166A_WRITEDATA,
DIDSERVICES_0x166B_WRITEDATA,
DIDSERVICES_0x166C_WRITEDATA,
DIDSERVICES_0x166D_WRITEDATA,
DIDSERVICES_0x166E_WRITEDATA,
DIDSERVICES_0x166F_WRITEDATA,
DIDSERVICES_0x1670_WRITEDATA,
DIDSERVICES_0x1671_WRITEDATA,
DIDSERVICES_0xD456_WRITEDATA,
DIDSERVICES_0xD460_WRITEDATA,
/***************Routine*********************************/
ROUTINESERVICES_0xDD55_START,               
ROUTINESERVICES_0xDD55_STOP,            
ROUTINESERVICES_0xDD55_REQUESTRESULTS, 
ROUTINESERVICES_0xDD92_START,
ROUTINESERVICES_0xDD92_STOP,
ROUTINESERVICES_0xDD92_REQUESTRESULTS,
ROUTINESERVICES_0x020A_START,
ROUTINESERVICES_0x020A_STOP,
ROUTINESERVICES_0x020A_REQUESTRESULTS,
ROUTINESERVICES_0x020B_START,
ROUTINESERVICES_0x020B_STOP,
ROUTINESERVICES_0x020B_REQUESTRESULTS,
ROUTINESERVICES_0x020C_START,
ROUTINESERVICES_0x020C_STOP,
ROUTINESERVICES_0x020C_REQUESTRESULTS,
ROUTINESERVICES_0x020D_START,
ROUTINESERVICES_0x020D_STOP,
ROUTINESERVICES_0x020D_REQUESTRESULTS,
ROUTINESERVICES_0x020E_START,
ROUTINESERVICES_0x020E_STOP,
ROUTINESERVICES_0x020E_REQUESTRESULTS,
ROUTINESERVICES_0x0401_START,
ROUTINESERVICES_0x0401_STOP,
ROUTINESERVICES_0x0401_REQUESTRESULTS,
ROUTINESERVICES_0x0641_START,
ROUTINESERVICES_0x0641_STOP,
ROUTINESERVICES_0x0641_REQUESTRESULTS,
ROUTINESERVICES_0x0301_START,
ROUTINESERVICES_0x0301_STOP,
ROUTINESERVICES_0x0301_REQUESTRESULTS,
ROUTINESERVICES_0xDD82_START,
ROUTINESERVICES_0xDD82_STOP,
ROUTINESERVICES_0xDD82_REQUESTRESULTS,
ROUTINESERVICES_0x0632_START,
ROUTINESERVICES_0x0632_STOP,
ROUTINESERVICES_0x0632_REQUESTRESULTS,
ROUTINESERVICES_0x0650_START,
ROUTINESERVICES_0x0650_STOP,
ROUTINESERVICES_0x0650_REQUESTRESULTS,
ROUTINESERVICES_0xDD60_START,
ROUTINESERVICES_0xDD60_STOP,
ROUTINESERVICES_0xDD60_REQUESTRESULTS,
ROUTINESERVICES_0x0200_START,
ROUTINESERVICES_0x0200_STOP,
ROUTINESERVICES_0x0200_REQUESTRESULTS,
ROUTINESERVICES_0xDD5A_START,
ROUTINESERVICES_0xDD5A_STOP,
ROUTINESERVICES_0xDD5A_REQUESTRESULTS,
ROUTINESERVICES_0xDD5B_START,
ROUTINESERVICES_0xDD5B_STOP,
ROUTINESERVICES_0xDD5B_REQUESTRESULTS,
ROUTINESERVICES_0xDD73_START,
ROUTINESERVICES_0xDD73_STOP,
ROUTINESERVICES_0xDD73_REQUESTRESULTS,
ROUTINESERVICES_0xDD5D_START,
ROUTINESERVICES_0xDD5D_STOP,
ROUTINESERVICES_0xDD5D_REQUESTRESULTS,
ROUTINESERVICES_0xDD5E_START,
ROUTINESERVICES_0xDD5E_STOP,
ROUTINESERVICES_0xDD5E_REQUESTRESULTS,
ROUTINESERVICES_0xDD5F_START,
ROUTINESERVICES_0xDD5F_STOP,
ROUTINESERVICES_0xDD5F_REQUESTRESULTS,
ROUTINESERVICES_0xDD56_START,
ROUTINESERVICES_0xDD56_STOP,
ROUTINESERVICES_0xDD56_REQUESTRESULTS,
ROUTINESERVICES_0xDD57_START,
ROUTINESERVICES_0xDD57_STOP,
ROUTINESERVICES_0xDD57_REQUESTRESULTS,
ROUTINESERVICES_0xDD58_START,
ROUTINESERVICES_0xDD58_STOP,
ROUTINESERVICES_0xDD58_REQUESTRESULTS,
ROUTINESERVICES_0xDD59_START,
ROUTINESERVICES_0xDD59_STOP,
ROUTINESERVICES_0xDD59_REQUESTRESULTS,
LAST_SERVICE_IDID,                                    

                  
};

enum rawCanSinal{
	NTW_RX_SIGNAL_PHASE_VIE = 0,
    NTW_RX_SIGNAL_KEY_POS,
    NTW_RX_SIGNAL_ETAT_JOUR_NUIT,
    NTW_RX_SIGNAL_BCCM_FUEL_TYPE,
    NTW_RX_SIGNAL_MINC,
    NTW_RX_SIGNAL_VITV,
    NTW_RX_SIGNAL_CONTACT_FREIN_PRK,
    NTW_RX_SIGNAL_BMS_TOTALBATTENERGY,
    NTW_RX_SIGNAL_HV_BATT_SOH_CAPA_HD,
    NTW_RX_SIGNAL_HV_BATT_SOC,
    NTW_RX_SIGNAL_HV_BATT_ALERT_LOW_SOC,
    NTW_RX_SIGNAL_MODE_CHARGE_IN_PROGRESS,
    NTW_RX_SIGNAL_OBC_PLUG_STATE_rx,
    NTW_RX_SIGNAL_AUTONOMIE_ZEV,
    NTW_RX_SIGNAL_T_EXT,
    NTW_RX_SIGNAL_VIS_1,
    NTW_RX_SIGNAL_VIS_2,
    NTW_RX_SIGNAL_VIS_3,
    NTW_RX_SIGNAL_VIS_4,
    NTW_RX_SIGNAL_VIS_5,
    NTW_RX_SIGNAL_VIS_6,
    NTW_RX_SIGNAL_VIS_7,
    NTW_RX_SIGNAL_VIS_8,
    NTW_RX_SIGNAL_WMI_1,
    NTW_RX_SIGNAL_WMI_2,
    NTW_RX_SIGNAL_WMI_3,
    NTW_RX_SIGNAL_VDS_1,
    NTW_RX_SIGNAL_VDS_2,
    NTW_RX_SIGNAL_VDS_3,
    NTW_RX_SIGNAL_VDS_4,
    NTW_RX_SIGNAL_VDS_5,
    NTW_RX_SIGNAL_VDS_6,
    NTW_RX_SIGNAL_ETAT_PRINCIP_SEV,
    NTW_RX_SIGNAL_ECALL_CRASH,
    NTW_RX_SIGNAL_BOUC_CEINT_PASS,
    NTW_RX_SIGNAL_BOUC_CEINT_AVMIL,
    NTW_RX_SIGNAL_BOUC_CEINT_AR2MIL,
    NTW_RX_SIGNAL_BOUC_CEINT_AR2D,
    NTW_RX_SIGNAL_BOUC_CEINT_AR2G,
    NTW_RX_SIGNAL_BOUC_CEINT_AR3G,
    NTW_RX_SIGNAL_BOUC_CEINT_AR3D,
    NTW_RX_SIGNAL_BOUC_CEINT_COND,
    NTW_RX_SIGNAL_FRONT_F_VIT,
    NTW_RX_SIGNAL_FRONT_G_VIT,
    NTW_RX_SIGNAL_FRONT_M_VIT_1,
    NTW_RX_SIGNAL_FRONT_M_VIT_2,
    NTW_RX_SIGNAL_FRONT_REPAR,
    NTW_RX_SIGNAL_ARR_F_VIT,
    NTW_RX_SIGNAL_ARR_G_VIT,
    NTW_RX_SIGNAL_ARR_M_VIT,
    NTW_RX_SIGNAL_ARR_REPAR,
    NTW_RX_SIGNAL_LAT_F_VIT,
    NTW_RX_SIGNAL_LAT_G_VIT,
    NTW_RX_SIGNAL_LAT_M_VIT,
    NTW_RX_SIGNAL_LAT_REPAR,
    NTW_RX_SIGNAL_T_HUILE,
    NTW_RX_SIGNAL_NIV_CRB,
    NTW_RX_SIGNAL_AUTONOMIE,
    NTW_RX_SIGNAL_KM_TOTAL,
    NTW_RX_SIGNAL_K_MAINT,
    NTW_RX_SIGNAL_NB_JOUR_MAINTENANCE,
    NTW_RX_SIGNAL_SIGN_MAINT,
    NTW_RX_SIGNAL_SIGN_ECHEANCE,
    NTW_RX_SIGNAL_ETAT_GMP,
    NTW_RX_SIGNAL_ACCES_JDA,
    NTW_RX_SIGNAL_ALERTES,
    NTW_RX_SIGNAL_NB_BLOC,
    NTW_RX_SIGNAL_NUMERO_BLOC,
    NTW_RX_SIGNAL_INFO_CRASH,
    NTW_RX_SIGNAL_PIETON,
    NTW_RX_SIGNAL_FEUX_POS,
    NTW_RX_SIGNAL_MODE_HEURE_CLIENT,
    NTW_RX_SIGNAL_HEURE_HORLOGE,
    NTW_RX_SIGNAL_MINUTE_HORLOGE,
    NTW_RX_SIGNAL_JOUR_HORLOGE,
    NTW_RX_SIGNAL_MOIS_HORLOGE,
    NTW_RX_SIGNAL_ANNEE_HORLOGE,
    NTW_RX_SIGNAL_TISTAMPSECURECLKDAY,
    NTW_RX_SIGNAL_TISTAMPSECURECLKHR,
    NTW_RX_SIGNAL_TISTAMPSECURECLKMINS,
    NTW_RX_SIGNAL_TISTAMPSECURECLKMTH,
    NTW_RX_SIGNAL_TISTAMPSECURECLKSEC1,
    NTW_RX_SIGNAL_TISTAMPSECURECLKYR,
    NTW_RX_SIGNAL_MODE_CONFIG_VHL,
    NTW_RX_SIGNAL_ETAT_CPO_COND,
    NTW_RX_SIGNAL_ETAT_CPO_PASS,
    NTW_RX_SIGNAL_PARC_USINE,
    NTW_RX_SIGNAL_PULS_CNT_WHEEL_FL,
    NTW_RX_SIGNAL_PULS_CNT_WHEEL_FR,
    NTW_RX_SIGNAL_PULS_CNT_WHEEL_RL,
    NTW_RX_SIGNAL_PULS_CNT_WHEEL_RR,
    NTW_RX_SIGNAL_POS_LEVIER_BV,
    
    NTW_RX_SIGNAL_CONSO_INSTANTANEE,
    NTW_RX_SIGNAL_POS_LEVIER_BV_TCU,
    NTW_RX_SIGNAL_UNITE_DISTANCE,
    NTW_RX_SIGNAL_MINI_CARB,
    NTW_RX_SIGNAL_UNITE_VOLUME,
    NTW_RX_SIGNAL_DISTRIBUTION_AVG,
    NTW_RX_SIGNAL_TYPE_DIR,
    NTW_RX_SIGNAL_SIDE_OF_LATERAL_CRASH,
    NTW_RX_SIGNAL_TEAU,
    NTW_RX_SIGNAL_FCT_PRIVACY_MODE,
    NTW_RX_SIGNAL_AFFICHAGE_HORLOGE,
    NTW_RX_SIGNAL_CONSO_INST_PCT_PWT_ELEC,
    NTW_RX_SIGNAL_CONSO_INST_PCT_CFT,
    NTW_RX_SIGNAL_CONSO_TRIP_ELEC_PWT,
    NTW_RX_SIGNAL_CONSO_TRIP_OTHER,
    NTW_RX_SIGNAL_CONSO_TRIP_ELEC_PWT_REGE,
    NTW_RX_SIGNAL_ETAT_MELAV,
    NTW_RX_SIGNAL_ETAT_MEL,
    NTW_RX_SIGNAL_SOC_BATT,
    NTW_RX_SIGNAL_BMS_RESTBATTENERGY,
    NTW_RX_SIGNAL_BMS_PROBETEMP1,
    NTW_RX_SIGNAL_BMS_PROBETEMP2,
    NTW_RX_SIGNAL_BMS_PROBETEMP3,
    NTW_RX_SIGNAL_BMS_PROBETEMP4,
    NTW_RX_SIGNAL_BMS_PROBETEMP5,
    NTW_RX_SIGNAL_BMS_PROBETEMP6,
    NTW_RX_SIGNAL_BMS_PROBETEMP7,
    NTW_RX_SIGNAL_BMS_PROBETEMP8,
    NTW_RX_SIGNAL_BMS_PROBETEMP9,
    NTW_RX_SIGNAL_BMS_PROBETEMP10,
    NTW_RX_SIGNAL_BMS_PROBETEMP11,
    NTW_RX_SIGNAL_BMS_PROBETEMP12,
    NTW_RX_SIGNAL_BMS_PROBETEMP13,
    NTW_RX_SIGNAL_BMS_PROBETEMP14,
    NTW_RX_SIGNAL_BMS_PROBETEMP15,
    NTW_RX_SIGNAL_BMS_PROBETEMP16,
    NTW_RX_SIGNAL_BMS_PROBETEMP17,
    NTW_RX_SIGNAL_BMS_PROBETEMP18,
    NTW_RX_SIGNAL_BMS_PROBETEMP19,
    NTW_RX_SIGNAL_BMS_PROBETEMP20,
    NTW_RX_SIGNAL_BMS_PROBETEMP21,
    NTW_RX_SIGNAL_BMS_PROBETEMP22,
    NTW_RX_SIGNAL_BMS_PROBETEMP23,
    NTW_RX_SIGNAL_BMS_PROBETEMP24,
    NTW_RX_SIGNAL_BMS_PROBETEMP25,
    NTW_RX_SIGNAL_BMS_PROBETEMP26,
    NTW_RX_SIGNAL_BMS_PROBETEMP27,
    NTW_RX_SIGNAL_BMS_PROBETEMP28,
    NTW_RX_SIGNAL_BMS_PROBETEMP29,
    NTW_RX_SIGNAL_BMS_PROBETEMP30,
    NTW_RX_SIGNAL_BMS_PROBETEMP31,
    NTW_RX_SIGNAL_BMS_PROBETEMP32,
    NTW_RX_SIGNAL_BMS_PROBETEMP33,
    NTW_RX_SIGNAL_BMS_PROBETEMP34,
    NTW_RX_SIGNAL_BMS_PROBETEMP35,
    NTW_RX_SIGNAL_BMS_PROBETEMP36,
    NTW_RX_SIGNAL_BMS_PROBETEMP37,
    NTW_RX_SIGNAL_BMS_PROBETEMP38,
    NTW_RX_SIGNAL_BMS_PROBETEMP39,
    NTW_RX_SIGNAL_BMS_PROBETEMP40,
    NTW_RX_SIGNAL_BMS_PROBETEMP41,
    NTW_RX_SIGNAL_BMS_PROBETEMP42,
    NTW_RX_SIGNAL_BMS_PROBETEMP43,
    NTW_RX_SIGNAL_BMS_PROBETEMP44,
    NTW_RX_SIGNAL_BMS_PROBETEMP45,
    NTW_RX_SIGNAL_BMS_PROBETEMP46,
    NTW_RX_SIGNAL_BMS_PROBETEMP47,
    NTW_RX_SIGNAL_BMS_PROBETEMP48,
    NTW_RX_SIGNAL_BMS_PROBETEMP49,
    NTW_RX_SIGNAL_BMS_PROBETEMP50,
    NTW_RX_SIGNAL_BMS_PROBETEMP51,
    NTW_RX_SIGNAL_BMS_PROBETEMP52,
    NTW_RX_SIGNAL_BMS_PROBETEMP53,
    NTW_RX_SIGNAL_BMS_PROBETEMP54,
    NTW_RX_SIGNAL_HV_BATT_TEMP_AVG,
    NTW_RX_SIGNAL_BMS_REMAININGBATTCAPACITY,
    NTW_RX_SIGNAL_POS_LEVIER_BV_AFFICH,
    NTW_RX_SIGNAL_ACCEL_LONGI_ROUES,
    NTW_RX_SIGNAL_EFCMNT_PDLE_ACCEL,
    NTW_RX_SIGNAL_VITESSE_VEHICULE_ROUES,
    NTW_RX_SIGNAL_FAULT_PULS_CNT_FL,
    NTW_RX_SIGNAL_FAULT_PULS_CNT_FR,
    NTW_RX_SIGNAL_FAULT_PULS_CNT_RL,
    NTW_RX_SIGNAL_FAULT_PULS_CNT_RR,
    NTW_RX_SIGNAL_DISTANCE_ROUES,
    NTW_RX_SIGNAL_ACCEL_LONGI_CALIB,
    NTW_RX_SIGNAL_ACCEL_LAT,
    NTW_RX_SIGNAL_VITESSE_LACET_BRUTE,
    NTW_RX_SIGNAL_INFO_CMDM_MODE_VHL,
    NTW_RX_SIGNAL_CLIGNO_G,
    NTW_RX_SIGNAL_CLIGNO_D,
    NTW_RX_SIGNAL_ETAT_CLIGNOTANTS,
    NTW_RX_SIGNAL_ANGLE_VOLANT,
    NTW_RX_SIGNAL_CPT_TEMPOREL,
    NTW_RX_SIGNAL_PRESENCE_RQUE,
    NTW_RX_SIGNAL_DRIVER_DETECT_CHANGE,
    NTW_RX_SIGNAL_ETAT_IND_ROUE_ARD,
    NTW_RX_SIGNAL_DIST_PARCOURUE_TRAJET,
    NTW_RX_SIGNAL_CONSO_MOY_ZEV_TRAJ,
    NTW_RX_SIGNAL_CONSO_MOY_ZEV_1,

    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMF_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_NEW_JDD_OBC_DCDC_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_VCU_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_MCU_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_UC_FREIN_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_TBMU_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_EMIS_NEW_JDD_AVAS_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMM_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_BV_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_NEW_JDD_DIRA_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_LVNSD_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_ESM_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_EPLU_ETAT_DTC_Mx,

    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMF_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_NEW_JDD_OBC_DCDC_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_VCU_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_MCU_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_UC_FREIN_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_TBMU_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_EMIS_NEW_JDD_AVAS_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMM_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_BV_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_NEW_JDD_DIRA_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_LVNSD_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_ESM_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_EPLU_KILOMETRAGE_JDD_Mx02,

    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMF_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_NEW_JDD_OBC_DCDC_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_VCU_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_MCU_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_UC_FREIN_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_TBMU_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_EMIS_NEW_JDD_AVAS_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMM_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_BV_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_NEW_JDD_DIRA_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_LVNSD_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_ESM_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_EPLU_NOMBRE_TRAMES_Mx,

    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMF_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_NEW_JDD_OBC_DCDC_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_VCU_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_MCU_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_UC_FREIN_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_TBMU_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_EMIS_NEW_JDD_AVAS_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMM_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_BV_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_NEW_JDD_DIRA_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_LVNSD_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_ESM_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_EPLU_NUMERO_TRAME_Mx01,

    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMF_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_NEW_JDD_OBC_DCDC_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_VCU_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_MCU_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_UC_FREIN_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_TBMU_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_EMIS_NEW_JDD_AVAS_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMM_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_BV_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_NEW_JDD_DIRA_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_LVNSD_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_ESM_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_EPLU_NUMERO_TRAME_Mx02,

    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMF_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_NEW_JDD_OBC_DCDC_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_VCU_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_MCU_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_UC_FREIN_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_TBMU_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_EMIS_NEW_JDD_AVAS_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMM_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_BV_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_NEW_JDD_DIRA_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_LVNSD_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_ESM_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_EPLU_SITUATION_VIE_JDD_Mx02,

    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMF_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_NEW_JDD_OBC_DCDC_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_VCU_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_MCU_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_UC_FREIN_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_TBMU_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_EMIS_NEW_JDD_AVAS_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMM_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_BV_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_NEW_JDD_DIRA_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_LVNSD_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_ESM_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_EPLU_TYPE_MESSAGE_Mx,

    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMF_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_NEW_JDD_OBC_DCDC_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_VCU_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_MCU_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_UC_FREIN_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_TBMU_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_EMIS_NEW_JDD_AVAS_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMM_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_BV_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_NEW_JDD_DIRA_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_LVNSD_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_ESM_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_EPLU_REFERENCE_HORAIRE_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_PEPS_REFERENCE_HORAIRE_Mx01,
        
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMF_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_NEW_JDD_OBC_DCDC_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_VCU_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_MCU_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_UC_FREIN_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_TBMU_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_EMIS_NEW_JDD_AVAS_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_CMM_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_BV_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_NEW_JDD_DIRA_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_LVNSD_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_ESM_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_EPLU_CODES_DEFAUT_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_PEPS_CODES_DEFAUT_Mx01,

    NTW_RX_SIGNAL_VCU_CTRL_GEAR_POS_INFO,
    NTW_RX_SIGNAL_REQ_VOL_CTRL,
    NTW_RX_SIGNAL_ETAT_CONDA,
    NTW_RX_SIGNAL_ETAT_CONDA_OUV_AR,
    NTW_RX_SIGNAL_ETAT_CPO_ARD,
    NTW_RX_SIGNAL_ETAT_CPO_ARG,
    NTW_RX_SIGNAL_ETAT_COFFRE,
    NTW_RX_SIGNAL_RP_LOCK_UNLOCK_REM_GEN2,
    NTW_RX_SIGNAL_RQ_HFA_BUZZER_ACTIVATION,
    NTW_RX_SIGNAL_TURN_LIGHTS_STATUS,
    NTW_RX_SIGNAL_HORN_STATUS,
    NTW_RX_SIGNAL_PRECOND_REQ_STATUS,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_LU_1,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_MA_1,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_ME_1,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_JE_1,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_VE_1,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_SA_1,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_DI_1,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_LU_2,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_MA_2,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_ME_2,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_JE_2,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_VE_2,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_SA_2,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_DI_2,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_LU_3,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_MA_3,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_ME_3,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_JE_3,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_VE_3,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_SA_3,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_DI_3,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_LU_4,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_MA_4,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_ME_4,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_JE_4,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_VE_4,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_SA_4,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_DI_4,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_H_1,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_H_2,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_H_3,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_H_4,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_ON_1,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_ON_2,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_ON_3,
    NTW_RX_SIGNAL_MENU_CLIM_PRECOND_ON_4,
    NTW_RX_SIGNAL_DISPO_ETAT_PRECOND,
    NTW_RX_SIGNAL_ETAT_MENU_CLIM_PREC_ASAP,
    NTW_RX_SIGNAL_MARCHE_AR_TCUM,
    NTW_RX_SIGNAL_REMOTE_IMMO_STATUS,
    NTW_RX_SIGNAL_UNITE_CONSO,
    NTW_RX_SIGNAL_DRV_INFO_BOOT,
    NTW_RX_SIGNAL_RPAS_ACTIVATION_STATUS,
    NTW_RX_SIGNAL_BAR_SECT_LEFT_RPAS,
    NTW_RX_SIGNAL_BAR_SECT_CENTER_LEFT_RPAS,
    NTW_RX_SIGNAL_BAR_SECT_CENTER_RIGHT_RPAS,
    NTW_RX_SIGNAL_BAR_SECT_RIGHT_RPAS,
    NTW_RX_SIGNAL_FPAS_ACTIVATION_STATUS,
    NTW_RX_SIGNAL_BAR_SECT_LEFT_FPAS,
    NTW_RX_SIGNAL_BAR_SECT_CENTER_LEFT_FPAS,
    NTW_RX_SIGNAL_BAR_SECT_CENTER_RIGHT_FPAS,
    NTW_RX_SIGNAL_BAR_SECT_RIGHT_FPAS,
    NTW_RX_SIGNAL_DISPO_UNITES_LANGUE,
    NTW_RX_SIGNAL_TEMP_EAU_MOT,
    NTW_RX_SIGNAL_CONTACT_FREIN1,
    NTW_RX_SIGNAL_REGIME_MOTEUR,
    NTW_RX_SIGNAL_DMD_REINIT_ALL_PARAM,
    NTW_RX_SIGNAL_SIGN_IVC,
    NTW_RX_SIGNAL_PEM_ESTIM_ELEC_POWER,
    NTW_RX_SIGNAL_ETAT_EMB_COMP,
    NTW_RX_SIGNAL_DYN_SLOPE_VALUE,
    NTW_RX_SIGNAL_ALLUM_FLECHE,
    NTW_RX_SIGNAL_CONSO_MOY_TRAJ,

    NTW_RX_SIGNAL_DIST_TRAJINST_TOT_ZEVPCT,
    NTW_RX_SIGNAL_ETAT_RESEAU_ELEC,
    NTW_RX_SIGNAL_ALARM_MONITORING_STATE,
    NTW_RX_SIGNAL_FLG_AVOL_ICN,
    NTW_RX_SIGNAL_FLG_INIT_AVOL,
    NTW_RX_SIGNAL_P_ACTI_BREAK_IN_NOTIF,
    NTW_RX_SIGNAL_RTAB_CHRG_REMAINING_TIME,
    NTW_RX_SIGNAL_TOT_ENERGY_ACCUMULATION,
    NTW_RX_SIGNAL_CONSO,
    NTW_RX_SIGNAL_CHIME_MUTE_RES,
    NTW_RX_SIGNAL_AUXILIARY_POWER,
    NTW_RX_SIGNAL_CONSO_MOY_ZEV_2,
    NTW_RX_SIGNAL_CONSO_MOY_ZEV_3,
    NTW_RX_SIGNAL_CONSO_MOY_ZEV_4,
    NTW_RX_SIGNAL_CONSO_MOY_ZEV_5,
    NTW_RX_SIGNAL_CONSO_MOY_ZEV_6,
    NTW_RX_SIGNAL_FATC_CABIN_TEMPERATURE,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_PEPS_TYPE_MESSAGE_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_PEPS_ETAT_DTC_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_PEPS_KILOMETRAGE_JDD_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_PEPS_NOMBRE_TRAMES_Mx,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_PEPS_NUMERO_TRAME_Mx01,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_PEPS_NUMERO_TRAME_Mx02,
    NTW_RX_SIGNAL_HS3_EMIS_NEW_JDD_PEPS_SITUATION_VIE_JDD_Mx02,
    NTW_RX_SIGNAL_NIV_HUILE,
    NTW_RX_SIGNAL_STATUS_SOC,
    NTW_RX_SIGNAL_Authorization_install,
    NTW_RX_SIGNAL_HS3_BSI_INF_PROFILS__FCT_MENU_ILV_ILV,
    NTW_RX_SIGNAL_HS3_UC_FREIN_5ED__ETAT_IND_ROUE_ARG,
    NTW_RX_SIGNAL_HS3_UC_FREIN_5ED__ETAT_IND_ROUE_AVD,
    NTW_RX_SIGNAL_HS3_UC_FREIN_5ED__ETAT_IND_ROUE_AVG,
    NTW_RX_SIGNAL_HS3_CDE_BSRF_7__ETAT_ESSUYAGE,
    NTW_RX_SIGNAL_HS3_CDE_BSRF_8__CDE_ESV_AR,
    NTW_RX_SIGNAL_HS3_CDE_BSRF_8__CDE_ESV,
    NTW_RX_SIGNAL_HS3_CDE_BSRF_7__NUM_VITESSE,
    NTW_RX_SIGNAL_HS3_CDE_CMB_SIGNALISATION__FEUX_ROUTE,
    NTW_RX_SIGNAL_HS3_CDE_CMB_SIGNALISATION__FEUX_CROIS,
    NTW_RX_SIGNAL_HS3_BSI_INF_PROFILS__UNITE_TEMPERATURE,
    NTW_RX_SIGNAL_COMMANDES_BSI_36__SENS_ROULAGE,
    NTW_RX_SIGNAL_HS3_CDE_BSRF_3__VITESSE_LACET,
    NTW_RX_SIGNAL_VCU_CTRL_INPUT_201h__HVBAT_CHARGING_STATUS,
    NTW_RX_SIGNAL_HS3_DONNEES_BSI_RAPIDES__VITM,
    NTW_RX_SIGNAL_HS3_REMOTE_ACTION_REP__REMOTE_LOCK_UNLOCK_STAT,
    NTW_RX_SIGNAL_HS3_BSI_INF_PRG_CLIM__RETOUR_ETAT_PRECOND,

    NTW_RX_SIGNAL_HS3_CDE_BSRF_16_HV_BATT_REAL_VOLT_HD,
    NTW_RX_SIGNAL_SC_INFO_BCM_442h_VCU_CHARG_TYPE,
    NTW_RX_SIGNAL_HS3_DAT_TBMU_494_TOTAL_HV_BATT_NEGATIVE_ENERGY,
    NTW_RX_SIGNAL_COMMANDES_BSI_36_ETAT_JN,
    NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_1,
    NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_2,
    NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_3,
    NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_4,
    NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_5,
    NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_6,
    NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_7,
    NTW_RX_SIGNAL_HS3_ETAT_HDC_3_STEERWHL_PUSH_8,
    NTW_RX_SIGNAL_HS3_CDE_BSRF_14_HV_BATT_SOE_HD,
    NTW_RX_SIGNAL_HS3_CDE_BSRF_16_HV_BATT_SOH_RES_HD,
    NTW_RX_SIGNAL_CRASH_MAX_DELTA_VX_oETAT_INFO_CRASH_19,
    NTW_RX_SIGNAL_CRASH_MAX_DELTA_VY_oETAT_INFO_CRASH_19,
    NTW_RX_SIGNAL_CRASH_MAX_DELTA_V_RANGE_oETAT_INFO_CRASH_19,
    //NTW_RX_SIGNAL_ENTREE_AIR,

    NTW_RX_SIGNAL_DMD_GAP_VCI,
    NTW_RX_SIGNAL_DMD_OK_VCI,
    NTW_RX_SIGNAL_DMDP_XVV_ACTIVE_DESACTIVE,
    NTW_RX_SIGNAL_DMDP_XVV_SET_MOINS,
    NTW_RX_SIGNAL_DMDP_XVV_SET_PLUS,
    NTW_RX_SIGNAL_ET_INHIBITION,
    NTW_RX_SIGNAL_RETOURN,
    NTW_RX_SIGNAL_DIST_VHL_L_LINE_EXT,
    NTW_RX_SIGNAL_DIST_VHL_R_LINE_EXT,
    NTW_RX_SIGNAL_SEC_DIST_VHL_L_LINE_EXT,
    NTW_RX_SIGNAL_SEC_DIST_VHL_R_LINE_EXT,
    NTW_RX_SIGNAL_DEM_EFFAC_DEF,
    NTW_RX_SIGNAL_DIAG_MUX_ON,
    NTW_RX_SIGNAL_MODE_ECO,
    NTW_RX_SIGNAL_VALID_CAFR,
    NTW_RX_SIGNAL_HV_BATT_NB_PROBE_TEMP_MAX,
    NTW_RX_SIGNAL_HV_BATT_NB_PROBE_TEMP_MIN,
    NTW_RX_SIGNAL_HV_BATT_PROB_TEMP_NUMBER,
    NTW_RX_SIGNAL_HV_BATT_PROB_VOLT_NUMBER,
    NTW_RX_SIGNAL_HV_BATT_TEMP_MIN,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT1_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT10_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT100_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT101_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT102_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT103_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT104_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT105_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT106_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT107_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT108_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT11_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT12_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT13_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT14_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT15_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT16_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT17_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT18_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT19_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT2_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT20_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT21_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT22_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT23_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT24_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT25_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT26_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT27_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT28_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT29_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT3_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT30_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT31_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT32_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT33_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT34_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT35_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT36_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT37_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT38_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT39_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT4_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT40_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT41_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT42_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT43_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT44_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT45_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT46_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT47_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT48_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT49_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT5_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT50_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT51_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT52_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT53_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT54_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT55_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT56_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT57_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT58_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT59_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT6_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT60_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT61_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT62_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT63_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT64_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT65_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT66_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT67_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT68_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT69_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT7_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT70_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT71_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT72_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT73_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT74_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT75_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT76_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT77_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT78_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT79_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT8_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT80_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT81_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT82_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT83_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT84_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT85_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT86_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT87_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT88_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT89_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT9_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT90_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT91_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT92_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT93_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT94_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT95_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT96_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT97_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT98_14B,
    NTW_RX_SIGNAL_BMS_BATTCELLVOLT99_14B,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx00,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx01,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx02,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx03,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx04,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx05,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx06,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx07,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx08,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx09,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx0A,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx0B,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx0C,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx0D,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx0E,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx0F,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx10,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx11,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx12,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx13,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx14,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx15,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx16,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx17,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx18,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx19,
    NTW_RX_SIGNAL_LID_BATTCELLVOLT_Mx1A,
    NTW_RX_SIGNAL_DIST,
    NTW_RX_SIGNAL_SECU_VITV,
    NTW_RX_SIGNAL_REGUL_ABR,
    NTW_RX_SIGNAL_HV_BATT_CELL_VOLT_MAX,
    NTW_RX_SIGNAL_HV_BATT_CELL_VOLT_MIN,
    NTW_RX_SIGNAL_HV_BATT_NB_CELL_VOLT_MAX,
    NTW_RX_SIGNAL_HV_BATT_NB_CELL_VOLT_MIN,
    NTW_RX_SIGNAL_ETAT_MA,
    NTW_RX_SIGNAL_DOOR_LOCK_STATE_BW,
    NTW_RX_SIGNAL_DOOR_LOCK_STATE_FD,
    NTW_RX_SIGNAL_DOOR_LOCK_STATE_FP,
    NTW_RX_SIGNAL_DOOR_LOCK_STATE_RL,
    NTW_RX_SIGNAL_DOOR_LOCK_STATE_RR,
    NTW_RX_SIGNAL_ESPACT,
    NTW_RX_SIGNAL_FEUX_ABAR,
    NTW_RX_SIGNAL_FEUX_ABAV,
    NTW_RX_SIGNAL_FEUX_DIURNES,
    NTW_RX_SIGNAL_FRPK,
    NTW_RX_SIGNAL_OUCC,
    NTW_RX_SIGNAL_SERVICE,
    NTW_RX_SIGNAL_STOP,
    NTW_RX_SIGNAL_DISPO_ETAT_RECHARGE,
    NTW_RX_SIGNAL_ETAT_MENU_RTAB_TYPE_RECH,
    NTW_RX_SIGNAL_MENU_RTAB_ENERGY_RECOVER,
    NTW_RX_SIGNAL_MENU_RTAB_REC_H_DEBUT,
    NTW_RX_SIGNAL_DMD_ALLUM_AFIL,
    NTW_RX_SIGNAL_COUPLE_REEL,
    NTW_RX_SIGNAL_ETAT_CHARGE_BATTERIE,
    NTW_RX_SIGNAL_PRESSION_MAITRE_CYL,
    NTW_RX_SIGNAL_U_BATT_BECB,
    NTW_RX_SIGNAL_ENT_PUSH_LKA,
    NTW_RX_SIGNAL_REQ_HEATING_STRWHL,
    NTW_RX_SIGNAL_LKA_LEFT_LINE_INFO,
    NTW_RX_SIGNAL_LKA_RIGHT_LINE_INFO,
    NTW_RX_SIGNAL_HVBAT_CHARGING_STATUS,
    NTW_RX_SIGNAL_LANGUE_VHL_2,
    NTW_RX_SIGNAL_FONCT_ACT_LVV_RVV,
    NTW_RX_SIGNAL_VIT_CONS_LVV_RVV,
    NTW_RX_SIGNAL_COMPTEUR_RAZ_GCT,
    NTW_RX_SIGNAL_CTX_JDD,
    NTW_RX_SIGNAL_ETAT_CPO_COND_REPLI,
    NTW_RX_SIGNAL_ETAT_CPO_LUNETTE,
    NTW_RX_SIGNAL_ON_REFUSE,
    NTW_RX_SIGNAL_HV_BATT_REAL_CURR_HD,
    NTW_RX_SIGNAL_DIST_VHL_TOTAL,
    NTW_RX_SIGNAL_VOL_FUEL_CONSO_TOTAL,
    NTW_RX_SIGNAL_DISPO_PARAM,
    NTW_RX_SIGNAL_VIT_A_AFFICHER_ILV,
    NTW_RX_SIGNAL_ETAT_SELECTIVITE_COND,
    NTW_RX_SIGNAL_REMOTE_IMMO_RESET,
    NTW_RX_SIGNAL_LID_PROBETEMP_Mx00,
    NTW_RX_SIGNAL_LID_PROBETEMP_Mx01,
    NTW_RX_SIGNAL_LID_PROBETEMP_Mx02,
    NTW_RX_SIGNAL_LID_PROBETEMP_Mx03,
    NTW_RX_SIGNAL_LID_PROBETEMP_Mx04,
    NTW_RX_SIGNAL_LID_PROBETEMP_Mx05,
    NTW_RX_SIGNAL_LID_PROBETEMP_Mx06,
    NTW_RX_SIGNAL_LID_PROBETEMP_Mx07,
    NTW_RX_SIGNAL_DRIVER_CAMERA_FAILURE,
    NTW_RX_SIGNAL_ETAT_GMP_HYB,
    NTW_RX_SIGNAL_PEM_SPEED_HD,
    NTW_RX_SIGNAL_PILOTAGE_PLAF1_FONC,
    NTW_RX_SIGNAL_VILL_ROUTE_ET_4POS,
    NTW_RX_SIGNAL_IMMO_EVENT,
    NTW_RX_SIGNAL_IMOBNREQACPTD,
    NTW_RX_SIGNAL_KEY_LEARNING_ALARM,
    NTW_RX_SIGNAL_UNAUTHENTDSTRTG,
    NTW_RX_SIGNAL_RETOUR_ETAT_PRECOND,
    NTW_RX_SIGNAL_RAP_BV_ENGAGE_MECA,
    NTW_RX_SIGNAL_RAP_ENGAGE_CALCULE,
    NTW_RX_SIGNAL_S_LVAV,
    NTW_RX_SIGNAL_ETAT_MT,
    NTW_RX_SIGNAL_KILOMETRAGE,
    NTW_RX_SIGNAL_DMDM_RTAB_RECHRG_H_DEBUT,
    NTW_RX_SIGNAL_DMDM_RTAB_TYPE_RECHRG,
    NTW_RX_SIGNAL_P_TEM_LED_BAS_NIV_BATT,
    NTW_RX_SIGNAL_CONSO_TRIP_CFT,
    NTW_RX_SIGNAL_ACPK_VALIDATION_DEMAND,
    NTW_RX_SIGNAL_AUTO_LOW_BEAM_CMD,
    NTW_RX_SIGNAL_CDE_CLG_ET_HDC,
    NTW_RX_SIGNAL_CHMSL_STOP_CMD,
    NTW_RX_SIGNAL_DIAG_LB_L,
    NTW_RX_SIGNAL_DIAG_LB_R,
    NTW_RX_SIGNAL_DMD_COMMUT_FEUX,
    NTW_RX_SIGNAL_LEFT_LINE_TYPE_EXT,
    NTW_RX_SIGNAL_OUV_CAPOT_ALE_EVE_US,
    NTW_RX_SIGNAL_POS_LEVIER_P_N,
    NTW_RX_SIGNAL_RIGHT_LINE_DETECTION_LKA,
    NTW_RX_SIGNAL_RIGHT_LINE_TYPE_EXT,
    NTW_RX_SIGNAL_HV_BATT_SOE_HD,/*R15 Release Need to be Deleted*/
    NTW_RX_SIGNAL_TEMP_AIR_MOT,
    NTW_RX_SIGNAL_DISTRIBUTION_AVD,
    NTW_RX_SIGNAL_DRV_INFO_BONNET,
    NTW_RX_SIGNAL_AFF_MENU_CLIM_PRECOND,
    NTW_RX_SIGNAL_AFF_MENU_RTAB_RECHARGE,
    NTW_RX_SIGNAL_RECHARGE_HMI_STATE_EVO,
    NTW_RX_SIGNAL_APPEL_PH_ET_HDC,
    NTW_RX_SIGNAL_ETAT_ANTIBR_AR,
    NTW_RX_SIGNAL_DMDP_GAV_EJECT_TRAPPE_CARB,
    NTW_RX_SIGNAL_REM_REAL_TRQ,
    NTW_RX_SIGNAL_OTA_UP_TYP,
    NTW_RX_SIGNAL_ETAT_FONCT_LVV_RVV,
    NTW_RX_SIGNAL_PEM_ENGINE_RPM,
    NTW_RX_SIGNAL_RECHARGE_HMI_STATE,
    NTW_RX_SIGNAL_RTAB_ZEV_RANGE_CHRG_RATE,
    NTW_RX_SIGNAL_ROT_1_CRAN_ESS,
    NTW_RX_SIGNAL_ROT_2_CRAN_ESS,
    NTW_RX_SIGNAL_SEC_LEFT_LINE_DETEC_LKA,
    NTW_RX_SIGNAL_SEC_LEFT_LINE_TYPE_EXT,
    NTW_RX_SIGNAL_SEC_RIGHT_LINE_DETEC_LKA,
    NTW_RX_SIGNAL_SEC_RIGHT_LINE_TYPE_EXT,
    NTW_RX_SIGNAL_DMD_AUTO_DRVRSEATHEATG,
    NTW_RX_SIGNAL_DMD_AUTO_PASSSEATHEATG,
    NTW_RX_SIGNAL_RVM_FOLDING_LOCAL_RQST,
    NTW_RX_SIGNAL_TIME_HV_BATT_OVER_CH_PWR,
    NTW_RX_SIGNAL_TIME_HV_BATT_OVER_DCH_PWR,
    NTW_RX_SIGNAL_TOTAL_HV_BATT_NEG_QTY_CURR,
    NTW_RX_SIGNAL_AFF_MAINT,
    NTW_RX_SIGNAL_PICTO_CLE,
    NTW_RX_SIGNAL_TYPE_MAINT,
    NTW_RX_SIGNAL_TYPE_MAINT_2,
    NTW_RX_SIGNAL_DMD_AAF,
    NTW_RX_SIGNAL_ETAT_BV_FREEWHEELING,
    NTW_RX_SIGNAL_LEFT_LINE_DETECTION_LKA,
    NTW_RX_SIGNAL_NIV_HUILE_MOT,
    NTW_RX_SIGNAL_VILL_ROUTE_ET_HDC,
    NTW_RX_SIGNAL_VOLONTE_COND,
    NTW_RX_SIGNAL_TIME_HV_BATT_HIGH_TEMP_HIGH_SOC,
    NTW_RX_SIGNAL_TIME_HV_BATT_HIGH_TEMP_LOW_SOC,
    NTW_RX_SIGNAL_TIME_HV_BATT_HIGH_VOLTAGE,
    NTW_RX_SIGNAL_TOTAL_HV_BATT_NEG_QTY_CURR_PLUG,
    NTW_RX_SIGNAL_DC_CHARGE_RELAY_ACTUATION_COUNT,
    NTW_RX_SIGNAL_MAIN_RELAY_ACTUATION_COUNTER,
    NTW_RX_SIGNAL_TIME_HV_BATT_OVER_TEMPERATURE,
    NTW_RX_SIGNAL_TOTAL_HV_BATT_POS_QTY_CURR,
    NTW_RX_SIGNAL_TOTAL_HV_BATT_POSITIVE_ENERGY,
    NTW_RX_SIGNAL_Flag_BattEnergy_Extd,
    NTW_RX_SIGNAL_DATA_DIAG_RES_BCM,
    NTW_RX_SIGNAL_DATA_DIAG_REQ_DCSD,
    NTW_RX_SIGNAL_DATA_DIAG_REQ_IVI,
    NTW_RX_SIGNAL_DATA_DIAG_REQ_7FFh,
    NTW_RX_SIGNAL_DMD_ALLUMAGE_FA,
    NTW_RX_SIGNAL_PRES_PRIVACY_MODE,
    NTW_RX_SIGNAL_ETAT_MENU_CHARGE_LIMIT,
    NTW_RX_DCSD_DIAG_REQ_Com_DCSD_Signal_Rx,
/*Upcoming signals should be to added above */
	NTW_RX_SIGNAL_NUMBER,                          //!< Receive signal number
} NTW_TxSGN_Key;


enum iviCanDatasend{
NTW_TX_SIGNAL_ACCUEIL_COND = 0, 
NTW_TX_SIGNAL_EXTGPSCLKYR,
NTW_TX_SIGNAL_EXTGPSCLKSEC1,
NTW_TX_SIGNAL_EXTGPSCLKMINS,
NTW_TX_SIGNAL_EXTGPSCLKMTH,
NTW_TX_SIGNAL_EXTGPSCLKHR,
NTW_TX_SIGNAL_EXTGPSCLKDAY,
NTW_TX_SIGNAL_UPD_SEC_CLK_DAY_REQ,
NTW_TX_SIGNAL_UPD_SEC_CLK_HR_REQ,
NTW_TX_SIGNAL_UPD_SEC_CLK_MIN_REQ,
NTW_TX_SIGNAL_UPD_SEC_CLK_MTH_REQ,
NTW_TX_SIGNAL_UPD_SEC_CLK_SEC_REQ,
NTW_TX_SIGNAL_UPD_SEC_CLK_YR_REQ,
NTW_TX_SIGNAL_CS_PARAM_VHL_MOBILITY,
NTW_TX_SIGNAL_DMD_MENU_PRIVACY_MODE,
NTW_TX_SIGNAL_MODE_HEURE_CLIENT_HS7,
NTW_TX_SIGNAL_HEURE_HORLOGE_HS7,
NTW_TX_SIGNAL_MINUTE_HORLOGE_HS7,
NTW_TX_SIGNAL_JOUR_HORLOGE_HS7,
NTW_TX_SIGNAL_MOIS_HORLOGE_HS7,
NTW_TX_SIGNAL_ANNEE_HORLOGE_HS7,
//NTW_TX_SIGNAL_AFFICHAGE_HORLOGE_HS7,
NTW_TX_SIGNAL_REV,
NTW_TX_SIGNAL_RQ_LOCK_UNLOCK_REM_GEN2,
NTW_TX_SIGNAL_REMOTE_HORN_REQUEST,
NTW_TX_REMOTE_TURN_LIGHTS_REQ,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ASAP,
NTW_TX_SIGNAL_REMOTE_IMMO_REQ,
NTW_TX_SIGNAL_REMOTE_IMMO_PWD,
NTW_TX_SIGNAL_DMD_MENU_RTAB_TYPE_RECH_REM,
NTW_TX_SIGNAL_DMD_MENU_RTAB_REC_H_DEB_REM,
NTW_TX_SIGNAL_DISPO_DMD_RECHARGE_REM,
NTW_TX_SIGNAL_NEXT_CHARG_STATION_ARRIVALSOC,
NTW_TX_SIGNAL_NEXT_CHARG_STATION_DISTANCE,
NTW_TX_SIGNAL_NEXT_CHARG_STATION_MAXPOWER,
NTW_TX_SIGNAL_NEXT_CHARG_STATION_TIMETOTRAVEL,
NTW_TX_SIGNAL_CHIME_MUTE_REQ,
NTW_TX_SIGNAL_IVI_Language_SELECTION,
NTW_TX_SIGNAL_Update_Req,
NTW_TX_SIGNAL_REMOTE_ACTION_REQ_254__REMOTE_LOCK_UNLOCK_REQ,

NTW_TX_SIGNAL_FUSEAU_HORAIRE,
NTW_TX_SIGNAL_DISPO_DMD_PRECOND,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_1,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_2,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_4,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_3,
NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_4,
NTW_TX_SIGNAL_GSMEXTCLKDAY,
NTW_TX_SIGNAL_GSMEXTCLKHR,
NTW_TX_SIGNAL_GSMEXTCLKMINS,
NTW_TX_SIGNAL_GSMEXTCLKMTH,
NTW_TX_SIGNAL_GSMEXTCLKSEC1,
NTW_TX_SIGNAL_GSMEXTCLKYR,
NTW_TX_SIGNAL_DISPO_DMD_RECHARGE,
NTW_TX_SIGNAL_DMD_MENU_RTAB_REC_H_DEB,
NTW_TX_SIGNAL_DMD_MENU_RTAB_TYPE_RECH,
NTW_TX_SIGNAL_DISPO_DMD_PRECOND_REM,
NTW_TX_SIGNAL_C_ABSENT,
NTW_TX_SIGNAL_C_BUSOFF,
NTW_TX_SIGNAL_C_DEFAUTPERTECOM,
NTW_TX_SIGNAL_C_MUTE,
NTW_TX_SIGNAL_DIAG_MUX_ON_IVI,
NTW_TX_SIGNAL_EFF_DEF_IVI_HAB,
NTW_TX_SIGNAL_DMD_MENU_PRIVACY_MODE_REM,
NTW_TX_SIGNAL_DMD_MUTE_SUAL,
NTW_TX_SIGNAL_XCALL_STATUS,
NTW_TX_DCSD_DIAG_RES_Com_DCSD_Signal_Tx,
/*Upcoming signals should be to added above */
NTW_TX_SIGNAL_NUMBER, 
 
};
enum phonecallstatus{
  PHONE_STATE_NEW = 00,
  PHONE_STATE_CONNECTING = 01,
  PHONE_STATE_SELECT_PHONE_ACCOUNT = 02,
  PHONE_STATE_DIALING = 03,
  PHONE_STATE_ACTIVE = 04,
  PHONE_STATE_RINGING = 05,
  PHONE_STATE_ON_HOLD = 06,
  PHONE_STATE_DISCONNECTED = 07,
  PHONE_STATE_ABORTED = 8,
  PHONE_STATE_DISCONNECTING = 9,
  PHONE_STATE_PULLING = 10,
  PHONE_STATE_ANSWERED = 11,
  PHONE_STATE_AUDIO_PROCESSING = 12,
  PHONE_STATE_SIMULATED_RINGING = 13,
  
  
};
enum SoftKeysStatus{
  AUTO_SWITCH_ON = 00,
  RADIO_OFF_DELAY = 01,
  SWITCH_OFF_WITH_DOOR = 02 ,
  DOOR_CONDITION_2 =03,
  RADIO_OFF_WITH_DOOR = 04,
  POWER_SOFT_BUTTON_PRESSED = 05,
 
};
enum smsStatus{
  SMS_RECEIVED = 01,
  MMS_RECEIVED = 02,
  WAP_PUSH_RECEIVED = 03,
};
enum ivi_hmi_touchData{
  SCREEN_TOUCH = 0,
  X_CALL_ACTIVE =1,
  AUDIO_AUTO_STATUS =2,
  PARK_ASSIST_APP_STATUS=3,
  PARTIAL_OP_SCREEN_MODE=4,
  IDLE_SCREEN_MODE=5,
  STANDBY_SCREEN_MODE=6 ,
  ANTI_THEFT_STATUS=7,
  FORCE_RESET=8,
};

enum dotteParam_upperlayer{
	VEHICLE_BRAND =0,
	DISPLAY_TYPE,
	COUNTRY_CODE,
	START_ANM_MODE,
	CONT_START_ANM,
	WELCOME_SOUND,
	VEH_LINE,
	VEH_FUEL_TYPE,
	DAYNYT_THM_EN,
	EV_CONN_TYPE,
	AMP_PRESENT,
	THEMED_SOUNDS,
	SGN_SRC_ENABLED,
	PAM_CHIME_EN,
	PAM_CONFIG,
	PAS_VOL_STRAT,
	PAM_CHIME_MODE,
	SYS_TYPE,
	SIGNAL_TYPE,
	RPAS_PRESENT,
	FPAS_PRESENT,
	RVC_TYPE,
	VP1_ENABLED,
	DRIVER_SIDE,
	START_AUD_VOL,
	REV_MUTE_EN,
	EQL_SETTING,
	SPK_CONFIG,
	SIVI_GEAR_BOX_TYPE,
	SIVI_HVAC_TYPE,
	VEH_THEME,
	DAB_PRESENT,
	AM_TUNER_EN,
	HW_IGN_PRESENT,
	HW_ACC_PRESENT,
    ACC_PRESENT,
	CONN_PRESENT,
	NAV_PRESENT,
	Reserve0,
	Reserve2,
	PWR_BTN_TYPE,
	Reserve3,
	REGULATION_TYPE,
	Reserve5,
	TRF_CONN_SVC,
	DISCL_CTRLS,
	DISCL_REG_SPEC,
	DISCL_SCR_MODE,
	TYPE_XCALL,
	Reserve6,
	Reserve7,
	AUDIO_BRAND,
	Reserve8,
	Reserve9,
	SWL_THM_EN,
	DRV_DIST_MODE,
	CLIM_HMI_EN,
	Reserve10,
	MHU_TYPE,
	Reserve11,
	ICS_PRESENT,
	XCALL_SPK_PRES,
	SURROUND,
	SDW_CONFIG,
	SDW_CHIME_MODE,
	Reserve12,
	BSS_PRESENT,
	BSS_CONFIG,
	SRRP_PRESENCE,
	SRRP2_PRESENCE,
	HALF_PRESENT,
	Reserve13,
	Reserve14,
	ACALLED_PRES,
	Reserve15,
	VEHICLE_TYPE,
	Reserve16,
	ECALL_BTN_PRESS,
	ACALL_BTN_PRESS,
	ECALLED_PRESS,
	BUB_PRESENCE,
	HTD_WHL_PRESENT,
	HTDS_DRV_PRES,
	HTDS_PAS_PRES,
	Reserve20,
	FRONT_DEFROSTER,
	REAR_DEFROSTER,
	Reserve21,
	SWS_TYPE,
	Reserve22,
	PROF_DET_EN,
	RVC_INST_LOC,
	DYN_GRID_EN,
	Reserve23,
	RPAS_CHM_ENT,
	FPAS_CHM_ENT,
	MEM_DYN_GRID,
	PRV_MODE_MENU,
	Reserve24,
	IDLE_SCR_MODE,
	Reserve25,
	Reserve26,
	SIVI_UNIT_SYNC_EN,
	SIVI_SHOW_UNITS,
	SIVI_AIRBAG_INT,
	Reserved27,
	SUSPEND_RAM_DURATION,
	LISTEN_MODE1_DURATION,
	LISTEN_MODE2_DURATION,
	LISTEN_MODE3_DURATION,
	LISTEN_MODE_1_CONN_RETRY_TIME,
	LISTEN_MODE_3_WAKE_UP_TIME,
	POWER_BUTTON_TYPE,
	SANITY_REBOOT_PERIOD,
	MAX_CALL_DURATION,
	MAX_AUTONOMY,
	HV_BATT_CAP,
	dotteParam_Lenth,
};
enum internalSignal{
INTERNAL_VoiceRecognitionReq = 0,
INTERNAL_SourceReq,
INTERNAL_SeekUpAcceptCallReq,
INTERNAL_SeekDownRejectCallReq,
INTERNAL_VolUpReq,
INTERNAL_VolDownReq,
INTERNAL_MuteReq,
INTERNAL_Ignition_Status,
INTERNAL_CurrentPower_Mode,
INTERNAL_PrevPower_Mode,
INTERNAL_InstantFuelConsInt,
INTERNAL_FuelLevelInt,
INTERNAL_AutonomyInt,
INTERNAL_FuelLevelMinimumStsInt,
INTERNAL_RegeneratedEnergyInt,
INTERNAL_PrivacyModeSts,
INTERNAL_FuelConsUnitInt,
InternalSig_Lenth
};
enum SystemTime_Internalsignals{
SIVIGNSSClkHour,
SIVIGNSSClkMinute,
SIVIGNSSClkSecond,
SIVIGNSSClkDay,
SIVIGNSSClkMonth,
SIVIGNSSClkYear,
SIVIGSMClkHour,
SIVIGSMClkMinute,
SIVIGSMClkSecond,
SIVIGSMClkDay,
SIVIGSMClkMonth,
SIVIGSMClkYear,
SystemSecureTimeHr,
SystemSecureTimeMin,
SystemSecureTimeSec,
SystemSecureTimeDay,
SystemSecureTimeMth,
SystemSecureTimeYr,
SystemUserTimeHr,
SystemUserTimeMin,
SystemUserTimeDay,
SystemUserTimeMth,
SystemUserTimeYr
};
enum ev_signals{
InstantPowerConsumption = 0,  
InstantComfortPowerConsumption,  
TripAvgPowertrainPowerConsumption,  
AvgComfortPowerConsumption,  
AvgOtherTripPowerConsumption,  
TripEnergyRecharged,  
FrontWheelsElectricPowerState,  
RearWheelsElectricPowerState,  
BatteryChargeStatus,  
HVBatteryIsCharging,  
HVBatteryMaxRange,  
HVBatterySOH,  
BatEnergyRemaining,  
AuxPwrUsage,  
RemainingBatCapacity,  
BatTempProbe1,  
BatTempProbe2,  
BatTempProbe3,  
BatTempProbe4,  
BatTempProbe5,  
BatTempProbe6,  
BatTempProbe7,  
BatTempProbe8,  
BatTempProbe9,  
BatTempProbe10,  
BatTempProbe11,  
BatTempProbe12,  
BatTempProbe13,  
BatTempProbe14,  
BatTempProbe15,  
BatTempProbe16,  
BatTempProbe17,  
BatTempProbe18,  
BatTempProbe19,  
BatTempProbe20,  
BatTempProbe21,  
BatTempProbe22,  
BatTempProbe23,  
BatTempProbe24,  
BatTempProbe25,  
BatTempProbe26,  
BatTempProbe27,  
BatTempProbe28,  
BatTempProbe29,  
BatTempProbe30,  
BatTempProbe31,  
BatTempProbe32,  
BatTempProbe33,  
BatTempProbe34,  
BatTempProbe35,  
BatTempProbe36,  
BatTempProbe37,  
BatTempProbe38,  
BatTempProbe39,  
BatTempProbe40,  
BatTempProbe41,  
BatTempProbe42,  
BatTempProbe43,  
BatTempProbe44,  
BatTempProbe45,  
BatTempProbe46,  
BatTempProbe47,  
BatTempProbe48,  
BatTempProbe49,  
BatTempProbe50,  
BatTempProbe51,  
BatTempProbe52,  
BatTempProbe53,  
BatTempProbe54,  
BatAvgTemperature,  
HvbatChargingStatus,  
BatteryPreConditioningSts,  
BatteryMaxVoltage,  
BatteryMaxCurrent,  
TractionBatteryTotalCapacity,  
HVBatteryActiveConnector,  
TripAvgBattPowerCons,  
ElectricMotorPower,  
HVBatteryPercentage,  
RegeneratedEnergy,  
VcuChargeType,  
HvBattNegativeEnergy,  
HvBatterySOE,  
HvBattSoH,  
HVBatteryMaxCharge,  
HeadlampsPosition,
EVSignal_Length
};

enum bcm_signals{
MonotonicClock = 0,  
TrailerPresent,  
DriverPresent,  
WiperSts,  
WiperSts2,  
WiperSts3,  
WiperSts4,  
OutTemp,  
HighBeamSts,  
LowBeamSts,  
DayNightMode,  
BCMClockYear,  
BCMClockHour,  
BCMClockDay,  
BCMClockMinute,  
BCMClockMonth,  
HourFormat,  
CanDistanceUnit,  
CanTemperatureUnit,  
ClkFormat,  
CapacityUnit,  
SpeedUnityValidity,  
EtatPrincipSev,  
FuelConsUnit,  
TotalKM,  
TripAverageFuelConsumption,  
DayNightStatus,  
SteerWhlPush1,  
SteerWhlPush2,  
SteerWhlPush3,  
SteerWhlPush4,  
SteerWhlPush5,  
SteerWhlPush6,  
SteerWhlPush7,  
SteerWhlPush8,  
VehPrivacyMode,
BCMSignal_Length
};

enum camera_signals{
RPASRightBarSts = 0,
PASChimeMuteRes,
CameraSignal_Length

};

enum CanCommonSignal{
EcallCrash = 0,
BoucCeintPass,
BoucCeintAvmil,
BoucCeintAr2mil,
BoucCeintAr2d,
BoucCeintAr2g,
BoucCeintAr3g,
BoucCeintAr3d,
BoucCeintCond,
FrontFVit,
FrontGVit,
FrontMVit1,
FrontMVit2,
FrontRepar,
ArrFVit,
ArrGVit,
ArrMVit,
ArrRepar,
LatFVit,
LatGVit,
LatMVit,
LatRepar,
TypeDir,
SideOfLateralCrash,
CrashMaxDeltaVY,
CrashMaxDeltaVX,
CrashMaxDeltaVRange,
Teau,
THuile,
NivHuile,
KMaint,
NbJourMaintenance,
SignMaint,
SignEcheance,
EtatGmp,
InfoCrash,
Pieton,
CabinTemperature,
GearPosition,
FuelType,
FuelConsumption,
WheelsSpeed,
PulseCountFLWheel,
PulseCountFRWheel,
PulseCountRLWheel,
PulseCountRRWheel,
Odometer,
AccelLongiWheels,
AccelPedalPos,
LatAcceleration,
PerfMode,
LTurnSignal,
RTurnSignal,
HazardLightSts,
RRTireState,
RLTireState,
FRTireState,
FLTireState,
DrivingDirection,
YawSpeed,
TripTravelDist,
WheelTorqueSign,
AbsSteeringWheelAngleAcu,
InitSteeringWheenAngleFlag,
AccellLongCall,
BattStatusSOC,
FuelLvLow,
HVBatteryLvLow,
EngineSpeed,
TripElectricOnlyPercent,
JDAAlertsData,
JDAAlertsBlocID,
JDAAccessStatus,
JDATotalBlocNumber,
HVBattEstimatedChargeTime,
HVBatteryCurrent,
HVBatteryVoltage,
DriverDoorSts,
PassengerDoorSts,
VehicleBackRightDoorLockingStatus,
VehicleBackLeftDoorLockingStatus,
VehicleTrunkDoorLockingStatus,
CanCommonSignal_Length
};

    bool isAvailable() const;
	bool isAvailable_dotteProxy() const;
    
	
private:
    class VehicleClientImpl;
    std::unique_ptr<VehicleClientImpl> m_pImpl;

};

 class CAPIVehicleTCUServerGateway{
	public:
	CAPIVehicleTCUServerGateway();

   ~ CAPIVehicleTCUServerGateway();
   
   void init_TCU();
   void SendEvent_MsdTransmissionTime_TCU_To_IVI(uint32_t _MsdTransmissionTime_TCU);
   void SendEvent_EcallState_TCU_To_IVI(uint32_t _EcallState_TCU);
   void SendEvent_AcallState_TCU_To_IVI(uint32_t _AcallState_TCU);
   void SendEvent_UserXcallFeedback_TCU_To_IVI(uint32_t _UserXcallFeedback_TCU);
   void SendEvent_EcallDurationTimer_TCU_To_IVI(uint32_t _EcallDurationTimer_TCU);
   void SendEvent_EcallTpsState_TCU_To_IVI(uint32_t _EcallTpsState_TCU);
   void SendEvent_XcallStatus_TCU_To_IVI(uint32_t _XcallStatus_TCU);
   void SendEvent_EcallWcbTimer_TCU_To_IVI(uint32_t _EcallWcbTimer_TCU);
   void SendEvent_DmdMuteSual_TCU_To_IVI(uint32_t _DmdMuteSual);
   void SendEvent_AppUrgMds_TCU_To_IVI(uint32_t _AppUrgMds);
   void SendEvent_SignalVoyantDysfunction_TCU_To_IVI(uint32_t _SignalVoyantDysfunction);
   void SendEvent_SignalVoyantEtat_TCU_To_IVI(uint32_t _SignalVoyantEtat);
   void SendEvent_BoostrapRetryCount_TCU_To_IVI(uint32_t _BoostrapRetryCount);
   void SendEvent_CloudConnectionStatus_CConnState_connectionGateID_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_connectionGateID);
   void SendEvent_CloudConnectionStatus_CConnState_isConnected_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_isConnected);
   void SendEvent_CloudConnectionStatus_CConnState_connType_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_connType);
   void SendEvent_CloudConnectionStatus_CConnState_isRoaming_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_isRoaming);
   void SendEvent_CloudConnectionStatus_CConnState_sigStrength_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_sigStrength);
   void SendEvent_CloudConnectionStatus_CConnState_ipVersion_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnState_ipVersion);
   void SendEvent_CellularNetworkType_TCU_To_IVI(uint32_t _CellularNetworkType);
   void SendEvent_NetConnection_TCU_To_IVI(uint32_t _NetConnection);
   void SendEvent_Cellsignal_TCU_To_IVI(uint32_t _Cellsignal);
   void SendEvent_CloudConnectionStatus_CConnCtrl_reqDisable_TCU_To_IVI(uint32_t _CloudConnectionStatus_CConnCtrl_reqDisable);
   void SendEvent_SLIRoadType_TCU_To_IVI(uint32_t _SLIRoadType);
   void SendEvent_SLISpeedLimit_TCU_To_IVI(uint32_t _SLISpeedLimit);
   void SendEvent_SLICountryCode_TCU_To_IVI(uint32_t _SLICountryCode);
   void Send_Register_App_TCU_To_IVI(std::string appId, std::string appName);
   void Send_UnRegister_App_TCU_To_IVI();
   void Send_WriteSev_App_TCU_To_IVI(StSecurityEvent &security_event, StLogIssuer &log_issuer, StEventSrc &event_src, StAction &action);
   void Send_ExportSecurityEvents_TCU_To_IVI();
   void Send_Vehicle_Phase_TCU_To_IVI(std::string vehiclePhase);
   android::sp<vendor::securethings::hardware::ids::ids_client::V1_0::IIdsClient> pIdsClientService;
   android::sp<vendor::securethings::hardware::ids::ids_manager::V1_0::IIdsManager> pIdsManagerService;

   void SendEvent_ReceiveFOTA_Check_for_Updates_Result(std::string FOTA_Check_for_Updates_result);
   void SendEvent_ReceiveFOTA_Update_Available(std::string FOTA_Update_Available);
   void SendEvent_ReceiveFOTA_Conditions_not_Met(std::string FOTA_Conditions_not_Met);
   void SendEvent_ReceiveFOTA_Installation_Status(std::string FOTA_Installation_Status);
   void SendEvent_ReceiveFOTAWhatsNew_Details_Response(std::string FOTAWhatsNew_Details_Response);
   void SendEvent_ReceiveFOTAUpdate_History_Response(std::string FOTAUpdate_History_Response);
   void SendEvent_ReceiveFOTAPending_Updates_Response(std::string FOTAPending_Updates_Response);
   void SendEvent_ReceiveFOTA_Update_Finished(std::string FOTA_Update_Finished);
   void SendEvent_ReceiveFOTASchedule_Update_Response(std::string FOTASchedule_Update_Response);
   void SendEvent_UIN_TCUToHMI(std::string UIN_TCU_toHMI);
   void SendEvent_TCU_Version_Number_toHMI(std::string TCU_version_number);
   void SendEvent_HMI_Trigger_For_UIN(std::string HMI_Trigger_for_UIN); 
   void SendEvent_ReceiveAOSP_RBUA_Trigger_Response(std::string AOSPRBUA_Trigger_Response);
   void SendEvent_FOTAHMI_Info_Result(std::string FOTAHMI_Info_Result);
   void SendEvent_USBUpdate_Details(std::string USBUpdate_details);

   void getTCUData(uint32_t* p1);
   void getPackagepath(std::string& packagePath);
   void getFOTAHMICheckforUpdateResultsUpdated(std::string& CheckforUpdateResult);
   void getFOTAHMIUpdateAvailable(std::string& UpdateAvailable);
   void getFOTAHMIConditionsNotMet(std::string& ConditionsNotMet);
   void getFOTAHMIInstallationStatus(std::string& InstallationStatus);
   void getFOTAHMIUpdateFinished(std::string& UpdateFinished);
   void getFOTAHMIWhatsNewDetails(std::string& WhatsnewDetails);
   void getFOTAHMIUpdateHistory(std::string& UpdateHistory);
   void getFOTAHMIPendingUpdates(std::string& PendingUpdates);
   void getFOTAHMIScheduleUpdate(std::string& ScheduleUpdate);
   void getAOSPRBUATrigger(std::string& AOSPRBUATrigger);
   void getUsbUpdateDetails(std::string& USBUpdatedetails);
   void getFotaHmiInfoResult(std::string& FotaHmiInfoResult);
   void registerTCUSignalCallback(std::function<void(uint32_t [])> TCU_internalSignal);
   void registerSLIDataCallback(std::function<void(uint32_t [])> SLI_Data);
   void registerFlashUnitIdCallback(std::function<void(uint32_t)> Flash_UnitId);
   void registerPackagePathCallback(std::function<void(std::string)> package_filepath);
   void registerFotaHMICheckforResultsCallback(std::function<void(std::string)> m_FOTAHMI_CheckforResults_DATA);
   void registerFotaHMIUpdateAvailableCallback(std::function<void(std::string)> m_FOTAHMI_UpdateAvailable_DATA);
   void registerFotaHMIConditionsnotmetCallback(std::function<void(std::string)> m_FOTAHMI_ConditionsnotMet_DATA);
   void registerFotaHMIInstallationStatusCallback(std::function<void(std::string)> m_FOTAHMI_InstallationStatus_DATA);
   void registerFotaHMIWhatNewDetailsCallback(std::function<void(std::string)> m_FOTAWhatsNew_Details_Response);
   void registerFotaHMIUpdateHistoryResponseCallback(std::function<void(std::string)> m_FOTAUpdate_History_Response);
   void registerFotaHMIPendingUpdateHistoryCallback(std::function<void(std::string)> m_FOTAPending_Updates_Response);
   void registerFotaHMIUpdateFinishedCallback(std::function<void(std::string)> m_FOTA_Update_Finished);
   void registerFotaHMIUpdateScheduleCallback(std::function<void(std::string)> m_FOTASchedule_Update_Response);
   
   
   void registerUIN_TCUToHMICallback(std::function<void(std::string)> UIN_TCU_ToHMI_Data);
   void registerTCUVersiontoHMICallback(std::function<void(std::string)> TCU_version_to_HMI);
   void registerAOSPRBUATriggerCallback(std::function<void(std::string)> m_AOSPRBUATrigger);
   void registerFotaHmiInfoResultCallback(std::function<void(std::string)> m_FOTAHMI_Info_Result);
   void registerUSB_Update_detailsCallback(std::function<void(std::string)> m_USBUpdate_details);
   void SetUserXcall(uint32_t UserXcall_Data);
   void SetLanguage_Selection(uint32_t Language_Selection);
   void Send_Privacy_mode_sts(uint32_t privacy_mode);
   void SendEvent_HMI_Trigger_For_UIN(uint32_t HMI_Trigger_for_UIN);

   void Send_HMIFOTA_Update_history(std::string Update_History);
   void Send_HMIFOTA_pending_updates(std::string Pending_updates); 
   void Send_FOTA_WhatsNew_Details(std::string Whatsnewdetails);
   void Send_FOTA_Schedule_Update(std::string ScheduleUpdate);
   void Send_FOTA_Update_Dismissed(std::string UpdateDismissed);
   void Send_FOTA_Installation_Finished_with_User_Conformation(std::string Installation_finished);
   void Send_FOTA_Start_Installation(std::string start_installation);
   void Send_FOTA_Check_for_Updates(std::string checkfor_updates);
   void send_AOSP_Flash_Result(std::string AOSP_Flash_Result);
   void Send_HMI_Status_SWUpdate(uint32_t HMI_Status_SWUpdate);
   void Send_HMIFOTA_Info(std::string hmifota_info);


   void getSLIData(uint32_t* p1);
   void SetXcall_HMI_Control(uint32_t XCallHmiControl);
   void SetFlashUnitId(uint32_t FlashUnitId);
   void SetPackagePath(std::string PackagePath);
   void SetFlashResult(uint32_t FlashResult);
   void SetActive_phone_call_status(uint32_t active_phone_call_status);
   void SetBTCallHMI_Status(uint32_t BtCallHmi_status);
   void SetBTDeviceAttached_Status(uint32_t BtDeviceAttached_status);
   void SetBTTransferTime_Duration(uint32_t BtTransfertime_duration);
   void SetIncomingPhoneCall_Status(uint32_t IncomingPhoneCall_status);
   void SetOutgoingPhoneCall_Status(uint32_t OutgoingPhoneCall_status);
   void SetBTTransferAcceptance_Status(uint32_t BTTransferAcceptance_status);



   enum TCU_Internal_signals{
        MsdTransmissionTime = 0,
        EcallState,
        AcallState,
        UserXcallFeedback,
        EcallDurationTimer,
        EcallTpsState,
        XcallStatus,
        EcallWcbTimer,
		DmdMuteSual,
		AppUrgMds,
		VoyantDysfunction,
		VoyantEtat,
		BoostrapRetryCount,
		CloudConnectionStatus_CConnState_connectionGateID,
		CloudConnectionStatus_CConnState_isConnected,
		CloudConnectionStatus_CConnState_connType,
		CloudConnectionStatus_CConnState_isRoaming,
		CloudConnectionStatus_CConnState_sigStrength,
		CloudConnectionStatus_CConnState_ipVersion,
		CellularNetworkType,
		NetConnection,
		Cellsignal,
		CloudConnectionStatus_CConnCtrl_reqDisable,
		TCU_Internal_signals_Length

   };
	private:
    class VehicleClientImpl;
	std::unique_ptr<VehicleClientImpl> m_pImpl_TCU;
	uint32_t langSelected;

};

}
}
}
}
}
