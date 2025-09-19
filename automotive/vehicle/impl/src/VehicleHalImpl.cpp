#define LOG_TAG __FILE_NAME__
#include <log/log_main.h>
#include <android-base/logging.h>
#include "VehicleHalImpl.h"
#include <utils/SystemClock.h>
#include <memory>
#include <vhal_v2_0/VehicleUtils.h>
#include <iomanip>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <limits>
#include <ctime>
#include <thread>
#include "DefaultConfig.h"
#include "../../../../../../../external/automotive-source/dlt-daemon-2.18.10/include/dlt/dlt.h"
uint16_t InternalSystemTime[Internal_SystemTime_Index];
std::vector<uint16_t> IVI_TX_signals(IVI_TX_Signals_size,0);
uint16_t IVI_Secure_oldData[SECURE_OLDDATA]{0};
uint16_t IVI_HMI_oldData[HMI_OLDDATA]{0};
uint16_t IVI_EXTGPS_oldData[EXTGPS_OLDDATA]{0};
uint16_t IVI_BATTERYPRE_oldData[BATTERYPRECONDITION_OLDDATA]{0};
std::vector<uint16_t> old_tx_buffer_IVI(IVI_TX_Signals_size,0);
std::vector<uint32_t> default_projection(internal_projection_length,0);
std::vector<uint32_t>default_second_party_App_length(internal_second_party_App_length,0);
std::vector<uint32_t> default_audio_length(internal_audio_length,0);
std::vector<uint32_t>default_hmi_length(internal_hmi_length,0);
uint16_t default_SystemTime_Index[Internal_SystemTime_Index];
uint32_t default_INTERNALSignal_index[INTERNALSignal_index];
std::vector<uint32_t>default_camera_length(internal_camera_length,0);
extern DltContext SYIN;
extern DltContext STLA;

int previousSystemTimeZone = 13;
namespace {

constexpr std::chrono::nanoseconds hertzToNanoseconds(float hz)
{
    return std::chrono::nanoseconds(static_cast<int64_t>(1000000000L / hz));
}

} // namespace

namespace vendor {
namespace marelli {
namespace vehicle {
namespace V1_0 {

namespace impl {

VehicleHalImpl::VehicleHalImpl(vhal_v2_0::VehiclePropertyStore* propStore, VehicleHalClient* client)
    : mPropStore(propStore)
    , mVehicleClient(client)
    , mRecurrentTimer(
          std::bind(&VehicleHalImpl::onContinuousPropertyTimer, this, std::placeholders::_1)) 
	, m_gateway()
	, m_gateway_TCU()
{
    regPropsInPropStore();
    ALOGD("VehicleHalImpl : start");
    mVehicleClient->registerPropertyValueCallback(std::bind(
        &VehicleHalImpl::onPropertyValue, this, std::placeholders::_1, std::placeholders::_2));
    m_gateway.init();
	m_gateway_TCU.init_TCU();
    
	b_CANstream = true;
	b_Dottestream = true;
	b_DTCstream = true;
	b_DLTstream = true;
	b_powermode = true;
	b_SocVersion = true;
	m_powermode = 0;
	m_vinnumber = 0;
	b_vinnumber = true;
	b_vinNumbersubscribe = true;
	b_calDatasubscribe = true;
	b_internalSignal = true;
   	b_TCUsignalSignal = true;
    b_Temperature = true;
    b_supplyvge = true ;
    b_CallActiveSTRStatus = true;
    b_STRCancelShutdown = true;
    b_USB_update = true;
    b_USB_update_details =true;
	b_IMUstream = true;
	b_DIDstream = true;
    b_SLI_data = true;
	b_txSignal = true;
    
		m_gateway.registerSpeedCallback([this](uint32_t speed) {
        onPropertyValue(*createSpeedDataReq(speed),true);
		});
		m_gateway.registerVinNumberCallback([this](std::vector<uint32_t> vinNumber) {
        onPropertyValue(*createVinNumberDataReq(vinNumber),true);
		});
		m_gateway.registerIgnitionStsCallback([this](uint32_t IgnitionSts) {
        onPropertyValue(*createIgnitionStsDataReq(IgnitionSts),true);
		});
		m_gateway.registerBCMSECURtimeCallback([this](uint32_t bcmSECURtime[SECUREtime_index]) {
        onPropertyValue(*createBCMsecureTimeDataReq(bcmSECURtime),true);
		});
		m_gateway.registerDotteParamCallback([this](std::vector<uint32_t> dotteParam) {
        onPropertyValue(*createDotteParamDataReq(dotteParam),true);
		});
		m_gateway.registerBCMHMItimeCallback([this](uint32_t bcmHMItime[HMItime_index]) {
        onPropertyValue(*createBCMHMIDataReq(bcmHMItime),true);
		});
		m_gateway.registerInternalSignalCallback([this](uint32_t internalSignal[INTERNALSignal_index]) {
        onPropertyValue(*createinternalSignalDataReq(internalSignal),true);
		});
        m_gateway.registerEVSignalsCallback([this](int32_t EVSignal[EVSignals_index]) {
        onPropertyValue(*createEVSignalsDataReq(EVSignal),true);
		});
        m_gateway.registerCameraSignalsCallback([this](uint32_t CameraSignal[CameraSignals_index]) {
        onPropertyValue(*createCameraSignalsDataReq(CameraSignal),true);
		});
        m_gateway.registerBCMSignalsCallback([this](uint32_t BCMSignal[BCMSignals_index]) {
        onPropertyValue(*createBCMSignalsDataReq(BCMSignal),true);
		});
		m_gateway.registerDTCParamCallback([this](std::vector<uint32_t> dtcParam) {
        onPropertyValue(*createDTCParamDataReq(dtcParam),true);
		});
		m_gateway.registerDLTParamCallback([this](std::vector<uint32_t> dLTParam) {
        onPropertyValue(*createDLTParamDataReq(dLTParam),true);
		});
		m_gateway.registerDLT_VectorParamCallback([this](std::vector<uint8_t> dLT_VectorValueParam){
		onPropertyValue(*createDLT_VectorParamDataReq(dLT_VectorValueParam),true);
		});
		m_gateway.registerEcallCrashCallback([this](uint32_t EcallCrash) {
        onPropertyValue(*createEcallCrashDataReq(EcallCrash),true);
		});
		m_gateway.registerParkLampCallback([this](uint32_t ParkLamp) {
        onPropertyValue(*createParkLampDataReq(ParkLamp),true);
		});
		m_gateway.registerDayNightModeCallback([this](uint32_t DayNightMode) {
        onPropertyValue(*createDayNightModeDataReq(DayNightMode),true);
		});
		m_gateway.registerFuellevelCallback([this](uint32_t Fuellevel) {
        onPropertyValue(*createFuelLevalDataReq(Fuellevel),true);
		});
		m_gateway.registerFuelTypeCallback([this](uint32_t FuelType) {
        onPropertyValue(*createFuelTypeDataReq(FuelType),true);
		});
		m_gateway.registerParkBreakStatusCallback([this](uint32_t ParkBreakStatus) {
        onPropertyValue(*createParkBreakStatusDataReq(ParkBreakStatus),true);
		});
		m_gateway.registerOutTempCallback([this](uint32_t OutTemp) {
        onPropertyValue(*createoutTempDataReq(OutTemp),true);
		});
		m_gateway.registerHVBatterySOHDataCallback([this](uint32_t HVBatterySOHData) {
        onPropertyValue(*createHVBatterySOHDataReq(HVBatterySOHData),true);
		});
		m_gateway.registerHVBatteryPercentageCallback([this](uint32_t HVBatteryPercentage) {
        onPropertyValue(*createHVBatteryPercentageDataReq(HVBatteryPercentage),true);
		});
		m_gateway.registerHVBatteryMinChargeCallback([this](uint32_t HVBatteryMinCharge) {
        onPropertyValue(*createHVBatteryMinChargeDataReq(HVBatteryMinCharge),true);
		});
		m_gateway.registerHVBatteryActiveConnectorCallback([this](uint32_t HVBatteryActiveConnector) {
        onPropertyValue(*createHVBatteryActiveConnectorDataReq(HVBatteryActiveConnector),true);
		});
		m_gateway.registerHVBatteryMaxRangeCallback([this](uint32_t HVBatteryMaxRange) {
        onPropertyValue(*createHVBatteryMaxRangeDataReq(HVBatteryMaxRange),true);
		});
		m_gateway.registerHVBatteryChargeLevelCallback([this](uint32_t HVBatteryChargeLevel) {
        onPropertyValue(*createHVBatteryChargeLevelDataReq(HVBatteryChargeLevel),true);
		});
		m_gateway.registerHVBatteryIsChargingCallback([this](uint32_t HVBatteryIsCharging) {
        onPropertyValue(*createHVBatteryIsChargingDataReq(HVBatteryIsCharging),true);
		});
		m_gateway.registerAutonomyCallback([this](uint32_t Autonomy) {
        onPropertyValue(*createAutonomyDataReq(Autonomy),true);
		});
		m_gateway.registerBrakePedalrStsCallback([this](uint32_t BrakePedalrSts) {
        onPropertyValue(*createBrakePedalrStsDataReq(BrakePedalrSts),true);
		});
		m_gateway.registerCanDistanceUnitCallback([this](uint32_t CanDistanceUnit) {
        onPropertyValue(*createCanDistanceUnitDataReq(CanDistanceUnit),true);
		});
		m_gateway.registerInstantFuelConsCallback([this](uint32_t InstantFuelCons) {
        onPropertyValue(*createInstantFuelConsDataReq(InstantFuelCons),true);
		});
/* 		m_gateway.registerReverseGearEngagedConsCallback([this](uint32_t ReverseGearEngaged) {
        onPropertyValue(*createReverseGearEngagedDataReq(ReverseGearEngaged),true);
		});*/
		m_gateway.registerRoadSlopeCallback([this](uint32_t RoadSlope) {
        onPropertyValue(*createRoadSlopeDataReq(RoadSlope),true);
		});
		m_gateway.registerSteeringAngleCallback([this](uint32_t SteeringAngle) {
        onPropertyValue(*createSteeringAngleReq(SteeringAngle),true);
		});
		m_gateway.registerTotalKMCallback([this](uint32_t TotalKM) {
        onPropertyValue(*createTotalKMReq(TotalKM),true);
		});
		m_gateway.registerTransmGearDisplayCallback([this](uint32_t TransmGearDisplay) {
        onPropertyValue(*createTransmGearDisplayReq(TransmGearDisplay),true);
		});
		m_gateway.registerTripAvgBattPowerConsCallback([this](uint32_t TripAvgBattPowerCons) {
        onPropertyValue(*createTripAvgBattPowerConsReq(TripAvgBattPowerCons),true);
		});
	    m_gateway.registerYawSpeedCallback([this](uint32_t YawSpeed) {
        onPropertyValue(*createYawSpeedDataReq(YawSpeed),true);
		});
		m_gateway.registerGrossYawRateCallback([this](uint32_t GrossYawRate) {
        onPropertyValue(*createGrossYawRateDataReq(GrossYawRate),true);
		});
		m_gateway.registerFLWheelTickCounterFaultCallback([this](uint32_t FLWheelTickCounterFault) {
        onPropertyValue(*createFLWheelTickCounterFaultDataReq(FLWheelTickCounterFault),true);
		});
		m_gateway.registerFRWheelTickCounterFaultCallback([this](uint32_t FRWheelTickCounterFault) {
        onPropertyValue(*createFRWheelTickCounterFaultDataReq(FRWheelTickCounterFault),true);
		});
		m_gateway.registerRLWheelTickCounterFaultCallback([this](uint32_t RLWheelTickCounterFault) {
        onPropertyValue(*createRLWheelTickCounterFaultDataReq(RLWheelTickCounterFault),true);
		});
		m_gateway.registerRRWheelTickCounterFaultCallback([this](uint32_t RRWheelTickCounterFault) {
        onPropertyValue(*createRRFWheelTickCounterFaultDataReq(RRWheelTickCounterFault),true);
		});
		m_gateway.registerPulseCountFLWheelCallback([this](uint32_t PulseCountFLWheel) {
        onPropertyValue(*createPulseCountFLWheelDataReq(PulseCountFLWheel),true);
		});
		m_gateway.registerPulseCountFRWheelCallback([this](uint32_t PulseCountFRWheel) {
        onPropertyValue(*createPulseCountFRWheelDataReq(PulseCountFRWheel),true);
		});
		m_gateway.registerPulseCountRLWheelCallback([this](uint32_t PulseCountRLWheel) {
        onPropertyValue(*createPulseCountRLWheelDataReq(PulseCountRLWheel),true);
		});
		m_gateway.registerPulseCountRRWheelCallback([this](uint32_t PulseCountRRWheel) {
        onPropertyValue(*createPulseCountRRWheelDataReq(PulseCountRRWheel),true);
		});
		m_gateway.registerFPASStsCallback([this](uint32_t FPASSts) {
        onPropertyValue(*createFPASStsDataReq(FPASSts),true);
		});
		m_gateway.registerDrivingDirectionCallback([this](uint32_t DrivingDirection) {
        onPropertyValue(*createDrivingDirectionDataReq(DrivingDirection),true);
		});
	    m_gateway.registerCapacityUnitCallback([this](uint32_t CapacityUnit) {
        onPropertyValue(*createCapacityUnitDataReq(CapacityUnit),true);
		});
        m_gateway.registerCombustFuelLvCallback([this](uint32_t CombustFuelLv) {
        onPropertyValue(*createCombustFuelLvDataReq(CombustFuelLv),true);
		});
        m_gateway.registerCompressorStsCallback([this](uint32_t CompressorSts) {
        onPropertyValue(*createCompressorStsDataReq(CompressorSts),true);
		});
        m_gateway.registerElectricMotorPowerCallback([this](int32_t ElectricMotorPower) {
        onPropertyValue(*createElectricMotorPowerDataReq(ElectricMotorPower),true);
		});
        m_gateway.registerEngineCoolantTempCallback([this](uint32_t EngineCoolantTemp) {
        onPropertyValue(*createEngineCoolantTempDataReq(EngineCoolantTemp),true);
		});
        m_gateway.registerEngineRPMCallback([this](uint32_t EngineRPM) {
        onPropertyValue(*createEngineRPMDataReq(EngineRPM),true);
		});
        m_gateway.registerFuelLevelMinimumStsCallback([this](uint32_t FuelLevelMinimumSts) {
        onPropertyValue(*createFuelLevelMinimumStsDataReq(FuelLevelMinimumSts),true);
		});
        m_gateway.registerAbsSteeringWheelAngleAcuCallback([this](uint32_t AbsSteeringWheelAngleAcu) {
        onPropertyValue(*createAbsSteeringWheelAngleAcuDataReq(AbsSteeringWheelAngleAcu),true);
		});
        m_gateway.registerInitSteeringWheenAngleFlagCallback([this](uint32_t InitSteeringWheenAngleFlag) {
        onPropertyValue(*createInitSteeringWheenAngleFlagDataReq(InitSteeringWheenAngleFlag),true);
		});
        m_gateway.registerNetworkMNGTCallback([this](uint32_t NetworkMNGT) {
        onPropertyValue(*createNetworkMNGTDataReq(NetworkMNGT),true);
		});
        m_gateway.registerFPASCenterLeftBarStsCallback([this](uint32_t FPASCenterLeftBarSts) {
        onPropertyValue(*createFPASCenterLeftBarStsDataReq(FPASCenterLeftBarSts),true);
		});
        m_gateway.registerFPASCenterRightBarStsCallback([this](uint32_t FPASCenterRightBarSts) {
        onPropertyValue(*createFPASCenterRightBarStsDataReq(FPASCenterRightBarSts),true);
		});
        m_gateway.registerFPASLeftBarStsCallback([this](uint32_t FPASLeftBarSts) {
        onPropertyValue(*createFPASLeftBarStsDataReq(FPASLeftBarSts),true);
		});
        m_gateway.registerFPASRightBarStsCallback([this](uint32_t FPASRightBarSts) {
        onPropertyValue(*createFPASRightBarStsDataReq(FPASRightBarSts),true);
		});
        m_gateway.registerFPASRightBarStsCallback([this](uint32_t FPASRightBarSts) {
        onPropertyValue(*createFPASRightBarStsDataReq(FPASRightBarSts),true);
		});
        m_gateway.registerRPASCenterLeftBarStsCallback([this](uint32_t RPASCenterLeftBarSts) {
        onPropertyValue(*createRPASCenterLeftBarStsDataReq(RPASCenterLeftBarSts),true);
		});
        m_gateway.registerRPASCenterRightBarStsCallback([this](uint32_t RPASCenterRightBarSts) {
        onPropertyValue(*createRPASCenterRightBarStsDataReq(RPASCenterRightBarSts),true);
		});
        m_gateway.registerRPASLeftBarStsCallback([this](uint32_t RPASLeftBarSts) {
        onPropertyValue(*createRPASLeftBarStsDataReq(RPASLeftBarSts),true);
		});
        m_gateway.registerTrailerPresentCallback([this](uint32_t TrailerPresent) {
        onPropertyValue(*createTrailerPresentDataReq(TrailerPresent),true);
		});
        m_gateway.registerTrunkDoorStsCallback([this](uint32_t TrunkDoorSts) {
        onPropertyValue(*createTrunkDoorStsDataReq(TrunkDoorSts),true);
		});
        m_gateway.registerRPASStsCallback([this](uint32_t RPASSts) {
        onPropertyValue(*createRPASStsDataReq(RPASSts),true);
		});
        m_gateway.registerEconShftRqCallback([this](uint32_t EconShftRq) {
        onPropertyValue(*createEconShftRqDataReq(EconShftRq),true);
		}); 
        m_gateway.registerEngineTorqueCallback([this](uint32_t EngineTorque) {
        onPropertyValue(*createEngineTorqueDataReq(EngineTorque),true);
		}); 
        m_gateway.registerFuelConsUnitCallback([this](uint32_t FuelConsUnit) {
        onPropertyValue(*createFuelConsUnitDataReq(FuelConsUnit),true);
		}); 
        m_gateway.registerRegeneratedEnergyCallback([this](uint32_t RegeneratedEnergy) {
        onPropertyValue(*createRegeneratedEnergyDataReq(RegeneratedEnergy),true);
		}); 
        m_gateway.registerShiftLeverPositionCallback([this](uint32_t ShiftLeverPosition) {
        onPropertyValue(*createShiftLeverPositionDataReq(ShiftLeverPosition),true);
		}); 
        m_gateway.registerTripAverageFuelConsumptionCallback([this](uint32_t TripAverageFuelConsumption) {
        onPropertyValue(*createTripAverageFuelConsumptionDataReq(TripAverageFuelConsumption),true);
		}); 
        m_gateway.registerReqVolCtrlCallback([this](uint32_t ReqVolCtrl) {
        onPropertyValue(*createReqVolCtrlDataReq(ReqVolCtrl),true);
		});
		m_gateway.registerCalibrationDataCallback([this](uint32_t calData[]) {
        onPropertyValue(*createCALDataReq(calData),true);
		});	
       /*  m_gateway.registercalibrationDataforSrecCallback([this](std::vector<uint32_t> calSrecDATA) {
        onPropertyValue(*createCALSrecDataReq(calSrecDATA),true);
        }); */
		m_gateway.registerVehPrivacyModeCallback([this](uint32_t VehPrivacyMode) {
        onPropertyValue(*createVehPrivacyModeDataReq(VehPrivacyMode),true);
		});
        m_gateway.registerTemperatureCallback([this](uint32_t Temperature) {
        onPropertyValue(*createTemperatureDataReq(Temperature),true);
		});	
        m_gateway.registerSupplyVoltageCallback([this](uint32_t VoltageData) {
        onPropertyValue(*createSupplyVoltageDataReq(VoltageData),true);
		});
        m_gateway.registerCallActiveSTRDisplayPopupStatusCallback([this](uint32_t callActiveSTRDisplayPopupStatus) {
        onPropertyValue(*createCallActiveSTRDisplayPopupStatusDataReq(callActiveSTRDisplayPopupStatus),true);
        });
        m_gateway.registerSTRCancelShutdownStatusCallback([this] (uint32_t STRCancelShutdownStatus){
            onPropertyValue(*createSTRCancelShutdownStatusDataReq(STRCancelShutdownStatus),true);
        });
        m_gateway.registerCanCommonSignalsCallback([this](int32_t CanCommonSignal[CanCommonSignal_index]) {
        onPropertyValue(*createCanCommonDataReq(CanCommonSignal),true);
		});
        m_gateway_TCU.registerUSB_Update_detailsCallback([this](std::string m_USBUpdate_Details) {
        onPropertyValue(*createUSB_Update_DetailsSignalsDataReq(m_USBUpdate_Details),true);
		});
        m_gateway.registerUSB_UpdateCallback([this](std::vector<uint16_t> USB_Update) {
        onPropertyValue(*createIMC_MCUUSB_UpdateSignalsDataReq(USB_Update),true);
		});
		m_gateway.registerCalibrationDataCallback_Audio([this](uint32_t calData[]) {
        onPropertyValue(*createCALDataReq_Audio(calData),true);
		});
		m_gateway.registerCalibrationDataCallback_Camera([this](uint32_t calData[]) {
        onPropertyValue(*createCALDataReq_Camera(calData),true);
		});
		m_gateway.registerCalibrationDataCallback_Array([this](std::vector<uint8_t> calData) {
        onPropertyValue(*createCALDataReq_Array(calData),true);
		});
		m_gateway.registerIMUdata_callback([this](std::vector<int32_t> imuData) {
        onPropertyValue(*createIMUDataReq(imuData),true);
		});
		m_gateway.registerDID_RID_callback([this](std::vector<uint16_t> m_dID_message_type) {
        onPropertyValue(*createDIDRIDDataReq(m_dID_message_type),true);
		});
        m_gateway_TCU.registerTCUSignalCallback([this](uint32_t TCU_internalSignal[TCU_InternalSignals_Index]) {
        onPropertyValue(*createInternalDataReqFrom_TCU(TCU_internalSignal),true);
		});
		m_gateway.registerPowermodeCallback([this](uint32_t powermode) {
        onPropertyValue(*createPowermodeDataReq(powermode),true);
        });
		m_gateway.registerCAN_RX_SignalsCallback([this](uint32_t CAN_RX_Signals[CAN_RX_Signal_Index]) {
        onPropertyValue(*createCAN_RX_DataReq(CAN_RX_Signals),true);
        });
		m_gateway.registerDEM_DTCstatuscallback([this](std::vector<uint16_t> m_DEM_DTC_message_type) {
        onPropertyValue(*createDEM_DTCStatusDataReq(m_DEM_DTC_message_type),true);
		});
		m_gateway.register_DriverdoorStatuscallback([this](uint8_t driverdoorStatus) {
        onPropertyValue(*createDriverdoorStatusDataReq(driverdoorStatus),true);
		});
		m_gateway.register_PassengerDoorStatuscallback([this](uint8_t passengerDoorStatus) {
        onPropertyValue(*createPassengerDoorStatusDataReq(passengerDoorStatus),true);
		});
        m_gateway.registerSigned_CAN_RX_SignalsCallback([this](int32_t Signed_CAN_RX_Signals[Signed_CAN_RX_signals_Index]) {
        onPropertyValue(*createSigned_CAN_RX_DataReq(Signed_CAN_RX_Signals),true);
        });
        m_gateway.registerLog_RetriverCallback([this](uint32_t Log_Retriver_data) {
        onPropertyValue(*createLog_retriver_DataReq(Log_Retriver_data),true);
		});
        m_gateway.registerlogerfilepathCallback([this](std::string logger_filepath) {
        onPropertyValue(*createLoggerfilepathDataReq(logger_filepath),true);
		});
        m_gateway_TCU.registerSLIDataCallback([this](uint32_t SLI_Data[SLI_data_length]) {
        onPropertyValue(*createSLIDataReqFrom_TCU(SLI_Data),true);
		});
        m_gateway.registerLog_RetriverStatusCallback([this](uint32_t Log_Retriver_status) {
        onPropertyValue(*createLog_retriver_statusDataReq(Log_Retriver_status),true);
		});
        m_gateway.registerSOC_WakeupReasonCallback([this](uint32_t WakeUpReason) {
        onPropertyValue(*createWakeUpReasonDataReq(WakeUpReason),true);
		});
		        m_gateway_TCU.registerFlashUnitIdCallback([this](uint32_t FlashUnitId){
        onPropertyValue(*createFlashUnitIdDataReq(FlashUnitId), true);
        });
        m_gateway_TCU.registerPackagePathCallback([this](std::string package_filepath) {
        onPropertyValue(*createPackagePathDataReq(package_filepath),true);
		});
        m_gateway.registerRXSignals_FloatCallback([this](float RXSignals_Float[]) {
        onPropertyValue(*createRXSignals_FloatDataReq(RXSignals_Float),true);
		});
        m_gateway.registerMarelliPNCallback([this](std::vector<uint8_t> MarelliPN){
		onPropertyValue(*createMarelliPNDataReq(MarelliPN),true);
		});
        m_gateway.registerEOLHWIDCallback([this](std::vector<uint8_t> EOLHWID){
		onPropertyValue(*createEOLHWIDDataReq(EOLHWID),true);
		});
        m_gateway.registerAuthenticationZoneCallback([this](std::vector<uint8_t> AuthenticationZone){
		onPropertyValue(*createAuthenticationZoneDataReq(AuthenticationZone),true);
		});
        m_gateway.registerApplicationSoftwareIdentificationCallback([this](std::vector<uint8_t> ApplicationSoftwareIdentification){
		onPropertyValue(*createApplicationSoftwareIdentificationDataReq(ApplicationSoftwareIdentification),true);
		});
        m_gateway.registerApplicationDataIdentificationCallback([this](std::vector<uint8_t> ApplicationDataIdentification){
		onPropertyValue(*createApplicationDataIdentificationDataReq(ApplicationDataIdentification),true);
		});
        m_gateway.registerDataLibraryidentifierCallback([this](std::vector<uint8_t> DataLibraryidentifier){
		onPropertyValue(*createDataLibraryidentifierDataReq(DataLibraryidentifier),true);
		});
        m_gateway.registerVehicleAppsIdentifierCallback([this](std::vector<uint8_t> VehicleAppsIdentifier){
		onPropertyValue(*createVehicleAppsIdentifierDataReq(VehicleAppsIdentifier),true);
		});
        m_gateway_TCU.registerFotaHMICheckforResultsCallback([this](std::string m_FOTAHMI_CheckforResults_DATA){
		LOG(INFO) << "VehicleHalImpl::registerFotaHMICheckforResultsCallback()";
        onPropertyValue(*createFotahmiCheckforResultsDataReq(m_FOTAHMI_CheckforResults_DATA),true);
        });
        m_gateway_TCU.registerFotaHMIUpdateAvailableCallback([this](std::string m_FOTAHMI_UpdateAvailable_DATA){
        onPropertyValue(*createFotaHmiUpdateAvailableDataReq(m_FOTAHMI_UpdateAvailable_DATA),true);
        });
        m_gateway_TCU.registerFotaHMIConditionsnotmetCallback([this](std::string m_FOTAHMI_ConditionsnotMet_DATA){
        onPropertyValue(*createFotaHmiConditionsnotmetDataReq(m_FOTAHMI_ConditionsnotMet_DATA),true); 
        });
        m_gateway_TCU.registerFotaHMIInstallationStatusCallback([this](std::string m_FOTAHMI_InstallationStatus_DATA){
        onPropertyValue(*createFotaHmiInstallationStatusDatareq(m_FOTAHMI_InstallationStatus_DATA),true);
        });
        m_gateway_TCU.registerFotaHMIWhatNewDetailsCallback([this](std::string m_FOTAWhatsNew_Details_Response){
        onPropertyValue(*createFotaHmiWhatNewDetailsDataReq(m_FOTAWhatsNew_Details_Response),true); 
        });
        m_gateway_TCU.registerFotaHMIUpdateHistoryResponseCallback([this](std::string m_FOTAUpdate_History_Response){
        onPropertyValue(*createFotaHmiUpdateHistoryResponseDataReq(m_FOTAUpdate_History_Response),true); 
        });  
        m_gateway_TCU.registerFotaHMIPendingUpdateHistoryCallback([this](std::string m_FOTAPending_Updates_Response){
        onPropertyValue(*createFotaHmiPendingUpdateHistoryDataReq(m_FOTAPending_Updates_Response),true); 
        });
        m_gateway_TCU.registerFotaHMIUpdateFinishedCallback([this](std::string m_FOTA_Update_Finished){
        onPropertyValue(*createFotaHmiUpdateFinishedDataReq(m_FOTA_Update_Finished),true); 
        });
        m_gateway_TCU.registerFotaHMIUpdateScheduleCallback([this](std::string m_FOTASchedule_Update_Response){
        onPropertyValue(*createFotaHmiUpdateScheduleDataReq(m_FOTASchedule_Update_Response),true); 
        });

        m_gateway_TCU.registerUIN_TCUToHMICallback ([this](std::string UIN_TCU_ToHMI_Data) {
        onPropertyValue(*createUIN_TCUToHMIDataReq(UIN_TCU_ToHMI_Data),true);
        });
        m_gateway_TCU.registerTCUVersiontoHMICallback ([this](std::string TCU_version_to_HMI) {
        onPropertyValue(*createTCUVersiontoHMIDataReq(TCU_version_to_HMI),true);
        });
        m_gateway.registerDCSD_DIAG_ReqCallback([this](std::vector<uint8_t> DCSD_DIAG_Req){
		onDCSD_DIAG_Reqdata(DCSD_DIAG_Req);
		});
        m_gateway_TCU.registerAOSPRBUATriggerCallback([this](std::string m_AOSPRBUATrigger){
        onPropertyValue(*createAOSPRBUATriggerDataReq(m_AOSPRBUATrigger),true); 
        });
        m_gateway_TCU.registerFotaHmiInfoResultCallback([this](std::string m_FOTAHMI_Info_Result){
        onPropertyValue(*createFotaHmiInfoResultDataReq(m_FOTAHMI_Info_Result),true); 
        });
}

void VehicleHalImpl::onCreate()
{
    ALOGD("VehicleHalImpl : onCreat()");
    initPropsInPropStore();
    m_gateway.init();
	m_gateway_TCU.init_TCU();
}

std::vector<vhal_v2_0::VehiclePropConfig> VehicleHalImpl::listProperties()
{
    ALOGD("VehicleHalImpl : listProperties()");
    return mPropStore->getAllConfigs();
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::get(
    const vhal_v2_0::VehiclePropValue& requestedPropValue,
    vhal_v2_0::StatusCode* outStatus)
{
    VehiclePropValuePtr v = nullptr;

    auto internalPropValue = mPropStore->readValueOrNull(requestedPropValue);

    if (internalPropValue != nullptr)
    {
        v = getValuePool()->obtain(*internalPropValue);
    }

    *outStatus = (v != nullptr ? vhal_v2_0::StatusCode::OK : vhal_v2_0::StatusCode::INVALID_ARG);

    auto propId = requestedPropValue.prop;
    ALOGD("%s(): VehicleHalImpl : get()", __func__);
    switch (propId) {
        case (int)vhal_v2_0::VehicleProperty::PERF_VEHICLE_SPEED:
        {}
        break;
		case (int)vhal_v2_0::VehicleProperty::FUEL_LEVEL:
        {}
        break;
        case (int)vhal_v2_0::VehicleProperty::AP_POWER_STATE_REQ:
		{
            LOG(INFO) << "VehicleHalImpl::get() vhal_v2_0::VehicleProperty::AP_POWER_STATE_REQ";			
		}
        break;
		case (int)vhal_v2_0::VehicleProperty::AP_POWER_STATE_REPORT:
		{
            LOG(INFO) << "VehicleHalImpl::get() vhal_v2_0::VehicleProperty::AP_POWER_STATE_REPORT";		
		}
        break;
		case (int)vhal_v2_0::VehicleProperty::DISPLAY_BRIGHTNESS:
		{
			ALOGD("VehicleHalImpl:: get  display brightness");
            onPropertyValue(requestedPropValue, true);			
		}
        break;
		case (int)vhalCustom_v1_0::VehiclePropertyCustom::NODEHMITIME_VENDOR:
		{
			ALOGD("VehicleHalImpl:: get  HMI Time");
            onPropertyValue(requestedPropValue, true);			
		}
        break;
        case (int)vhalCustom_v1_0::VehiclePropertyCustom::NODESECURETIME_VENDOR:
		{
			ALOGD("VehicleHalImpl:: get  REFERENCE_TIME_VENDOR Time");
            onPropertyValue(requestedPropValue, true);			
		}
        break;
        case (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_CAMERA_VENDOR:
		{
			ALOGD("VehicleHalImpl:: get  IVI_INTERNAL_CAMERA_VENDOR");
            onPropertyValue(requestedPropValue, true);			
		}
        break;
        case (int)vhalCustom_v1_0::VehiclePropertyCustom::DISPLAY_STATUS_VENDOR:
		{
			ALOGD("VehicleHalImpl:: get  Display Status :");
            onPropertyValue(requestedPropValue, true);			
		}
        case (int)vhalCustom_v1_0::VehiclePropertyCustom::DOTTEPARAM_VENDOR:
		{
			ALOGD("VehicleHalImpl:: get  DOTTEPARAM_VENDOR :");
            onPropertyValue(requestedPropValue, true);			
		}
        break;
    

        default:
        {
             
            ALOGD("VehicleHalImpl:: propId : %d : not handled",propId);
        }
        break;
    }

    return v;
}

vhal_v2_0::StatusCode VehicleHalImpl::set(const vhal_v2_0::VehiclePropValue& propValue)
{
	
    ALOGD("VehicleHalImpl : set()");
    vhal_v2_0::StatusCode ret = vhal_v2_0::StatusCode::OK;

    auto currentPropValue = mPropStore->readValueOrNull(propValue);

    if (currentPropValue == nullptr)
    {
        ret = vhal_v2_0::StatusCode::INVALID_ARG;
		ALOGD("VehicleHalImpl : set()---***NULL");
    }
    else if (currentPropValue->status != vhal_v2_0::VehiclePropertyStatus::AVAILABLE)
    {
		ALOGD("VehicleHalImpl : set()---***NOT_AVAILABLE");
        return vhal_v2_0::StatusCode::NOT_AVAILABLE;
    }
    else
    {
		auto propId = propValue.prop;
		ALOGD("VehicleHalImpl : set()  PropertyID : %d" , propId);
		mPropStore->writeValue(propValue, true);
        ret = mVehicleClient->setProperty(propValue, true);			
        if (ret == vhal_v2_0::StatusCode::OK) 
		{
			LOG(INFO) << "VHAL VehicleHalImpl::set() vhal_v2_0::StatusCode::OK";
            if(propId == (int)vhal_v2_0::VehicleProperty::AP_POWER_STATE_REQ)
			{		
                LOG(INFO) << "VHAL VehicleHalImpl::set() vhal_v2_0::VehicleProperty::AP_POWER_STATE_REQ";
		    }

            if(propId == (int)vhal_v2_0::VehicleProperty::AP_POWER_STATE_REPORT)
			{		
                LOG(INFO) << "VehicleHalImpl::set() vhal_v2_0::VehicleProperty::AP_POWER_STATE_REPORT";
                switch (propValue.value.int32Values[0]) {

                    case toInt(vhal_v2_0::VehicleApPowerStateReport::DEEP_SLEEP_EXIT):
                         LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::DEEP_SLEEP_EXIT";                       
                         onPropertyValue(*createApPowerStateReq(vhal_v2_0::VehicleApPowerStateReq::ON, 0), true);
                         break;

                    case toInt(vhal_v2_0::VehicleApPowerStateReport::SHUTDOWN_CANCELLED):
                         LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::SHUTDOWN_CANCELLED";
                         //LVDS into wake state
                         if(handleLVDSWakeupAndSleepState == true){
                             outputForLVDSWakeupAndSleepState = LVDSMgr_SetProperty(LVDSMgr_MODE_SLEEP, false);
                             LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::SHUTDOWN_CANCELLED LVDSMgr_SetProperty() for wake up outputForLVDSWakeupAndSleepState = " << outputForLVDSWakeupAndSleepState;    
                             handleLVDSWakeupAndSleepState = false;  
                             outputForLVDSWakeupAndSleepState = 1; 
                         }
                         onPropertyValue(*createApPowerStateReq(vhal_v2_0::VehicleApPowerStateReq::ON, 0), true);
                         break;

                    case toInt(vhal_v2_0::VehicleApPowerStateReport::WAIT_FOR_VHAL):
                         LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::WAIT_FOR_VHAL";
                         onPropertyValue(*createApPowerStateReq(vhal_v2_0::VehicleApPowerStateReq::ON, 0), true);
                         break;

                    case toInt(vhal_v2_0::VehicleApPowerStateReport::SHUTDOWN_POSTPONE):
                         LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::SHUTDOWN_POSTPONE";
                         pollForSTRCancelShutdown();
                         if( (cancelShutdownSTRInputFromMCU == cancelShutdownSTR) || (flagForPowerModeToCancelSTR == true) ){
                            LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::SHUTDOWN_POSTPONE cancelShutdownSTRInputFromMCU value = " << cancelShutdownSTRInputFromMCU;
                            LOG(INFO) << "VehicleHalImpl::set() triggering VehicleApPowerStateReport::CANCEL_SHUTDOWN";
                            LOG(INFO) << "VehicleHalImpl::set() flagForPowerModeToCancelSTR value at AP_POWER_STATE_REPORT SHUTDOWN_POSTPONE = "
                                   << (flagForPowerModeToCancelSTR ? "true" : "false");
                            powerModeControlDuringSTR = false;
                            LOG(INFO) << "VehicleHalImpl::set() resetting powerModeControlDuringSTR value at AP_POWER_STATE_REPORT SHUTDOWN_POSTPONE = "
                                   << (powerModeControlDuringSTR ? "true" : "false");
                            flagForPowerModeToCancelSTR = false;
                            LOG(INFO) << "VehicleHalImpl::set() resetting flagForPowerModeToCancelSTR value at AP_POWER_STATE_REPORT SHUTDOWN_POSTPONE = "
                                   << (flagForPowerModeToCancelSTR ? "true" : "false");
                            onPropertyValue(*createApPowerStateReq(vhal_v2_0::VehicleApPowerStateReq::CANCEL_SHUTDOWN, 2), true);
							handleDisplayUpInterruptDuringSTR = true;
                         }
                         else{
                            LOG(INFO) << "VehicleHalImpl::set() not cosidering the trigger VehicleApPowerStateReport::CANCEL_SHUTDOWN from SHUTDOWN_POSTPONE";
                         }              
                         break;

                    case toInt(vhal_v2_0::VehicleApPowerStateReport::SHUTDOWN_PREPARE):
                         LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::SHUTDOWN_PREPARE";
                         pollForSTRCancelShutdown();
                         if( (cancelShutdownSTRInputFromMCU == cancelShutdownSTR) || (flagForPowerModeToCancelSTR == true) ){
                            LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::SHUTDOWN_PREPARE cancelShutdownSTRInputFromMCU value = " << cancelShutdownSTRInputFromMCU;
                            LOG(INFO) << "VehicleHalImpl::set() triggering VehicleApPowerStateReport::CANCEL_SHUTDOWN";
                            LOG(INFO) << "VehicleHalImpl::set()  flagForPowerModeToCancelSTR value at AP_POWER_STATE_REPORT SHUTDOWN_PREPARE = "
                                   << (flagForPowerModeToCancelSTR ? "true" : "false");
                            flagForPowerModeToCancelSTR = false;
                            LOG(INFO) << "VehicleHalImpl::set() resetting flagForPowerModeToCancelSTR value at AP_POWER_STATE_REPORT SHUTDOWN_PREPARE = "
                                   << (flagForPowerModeToCancelSTR ? "true" : "false");
                            powerModeControlDuringSTR = false;
                            LOG(INFO) << "VehicleHalImpl::set() resetting powerModeControlDuringSTR value at AP_POWER_STATE_REPORT SHUTDOWN_PREPARE = "
                                   << (powerModeControlDuringSTR ? "true" : "false");
                            onPropertyValue(*createApPowerStateReq(vhal_v2_0::VehicleApPowerStateReq::CANCEL_SHUTDOWN, 2), true);
							handleDisplayUpInterruptDuringSTR = true;
                         }
                         else{
                            LOG(INFO) << "VehicleHalImpl::set() not cosidering the trigger VehicleApPowerStateReport::CANCEL_SHUTDOWN from SHUTDOWN_PREPARE";
                         }
                         break;

                    case toInt(vhal_v2_0::VehicleApPowerStateReport::ON):
                         LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::ON";
                         powerModeControlDuringSTR = false;
                         LOG(INFO) << "VehicleHalImpl::set() resetting powerModeControlDuringSTR value at AP_POWER_STATE_REPORT ON = "
                                   << (powerModeControlDuringSTR ? "true" : "false");     
                        flagForPowerModeToCancelSTR = false;
                         LOG(INFO) << "VehicleHalImpl::set() resetting flagForPowerModeToCancelSTR value at AP_POWER_STATE_REPORT ON = "
                                   << (flagForPowerModeToCancelSTR ? "true" : "false");  
                         if((m_powermode == powerModeSix) && (handleDisplayUpInterruptDuringSTR == false)){
                           onPropertyValue(*createPowermodeDataReq(powerModeSix),true);
                         }
                         if((m_powermode == powerModeNine) && (handleDisplayUpInterruptDuringSTR == false)){
                           onPropertyValue(*createPowermodeDataReq(powerModeNine),true);
                         }                         
                         if((m_powermode == powerModeEight) && (handleDisplayUpInterruptDuringSTR == false)){
                           onPropertyValue(*createPowermodeDataReq(powerModeEight),true);
                         }                         
                         if((m_powermode == powerModeEleven) && (handleDisplayUpInterruptDuringSTR == false)){
                           onPropertyValue(*createPowermodeDataReq(powerModeEleven),true);
                         }
                         if((m_powermode == powerModeTwelve) && (handleDisplayUpInterruptDuringSTR == false)){
                           onPropertyValue(*createPowermodeDataReq(powerModeTwelve),true);
                         }
                         if((m_powermode == powerModeThirteen) && (handleDisplayUpInterruptDuringSTR == false)){
                           onPropertyValue(*createPowermodeDataReq(powerModeThirteen),true);
                         }
                         if((m_powermode == powerModeNineteen) && (handleDisplayUpInterruptDuringSTR == false)){
                           onPropertyValue(*createPowermodeDataReq(powerModeNineteen),true);
                         }
			 if(flagForSchedulingTable == true){
			   LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::ON Start scheduling table";
			   system("echo 1 > /sys/devices/platform/soc/4ac0000.qcom,qupv3_0_geni_se/4a8c000.qcom,qup_uart/scheduleTableSwitch");
			   flagForSchedulingTable = false;
			 }
                         break;

                    case toInt(vhal_v2_0::VehicleApPowerStateReport::DEEP_SLEEP_ENTRY):
                         LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::DEEP_SLEEP_ENTRY";
                         onPropertyValue(*createApPowerStateReq(vhal_v2_0::VehicleApPowerStateReq::FINISHED, 3), true);
                         break;

                    case toInt(vhal_v2_0::VehicleApPowerStateReport::SHUTDOWN_START):
                         LOG(INFO) << "VehicleHalImpl::set() VehicleApPowerStateReport::SHUTDOWN_START";
                         break;

                    default:
                         LOG(INFO) << "VehicleHalImpl::set() vhal_v2_0::VehicleProperty::AP_POWER_STATE_REPORT default";
						 break;
			    }
            }

            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::BRIGHTNESS_VENDOR){
                LOG(INFO) << "VehicleHalImpl::set() propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::BRIGHTNESS_VENDOR";
                LOG(INFO) << "VehicleHalImpl::set() BRIGHTNESS_VENDOR thread id = "<< std::this_thread::get_id();
                propertyId = std::to_string(propId);
                brightness = propValue.value.int32Values[0];
                LVDSLibrarySharedPtr->setBrightness(brightness, propertyId);                    
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::DISPLAY_STATUS_VENDOR)
            {
                LOG(INFO) << "VehicleHalImpl::set() propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::DISPLAY_STATUS_VENDOR";
                LOG(INFO) << "VehicleHalImpl::set() DISPLAY_STATUS_VENDOR thread id = "<< std::this_thread::get_id();
                propertyId = std::to_string(propId);
                displayStatus = propValue.value.int32Values[0];
                //LVDS into wake state
                if((displayStatus == inputOneFromUpperLayer) && (handleLVDSWakeupAndSleepState == true)){
                    outputForLVDSWakeupAndSleepState = LVDSMgr_SetProperty(LVDSMgr_MODE_SLEEP, false);
                    LOG(INFO) << "VehicleHalImpl::set() VehiclePropertyCustom::DISPLAY_STATUS_VENDOR LVDSMgr_SetProperty() for wake up outputForLVDSWakeupAndSleepState = " << outputForLVDSWakeupAndSleepState;    
                    handleLVDSWakeupAndSleepState = false;  
                    outputForLVDSWakeupAndSleepState = 1; 
                }

                LVDSLibrarySharedPtr->displayManager(displayStatus, propertyId);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::DISPLAY_SCREEN_STATUS_VENDOR)
            {
                LOG(INFO) << "VehicleHalImpl::set() propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::DISPLAY_SCREEN_STATUS_VENDOR";
                LOG(INFO) << "VehicleHalImpl::set() DISPLAY_SCREEN_STATUS_VENDOR thread id = "<< std::this_thread::get_id();
                propertyId = std::to_string(propId);
                displayStatus = propValue.value.int32Values[0];
                LVDSLibrarySharedPtr->displayManager(displayStatus, propertyId);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_CAMERA_VENDOR)
            {
                for(unsigned i= 0 ; i<internal_camera_length ; i++)
                {
                    camera_data.push_back(propValue.value.int32Values[i]);
                    ALOGD("VehicleHalImpl::camera_data: %d %d" , i,camera_data[i]); 
                }
                 createInternalCameraSignalsDataReq(camera_data);
                 camera_data.clear();
				

            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_AUDIO_VENDOR)
            {
                for(unsigned i= 0 ; i<internal_audio_length ; i++)
                {
                    audio_data.push_back(propValue.value.int32Values[i]);
                    ALOGD("VehicleHalImpl::audio_data: %d %d" , i,audio_data[i]); 
                } 
                createInternalAudioSignalsDataReq(audio_data);
                audio_data.clear();
				

            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_HMI_VENDOR)
            {
                for(unsigned i= 0 ; i<internal_hmi_length ; i++)
                {
                    hmi_data.push_back(propValue.value.int32Values[i]);
                    ALOGD("VehicleHalImpl::hmi_data: %d %d" , i,hmi_data[i]);
                }
                createInternalHmiSignalsDataReq(hmi_data);
                hmi_data.clear();              
 
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_PROJECTION_VENDOR)
			{
				std::vector<uint32_t> vec_val;
                int size = sizeof(propValue.value.int32Values)/sizeof(propValue.value.int32Values[0]);
                for(int i = 0; i < internal_projection_length; i++){
				    vec_val.push_back(propValue.value.int32Values[i]);
                }
                for(auto &data : vec_val){
			        ALOGD("VHAL VehicleHalImpl::IVI_INTERNAL_PROJECTION_VENDOR  %d" , data);
                }
				createInternalProjectionDataReq(vec_val);
                m_gateway_TCU.SetActive_phone_call_status(vec_val[0]) ;
                m_gateway_TCU.SetBTCallHMI_Status(vec_val[7]) ;
                m_gateway_TCU.SetBTDeviceAttached_Status(vec_val[8]) ;
                m_gateway_TCU.SetBTTransferTime_Duration(vec_val[9]) ;
                m_gateway_TCU.SetIncomingPhoneCall_Status(vec_val[1]);
                m_gateway_TCU.SetOutgoingPhoneCall_Status(vec_val[2]);
                m_gateway_TCU.SetBTTransferAcceptance_Status(vec_val[6]);

                vec_val.clear();
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_CONNECTIVITY_VENDOR)
			{
				std::vector<uint32_t> vec_val;
                int size = sizeof(propValue.value.int32Values)/sizeof(propValue.value.int32Values[0]);
                for(int i = 0; i < size; i++){
				    vec_val.push_back(propValue.value.int32Values[i]);
                }
                for(auto &data : vec_val){
			        ALOGD("VHAL VehicleHalImpl::IVI_INTERNAL_CONNECTIVITY_VENDOR  %d" , data);
                }
				createInternalConnectivityDataReq(vec_val);
                vec_val.clear();
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_SECOND_PARTY_APP_VENDOR)
            {
                std::vector<uint32_t> second_party_app_data;
                uint32_t array_size = sizeof(propValue.value.int32Values)/sizeof(propValue.value.int32Values[0]);
                for(unsigned i= 0 ; i<internal_second_party_App_length ; i++)
                {
                    second_party_app_data.push_back(propValue.value.int32Values[i]);
                    ALOGD("VehicleHalImpl::second_party_app_data: %d %d" , i,second_party_app_data[i]); 
                } 
                 createInternalSecondPartyAppSignalsDataReq(second_party_app_data);
                 second_party_app_data.clear();
				

            }
               if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::SIVIGNSSGSM_AVAILABILITY_STATUS_VENDOR )
			{		 
                sivistatus = propValue.value.int32Values[0];
                createSiviGnssGsmAvailableStatusDataReq(sivistatus);			  
            } 
            
			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::NODEHMITIME_VENDOR){
                    std::string path ="";
                    if (auto var_hmitime = std::ofstream(path + "/dev/hmi_time")) {
                    std::vector<uint16_t> hmi_node;
                    hmi_node.push_back(propValue.value.int32Values[5]);
                    ALOGD("VehicleHalImpl:: HMITIMENODE[5] : %d :" , propValue.value.int32Values[5]);
                    hmi_node.push_back(propValue.value.int32Values[4]);
                    ALOGD("VehicleHalImpl:: HMITIMENODE[4] : %d :" , propValue.value.int32Values[4]);
                    hmi_node.push_back(propValue.value.int32Values[3]);
                    ALOGD("VehicleHalImpl:: HMITIMENODE[3] : %d :" , propValue.value.int32Values[3]);
                    hmi_node.push_back(propValue.value.int32Values[0]);
                    ALOGD("VehicleHalImpl:: HMITIMENODE[0] : %d :" , propValue.value.int32Values[0]);
                    hmi_node.push_back(propValue.value.int32Values[1]);
                    ALOGD("VehicleHalImpl:: HMITIMENODE[1] : %d :" , propValue.value.int32Values[1]); 
                    hmi_node.push_back(propValue.value.int32Values[2]);
                    ALOGD("VehicleHalImpl:: HMITIMENODE[2] : %d :" , propValue.value.int32Values[2]); 
                    
                   std::stringstream SS1; 
                   for(size_t i = 0; i < hmi_node.size(); ++i) {
                   SS1 << hmi_node[i]; 
                   if (i != hmi_node.size() - 1) {
                   SS1 << ".";
                    }
                }
    
               std::string result1 = SS1.str();
               ALOGD("VehicleHalImpl:: result1 : %s :", result1.c_str());
               var_hmitime <<result1;
               } else {
                   ALOGE("Failed to write HMI time");
                     }

            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::NODESECURETIME_VENDOR){
                    std::string path ="";
                    if (auto var_securenode = std::ofstream(path + "/dev/reference_time")) {
                std::vector<uint16_t> reference_node;
                reference_node.push_back(propValue.value.int32Values[5]);
                ALOGD("VehicleHalImpl:: SECURETIMENODE[5] : %d :" , propValue.value.int32Values[5]);	
                reference_node.push_back(propValue.value.int32Values[4]);
                ALOGD("VehicleHalImpl:: SECURETIMENODE[4] : %d :" , propValue.value.int32Values[4]);
                reference_node.push_back(propValue.value.int32Values[3]);
                ALOGD("VehicleHalImpl:: SECURETIMENODE[3] : %d :" , propValue.value.int32Values[3]);
                reference_node.push_back(propValue.value.int32Values[0]);
                ALOGD("VehicleHalImpl:: SECURETIMENODE[0] : %d :" , propValue.value.int32Values[0]);
                reference_node.push_back(propValue.value.int32Values[1]);
                ALOGD("VehicleHalImpl:: SECURETIMENODE[1] : %d :" , propValue.value.int32Values[1]); 
                reference_node.push_back(propValue.value.int32Values[2]);
                ALOGD("VehicleHalImpl:: SECURETIMENODE[2] : %d :" , propValue.value.int32Values[2]);
                std::stringstream SS; 
               for(size_t i = 0; i < reference_node.size(); ++i) {
                   SS << reference_node[i]; 
                   if (i != reference_node.size() - 1) {
                   SS << ".";
                    }
                }
    
               std::string result = SS.str();
               var_securenode << result;
               ALOGD("VehicleHalImpl:: result : %s :", result.c_str());
               std::tm tm = {};
               std::istringstream ss(result);

               ss >> std::get_time(&tm, "%Y.%m.%d.%H.%M.%S");
               if (ss.fail()) {
               std::cerr << "Error: Failed to parse date" << std::endl;
               
               }

              tm.tm_isdst = -1;  
              std::time_t timestamp = std::mktime(&tm);

              if (timestamp == -1) {
              std::cerr << "Error: Failed to convert to timestamp" << std::endl;
             
              }

             if (timestamp < std::numeric_limits<int>::min() || timestamp > std::numeric_limits<int>::max()) {
             std::cerr << "Error: Timestamp outside int range" << std::endl;
             
             }

             int intTimestamp = static_cast<int>(timestamp);
             std::cout << "Timestamp: " << intTimestamp << std::endl;
             createSystemTimeStampDataReq(intTimestamp);

            } else {
                   ALOGE("Failed to write `Reference time` to ");
                     }

            }
		    if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::PASSENGER_CONFIRM_STS_VENDOR )
			{		uint8_t Passenger_confirmSts = 0;
				    Passenger_confirmSts = propValue.value.int32Values[0];
                    ALOGD("VehicleHalImpl::Passenger Confirm STS from set function %d",Passenger_confirmSts); 
                    createPassengerConfirmStsDataReq(Passenger_confirmSts);

                
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_HMI_DATA_VENDOR )
			{				
				std::vector<uint16_t> vec_val;    
            vec_val.push_back(propValue.value.int32Values[0]);
			vec_val.push_back(propValue.value.int32Values[1]);
            vec_val.push_back(propValue.value.int32Values[2]);
			vec_val.push_back(propValue.value.int32Values[3]);
            vec_val.push_back(propValue.value.int32Values[4]);
			vec_val.push_back(propValue.value.int32Values[5]);
            vec_val.push_back(propValue.value.int32Values[6]);
			vec_val.push_back(propValue.value.int32Values[7]);
			vec_val.push_back(propValue.value.int32Values[8]);

			for(unsigned i = 0; i < vec_val.size(); i++)
			ALOGD("VehicleHalImpl::IVIHMIIVI-IMC data %d" , vec_val[i]);
			m_gateway.setIVIHMI_iviData(vec_val);
			createHMITouchDataReq(vec_val);
                
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMI_SOFTKEY_VENDOR )
			{				
				std::vector<uint16_t> vec_val;    
            vec_val.push_back(propValue.value.int32Values[0]);
			vec_val.push_back(propValue.value.int32Values[1]);
            vec_val.push_back(propValue.value.int32Values[2]);
            vec_val.push_back(propValue.value.int32Values[3]);
            vec_val.push_back(propValue.value.int32Values[4]);
			vec_val.push_back(propValue.value.int32Values[5]);

			for(unsigned i = 0; i < vec_val.size(); i++)
			ALOGD("VehicleHalImpl::HMI_SOFTKEY data %d" , vec_val[i]);
			m_gateway.setIVIHMISoftKeyStatus(vec_val);
			createsoftkeyDataReq(vec_val);
			} 			
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVIPHONECALLSTATUS_VENDOR )
		    {				
			ALOGD("VehicleHalImpl:: send  IVI Phone Call status On request ");
			uint16_t phonecall_status;
            std::vector<uint32_t> vec_val(6, 0);
			phonecall_status = propValue.value.int32Values[0];
			m_gateway.setIVIPhoneCallStatus(phonecall_status);
            if((phonecall_status == phonecallstatus::PHONE_STATE_ACTIVE) | (phonecall_status == phonecallstatus::PHONE_STATE_RINGING) | (phonecall_status == phonecallstatus::PHONE_STATE_DIALING))
            {
                vec_val[0] = 1;
            }
            else if(phonecall_status == phonecallstatus::PHONE_STATE_RINGING)
            {
                vec_val[1] = 1;
            }
            else if(phonecall_status == phonecallstatus::PHONE_STATE_DIALING)
            {
                vec_val[2] = 1;
            }
            createPhonecallstatusDataReq(phonecall_status);
			    vec_val.clear();
			    ALOGD("VehicleHalImpl::IVI PhoneCall status %d" , phonecall_status); 
			}
			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVISMSSTATUS_VENDOR)
			{				
				uint16_t smsStatus;
				smsStatus = propValue.value.int32Values[0];
			     ALOGD("VehicleHalImpl::IVI SMS status %d" , smsStatus); 
				 m_gateway.setIVISmsStatus(smsStatus);
			}
  			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVISECURTIME_VENDOR )
			{				
				std::vector<uint16_t> vec_val;
				vec_val.push_back(propValue.value.int32Values[0]);
				vec_val.push_back(propValue.value.int32Values[1]);
                vec_val.push_back(propValue.value.int32Values[2]);
                vec_val.push_back(propValue.value.int32Values[3]);
                vec_val.push_back(propValue.value.int32Values[4]);
                vec_val.push_back(propValue.value.int32Values[5]);

			    for(unsigned i = 0 , j = iviCanDatasend::NTW_TX_SIGNAL_UPD_SEC_CLK_DAY_REQ; i < vec_val.size(); i++ , j++)
                {
			        ALOGD("VehicleHalImpl::IVI secureTimedata %d %d" , i, vec_val[i]);
                    IVI_TX_signals[j] = vec_val[i];
			    }
				    if(IVI_Secure_oldData[0]!=vec_val[0])
                    m_gateway.setIVI_TX_Signals(vec_val[0],NTW_TX_SIGNAL_UPD_SEC_CLK_SEC_REQ);
					if(IVI_Secure_oldData[1]!=vec_val[1])
                    m_gateway.setIVI_TX_Signals(vec_val[1],NTW_TX_SIGNAL_UPD_SEC_CLK_MIN_REQ);
					if(IVI_Secure_oldData[2]!=vec_val[2])
                    m_gateway.setIVI_TX_Signals(vec_val[2],NTW_TX_SIGNAL_UPD_SEC_CLK_HR_REQ);
					if(IVI_Secure_oldData[3]!=vec_val[3])
                    m_gateway.setIVI_TX_Signals(vec_val[3],NTW_TX_SIGNAL_UPD_SEC_CLK_DAY_REQ);
					if(IVI_Secure_oldData[4]!=vec_val[4])
                    m_gateway.setIVI_TX_Signals(vec_val[4],NTW_TX_SIGNAL_UPD_SEC_CLK_MTH_REQ);
					if(IVI_Secure_oldData[5]!=vec_val[5])
                    m_gateway.setIVI_TX_Signals(vec_val[5],NTW_TX_SIGNAL_UPD_SEC_CLK_YR_REQ);

			     for(unsigned i = 0; i < SECURE_OLDDATA; i++)
				  IVI_Secure_oldData[i] = vec_val[i];               

			   for(unsigned i = 0; i < (SECUREtime_index-1) ; i++)
                {
                   InternalSystemTime[i] = vec_val[i];
                }
                createInternalSystemtimeDataReq(InternalSystemTime);

			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_EXTGPS_VENDOR )
			{				
				std::vector<uint16_t> vec_val;
				vec_val.push_back(propValue.value.int32Values[0]);
				vec_val.push_back(propValue.value.int32Values[1]);
                vec_val.push_back(propValue.value.int32Values[2]);
                vec_val.push_back(propValue.value.int32Values[3]);
                vec_val.push_back(propValue.value.int32Values[4]);
                vec_val.push_back(propValue.value.int32Values[5]);

                 for(unsigned i = 0 , j = iviCanDatasend::NTW_TX_SIGNAL_EXTGPSCLKYR; i < vec_val.size(); i++ , j++)
                 {
			        ALOGD("VehicleHalImpl::IVIEXTGPS %d %d" , i, vec_val[i]);
                    IVI_TX_signals[j] = vec_val[i];

                 }
                    if(IVI_EXTGPS_oldData[0]!=vec_val[0])
                    m_gateway.setIVI_TX_Signals(vec_val[0],NTW_TX_SIGNAL_EXTGPSCLKHR);
					if(IVI_EXTGPS_oldData[1]!=vec_val[1])
                    m_gateway.setIVI_TX_Signals(vec_val[1],NTW_TX_SIGNAL_EXTGPSCLKMINS);
					if(IVI_EXTGPS_oldData[2]!=vec_val[2])
                    m_gateway.setIVI_TX_Signals(vec_val[2],NTW_TX_SIGNAL_EXTGPSCLKSEC1);
					if(IVI_EXTGPS_oldData[3]!=vec_val[3])
                    m_gateway.setIVI_TX_Signals(vec_val[3],NTW_TX_SIGNAL_EXTGPSCLKDAY);
					if(IVI_EXTGPS_oldData[4]!=vec_val[4])
                    m_gateway.setIVI_TX_Signals(vec_val[4],NTW_TX_SIGNAL_EXTGPSCLKMTH);
					if(IVI_EXTGPS_oldData[5]!=vec_val[5])
                    m_gateway.setIVI_TX_Signals(vec_val[5],NTW_TX_SIGNAL_EXTGPSCLKYR);
		    	}

			     for(unsigned i = 0; i < EXTGPS_OLDDATA; i++)
				  IVI_EXTGPS_oldData[i] = vec_val[i];
			
 			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVIHMITIME_VENDOR )
			{				
				std::vector<uint16_t> vec_val;    

			vec_val.push_back(propValue.value.int32Values[0]);
			vec_val.push_back(propValue.value.int32Values[1]);
            vec_val.push_back(propValue.value.int32Values[2]);
            vec_val.push_back(propValue.value.int32Values[4]);
			vec_val.push_back(propValue.value.int32Values[5]);
            vec_val.push_back(propValue.value.int32Values[6]);

			     for(unsigned i = 0 , j = iviCanDatasend::NTW_TX_SIGNAL_MODE_HEURE_CLIENT_HS7; i < vec_val.size(); i++ , j++)
                 {
			        ALOGD("VehicleHalImpl::IVIHMITimedata %d %d" , i,vec_val[i]);
                    IVI_TX_signals[j] = vec_val[i];
                 }
				    if(IVI_HMI_oldData[0]!=vec_val[0])
                    m_gateway.setIVI_TX_Signals(vec_val[0],NTW_TX_SIGNAL_MODE_HEURE_CLIENT_HS7);
					if(IVI_HMI_oldData[1]!=vec_val[1])
                    m_gateway.setIVI_TX_Signals(vec_val[1],NTW_TX_SIGNAL_HEURE_HORLOGE_HS7);
					if(IVI_HMI_oldData[2]!=vec_val[2])
                    m_gateway.setIVI_TX_Signals(vec_val[2],NTW_TX_SIGNAL_MINUTE_HORLOGE_HS7);
					if(IVI_HMI_oldData[4]!=vec_val[4])
                    m_gateway.setIVI_TX_Signals(vec_val[4],NTW_TX_SIGNAL_JOUR_HORLOGE_HS7);
					if(IVI_HMI_oldData[5]!=vec_val[5])
                    m_gateway.setIVI_TX_Signals(vec_val[5],NTW_TX_SIGNAL_MOIS_HORLOGE_HS7);
					if(IVI_HMI_oldData[6]!=vec_val[6])
                    m_gateway.setIVI_TX_Signals(vec_val[6],NTW_TX_SIGNAL_ANNEE_HORLOGE_HS7);
							     
				for(unsigned i = 0; i < HMI_OLDDATA; i++)
				  IVI_HMI_oldData[i] = vec_val[i];
                    
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::AUDIO_ADAS_VENDOR)
			{				
				std::vector<uint8_t> vec_val;    
              for(unsigned i = 0 ;i < audioAdasVendorDatasend::Audio_adasSignal;i++){
        			vec_val.push_back(propValue.value.int32Values[i]);
                    ALOGD("VehicleHalImpl::Audio_adasSignal %d" , vec_val[i]);
                 }
                    createAudioAdasDataReq(vec_val);
            }
                    
 
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_BATTERY_PRECONDITION_VENDOR )
			{				
				std::vector<uint16_t> vec_val;    

			vec_val.push_back(propValue.value.int32Values[0]);
			vec_val.push_back(propValue.value.int32Values[1]);
            vec_val.push_back(propValue.value.int32Values[2]);
            vec_val.push_back(propValue.value.int32Values[3]);
            for(unsigned i = 0 , j = iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_ARRIVALSOC; i < vec_val.size(); i++ , j++)
            {
                IVI_TX_signals[j] = vec_val[i];
            }
			        if(IVI_BATTERYPRE_oldData[0]!=vec_val[0])
                    m_gateway.setIVI_TX_Signals(vec_val[0],NTW_TX_SIGNAL_NEXT_CHARG_STATION_ARRIVALSOC);
					if(IVI_BATTERYPRE_oldData[1]!=vec_val[1])
                    m_gateway.setIVI_TX_Signals(vec_val[1],NTW_TX_SIGNAL_NEXT_CHARG_STATION_DISTANCE);
					if(IVI_BATTERYPRE_oldData[2]!=vec_val[2])
                    m_gateway.setIVI_TX_Signals(vec_val[2],NTW_TX_SIGNAL_NEXT_CHARG_STATION_MAXPOWER);
					if(IVI_BATTERYPRE_oldData[3]!=vec_val[3])
                    m_gateway.setIVI_TX_Signals(vec_val[3],NTW_TX_SIGNAL_NEXT_CHARG_STATION_TIMETOTRAVEL);

            for(unsigned i = 0; i < EXTGPS_OLDDATA; i++)
             {
			    ALOGD("VehicleHalImpl::IVI Battery Preconditioning data  %d %d" , i,vec_val[i]);
				IVI_BATTERYPRE_oldData[i] = vec_val[i];
             }
            }
			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMI_PRIVACYMODE_CHANGEREQ_VENDOR)
			{				
				uint32_t Hmi_privacymode;
				Hmi_privacymode = propValue.value.int32Values[0];
				IVI_TX_signals[18] = Hmi_privacymode;
			    ALOGD("VehicleHalImpl::Hmi_privacymode: %d",Hmi_privacymode);
			    m_gateway.setIVI_TX_Signals(Hmi_privacymode ,iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_PRIVACY_MODE);
                m_gateway_TCU.Send_Privacy_mode_sts(Hmi_privacymode);
		    }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMITRIGGERFOR_UIN_VENDOR)
            {
                uint32_t Hmi_trigger_for_UIN;
                Hmi_trigger_for_UIN = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::Hmi_trigger_for_UIN: %d",Hmi_trigger_for_UIN);
                m_gateway_TCU.SendEvent_HMI_Trigger_For_UIN(Hmi_trigger_for_UIN);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVITHEME_VENDOR)
            {
                uint32_t IVI_Theme;
                IVI_Theme = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::IVI_Theme: %d",IVI_Theme);
                createIVIThemeVendorDataReq(IVI_Theme);
                
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMI_LANGUAGE_CHANGEREQ_VENDOR)
			{
				uint32_t Hmi_Language_Selection;
				Hmi_Language_Selection = propValue.value.int32Values[0];
				IVI_TX_signals[76] = Hmi_Language_Selection;
			    ALOGD("VehicleHalImpl::Hmi_Language_Selection: %d",Hmi_Language_Selection);
                m_gateway.setIVI_TX_Signals(Hmi_Language_Selection, iviCanDatasend::NTW_TX_SIGNAL_IVI_Language_SELECTION);
                m_gateway_TCU.SetLanguage_Selection(Hmi_Language_Selection);
			} 
			
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::CAN_IVI_TX_SIGNAL_VENDOR)
			{	std::stringstream ss;
				uint32_t Language_Selection;
                 for(unsigned i = 0;i<IVI_TX_Signals_size;i++){
                IVI_TX_signals.push_back(propValue.value.int32Values[i]);
				ss << IVI_TX_signals[i] << " ";
				ALOGD("VehicleHalImpl::IVI_TX_signals: %d %d",i,IVI_TX_signals[i]);
				}


               if (IVI_TX_signals[0] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_ACCUEIL_COND])
                    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_ACCUEIL_COND] ,iviCanDatasend::NTW_TX_SIGNAL_ACCUEIL_COND);
 
  
                if (IVI_TX_signals[17] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_CS_PARAM_VHL_MOBILITY])
                    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_CS_PARAM_VHL_MOBILITY] ,iviCanDatasend::NTW_TX_SIGNAL_CS_PARAM_VHL_MOBILITY);
 

                if (IVI_TX_signals[25] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_REV])
                    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_REV] ,iviCanDatasend::NTW_TX_SIGNAL_REV);
 
                if (IVI_TX_signals[26] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_RQ_LOCK_UNLOCK_REM_GEN2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_RQ_LOCK_UNLOCK_REM_GEN2], iviCanDatasend::NTW_TX_SIGNAL_RQ_LOCK_UNLOCK_REM_GEN2);

if (IVI_TX_signals[27] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_REMOTE_HORN_REQUEST])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_REMOTE_HORN_REQUEST], iviCanDatasend::NTW_TX_SIGNAL_REMOTE_HORN_REQUEST);

if (IVI_TX_signals[28] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_REMOTE_TURN_LIGHTS_REQ])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_REMOTE_TURN_LIGHTS_REQ], iviCanDatasend::NTW_TX_REMOTE_TURN_LIGHTS_REQ);

if (IVI_TX_signals[29] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_1);

if (IVI_TX_signals[30] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_2);

if (IVI_TX_signals[31] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_3);

if (IVI_TX_signals[32] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_4);

if (IVI_TX_signals[33] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_1);

if (IVI_TX_signals[34] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_1);

if (IVI_TX_signals[35] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_1);

if (IVI_TX_signals[36] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_1);

if (IVI_TX_signals[37] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_1);

if (IVI_TX_signals[38] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_1);

if (IVI_TX_signals[39] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_1);

if (IVI_TX_signals[40] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_2);

if (IVI_TX_signals[41] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_2);

if (IVI_TX_signals[42] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_2);

if (IVI_TX_signals[43] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_2);

if (IVI_TX_signals[44] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_2);

if (IVI_TX_signals[45] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_2);

if (IVI_TX_signals[46] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_2);

if (IVI_TX_signals[47] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_3);

if (IVI_TX_signals[48] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_3);

if (IVI_TX_signals[49] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_3);

if (IVI_TX_signals[50] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_3);

if (IVI_TX_signals[51] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_3);

if (IVI_TX_signals[52] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_3);

if (IVI_TX_signals[53] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_3);

if (IVI_TX_signals[54] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_4);

if (IVI_TX_signals[55] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_4);

if (IVI_TX_signals[56] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_4);

if (IVI_TX_signals[57] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_4);

if (IVI_TX_signals[58] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_4);

if (IVI_TX_signals[59] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_4);

if (IVI_TX_signals[60] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_4);

if (IVI_TX_signals[61] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_1);

if (IVI_TX_signals[62] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_2);

if (IVI_TX_signals[63] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_3);

if (IVI_TX_signals[64] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_4);

if (IVI_TX_signals[65] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ASAP])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ASAP], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ASAP);

if (IVI_TX_signals[66] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_REMOTE_IMMO_REQ])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_REMOTE_IMMO_REQ], iviCanDatasend::NTW_TX_SIGNAL_REMOTE_IMMO_REQ);

if (IVI_TX_signals[67] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_REMOTE_IMMO_PWD])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_REMOTE_IMMO_PWD], iviCanDatasend::NTW_TX_SIGNAL_REMOTE_IMMO_PWD);

if (IVI_TX_signals[68] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_TYPE_RECH_REM])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_TYPE_RECH_REM], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_TYPE_RECH_REM);
               
 if (IVI_TX_signals[69] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_REC_H_DEB_REM])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_REC_H_DEB_REM], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_REC_H_DEB_REM);

if (IVI_TX_signals[70] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_RECHARGE_REM])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_RECHARGE_REM], iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_RECHARGE_REM);

if (IVI_TX_signals[71] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_ARRIVALSOC])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_ARRIVALSOC], iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_ARRIVALSOC);

if (IVI_TX_signals[72] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_DISTANCE])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_DISTANCE], iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_DISTANCE);

if (IVI_TX_signals[73] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_MAXPOWER])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_MAXPOWER], iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_MAXPOWER);

if (IVI_TX_signals[74] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_TIMETOTRAVEL])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_TIMETOTRAVEL], iviCanDatasend::NTW_TX_SIGNAL_NEXT_CHARG_STATION_TIMETOTRAVEL);

if (IVI_TX_signals[75] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_CHIME_MUTE_REQ])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_CHIME_MUTE_REQ], iviCanDatasend::NTW_TX_SIGNAL_CHIME_MUTE_REQ);

/* if (IVI_TX_signals[76] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_IVI_Language_SELECTION])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_IVI_Language_SELECTION], iviCanDatasend::NTW_TX_SIGNAL_IVI_Language_SELECTION);
    m_gateway_TCU.SetLanguage_Selection(Language_Selection); */

if (IVI_TX_signals[77] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_Update_Req])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_Update_Req], iviCanDatasend::NTW_TX_SIGNAL_Update_Req);

if (IVI_TX_signals[78] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_REMOTE_ACTION_REQ_254__REMOTE_LOCK_UNLOCK_REQ])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_REMOTE_ACTION_REQ_254__REMOTE_LOCK_UNLOCK_REQ], iviCanDatasend::NTW_TX_SIGNAL_REMOTE_ACTION_REQ_254__REMOTE_LOCK_UNLOCK_REQ);

if (IVI_TX_signals[79] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_FUSEAU_HORAIRE])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_FUSEAU_HORAIRE], iviCanDatasend::NTW_TX_SIGNAL_FUSEAU_HORAIRE);

if (IVI_TX_signals[80] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_PRECOND])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_PRECOND], iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_PRECOND);

if (IVI_TX_signals[81] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_1);

if (IVI_TX_signals[82] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_2);

if (IVI_TX_signals[83] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_1);

if (IVI_TX_signals[84] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_2);

if (IVI_TX_signals[85] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_1);

if (IVI_TX_signals[86] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_2);

if (IVI_TX_signals[87] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_1);

if (IVI_TX_signals[88] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_2);

if (IVI_TX_signals[89] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_1);

if (IVI_TX_signals[90] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_2);

if (IVI_TX_signals[91] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_1);

if (IVI_TX_signals[92] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_2);

if (IVI_TX_signals[93] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_1);

if (IVI_TX_signals[94] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_2);

if (IVI_TX_signals[95] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_1);

if (IVI_TX_signals[96] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_2);

if (IVI_TX_signals[97] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_1], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_1);

if (IVI_TX_signals[98] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_2])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_2], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_2);

if (IVI_TX_signals[99] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_3);

if (IVI_TX_signals[100] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_4);

 if (IVI_TX_signals[101] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_3);

if (IVI_TX_signals[102] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_4);

if (IVI_TX_signals[103] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_3);

if (IVI_TX_signals[104] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_4);

if (IVI_TX_signals[105] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_3);

if (IVI_TX_signals[106] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_4);

if (IVI_TX_signals[107] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_3);

if (IVI_TX_signals[108] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_4);

if (IVI_TX_signals[109] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_3);

if (IVI_TX_signals[110] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_4);

if (IVI_TX_signals[111] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_3);

if (IVI_TX_signals[112] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_4);

if (IVI_TX_signals[113] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_3);

if (IVI_TX_signals[114] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_4);

if (IVI_TX_signals[115] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_3])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_3], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_3);

if (IVI_TX_signals[116] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_4])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_4], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_4);

if (IVI_TX_signals[117] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKDAY])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKDAY], iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKDAY);

if (IVI_TX_signals[118] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKHR])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKHR], iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKHR);

if (IVI_TX_signals[119] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKMINS])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKMINS], iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKMINS);
 
 if (IVI_TX_signals[120] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKMTH])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKMTH], iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKMTH);

if (IVI_TX_signals[121] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKSEC1])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKSEC1], iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKSEC1);

if (IVI_TX_signals[122] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKYR])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKYR], iviCanDatasend::NTW_TX_SIGNAL_GSMEXTCLKYR);

if (IVI_TX_signals[123] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_RECHARGE])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_RECHARGE], iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_RECHARGE);

if (IVI_TX_signals[124] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_REC_H_DEB])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_REC_H_DEB], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_REC_H_DEB);

if (IVI_TX_signals[125] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_TYPE_RECH])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_TYPE_RECH], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_RTAB_TYPE_RECH);

if (IVI_TX_signals[126] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_PRECOND_REM])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_PRECOND_REM], iviCanDatasend::NTW_TX_SIGNAL_DISPO_DMD_PRECOND_REM);

if (IVI_TX_signals[127] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_C_ABSENT])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_C_ABSENT], iviCanDatasend::NTW_TX_SIGNAL_C_ABSENT);

if (IVI_TX_signals[128] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_C_BUSOFF])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_C_BUSOFF], iviCanDatasend::NTW_TX_SIGNAL_C_BUSOFF);

if (IVI_TX_signals[129] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_C_DEFAUTPERTECOM])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_C_DEFAUTPERTECOM], iviCanDatasend::NTW_TX_SIGNAL_C_DEFAUTPERTECOM);

if (IVI_TX_signals[130] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_C_MUTE])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_C_MUTE], iviCanDatasend::NTW_TX_SIGNAL_C_MUTE);

if (IVI_TX_signals[131] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DIAG_MUX_ON_IVI])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DIAG_MUX_ON_IVI], iviCanDatasend::NTW_TX_SIGNAL_DIAG_MUX_ON_IVI);

if (IVI_TX_signals[132] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_EFF_DEF_IVI_HAB])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_EFF_DEF_IVI_HAB], iviCanDatasend::NTW_TX_SIGNAL_EFF_DEF_IVI_HAB);

if (IVI_TX_signals[133] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_PRIVACY_MODE_REM])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_PRIVACY_MODE_REM], iviCanDatasend::NTW_TX_SIGNAL_DMD_MENU_PRIVACY_MODE_REM);

/* if (IVI_TX_signals[134] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_DMD_MUTE_SUAL])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MUTE_SUAL], iviCanDatasend::NTW_TX_SIGNAL_DMD_MUTE_SUAL);

if (IVI_TX_signals[135] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_SIGNAL_XCALL_STATUS])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_XCALL_STATUS], iviCanDatasend::NTW_TX_SIGNAL_XCALL_STATUS);
 */
/* if (IVI_TX_signals[136] != old_tx_buffer_IVI[iviCanDatasend::NTW_TX_DCSD_DIAG_RES_Com_DCSD_Signal_Tx])
    m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_DCSD_DIAG_RES_Com_DCSD_Signal_Tx], iviCanDatasend::NTW_TX_DCSD_DIAG_RES_Com_DCSD_Signal_Tx); */

               
              old_tx_buffer_IVI = IVI_TX_signals;
	createIVI_TX_SIGNALDataReq(old_tx_buffer_IVI);
		std::string result = ss.str();
			ALOGD("VehicleHalImpl::IVI_TX_signals is:[size  %d ] %s  :", old_tx_buffer_IVI.size(),result.c_str());
			old_tx_buffer_IVI.clear();
			IVI_TX_signals.clear();
            }

 			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVIDLT_TRIGGER_VENDOR)
			{				
				uint16_t dlt_ExtractionTriggerStatus;
				dlt_ExtractionTriggerStatus = propValue.value.int32Values[0];
			     ALOGD("VehicleHalImpl::IVI DLT_TRIGGER status %d" , dlt_ExtractionTriggerStatus); 
				m_gateway.setIVI_DLTExtractionTriggerStatus(dlt_ExtractionTriggerStatus);
			}
			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVILOGEXTRACTSIGNAL_VENDOR)
			{				
				uint16_t logEXTRACT_triggerStatus;
				logEXTRACT_triggerStatus = propValue.value.int32Values[0];
			     ALOGD("VehicleHalImpl::IVI LOGEXTRACT DTC status %d" , logEXTRACT_triggerStatus); 
				m_gateway.setIVI_LogExtractionTriggerStatus(logEXTRACT_triggerStatus);
			} 
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::SATELLITES_NUMBER_VENDOR)
			{				
				uint16_t Satellites_Number;
                Satellites_Number = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::Satellites_Number %d" , Satellites_Number);
                createSatelliteNumberDataReq(Satellites_Number);

			} 
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::LATITUDE_DATA_VENDOR)
			{				
				float latitude;
                latitude = propValue.value.floatValues[0];
                ALOGD("VehicleHalImpl::latitude %f" , latitude);
                createlatitudeDataReq(latitude);
                

			} 
             if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::LONGITUDE_DATA_VENDOR)
			{				
				float longitude;
                longitude = propValue.value.floatValues[0];
                ALOGD("VehicleHalImpl::longitude %f" , longitude);
                createlongitudeDataReq(longitude);
                

			} 
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::CURRENT_ELEVATION_DATA_VENDOR)
			{				
				float current_elevation;
                current_elevation = propValue.value.floatValues[0];
                ALOGD("VehicleHalImpl::current_elevation %f" , current_elevation);
                createcurrentelevationDataReq(current_elevation);
                

			}
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::MAP_VERSION_DATA_VENDOR) {
            std::string MapVersion;
            MapVersion = propValue.value.stringValue;
            ALOGD("VehicleHalImpl::MapVersion %s", MapVersion.c_str());
            createMapVersionDataReq(MapVersion);
            } 
		
			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_STR_STATUS_VENDOR )
			{			
				LOG(INFO) << "VehicleHalImpl::set() vhalCustom_v1_0::VehiclePropertyCustom::IVI_STR_STATUS_VENDOR";
                if(vec_val[0] == propValue.value.int32Values[0]){
                    output[0] = propValue.value.int32Values[0];
                    LOG(INFO) << "VehicleHalImpl::set() IVI_STR_STATUS_VENDOR Audio output[0] : " << output[0];
                }
                else if(vec_val[1] == propValue.value.int32Values[1]){
                    output[1] = propValue.value.int32Values[1];
                    LOG(INFO) << "VehicleHalImpl::set() IVI_STR_STATUS_VENDOR HMI output[1] : " << output[1];
                }
                else if(vec_val[2] == propValue.value.int32Values[2]){
                    output[2] = propValue.value.int32Values[2];
                    LOG(INFO) << "VehicleHalImpl::set() IVI_STR_STATUS_VENDOR Carplay output[2] : " << output[2];
                }
                else if(vec_val[3] == propValue.value.int32Values[3]){
                    output[3] = propValue.value.int32Values[3];
                    LOG(INFO) << "VehicleHalImpl::set() IVI_STR_STATUS_VENDOR Connectivity (Wifi) output[3] : " << output[3];
                }
                else if(vec_val[4] == propValue.value.int32Values[4]){
                    output[4] = propValue.value.int32Values[4];
                    LOG(INFO) << "VehicleHalImpl::set() IVI_STR_STATUS_VENDOR Connectivity (BT) output[4] : " << output[4];
                }
                else if(vec_val[5] == propValue.value.int32Values[5]){
                    output[5] = propValue.value.int32Values[5];
                    LOG(INFO) << "VehicleHalImpl::set() IVI_STR_STATUS_VENDOR GNSS location output[5] : " << output[5];
                }
                else if(vec_val[6] == propValue.value.int32Values[6]){
                    output[6] = propValue.value.int32Values[6];
                    LOG(INFO) << "VehicleHalImpl::set() IVI_STR_STATUS_VENDOR Radio output[6] : " << output[6];
                }

                if( (output[0] == 1) && (output[1] == 1) && (output[2] == 1) && (output[3] == 1) && (output[4] == 1) && (output[5] == 1) && (output[6] == 1) ){
					LOG(INFO) << "VehicleHalImpl::set() vhalCustom_v1_0::VehiclePropertyCustom::IVI_STR_STATUS_VENDOR TRUE for sending 1 to MCU";
					output = {0, 0, 0, 0, 0, 0, 0};
                    uint8_t str_status = 1U;
				    m_gateway.setIVI_S2RStatus(str_status);
			    }
                else if( (propValue.value.int32Values[0] == 0) && (propValue.value.int32Values[1] == 0) && (propValue.value.int32Values[2] == 0) && (propValue.value.int32Values[3] == 0) && (propValue.value.int32Values[4] == 0) && (propValue.value.int32Values[5] == 0) && (propValue.value.int32Values[6] == 0) ){
                    LOG(INFO) << "VehicleHalImpl::set() vhalCustom_v1_0::VehiclePropertyCustom::IVI_STR_STATUS_VENDOR FALSE due to Getting all 0's";
					uint8_t str_status = 0U;
				    m_gateway.setIVI_S2RStatus(str_status);
                }              
                else{
                    LOG(INFO) << "VehicleHalImpl::set() vhalCustom_v1_0::VehiclePropertyCustom::IVI_STR_STATUS_VENDOR FALSE";
                }
			}
			
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::INTERNAL_SYSTEM_TIME_VENDOR )
			{				
				ALOGD("VehicleHalImpl:: send  INTERNAL_SYSTEM_TIME_VENDOR ");
				std::vector<uint16_t> vec_val;    

			vec_val.push_back(propValue.value.int32Values[0]);
			vec_val.push_back(propValue.value.int32Values[1]);
            vec_val.push_back(propValue.value.int32Values[2]);
            vec_val.push_back(propValue.value.int32Values[3]);
            vec_val.push_back(propValue.value.int32Values[4]);
			vec_val.push_back(propValue.value.int32Values[5]);

			for(unsigned i = 0; i < vec_val.size(); i++)
			ALOGD("VehicleHalImpl::IVIGSMData %d" , vec_val[i]);
            for(unsigned i = internalsystemTime::SIVIGSMClkHour,j = 0; i <= internalsystemTime::SIVIGSMClkYear,j <= internalsystemTime::SIVIGNSSClkYear; i++ ,j++)
            {
                InternalSystemTime[i] = vec_val[j];
			    ALOGD("VehicleHalImpl::InternalSystemTime %d" , InternalSystemTime[i]);
            }
            createInternalSystemtimeDataReq(InternalSystemTime);
			vec_val.clear();
			} 
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::USBUPDATE_VENDOR )
			{  
                ALOGD("VehicleHalImpl:: send  USBUPDATE_VENDOR ");
                std::vector<uint16_t> vec_val;
    //		    std::vector<uint16_t> USBUpdate_val_HMItoIMC;
				vec_val.push_back(propValue.value.int32Values[0]);
				vec_val.push_back(propValue.value.int32Values[1]);
				vec_val.push_back(propValue.value.int32Values[2]);
				vec_val.push_back(propValue.value.int32Values[3]);
				vec_val.push_back(propValue.value.int32Values[4]);
				vec_val.push_back(propValue.value.int32Values[5]);
				vec_val.push_back(propValue.value.int32Values[6]);
				vec_val.push_back(propValue.value.int32Values[7]);
				vec_val.push_back(propValue.value.int32Values[8]);
				vec_val.push_back(propValue.value.int32Values[9]);
	//			vec_val.push_back(propValue.value.int32Values[10]);
	//			vec_val.push_back(propValue.value.int32Values[11]);
    //          USBUpdate_val_HMItoIMC.push_back(vec_val[10]);
	//		    USBUpdate_val_HMItoIMC.push_back(vec_val[11]);
                for(unsigned i = 0; i < vec_val.size(); i++) 				
				ALOGD("VehicleHalImpl:: send  USBUPDATE_VENDOR: %d",vec_val[i]);
    //          m_gateway.setIVI_USB_Update(USBUpdate_val_HMItoIMC);
				createUSB_UpdateSignalsDataReq(vec_val);
			    vec_val.clear();
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IMCMCU_USBUPDATE_VENDOR  )
			{  
                // ALOGD("VehicleHalImpl:: send  IMCMCU_USBUPDATE_VENDOR  ");
                // std::vector<uint16_t> vec_val;
        	    std::vector<uint16_t> USBUpdate_val_HMItoIMC;
				// USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[0]);
				//  USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[1]);
				//  USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[2]);
				//  USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[3]);
				//  USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[4]);
				//  USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[5]);
				//  USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[6]);
				//  USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[7]);
				//  USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[8]);
				//  USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[9]);
				 USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[10]);
				 USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[11]);

                for(unsigned i = 0; i < USBUpdate_val_HMItoIMC.size(); i++) 				
				ALOGD("VehicleHalImpl:: send  IMCMCU_USBUPDATE_VENDOR: %d",vec_val[i]);
                m_gateway.setIVI_USB_Update(USBUpdate_val_HMItoIMC);
				//createUSB_UpdateSignalsDataReq(vec_val);
			    USBUpdate_val_HMItoIMC.clear();
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMIUSBUPDATE_USERCONSENT_VENDOR )
			{  
               // ALOGD("VehicleHalImpl:: send  HMIUSBUPDATE_USERCONSENT_VENDOR ");
				std::vector<uint16_t> USBUpdate_val_HMItoIMC;
				USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[0]);
				USBUpdate_val_HMItoIMC.push_back(propValue.value.int32Values[1]);
 
                for(unsigned i = 0; i < vec_val.size(); i++) 				
				ALOGD("VehicleHalImpl:: send  HMIUSBUPDATE_USERCONSENT_VENDOR: %d",USBUpdate_val_HMItoIMC[i]);
                // m_gateway.setIVI_USB_Update(USBUpdate_val_HMItoIMC);
			    USBUpdate_val_HMItoIMC.clear();
			}
			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::DID_RID_DATA_VENDOR )
			{   std::vector<uint16_t> vec_val;
                ALOGD("VehicleHalImpl:: send  DID_RID_DATA_VENDOR ");
                for(auto N : propValue.value.int32Values)
                {
                vec_val.push_back(N); 
                }
                for(unsigned i = 0; i < vec_val.size(); i++) 				
				ALOGD("VehicleHalImpl:: send  DID_RID_DATA_VENDOR: %d",vec_val[i]);
                m_gateway.setIVI_DIDdata_request(vec_val);
			    vec_val.clear();
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_RESET_USB_VENDOR)
			{				
				uint16_t Usb_Reset_Status;
				Usb_Reset_Status = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::send RESET_USB_VENDOR status %d",Usb_Reset_Status); 
				m_gateway.setIVI_Usb_Reset_TriggerStatus(Usb_Reset_Status);
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::INTERNAL_ACTIVE_PROJECTION_VENDOR)
			{				
				uint16_t Projection_active_status;
				Projection_active_status = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::send INTERNAL_ACTIVE_PROJECTION_VENDOR status %d",Projection_active_status);
                createProjectionActiveStatusDataReq(Projection_active_status); 
				
			} 
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::CALL_ACTIVE_STR_INPUT_FROM_POPUP_VENDOR)
			{			
				LOG(INFO) << "VehicleHalImpl::set() vhalCustom_v1_0::VehiclePropertyCustom::CALL_ACTIVE_STR_INPUT_FROM_POPUP_VENDOR";
                uint8_t userInputFromHMIPopup = propValue.value.int32Values[0];
				m_gateway.setIVI_CallActiveSTR_HMIPopup_Status(userInputFromHMIPopup);
			}
			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::DIAG_DTC_VENDOR )
			{   ALOGD("VehicleHalImpl:: send  DIAG_DTC_VENDOR");
				std::vector<uint8_t> dtc_value;    

			    dtc_value.push_back(propValue.value.int32Values[0]);
			    dtc_value.push_back(propValue.value.int32Values[1]);
                dtc_value.push_back(propValue.value.int32Values[2]);
                dtc_value.push_back(propValue.value.int32Values[3]);
				
                for(unsigned i = 0; i < dtc_value.size(); i++) 				
				ALOGD("VehicleHalImpl:: Valur  DIAG_DTC_VENDOR: %d %d",i,dtc_value[i]);
			
                m_gateway.setDEM_DTCStatus(dtc_value);	
			    dtc_value.clear();
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_USER_SETTING_DATA_VENDOR)
			{   std::vector<uint32_t> vec_val;
                ALOGD("VehicleHalImpl:: send_IVI_USER_SETTING_DATA_VENDOR");
                for(auto N : propValue.value.int32Values)
                {
                vec_val.push_back(N); 
                }
                for(unsigned i = 0; i < vec_val.size(); i++) 				
				ALOGD("VehicleHalImpl:: send_IVI_USER_SETTING_DATA_VENDOR: %d",vec_val[i]);
                createUserSettingsDataReq(vec_val);
			    vec_val.clear();
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::LOG_RETRIVER_STATUS_VENDOR)
			{   std::vector<uint32_t> vec_val;
                ALOGD("VehicleHalImpl:: send_LOG_RETRIVER_STATUS_VENDOR");
                for(auto N : propValue.value.int32Values)
                {
                vec_val.push_back(N); 
                }
                for(unsigned i = 0; i < vec_val.size(); i++) 				
				ALOGD("VehicleHalImpl:: send_LOG_RETRIVER_STATUS_VENDOR: %d",vec_val[i]);
                m_gateway.SetLoggerdata(vec_val);
			    vec_val.clear();
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_USER_XCALL_INPUT_VENDOR)
			{				
				uint32_t UserXcall_status;
				UserXcall_status = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::IVI_USER_XCALL_INPUT_VENDOR %d",UserXcall_status); 
				m_gateway_TCU.SetUserXcall(UserXcall_status);
			}

			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::ANTITHEFT_VENDOR)
			{				
				uint8_t antitheft_status;
				antitheft_status = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::antitheft status from set function %d",antitheft_status); 
				createAntitheftStatus_DataReq(antitheft_status);
			}
			 if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::FAN_SPEED_DATA_VENDOR)
			{				
				uint32_t Fan_speed;
				Fan_speed = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::IVI_Fan_speed: %d",Fan_speed); 
				m_gateway.SetInternal_Fan_Speed(Fan_speed);
			}
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::LOGFILEPATH_HMI_VENDOR) 
            {
                std::string filepath;
                filepath = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::filepath %s", filepath.c_str());
                m_gateway.SendFilepathtoTCU(filepath);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::LOGRETRIEVER_STATUS_VENDOR)
			{				
				uint32_t logretriever_status;
				logretriever_status = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::logretriever_status: %d",logretriever_status); 
				m_gateway.getlogretrieverstatus(logretriever_status);
			}
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::POI_DATABASE_VERSION_VENDOR) 
            {
                std::string POI_databaseversion;
                POI_databaseversion = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::POI_databaseversion %s", POI_databaseversion.c_str());
                createPOI_databaseversionDataReq(POI_databaseversion);
                
            }
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::LASTUPDATE_TIME_VENDOR) 
            {
                std::string lastUpdateTime;
                lastUpdateTime = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::lastUpdateTime %s", lastUpdateTime.c_str());
                createlastUpdateTimeDataReq(lastUpdateTime);
                
            }
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMIFOTA_UPDATEHISTORY_VENDOR ) 
            {
                std::string update_history;
                update_history = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::update_history %s", update_history.c_str());
                m_gateway_TCU.Send_HMIFOTA_Update_history(update_history);
            }
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMIFOTA_PENDINGUPDATES_VENDOR ) 
            {
                std::string pending_updates;
                pending_updates = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::pending_updates %s", pending_updates.c_str());
                m_gateway_TCU.Send_HMIFOTA_pending_updates(pending_updates);
            }
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMIFOTA_WHATSNEWDETAILS_VENDOR ) 
            {
                std::string Whatsnewdetails;
                Whatsnewdetails = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::Whatsnewdetails %s", Whatsnewdetails.c_str());
                m_gateway_TCU.Send_FOTA_WhatsNew_Details(Whatsnewdetails);
            }
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMIFOTA_SHEDULE_UPDATE ) 
            {
                std::string ScheduleUpdate;
                ScheduleUpdate = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::ScheduleUpdate %s", ScheduleUpdate.c_str());
                m_gateway_TCU.Send_FOTA_Schedule_Update(ScheduleUpdate);
            }
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMIFOTA_UPDATE_DISMISSED ) 
            {
                std::string UpdateDismissed;
                UpdateDismissed = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::UpdateDismissed %s", UpdateDismissed.c_str());
                m_gateway_TCU.Send_FOTA_Update_Dismissed(UpdateDismissed);
            }

            // new FOTA att
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMIFOTA_INSTALLATION_FINISHED_WITH_USER_CONFIRMATION ) 
            {
                std::string Installation_finished;
                Installation_finished = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::Installation_finished %s", Installation_finished.c_str());
                m_gateway_TCU.Send_FOTA_Installation_Finished_with_User_Conformation(Installation_finished);
            }
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMIFOTA_START_INSTALLATION ) 
            {
                std::string start_installation;
                start_installation = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::start_installation %s", start_installation.c_str());
                m_gateway_TCU.Send_FOTA_Start_Installation(start_installation);
            }
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMIFOTA_CHECK_FOR_UPDATES ) 
            {
                std::string checkfor_updates;
                checkfor_updates = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::checkfor_updates %s", checkfor_updates.c_str());
                m_gateway_TCU.Send_FOTA_Check_for_Updates(checkfor_updates);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_RESET_DATA_VENDOR)
			{				
				uint32_t Reset_data;
				Reset_data = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::IVI_Reset_data: %d",Reset_data); 
				m_gateway.setIVIResettrigger(Reset_data);
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::XCALL_HMI_CONTROL_VENDOR)
			{				
				uint32_t Xcall_HMI_Control;
				Xcall_HMI_Control = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::Xcall_HMI_Control: %d",Xcall_HMI_Control); 
				m_gateway_TCU.SetXcall_HMI_Control(Xcall_HMI_Control);
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::IVI_DRIVER_LOCKOUTSTS_VENDOR)
			{				
				uint32_t Driver_lockout;
				Driver_lockout = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::Driver_lockout: %d",Driver_lockout);
                createDriverlockoutDataReq(Driver_lockout); 
				
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::ISP_CAMERA_VENDOR)
			{   std::vector<uint32_t> vec_val;
                ALOGD("VehicleHalImpl:: ISP_CAMERA_VENDOR");
                for(auto N : propValue.value.int32Values)
                {
                vec_val.push_back(N); 
                }
                for(unsigned i = 0; i < vec_val.size(); i++) 				
				ALOGD("VehicleHalImpl:: send_ISP_CAMERA_VENDOR: %d",vec_val[i]);
                createCameraISPDataReq(vec_val);
                vec_val.clear();
			} 
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::INTERNAL_SYSTEM_TIME_ZONE)
			{				
				int32_t SystemTimeZone;
                SystemTimeZone = propValue.value.int32Values[0];
                if (SystemTimeZone != previousSystemTimeZone) {
                ALOGD("VehicleHalImpl::SystemTimeZone changed to %d", SystemTimeZone);
                m_gateway.SetSystemTimeZone(SystemTimeZone);
                previousSystemTimeZone = SystemTimeZone;
               }
               createSystemTimeZoneDataReq(SystemTimeZone);
                
			}
			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::FLASH_RESULT_VENDOR)
			{				
				uint32_t Flash_Result;
				Flash_Result = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::FLASH_RESULT_VENDOR: %d",Flash_Result);
                m_gateway_TCU.SetFlashResult(Flash_Result);
			} 
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::FACTORY_RESET_COUNT)
			{				
				uint32_t Flash_Reset_Count;
				Flash_Reset_Count = propValue.value.int32Values[0];
			    ALOGD("VehicleHalImpl::FACTORY_RESET_COUNT: %d",Flash_Reset_Count);
                createFactoryResetCountDataReq(Flash_Reset_Count);
			}
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::AOSP_FLASH_RESULT_VENDOR)
            {
                std::string flash_result;
                flash_result = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::AOSP_FLASH_RESULT_VENDOR %s", flash_result.c_str());
                m_gateway_TCU.send_AOSP_Flash_Result(flash_result);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::ANDROID_RVCSTATUS_VENDOR)
            {
                uint8_t arvc_status;
                arvc_status = propValue.value.int32Values[0];
                LOG(INFO) << "VehicleHalImpl::arvc_status" << (uint32_t)arvc_status;
                m_gateway.send_arvc_status_mcu(arvc_status);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMI_STATUS_SWUPDATE_VENDOR)
            {                
				uint32_t HMI_Status_SWUpdate;
			    HMI_Status_SWUpdate = propValue.value.int32Values[0];
				ALOGD("VehicleHalImpl::HMI_STATUS_SWUPDATE_VENDOR %d", HMI_Status_SWUpdate);
				m_gateway_TCU.Send_HMI_Status_SWUpdate(HMI_Status_SWUpdate);
			}
			if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::GPSHISTOGRAPH_DISTELEC_VENDOR)
            {
                uint32_t graph_elec_cons_hist;
                graph_elec_cons_hist = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::GPSHISTOGRAPH_DISTELEC_VENDOR: %d", graph_elec_cons_hist);
                createGraphElecConsHistDataReq(graph_elec_cons_hist);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::CGTRTABTYPERECHARGE_VENDOR)
            {
                uint32_t charge_type_req;
                charge_type_req = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::CGTRTABTYPERECHARGE_VENDOR: %d", charge_type_req);
                createChargeTypeReqHMIDataReq(charge_type_req);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::CGTRTABRECHARGE_HDEBUT_VENDOR)
            {
                uint32_t start_time_delayed_charge;
                start_time_delayed_charge = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::CGTRTABRECHARGE_HDEBUT_VENDOR: %d", start_time_delayed_charge);
                createStartTimeDelayedChargeHMIDataReq(start_time_delayed_charge);
            } 
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::TEMPORTAB_RECHARGE_VENDOR){
                uint32_t charge_menu_delay_sts;
                charge_menu_delay_sts = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::TEMPORTAB_RECHARGE_VENDOR: %d", charge_menu_delay_sts);
                createChargeMenuDelayStsDataReq(charge_menu_delay_sts);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::VEHGEARCONFIG_VENDOR){
                uint32_t veh_gear_config;
                veh_gear_config = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::VEHGEARCONFIG_VENDOR: %d", veh_gear_config);
                createVehGearConfigDataReq(veh_gear_config);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::VEHGEAR_INT_VENDOR){
                uint32_t veh_gear_net;
                veh_gear_net = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::VEHGEAR_INT_VENDOR: %d", veh_gear_net);
                createVehGearNetDataReq(veh_gear_net);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::VEHGEARPOS_INT_VENDOR){
                uint32_t veh_gear_pos_int;
                veh_gear_pos_int = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::VEHGEARPOS_INT_VENDOR: %d", veh_gear_pos_int);
                createVehPosIntDataReq(veh_gear_pos_int);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::REVERSEGEARSTS_VENDOR){
                uint32_t reverse_gear_sts;
                reverse_gear_sts = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::REVERSEGEARSTS_VENDOR: %d", reverse_gear_sts);
                createReverseGearStsDataReq(reverse_gear_sts);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::MANUALPARKBREAKSTS_VENDOR){
                uint32_t manual_park_break_sts;
                manual_park_break_sts = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::MANUALPARKBREAKSTS_VENDOR: %d", manual_park_break_sts);
                createManualParkBreakStsDataReq(manual_park_break_sts);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::PARKBREAKSTSEST_VENDOR){
                uint32_t park_break_sts_est;
                park_break_sts_est = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::PARKBREAKSTSEST_VENDOR: %d", park_break_sts_est);
                createParkBreakStsEstDataReq(park_break_sts_est);
            }
            if (propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::HMIFOTA_INFO_VENDOR ) {
                std::string hmifota_info;
                hmifota_info = propValue.value.stringValue;
                ALOGD("VehicleHalImpl::HMIFOTA_INFO_VENDOR %s", hmifota_info.c_str());
                m_gateway_TCU.Send_HMIFOTA_Info(hmifota_info);
            }
            if(propId == (int)vhalCustom_v1_0::VehiclePropertyCustom::GNSSSIGNALSTRENGTH_VENDOR){
                int32_t gnss_signal_strength;
                gnss_signal_strength = propValue.value.int32Values[0];
                ALOGD("VehicleHalImpl::GNSSSIGNALSTRENGTH_VENDOR: %d", gnss_signal_strength);
                createGnssSignalStrengthDataReq(gnss_signal_strength);
            }
		}
    }

    return ret;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createApPowerStateReq(
        vhal_v2_0::VehicleApPowerStateReq state, int32_t param) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, 2);
    req->prop = toInt(vhal_v2_0::VehicleProperty::AP_POWER_STATE_REQ);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
    req->value.int32Values[0] = toInt(state);
    req->value.int32Values[1] = param;
    return req;
}
vhal_v2_0::StatusCode VehicleHalImpl::subscribe(int32_t property, float sampleRate)
{
    ALOGD("VehicleHalImpl : subscribe()");
	ALOGD("%s(): ContinuousProperty: %d, sampleRate: %f", __func__, property, sampleRate);
    if (isContinuousProperty(property))
    {
        ALOGD("%s(): ContinuousProperty: %d, sampleRate: %f", __func__, property, sampleRate);
        mRecurrentTimer.registerRecurrentEvent(hertzToNanoseconds(sampleRate), property);
    }
    return vhal_v2_0::StatusCode::OK;
}

vhal_v2_0::StatusCode VehicleHalImpl::unsubscribe(int32_t property)
{
    ALOGD("VehicleHalImpl : unsubscribe()");
	ALOGD("%s(): ContinuousProperty: %d", __func__, property);
    if (isContinuousProperty(property))
    {
        mRecurrentTimer.unregisterRecurrentEvent(property);
    }
    return vhal_v2_0::StatusCode::OK;
}

void VehicleHalImpl::onPropertyValue(const vhal_v2_0::VehiclePropValue& value, bool updateStatus)
{
    ALOGD("VehicleHalImpl : onPropertyValue()");
    vhal_v2_0::VehicleHal::VehiclePropValuePtr updatedPropValue =
        vhal_v2_0::VehicleHal::getValuePool()->obtain(value);

    if (mPropStore->writeValue(*updatedPropValue, updateStatus))
    {
		ALOGD("VehicleHalImpl : set property doHAL");
        vhal_v2_0::VehicleHal::doHalEvent(std::move(updatedPropValue));
    }
}

void VehicleHalImpl::regPropsInPropStore()
{
    ALOGD("VehicleHalImpl : regPropsInPropStore()");
    for (const auto& it : vhal_v2_0::impl::kVehicleProperties) mPropStore->registerProperty(it.config);
}

void VehicleHalImpl::initPropsInPropStore()
{
    ALOGD("VehicleHalImpl : initPropsInPropStore()");
    static constexpr bool shouldUpdateStatus = true;

    for (const auto& p : vhal_v2_0::impl::kVehicleProperties)
    {
        vhal_v2_0::VehiclePropValue prop = {.prop = p.config.prop};

        if (vhal_v2_0::isGlobalProp(prop.prop))
        {
            prop.areaId = 0;
            prop.value = p.initialValue;

            mPropStore->writeValue(prop, shouldUpdateStatus);
        }
        else
        {
            for (auto const& ac : p.config.areaConfigs)
            {
                prop.areaId = ac.areaId;
                prop.value = p.initialValue; 

                auto search = p.initialAreaValues.find(ac.areaId);
                if (search != p.initialAreaValues.end()) prop.value = search->second;

                mPropStore->writeValue(prop, shouldUpdateStatus);
            }
        }
    }
}

void VehicleHalImpl::onContinuousPropertyTimer(const std::vector<int32_t>& properties)
{
    // ALOGD("VehicleHalImpl : onContinuousPropertyTimer()");
    VehiclePropValuePtr v;
    

    auto& pool = *getValuePool();

    for (int32_t property : properties)
    {
        if (isContinuousProperty(property))
        {
            auto internalPropValue = mPropStore->readValueOrNull(property);
            if (internalPropValue != nullptr)
            {
                v = pool.obtain(*internalPropValue);
            }
        }
        else
        {
            ALOGE("%s(): Unexpected for property: %d", __func__, property);
        }

        if (v.get())
        {
            v->timestamp = android::elapsedRealtimeNano();
            doHalEvent(std::move(v));
        }
    }
	pollForCANData();
	pollForPowerModeData();
	pollForIMUsubscribe();
	pollForDIDsubscribe();
	pollForDotteDatasubscribe();
	pollForDTCsubscribe();
	pollForDLTsubscribe();
	pollforVinNumbersubscribe();
    pollForInternalSignalsubscribe();
    pollForTemperature();
    pollforsupplyvoltage();
    pollForUSB_update();
    pollForUSB_update_details();
	pollForDLT_DTC_LOG();
	pollforCALDATAsubscribe();
    pollForTCUData();
    pollForCallActiveSTRDisplayPopupStatus();
	pollForDEM_DTCStatussubscribe();
	pollForSoCVersionData();
	pollForLoggerdata();
    pollForSLI_Data();
    pollForhw_variant_id();
    pollForSOC_Wakeup_reason();
    pollForMarelliPartNumber();
    pollForEOL_hw_variant_id();
    pollForPackagePath();
	pollforCheckforUpdateResults();
	pollforUpdateAvailable();
    pollforConditionsNotMet();
    pollforInstallationStatus();
    pollforUpdateFinished();
    pollforWhatsNewDetails();
    pollforUpdateHistory();
    pollforPendingUpdates();
    pollforScheduleUpdate();
    pollForInternalSignalInitialize();
    pollforAOSPRBUATrigger();
    pollforFotaHmiInfoResult();
    pollforUSBUpdateDetails();
    	
}

void VehicleHalImpl::pollForDLT_DTC_LOG(){

if(b_dlt_trigger == true){
m_gateway.setIVI_LogExtractionTriggerStatus(logtigger_dammy);
}
b_dlt_trigger = false;
if(b_calib_trigger == true){
m_gateway.setIVI_CalibrationTriggerStatus(calibtigger_dammy);
}
b_calib_trigger =false;

}

void VehicleHalImpl::pollForCANData(){
 if(m_gateway.isAvailable())
	{
		if(b_CANstream == true)
		{
			m_gateway.getCANData();
			m_gateway.getApplicationSoftwareIdentification();
            m_gateway.setSOC_Ready_Status();
			createIVI_TX_SIGNALDataReq(old_tx_buffer_IVI);
			b_CANstream = false;
		}
	}
}
void VehicleHalImpl::pollForDIDsubscribe(){
 if(m_gateway.isAvailable())
	{
		if(b_DIDstream == true)
		{
			m_gateway.getDIDsubscribe();
			b_DIDstream = false;
		}
	}
}
void VehicleHalImpl::pollForDEM_DTCStatussubscribe(){
		if(b_DIAGstream == true)
		{
			m_gateway.getDEM_DTCStatussubscribe();
			b_DIAGstream = false;
		}
}
void VehicleHalImpl::pollForLoggerdata(){
    if(b_Logdata == true)
    {
        m_gateway.getLoggerdatasubscribe();
        b_Logdata = false;
    }
}
void VehicleHalImpl::pollForIMUsubscribe(){
 if(m_gateway.isAvailable())
	{
		if(b_IMUstream == true)
		{
			m_gateway.getIMUsubscribe();
			b_IMUstream = false;
		}
	}
}

void VehicleHalImpl::pollForDotteDatasubscribe(){
	if(m_gateway.isAvailable_dotteProxy())
		{
		if(b_Dottestream == true)
		{
			m_gateway.getDottesubscribe();
			b_Dottestream = false;
		}
	}
}
void VehicleHalImpl::pollForDTCsubscribe(){
		if(b_DTCstream == true)
		{
			m_gateway.getDTCsubscribe();
			b_DTCstream = false;
		}
}
void VehicleHalImpl::pollForDLTsubscribe(){
		if(b_DLTstream == true)
		{
			m_gateway.getDLTsubscribe();
			b_DLTstream = false;
		}
}
void VehicleHalImpl::pollForTemperature(){
		if(b_Temperature == true)
		{
			m_gateway.getTemperaturesubscribe();
			b_Temperature = false;
		}
}
void VehicleHalImpl::pollforsupplyvoltage(){
    if(b_supplyvge == true)
    {
        m_gateway.getsupplyvgesubscribe();
        b_supplyvge = false ;
    }
}

void VehicleHalImpl::pollForCallActiveSTRDisplayPopupStatus(){
    if(b_CallActiveSTRStatus == true){
        m_gateway.getCallActiveSTRDisplayPopupStatusSubscribe();
        b_CallActiveSTRStatus = false;
    }
}

void VehicleHalImpl::pollForSTRCancelShutdown(){
    LOG(INFO) << "VehicleHalImpl::pollForSTRCancelShutdown()";
    if(b_STRCancelShutdown == true){
        m_gateway.getSTRCancelShutdownSubscribe();
        b_STRCancelShutdown = false;
    }
}

void VehicleHalImpl::pollForUSB_update(){
		if(b_USB_update == true)
		{
			m_gateway.getUSB_Updatesubscribe();
			b_USB_update = false;
		}
}
void VehicleHalImpl::pollForUSB_update_details(){
		if(b_USB_update_details == true)
		{
			m_gateway.getUSB_Updatesubscribe();
			b_USB_update_details = false;
		}
}
void VehicleHalImpl::pollForInternalSignalsubscribe(){
		if(b_internalSignal == true)
		{
			m_gateway.getInternalSigsubscribe();
			b_internalSignal = false;
		}
}

void VehicleHalImpl::pollforVinNumbersubscribe(){
		if(b_vinNumbersubscribe == true)
		{ 
			m_gateway.getVinNumbersubscribe();
 			b_vinNumbersubscribe = false;
		}
}
void VehicleHalImpl::pollforCALDATAsubscribe(){
		if(b_calDatasubscribe == true)
		{ 
			m_gateway.getCalibrationDatasubscribe();
 			b_calDatasubscribe = false;
		}
}
 void VehicleHalImpl::pollForPowerModeData(){
    if(b_powermode == true)
    {
        m_gateway.getPowerMode();
        b_powermode = false;
    }
}
 void VehicleHalImpl::pollForSoCVersionData(){
    if(b_SocVersion == true)
    {
        m_gateway.setIVI_SoCVersion_Send_MCU();
        b_SocVersion = false;
    }
}
void VehicleHalImpl::pollForhw_variant_id(){
    if(b_hw_variant == true)
    {
        m_gateway.setIVI_hw_variant_id_Send_MCU();
        b_hw_variant = false;
    }
}
void VehicleHalImpl::pollForSOC_Wakeup_reason(){
    if(b_SOC_wakeup_reason == true)
    {
        m_gateway.getSOC_Wakeup_reasonsubscribe();
        b_SOC_wakeup_reason = false;
    }
}
void VehicleHalImpl::pollForMarelliPartNumber(){
    if(b_part_number == true)
    {
        m_gateway.getMarelliPartNumbersubscribe();
        b_part_number = false;
    }
}
void VehicleHalImpl::pollForEOL_hw_variant_id(){
    if(b_EOL_Hw_id == true)
    {
        m_gateway.getEOL_hw_variantsubscribe();
        b_EOL_Hw_id = false;
    }
}

 void VehicleHalImpl::pollForInternalSignalInitialize(){
    if(b_internal_hmi_length == true)
    {
       createInternalHmiSignalsDataReq(default_hmi_length);
       createInternalSecondPartyAppSignalsDataReq(default_second_party_App_length);
       createInternalAudioSignalsDataReq(default_audio_length);
       createInternalSystemtimeDataReq(default_SystemTime_Index);
       createinternalSignalDataReq(default_INTERNALSignal_index);
       createInternalCameraSignalsDataReq(default_camera_length);
       createInternalAudioSignalsDataReq(default_audio_length);
       createInternalProjectionDataReq(default_projection);
       b_internal_hmi_length = false;
    } 
 }  
void VehicleHalImpl::startSuspendToRam(uint32_t powerModeValue){
    if((powerModeValue == powerModeSTR) || (powerModeValue == powerModeSTRLM1) || (powerModeValue == powerModeSTRLM2)){
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() powerModeValue = "<<powerModeValue;
	LOG(INFO) << "VehicleHalImpl::startSuspendToRam() Stop scheduling table";
	system("echo 0 > /sys/devices/platform/soc/4ac0000.qcom,qupv3_0_geni_se/4a8c000.qcom,qup_uart/scheduleTableSwitch");
	flagForSchedulingTable = true;

        //LVDS into sleep state
        outputForLVDSWakeupAndSleepState = LVDSMgr_SetProperty(LVDSMgr_MODE_SLEEP, true);
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() LVDSMgr_SetProperty() for sleep outputForLVDSWakeupAndSleepState = " << outputForLVDSWakeupAndSleepState;    
        handleLVDSWakeupAndSleepState = true;  
        outputForLVDSWakeupAndSleepState = 1; 
    }

    if(powerModeValue == powerModeSTR){
		handleDisplayUpInterruptDuringSTR = false;
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam()";
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() powerModeValue = "<<powerModeValue;
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() triggering VehicleApPowerStateReq::SHUTDOWN_PREPARE for powerModeSTR";
        powerModeControlDuringSTR = true;
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() powerModeControlDuringSTR setting value from powerModeSTR = "
            << (powerModeControlDuringSTR ? "true" : "false");
        onPropertyValue(*createApPowerStateReq(vhal_v2_0::VehicleApPowerStateReq::SHUTDOWN_PREPARE, 2), true);
    }
    if(powerModeValue == powerModeSTRLM1){
		handleDisplayUpInterruptDuringSTR = false;
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam()";
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() powerModeValue = "<<powerModeValue;
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() triggering VehicleApPowerStateReq::SHUTDOWN_PREPARE for powerModeSTRLM1";
        powerModeControlDuringSTR = true;
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() powerModeControlDuringSTR setting value from powerModeSTRLM1= "
            << (powerModeControlDuringSTR ? "true" : "false");
        onPropertyValue(*createApPowerStateReq(vhal_v2_0::VehicleApPowerStateReq::SHUTDOWN_PREPARE, 2), true);
    }
    if(powerModeValue == powerModeSTRLM2){
		handleDisplayUpInterruptDuringSTR = false;
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam()";
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() powerModeValue = "<<powerModeValue;
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() triggering VehicleApPowerStateReq::SHUTDOWN_PREPARE for powerModeSTRLM2";
        powerModeControlDuringSTR = true;
        LOG(INFO) << "VehicleHalImpl::startSuspendToRam() powerModeControlDuringSTR setting value from powerModeSTRLM2= "
            << (powerModeControlDuringSTR ? "true" : "false");
        onPropertyValue(*createApPowerStateReq(vhal_v2_0::VehicleApPowerStateReq::SHUTDOWN_PREPARE, 2), true);
    }
}

bool VehicleHalImpl::areArraysEqual(const uint32_t* arr1, const uint32_t* arr2, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        if (arr1[i] != arr2[i]) {
            return false;
        }
    }
    return true;
}
void VehicleHalImpl::pollForTCUData(){
    uint32_t TCU_Data[TCU_InternalSignals_Index];
    m_gateway_TCU.getTCUData(TCU_Data);
    if(b_TCUsignalSignal == true)
    {
        for (int i = 0; i < TCU_InternalSignals_Index; ++i) {
            m_TCUsignalSignal[i] = TCU_Data[i];
        }
         b_TCUsignalSignal = false;
             onPropertyValue(*createInternalDataReqFrom_TCU(TCU_Data),true);
     }
     else
     {
          if (!areArraysEqual(m_TCUsignalSignal, TCU_Data,TCU_InternalSignals_Index))
          { 
          onPropertyValue(*createInternalDataReqFrom_TCU(TCU_Data),true);
          }
          for (int i = 0; i < TCU_InternalSignals_Index; ++i) {
            m_TCUsignalSignal[i] = TCU_Data[i];
        }
     }
}
void VehicleHalImpl::pollForPackagePath(){
	std::string packagePath;
    m_gateway_TCU.getPackagepath(packagePath);
		 
	if(b_Packagepath == true)
	{
		 m_Packagepath = packagePath;
		 b_Packagepath = false;
		 onPropertyValue(*createPackagePathDataReq(packagePath),true /* updateStatus */);
 	 }
	 else
	 {
		 if(m_Packagepath != packagePath)
		 {
			onPropertyValue(*createPackagePathDataReq(packagePath),true /* updateStatus */);
			 ALOGD("VehicleHalImpl : notify PackagePath data successful");
			 }
		 m_Packagepath = packagePath;
    }
 }
void VehicleHalImpl::pollforCheckforUpdateResults(){
    std::string CheckforUpdateResults;
    m_gateway_TCU.getFOTAHMICheckforUpdateResultsUpdated(CheckforUpdateResults);
    if(b_CheckforUpdateResults  == true){
        b_CheckforUpdateResults = false;
        m_CheckforUpdateResults = CheckforUpdateResults;
        onPropertyValue(*createFotahmiCheckforResultsDataReq(CheckforUpdateResults),true);
    }
    else{
        if(m_CheckforUpdateResults != CheckforUpdateResults){
            onPropertyValue(*createFotahmiCheckforResultsDataReq(CheckforUpdateResults),true);
        }
        m_CheckforUpdateResults = CheckforUpdateResults;
    }
}
void VehicleHalImpl::pollforUpdateAvailable(){
    std::string UpdateAvailable;
    m_gateway_TCU.getFOTAHMIUpdateAvailable(UpdateAvailable);
    if(b_UpdateAvailable  == true){
        b_UpdateAvailable = false;
        m_UpdateAvailable = UpdateAvailable;
        onPropertyValue(*createFotaHmiUpdateAvailableDataReq(UpdateAvailable),true);
    }
    else{
        if(m_UpdateAvailable != UpdateAvailable){
            onPropertyValue(*createFotaHmiUpdateAvailableDataReq(UpdateAvailable),true);
        }
        m_UpdateAvailable = UpdateAvailable;
    }
}
void VehicleHalImpl::pollforConditionsNotMet(){
    std::string ConditionsNotMet;
    m_gateway_TCU.getFOTAHMIConditionsNotMet(ConditionsNotMet);
    if(b_ConditionsNotMet  == true){
        b_ConditionsNotMet = false;
        m_ConditionsNotMet = ConditionsNotMet;
        onPropertyValue(*createFotaHmiConditionsnotmetDataReq(ConditionsNotMet),true);
    }
    else{
        if(m_ConditionsNotMet != ConditionsNotMet){
            onPropertyValue(*createFotaHmiConditionsnotmetDataReq(ConditionsNotMet),true);
        }
        m_ConditionsNotMet = ConditionsNotMet;
    }
}
void VehicleHalImpl::pollforInstallationStatus(){
    std::string InstallationStatus;
    m_gateway_TCU.getFOTAHMIInstallationStatus(InstallationStatus);
    if(b_InstallationStatus  == true){
        b_InstallationStatus = false;
        m_InstallationStatus = InstallationStatus;
        onPropertyValue(*createFotaHmiInstallationStatusDatareq(InstallationStatus),true);
    }
    else{
        if(m_InstallationStatus != InstallationStatus){
            onPropertyValue(*createFotaHmiInstallationStatusDatareq(InstallationStatus),true);
        }
        m_InstallationStatus = InstallationStatus;
    }
}
void VehicleHalImpl::pollforUpdateFinished(){
    std::string UpdateFinished;
    m_gateway_TCU.getFOTAHMIUpdateFinished(UpdateFinished);
    if(b_UpdateFinished  == true){
        b_UpdateFinished = false;
        m_UpdateFinished = UpdateFinished;
        onPropertyValue(*createFotaHmiUpdateFinishedDataReq(UpdateFinished),true);
    }
    else{
        if(m_UpdateFinished != UpdateFinished){
            onPropertyValue(*createFotaHmiUpdateFinishedDataReq(UpdateFinished),true);
        }
        m_UpdateFinished = UpdateFinished;
    }
}
void VehicleHalImpl::pollforWhatsNewDetails(){
    std::string WhatsNewDetails;
    m_gateway_TCU.getFOTAHMIWhatsNewDetails(WhatsNewDetails);
    if(b_WhatsNewDetails  == true){
        b_WhatsNewDetails = false;
        m_WhatsNewDetails = WhatsNewDetails;
        onPropertyValue(*createFotaHmiWhatNewDetailsDataReq(WhatsNewDetails),true);
    }
    else{
        if(m_WhatsNewDetails != WhatsNewDetails){
            onPropertyValue(*createFotaHmiWhatNewDetailsDataReq(WhatsNewDetails),true);
        }
        m_WhatsNewDetails = WhatsNewDetails;
    }
}
void VehicleHalImpl::pollforUpdateHistory(){
    std::string UpdateHistory;
    m_gateway_TCU.getFOTAHMIUpdateHistory(UpdateHistory);
    if(b_UpdateHistory  == true){
        b_UpdateHistory = false;
        m_UpdateHistory = UpdateHistory;
        onPropertyValue(*createFotaHmiUpdateHistoryResponseDataReq(UpdateHistory),true);
    }
    else{
        if(m_UpdateHistory != UpdateHistory){
            onPropertyValue(*createFotaHmiUpdateHistoryResponseDataReq(UpdateHistory),true);
        }
        m_UpdateHistory = UpdateHistory;
    }
}
void VehicleHalImpl::pollforPendingUpdates(){
    std::string PendingUpdates;
    m_gateway_TCU.getFOTAHMIPendingUpdates(PendingUpdates);
    if(b_PendingUpdates  == true){
        b_PendingUpdates = false;
        m_PendingUpdates = PendingUpdates;
        onPropertyValue(*createFotaHmiPendingUpdateHistoryDataReq(PendingUpdates),true);
    }
    else{
        if(m_PendingUpdates != PendingUpdates){
            onPropertyValue(*createFotaHmiPendingUpdateHistoryDataReq(PendingUpdates),true);
        }
        m_PendingUpdates = PendingUpdates;
    }
}
void VehicleHalImpl::pollforScheduleUpdate(){
    std::string ScheduleUpdate;
    m_gateway_TCU.getFOTAHMIScheduleUpdate(ScheduleUpdate);
    if(b_ScheduleUpdate  == true){
        b_ScheduleUpdate = false;
        m_ScheduleUpdate = ScheduleUpdate;
        onPropertyValue(*createFotaHmiUpdateScheduleDataReq(ScheduleUpdate),true);
    }
    else{
        if(m_ScheduleUpdate != ScheduleUpdate){
            onPropertyValue(*createFotaHmiUpdateScheduleDataReq(ScheduleUpdate),true);
        }
        m_ScheduleUpdate = ScheduleUpdate;
    }
}
void VehicleHalImpl::pollforFotaHmiInfoResult(){
    std::string FotaHmiInfoResult;
    m_gateway_TCU.getFotaHmiInfoResult(FotaHmiInfoResult);
    if(b_FotaHmi_InfoResult  == true){
        b_FotaHmi_InfoResult = false;
        m_FotaHmi_InfoResult = FotaHmiInfoResult;
        onPropertyValue(*createFotaHmiInfoResultDataReq(FotaHmiInfoResult),true);
    }
    else{
        if(m_FotaHmi_InfoResult != FotaHmiInfoResult){
            onPropertyValue(*createFotaHmiInfoResultDataReq(FotaHmiInfoResult),true);
        }
        m_FotaHmi_InfoResult = FotaHmiInfoResult;
    }
}
void VehicleHalImpl::pollforAOSPRBUATrigger(){
    std::string AOSPRBUATrigger;
    m_gateway_TCU.getAOSPRBUATrigger(AOSPRBUATrigger);
    if(b_AOSP_RBUA_Trigger  == true){
        b_AOSP_RBUA_Trigger = false;
        m_AOSP_RBUA_Trigger = AOSPRBUATrigger;
        onPropertyValue(*createAOSPRBUATriggerDataReq(AOSPRBUATrigger),true);
    }
    else{
        if(m_AOSP_RBUA_Trigger != AOSPRBUATrigger){
            onPropertyValue(*createAOSPRBUATriggerDataReq(AOSPRBUATrigger),true);
        }
        m_AOSP_RBUA_Trigger = AOSPRBUATrigger;
    }
}
void VehicleHalImpl::pollforUSBUpdateDetails(){
    std::string USBUpdateDetails;
    m_gateway_TCU.getUsbUpdateDetails(USBUpdateDetails);
    if(b_USBUpdateDetails  == true){
        b_USBUpdateDetails = false;
        m_USBUpdateDetails = USBUpdateDetails;
        onPropertyValue(*createUSB_Update_DetailsSignalsDataReq(USBUpdateDetails),true);
    }
    else{
        if(m_USBUpdateDetails != USBUpdateDetails){
            onPropertyValue(*createUSB_Update_DetailsSignalsDataReq(USBUpdateDetails),true);
        }
        m_USBUpdateDetails = USBUpdateDetails;
    }
}
void VehicleHalImpl::pollForSLI_Data(){
    uint32_t SLI_Data_array[SLI_data_length];
    m_gateway_TCU.getSLIData(SLI_Data_array);
    if(b_SLI_data == true)
    {
        for (int i = 0; i < SLI_data_length; ++i) {
            m_SLIDataSignal[i] = SLI_Data_array[i];
        }
        b_SLI_data = false;
             onPropertyValue(*createSLIDataReqFrom_TCU(SLI_Data_array),true);
     }
     else
     {
          if (!areArraysEqual(m_SLIDataSignal, SLI_Data_array,SLI_data_length))
          { 
          onPropertyValue(*createSLIDataReqFrom_TCU(SLI_Data_array),true);
          }
          for (int i = 0; i < SLI_data_length; ++i) {
            m_SLIDataSignal[i] = SLI_Data_array[i];
        }
     }
}
 
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createSpeedDataReq(float param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::SPEED_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createSpeedDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createPassengerConfirmStsDataReq(uint8_t Passenger_confirmSts) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::PASSENGER_CONFIRM_STS_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = Passenger_confirmSts;
	
	ALOGD("VehicleHalImpl:: Passenger_confirmSts : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCapacityUnitDataReq(uint32_t param) {	
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CAPACITYUNIT_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createCapacityUnitDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createPowermodeDataReq(uint32_t powerMode) {
    //LVDS requirement REQ-0041562 A handling during power on
    if(handlePowerStatusForLVDSManagerAfterDeepSleepWakeUp == true){
        int32_t powerOnResult = LVDSMgr_SetProperty(LVDSMgr_MODE_POWER_STATUS, true);
        LOG(INFO) << "VehicleHalImpl::createPowermodeDataReq() powerOnResult = "<< powerOnResult;
        LOG(INFO) << "VehicleHalImpl::createPowermodeDataReq() powerMode value during startup after LM3/DeepSleep = "<< powerMode;
        handlePowerStatusForLVDSManagerAfterDeepSleepWakeUp = false;
        LOG(INFO) << "VehicleHalImpl::createPowermodeDataReq() resetting handlePowerStatusForLVDSManagerAfterDeepSleepWakeUp value = "
            << (handlePowerStatusForLVDSManagerAfterDeepSleepWakeUp ? "true" : "false");                   
    }
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::POWERMODE_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;

    //LVDS requirement REQ-0041562 A handling during power off
    if( (powerMode == powerModeSeventeen) || (powerMode == powerModeEighteen) ){
        int32_t powerOffResult = LVDSMgr_SetProperty(LVDSMgr_MODE_POWER_STATUS, false);
        LOG(INFO) << "VehicleHalImpl::createPowermodeDataReq() powerOffResult = "<< powerOffResult;
        LOG(INFO) << "VehicleHalImpl::createPowermodeDataReq() powerMode value during LM3/DeepSleep = "<< powerMode;
    }

    //Cancel STR if powerModeNine or powerModeSix triggers when STR is in progress
    if(powerModeControlDuringSTR == true){ 
        if((powerMode == powerModeNine) || (powerMode == powerModeSix)){
            flagForPowerModeToCancelSTR = true;
            LOG(INFO) << "createPowermodeDataReq() flagForPowerModeToCancelSTR value = "<< (flagForPowerModeToCancelSTR ? "true" : "false");
            LOG(INFO) << "createPowermodeDataReq() powerMode value initiated during STR = "<< powerMode;
        }
    }

    //Block the powermodes when STR is in progress
    if(powerModeControlDuringSTR == true){
        LOG(INFO) << "createPowermodeDataReq() powerModeControlDuringSTR value = "<< (powerModeControlDuringSTR ? "true" : "false");
        req->value.int32Values[0] = powerModeDuringSTR;
    } 
    else{
        req->value.int32Values[0] = powerMode;
    }
    if(m_powermode != powerMode){
        startSuspendToRam(powerMode);	
    }
    DLT_LOG(SYIN, DLT_LOG_DEBUG, DLT_STRING("Hello DLT_LOG_DEBUG!"));
    m_powermode = powerMode;
    ALOGD("VehicleHalImpl:: createPowermodeDataReq : %d:" , req->value.int32Values[0]);	
    DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("ECU Power State : "), DLT_UINT32(powerMode));
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createParkLampDataReq(uint32_t param) {	
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::PARKLAMP_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createParkLampDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createIgnitionStsDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IGNITIONSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createIgnitionStsDataReq : %d:" , req->value.int32Values[0]);	
  DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Ignition status"));
  LOG(INFO) << "STLA_LOGS :: Ignition Status: "<< param;
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createEcallCrashDataReq(uint32_t param) {	
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::ECALLCRASH_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createEcallCrashDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDayNightModeDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::DAYNIGHTMODE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createDayNightModeDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFuelTypeDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FUELTYPE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFuelTypeDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFPASStsDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FPASSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFPASStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createPulseCountRRWheelDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::PULSECOUNTRRWHEEL_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createPulseCountRRWheelDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createPulseCountRLWheelDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::PULSECOUNTRLWHEEL_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createPulseCountRLWheelDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createPulseCountFRWheelDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::PULSECOUNTFRWHEEL_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createPulseCountFRWheelDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createPulseCountFLWheelDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::PULSECOUNTFLWHEEL_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createPulseCountFLWheelDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createRLWheelTickCounterFaultDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::RLWHEELTICKCOUNTERFAULT_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createRLWheelTickCounterFaultDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createRRFWheelTickCounterFaultDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::RRWHEELTICKCOUNTERFAULT_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createRRFWheelTickCounterFaultDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}


VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFRWheelTickCounterFaultDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FRWHEELTICKCOUNTERFAULT_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFRWheelTickCounterFaultDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createAutonomyDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::AUTONOMY_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createAutonomyDataReq : %d:" , req->value.int32Values[0]);	
    return req;	
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDrivingDirectionDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::DRIVINGDIRECTION_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createDrivingDirectionDataReq : %d:" , req->value.int32Values[0]);	
    return req;	
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createGrossYawRateDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::GROSSYAWRATE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createGrossYawRateDataReq : %d:" , req->value.int32Values[0]);	
    return req;	
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createYawSpeedDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::YAWSPEED_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createYawSpeedDataReq : %d:" , req->value.int32Values[0]);	
    return req;	
}


VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createTripAvgBattPowerConsReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::TRIPAVGBATTPOWERCONS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createTripAvgBattPowerConsReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createTransmGearDisplayReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::REVERSEGEARENGAGED_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createTransmGearDisplayReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createTotalKMReq(uint32_t param) {	
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::TOTALKM_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createTotalKMReq : %d:" , req->value.int32Values[0]);	
    return req;
}


VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createSteeringAngleReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::STEERINGANGLE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createSteeringAngleReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createReverseGearEngagedDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::REVERSEGEARENGAGED_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createReverseGearEngagedDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createRoadSlopeDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::ROADSLOPE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createRoadSlopeDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}



VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createBrakePedalrStsDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::BRAKEPEDALRSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createBrakePedalrStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInstantFuelConsDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::INSTANTFUELCONS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createInstantFuelConsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFLWheelTickCounterFaultDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FLWHEELTICKCOUNTERFAULT_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFLWheelTickCounterFaultDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createParkBreakStatusDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::PARKBREAKSTATUS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createParkBreakStatusDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFuelLevalDataReq(uint32_t param) {	
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FUELLEVAL_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFuelLevalDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCanDistanceUnitDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CANDISTANCEUNIT_VENDOR  );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createCanDistanceUnitDataReq: %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createHVBatteryIsChargingDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::HVBATTERYISCHARGING_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createHVBatteryIsChargingDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createHVBatteryChargeLevelDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::HVBATTERYCHARGELEVEL_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createHVBatteryChargeLevelDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}


VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createHVBatteryMaxRangeDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::HVBATTERYMAXRANGE_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createHVBatteryMaxRangeDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createHVBatterySOHDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::HVBATTERYSOH_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl::  createHVBatterySOHDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createHVBatteryPercentageDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::HVBATTERYPERCENTAGE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createHVBatteryPercentageDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createHVBatteryMinChargeDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::HVBATTERYMINCHARGE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createHVBatteryMinChargeDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createHVBatteryActiveConnectorDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::HVBATTERYACTIVECONNECTOR_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createHVBatteryActiveConnectorDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCombustFuelLvDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::COMBUSTFUELLV_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createCombustFuelLvDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCompressorStsDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::COMPRESSORSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createCompressorStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createElectricMotorPowerDataReq(int32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::ELECTRICMOTORPOWER_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createElectricMotorPowerDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createEngineCoolantTempDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::ENGINECOOLANTTEMP_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createEngineCoolantTempDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createEngineRPMDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::ENGINERPM_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createEngineRPMDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFuelLevelMinimumStsDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FUELLEVELMINIMUMSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFuelLevelMinimumStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createAbsSteeringWheelAngleAcuDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::ABSSTEERINGWHEELANGLEACU_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createAbsSteeringWheelAngleAcuDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInitSteeringWheenAngleFlagDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::INITSTEERINGWHEENANGLE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createInitSteeringWheenAngleFlagDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createNetworkMNGTDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::NETWORKMNGT_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createNetworkMNGTDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFPASCenterLeftBarStsDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FPASCENTERLEFTBARSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFPASCenterLeftBarStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFPASCenterRightBarStsDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FPASCENTERRIGHTBARSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFPASCenterRightBarStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFPASLeftBarStsDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FPASLEFTBARSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFPASLeftBarStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFPASRightBarStsDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FPASRIGHTBARSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFPASRightBarStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createRPASCenterLeftBarStsDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::RPASCENTERLEFTBARSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createRPASCenterLeftBarStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createRPASCenterRightBarStsDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::RPASCENTERRIGHTBARSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createRPASCenterRightBarStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createRPASLeftBarStsDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::RPASLEFTBARSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createRPASLeftBarStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createTrailerPresentDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::TRAILERPRESENT_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createTrailerPresentDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createTrunkDoorStsDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::TRUNKDOORSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createTrunkDoorStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createRPASStsDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::RPASSTS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createRPASStsDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createEconShftRqDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::ECONSHFTRQ_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createEconShftRqDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createEngineTorqueDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::ENGINETORQUE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createEngineTorqueDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFuelConsUnitDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FUELCONSUNIT_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createFuelConsUnitDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createSystemTimeStampDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::STYSTEM_TIME_STAMP_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createSystemTimeStampDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createRegeneratedEnergyDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::REGENERATEDENERGY_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createRegeneratedEnergyDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createShiftLeverPositionDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::SHIFTLEVERPOSITION_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
  DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Shift Lever Status : "), DLT_UINT32(param));
	LOG(INFO) << "STLA_LOGS :: Shift Lever Status: "<< param;
	ALOGD("VehicleHalImpl:: createShiftLeverPositionDataReq : %d:" , req->value.int32Values[0]);
  	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createTripAverageFuelConsumptionDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::TRIPAVERAGEFUELCONSUMPTION_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createTripAverageFuelConsumptionDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createReqVolCtrlDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::REQVOLCTRL_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createReqVolCtrlDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createVehPrivacyModeDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::VEH_PRIVACYMODE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createVehPrivacyModeDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createPhonecallstatusDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVIPHONECALLSTATUS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createPhonecallstatusDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createTemperatureDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::TEMPERATURE_MODE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createTemperatureDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createSupplyVoltageDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::SUPPLY_VOLTAGE_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createSupplyVoltageDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCallActiveSTRDisplayPopupStatusDataReq(uint32_t callActiveSTRDisplayPopupStatus){			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CALL_ACTIVE_STR_DISPLAY_POPUP_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = callActiveSTRDisplayPopupStatus;
	ALOGD("VehicleHalImpl::createCallActiveSTRDisplayPopupStatusDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createSTRCancelShutdownStatusDataReq(uint32_t STRCancelShutdownStatus){	
    LOG(INFO) << "VehicleHalImpl::createSTRCancelShutdownStatusDataReq() STRCancelShutdownStatus : " << STRCancelShutdownStatus;	
    cancelShutdownSTRInputFromMCU = STRCancelShutdownStatus;
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::STR_CANCEL_SHUTDOWN_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = STRCancelShutdownStatus;
	ALOGD("VehicleHalImpl::createSTRCancelShutdownStatusDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createSatelliteNumberDataReq(uint32_t param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::SATELLITES_NUMBER_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createSatelliteNumberDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createlatitudeDataReq(float param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::FLOAT, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::LATITUDE_DATA_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.floatValues[0] = param;
	
	ALOGD("VehicleHalImpl:: createlatitudeDataReq : %f:" , req->value.floatValues[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createlongitudeDataReq(float param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::FLOAT, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::LONGITUDE_DATA_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.floatValues[0] = param;
	
	ALOGD("VehicleHalImpl:: createlongitudeDataReq : %f:" , req->value.floatValues[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createcurrentelevationDataReq(float param) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::FLOAT, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CURRENT_ELEVATION_DATA_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.floatValues[0] = param;
	
	ALOGD("VehicleHalImpl:: createcurrentelevationDataReq : %f:" , req->value.floatValues[0]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createMapVersionDataReq(const std::string& param) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::MAP_VERSION_DATA_VENDOR);
    req->areaId = 0;
    req->timestamp = android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
    req->value.stringValue = param;

    ALOGD("VehicleHalImpl:: createMapVersionDataReq : %s:", req->value.stringValue.c_str());
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createPOI_databaseversionDataReq(const std::string& param) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::POI_DATABASE_VERSION_VENDOR);
    req->areaId = 0;
    req->timestamp = android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
    req->value.stringValue = param;

    ALOGD("VehicleHalImpl:: createPOI_databaseversionDataReq : %s", req->value.stringValue.c_str());
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createlastUpdateTimeDataReq(const std::string& param) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::LASTUPDATE_TIME_VENDOR);
    req->areaId = 0;
    req->timestamp = android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
    req->value.stringValue = param;

    ALOGD("VehicleHalImpl:: createlastUpdateTimeDataReq : %s", req->value.stringValue.c_str());
    DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("IVIHMITIME : "), DLT_STRING(param.c_str()));
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createoutTempDataReq(uint32_t param) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::OUTTEMP_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = param;
	
	ALOGD("VehicleHalImpl:: createoutTempDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFactoryResetCountDataReq(uint32_t FactoryresetCount) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FACTORY_RESET_COUNT );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = FactoryresetCount;
	
	ALOGD("VehicleHalImpl:: createFactoryResetCountDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createBCMsecureTimeDataReq(uint32_t param_secure[]) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, 7);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::BCMSECURTIME_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
    for(unsigned i = 0; i < SECUREtime_index; i++)	
    req->value.int32Values[i] = param_secure[i];
    for(unsigned i = internalsystemTime::SystemSecureTimeHr,j = 1; i <= internalsystemTime::SystemSecureTimeYr,j <= (SECUREtime_index-1); i++ ,j++)
    {
       InternalSystemTime[i] = param_secure[j];
	   ALOGD("VehicleHalImpl::InternalSystemTime %d" , InternalSystemTime[i]);
    }
    createInternalSystemtimeDataReq(InternalSystemTime);
	for(unsigned i = 0; i < SECUREtime_index; i++)
	ALOGD("VehicleHalImpl:: createBCMsecureTimeDataReq : %d  %d:" , i ,req->value.int32Values[i]);
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createAudioAdasDataReq(std::vector<uint8_t> audio_adas_signal){		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, audio_adas_signal.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::AUDIO_ADAS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    for(unsigned i = 0; i < audio_adas_signal.size(); i++)
	{
		req->value.int32Values[i] = audio_adas_signal[i];
			ALOGD("VehicleHalImpl:: createAudioAdasDataReq : %d %d :" , i,req->value.int32Values[i]);	
	}
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createIVIThemeVendorDataReq(uint32_t IVI_Theme) {		
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVITHEME_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = IVI_Theme;
	
	ALOGD("VehicleHalImpl:: createIVIThemeVendorDataReq : %d:" , req->value.int32Values[0]);	
    return req;
}
 VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createBCMHMIDataReq(uint32_t param_HMI[]) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, 6);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::BCMHMITIME_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
    for(unsigned i = 0; i < HMItime_index; i++)    
    req->value.int32Values[i] = param_HMI[i];
    for(unsigned i = internalsystemTime::SystemUserTimeHr,j = 1; i <= internalsystemTime::SystemUserTimeYr,j <= (HMItime_index-1); i++ ,j++)
    {
       InternalSystemTime[i] = param_HMI[j];
	   ALOGD("VehicleHalImpl::InternalSystemTime %d" , InternalSystemTime[i]);
    }
    createInternalSystemtimeDataReq(InternalSystemTime);
	
	
	for(unsigned i = 0; i < HMItime_index; i++)
	ALOGD("VehicleHalImpl:: createBCMHMIDataReq : %d  %d:" , i,req->value.int32Values[i]);	
    return req;
}
 VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDotteParamDataReq(std::vector<uint32_t>& dotteParam) {			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, dotteParam.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::DOTTEPARAM_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < dotteParam.size(); i++)
	{
		req->value.int32Values[i] = dotteParam[i];
			ALOGD("VehicleHalImpl:: createDotteParamDataReq : %d %d :" , i,req->value.int32Values[i]);	
	}
	    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createUSB_UpdateSignalsDataReq(std::vector<uint16_t>& USB_Update) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, USB_Update.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::USBUPDATE_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < USB_Update.size(); i++)
	{
		req->value.int32Values[i] = USB_Update[i];
       ALOGD("VehicleHalImpl:: createUSB_UpdateSignalsDataReq : %d %d :" , i,req->value.int32Values[i]);	 
	}
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createIMC_MCUUSB_UpdateSignalsDataReq(std::vector<uint16_t>& USB_Update) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, USB_Update.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IMCMCU_USBUPDATE_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < USB_Update.size(); i++)
	{
		req->value.int32Values[i] = USB_Update[i];
       ALOGD("VehicleHalImpl:: createIMC_MCUUSB_UpdateSignalsDataReq: %d %d :" , i,req->value.int32Values[i]);	 
	}
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createUSB_Update_DetailsSignalsDataReq(const std::string& m_USBUpdate_Details) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::USBUPDATE_DETAILS_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
    req->value.stringValue = m_USBUpdate_Details;
    ALOGD("VehicleHalImpl:: createUSB_Update_DetailsSignalsDataReq : %s:" ,m_USBUpdate_Details.c_str());
	
    return req;
}

 VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDTCParamDataReq(std::vector<uint32_t>& dtcParam) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, dtcParam.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::DTCSIGNAL_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < dtcParam.size(); i++)
	{
		req->value.int32Values[i] = dtcParam[i];
        ALOGD("VehicleHalImpl:: createDTCParamDataReq : %d %d :" , i,req->value.int32Values[i]);	
	}
    return req;
}
 VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDLTParamDataReq(std::vector<uint32_t>& dLTParam) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, dLTParam.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::DLTSIGNAL_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < dLTParam.size(); i++)
	{
		req->value.int32Values[i] = dLTParam[i];
    DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("HandBrake Status : "), DLT_UINT32(dLTParam[0]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("HeadLight Status : "), DLT_UINT32(dLTParam[1]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Steering Wheel Control Change : "), DLT_UINT32(dLTParam[2]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Temp Sensor Outputs : "), DLT_UINT32(dLTParam[3]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("System Interface Failures : "), DLT_UINT32(dLTParam[4]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("LoudSpeaker Status : "), DLT_UINT32(dLTParam[5]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Microphone Status : "), DLT_UINT32(dLTParam[6]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Gyro Accelerometer status : "), DLT_UINT32(dLTParam[7]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Active ECU DTC's : "), DLT_UINT32(dLTParam[8]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Bub Charge Percentage : "), DLT_UINT32(dLTParam[9]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Bub Overcurrent status : "), DLT_UINT32(dLTParam[10]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Bub DisConnection status : "), DLT_UINT32(dLTParam[11]));
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("EMMC HEALTH From _catproc/quec_emmc_life : 0x01"));
    ALOGD("VehicleHalImpl:: createDLTParamDataReq : %d %d :" , i,req->value.int32Values[i]);
	}
    return req;
}
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createGraphElecConsHistDataReq(uint32_t graph_elec_cons_hist) 
   {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::GPSHISTOGRAPH_DISTELEC_VENDOR);
    req->areaId = 0;
    req->timestamp = android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
    req->value.int32Values[0] = graph_elec_cons_hist;

    ALOGD("VehicleHalImpl:: createGraphElecConsHistDataReq : %d", req->value.int32Values[0]);
    return req;
    }

    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createChargeTypeReqHMIDataReq(uint32_t charge_type_req) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CGTRTABTYPERECHARGE_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.int32Values[0] = charge_type_req;

        ALOGD("VehicleHalImpl:: createChargeTypeReqHMIDataReq : %d", req->value.int32Values[0]);
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createStartTimeDelayedChargeHMIDataReq(uint32_t start_time_delayed_charge) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CGTRTABRECHARGE_HDEBUT_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.int32Values[0] = start_time_delayed_charge;

        ALOGD("VehicleHalImpl:: createStartTimeDelayedChargeHMIDataReq : %d", req->value.int32Values[0]);
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createChargeMenuDelayStsDataReq(uint32_t charge_menu_delay_sts) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::TEMPORTAB_RECHARGE_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.int32Values[0] = charge_menu_delay_sts;

        ALOGD("VehicleHalImpl:: createChargeMenuDelayStsDataReq : %d", req->value.int32Values[0]);
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createVehGearConfigDataReq(uint32_t veh_gear_config) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::VEHGEARCONFIG_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.int32Values[0] = veh_gear_config;

        ALOGD("VehicleHalImpl:: createVehGearConfigDataReq : %d", req->value.int32Values[0]);
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createVehGearNetDataReq(uint32_t veh_gear_net) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::VEHGEAR_INT_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.int32Values[0] = veh_gear_net;

        ALOGD("VehicleHalImpl:: createVehGearNetDataReq : %d", req->value.int32Values[0]);
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createVehPosIntDataReq(uint32_t veh_pos_int) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::VEHGEARPOS_INT_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.int32Values[0] = veh_pos_int;

        ALOGD("VehicleHalImpl:: createVehPosIntDataReq : %d", req->value.int32Values[0]);
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createReverseGearStsDataReq(uint32_t reverse_gear_sts) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::REVERSEGEARSTS_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.int32Values[0] = reverse_gear_sts;

        ALOGD("VehicleHalImpl:: createReverseGearStsDataReq : %d", req->value.int32Values[0]);
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createManualParkBreakStsDataReq(uint32_t manual_park_break_sts) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::MANUALPARKBREAKSTS_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.int32Values[0] = manual_park_break_sts;

        ALOGD("VehicleHalImpl:: createManualParkBreakStsDataReq : %d", req->value.int32Values[0]);
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createParkBreakStsEstDataReq(uint32_t park_break_sts_est) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::PARKBREAKSTSEST_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.int32Values[0] = park_break_sts_est;

        ALOGD("VehicleHalImpl:: createParkBreakStsEstDataReq : %d", req->value.int32Values[0]);
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFotaHmiInfoResultDataReq(const std::string&  m_FOTAHMI_Info_Result){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FOTAHMI_INFO_RESULT_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_FOTAHMI_Info_Result;
        ALOGD("VehicleHalImpl:: createFotaHmiInfoResultDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createGnssSignalStrengthDataReq(int32_t gnss_signal_strength) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::GNSSSIGNALSTRENGTH_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.int32Values[0] = gnss_signal_strength;

        ALOGD("VehicleHalImpl:: createGnssSignalStrengthDataReq : %d", req->value.int32Values[0]);
        return req;
    }


    std::string asciiToSentence(const std::string& str, int len) {
    int num = 0;
    std::string result; // Use a string to store the result

    for (int i = 0; i < len; i++) {
        // Append the current digit
        num = num * 10 + (str[i] - '0');

        // If num is within the required range
        if (num >= 32 && num <= 122) {
            // Convert num to char and append to result
            result += static_cast<char>(num);

            // Reset num to 0
            num = 0;
        }
    }

    return result; // Return the constructed string
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDLT_VectorParamDataReq(std::vector<uint8_t> vec_arr) {
	//ALOGD("VehicleHalImpl : createDLT_VectorParamDataReq()");			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, vec_arr.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::DLT_VECTORVALUE_SIGNAL_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
   	std::string mcpu_vcpu_sw_ver;
   	std::string ecu_serial_num;
	  std::string ecu_hw_ver_num;
  	std::string ecu_hw_part_num;
	  std::string ecu_hw_num;
	  std::string ecu_sw_ver_num;
	  std::string ecu_sw_part_num;
	  std::string ecu_sw_num;
	  std::string ecu_spare_part_num;
	for(unsigned i = 0; i < vec_arr.size(); i++)
	{
			req->value.int32Values[i] = vec_arr[i];
            ALOGD("VehicleHalImpl:: createDLT_VectorParamDataReq : %d  %d :" , i,vec_arr[i]);
	}
      if (!vec_arr.empty() && vec_arr[0] == 1) {
        // Append elements from index 1 to the end of the vector
        for (unsigned i = 1; i < vec_arr.size(); i++) {
            mcpu_vcpu_sw_ver.append(std::to_string(vec_arr[i]));
            if (i < vec_arr.size() - 1) {
                mcpu_vcpu_sw_ver.append("");
            }
        }
        // Use DLT_LOG to log the assembled string
        DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("mcpuVcpuSwVersion : "), DLT_STRING(mcpu_vcpu_sw_ver.c_str()));
    }
       if (!vec_arr.empty() && vec_arr[0] == 2) {
        // Append elements from index 1 to the end of the vector
        for (unsigned i = 1; i < vec_arr.size(); i++) {
            ecu_serial_num.append(std::to_string(vec_arr[i]));
            if (i < vec_arr.size() - 1) {
                ecu_serial_num.append("");
            }
        }
        // Use DLT_LOG to log the assembled string
        DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("ECU Serial Number : "), DLT_STRING(ecu_serial_num.c_str()));
    }
		if (!vec_arr.empty() && vec_arr[0] == 3) {
        // Append elements from index 1 to the end of the vector
        for (unsigned i = 1; i < vec_arr.size(); i++) {
            ecu_hw_ver_num.append(std::to_string(vec_arr[i]));
            if (i < vec_arr.size() - 1) {
                ecu_hw_ver_num.append("");
            }
        }
        // Use DLT_LOG to log the assembled string
        DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("ECU HW Version Number : "), DLT_STRING(ecu_hw_ver_num.c_str()));
    }
    if(!vec_arr.empty() && vec_arr[0] == 4){
		 for(unsigned i = 1;i<vec_arr.size();i++){
			 ecu_hw_part_num.append(std::to_string(vec_arr[i]));
			 if(i < vec_arr.size() - 1){
				 ecu_hw_part_num.append("");
			 }
		 }
        DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("ECU HW Part Number : "), DLT_STRING(ecu_hw_part_num.c_str()));		 
	 }
	 if(!vec_arr.empty() && vec_arr[0] == 5){
		 for(unsigned i = 1;i < vec_arr.size();i++){
			 ecu_hw_num.append(std::to_string(vec_arr[i]));
		 if(i < vec_arr.size()-1){
			 ecu_hw_num.append("");
		    }
	    }
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("ECU HW Number : "), DLT_STRING(ecu_hw_num.c_str()));	
	 }
	 if(!vec_arr.empty() && vec_arr[0] == 6){
		 for(unsigned i = 1;i < vec_arr.size();i++){
			 ecu_sw_ver_num.append(std::to_string(vec_arr[i]));
		 if(i < vec_arr.size()-1){
			 ecu_sw_ver_num.append("");
		    }
	    }
		std::string SW_VER = asciiToSentence(ecu_sw_ver_num,ecu_sw_ver_num.length());
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("ECU SW Version Number : "), DLT_STRING(SW_VER.c_str()));	
	 }
	 	if(!vec_arr.empty() && vec_arr[0] == 7){
		 for(unsigned i = 1;i < vec_arr.size();i++){
			 ecu_sw_part_num.append(std::to_string(vec_arr[i]));
		 if(i < vec_arr.size()-1){
			 ecu_sw_part_num.append("");
		    }
	    }
		std::string SW_part = asciiToSentence(ecu_sw_part_num,ecu_sw_part_num.length());
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("ECU SW Part Number : "), DLT_STRING(SW_part.c_str()));	
	 }
	    if(!vec_arr.empty() && vec_arr[0] == 8){
		 for(unsigned i = 1;i < vec_arr.size();i++){
			 ecu_sw_num.append(std::to_string(vec_arr[i]));
		 if(i < vec_arr.size()-1){
			 ecu_sw_num.append("");
		    }
	    }
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("ECU SW Number : "), DLT_STRING(ecu_sw_num.c_str()));	
		
	 }
	 if(!vec_arr.empty() && vec_arr[0] == 9){
		 for(unsigned i = 1;i < vec_arr.size();i++){
			 ecu_spare_part_num.append(std::to_string(vec_arr[i]));
		 if(i < vec_arr.size()-1){
			 ecu_spare_part_num.append("");
		    }
	    }
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("ECU Spare Part Number : "), DLT_STRING(ecu_spare_part_num.c_str()));	
	 }
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createinternalSignalDataReq(uint32_t internalSignal[]) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, INTERNALSignal_index);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::INTERNALCANSIG_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < INTERNALSignal_index; i++)
    {
    req->value.int32Values[i] = internalSignal[i];
    ALOGD("VehicleHalImpl:: createinternalSignalDataReq : %d %d :" , i,req->value.int32Values[i]);	
    }
    uint32_t privacy_mode = internalSignal[PrivacyMode];
    m_gateway_TCU.Send_Privacy_mode_sts(privacy_mode);
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createEVSignalsDataReq(int32_t EVSignal[]) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, EVSignals_index);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::EV_SIGNAL_SDK_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < EVSignals_index; i++)
    {
    req->value.int32Values[i] = EVSignal[i];
    ALOGD("VehicleHalImpl:: createEVSignalsDataReq : %d %d :" , i,req->value.int32Values[i]);
    }	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCanCommonDataReq(int32_t CanCommonSignal[]) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, CanCommonSignal_index);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CAN_COMMON_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < CanCommonSignal_index; i++)
    {
    req->value.int32Values[i] = CanCommonSignal[i];
    ALOGD("VehicleHalImpl:: createCanCommonDataReq : %d %d :" , i,req->value.int32Values[i]);
    }	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCameraSignalsDataReq(uint32_t CameraSignal[]) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, CameraSignals_index);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::BCM_SIGNALS_CAMERA_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < CameraSignals_index; i++)
    req->value.int32Values[i] = CameraSignal[i];
	
	for(unsigned i = 0; i < CameraSignals_index; i++)
	ALOGD("VehicleHalImpl:: createCameraSignalsDataReq : %d %d :" , i,req->value.int32Values[i]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalCameraSignalsDataReq(std::vector<uint32_t>& camera_data) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, internal_camera_length);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_CAMERA_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    for(unsigned i = 0; i < internal_camera_length; i++)
    req->value.int32Values[i] = camera_data[i];
	int array_size = sizeof(req->value.int32Values)/sizeof(req->value.int32Values[0]);
	for(int i = 0; i < internal_camera_length; i++)
	ALOGD("VehicleHalImpl:: createInternalCameraSignalsDataReq : %d %d :" , i,req->value.int32Values[i]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalAudioSignalsDataReq(std::vector<uint32_t>& audio_data) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, internal_audio_length);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_AUDIO_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    for(unsigned i = 0; i < internal_audio_length; i++)
    req->value.int32Values[i] = audio_data[i];
	int array_size = sizeof(req->value.int32Values)/sizeof(req->value.int32Values[0]);
	for(int i = 0; i < internal_audio_length; i++)
	ALOGD("VehicleHalImpl:: createInternalAudioSignalsDataReq : %d %d :" , i,req->value.int32Values[i]);	
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalSecondPartyAppSignalsDataReq(std::vector<uint32_t>& Second_party_app) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, Second_party_app.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_SECOND_PARTY_APP_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    for(unsigned i = 0; i < Second_party_app.size(); i++)
    req->value.int32Values[i] = Second_party_app[i];
	int array_size = sizeof(req->value.int32Values)/sizeof(req->value.int32Values[0]);
	for(int i = 0; i < internal_second_party_App_length; i++)
	ALOGD("VehicleHalImpl:: createInternalSecondPartyAppSignalsDataReq : %d %d :" , i,req->value.int32Values[i]);	
    return req;
}

VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createSiviGnssGsmAvailableStatusDataReq(uint32_t sivi_gnssgsm_data) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::SIVIGNSSGSM_AVAILABILITY_STATUS_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;       
    req->value.int32Values[0] = sivi_gnssgsm_data;
	
	ALOGD("VehicleHalImpl:: createSiviGnssGsmAvailableStatusDataReq : %d:" , req->value.int32Values[0]);
    if(sivi_gnssgsm_data == 0 || sivi_gnssgsm_data == 2){
        DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("GNSS is connected if it is 0 or 2 : "), DLT_UINT32(sivi_gnssgsm_data));
    }
    if(sivi_gnssgsm_data == 1 || sivi_gnssgsm_data == 3){
        DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("GNSS is not connected if it is 1 or 3 : "), DLT_UINT32(sivi_gnssgsm_data));
    }
    
    return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalHmiSignalsDataReq(std::vector<uint32_t>& hmi_data) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, internal_hmi_length);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_HMI_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    for(unsigned i = 0; i < internal_hmi_length; i++)
	{
    req->value.int32Values[i] = hmi_data[i];
	ALOGD("VehicleHalImpl:: createInternalHmiSignalsDataReq : %d %d :" , i,req->value.int32Values[i]);
	}  
      
	return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalProjectionDataReq(std::vector<uint32_t>& internalProjection) {
    LOG(INFO) << "VHAL VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalProjectionDataReq()";
    LOG(INFO) << "VHAL VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalProjectionDataReq() thread id = "<< std::this_thread::get_id();
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, internal_projection_length);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_PROJECTION_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
    for(unsigned i = 0; i < internal_projection_length; i++){  
        req->value.int32Values[i] = internalProjection[i];
		ALOGD("VHAL VehicleHalImpl:: createInternalProjectionDataReq : %d  %d:" , i, req->value.int32Values[i]);
    }
     return req;
}
VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalConnectivityDataReq(std::vector<uint32_t>& internalConnectivity) {
    LOG(INFO) << "VHAL VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalConnectivityDataReq()";
    LOG(INFO) << "VHAL VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalConnectivityDataReq() thread id = "<< std::this_thread::get_id();
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, internalConnectivity.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVI_INTERNAL_CONNECTIVITY_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
    for(unsigned i = 0; i < internalConnectivity.size(); i++){  
        req->value.int32Values[i] = internalConnectivity[i];
		 ALOGD("VHAL VehicleHalImpl:: createInternalConnectivityDataReq : %d  %d:" , i, req->value.int32Values[i]);	
    }
	return req;
}


VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createBCMSignalsDataReq(uint32_t BCMSignal[]) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, BCMSignals_index);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::BCM_CAN_SDK__VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < BCMSignals_index; i++)
    req->value.int32Values[i] = BCMSignal[i];
	
	for(unsigned i = 0; i < BCMSignals_index; i++)
	ALOGD("VehicleHalImpl:: createBCMSignalsDataReq : %d %d :" , i,req->value.int32Values[i]);	
    return req;
}

 VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createVinNumberDataReq(std::vector<uint32_t>& vinNumber){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, vinNumber.size());
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::VINNUMBER_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        std::string final_vin;
	    for(unsigned i = 0; i < vinNumber.size(); i++){
        req->value.int32Values[i] = vinNumber[i];
        ALOGD("VehicleHalImpl:: createVinNumberDataReq : %d  %d :" ,i,vinNumber[i]);
        		
		if(!vinNumber.empty()){
		if(i>3 && i<=21){
			final_vin.append(std::to_string(vinNumber[i]));
          if(i<21){
              	final_vin.append("");		  
		              }
                    }
		                      }
		} 
		
		DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("VIN Number Vendor : "), DLT_STRING(final_vin.c_str()));
		
        return req;
    }
	
	 VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCALDataReq(uint32_t calData[]){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, CALDATA);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CALIBRATION_DATA_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
	    for(unsigned i = 0; i <= CALDATA; i++){
        req->value.int32Values[i] = calData[i];
        ALOGD("VehicleHalImpl:: createCALDataReq : %d  %d :" ,i,req->value.int32Values[i]);
		}
        return req;
    }
		 VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCALDataReq_Audio(uint32_t calData[]){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, CALDATA_AUDIO);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CALIBRATION_DATA_AUDIO_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
	    for(unsigned i = 0; i <= CALDATA_AUDIO; i++){
        req->value.int32Values[i] = calData[i];
        ALOGD("VehicleHalImpl:: createCALDataReq_Audio : %d  %d :" ,i,req->value.int32Values[i]);
		}
	//	calData.clear();
        return req;
    }
		 VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCALDataReq_Camera(uint32_t calData[]){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, CALDATA_CAMERA);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CALIBRATION_DATA_CAMERA_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
	    for(unsigned i = 0; i <= CALDATA_CAMERA; i++){
        req->value.int32Values[i] = calData[i];
        ALOGD("VehicleHalImpl:: createCALDataReq_Camera : %d  %d :" ,i,req->value.int32Values[i]);
		}
	//	calData.clear();
        return req;
    }
	    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCALDataReq_Array(std::vector<uint8_t>& calData){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, calData.size());
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CALIBRATION_ARRAY_DATA_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
	    for(unsigned i = 0; i < calData.size(); i++){
        req->value.int32Values[i] = calData[i];
        ALOGD("VehicleHalImpl:: createCALDataReq_Array : %d  %d :" ,i,req->value.int32Values[i]);
		}
		calData.clear();
        return req;
    }
    /*  VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCALSrecDataReq(std::vector<uint32_t> calSrecDATA){
    	ALOGD("VehicleHalImpl : createCALSrecDataReq()");
       size_t vectorSize = calSrecDATA.size();
       auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, vectorSize);
       req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CALDATA_SRECFILE_VENDOR);
       req->areaId = 0;
       req->timestamp = android::elapsedRealtimeNano();
       req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
	   for(unsigned i = 0; i < calSrecDATA.size(); i++){
       req->value.int32Values[i] = calSrecDATA[i];
       ALOGD("VehicleHalImpl:: createCALSrecDataReq : %d  %d :" ,i,req->value.int32Values[i]);
	   }
        calSrecDATA.clear();  
        return req;
    } */
	    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createIMUDataReq(std::vector<int32_t>& imuData){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, imuData.size());
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IMU_DATA_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
	    for(unsigned i = 0; i < imuData.size(); i++){
        req->value.int32Values[i] = imuData[i];
        // LOG(INFO) << "VehicleHalImpl:: IMUData: " << IMU_Fields[i]<< ":" << imuData[i];
		}
		imuData.clear();
        return req;
    }
	
	VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDIDRIDDataReq(std::vector<uint16_t> m_dID_message_type){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, m_dID_message_type.size());
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::DID_RID_DATA_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
	    for(unsigned i = 0; i < m_dID_message_type.size(); i++){
        req->value.int32Values[i] = m_dID_message_type[i];
        ALOGD("VehicleHalImpl:: createDIDRIDDataReq : %d  %d :" ,i,m_dID_message_type[i]);
		}
        return req;
    }
	
		VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDEM_DTCStatusDataReq(std::vector<uint16_t> m_DEM_DTC_message_type){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, m_DEM_DTC_message_type.size());
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::DIAG_DTC_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
	    for(unsigned i = 0; i < m_DEM_DTC_message_type.size(); i++){
        req->value.int32Values[i] = m_DEM_DTC_message_type[i];
        ALOGD("VehicleHalImpl:: createDEM_DTCStatusDataReq : %d  %d :" ,i,m_DEM_DTC_message_type[i]);
		}
        return req;
    }

   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFotahmiCheckforResultsDataReq(const std::string&  m_FOTAHMI_CheckforResults_DATA){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FOTAHMI_CHECK_FOR_UPDATES_RESULT );
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_FOTAHMI_CheckforResults_DATA;
        ALOGD("VehicleHalImpl:: createFotahmiCheckforResultsDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFotaHmiUpdateAvailableDataReq(const std::string&  m_FOTAHMI_UpdateAvailable_DATA){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FOTAHMI_UPDATE_AVAILABLE );
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_FOTAHMI_UpdateAvailable_DATA;
        ALOGD("VehicleHalImpl:: createFotaHmiUpdateAvailableDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFotaHmiConditionsnotmetDataReq(const std::string&  m_FOTAHMI_ConditionsnotMet_DATA){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FOTAHMI_CONDITIONS_NOT_MET );
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_FOTAHMI_ConditionsnotMet_DATA;
        ALOGD("VehicleHalImpl:: createFotaHmiConditionsnotmetDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFotaHmiInstallationStatusDatareq(const std::string&  m_FOTAHMI_InstallationStatus_DATA){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FOTAHMI_INSTALLATION_STATUS );
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_FOTAHMI_InstallationStatus_DATA;
        ALOGD("VehicleHalImpl:: createFotaHmiInstallationStatusDatareq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFotaHmiWhatNewDetailsDataReq(const std::string&  m_FOTAWhatsNew_Details_Response){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FOTAHMI_WHATSNEWDETAILS_RESPONSE);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_FOTAWhatsNew_Details_Response;
        ALOGD("VehicleHalImpl:: createFotaHmiWhatNewDetailsDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFotaHmiUpdateHistoryResponseDataReq(const std::string&  m_FOTAUpdate_History_Response){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FOTAHMI_UPDATEHISTORY_RESPONSE);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_FOTAUpdate_History_Response;
        ALOGD("VehicleHalImpl:: createFotaHmiUpdateHistoryResponseDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFotaHmiPendingUpdateHistoryDataReq(const std::string&  m_FOTAPending_Updates_Response){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FOTAHMI_PENDINGUPDATES_RESPONSE);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_FOTAPending_Updates_Response;
        ALOGD("VehicleHalImpl:: createFotaHmiPendingUpdateHistoryDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFotaHmiUpdateFinishedDataReq(const std::string&  m_FOTA_Update_Finished){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FOTAHMI_UPDATE_FINISHED);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_FOTA_Update_Finished;
        ALOGD("VehicleHalImpl:: createFotaHmiUpdateFinishedDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFotaHmiUpdateScheduleDataReq(const std::string&  m_FOTASchedule_Update_Response){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FOTAHMI_SHEDULE_UPDATE_RESPONSE);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_FOTASchedule_Update_Response;
        ALOGD("VehicleHalImpl:: createFotaHmiUpdateScheduleDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createUIN_TCUToHMIDataReq(const std::string&  UIN_TCU_ToHMI_Data){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom:: UIN_TCU_VENDOR );
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = UIN_TCU_ToHMI_Data;
        ALOGD("VehicleHalImpl:: createUIN_TCUToHMIDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createTCUVersiontoHMIDataReq(const std::string&  TCU_version_to_HMI){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom:: TCUVERSION_HMISHOW_VENDOR ); 
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = TCU_version_to_HMI;
        ALOGD("VehicleHalImpl:: createTCUVersiontoHMIDataReq : %s :" , req->value.stringValue.c_str());
	    return req;
   }

    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createAOSPRBUATriggerDataReq(const std::string& m_AOSPRBUATrigger){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::AOSP_RBUA_TRIGGER_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = m_AOSPRBUATrigger;
        ALOGD("VehicleHalImpl:: createAOSPRBUATriggerDataReq : %s :" , req->value.stringValue.c_str());
        return req;
    }   
		VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDriverdoorStatusDataReq(uint8_t driverdoorStatus){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::DRIVERDOORSTATUS_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
	    for(unsigned i = 0; i < 1; i++){
        req->value.int32Values[i] = driverdoorStatus;
       	DLT_LOG(STLA, DLT_LOG_DEBUG, DLT_STRING("Driver Door Status : "), DLT_UINT8(driverdoorStatus));
		    LOG(INFO) << "STLA_LOGS :: Driver Dooor Status: "<< driverdoorStatus;
        ALOGD("VehicleHalImpl:: createDriverdoorStatusDataReq : %d:" ,driverdoorStatus);
		}
        return req;
    }
		VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createPassengerDoorStatusDataReq(uint8_t passengerDoorStatus){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::PASSENGERDOORSTATUS_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
	    for(unsigned i = 0; i < 1; i++){
        req->value.int32Values[i] = passengerDoorStatus;
        ALOGD("VehicleHalImpl:: createPassengerDoorStatusDataReq : %d :" ,passengerDoorStatus);
		}
        return req;
    }
	
	VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalDataReqFrom_TCU(uint32_t TCU_Signal_data[]){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, TCU_InternalSignals_Index);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::TCU_SIGNALS_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        for(unsigned i = 0; i < TCU_InternalSignals_Index; i++){
        req->value.int32Values[i] = TCU_Signal_data[i];
        ALOGD("VehicleHalImpl:: createInternalDataReqFrom_TCU : %d  %d :" ,i,req->value.int32Values[i]);
		}
        IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_XCALL_STATUS] = TCU_Signal_data[6];
		IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MUTE_SUAL] = TCU_Signal_data[8];
		
		m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_XCALL_STATUS],iviCanDatasend::NTW_TX_SIGNAL_XCALL_STATUS);
        m_gateway.setIVI_TX_Signals(IVI_TX_signals[iviCanDatasend::NTW_TX_SIGNAL_DMD_MUTE_SUAL],iviCanDatasend::NTW_TX_SIGNAL_DMD_MUTE_SUAL);
        return req;
    }

    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createSLIDataReqFrom_TCU(uint32_t SLI_Data[]){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, SLI_data_length);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::SLI_DATA_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        for(unsigned i = 0; i < SLI_data_length; i++){
        req->value.int32Values[i] = SLI_Data[i];
        ALOGD("VehicleHalImpl:: createSLIDataReqFrom_TCU : %d  %d :" ,i,req->value.int32Values[i]);
		}

        return req;
    }

    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createInternalSystemtimeDataReq(uint16_t InternalSystemTime[]){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, Internal_SystemTime_Index);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::INTERNAL_SYSTEM_TIME_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        for(unsigned i = 0; i < Internal_SystemTime_Index; i++){
        req->value.int32Values[i] = InternalSystemTime[i];
        ALOGD("VehicleHalImpl:: createInternalSystemtimeDataReq : %d  %d :" ,i,req->value.int32Values[i]);
		}
        return req;
    }
	VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createsoftkeyDataReq(std::vector<uint16_t> softkeystatus){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, Softkeystatus_index);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::HMI_SOFTKEY_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        for(unsigned i = 0; i < Softkeystatus_index; i++){
        req->value.int32Values[i] = softkeystatus[i];
        ALOGD("VehicleHalImpl:: createsoftkeyDataReq : %d  %d :" ,i,req->value.int32Values[i]);
		}
        return req;
    }
	VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createHMITouchDataReq(std::vector<uint16_t> HMITouchData){
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, HMITouchData_index);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVI_HMI_DATA_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        for(unsigned i = 0; i < HMITouchData_index; i++){
        req->value.int32Values[i] = HMITouchData[i];
        ALOGD("VehicleHalImpl:: createHMITouchDataReq : %d  %d :" ,i,req->value.int32Values[i]);
        }
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createProjectionActiveStatusDataReq(uint32_t param) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::INTERNAL_ACTIVE_PROJECTION_VENDOR );
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
        req->value.int32Values[0] = param;
        
        ALOGD("VehicleHalImpl:: createProjectionActiveStatusDataReq : %d:" , req->value.int32Values[0]);	
        return req;
    }
	VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCAN_RX_DataReq(uint32_t CAN_RX_Signals[]) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, CAN_RX_Signal_Index);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CAN_RX_SIGNALS_VENDOR);
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
        for(unsigned i = 0; i < CAN_RX_Signal_Index; i++)
        {
        req->value.int32Values[i] = CAN_RX_Signals[i];
        ALOGD("VehicleHalImpl:: createCAN_RX_DataReq : %d %d :" , i,req->value.int32Values[i]);
        }	
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createSigned_CAN_RX_DataReq(int32_t Signed_CAN_RX_Signals[]) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, Signed_CAN_RX_signals_Index);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::SIGNED_CAN_RX_SIGNALS_VENDOR);
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
        for(unsigned i = 0; i < Signed_CAN_RX_signals_Index; i++)
        {
        req->value.int32Values[i] = Signed_CAN_RX_Signals[i];
        ALOGD("VehicleHalImpl:: createSigned_CAN_RX_DataReq : %d %d :" , i,req->value.int32Values[i]);
        }	
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createUserSettingsDataReq(std::vector<uint32_t>& User_Settings) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, User_Settings.size());
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVI_USER_SETTING_DATA_VENDOR);
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
        for(unsigned i = 0; i < User_Settings.size(); i++)
        {
            req->value.int32Values[i] = User_Settings[i];
           ALOGD("VehicleHalImpl:: createUserSettingsDataReq : %d %d :" , i,req->value.int32Values[i]);	 
        }
        return req;
    }
	    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createIVI_TX_SIGNALDataReq(std::vector<uint16_t>& ivi_TX_signal) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, IVI_TX_Signals_size);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CAN_IVI_TX_SIGNAL_VENDOR);
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
        for(unsigned i = 0; i < IVI_TX_Signals_size; i++)
        {
            req->value.int32Values[i] = ivi_TX_signal[i];
           ALOGD("VehicleHalImpl:: createIVI_TX_SIGNALDataReq : %d %d :" , i,req->value.int32Values[i]);	 
        }
        return req;
    }
	VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createAntitheftStatus_DataReq(uint8_t antitheft_status) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::ANTITHEFT_VENDOR );
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
    req->value.int32Values[0] = antitheft_status;
	
	ALOGD("VehicleHalImpl:: Antitheft_status : %d:" , req->value.int32Values[0]);	
    return req;
   }
   VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createLog_retriver_DataReq(uint32_t Log_Retriver_Ack) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::LOG_RETRIVER_ACK_VENDOR );
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
        req->value.int32Values[0] = Log_Retriver_Ack;
        
        ALOGD("VehicleHalImpl:: createLog_retriver_DataReq : %d:" , req->value.int32Values[0]);	
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createLog_retriver_statusDataReq(uint32_t Log_Retriver_status) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::LOGRETRIEVER_STATUS_VENDOR );
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
        req->value.int32Values[0] = Log_Retriver_status;
        
        ALOGD("VehicleHalImpl:: createLog_retriver_statusDataReq : %d:" , req->value.int32Values[0]);	
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createWakeUpReasonDataReq(uint32_t WakeUpReason) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::SOC_WAKEUP_REASON_VENDOR );
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
        req->value.int32Values[0] = WakeUpReason;
        
        ALOGD("VehicleHalImpl:: createWakeUpReasonDataReq : %d:" , req->value.int32Values[0]);	
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createLoggerfilepathDataReq(const std::string& param) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::LOGFILEPATH_TCU_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = param;
    
        ALOGD("VehicleHalImpl:: createLoggerfilepathDataReq : %s:", req->value.stringValue.c_str());
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDriverlockoutDataReq(uint32_t Driverlockout) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::IVI_DRIVER_LOCKOUTSTS_VENDOR );
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
        req->value.int32Values[0] = Driverlockout;
        
        ALOGD("VehicleHalImpl:: createDriverlockoutDataReq : %d:" , req->value.int32Values[0]);	
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createCameraISPDataReq(std::vector<uint32_t>& CameraISP) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, CameraISP.size());
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::ISP_CAMERA_VENDOR);
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
        for(unsigned i = 0; i < CameraISP.size(); i++)
        {
            req->value.int32Values[i] = CameraISP[i];
           ALOGD("VehicleHalImpl:: createCameraISPDataReq : %d %d :" , i,req->value.int32Values[i]);	 
        }
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createSystemTimeZoneDataReq(int32_t SystemTimeZone) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::INTERNAL_SYSTEM_TIME_ZONE);
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
        req->value.int32Values[0] = SystemTimeZone;
        
        ALOGD("VehicleHalImpl:: createSystemTimeZoneDataReq : %d:" , req->value.int32Values[0]);	
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createRXSignals_FloatDataReq(float RXSignals_Float[]) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::FLOAT_VEC, RXSignals_Float_Index);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::CAN_RX_FLOAT_VENDOR);
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
        for(unsigned i = 0; i < RXSignals_Float_Index; i++)
        {
            req->value.floatValues[i] = RXSignals_Float[i]; 
            ALOGD("VehicleHalImpl:: createRXSignals_FloatDataReq : %d %f :", i, req->value.floatValues[i]);	 
        }
        return req;
    }

    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createMarelliPNDataReq(std::vector<uint8_t>& MarelliPN) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, MarelliPN.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::EOL_MARELLI_PN_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < MarelliPN.size(); i++)
	{
		req->value.int32Values[i] = MarelliPN[i];
        ALOGD("VehicleHalImpl:: createMarelliPNDataReq : %d %d :" , i,req->value.int32Values[i]);	
	}
    return req;
}

    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createEOLHWIDDataReq(std::vector<uint8_t>& EOLHWID) {
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, EOLHWID.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::EOL_VARIANT_ID_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < EOLHWID.size(); i++)
	{
		req->value.int32Values[i] = EOLHWID[i];
        ALOGD("VehicleHalImpl:: createEOLHWIDDataReq : %d %d :" , i,req->value.int32Values[i]);	
	}
    return req;
}
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createAuthenticationZoneDataReq(std::vector<uint8_t>& AuthenticationZone) {
	//ALOGD("VehicleHalImpl : createAuthenticationZoneDataReq()");			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, AuthenticationZone.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::AUTHENTICATION_ZONE_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < AuthenticationZone.size(); i++)
	{
		req->value.int32Values[i] = AuthenticationZone[i];
        ALOGD("VehicleHalImpl:: createAuthenticationZoneDataReq : %d %d :" , i,req->value.int32Values[i]);	
	}
    return req;
}
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createApplicationSoftwareIdentificationDataReq(std::vector<uint8_t>& ApplicationSoftwareIdentification) {
	//ALOGD("VehicleHalImpl : createApplicationSoftwareIdentificationDataReq()");			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, ApplicationSoftwareIdentification.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::APPLICATION_SW_INDENTIFICATION_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < ApplicationSoftwareIdentification.size(); i++)
	{
		req->value.int32Values[i] = ApplicationSoftwareIdentification[i];
        ALOGD("VehicleHalImpl:: createApplicationSoftwareIdentificationDataReq : %d %d :" , i,req->value.int32Values[i]);	
	}
    return req;
}

    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createApplicationDataIdentificationDataReq(std::vector<uint8_t>& ApplicationDataIdentification) {
	//ALOGD("VehicleHalImpl : createApplicationDataIdentificationDataReq()");			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, ApplicationDataIdentification.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::APPLICATION_DATA_INDENTIFICATION_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < ApplicationDataIdentification.size(); i++)
	{
		req->value.int32Values[i] = ApplicationDataIdentification[i];
        ALOGD("VehicleHalImpl:: createApplicationDataIdentificationDataReq : %d %d :" , i,req->value.int32Values[i]);	
	}
    return req;
}
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createDataLibraryidentifierDataReq(std::vector<uint8_t>& DataLibraryidentifier) {
	//ALOGD("VehicleHalImpl : createDataLibraryidentifierDataReq()");			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, DataLibraryidentifier.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::DATA_LIBRARY_IDENTIFIER_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < DataLibraryidentifier.size(); i++)
	{
		req->value.int32Values[i] = DataLibraryidentifier[i];
        ALOGD("VehicleHalImpl:: createDataLibraryidentifierDataReq : %d %d :" , i,req->value.int32Values[i]);	
	}
    return req;
}
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createVehicleAppsIdentifierDataReq(std::vector<uint8_t>& VehicleAppsIdentifier) {
	//ALOGD("VehicleHalImpl : createVehicleAppsIdentifierDataReq()");			
    auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32_VEC, VehicleAppsIdentifier.size());
    req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::VEHICLE_APPS_IDENTIFIER_VENDOR);
    req->areaId = 0;
    req->timestamp =  android::elapsedRealtimeNano();
    req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE; 
	for(unsigned i = 0; i < VehicleAppsIdentifier.size(); i++)
	{
		req->value.int32Values[i] = VehicleAppsIdentifier[i];
        ALOGD("VehicleHalImpl:: createVehicleAppsIdentifierDataReq : %d %d :" , i,req->value.int32Values[i]);	
	}
    return req;
}
	 VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createFlashUnitIdDataReq(uint32_t FlashUnitId) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::INT32, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::FLASH_UNITID_VENDOR );
        req->areaId = 0;
        req->timestamp =  android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;    
        req->value.int32Values[0] = FlashUnitId;
        
        ALOGD("VehicleHalImpl:: createFlashUnitIdDataReq : %d:" , req->value.int32Values[0]);	
        return req;
    }
    VehicleHalImpl::VehiclePropValuePtr VehicleHalImpl::createPackagePathDataReq(const std::string& param) {
        auto req = getValuePool()->obtain(vhal_v2_0::VehiclePropertyType::STRING, 1);
        req->prop = (int)(vhalCustom_v1_0::VehiclePropertyCustom::PACKAGE_PATH_VENDOR);
        req->areaId = 0;
        req->timestamp = android::elapsedRealtimeNano();
        req->status = vhal_v2_0::VehiclePropertyStatus::AVAILABLE;
        req->value.stringValue = param;
    
        ALOGD("VehicleHalImpl:: createPackagePathDataReq : %s:", req->value.stringValue.c_str());
        return req;
    }
    void VehicleHalImpl::onDCSD_DIAG_Reqdata(std::vector<uint8_t>& DCSD_DIAG_Req) {
    ALOGD("VehicleHalImpl : onDCSD_DIAG_Reqdata()");
    
    for (unsigned i = 0; i < DCSD_DIAG_Req.size(); i++) {
        ALOGD("DCSD_DIAG_Req: %d", DCSD_DIAG_Req[i]);
    }

    uint8_t* data = DCSD_DIAG_Req.data();
    uint32_t len = DCSD_DIAG_Req.size();

    /** Send the diagnostic request **/
    int result = LVDSMgr_SendDiagReq(data, len);
    sleep(2);

    if (result < 0) {
        ALOGE("VehicleHalImpl : Failed to send diagnostic request.");
        return;
    }

    /** Buffer to receive the diagnostic response **/
    uint8_t responseBuffer[100] = {0};  
    uint32_t responseLen = 100;

    /** Get the diagnostic response **/
    int32_t ret = LVDSMgr_GetDiagResponse(responseBuffer, responseLen);
    LOG(INFO) << "LVDSMgr_GetDiagResponse::return" << ret;
    if (ret < 0) {
        ALOGE("VehicleHalImpl : Failed to get diagnostic response.");
        return;
    }

    for (int i = 0; i < ret; ++i) {
        ALOGI("DiagResponse: 0x%02x", responseBuffer[i]);
    }

    std::vector<uint8_t> DCSD_DIAG_Res(responseBuffer, responseBuffer + ret);

    m_gateway.setDCSD_DIAG_Res(DCSD_DIAG_Res);
}


bool VehicleHalImpl::isContinuousProperty(int32_t propId) const
{
    // ALOGD("VehicleHalImpl : isContinuousProperty()");
    const vhal_v2_0::VehiclePropConfig* config = mPropStore->getConfigOrNull(propId);
    if (config == nullptr)
    {
        ALOGW("%s(): Config not found for property: %d", __func__, propId);
        return false;
    }
    return config->changeMode == vhal_v2_0::VehiclePropertyChangeMode::CONTINUOUS;
}

bool VehicleHalImpl::isProxyAvailable()
{
return m_gateway.isAvailable();
}

std::string generateChecksum(const std::string& sentence) {
    int checksum = 0;
    for (char c : sentence) {
        if(c!='$' && c!='*')
        {
        checksum ^= c;
        }
    }
    std::stringstream ss;
    ss << std::hex << std::uppercase << checksum;
    return ss.str();
}

std::string generatePASCDSentence(float timestamp, char sensorType,char transmissionState, int slipDetect, const std::vector<VehicleHalImpl::SensorData>& sensorData) {
    std::ostringstream oss;
     oss << std::fixed << std::setprecision(3);
    
    oss << "$PASCD," << timestamp << ',' << sensorType << ',' << transmissionState << ',' << slipDetect << ',' << sensorData.size();
    for (const auto& data : sensorData) {
        oss << ',' << std::fixed << std::setprecision(2) << data.timeOffset << ',';
        oss << std::fixed << std::setprecision(3) << data.speed;
    }
    std::string sentence = oss.str();
    std::string checksum = generateChecksum(sentence);
    return sentence + '*' + checksum;
}

}

}
}
}
}
