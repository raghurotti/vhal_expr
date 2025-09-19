#pragma once

#include <vhal_v2_0/RecurrentTimer.h>
#include <vhal_v2_0/VehicleHal.h>
#include <vhal_v2_0/VehiclePropertyStore.h>
#include "../../../marelli_lvds/include/MarelliLVDSImpl.h"
#include "VehicleHalClient.h"
#include "CAPIVehicleClientGateway.h"
#include <iomanip>
#include <sstream>
#include <vector>
#include <list>
#include <string>
#define PrivacyMode 15


namespace vhal_v2_0 = android::hardware::automotive::vehicle::V2_0;

namespace vendor {
namespace marelli {
namespace vehicle {
namespace V1_0 {

namespace impl {
#define vinNumber_size 21
class VehicleHalImpl : public vhal_v2_0::VehicleHal
{
public:
    VehicleHalImpl(vhal_v2_0::VehiclePropertyStore* propStore, VehicleHalClient* client);
    ~VehicleHalImpl() = default;

    void onCreate() override;
    std::vector<vhal_v2_0::VehiclePropConfig> listProperties() override;
    VehiclePropValuePtr get(
        const vhal_v2_0::VehiclePropValue& requestedPropValue,
        vhal_v2_0::StatusCode* outStatus) override;
    vhal_v2_0::StatusCode set(const vhal_v2_0::VehiclePropValue& propValue) override;
    vhal_v2_0::StatusCode subscribe(int32_t property, float sampleRate) override;
    vhal_v2_0::StatusCode unsubscribe(int32_t property) override;
    bool isProxyAvailable();
	void pollForCANData();
	void pollForIMUsubscribe();
	void pollForDIDsubscribe();
  	void pollForDotteDatasubscribe();
    void pollForDTCsubscribe();
    void pollForDLTsubscribe();
  	void pollForInternalSignalsubscribe();
	void pollforVinNumbersubscribe();
    void pollForDLT_DTC_LOG();
  	void pollForPowerModeData();
	void pollforCALDATAsubscribe();
    void pollForTemperature();
    void pollforsupplyvoltage();
    void pollForUSB_update();
    void pollForUSB_update_details();
    int convertToTimestamp(const std::string& dateStr);
	void pollForTCUData();
	void pollForDEM_DTCStatussubscribe();
    bool areArraysEqual(const uint32_t* arr1, const uint32_t* arr2, size_t size);
    void pollForCallActiveSTRDisplayPopupStatus();
    void pollForSTRCancelShutdown();
    void pollForLoggerdata();
    void pollForSLI_Data();
    void pollForSoCVersionData();
    void pollForhw_variant_id();
    void pollForSOC_Wakeup_reason();
    void pollForMarelliPartNumber();
    void pollForEOL_hw_variant_id();
    void pollForPackagePath();
    void pollforCheckforUpdateResults();
    void pollforUpdateAvailable();
    void pollforConditionsNotMet();
    void pollforInstallationStatus();
    void pollforUpdateFinished();
    void pollforWhatsNewDetails();
    void pollforUpdateHistory();
    void pollforPendingUpdates();
    void pollforScheduleUpdate();
    void pollforAOSPRBUATrigger();
    void pollforFotaHmiInfoResult();
    void pollforUSBUpdateDetails();
	
    void pollForInternalSignalInitialize();
   
 struct SensorData {
    float timeOffset;
    float speed;
	uint32_t parklamp;
};
enum phonecallstatus{
  PHONE_STATE_NEW = 00,
  PHONE_STATE_CONNECTING = 01,
  PHONE_STATE_SELECT_PHONE_ACCOUNT = 02,
  PHONE_STATE_DIALING = 03,
  PHONE_STATE_RINGING = 04,
  PHONE_STATE_ACTIVE = 05,
  PHONE_STATE_ON_HOLD = 06,
  PHONE_STATE_DISCONNECTED = 07,
  PHONE_STATE_ABORTED = 8,
  PHONE_STATE_DISCONNECTING = 9,
  PHONE_STATE_PULLING = 10,
  PHONE_STATE_ANSWERED = 11,
  PHONE_STATE_AUDIO_PROCESSING = 12,
  PHONE_STATE_SIMULATED_RINGING = 13,
  
};
enum internalsystemTime{
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
SystemUserTimeYr,
};
enum audioAdasVendorDatasend{
    BSISFrontChimeType = 0,
    BSISFrontReq =1,
    BSISRearChimeType =2,
    BSISRearReq =3,
    BSSChimeRqLF =4,
    BSSChimeRqLR =5,
    BSSChimeRqRF =6,
    BSSChimeRqRR =7,
    BSSChimeType =8,
    CADMChimeLF =9,
    CADMChimeLR =10,
    CADMChimePriority =11,
    CADMChimeRate =12,
    CADMChimeRF = 13,
    CADMChimeRR =14,
    CADMChimeType =15,
    DASMChimeRate =16,
    DASMChimeRqLF =17,
    DASMChimeRqRF =18,
    DASMChimeType =19,
    HALFChimeRqLF =20,
    HALFChimeRqRF =21,
    HALFChimeType =22,
    HALFChimeRate =23,
    SDWChimeRepRate =24,
    SDWChimeRqFL =25,
    SDWChimeRqRL =26,
    SDWChimeRqFR =27,
    SDWChimeRqRR =28,
    SDWChimeType =29,
    SDWChimeVolume =30,
    SDWChimeMode =31,
    Audio_adasSignal= 32
 
};
enum iviCanDatasend{
	NTW_TX_SIGNAL_ACCUEIL_COND = 0,
    NTW_TX_SIGNAL_EXTGPSCLKYR = 1,
    NTW_TX_SIGNAL_EXTGPSCLKSEC1 = 2,
    NTW_TX_SIGNAL_EXTGPSCLKMINS = 3,
    NTW_TX_SIGNAL_EXTGPSCLKMTH = 4,
    NTW_TX_SIGNAL_EXTGPSCLKHR = 5,
    NTW_TX_SIGNAL_EXTGPSCLKDAY = 6,
    NTW_TX_SIGNAL_UPD_SEC_CLK_DAY_REQ = 7,
    NTW_TX_SIGNAL_UPD_SEC_CLK_HR_REQ = 8,
    NTW_TX_SIGNAL_UPD_SEC_CLK_MIN_REQ = 9,
    NTW_TX_SIGNAL_UPD_SEC_CLK_MTH_REQ = 10,
    NTW_TX_SIGNAL_UPD_SEC_CLK_SEC_REQ = 11,
    NTW_TX_SIGNAL_UPD_SEC_CLK_YR_REQ = 12,
    NTW_TX_SIGNAL_SLI_CS_CONFIDENCE_LEVEL = 13,
    NTW_TX_SIGNAL_SLI_CS_COUNTRY_CODE = 14,
    NTW_TX_SIGNAL_SLI_CS_ROAD_TYPE = 15,
    NTW_TX_SIGNAL_SLI_CS_SPEED_LIMIT = 16,
    NTW_TX_SIGNAL_CS_PARAM_VHL_MOBILITY = 17,
    NTW_TX_SIGNAL_DMD_MENU_PRIVACY_MODE = 18,
    NTW_TX_SIGNAL_MODE_HEURE_CLIENT_HS7 = 19,
    NTW_TX_SIGNAL_HEURE_HORLOGE_HS7 = 20,
    NTW_TX_SIGNAL_MINUTE_HORLOGE_HS7 = 21,
    NTW_TX_SIGNAL_JOUR_HORLOGE_HS7 = 22,
    NTW_TX_SIGNAL_MOIS_HORLOGE_HS7 = 23,
    NTW_TX_SIGNAL_ANNEE_HORLOGE_HS7 = 24,
    // NTW_TX_SIGNAL_AFFICHAGE_HORLOGE_HS7,
    NTW_TX_SIGNAL_REV = 25,
    NTW_TX_SIGNAL_RQ_LOCK_UNLOCK_REM_GEN2 = 26,
    NTW_TX_SIGNAL_REMOTE_HORN_REQUEST = 27,
    NTW_TX_REMOTE_TURN_LIGHTS_REQ = 28,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_1 = 29,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_2 = 30,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_3 = 31,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_REM_4 = 32,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_1 = 33,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_1 = 34,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_1 = 35,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_1 = 36,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_1 = 37,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_1 = 38,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_1 = 39,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_2 = 40,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_2 = 41,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_2 = 42,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_2 = 43,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_2 = 44,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_2 = 45,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_2 = 46,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_3 = 47,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_3 = 48,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_3 = 49,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_3 = 50,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_3 = 51,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_3 = 52,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_3 = 53,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_REM_4 = 54,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_REM_4 = 55,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_REM_4 = 56,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_REM_4 = 57,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_REM_4 = 58,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_REM_4 = 59,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_REM_4 = 60,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_1 = 61,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_2 = 62,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_3 = 63,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_REM_4 = 64,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ASAP = 65,
    NTW_TX_SIGNAL_REMOTE_IMMO_REQ = 66,
    NTW_TX_SIGNAL_REMOTE_IMMO_PWD = 67,
    NTW_TX_SIGNAL_DMD_MENU_RTAB_TYPE_RECH_REM = 68,
    NTW_TX_SIGNAL_DMD_MENU_RTAB_REC_H_DEB_REM = 69,
    NTW_TX_SIGNAL_DISPO_DMD_RECHARGE_REM = 70,
    NTW_TX_SIGNAL_NEXT_CHARG_STATION_ARRIVALSOC = 71,
    NTW_TX_SIGNAL_NEXT_CHARG_STATION_DISTANCE = 72,
    NTW_TX_SIGNAL_NEXT_CHARG_STATION_MAXPOWER = 73,
    NTW_TX_SIGNAL_NEXT_CHARG_STATION_TIMETOTRAVEL = 74,
    NTW_TX_SIGNAL_CHIME_MUTE_REQ = 75,
    NTW_TX_SIGNAL_IVI_Language_SELECTION = 76,
    NTW_TX_SIGNAL_Update_Req = 77,
    NTW_TX_SIGNAL_REMOTE_ACTION_REQ_254__REMOTE_LOCK_UNLOCK_REQ = 78,
    
    NTW_TX_SIGNAL_FUSEAU_HORAIRE = 79,
    NTW_TX_SIGNAL_DISPO_DMD_PRECOND = 80,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_1 = 81,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_2 = 82,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_1 = 83,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_2 = 84,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_1 = 85,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_2 = 86,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_1 = 87,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_2 = 88,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_1 = 89,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_2 = 90,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_1 = 91,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_2 = 92,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_1 = 93,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_2 = 94,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_1 = 95,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_2 = 96,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_1 = 97,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_2 = 98,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_3 = 99,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_DI_4 = 100,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_3 = 101,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_H_4 = 102,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_3 = 103,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_JE_4 = 104,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_3 = 105,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_LU_4 = 106,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_3 = 107,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_MA_4 = 108,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_3 = 109,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ME_4 = 110,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_3 = 111,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_ON_4 = 112,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_3 = 113,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_SA_4 = 114,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_3 = 115,
    NTW_TX_SIGNAL_DMD_MENU_CLIM_PREC_VE_4 = 116,
    NTW_TX_SIGNAL_GSMEXTCLKDAY = 117,
    NTW_TX_SIGNAL_GSMEXTCLKHR = 118,
    NTW_TX_SIGNAL_GSMEXTCLKMINS = 119,
    NTW_TX_SIGNAL_GSMEXTCLKMTH = 120,
    NTW_TX_SIGNAL_GSMEXTCLKSEC1 = 121,
    NTW_TX_SIGNAL_GSMEXTCLKYR = 122,
    NTW_TX_SIGNAL_DISPO_DMD_RECHARGE = 123,
    NTW_TX_SIGNAL_DMD_MENU_RTAB_REC_H_DEB = 124,
    NTW_TX_SIGNAL_DMD_MENU_RTAB_TYPE_RECH = 125,
    NTW_TX_SIGNAL_DISPO_DMD_PRECOND_REM = 126,
    NTW_TX_SIGNAL_C_ABSENT = 127,
    NTW_TX_SIGNAL_C_BUSOFF = 128,
    NTW_TX_SIGNAL_C_DEFAUTPERTECOM = 129,
    NTW_TX_SIGNAL_C_MUTE = 130,
    NTW_TX_SIGNAL_DIAG_MUX_ON_IVI = 131,
    NTW_TX_SIGNAL_EFF_DEF_IVI_HAB = 132,
    NTW_TX_SIGNAL_DMD_MENU_PRIVACY_MODE_REM = 133,
    NTW_TX_SIGNAL_DMD_MUTE_SUAL = 134,
    NTW_TX_SIGNAL_XCALL_STATUS = 135,
    NTW_TX_SIGNAL_EDITION_CALIB = 136,
    NTW_TX_SIGNAL_EDITION_SOFT = 137,
    NTW_TX_SIGNAL_SUPPLIER_CODE_ZA = 138,
    NTW_TX_SIGNAL_VERSION_APPLI = 139,
    NTW_TX_SIGNAL_VERSION_SOFT = 140,
    NTW_TX_SIGNAL_VERSION_SYSTEME = 141,

    /* Upcoming signals should be to added above */
    NTW_TX_SIGNAL_NUMBER = 142 
};

  std::string generateChecksum(const std::string& sentence);
  std::string generatePASCDSentence(float timestamp, char sensorType, char transmissionState, int slipDetect, const std::vector<VehicleHalImpl::SensorData>& sensorData);
  VehiclePropValuePtr createSpeedDataReq(float param);
  VehiclePropValuePtr createParkLampDataReq(uint32_t param);
  VehiclePropValuePtr createDayNightModeDataReq(uint32_t param);
  VehiclePropValuePtr createFuelTypeDataReq(uint32_t param);
  VehiclePropValuePtr createFuelLevalDataReq(uint32_t param);
  VehiclePropValuePtr createParkBreakStatusDataReq(uint32_t param);
  VehiclePropValuePtr createoutTempDataReq(uint32_t param);
  VehiclePropValuePtr createFactoryResetCountDataReq(uint32_t factoryresetcount);
  VehiclePropValuePtr createBCMsecureTimeDataReq(uint32_t param[]);
  VehiclePropValuePtr createBCMHMIDataReq(uint32_t param[]);
  VehiclePropValuePtr createPowermodeDataReq(uint32_t param);
  VehiclePropValuePtr createDotteParamDataReq(std::vector<uint32_t>&dotteParam);
  VehiclePropValuePtr createHVBatteryMaxRangeDataReq(uint32_t param);
  VehiclePropValuePtr createHVBatterySOHDataReq(uint32_t param);
  VehiclePropValuePtr createHVBatteryPercentageDataReq(uint32_t param);
  VehiclePropValuePtr createHVBatteryActiveConnectorDataReq(uint32_t param);
  VehiclePropValuePtr createHVBatteryMinChargeDataReq(uint32_t param);
  VehiclePropValuePtr createHVBatteryIsChargingDataReq(uint32_t param);
  VehiclePropValuePtr createHVBatteryChargeLevelDataReq(uint32_t param);
  VehiclePropValuePtr createVinNumberDataReq(std::vector<uint32_t>& vinNumber);
  VehiclePropValuePtr createinternalSignalDataReq(uint32_t internalSignal[]);
  VehiclePropValuePtr createAutonomyDataReq(uint32_t param);
  VehiclePropValuePtr createBrakePedalrStsDataReq(uint32_t param);
  VehiclePropValuePtr createCanDistanceUnitDataReq(uint32_t param);
  VehiclePropValuePtr createInstantFuelConsDataReq(uint32_t param);
  VehiclePropValuePtr createReverseGearEngagedDataReq(uint32_t param);
  VehiclePropValuePtr createRoadSlopeDataReq(uint32_t param);
  VehiclePropValuePtr createSteeringAngleReq(uint32_t param);
  VehiclePropValuePtr createTotalKMReq(uint32_t param);
  VehiclePropValuePtr createTransmGearDisplayReq(uint32_t param);
  VehiclePropValuePtr createTripAvgBattPowerConsReq(uint32_t param);
  VehiclePropValuePtr createYawSpeedDataReq(uint32_t param);
  VehiclePropValuePtr createGrossYawRateDataReq(uint32_t param);
  VehiclePropValuePtr createFLWheelTickCounterFaultDataReq(uint32_t param);
  VehiclePropValuePtr createFRWheelTickCounterFaultDataReq(uint32_t param);
  VehiclePropValuePtr createRLWheelTickCounterFaultDataReq(uint32_t param);
  VehiclePropValuePtr createRRFWheelTickCounterFaultDataReq(uint32_t param);
  VehiclePropValuePtr createPulseCountFLWheelDataReq(uint32_t param);
  VehiclePropValuePtr createPulseCountFRWheelDataReq(uint32_t param);
  VehiclePropValuePtr createPulseCountRLWheelDataReq(uint32_t param);
  VehiclePropValuePtr createPulseCountRRWheelDataReq(uint32_t param);
  VehiclePropValuePtr createFPASStsDataReq(uint32_t param);
  VehiclePropValuePtr createDrivingDirectionDataReq(uint32_t param);
  VehiclePropValuePtr createIgnitionStsDataReq(uint32_t param);
  VehiclePropValuePtr createEcallCrashDataReq(uint32_t param);
  VehiclePropValuePtr createDTCParamDataReq(std::vector<uint32_t>& dtcParam);

  VehiclePropValuePtr createFotahmiCheckforResultsDataReq(const std::string&  m_FOTAHMI_CheckforResults_DATA);
  VehiclePropValuePtr createFotaHmiUpdateAvailableDataReq(const std::string&  m_FOTAHMI_UpdateAvailable_DATA);
  VehiclePropValuePtr createFotaHmiConditionsnotmetDataReq(const std::string&  m_FOTAHMI_ConditionsnotMet_DATA);
  VehiclePropValuePtr createFotaHmiInstallationStatusDatareq(const std::string&  m_FOTAHMI_InstallationStatus_DATA);
  VehiclePropValuePtr createFotaHmiWhatNewDetailsDataReq(const std::string& m_FOTAWhatsNew_Details_Response);
  VehiclePropValuePtr createFotaHmiUpdateHistoryResponseDataReq(const std::string& m_FOTAUpdate_History_Response);
  VehiclePropValuePtr createFotaHmiPendingUpdateHistoryDataReq(const std::string& m_FOTAPending_Updates_Response);
  VehiclePropValuePtr createFotaHmiUpdateFinishedDataReq(const std::string& m_FOTA_Update_Finished);
  VehiclePropValuePtr createFotaHmiUpdateScheduleDataReq(const std::string& m_FOTASchedule_Update_Response);

  VehiclePropValuePtr createUIN_TCUToHMIDataReq(const std::string&  UIN_TCU_ToHMI_Data);
  VehiclePropValuePtr createTCUVersiontoHMIDataReq(const std::string&  TCU_version_to_HMI);
  VehiclePropValuePtr createDLTParamDataReq(std::vector<uint32_t>& dLTParam);
  VehiclePropValuePtr createDLT_VectorParamDataReq(std::vector<uint8_t> vec_arr);
  VehiclePropValuePtr createCapacityUnitDataReq(uint32_t param);
  VehiclePropValuePtr createCombustFuelLvDataReq(uint32_t param);
  VehiclePropValuePtr createCompressorStsDataReq(uint32_t param);
  VehiclePropValuePtr createElectricMotorPowerDataReq(int32_t param);
  VehiclePropValuePtr createEngineCoolantTempDataReq(uint32_t param);
  VehiclePropValuePtr createEngineRPMDataReq(uint32_t param);
  VehiclePropValuePtr createFuelLevelMinimumStsDataReq(uint32_t param);
  VehiclePropValuePtr createAbsSteeringWheelAngleAcuDataReq(uint32_t param);
  VehiclePropValuePtr createInitSteeringWheenAngleFlagDataReq(uint32_t param);
  VehiclePropValuePtr createNetworkMNGTDataReq(uint32_t param);
  VehiclePropValuePtr createFPASCenterLeftBarStsDataReq(uint32_t param);
  VehiclePropValuePtr createFPASCenterRightBarStsDataReq(uint32_t param);
  VehiclePropValuePtr createFPASLeftBarStsDataReq(uint32_t param);
  VehiclePropValuePtr createFPASRightBarStsDataReq(uint32_t param);
  VehiclePropValuePtr createRPASCenterLeftBarStsDataReq(uint32_t param);
  VehiclePropValuePtr createRPASCenterRightBarStsDataReq(uint32_t param);
  VehiclePropValuePtr createRPASLeftBarStsDataReq(uint32_t param);
  VehiclePropValuePtr createTrailerPresentDataReq(uint32_t param);
  VehiclePropValuePtr createTrunkDoorStsDataReq(uint32_t param);
  VehiclePropValuePtr createRPASStsDataReq(uint32_t param);
  VehiclePropValuePtr createEconShftRqDataReq(uint32_t param);
  VehiclePropValuePtr createEngineTorqueDataReq(uint32_t param);
  VehiclePropValuePtr createFuelConsUnitDataReq(uint32_t param);
  VehiclePropValuePtr createRegeneratedEnergyDataReq(uint32_t param);
  VehiclePropValuePtr createShiftLeverPositionDataReq(uint32_t param);
  VehiclePropValuePtr createTripAverageFuelConsumptionDataReq(uint32_t param);
  VehiclePropValuePtr createReqVolCtrlDataReq(uint32_t param);
  VehiclePropValuePtr createCALDataReq(uint32_t calData[]);
  VehiclePropValuePtr createEVSignalsDataReq(int32_t EVSignal[]);
  VehiclePropValuePtr createBCMSignalsDataReq(uint32_t BCMSignal[]);
  VehiclePropValuePtr createVehPrivacyModeDataReq(uint32_t param);
  VehiclePropValuePtr createCameraSignalsDataReq(uint32_t CameraSignal[]);
  VehiclePropValuePtr createTemperatureDataReq(uint32_t param);
  VehiclePropValuePtr createSupplyVoltageDataReq(uint32_t Voltage_Data);
  VehiclePropValuePtr createCallActiveSTRDisplayPopupStatusDataReq(uint32_t callActiveSTRDisplayPopupStatus);
  VehiclePropValuePtr createSTRCancelShutdownStatusDataReq(uint32_t STRCancelShutdownStatus);
  VehiclePropValuePtr createInternalCameraSignalsDataReq(std::vector<uint32_t>& camera_data);
  VehiclePropValuePtr createCanCommonDataReq(int32_t CanCommonSignal[]);
  VehiclePropValuePtr createInternalAudioSignalsDataReq(std::vector<uint32_t>& audio_data);
  VehiclePropValuePtr createInternalProjectionDataReq(std::vector<uint32_t>& internalCarplay);
  VehiclePropValuePtr createInternalConnectivityDataReq(std::vector<uint32_t>& internalConnectivity); 
  VehiclePropValuePtr createInternalHmiSignalsDataReq(std::vector<uint32_t>& hmi_data);
  VehiclePropValuePtr createInternalSecondPartyAppSignalsDataReq(std::vector<uint32_t>& Second_party_app);
  VehiclePropValuePtr createUSB_UpdateSignalsDataReq(std::vector<uint16_t>& USB_Update);
  VehiclePropValuePtr createIMC_MCUUSB_UpdateSignalsDataReq(std::vector<uint16_t>& USB_Update);
  VehiclePropValuePtr createUSB_Update_DetailsSignalsDataReq(const std::string& m_USBUpdate_Details);
  VehiclePropValuePtr createSystemTimeStampDataReq(uint32_t param);
  VehiclePropValuePtr createCALDataReq_Audio(uint32_t calData[]);
  VehiclePropValuePtr createCALDataReq_Camera(uint32_t calData[]);
  VehiclePropValuePtr createCALDataReq_Array(std::vector<uint8_t>& calData);
  VehiclePropValuePtr createInternalDataReqFrom_TCU(uint32_t TCU_Signal_data[]);
  VehiclePropValuePtr createSatelliteNumberDataReq(uint32_t param);
  VehiclePropValuePtr createlatitudeDataReq(float param);
  VehiclePropValuePtr createlongitudeDataReq(float param);
  VehiclePropValuePtr createcurrentelevationDataReq(float param);
  VehiclePropValuePtr createMapVersionDataReq(const std::string& param);
   VehiclePropValuePtr createIMUDataReq(std::vector<int32_t>& imuData);
   VehiclePropValuePtr createInternalSystemtimeDataReq(uint16_t InternalSystemTime[]);
   VehiclePropValuePtr createDIDRIDDataReq(std::vector<uint16_t> dID_message_type );
   VehiclePropValuePtr createPhonecallstatusDataReq(uint32_t param);
   VehiclePropValuePtr createsoftkeyDataReq(std::vector<uint16_t> softkeystatus);
   VehiclePropValuePtr createHMITouchDataReq(std::vector<uint16_t> HMITouchData);
   VehiclePropValuePtr createProjectionActiveStatusDataReq(uint32_t param);
   void startSuspendToRam(uint32_t powerModeValue);
    VehiclePropValuePtr createCAN_RX_DataReq(uint32_t CAN_RX_Signals[]);
   VehiclePropValuePtr createDEM_DTCStatusDataReq(std::vector<uint16_t> m_diag_wakeup_message_type);
   VehiclePropValuePtr createDriverdoorStatusDataReq(uint8_t driverdoorStatus);
   VehiclePropValuePtr createPassengerDoorStatusDataReq(uint8_t passengerDoorStatus);
   VehiclePropValuePtr createUserSettingsDataReq(std::vector<uint32_t>& User_Settings);
   VehiclePropValuePtr createSigned_CAN_RX_DataReq(int32_t Signed_CAN_RX_Signals[]);
   VehiclePropValuePtr createAntitheftStatus_DataReq(uint8_t antitheft_status);
   VehiclePropValuePtr createPassengerConfirmStsDataReq(uint8_t Passenger_confirmSts);
   VehiclePropValuePtr createLog_retriver_DataReq(uint32_t Log_Retriver_Ack);
   VehiclePropValuePtr createLoggerfilepathDataReq(const std::string& param);
   VehiclePropValuePtr createSLIDataReqFrom_TCU(uint32_t SLI_Data[]);
   VehiclePropValuePtr createLog_retriver_statusDataReq(uint32_t Log_Retriver_status);
   VehiclePropValuePtr createPOI_databaseversionDataReq(const std::string& param);
   VehiclePropValuePtr createlastUpdateTimeDataReq(const std::string& param);
   VehiclePropValuePtr createWakeUpReasonDataReq(uint32_t WakeupReason);
   VehiclePropValuePtr createDriverlockoutDataReq(uint32_t Driver_lockout);
   VehiclePropValuePtr createCameraISPDataReq(std::vector<uint32_t>& CameraISP);
   VehiclePropValuePtr createSystemTimeZoneDataReq(int32_t SystemTimeZone);
   VehiclePropValuePtr createAudioAdasDataReq(std::vector<uint8_t> audio_adas_signal);
   VehiclePropValuePtr createFlashUnitIdDataReq(uint32_t Flash_UnitId);
   VehiclePropValuePtr createPackagePathDataReq(const std::string& param);
   VehiclePropValuePtr createRXSignals_FloatDataReq(float RXSignals_Float[]);
   VehiclePropValuePtr createMarelliPNDataReq(std::vector<uint8_t>& MarelliPN);
   VehiclePropValuePtr createEOLHWIDDataReq(std::vector<uint8_t>& EOLhwid);
   VehiclePropValuePtr createAuthenticationZoneDataReq(std::vector<uint8_t>& AuthenticationZone);
   VehiclePropValuePtr createApplicationSoftwareIdentificationDataReq(std::vector<uint8_t>& ApplicationSoftwareIdentification);
   VehiclePropValuePtr createApplicationDataIdentificationDataReq(std::vector<uint8_t>& ApplicationDataIdentification);
   VehiclePropValuePtr createDataLibraryidentifierDataReq(std::vector<uint8_t>& DataLibraryidentifier);
   VehiclePropValuePtr createVehicleAppsIdentifierDataReq(std::vector<uint8_t>& VehicleAppsIdentifier);
   VehiclePropValuePtr createIVI_TX_SIGNALDataReq(std::vector<uint16_t>& ivi_TX_signal);
   VehiclePropValuePtr createSiviGnssGsmAvailableStatusDataReq(uint32_t sivi_gnssgsm_data);
   VehiclePropValuePtr createIVIThemeVendorDataReq(uint32_t IVI_Theme);
   //VehiclePropValuePtr createCALSrecDataReq(std::vector<uint32_t> calSrecDATA);
   VehiclePropValuePtr createAOSPRBUATriggerDataReq(const std::string& m_AOSPRBUATrigger);   
   VehiclePropValuePtr createGraphElecConsHistDataReq(uint32_t gps_histo_graph_dist_elec); 
   VehiclePropValuePtr createChargeTypeReqHMIDataReq(uint32_t cgt_rtab_type_recharge);
   VehiclePropValuePtr createStartTimeDelayedChargeHMIDataReq(uint32_t start_time_delayed_charge);  
   VehiclePropValuePtr createChargeMenuDelayStsDataReq(uint32_t charge_menu_delay_sts);
   VehiclePropValuePtr createVehGearConfigDataReq(uint32_t veh_gear_config);
   VehiclePropValuePtr createVehGearNetDataReq(uint32_t veh_gear_net);
   VehiclePropValuePtr createVehPosIntDataReq(uint32_t veh_pos_int);
   VehiclePropValuePtr createReverseGearStsDataReq(uint32_t reverse_gear_sts);
   VehiclePropValuePtr createManualParkBreakStsDataReq(uint32_t manual_park_break_sts);
   VehiclePropValuePtr createParkBreakStsEstDataReq(uint32_t park_break_sts_est);
   VehiclePropValuePtr createFotaHmiInfoResultDataReq(const std::string& m_FOTAHMI_Info_Result);
   VehiclePropValuePtr createGnssSignalStrengthDataReq(int32_t gnss_signal_strength);


   void onDCSD_DIAG_Reqdata(std::vector<uint8_t>& DCSD_DIAG_Req); 
   std::list <std::string> dotte_string
      = {"VEHICLE_BRAND" ,"DISPLAY_TYPE","COUNTRY_CODE","START_ANM_MODE","CONT_START_ANM","WELCOME_SOUND","VEH_LINE",
	      "VEH_FUEL_TYPE","DAYNYT_THM_EN","EV_CONN_TYPE","AMP_PRESENT","THEMED_SOUNDS","SIGNAL_SOURCE_ENABLE",
          "PAM_CHIME_ENABLE","PAM_CONFIGURATION","PARK_ASSIST_VOLUME_STRATEGY","PAM_CHIME_MODE"};
   std::vector<std::string> IMU_Fields = {"X_Acceleration","Y_Acceleration","Z_Acceleration","Alpha_Angle","Beta_Angle","Gamma_Angle","Heading","TimeStamp"};     
     
  
private:
    void onPropertyValue(const vhal_v2_0::VehiclePropValue& value, bool updateStatus);
    void regPropsInPropStore();
    void initPropsInPropStore();
    void onContinuousPropertyTimer(const std::vector<int32_t>& properties);
	VehiclePropValuePtr createApPowerStateReq(vhal_v2_0::VehicleApPowerStateReq req, int32_t param);
    bool isContinuousProperty(int32_t propId) const;
    std::shared_ptr<LVDSLibrary>LVDSLibrarySharedPtr = std::make_shared<LVDSLibrary>();
    vhal_v2_0::VehiclePropertyStore* mPropStore;
    VehicleHalClient* mVehicleClient;
    RecurrentTimer mRecurrentTimer;
    android::hardware::automotive::vehicle::V2_0::CAPIVehicleClientGateway m_gateway;
	android::hardware::automotive::vehicle::V2_0::CAPIVehicleTCUServerGateway m_gateway_TCU;
	bool b_CANstream;
	bool b_Dottestream;
    bool b_DTCstream;
    bool b_DLTstream;
	bool b_internalSignal;
	bool b_vinNumbersubscribe;
	bool b_calDatasubscribe;
	uint32_t m_powermode;
	bool b_powermode;
	bool b_SocVersion;
	uint16_t m_vinnumber;
	bool b_vinnumber;
	uint16_t logtigger_dammy = 1U;
	uint16_t dltrtigger_dammy = 1U;
	uint16_t calibtigger_dammy = 1U;
	uint8_t count_calib =0U;
	uint8_t count_dlt =0U;
	bool b_Temperature;
    bool b_supplyvge;
    bool b_CallActiveSTRStatus;
    bool b_STRCancelShutdown;
    bool b_USB_update;
    bool b_USB_update_details;
    std::string propertyId;
    uint32_t displayStatus;
    uint32_t brightness;  
    uint32_t sivistatus;
    bool b_calib_trigger =true;
	bool b_dlt_trigger =true;
	uint32_t m_TCUsignalSignal[TCU_InternalSignals_Index];
    uint32_t m_SLIDataSignal[SLI_data_length];
    bool b_TCUsignalSignal = true;
	bool b_IMUstream;
	bool b_DIDstream;
	bool b_txSignal;
    std::vector<int> vec_val = {1, 1, 1, 1, 1, 1, 1};
    std::vector<int> output = {0, 0, 0, 0, 0, 0, 0};
    const uint32_t powerModeSTR = 14;
    const uint32_t powerModeSTRLM1 = 15;
    const uint32_t powerModeSTRLM2 = 16;
    const uint32_t cancelShutdownSTR = 1;
    uint32_t cancelShutdownSTRInputFromMCU = 0;
	bool b_DIAGstream;
	bool b_PassengerDoorstream;
	bool b_Driverdoorstream;
    bool b_Logdata = true;
    bool b_SLI_data = true;
    bool b_hw_variant = true;
    bool b_SOC_wakeup_reason = true;
    bool b_part_number = true;
    bool b_EOL_Hw_id = true;
    bool b_Packagepath = true;
    std::string m_Packagepath;
	bool b_CheckforUpdateResults = true;
    bool b_UpdateAvailable = true;
    bool b_ConditionsNotMet = true;
    bool b_InstallationStatus = true;
    bool b_UpdateFinished = true;
    bool b_WhatsNewDetails = true;
    bool b_UpdateHistory = true;
    bool b_PendingUpdates = true;
    bool b_ScheduleUpdate = true;
    bool b_AOSP_RBUA_Trigger = true;
    bool b_FotaHmi_InfoResult = true;
    bool b_USBUpdateDetails = true;
    std::string m_CheckforUpdateResults ;
    std::string m_UpdateAvailable;
    std::string m_ConditionsNotMet;
    std::string m_InstallationStatus;
    std::string m_UpdateFinished;
    std::string m_WhatsNewDetails;
    std::string m_UpdateHistory;
    std::string m_PendingUpdates;
    std::string m_ScheduleUpdate;
    std::string m_AOSP_RBUA_Trigger;
    std::string m_FotaHmi_InfoResult;
	std::string m_USBUpdateDetails;
	bool handleDisplayUpInterruptDuringSTR = false;
	bool b_internal_hmi_length = true;
    bool powerModeControlDuringSTR = false;
    const uint32_t powerModeDuringSTR = 50;
    bool flagForPowerModeToCancelSTR = false;
    const uint32_t powerModeSix = 6;
    const uint32_t powerModeNine = 9;
    const uint32_t powerModeEight = 8;
    bool handlePowerStatusForLVDSManagerAfterDeepSleepWakeUp = true;
    const uint32_t powerModeSeventeen = 17;
    const uint32_t powerModeEighteen = 18;
    const uint32_t powerModeEleven = 11;
    const uint32_t powerModeTwelve = 12;
    const uint32_t powerModeThirteen = 13;
    const uint32_t powerModeNineteen = 19;
    const uint32_t inputOneFromUpperLayer = 1;
    int outputForLVDSWakeupAndSleepState = 1;
    bool handleLVDSWakeupAndSleepState = false;
    bool flagForSchedulingTable = false;
    std::vector<uint32_t> camera_data;
    std::vector<uint32_t> audio_data;
    std::vector<uint32_t> hmi_data;
};

}

}
}
}
}
