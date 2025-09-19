// Copyright (C) 2014-2019 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "tcuStubImpl.hpp"
#if defined ANDROID || defined __ANDROID__
#include <log/log_main.h>
#include <android-base/logging.h>
#ifndef _WIN32
#include <unistd.h>
#endif

#define LOG_TAG "vsomeip_client"
//#define LOG_INF(...) fprintf(stdout, __VA_ARGS__), fprintf(stdout, "\n"), (void)__android_log_print(ANDROID_LOG_INFO, LOG_TAG, ##__VA_ARGS__)
//#define LOG_ERR(...) fprintf(stderr, __VA_ARGS__), fprintf(stderr, "\n"), (void)__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, ##__VA_ARGS__)
#define LOG_INF(...)  (void)__android_log_print(ANDROID_LOG_INFO, LOG_TAG, ##__VA_ARGS__)
#define LOG_ERR(...)  (void)__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, ##__VA_ARGS__)
#else
#include <cstdio>
#define LOG_INF(...) fprintf(stdout, __VA_ARGS__), fprintf(stdout, "\n")
#define LOG_ERR(...) fprintf(stderr, __VA_ARGS__), fprintf(stderr, "\n")
#endif

//#include "VehicleHalImpl.h"

tcuStubImpl::tcuStubImpl() {
    
}

tcuStubImpl::~tcuStubImpl() {
}

void tcuStubImpl::Receive_MsdTransmissionTime_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _MsdTransmissionTime_TCU, Receive_MsdTransmissionTime_TCUReply_t _reply){
    ALOGD("Inside Receive_MsdTransmissionTime_TCU %d:" ,_MsdTransmissionTime_TCU );
    
    
m_pImpl_TCU->SendEvent_MsdTransmissionTime_TCU_To_IVI(_MsdTransmissionTime_TCU);
_reply();
}
void tcuStubImpl::Receive_EcallState_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _EcallState_TCU, Receive_EcallState_TCUReply_t _reply){
     ALOGD("Inside Receive_EcallState_TCU %d:" ,_EcallState_TCU );
m_pImpl_TCU->SendEvent_EcallState_TCU_To_IVI(_EcallState_TCU);
_reply();
}
void tcuStubImpl::Receive_AcallState_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _AcallState_TCU, Receive_AcallState_TCUReply_t _reply){
 ALOGD("Inside Receive_AcallState_TCU %d:" ,_AcallState_TCU );
m_pImpl_TCU->SendEvent_AcallState_TCU_To_IVI(_AcallState_TCU);
_reply();
}
void tcuStubImpl::Receive_UserXcallFeedback_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _UserXcallFeedback_TCU, Receive_UserXcallFeedback_TCUReply_t _reply){
 ALOGD("Inside Receive_UserXcallFeedback_TCU %d:" ,_UserXcallFeedback_TCU );
m_pImpl_TCU->SendEvent_UserXcallFeedback_TCU_To_IVI(_UserXcallFeedback_TCU);
_reply();
}
void tcuStubImpl::Receive_EcallDurationTimer_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _EcallDurationTimer_TCU, Receive_EcallDurationTimer_TCUReply_t _reply){
 ALOGD("Inside Receive_EcallDurationTimer_TCU %d:",_EcallDurationTimer_TCU );
m_pImpl_TCU->SendEvent_EcallDurationTimer_TCU_To_IVI(_EcallDurationTimer_TCU);
_reply();
}
void tcuStubImpl::Receive_EcallTpsState_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _EcallTpsState_TCU, Receive_EcallTpsState_TCUReply_t _reply){
ALOGD("Inside Receive_EcallTpsState_TCU %d:", _EcallTpsState_TCU);
m_pImpl_TCU->SendEvent_EcallTpsState_TCU_To_IVI(_EcallTpsState_TCU);
_reply();
}
void tcuStubImpl::Receive_XcallStatus_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _XcallStatus_TCU, Receive_XcallStatus_TCUReply_t _reply){
ALOGD("Inside Receive_XcallStatus_TCU %d:", _XcallStatus_TCU);
m_pImpl_TCU->SendEvent_XcallStatus_TCU_To_IVI(_XcallStatus_TCU);
_reply();
}
void tcuStubImpl::Receive_EcallWcbTimer_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _EcallWcbTimer_TCU, Receive_EcallWcbTimer_TCUReply_t _reply){
ALOGD("Inside Receive_EcallWcbTimer_TCU %d:", _EcallWcbTimer_TCU);
m_pImpl_TCU->SendEvent_EcallWcbTimer_TCU_To_IVI(_EcallWcbTimer_TCU);
_reply();
}
void tcuStubImpl::Receive_DmdMuteSual(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _DmdMuteSual, Receive_DmdMuteSualReply_t _reply){
ALOGD("Inside Receive_DmdMuteSual %d:", _DmdMuteSual);
m_pImpl_TCU->SendEvent_DmdMuteSual_TCU_To_IVI(_DmdMuteSual);
_reply();
}
void tcuStubImpl::Receive_AppUrgMds(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _AppUrgMds, Receive_AppUrgMdsReply_t _reply){
ALOGD("Inside Receive_AppUrgMds %d:", _AppUrgMds);
m_pImpl_TCU->SendEvent_AppUrgMds_TCU_To_IVI(_AppUrgMds);
_reply();
}
void tcuStubImpl::Receive_SignalVoyantDysfunction(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _SignalVoyantDysfunction, Receive_SignalVoyantDysfunctionReply_t _reply){
ALOGD("Inside Receive_SignalVoyantDysfunction %d:", _SignalVoyantDysfunction);
m_pImpl_TCU->SendEvent_SignalVoyantDysfunction_TCU_To_IVI(_SignalVoyantDysfunction);
_reply();
}
void tcuStubImpl::Receive_SignalVoyantEtat(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _SignalVoyantEtat, Receive_SignalVoyantEtatReply_t _reply){
ALOGD("Inside Receive_SignalVoyantEtat %d:", _SignalVoyantEtat);
m_pImpl_TCU->SendEvent_SignalVoyantEtat_TCU_To_IVI(_SignalVoyantEtat);
_reply();
}
void tcuStubImpl::Receive_BoostrapRetryCount(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _BoostrapRetryCount, Receive_BoostrapRetryCountReply_t _reply){
ALOGD("Inside Receive_BoostrapRetryCount %d:", _BoostrapRetryCount);
m_pImpl_TCU->SendEvent_BoostrapRetryCount_TCU_To_IVI(_BoostrapRetryCount);
_reply();
}
void tcuStubImpl::Receive_CloudConnectionStatus_CConnState_connectionGateID(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_connectionGateID, Receive_CloudConnectionStatus_CConnState_connectionGateIDReply_t _reply){
ALOGD("Inside Receive_CloudConnectionStatus_CConnState_connectionGateID %d:", _CloudConnectionStatus_CConnState_connectionGateID);
m_pImpl_TCU->SendEvent_CloudConnectionStatus_CConnState_connectionGateID_TCU_To_IVI(_CloudConnectionStatus_CConnState_connectionGateID);
_reply();
}
void tcuStubImpl::Receive_CloudConnectionStatus_CConnState_isConnected(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_isConnected, Receive_CloudConnectionStatus_CConnState_isConnectedReply_t _reply){
ALOGD("Inside Receive_CloudConnectionStatus_CConnState_connectionGateID %d:", _CloudConnectionStatus_CConnState_isConnected);
m_pImpl_TCU->SendEvent_CloudConnectionStatus_CConnState_isConnected_TCU_To_IVI(_CloudConnectionStatus_CConnState_isConnected);
_reply();
}
void tcuStubImpl::Receive_CloudConnectionStatus_CConnState_connType(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_connType, Receive_CloudConnectionStatus_CConnState_connTypeReply_t _reply){
ALOGD("Inside Receive_CloudConnectionStatus_CConnState_connType %d:", _CloudConnectionStatus_CConnState_connType);
m_pImpl_TCU->SendEvent_CloudConnectionStatus_CConnState_connType_TCU_To_IVI(_CloudConnectionStatus_CConnState_connType);
_reply();
}
void tcuStubImpl::Receive_CloudConnectionStatus_CConnState_isRoaming(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_isRoaming, Receive_CloudConnectionStatus_CConnState_isRoamingReply_t _reply){
ALOGD("Inside Receive_CloudConnectionStatus_CConnState_isRoaming %d:", _CloudConnectionStatus_CConnState_isRoaming);
m_pImpl_TCU->SendEvent_CloudConnectionStatus_CConnState_isRoaming_TCU_To_IVI(_CloudConnectionStatus_CConnState_isRoaming);
_reply();
}
void tcuStubImpl::Receive_CloudConnectionStatus_CConnState_sigStrength(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_sigStrength, Receive_CloudConnectionStatus_CConnState_sigStrengthReply_t _reply){
ALOGD("Inside Receive_CloudConnectionStatus_CConnState_sigStrength %d:", _CloudConnectionStatus_CConnState_sigStrength);
m_pImpl_TCU->SendEvent_CloudConnectionStatus_CConnState_sigStrength_TCU_To_IVI(_CloudConnectionStatus_CConnState_sigStrength);
_reply();
}
void tcuStubImpl::Receive_CloudConnectionStatus_CConnState_ipVersion(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_ipVersion, Receive_CloudConnectionStatus_CConnState_ipVersionReply_t _reply){
ALOGD("Inside Receive_CloudConnectionStatus_CConnState_ipVersion %d:", _CloudConnectionStatus_CConnState_ipVersion);
m_pImpl_TCU->SendEvent_CloudConnectionStatus_CConnState_ipVersion_TCU_To_IVI(_CloudConnectionStatus_CConnState_ipVersion);
_reply();
}
void tcuStubImpl::Receive_CellularNetworkType(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CellularNetworkType, Receive_CellularNetworkTypeReply_t _reply){
ALOGD("Inside Receive_CellularNetworkType %d:", _CellularNetworkType);
m_pImpl_TCU->SendEvent_CellularNetworkType_TCU_To_IVI(_CellularNetworkType);
_reply();
}
void tcuStubImpl::Receive_NetConnection(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _NetConnection, Receive_NetConnectionReply_t _reply){
ALOGD("Inside Receive_NetConnection %d:", _NetConnection);
m_pImpl_TCU->SendEvent_NetConnection_TCU_To_IVI(_NetConnection);
_reply();
}
void tcuStubImpl::Receive_Cellsignal(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _Cellsignal, Receive_CellsignalReply_t _reply){
ALOGD("Inside Receive_Cellsignal %d:", _Cellsignal);
m_pImpl_TCU->SendEvent_Cellsignal_TCU_To_IVI(_Cellsignal);
_reply();
}
void tcuStubImpl::Receive_CloudConnectionStatus_CConnCtrl_reqDisable(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnCtrl_reqDisable, Receive_CloudConnectionStatus_CConnCtrl_reqDisableReply_t _reply){
ALOGD("Inside Receive_CloudConnectionStatus_CConnCtrl_reqDisable %d:", _CloudConnectionStatus_CConnCtrl_reqDisable);
m_pImpl_TCU->SendEvent_CloudConnectionStatus_CConnCtrl_reqDisable_TCU_To_IVI(_CloudConnectionStatus_CConnCtrl_reqDisable);
_reply();
}
void tcuStubImpl::Receive_SLIRoadType(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _SLIRoadType, Receive_SLIRoadTypeReply_t _reply){
ALOGD("Inside Receive_SLIRoadType %d:", _SLIRoadType);
m_pImpl_TCU->SendEvent_SLIRoadType_TCU_To_IVI(_SLIRoadType);
_reply();
}
void tcuStubImpl::Receive_SLISpeedLimit(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _SLISpeedLimit, Receive_SLISpeedLimitReply_t _reply){
ALOGD("Inside Receive_SLISpeedLimit %d:", _SLISpeedLimit);
m_pImpl_TCU->SendEvent_SLISpeedLimit_TCU_To_IVI(_SLISpeedLimit);
_reply();
}
void tcuStubImpl::Receive_SLICountryCode(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _SLICountryCode, Receive_SLICountryCodeReply_t _reply){
ALOGD("Inside Receive_SLICountryCode %d:", _SLICountryCode);
m_pImpl_TCU->SendEvent_SLICountryCode_TCU_To_IVI(_SLICountryCode);
_reply();
}
void tcuStubImpl::getLanguageSelectionOnBootup(const std::shared_ptr<CommonAPI::ClientId> _client, getLanguageSelectionOnBootupReply_t _reply) {
    ALOGD("Inside getLanguageSelectionOnBootup:");
    _reply(lang);
}
void tcuStubImpl::SetLanguageSelection(uint32_t langOnBootup){
    ALOGD("Inside SetLanguageSelection: %u", langOnBootup);
    lang = langOnBootup;
}

void tcuStubImpl::sIdsRegisterApp(const std::shared_ptr<CommonAPI::ClientId> _client, std::string _appId, std::string _appName, sIdsRegisterAppReply_t _reply) {
    ALOGD("Inside sIdsRegisterApp");
    ALOGD("Inside sIdsRegisterApp appId: %s appName: %s", _appId.c_str(), _appName.c_str());
    m_pImpl_TCU->Send_Register_App_TCU_To_IVI(_appId, _appName);
    _reply("sIdsRegisterApp OK from SOMEIP SERVER");
}

void tcuStubImpl::sIdsUnRegisterApp(const std::shared_ptr<CommonAPI::ClientId> _client, sIdsUnRegisterAppReply_t _reply) {
    ALOGD("Inside sIdsUnRegisterApp");
    m_pImpl_TCU->Send_UnRegister_App_TCU_To_IVI();
    _reply("sIdsUnRegisterApp OK from SOMEIP SERVER");
}


void tcuStubImpl::Flash_UnitId(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t FlashUnitId, Flash_UnitidReply_t _reply){
ALOGD("Inside Flash_Unitid %d:", FlashUnitId);
m_pImpl_TCU->SetFlashUnitId(FlashUnitId);
_reply();
}
void tcuStubImpl::Package_Path(const std::shared_ptr<CommonAPI::ClientId> _client, std::string PackagePath, Package_PathReply_t _reply){
ALOGD("Inside PackagePath %s:", PackagePath.c_str());
m_pImpl_TCU->SetPackagePath(PackagePath);
_reply();
}

void tcuStubImpl::sIdsWriteSev(const std::shared_ptr<CommonAPI::ClientId> _client, ::VHAL::sIdsFramework_Types::sStSecurityEvent _sevEvent, ::VHAL::sIdsFramework_Types::sStLogIssuer _issuer1, ::VHAL::sIdsFramework_Types::sStEventSrc _issuer2, ::VHAL::sIdsFramework_Types::sStAction _issuer3, sIdsWriteSevReply_t _reply) {
    ALOGD("Inside sIdsWriteSev");

    StSecurityEvent TcuSecEvent;
    TcuSecEvent.SevCode = _sevEvent.getSevCode();
    TcuSecEvent.SevText = _sevEvent.getSevText();
    TcuSecEvent.SevGravity = _sevEvent.getSevGravity();

    StLogIssuer TcuLogIssuer;
    TcuLogIssuer.sysName = _issuer1.getSysName();
    TcuLogIssuer.appName = _issuer1.getAppName();
    TcuLogIssuer.procName = _issuer1.getProcName();

    StEventSrc TcuEventSrc;
    TcuEventSrc.sysName = _issuer2.getSysName();
    TcuEventSrc.appName = _issuer2.getAppName();
    TcuEventSrc.procName = _issuer2.getProcName();
    TcuEventSrc.srcIP = _issuer2.getSrcIP();
    TcuEventSrc.destIP = _issuer2.getDestIP();

    StAction TcuAction;
    TcuAction.ActCode = _issuer3.getActCode();
    TcuAction.ActText = _issuer3.getActText();

    m_pImpl_TCU->Send_WriteSev_App_TCU_To_IVI(TcuSecEvent, TcuLogIssuer, TcuEventSrc, TcuAction);
    _reply("sIdsWriteSev OK from SOMEIP SERVER");
}

void tcuStubImpl::sIdsExportSecurityEvents(const std::shared_ptr<CommonAPI::ClientId> _client, sIdsExportSecurityEventsReply_t _reply){
    ALOGD("Inside sIdsExportSecurityEvents");
    m_pImpl_TCU->Send_ExportSecurityEvents_TCU_To_IVI();
    _reply("sIdsExportSecurityEvents OK from SOMEIP SERVER");
}

void tcuStubImpl::sIdsSetVehiclePhase(const std::shared_ptr<CommonAPI::ClientId> _client, std::string _vehiclePhase, sIdsSetVehiclePhaseReply_t _reply){
    ALOGD("Inside sIdsSetVehiclePhase");
    m_pImpl_TCU->Send_Vehicle_Phase_TCU_To_IVI(_vehiclePhase);
    _reply("sIdsSetVehiclePhase OK from SOMEIP SERVER");
}

void tcuStubImpl::ReceiveFOTA_Check_for_Updates_Result(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTA_Check_for_Updates_result, ReceiveFOTA_Check_for_Updates_ResultReply_t _reply){
    ALOGD("Inside ReceiveFOTA_Check_for_Updates_Result %s:", FOTA_Check_for_Updates_result.c_str());
    m_pImpl_TCU-> SendEvent_ReceiveFOTA_Check_for_Updates_Result(FOTA_Check_for_Updates_result);
    _reply();
}
void tcuStubImpl::ReceiveFOTA_Update_Available(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTA_Update_Available, ReceiveFOTA_Update_AvailableReply_t _reply){
    ALOGD("Inside ReceiveFOTA_Update_Available %s:", FOTA_Update_Available.c_str());
    m_pImpl_TCU-> SendEvent_ReceiveFOTA_Update_Available(FOTA_Update_Available);
    _reply();
}
void tcuStubImpl::ReceiveFOTA_Conditions_not_Met(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTA_Conditions_not_Met, ReceiveFOTA_Conditions_not_MetReply_t _reply){
    ALOGD("Inside ReceiveFOTA_Conditions_not_Met %s:", FOTA_Conditions_not_Met.c_str());
    m_pImpl_TCU-> SendEvent_ReceiveFOTA_Conditions_not_Met(FOTA_Conditions_not_Met);
    _reply();
}
void tcuStubImpl::ReceiveFOTA_Installation_Status(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTA_Installation_Status, ReceiveFOTA_Installation_StatusReply_t _reply){
    ALOGD("Inside ReceiveFOTA_Installation_Status %s:", FOTA_Installation_Status.c_str());
    m_pImpl_TCU-> SendEvent_ReceiveFOTA_Installation_Status(FOTA_Installation_Status);
    _reply();
}
void tcuStubImpl::ReceiveFOTAWhatsNew_Details_Response(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTAWhatsNew_Details_Response, ReceiveFOTAWhatsNew_Details_ResponseReply_t _reply){
    ALOGD("Inside ReceiveFOTAWhatsNew_Details_Response %s:", FOTAWhatsNew_Details_Response.c_str());
    m_pImpl_TCU-> SendEvent_ReceiveFOTAWhatsNew_Details_Response(FOTAWhatsNew_Details_Response);
    _reply();
}
void tcuStubImpl::ReceiveFOTAUpdate_History_Response(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTAUpdate_History_Response, ReceiveFOTAUpdate_History_ResponseReply_t _reply){
    ALOGD("Inside ReceiveFOTAUpdate_History_Response %s:", FOTAUpdate_History_Response.c_str());
    m_pImpl_TCU-> SendEvent_ReceiveFOTAUpdate_History_Response(FOTAUpdate_History_Response);
    _reply();
}
void tcuStubImpl::ReceiveFOTAPending_Updates_Response(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTAPending_Updates_Response, ReceiveFOTAPending_Updates_ResponseReply_t _reply){
    ALOGD("Inside ReceiveFOTAPending_Updates_Response %s:", FOTAPending_Updates_Response.c_str());
    m_pImpl_TCU-> SendEvent_ReceiveFOTAPending_Updates_Response(FOTAPending_Updates_Response);
    _reply();
}
void tcuStubImpl::ReceiveFOTA_Update_Finished(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTA_Update_Finished, ReceiveFOTA_Update_FinishedReply_t _reply){
    ALOGD("Inside ReceiveFOTAPending_Updates_Response %s:", FOTA_Update_Finished.c_str());
    m_pImpl_TCU-> SendEvent_ReceiveFOTA_Update_Finished(FOTA_Update_Finished);
    _reply();
} 
void tcuStubImpl::ReceiveFOTASchedule_Update_Response(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTASchedule_Update_Response, ReceiveFOTASchedule_Update_ResponseReply_t _reply){
    ALOGD("Inside ReceiveFOTASchedule_Update_Response %s:", FOTASchedule_Update_Response.c_str());
    m_pImpl_TCU-> SendEvent_ReceiveFOTASchedule_Update_Response(FOTASchedule_Update_Response);
    _reply();
} 
void tcuStubImpl::Receive_UIN_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, std::string UIN_TCU_toHMI, Receive_UIN_TCUReply_t _reply){
    ALOGD("Inside Receive_UIN_TCU %s:", UIN_TCU_toHMI.c_str());
    m_pImpl_TCU-> SendEvent_UIN_TCUToHMI(UIN_TCU_toHMI);
    _reply();
}

void tcuStubImpl::ReceiveTCU_Version_Number(const std::shared_ptr<CommonAPI::ClientId> _client, std::string TCU_version_number, ReceiveTCU_Version_NumberReply_t _reply){
    ALOGD("Inside ReceiveTCU_Version_Number %s:", TCU_version_number.c_str());
    m_pImpl_TCU-> SendEvent_TCU_Version_Number_toHMI(TCU_version_number);
    _reply();
}
void tcuStubImpl::ReceiveAOSP_RBUA_Trigger(const std::shared_ptr<CommonAPI::ClientId> _client, std::string AOSP_RBUA_Trigger_Response, ReceiveAOSP_RBUA_TriggerReply_t _reply){
    ALOGD("Inside ReceiveAOSP_RBUA_Trigger %s:", AOSP_RBUA_Trigger_Response.c_str());
    m_pImpl_TCU-> SendEvent_ReceiveAOSP_RBUA_Trigger_Response(AOSP_RBUA_Trigger_Response);
    _reply();
}
void tcuStubImpl::FOTAHMI_Info_Result(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTAHMI_InfoResult, FOTAHMI_Info_ResultReply_t _reply){
    ALOGD("Inside FOTAHMI_Info_Result %s:", FOTAHMI_InfoResult.c_str());
    m_pImpl_TCU-> SendEvent_FOTAHMI_Info_Result(FOTAHMI_InfoResult);
    _reply();
}
void tcuStubImpl::USBUpdate_Details(const std::shared_ptr<CommonAPI::ClientId> _client, std::string USBUpdate_details, USBUpdate_DetailsReply_t _reply){
    ALOGD("Inside USBUpdate_Details %s:", USBUpdate_details.c_str());
    m_pImpl_TCU-> SendEvent_USBUpdate_Details(USBUpdate_details);
    _reply();
}