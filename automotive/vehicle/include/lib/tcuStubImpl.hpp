// Copyright (C) 2014-2019 Bayerische Motoren Werke Aktiengesellschaft (BMW AG)
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef TCU
#define TCU

#include <CommonAPI/CommonAPI.hpp>
#include <v0/VHAL/VHAL_LIFECYCLEStubDefault.hpp>


#include "CAPIVehicleClientGateway.h"

class tcuStubImpl: public v0_1::VHAL::VHAL_LIFECYCLEStubDefault {


     uint32_t lang = 0;
    public:
        tcuStubImpl();
        virtual ~tcuStubImpl();
        void setGateway(android::hardware::automotive::vehicle::V2_0::CAPIVehicleTCUServerGateway* gateway);
        void Receive_MsdTransmissionTime_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _MsdTransmissionTime_TCU, Receive_MsdTransmissionTime_TCUReply_t _reply);
        void Receive_EcallState_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _EcallState_TCU, Receive_EcallState_TCUReply_t _reply);
        void Receive_AcallState_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _AcallState_TCU, Receive_AcallState_TCUReply_t _reply);
        void Receive_UserXcallFeedback_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _UserXcallFeedback_TCU, Receive_UserXcallFeedback_TCUReply_t _reply);
        void Receive_EcallDurationTimer_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _EcallDurationTimer_TCU, Receive_EcallDurationTimer_TCUReply_t _reply);
        void Receive_EcallTpsState_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _EcallTpsState_TCU, Receive_EcallTpsState_TCUReply_t _reply);
        void Receive_XcallStatus_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _XcallStatus_TCU, Receive_XcallStatus_TCUReply_t _reply);
        void Receive_EcallWcbTimer_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _EcallWcbTimer_TCU, Receive_EcallWcbTimer_TCUReply_t _reply);
        void Receive_DmdMuteSual(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _DmdMuteSual, Receive_DmdMuteSualReply_t _reply);
        void Receive_AppUrgMds(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _AppUrgMds, Receive_AppUrgMdsReply_t _reply);
        void Receive_SignalVoyantDysfunction(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _SignalVoyantDysfunction, Receive_SignalVoyantDysfunctionReply_t _reply);
        void Receive_SignalVoyantEtat(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _SignalVoyantEtat, Receive_SignalVoyantEtatReply_t _reply);
        void Receive_BoostrapRetryCount(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _BoostrapRetryCount, Receive_BoostrapRetryCountReply_t _reply);
        void Receive_CloudConnectionStatus_CConnState_connectionGateID(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_connectionGateID, Receive_CloudConnectionStatus_CConnState_connectionGateIDReply_t _reply);
        void Receive_CloudConnectionStatus_CConnState_isConnected(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_isConnected, Receive_CloudConnectionStatus_CConnState_isConnectedReply_t _reply);
        void Receive_CloudConnectionStatus_CConnState_connType(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_connType, Receive_CloudConnectionStatus_CConnState_connTypeReply_t _reply);
        void Receive_CloudConnectionStatus_CConnState_isRoaming(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_isRoaming, Receive_CloudConnectionStatus_CConnState_isRoamingReply_t _reply);
        void Receive_CloudConnectionStatus_CConnState_sigStrength(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_sigStrength, Receive_CloudConnectionStatus_CConnState_sigStrengthReply_t _reply);
        void Receive_CloudConnectionStatus_CConnState_ipVersion(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnState_ipVersion, Receive_CloudConnectionStatus_CConnState_ipVersionReply_t _reply);
        void Receive_CellularNetworkType(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CellularNetworkType, Receive_CellularNetworkTypeReply_t _reply);
        void Receive_NetConnection(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _NetConnection, Receive_NetConnectionReply_t _reply);
        void Receive_Cellsignal(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _Cellsignal, Receive_CellsignalReply_t _reply);
        void Receive_CloudConnectionStatus_CConnCtrl_reqDisable(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _CloudConnectionStatus_CConnCtrl_reqDisable, Receive_CloudConnectionStatus_CConnCtrl_reqDisableReply_t _reply);
        void Receive_SLIRoadType(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _SLIRoadType, Receive_SLIRoadTypeReply_t _reply);
        void Receive_SLISpeedLimit(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _SLISpeedLimit, Receive_SLISpeedLimitReply_t _reply);
        void Receive_SLICountryCode(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t _SLICountryCode, Receive_SLICountryCodeReply_t _reply);
        void getLanguageSelectionOnBootup(const std::shared_ptr<CommonAPI::ClientId> _client, getLanguageSelectionOnBootupReply_t _reply);
        void SetLanguageSelection(uint32_t language);
        void Flash_UnitId(const std::shared_ptr<CommonAPI::ClientId> _client, uint32_t FlashUnitId, Flash_UnitidReply_t _reply);
        void Package_Path(const std::shared_ptr<CommonAPI::ClientId> _client, std::string PackagePath, Package_PathReply_t _reply);
        void sIdsRegisterApp(const std::shared_ptr<CommonAPI::ClientId> _client, std::string _appId, std::string _appName, sIdsRegisterAppReply_t _reply);
        void sIdsUnRegisterApp(const std::shared_ptr<CommonAPI::ClientId> _client, sIdsUnRegisterAppReply_t _reply);
        void sIdsWriteSev(const std::shared_ptr<CommonAPI::ClientId> _client, ::VHAL::sIdsFramework_Types::sStSecurityEvent _sevEvent, ::VHAL::sIdsFramework_Types::sStLogIssuer _issuer1, ::VHAL::sIdsFramework_Types::sStEventSrc _issuer2, ::VHAL::sIdsFramework_Types::sStAction _issuer3, sIdsWriteSevReply_t _reply);
        void sIdsExportSecurityEvents(const std::shared_ptr<CommonAPI::ClientId> _client, sIdsExportSecurityEventsReply_t _reply);
        void sIdsSetVehiclePhase(const std::shared_ptr<CommonAPI::ClientId> _client, std::string _vehiclePhase, sIdsSetVehiclePhaseReply_t _reply);


        void ReceiveFOTA_Check_for_Updates_Result(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTA_Check_for_Updates_result, ReceiveFOTA_Check_for_Updates_ResultReply_t _reply);
        void ReceiveFOTA_Update_Available(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTA_Update_Available, ReceiveFOTA_Update_AvailableReply_t _reply);
        void ReceiveFOTA_Conditions_not_Met(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTA_Conditions_not_Met, ReceiveFOTA_Conditions_not_MetReply_t _reply);
        void ReceiveFOTA_Installation_Status(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTA_Installation_Status, ReceiveFOTA_Installation_StatusReply_t _reply);
        void ReceiveFOTAWhatsNew_Details_Response(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTAWhatsNew_Details_Response, ReceiveFOTAWhatsNew_Details_ResponseReply_t _reply);
        void ReceiveFOTAUpdate_History_Response(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTAUpdate_History_Response, ReceiveFOTAUpdate_History_ResponseReply_t _reply);
        void ReceiveFOTAPending_Updates_Response(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTAPending_Updates_Response, ReceiveFOTAPending_Updates_ResponseReply_t _reply);
        void ReceiveFOTA_Update_Finished(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTA_Update_Finished, ReceiveFOTA_Update_FinishedReply_t _reply);
        void ReceiveFOTASchedule_Update_Response(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTASchedule_Update_Response, ReceiveFOTASchedule_Update_ResponseReply_t _reply);
        void Receive_UIN_TCU(const std::shared_ptr<CommonAPI::ClientId> _client, std::string UIN_TCU_toHMI, Receive_UIN_TCUReply_t _reply);
        void ReceiveTCU_Version_Number(const std::shared_ptr<CommonAPI::ClientId> _client, std::string TCU_version_number, ReceiveTCU_Version_NumberReply_t _reply);
        void ReceiveAOSP_RBUA_Trigger(const std::shared_ptr<CommonAPI::ClientId> _client, std::string AOSP_RBUA_Trigger_Response, ReceiveAOSP_RBUA_TriggerReply_t _reply);
        void FOTAHMI_Info_Result(const std::shared_ptr<CommonAPI::ClientId> _client, std::string FOTAHMI_InfoResult, FOTAHMI_Info_ResultReply_t _reply);
        void USBUpdate_Details(const std::shared_ptr<CommonAPI::ClientId> _client, std::string USBUpdate_details, USBUpdate_DetailsReply_t _reply);

          android::hardware::automotive::vehicle::V2_0::CAPIVehicleTCUServerGateway* m_gateway_TCU = nullptr;

    private:
        android::hardware::automotive::vehicle::V2_0::CAPIVehicleTCUServerGateway& getGateway() const;
};

#endif // TCU
