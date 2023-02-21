/*    ========================================================================
    =                   Doosan Robot Framework Library                        =
    =                   Copyright (c) Doosan Robotics.                        =
    =_______________________________________________________________________  =
    = Title             : Doosan Robot Framwork Library                       =
    = Author            : Lee Jeong-Woo<jeongwoo1.lee@doosan.com>             =
    = Description       : -                                                   =
    = Version           : 1.0 (GL010105) first release                        =
    =                     1.1 (GF020300) add force control                    =
    =                                    add coordinate sytem control function      =
    =                                    fix GetCurrentTool, GetCurrentTCP function = -
    ======================================================================== */

    /*********************************************************************
     * Software License Agreement (BSD License)
     *
     *  Copyright (c) 2019, Doosan Robotics
     *  All rights reserved.
     *
     *  Redistribution and use in source and binary forms, with or without
     *  modification, are permitted provided that the following conditions
     *  are met:
     *
     *   * Redistributions of source code must retain the above copyright
     *     notice, this list of conditions and the following disclaimer.
     *   * Redistributions in binary form must reproduce the above
     *     copyright notice, this list of conditions and the following
     *     disclaimer in the documentation and/or other materials provided
     *     with the distribution.
     *   * Neither the name of the Georgia Institute of Technology nor the names of
     *     its contributors may be used to endorse or promote products derived
     *     from this software without specific prior written permission.
     *
     *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
     *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
     *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
     *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
     *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
     *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
     *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
     *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
     *  POSSIBILITY OF SUCH DAMAGE.
     *********************************************************************/

#pragma once

#if defined(_WIN32)
#if defined(DRFL_EXPORTS)
#define DRFL_API __declspec(dllexport)
#else
#define DRFL_API __declspec(dllimport)
#endif
#endif

#if !defined(DRFL_API)
#define DRFL_API
#endif

#include "DRFL.h"
#include "DRFLEx.h"

#define PRINT_VARNAME
#define ALPHA_1

#define DRCF_TIMEOUT (5000)
#define DRCF_TIMEOUT_STEP (50)

#ifdef PRINT_VARNAME
#define DRAW() {printf("=================================================================================\n");}
#define CHECK_EXCEPTION_RANGE(target, minValue, maxValue) if (target < minValue || target > maxValue) {DRAW(); printf("***ERROR***\r\n\t. File %s\r\n\t. Function: %s\r\n\t. Line code: %d\r\n\t. Variable name: %s\r\n\t. Your input: 0x%0.2x\r\n\t. Valid: from 0x%0.2x to 0x%0.2x\n", __FILE__, __FUNCTION__, __LINE__, #target, target, minValue, maxValue); DRAW(); return ERROR_PARAMETER;}
#define CHECK_EXCEPTION_RANGE_RIGHT(target, minValue, maxValue) if (target < minValue || target >= maxValue) {DRAW(); printf("***ERROR***\r\n\t. File %s\r\n\t. Function: %s\r\n\t. Line code: %d\r\n\t. Variable name: %s\r\n\t. Your input: 0x%0.2x\r\n\t. Valid: from 0x%0.2x to left of 0x%0.2x\n", __FILE__, __FUNCTION__, __LINE__, #target, target, minValue, maxValue); DRAW(); return ERROR_PARAMETER;}
#ifdef ALPHA_1
	#define CHECK_EXCEPTION_RANGE_STRING(target, length) if (strlen(target) >= length) { DRAW(); printf("***ERROR***\r\n\t. File %s\r\n\t. Function: %s\r\n\t. Line code: %d\r\n\t. Variable name: %s\r\n\t. Your input: 0x%0.2x\r\n\t. Max length: 0x%0.2x\n", __FILE__, __FUNCTION__, __LINE__, #target, target, length); DRAW(); return ERROR_PARAMETER;}
	#define CHECK_EXCEPTION_RANGE_STRING_NOT_NULL(target, length) if (strlen(target) >= length && strlen(target) > 0) { DRAW(); printf("***ERROR***\r\n\t. File %s\r\n\t. Function: %s\r\n\t. Line code: %d\r\n\t. Variable name: %s\r\n\t. Your input: 0x%0.2x\r\n\t. Max length: 0x%0.2x\n", __FILE__, __FUNCTION__, __LINE__, #target, target, length); DRAW(); return ERROR_PARAMETER;}
#else //ifdef ALPHA_1
	#define CHECK_EXCEPTION_RANGE_STRING(target, length) if (strlen(target) >= length && strlen(target) > 0) { DRAW(); printf("***ERROR***\r\n\t. File %s\r\n\t. Function: %s\r\n\t. Line code: %d\r\n\t. Variable name: %s\r\n\t. Your input: 0x%0.2x\r\n\t. Max length: 0x%0.2x\n", __FILE__, __FUNCTION__, __LINE__, #target, target, length); DRAW(); return ERROR_PARAMETER;}
#endif //endof ALPHA_1
#else
#define CHECK_EXCEPTION_RANGE(target, minValue, maxValue) if (target < minValue || target > maxValue) {return ERROR_PARAMETER;}
#define CHECK_EXCEPTION_RANGE_RIGHT(target, minValue, maxValue) if (target < minValue || target >= maxValue) {return ERROR_PARAMETER;}
// Check length of the characters with the trailing '\0'
#define CHECK_EXCEPTION_RANGE_STRING(target, length) if (strlen(target) >= length) {return ERROR_PARAMETER;}
#endif /* PRINT_VARNAME */
#define CHECK_EXCEPTION_NULL(p) if(p==NULL) return ERROR_PARAMETER;
#define CHECK_EXCEPTION_PERMISSION(pCtrl) if(_check_permission_ex2(pCtrl) != RET_OK) {return ERROR_CONTROL_PERMISSION;}
#if defined(_WIN32)
#define CHECK_EXCEPTION_START_TIMEOUT(pCtrl) if(_start_timeout_thread_ex2(pCtrl) == FALSE) {return ERROR_FAILED;}
#define CHECK_EXCEPTION_END_TIMEOUT(bTimeout) if (bTimeout == TRUE) {return ERROR_DRCF_TIMEOUT;}
#endif
#define START_THREAD_TIMEOUT(worker, threadTimeout)   {threadTimeout.start(worker);}
#define END_THREAD_TIMEOUT(bTimeout) if (bTimeout == TRUE) {return ERROR_DRCF_TIMEOUT;}
namespace DRAFramework
{
	enum DRFL_RETURN
	{
		RET_OK = 1,
		ERROR_FAILED = 0,
		ERROR_CONNEC_TIMEOUT = 1000,
		ERROR_RECONNECTION = 2000,
		ERROR_CONTROL_PERMISSION = 3000,
		ERROR_PARAMETER = 4000,
		ERROR_DRCF_TIMEOUT = 5000
	};

#ifdef __cplusplus
    extern "C"
    {
		//////////////////////
        /////DRL Wrapping/////
        //////////////////////

		//API for exception
		DRFL_API DRFL_RETURN _check_permission_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API bool _start_timeout_thread_ex2(LPROBOTCONTROL pCtrl);

		//API for calling
		DRFL_API DRFL_RETURN _open_connection_ex2(LPROBOTCONTROL pCtrl, const char* lpszIpAddr = "192.168.137.100", unsigned int usPort = 12345);
		DRFL_API DRFL_RETURN _query_operating_system_ex2(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM* robotSystem);
        DRFL_API DRFL_RETURN _query_operating_state_ex2(LPROBOTCONTROL pCtrl, ROBOT_STATE* robotState);
		DRFL_API DRFL_RETURN _release_protective_stop_ex2(LPROBOTCONTROL pCtrl, RELEASE_MODE eReleaseMode);
		DRFL_API DRFL_RETURN _get_workpiece_weight_ex2(LPROBOTCONTROL pCtrl, float *fParam);
		DRFL_API DRFL_RETURN _measure_tcp_ex2(LPROBOTCONTROL pCtrl, unsigned char iTargetRef, float fTargetPos[4][NUMBER_OF_JOINT], LPMEASURE_TCP_RESPONSE tcpResp);
		DRFL_API DRFL_RETURN _measure_install_pose_ex2(LPROBOTCONTROL pCtrl, float fGradient, float fRotation, bool* ret);
		DRFL_API DRFL_RETURN _change_collaborative_speed_ex2(LPROBOTCONTROL pCtrl, float fSpeed);
		DRFL_API DRFL_RETURN _wait_manual_guide_response_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _change_reduced_speed_ex2(LPROBOTCONTROL pCtrl, float fSpeed);
		DRFL_API DRFL_RETURN _del_user_coord_ex2(LPROBOTCONTROL pCtrl, unsigned char iIdentifier);
		DRFL_API DRFL_RETURN _get_user_coord_ex2(LPROBOTCONTROL pCtrl, unsigned char iIdentifier);
		DRFL_API DRFL_RETURN _get_pattern_point_ex2(LPROBOTCONTROL pCtrl, CONTROL_TRANS_PALLET_POS pControltranspalletpos, LPRESPONSE_TRANS_PALLET_POS pRestranspalletpos);
		DRFL_API DRFL_RETURN _drl_nudge_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _get_user_coord_external_force_ex2(LPROBOTCONTROL pCtrl, unsigned char bIsMonitoring, unsigned char iUserID[MAX_USER_COORD_MONITORING_EXT_FORCE_SIZE]);
		DRFL_API DRFL_RETURN _set_ie_monitoring_ex2(LPROBOTCONTROL pCtrl, unsigned char bStart);
		DRFL_API DRFL_RETURN _get_friction_value_ex2(LPROBOTCONTROL pCtrl,unsigned char iType,unsigned char iSelect[NUMBER_OF_JOINT], float fStart[NUMBER_OF_JOINT], float fRange[NUMBER_OF_JOINT], LPMEASURE_FRICTION_RESPONSE pFricRes);
		DRFL_API DRFL_RETURN _get_serial_port_ex2(LPROBOTCONTROL pCtrl, LPSERIAL_SEARCH pSerOut);
		DRFL_API DRFL_RETURN _get_sub_system_ex2(LPROBOTCONTROL pCtrl, LPINSTALL_SUB_SYSTEM p_tInstallSubSystem);
		DRFL_API DRFL_RETURN _operation_industrial_ethernet_ex2(LPROBOTCONTROL pCtrl, unsigned char nEtherNetIPOpMode, CONFIG_INDUSTRIAL_ETHERNET tConfig);
		DRFL_API DRFL_RETURN _get_sysversion_ex2(LPROBOTCONTROL pCtrl, LPSYSTEM_VERSION_EX pSysVer);
		DRFL_API DRFL_RETURN _update_local_svm_ex2(LPROBOTCONTROL pCtrl, const char szFileName[MAX_SYMBOL_SIZE]);
		DRFL_API DRFL_RETURN _update_network_svm_ex2(LPROBOTCONTROL pCtrl, unsigned char iNetType, const char szIpAddress[16], const char szFileName[MAX_SYMBOL_SIZE]);
		DRFL_API DRFL_RETURN _get_tcp_coord_ex2(LPROBOTCONTROL pCtrl, unsigned char iTargetRef, float fTargetPos1[NUMBER_OF_JOINT], float fTargetPos2[NUMBER_OF_JOINT], float fTargetPos3[NUMBER_OF_JOINT], float fTargetPos4[NUMBER_OF_JOINT], LPMEASURE_TCP_RESPONSE pMeaTcpRes);
		DRFL_API DRFL_RETURN _set_digital_outputs_ex2(LPROBOTCONTROL pCtrl, unsigned char iLocation, unsigned short iCount, GPIO_PORT tPort[MAX_DIGITAL_BURST_SIZE]);
        DRFL_API DRFL_RETURN _set_workpiece_weight_ex2(LPROBOTCONTROL pCtrl, float fWeight = 0.0, float fCog[3] = COG_DEFAULT, COG_REFERENCE eCogRef = COG_REFERENCE_TCP, ADD_UP eAddUp = ADD_UP_REPLACE, float fStartTime = -10000, float fTransitionTIme = -10000);
        DRFL_API DRFL_RETURN _safe_movejx_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
	    DRFL_API DRFL_RETURN _get_system_time_ex2(LPROBOTCONTROL pCtrl, LPSYSTEM_TIME pTime);
		DRFL_API DRFL_RETURN _add_modbus_signal_ex2(LPROBOTCONTROL pCtrl, const char* lpszSymbol, const char* lpszIpAddress, unsigned short nPort, MODBUS_REGISTER_TYPE eRegType, unsigned short iRegIndex, unsigned short nRegValue = 0, unsigned char nSlaveId = 255);
		DRFL_API DRFL_RETURN _add_tcp_ex2(LPROBOTCONTROL pCtrl, const char* lpszSymbol, float fPostion[NUM_TASK]);
		DRFL_API DRFL_RETURN _add_tool_ex2(LPROBOTCONTROL pCtrl, const char* lpszSymbol, float fWeight, float fCog[3], float fInertia[NUM_TASK]);
		DRFL_API DRFL_RETURN _calc_coord_ex2(LPROBOTCONTROL pCtrl, unsigned short nCnt, unsigned short nInputMode, COORDINATE_SYSTEM eTargetRef, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fTargetPos4[NUM_TASK], LPROBOT_POSE pRoboPose);
		DRFL_API DRFL_RETURN _check_force_condition_ex2(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference , bool* pCheckForceCondition);
		DRFL_API DRFL_RETURN _config_program_watch_variable_ex2(LPROBOTCONTROL pCtrl, VARIABLE_TYPE eDivision, DATA_TYPE eType, const char* szName, const char* szData);
		DRFL_API DRFL_RETURN _move_home_ex2(LPROBOTCONTROL pCtrl, MOVE_HOME eMode = MOVE_HOME_MECHANIC, unsigned char bRun = (unsigned char)1);
		DRFL_API DRFL_RETURN _moveb_ex2(LPROBOTCONTROL pCtrl, MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE);
        DRFL_API DRFL_RETURN _movec_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API DRFL_RETURN _movej_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API DRFL_RETURN _movejx_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
		DRFL_API DRFL_RETURN _movel_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
		DRFL_API DRFL_RETURN _move_pause_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _move_periodic_ex2(LPROBOTCONTROL pCtrl, float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned char nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
		DRFL_API DRFL_RETURN _movesj_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
		DRFL_API DRFL_RETURN _move_spiral_ex2(LPROBOTCONTROL pCtrl, TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
		DRFL_API DRFL_RETURN _movesx_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);
		DRFL_API DRFL_RETURN _mwait_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _control_brake_ex2(LPROBOTCONTROL pCtrl, unsigned char iTargetAxis, unsigned char bValue);
		DRFL_API DRFL_RETURN _coord_transform_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eInCoordSystem, COORDINATE_SYSTEM eOutCoordSystem, LPROBOT_POSE pRoPose);
		DRFL_API DRFL_RETURN _drl_pause_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _drl_resume_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _drl_start_ex2(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem, const char* lpszDrlProgram);
		DRFL_API DRFL_RETURN _flange_serial_close_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _flange_serial_open_ex2(LPROBOTCONTROL pCtrl, int baudrate = 115200, BYTE_SIZE eByteSize = BYTE_SIZE_EIGHTBITS, PARITY_CHECK eParity = PARITY_CHECK_NONE, STOP_BITS eStopBits = STOPBITS_ONE);
		DRFL_API DRFL_RETURN _flange_serial_read_ex2(LPROBOTCONTROL pCtrl, LPFLANGE_SER_RXD_INFO pRx, float fTimeout = -1, int nPort = 1); 
		DRFL_API DRFL_RETURN _flange_serial_write_ex2(LPROBOTCONTROL pCtrl, int nSize, char* pSendData, int nPort = 1);
		DRFL_API DRFL_RETURN _focas_cnc_read_param_ex2(LPROBOTCONTROL pCtrl, unsigned short hHandle, short iParamNumber, short iAxisNumber, short iDataLength, LPMACHINE_TENDING_RESPONSE_FOCAS_CNC_PARAM pCnc);
		DRFL_API DRFL_RETURN _focas_connect_ex2(LPROBOTCONTROL pCtrl, short ErrorCode, const char szIpAddr[16], unsigned short iPort, unsigned short hHandle, float fTimeOut, LPMACHINE_TENDING_FOCAS_CONNECT pCnc);
		DRFL_API DRFL_RETURN _focas_disconnect_ex2(LPROBOTCONTROL pCtrl, short ErrorCode, unsigned short hHandle,LPMACHINE_TENDING_FOCAS_DISCONNECT pCnc);
		DRFL_API DRFL_RETURN _focas_get_error_str_ex2(LPROBOTCONTROL pCtrl, unsigned short hHandle, short ErrorCode, const char szErrorString[256], LPMACHINE_TENDING_FOCAS_ERR_STRING pCnc);
		DRFL_API DRFL_RETURN _focas_pmc_read_bit_ex2(LPROBOTCONTROL pCtrl, short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset, MACHINE_TENDING_FOCAS_PMC_DATA tData, LPMACHINE_TENDING_FOCAS_PMC pCnc);
		DRFL_API DRFL_RETURN _focas_pmc_write_bit_ex2(LPROBOTCONTROL pCtrl, short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset, MACHINE_TENDING_FOCAS_PMC_DATA tData, LPMACHINE_TENDING_FOCAS_PMC pCnc);
		DRFL_API DRFL_RETURN _focas_program_num_ex2(LPROBOTCONTROL pCtrl, unsigned short nHandle, LPMACHINE_TENDING_RESPONSE_FOCAS_PROGRAM_NUMBER pCnc);
		DRFL_API DRFL_RETURN _get_control_mode_ex2(LPROBOTCONTROL pCtrl, CONTROL_MODE* pMode);
		DRFL_API DRFL_RETURN _get_current_posj_ex2(LPROBOTCONTROL pCtrl, LPROBOT_POSE pRoboPose);
		DRFL_API DRFL_RETURN _get_current_posx_ex2(LPROBOTCONTROL pCtrl, LPROBOT_TASK_POSE pRoboPose, COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE);
		DRFL_API DRFL_RETURN _get_current_tool_flange_posx_ex2(LPROBOTCONTROL pCtrl, LPROBOT_POSE pRoboPose);
		DRFL_API DRFL_RETURN _get_current_velj_ex2(LPROBOTCONTROL pCtrl, LPROBOT_VEL pRoboVel);
		DRFL_API DRFL_RETURN _get_current_velx_ex2(LPROBOTCONTROL pCtrl, LPROBOT_VEL pRoboVel);
		DRFL_API DRFL_RETURN _change_operation_speed_ex2(LPROBOTCONTROL pCtrl, float fSpeed);
		DRFL_API DRFL_RETURN _get_current_rotm_ex2(LPROBOTCONTROL pCtrl, float (*fRotationMatrix)[3][3], COORDINATE_SYSTEM eTargetRef);
		DRFL_API DRFL_RETURN _open_user_tcp(LPROBOTCONTROL pCtrl, const char* strUniqueID, EN_USER_MODULE_TYPE enType, const char* strIPAddress, int nPort);
		DRFL_API DRFL_RETURN _close_user_tcp(LPROBOTCONTROL pCtrl, const char* strUniqueID);
		DRFL_API DRFL_RETURN _write_user_tcp(LPROBOTCONTROL pCtrl, const char* strUniqueID, const char* pData);
		DRFL_API DRFL_RETURN _write_user_tcp_client(LPROBOTCONTROL pCtrl, const char* strUniqueID, const char* pData, const int nClientID);
		DRFL_API DRFL_RETURN _query_user_tcp_info_(LPROBOTCONTROL pCtrl, const char* strUniqueID, LPUSER_TCP tUserTcpInfo);
		DRFL_API DRFL_RETURN _query_user_tcp_client_info_(LPROBOTCONTROL pCtrl, const char* strUniqueID, const int nClientID, LPUSER_TCP_CLIENT tClientInfo);
		DRFL_API void _set_on_tcp_server_message_received(LPROBOTCONTROL pCtrl, TOnTcpServerMessageReceivedCB pCallbackFunc);
		DRFL_API void _set_on_tcp_client_message_received(LPROBOTCONTROL pCtrl, TOnTcpClientMessageReceivedCB pCallbackFunc);
		DRFL_API void _set_on_tcp_server_notify_new_connect(LPROBOTCONTROL pCtrl, TOnTcpServerNotifyNewConnectCB pCallbackFunc);
		DRFL_API void _set_on_tcp_server_notify_disconnect(LPROBOTCONTROL pCtrl, TOnTcpServerNotifyDisconnectCB pCallbackFunc);
		DRFL_API void _set_on_tcp_client_notify_new_connect(LPROBOTCONTROL pCtrl, TOnTcpClientNotifyNewConnectCB pCallbackFunc);
		DRFL_API void _set_on_tcp_client_notify_disconnect(LPROBOTCONTROL pCtrl, TOnTcpClientNotifyDisconnectCB pCallbackFunc);
		DRFL_API void _set_on_tcp_client_notify_reconnect(LPROBOTCONTROL pCtrl, TOnTcpClientNotifyReconnectCB pCallbackFunc);
		DRFL_API void _set_on_open_connect_result(LPROBOTCONTROL pCtrl, TOnOpenConnectResultCB pCallbackFunc);
		DRFL_API void _set_on_close_connect_result(LPROBOTCONTROL pCtrl, TOnCloseConnectResultCB pCallbackFunc);
		DRFL_API void _set_on_tcp_client_disconnect(LPROBOTCONTROL pCtrl, TOnTcpClientDisconnectCB pCallbackFunc);
		// focas v2 start.
		DRFL_API DRFL_RETURN _focas_cnc_read_param_ex2_v2(LPROBOTCONTROL pCtrl, unsigned short hHandle, short iParamNumber, short iAxisNumber, short iDataLength, LPMACHINE_TENDING_RESPONSE_FOCAS_CNC_PARAM pCnc);
		DRFL_API DRFL_RETURN _focas_connect_ex2_v2(LPROBOTCONTROL pCtrl, short ErrorCode, const char szIpAddr[16], unsigned short iPort, unsigned short hHandle, float fTimeOut, LPMACHINE_TENDING_FOCAS_CONNECT pCnc);
		DRFL_API DRFL_RETURN _focas_disconnect_ex2_v2(LPROBOTCONTROL pCtrl, short ErrorCode, unsigned short hHandle,LPMACHINE_TENDING_FOCAS_DISCONNECT pCnc);
		DRFL_API DRFL_RETURN _focas_get_error_str_ex2_v2(LPROBOTCONTROL pCtrl, unsigned short hHandle, short ErrorCode, const char szErrorString[256], LPMACHINE_TENDING_FOCAS_ERR_STRING pCnc);
		DRFL_API DRFL_RETURN _focas_pmc_read_bit_ex2_v2(LPROBOTCONTROL pCtrl, short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset, MACHINE_TENDING_FOCAS_PMC_DATA tData, LPMACHINE_TENDING_FOCAS_PMC pCnc);
		DRFL_API DRFL_RETURN _focas_pmc_write_bit_ex2_v2(LPROBOTCONTROL pCtrl, short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset, MACHINE_TENDING_FOCAS_PMC_DATA tData, LPMACHINE_TENDING_FOCAS_PMC pCnc);
		DRFL_API DRFL_RETURN _focas_program_num_ex2_v2(LPROBOTCONTROL pCtrl, unsigned short nHandle, LPMACHINE_TENDING_RESPONSE_FOCAS_PROGRAM_NUMBER pCnc);
		
		// focas v2 end.
		////
        DRFL_API DRFL_RETURN _app_weld_disable_digital_ex2(LPROBOTCONTROL pCtrl, unsigned char bMode);
        DRFL_API DRFL_RETURN _constrained_axis_ex2(LPROBOTCONTROL pCtrl, unsigned char iTargetAxis[NUMBER_OF_JOINT], unsigned char iTargetRef, float fTargetStf, float fTargetDump);
        DRFL_API DRFL_RETURN _drl_save_main_ex2(LPROBOTCONTROL pCtrl, unsigned char bMode, unsigned int uLength, const char* lpszString);
        DRFL_API DRFL_RETURN  _drl_stop_ex2(LPROBOTCONTROL pCtrl, unsigned char eStopType = STOP_TYPE_QUICK);
        DRFL_API DRFL_RETURN _drl_start_main_ex2(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem);
        DRFL_API DRFL_RETURN _set_modbus_outputs_ex2(LPROBOTCONTROL pCtrl, MODBUS_REGISTER_BURST pModbusregisterburst);
        DRFL_API DRFL_RETURN _add_modbus_rtu_signal_ex2(LPROBOTCONTROL pCtrl, WRITE_MODBUS_RTU_DATA pWritemodbusrtudata);
        DRFL_API DRFL_RETURN _add_modbus_multi_signal_ex2(LPROBOTCONTROL pCtrl, const char szSymbol[MAX_SYMBOL_SIZE], const char szIpAddr[16], unsigned short iPort, int iSlaveID, unsigned char iRegType, unsigned short iRegIndex, unsigned char iRegCount);
        DRFL_API DRFL_RETURN _del_modbus_multi_signal_ex2(LPROBOTCONTROL pCtrl, const char* szSymbol);
        DRFL_API DRFL_RETURN _add_modbus_rtu_multi_signal_ex2(LPROBOTCONTROL pCtrl, WRITE_MODBUS_RTU_MULTI_DATA pWritemodbusrtumultidata);
        DRFL_API DRFL_RETURN _set_output_register_bit_ex2(LPROBOTCONTROL pCtrl, unsigned char iGprType, unsigned char iGprAddr, const char szData[128]);
        DRFL_API DRFL_RETURN _set_install_pose_ex2(LPROBOTCONTROL pCtrl, float fGradient, float fRotation);
        DRFL_API DRFL_RETURN _change_collision_sensitivity_ex2(LPROBOTCONTROL pCtrl, float fSensitivity);
        DRFL_API DRFL_RETURN _set_safety_function_ex2(LPROBOTCONTROL pCtrl, unsigned char iStopCode[SAFETY_FUNC_LAST]);
        DRFL_API DRFL_RETURN _set_safety_mode_ex2(LPROBOTCONTROL pCtrl, SAFETY_MODE eSafetyMode, SAFETY_MODE_EVENT eSafetyEvent);
        DRFL_API DRFL_RETURN _set_cockpit_ex2(LPROBOTCONTROL pCtrl, unsigned char bEnable, unsigned char iButton[2], unsigned char bRecoveryTeach);
        DRFL_API DRFL_RETURN _get_user_home_ex2(LPROBOTCONTROL pCtrl, LPROBOT_POSE pPosition);
		DRFL_API DRFL_RETURN _set_modbus_multi_output_ex2(LPROBOTCONTROL pCtrl, const char* szSymbol, unsigned char iRegCount, unsigned short iRegValue[MAX_MODBUS_REGISTER_PER_DEVICE]);
        DRFL_API DRFL_RETURN _get_input_register_bit_ex2(LPROBOTCONTROL pCtrl, unsigned char iGprType, unsigned char iGprAddr, unsigned char iInOut, LPIETHERNET_SLAVE_RESPONSE_DATA_EX pEthernetSlaveData);
        DRFL_API DRFL_RETURN _set_general_range_ex2(LPROBOTCONTROL pCtrl, GENERAL_RANGE tNormal, GENERAL_RANGE tReduced);
        DRFL_API DRFL_RETURN _set_joint_range_ex2(LPROBOTCONTROL pCtrl, JOINT_RANGE tNormal, JOINT_RANGE tReduced);
        DRFL_API DRFL_RETURN _set_safety_io_ex2(LPROBOTCONTROL pCtrl, unsigned char iIO[TYPE_LAST][NUM_SAFETY]);
        DRFL_API DRFL_RETURN _add_tool_shape_ex2(LPROBOTCONTROL pCtrl, CONFIG_TOOL_SHAPE_SYMBOL pConfigtoolshapesymbol);
        DRFL_API DRFL_RETURN _add_safety_zone_ex2(LPROBOTCONTROL pCtrl, const char* szIdentifier, const char* szAlias, unsigned char iZoneType, SAFETY_ZONE_PROPERTY_DATA tZoneProperty, SAFETY_ZONE_SHAPE tShape);
        DRFL_API DRFL_RETURN _config_setting_enable_ex2(LPROBOTCONTROL pCtrl, unsigned short wPreviousCmdid, unsigned int iRefCrc32);
        DRFL_API DRFL_RETURN _config_configurable_io_ex2(LPROBOTCONTROL pCtrl, unsigned char niIO[TYPE_LAST][NUM_DIGITAL]);
        DRFL_API DRFL_RETURN _safe_move_home_ex2(LPROBOTCONTROL pCtrl, unsigned char bRun);
        DRFL_API DRFL_RETURN _safe_movej_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API DRFL_RETURN _safe_drl_start_ex2(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem);
        DRFL_API DRFL_RETURN _safe_move_userhome_ex2(LPROBOTCONTROL pCtrl, unsigned char bRun);
        DRFL_API DRFL_RETURN _set_system_power_ex2(LPROBOTCONTROL pCtrl, unsigned char iTarget, unsigned char iPower);
		DRFL_API DRFL_RETURN _get_desired_posj_ex2(LPROBOTCONTROL pCtrl, LPROBOT_POSE robotPose);
        DRFL_API DRFL_RETURN _get_desired_posx_ex2(LPROBOTCONTROL pCtrl, LPROBOT_POSE robotPose, COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE);
        DRFL_API DRFL_RETURN _get_desired_velx_ex2(LPROBOTCONTROL pCtrl, LPROBOT_VEL robotVel);
        DRFL_API DRFL_RETURN _get_digital_output_ex2(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool* retVal);
        DRFL_API DRFL_RETURN _get_external_torque_ex2(LPROBOTCONTROL pCtrl, LPROBOT_FORCE robotForce);
        DRFL_API DRFL_RETURN _get_joint_torque_ex2(LPROBOTCONTROL pCtrl, LPROBOT_FORCE robotForce);
        DRFL_API DRFL_RETURN _get_library_version_ex2(LPROBOTCONTROL pCtrl, string* libVersion);
        DRFL_API DRFL_RETURN _get_program_state_ex2(LPROBOTCONTROL pCtrl, DRL_PROGRAM_STATE* programState);
        DRFL_API DRFL_RETURN _get_robot_control_mode_ex2(LPROBOTCONTROL pCtrl, unsigned char* controlMode);
        DRFL_API DRFL_RETURN  _get_robot_state_ex2(LPROBOTCONTROL pCtrl, ROBOT_STATE* robotState);
        DRFL_API DRFL_RETURN _get_robot_system_ex2(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM* robotSystem);
        DRFL_API DRFL_RETURN _get_solution_space_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], unsigned char* solutionSpace);
        DRFL_API DRFL_RETURN _get_system_version_ex2(LPROBOTCONTROL pCtrl, LPSYSTEM_VERSION pVersion, bool* retVal);
        DRFL_API DRFL_RETURN _get_tcp_ex2(LPROBOTCONTROL pCtrl, string* retTcp);
        DRFL_API DRFL_RETURN _get_tool_ex2(LPROBOTCONTROL pCtrl, string* retTool);
        DRFL_API DRFL_RETURN _get_tool_analog_input_ex2(LPROBOTCONTROL pCtrl, int nCh, float* retTool);
        DRFL_API DRFL_RETURN _get_tool_digital_output_ex2(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool* retVal);
        DRFL_API DRFL_RETURN _get_tool_force_ex2(LPROBOTCONTROL pCtrl, LPROBOT_FORCE robotForce);
        DRFL_API DRFL_RETURN _get_tool_weight_ex2(LPROBOTCONTROL pCtrl, unsigned char iMode, LPMEASURE_TOOL_RESPONSE retTool, float fWeight, float fXYZ_X, float fXYZ_Y, float fXYZ_Z );
        DRFL_API DRFL_RETURN _is_focas_alive_ex2(LPROBOTCONTROL pCtrl, unsigned short nHandle, LPFOCAS_IS_ALIVE_RESPONSE retFocasAlive);
        DRFL_API DRFL_RETURN _manage_access_control_ex2(LPROBOTCONTROL pCtrl, MANAGE_ACCESS_CONTROL eAccessControl = MANAGE_ACCESS_CONTROL_REQUEST);
        DRFL_API DRFL_RETURN _measure_friction_ex2(LPROBOTCONTROL pCtrl, unsigned char iType, unsigned char iSelect[NUMBER_OF_JOINT], float fStart[NUMBER_OF_JOINT], float fRange[NUMBER_OF_JOINT], LPMEASURE_FRICTION_RESPONSE retFriction);
        DRFL_API DRFL_RETURN _overwrite_user_cart_coord_ex2(LPROBOTCONTROL pCtrl, bool bTargetUpdate, int iReqId, float fTargetPos[NUM_TASK], int* retOverWrite, COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
        DRFL_API DRFL_RETURN _drl_check_syntax_ex2(LPROBOTCONTROL pCtrl, unsigned int iTextLength, const char* lpszString);
        DRFL_API DRFL_RETURN _reset_safety_config_ex2(LPROBOTCONTROL pCtrl);
        DRFL_API DRFL_RETURN _reset_workpiece_weight_ex2(LPROBOTCONTROL pCtrl);
        DRFL_API DRFL_RETURN _get_last_alarm_ex2(LPROBOTCONTROL pCtrl, LPLOG_ALARM retLogAlarm);
        DRFL_API DRFL_RETURN _enable_friction_compensation_ex2(LPROBOTCONTROL pCtrl, unsigned char iSelect[NUMBER_OF_JOINT], float fPositive[4][NUMBER_OF_JOINT], float fNegative[4][NUMBER_OF_JOINT], float fTemperature[NUMBER_OF_JOINT]);
        DRFL_API DRFL_RETURN _get_robot_mode_ex2(LPROBOTCONTROL pCtrl, ROBOT_MODE* robotMode);
        DRFL_API DRFL_RETURN _get_current_solution_space_ex2(LPROBOTCONTROL pCtrl, unsigned char* retSolutionSpace);
        DRFL_API DRFL_RETURN _get_robot_speed_mode_ex2(LPROBOTCONTROL pCtrl, SPEED_MODE* speedMode);
        DRFL_API DRFL_RETURN _save_sub_program_ex2(LPROBOTCONTROL pCtrl, int iTargetType, const char* szFileName, const char* lpszTextString);
        DRFL_API DRFL_RETURN _servoj_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
        DRFL_API DRFL_RETURN _servol_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime);
        DRFL_API DRFL_RETURN _servo_off_ex2(LPROBOTCONTROL pCtrl, STOP_TYPE eStopType, int* retServoOff);
        DRFL_API DRFL_RETURN _servo_on_ex2(LPROBOTCONTROL pCtrl);
        DRFL_API DRFL_RETURN _set_analog_output_ex2(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue);
        DRFL_API DRFL_RETURN _set_digital_output_ex2(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        DRFL_API DRFL_RETURN _set_ip_address_ex2(LPROBOTCONTROL pCtrl, unsigned char iUsage, unsigned char iIpType, const char* szHostIp, const char* szSubnet, const char* szGateway, const char* szPrimaryDNS, const char* szSecondaryDNS);
        DRFL_API DRFL_RETURN _set_modbus_output_ex2(LPROBOTCONTROL pCtrl, const char* lpszSymbol, unsigned short nValue);
        DRFL_API DRFL_RETURN _set_mode_analog_input_ex2(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT); 
        DRFL_API DRFL_RETURN _set_mode_analog_output_ex2(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT);
        DRFL_API DRFL_RETURN _set_mode_tool_analog_input_ex2(LPROBOTCONTROL pCtrl, int nCh, GPIO_ANALOG_TYPE eAnalogType);
        DRFL_API DRFL_RETURN _set_robot_control_ex2(LPROBOTCONTROL pCtrl, ROBOT_CONTROL eControl);
        DRFL_API DRFL_RETURN _set_robot_system_ex2(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem);
        DRFL_API DRFL_RETURN _set_tcp_ex2(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        DRFL_API DRFL_RETURN _set_tool_ex2(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        DRFL_API DRFL_RETURN _set_tool_digital_output_ex2(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        ////
		//
		DRFL_API DRFL_RETURN _jog_ex2(LPROBOTCONTROL pCtrl, JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity);
		DRFL_API DRFL_RETURN _del_modbus_signal_ex2(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
		DRFL_API DRFL_RETURN _del_tcp_ex2(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
		DRFL_API DRFL_RETURN _del_tool_ex2(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
		DRFL_API DRFL_RETURN _del_tool_shape_ex2(LPROBOTCONTROL pCtrl, const char* pcArg);
		DRFL_API DRFL_RETURN _get_analog_input_ex2(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float *pGetAnalog);
		DRFL_API DRFL_RETURN _get_digital_input_ex2(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool* retVal);
		DRFL_API DRFL_RETURN _get_modbus_input_ex2(LPROBOTCONTROL pCtrl, const char* lpszSymbol, unsigned short *pModbusInput);
		DRFL_API DRFL_RETURN _get_tool_digital_input_ex2(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool* retVal);
		DRFL_API DRFL_RETURN _get_user_cart_coord_ex2(LPROBOTCONTROL pCtrl, int iReqId, LPUSER_COORDINATE GetUserCartCoord);
		DRFL_API DRFL_RETURN _drl_break2_ex2(LPROBOTCONTROL pCtrl, int nLine, const char* szFile);
		DRFL_API DRFL_RETURN _get_current_pose_ex2(LPROBOTCONTROL pCtrl, ROBOT_SPACE eSpaceType, LPROBOT_POSE GetCurrentPose);
		DRFL_API DRFL_RETURN _parallel_axis1_ex2(LPROBOTCONTROL pCtrl, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);
        DRFL_API DRFL_RETURN _parallel_axis2_ex2(LPROBOTCONTROL pCtrl, float fTargetVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);
        DRFL_API DRFL_RETURN _set_user_cart_coord1_ex2(LPROBOTCONTROL pCtrl, int iReqId, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eTargetRef, int *pSetUserCartCoord1);
        DRFL_API DRFL_RETURN _set_user_cart_coord2_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[3][NUM_TASK], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef, int *pSetUserCartCoord2);
        DRFL_API DRFL_RETURN _set_user_cart_coord3_ex2(LPROBOTCONTROL pCtrl, float fTargetVec[2][3], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef, int *pSetUserCartCoord3);
        DRFL_API DRFL_RETURN _align_axis1_ex2(LPROBOTCONTROL pCtrl, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);
        DRFL_API DRFL_RETURN _align_axis2_ex2(LPROBOTCONTROL pCtrl, float fTargetVec[3], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);
		DRFL_API DRFL_RETURN _hold2run_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _stop_ex2(LPROBOTCONTROL pCtrl, STOP_TYPE eStopType = STOP_TYPE_QUICK);
		//
		///
		DRFL_API DRFL_RETURN _set_nudge_ex2(LPROBOTCONTROL pCtrl, unsigned char bEnable, float fInputForce, float fDelayTime);
		DRFL_API DRFL_RETURN _set_world_coord_ex2(LPROBOTCONTROL pCtrl, unsigned char iType, float fPosition[NUMBER_OF_JOINT]);
		DRFL_API DRFL_RETURN _set_tool_shape_ex2(LPROBOTCONTROL pCtrl, string pcArg);
		DRFL_API DRFL_RETURN _del_safety_zone_ex2(LPROBOTCONTROL pCtrl, string szIdentifier);
		DRFL_API DRFL_RETURN _reset_sequence_ex2(LPROBOTCONTROL pCtrl, unsigned char iIdentifier);
		DRFL_API DRFL_RETURN _safe_movel_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
		DRFL_API DRFL_RETURN _get_tool_shape_ex2(LPROBOTCONTROL pCtrl, string *lpszString);
		DRFL_API DRFL_RETURN _set_tool_digital_output_level_ex2(LPROBOTCONTROL pCtrl, int nLv);
		DRFL_API DRFL_RETURN _set_tool_digital_output_type_ex2(LPROBOTCONTROL pCtrl, int nPort, OUTPUT_TYPE eOutputType);
		DRFL_API DRFL_RETURN _set_user_home_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _speedj_ex2(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
		DRFL_API DRFL_RETURN _speedl_ex2(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_TASK], float fTargetAcc[2], float fTargetTime);
		DRFL_API DRFL_RETURN _system_shut_down_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _tp_get_user_input_response_ex2(LPROBOTCONTROL pCtrl, string strUserInput);
		DRFL_API DRFL_RETURN _tp_popup_response_ex2(LPROBOTCONTROL pCtrl, POPUP_RESPONSE eRes);
        DRFL_API DRFL_RETURN _get_ip_address_ex2(LPROBOTCONTROL pCtrl, unsigned char iIpType, LPSYSTEM_IPADDRESS ip);
		DRFL_API DRFL_RETURN _trans_ex2(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_TASK], float fOffset[NUM_TASK], COORDINATE_SYSTEM eSourceRef, COORDINATE_SYSTEM eTargetRef, LPROBOT_POSE iRobot);
        DRFL_API DRFL_RETURN _get_control_space_ex2(LPROBOTCONTROL pCtrl, ROBOT_SPACE* iRobotSpace);
		///

		//API Second Priority Start
		DRFL_API DRFL_RETURN _app_weld_adj_motion_offset_ex2(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ);
		DRFL_API DRFL_RETURN _app_weld_adj_welding_cond_analog_ex2(LPROBOTCONTROL pCtrl, unsigned char bRealTime, unsigned char bResetFlag, float fTargetVol, float fFeedingVel, float fTargetVel, float fOffsetY, float fOffsetZ, float fWidthRate);
		DRFL_API DRFL_RETURN _app_weld_adj_welding_cond_digital_ex2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_ADJUST pConfigdigitalweldingadjust);
		DRFL_API DRFL_RETURN _app_weld_enable_analog_ex2(LPROBOTCONTROL pCtrl, CONFIG_ANALOG_WELDING_INTERFACE pConfiganalogweldinginterface);
		DRFL_API DRFL_RETURN _app_weld_enable_digital_ex2(LPROBOTCONTROL pCtrl, unsigned char bMode);
		DRFL_API DRFL_RETURN _app_weld_reset_interface_ex2(LPROBOTCONTROL pCtrl, unsigned char bReset);
		DRFL_API DRFL_RETURN _app_weld_set_interface_eip_m2r_monitoring_ex2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_MONITORING pConfigdigitalweldinginterfacemonitoring);
		DRFL_API DRFL_RETURN _app_weld_set_interface_eip_m2r_other_ex2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_OTHER pConfigdigitalweldinginterfacemonitoring);
		DRFL_API DRFL_RETURN _app_weld_set_interface_eip_r2m_condition_ex2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_CONDITION pConfigdigitalweldinginterfacecondition);
		DRFL_API DRFL_RETURN _app_weld_set_interface_eip_r2m_mode_ex2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_MODE pConfigdigitalweldinginterfacemode);
		DRFL_API DRFL_RETURN _app_weld_set_interface_eip_r2m_option_ex2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_OPTION pConfigdigitalweldinginterfaceoption);
		DRFL_API DRFL_RETURN _app_weld_set_interface_eip_r2m_process_ex2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS pConfigdigitalweldinginterfaceprocess);
		DRFL_API DRFL_RETURN _app_weld_set_interface_eip_r2m_test_ex2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_TEST pConfigdigitalweldinginterfacetest);
		DRFL_API DRFL_RETURN _app_weld_set_weld_cond_analog_ex2(LPROBOTCONTROL pCtrl, unsigned char iVirtualWelding, float fTargetVoltage, float fTargetCurrent, float fTargetVel, float fMinVel, float fMaxVel, float fTargetFeedingSpeed);
		DRFL_API DRFL_RETURN _app_weld_set_weld_cond_digital_ex2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_CONDITION pConfigdigitalweldingcondition);
		DRFL_API DRFL_RETURN _app_weld_weave_cond_circular_ex2(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ, float fGradient, float fwWdt[2], float fwT[2]);
		DRFL_API DRFL_RETURN _app_weld_weave_cond_sinusoidal_ex2(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle);
		DRFL_API DRFL_RETURN _app_weld_weave_cond_trapezoidal_ex2(LPROBOTCONTROL pCtrl, CONFIG_TRAPEZOID_WEAVING_SETTING pConfigtrapezoidweavingsetting);
		DRFL_API DRFL_RETURN _app_weld_weave_cond_zigzag_ex2(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle);
		DRFL_API DRFL_RETURN _amoveb_ex2(LPROBOTCONTROL pCtrl, MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, MOVE_MODE eMoveMode, MOVE_REFERENCE eMoveReference);
		DRFL_API DRFL_RETURN _amovec_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
		DRFL_API DRFL_RETURN _amovej_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
		DRFL_API DRFL_RETURN _amovejx_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
		DRFL_API DRFL_RETURN _amovel_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
		DRFL_API DRFL_RETURN _amove_periodic_ex2(LPROBOTCONTROL pCtrl, float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned int nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
		DRFL_API DRFL_RETURN _amovesj_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
		DRFL_API DRFL_RETURN _amove_spiral_ex2(LPROBOTCONTROL pCtrl, TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
		DRFL_API DRFL_RETURN _amovesx_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);
		DRFL_API DRFL_RETURN _get_conveyor_object_ex2(LPROBOTCONTROL pCtrl, unsigned char iConId, float fTimeout, unsigned char iContainerType, POSITION tPosObjCoord, int *pGetConveyorObj);
		DRFL_API DRFL_RETURN _get_cpu_usage_ex2(LPROBOTCONTROL pCtrl, LPSYSTEM_CPUUSAGE iCpuUsage);
		DRFL_API DRFL_RETURN _get_disk_usage_ex2(LPROBOTCONTROL pCtrl, LPSYSTEM_DISKSIZE iDiskUsage);
		DRFL_API DRFL_RETURN _get_fts_value_ex2(LPROBOTCONTROL pCtrl, LPCALIBRATE_FTS_RESPONSE iFtsValue);
		DRFL_API DRFL_RETURN _get_kernel_log_ex2(LPROBOTCONTROL pCtrl, const char* iStrKernel);
		DRFL_API DRFL_RETURN _measure_welding_tcp_ex2(LPROBOTCONTROL pCtrl, unsigned char iMode, float fStickout, float fTargetPos[9][NUMBER_OF_JOINT]);
		DRFL_API DRFL_RETURN _get_conveyor_coord_ex2(LPROBOTCONTROL pCtrl, POSITION tPosTeachPointQ, unsigned char nTeachCount, POSITION tPosTeachPointP[5], unsigned int EncoderCount[5], LPMEASURE_CONVEYOR_COORD_RESPONSE iConvCoord);
		DRFL_API DRFL_RETURN _get_conveyor_coord2_ex2(LPROBOTCONTROL pCtrl, int nResolution, LPMEASURE_CONVEYOR_COORD_RESPONSE iConvCoord2);
		DRFL_API DRFL_RETURN _get_conveyor_distance_ex2(LPROBOTCONTROL pCtrl, unsigned int nFilterSize, LPMEASURE_CONVEYOR_DISTANCE_RESPONSE iConvDist);
		DRFL_API DRFL_RETURN _add_conveyor_ex2(LPROBOTCONTROL pCtrl, const char* szName, CONVEYOR_BASIC tBASIC, CONVEYOR_COORD_EX tCoord, CONVEYOR_DISTANCE tDistance);
		DRFL_API DRFL_RETURN _del_conveyor_ex2(LPROBOTCONTROL pCtrl, const char* szName);
		DRFL_API DRFL_RETURN _set_conveyor_monitoring_ex2(LPROBOTCONTROL pCtrl, const char* szName, unsigned char bStart);
		DRFL_API DRFL_RETURN _calc_conveyor_param_ex2(LPROBOTCONTROL pCtrl, unsigned char iType, unsigned char iEncoderChannel, unsigned char iTriggerChannel, unsigned char iTriggerEdgeType, float fTriggerMuteTime, char iChannel[2], unsigned char iValue[2]);
		DRFL_API DRFL_RETURN _calc_conveyor_param2_ex2(LPROBOTCONTROL pCtrl, int iDistance2Count, POSITION tPosConCoord, unsigned char iTargetRef);
		DRFL_API DRFL_RETURN _drl_step_run2_ex2(LPROBOTCONTROL pCtrl, unsigned int nLine);
		DRFL_API DRFL_RETURN _reset_encorder_ex2(LPROBOTCONTROL pCtrl, unsigned char iChannel);
		DRFL_API DRFL_RETURN _set_conveyor_ex2(LPROBOTCONTROL pCtrl, const char* szName, unsigned char *pSetConveyor);
		DRFL_API DRFL_RETURN _set_conveyor_ex_ex2(LPROBOTCONTROL pCtrl, const char* szName, CONVEYOR_BASIC tBASIC, CONVEYOR_COORD_EX tCoord, CONVEYOR_DISTANCE tDistance, unsigned char *pSetConveyorex);
		DRFL_API DRFL_RETURN _set_conveyor_track_ex2(LPROBOTCONTROL pCtrl, unsigned char iConId, unsigned char bTracking, unsigned char bMate, float fDuration, float fDummy[5]);
		DRFL_API DRFL_RETURN _set_digital_welding_monitoring_mode_ex2(LPROBOTCONTROL pCtrl, unsigned char bEnable);
		DRFL_API DRFL_RETURN _set_digital_welding_signal_output_ex2(LPROBOTCONTROL pCtrl, unsigned char cDataType, float fData);
		DRFL_API DRFL_RETURN _set_encorder_mode_ex2(LPROBOTCONTROL pCtrl, unsigned char iChannel, unsigned char iABMode, unsigned char iZMode, unsigned char iSMode, unsigned char iInvMode, unsigned int  nPulseAZ);
		DRFL_API DRFL_RETURN _set_encorder_polarity_ex2(LPROBOTCONTROL pCtrl, unsigned char iChannel, unsigned char iPolarity[ENCORDER_POLARITY_LAST]);
		DRFL_API DRFL_RETURN _set_process_button_ex2(LPROBOTCONTROL pCtrl, unsigned char iUsage);
		DRFL_API DRFL_RETURN _set_remote_control_ex2(LPROBOTCONTROL pCtrl, unsigned char bEnable, CONFIG_IO_FUNC tFunc[TYPE_LAST][NUM_REMOTE_CONTROL]);
		DRFL_API DRFL_RETURN _set_system_time_ex2(LPROBOTCONTROL pCtrl, const char* szData, const char* szTime);
		DRFL_API DRFL_RETURN _set_time_sync_ex2(LPROBOTCONTROL pCtrl, unsigned char iUsage);
		DRFL_API DRFL_RETURN _set_welding_cockpit_setting_ex2(LPROBOTCONTROL pCtrl, unsigned char bEnable, unsigned char bWeldingType);
		DRFL_API DRFL_RETURN _update_license_ex2(LPROBOTCONTROL pCtrl, const char* lpszLicense);
		//API Second Priority End

		//API Low Priority Start
		DRFL_API DRFL_RETURN _alter_motion_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK]);
		DRFL_API DRFL_RETURN _check_motion_ex2(LPROBOTCONTROL pCtrl, int *CheckMotion);
		DRFL_API DRFL_RETURN _check_motion_ex_ex2(LPROBOTCONTROL pCtrl, int *CheckMotionEx);
		DRFL_API DRFL_RETURN _check_orientation_condition_abs_ex2(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin[NUM_TASK], float fTargetMax[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
		DRFL_API DRFL_RETURN _check_orientation_condition_rel_ex2(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
		DRFL_API DRFL_RETURN _check_position_condition_ex2(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], MOVE_MODE eMode = MOVE_MODE_ABSOLUTE, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
		DRFL_API DRFL_RETURN _check_position_condition_abs_ex2(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
		DRFL_API DRFL_RETURN _check_position_condition_rel_ex2(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
		DRFL_API DRFL_RETURN _config_kt_factory_makers_ex2(LPROBOTCONTROL pCtrl, int bEnable, const char* szIpAddress, int nPort, const char* szDeviceId, const char* szDevicePw, const char* szGatewayId);
		DRFL_API DRFL_RETURN _multi_jog_ex2(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], MOVE_REFERENCE eMoveReference, float fVelocity);
		DRFL_API DRFL_RETURN _move_resume_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _delete_file_ex2(LPROBOTCONTROL pCtrl, const char* szFileName);
		DRFL_API DRFL_RETURN _disable_alter_motion_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _drl_break_ex2(LPROBOTCONTROL pCtrl, int nLineNum);
		DRFL_API DRFL_RETURN _drl_step_run_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _enable_alter_motion_ex2(LPROBOTCONTROL pCtrl, int iCycleTime, PATH_MODE ePathMode, COORDINATE_SYSTEM eTargetRef, float fLimitDpos[2], float fLimitDposPer[2]);
		DRFL_API DRFL_RETURN _fkin_ex2(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_JOINT], COORDINATE_SYSTEM eTargetRef , LPROBOT_POSE pFkin);
		DRFL_API DRFL_RETURN _get_normal_ex2(LPROBOTCONTROL pCtrl, float fTargetPosX[NUMBER_OF_JOINT], float fTargetPosY[NUMBER_OF_JOINT], float fTargetPosZ[NUMBER_OF_JOINT],  LPNORMAL_VECTOR_RESPONSE pGetNormal);
		DRFL_API DRFL_RETURN _get_orientation_error_ex2(LPROBOTCONTROL pCtrl, float fPosition1[NUM_TASK], float fPosition2[NUM_TASK], TASK_AXIS eTaskAxis, float *pGetOrientationError);
		DRFL_API DRFL_RETURN _ikin_ex2(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef , LPROBOT_POSE pIkin);
		DRFL_API DRFL_RETURN _is_done_bolt_tightening_ex2(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetTor = 0.f, float fTimeout = 0.f);
		DRFL_API DRFL_RETURN _release_compliance_ctrl_ex2(LPROBOTCONTROL pCtrl);
		DRFL_API DRFL_RETURN _release_force_ex2(LPROBOTCONTROL pCtrl, float fTargetTime = 0.f);
		DRFL_API DRFL_RETURN _report_tcp_client_ex2(LPROBOTCONTROL pCtrl, unsigned char iIdentifier, LPREPORT_TCP_CLIENT pReportTcpClient);
		DRFL_API DRFL_RETURN _response_restore_controller_ex2(LPROBOTCONTROL pCtrl, unsigned char iProcess);
		DRFL_API DRFL_RETURN _set_deflection_comp_mode_ex2(LPROBOTCONTROL pCtrl, unsigned char bEnable);
		DRFL_API DRFL_RETURN _set_desired_force_ex2(LPROBOTCONTROL pCtrl, float fTargetForce[NUM_TASK], unsigned char iTargetDirection[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f, FORCE_MODE eForceMode = FORCE_MODE_ABSOLUTE);
		//DRFL_API DRFL_RETURN _set_license_option_ex2(LPROBOTCONTROL pCtrl, unsigned int nOption);
		DRFL_API DRFL_RETURN _set_monitoring_control_ex2(LPROBOTCONTROL pCtrl, unsigned char iControl);
		DRFL_API DRFL_RETURN _set_oscillation_check_ex2(LPROBOTCONTROL pCtrl, unsigned int puArg);
		DRFL_API DRFL_RETURN _set_package_restore_ex2(LPROBOTCONTROL pCtrl, const char szVersName[MAX_SYMBOL_SIZE]);
		DRFL_API DRFL_RETURN _set_ref_coord_ex2(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eTargetCoordSystem);
		DRFL_API DRFL_RETURN _set_restore_list_ex2(LPROBOTCONTROL pCtrl, LPPACKAGE_RESTORE_LIST iPackList);
		DRFL_API DRFL_RETURN _set_singularity_handling_ex2(LPROBOTCONTROL pCtrl, SINGULARITY_AVOIDANCE eMode);
		DRFL_API DRFL_RETURN _set_stiffnessx_ex2(LPROBOTCONTROL pCtrl, float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f);
		DRFL_API DRFL_RETURN _set_teach_mode_ex2(LPROBOTCONTROL pCtrl, unsigned char bMode);
		//DRFL_API DRFL_RETURN _set_unzip_ex2(LPROBOTCONTROL pCtrl, unsigned char iResult, const char szDirName[MAX_SYMBOL_SIZE]);
		DRFL_API DRFL_RETURN _simulatior_keyin_ex2(LPROBOTCONTROL pCtrl, unsigned char iKey);
		DRFL_API DRFL_RETURN _task_compliance_ctrl_ex2(LPROBOTCONTROL pCtrl, float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f);
		DRFL_API DRFL_RETURN _twait_ex2(LPROBOTCONTROL pCtrl, float fTime);
		DRFL_API DRFL_RETURN _update_excute_package_ex2(LPROBOTCONTROL pCtrl, unsigned char iExecute);
		DRFL_API DRFL_RETURN _update_fts_data_ex2(LPROBOTCONTROL pCtrl, float fOffset[NUMBER_OF_JOINT]);
		DRFL_API DRFL_RETURN _update_gravity_param_ex2(LPROBOTCONTROL pCtrl, COUNTER_BALANCE_PARAM_DATA tClParam, CALIBRATION_PARAM_DATA tCrParam);
		DRFL_API DRFL_RETURN _update_jts_data_ex2(LPROBOTCONTROL pCtrl, float fOffset[NUMBER_OF_JOINT], float fScale[NUMBER_OF_JOINT]);
		DRFL_API DRFL_RETURN _update_local_package_ex2(LPROBOTCONTROL pCtrl, unsigned char iTarget[UPDATE_TARGET_LAST], const char szDirName[MAX_SYMBOL_SIZE]);
		DRFL_API DRFL_RETURN _update_network_package_ex2(LPROBOTCONTROL pCtrl, unsigned char iTarget[UPDATE_TARGET_LAST], unsigned char iNetType, const char szIpAddress[16], const char szFileName[MAX_SYMBOL_SIZE]);
		DRFL_API DRFL_RETURN _update_process_ex2(LPROBOTCONTROL pCtrl, unsigned char pProcessInfo, unsigned char pStatusInfo, LPSYSTEM_UPDATE_RESPONSE iUpdateProcs);
		//API Low Priority End

		// API for user serial
		DRFL_API DRFL_RETURN  _open_user_serial(LPROBOTCONTROL pCtrl, SERIAL_CONFIG serialConfig, bool* serialOpenStatus, EN_USER_MODULE_TYPE moduleType = EN_USER_SERIAL);
		DRFL_API DRFL_RETURN _close_user_serial(LPROBOTCONTROL pCtrl, char* uniqueID, bool* serialStatus);
		DRFL_API DRFL_RETURN _write_data_user_serial(LPROBOTCONTROL pCtrl, const char* uniqueID, const char* serialData, bool* serialWriteStatus);
		DRFL_API DRFL_RETURN _query_user_serial(LPROBOTCONTROL pCtrl, const char* uniqueID, LPUSER_SERIAL_INFO serialInfo);
		DRFL_API DRFL_RETURN _query_all_user_serial(LPROBOTCONTROL pCtrl, LPUSER_SERIAL_INFO serialInfo);
		DRFL_API void _set_on_serial_data_updated(LPROBOTCONTROL pCtrl, TOnSerialDataUpdatedCB pCallbackFunc);
		DRFL_API void _set_on_serial_connection_status_updated(LPROBOTCONTROL pCtrl, TOnSerialConnectionStatusUpdatedCB pCallbackFunc);

		// API for module manager
		DRFL_API DRFL_RETURN _send_module_intallation_ex2(LPROBOTCONTROL pCtrl, char *pModulePackageName, LPMODULE_MANAGER_STATUS retStatus, int nInstallType);
		DRFL_API DRFL_RETURN _send_load_module_ex2(LPROBOTCONTROL pCtrl, char *pModuleName, char* uniqueID, LPMODULE_MANAGER_STATUS retStatus);
		DRFL_API DRFL_RETURN _send_unload_module_ex2(LPROBOTCONTROL pCtrl, char *pModuleName, char* uniqueID, LPMODULE_MANAGER_STATUS retStatus);
		DRFL_API DRFL_RETURN _send_delete_module_ex2(LPROBOTCONTROL pCtrl, char *pModuleName, LPMODULE_MANAGER_STATUS retStatus);

		// Handling Exception Function
		bool is_valid_ip_v4(string ipAddress);
		bool is_valid_coord_system(unsigned char iCoordId);
		bool is_valid_move_reference(unsigned char iMoveReference);
		DRFL_RETURN check_valid_general_range(GENERAL_RANGE tNormal, GENERAL_RANGE tReduced);
	}
#endif
#ifdef __cplusplus
    class CDRFLEx2 : public CDRFLEx
    {
    public:
        CDRFLEx2() {}
        virtual ~CDRFLEx2() {}
	public:
		DRFL_RETURN open_connection(string strIpAddr = "192.168.137.100", unsigned int usPort= 12345) { return _open_connection_ex2(_rbtCtrl, strIpAddr.c_str(), usPort); };
		DRFL_RETURN query_operating_system(ROBOT_SYSTEM* robotSystem) { return _query_operating_system_ex2(_rbtCtrl, robotSystem); };
		DRFL_RETURN query_operating_state(ROBOT_STATE* robotState) { return _query_operating_state_ex2(_rbtCtrl, robotState); };
		DRFL_RETURN release_protective_stop(RELEASE_MODE eReleaseMode){ return _release_protective_stop_ex2(_rbtCtrl, eReleaseMode); };
		DRFL_RETURN get_workpiece_weight(float *fParam){return _get_workpiece_weight_ex2(_rbtCtrl, fParam);};
        DRFL_RETURN measure_tcp(unsigned char iTargetRef, float fTargetPos[4][NUMBER_OF_JOINT], LPMEASURE_TCP_RESPONSE tcpResp) { return _measure_tcp_ex2(_rbtCtrl, iTargetRef, fTargetPos, tcpResp); };
		DRFL_RETURN measure_install_pose(float fGradient, float fRotation, bool* ret) { return _measure_install_pose_ex2(_rbtCtrl, fGradient, fRotation, ret); };
		DRFL_RETURN change_collaborative_speed(float fSpeed) { return _change_collaborative_speed_ex2(_rbtCtrl, fSpeed); };
		DRFL_RETURN wait_manual_guide_response() { return _wait_manual_guide_response_ex2(_rbtCtrl); };
		DRFL_RETURN change_reduced_speed(float fSpeed) { return _change_reduced_speed_ex2(_rbtCtrl, fSpeed); };
		DRFL_RETURN del_user_coord(unsigned char iIdentifier) { return _del_user_coord_ex2(_rbtCtrl, iIdentifier); };
		DRFL_RETURN get_user_coord(unsigned char iIdentifier) { return _get_user_coord_ex2(_rbtCtrl, iIdentifier); };
		DRFL_RETURN get_pattern_point(CONTROL_TRANS_PALLET_POS pControltranspalletpos, LPRESPONSE_TRANS_PALLET_POS pRestranspalletpos) { return _get_pattern_point_ex2(_rbtCtrl, pControltranspalletpos, pRestranspalletpos); };
		DRFL_RETURN drl_nudge() { return _drl_nudge_ex2(_rbtCtrl); };
		DRFL_RETURN get_user_coord_external_force(unsigned char bIsMonitoring, unsigned char iUserID[MAX_USER_COORD_MONITORING_EXT_FORCE_SIZE]) { return _get_user_coord_external_force_ex2(_rbtCtrl, bIsMonitoring, iUserID); };
		DRFL_RETURN set_ie_monitoring(unsigned char bStart) { return _set_ie_monitoring_ex2(_rbtCtrl, bStart); };
		DRFL_RETURN get_friction_value(unsigned char iType,unsigned char iSelect[NUMBER_OF_JOINT], float fStart[NUMBER_OF_JOINT], float fRange[NUMBER_OF_JOINT], LPMEASURE_FRICTION_RESPONSE pFricRes){ return _get_friction_value_ex2(_rbtCtrl ,iType, iSelect,  fStart, fRange, pFricRes);  };
		DRFL_RETURN get_serial_port(LPSERIAL_SEARCH pSerOut) { return _get_serial_port_ex2(_rbtCtrl, pSerOut); };
		DRFL_RETURN get_sub_system(LPINSTALL_SUB_SYSTEM p_tInstallSubSystem) { return _get_sub_system_ex2(_rbtCtrl, p_tInstallSubSystem); };
		DRFL_RETURN operation_industrial_ethernet(unsigned char nEtherNetIPOpMode, CONFIG_INDUSTRIAL_ETHERNET tConfig) { return _operation_industrial_ethernet_ex2(_rbtCtrl, nEtherNetIPOpMode, tConfig); };
		DRFL_RETURN get_sysversion_ex(LPSYSTEM_VERSION_EX pSysVer) { return _get_sysversion_ex2(_rbtCtrl, pSysVer); };
		DRFL_RETURN update_local_svm(const char szFileName[MAX_SYMBOL_SIZE]) { return _update_local_svm_ex2(_rbtCtrl, szFileName); };
		DRFL_RETURN update_network_svm(unsigned char iNetType, const char szIpAddress[16], const char szFileName[MAX_SYMBOL_SIZE]) { return _update_network_svm_ex2(_rbtCtrl, iNetType, szIpAddress, szFileName); };
		DRFL_RETURN get_tcp_coord(unsigned char iTargetRef, float fTargetPos1[NUMBER_OF_JOINT], float fTargetPos2[NUMBER_OF_JOINT], float fTargetPos3[NUMBER_OF_JOINT], float fTargetPos4[NUMBER_OF_JOINT], LPMEASURE_TCP_RESPONSE pMeaTcpRes) { return _get_tcp_coord_ex2(_rbtCtrl, iTargetRef, fTargetPos1, fTargetPos2, fTargetPos3, fTargetPos4, pMeaTcpRes); };
		DRFL_RETURN set_digital_outputs(unsigned char iLocation, unsigned short iCount, GPIO_PORT tPort[MAX_DIGITAL_BURST_SIZE]) { return _set_digital_outputs_ex2(_rbtCtrl, iLocation, iCount, tPort); };
		DRFL_RETURN set_workpiece_weight(float fWeight = 0.0, float fCog[3] = COG_DEFAULT, COG_REFERENCE eCogRef = COG_REFERENCE_TCP, ADD_UP eAddUp = ADD_UP_REPLACE, float fStartTime = -10000, float fTransitionTIme = -10000){return _set_workpiece_weight_ex2(_rbtCtrl, fWeight, fCog, eCogRef, eAddUp, fStartTime, fTransitionTIme); };
		DRFL_RETURN safe_movejx(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _safe_movejx_ex2(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
        DRFL_RETURN get_system_time(LPSYSTEM_TIME pTime) { return _get_system_time_ex2(_rbtCtrl, pTime); };
        DRFL_RETURN add_modbus_signal(string strSymbol, string strIpAddress, unsigned short nPort, MODBUS_REGISTER_TYPE eRegType, unsigned short iRegIndex, unsigned short nRegValue = 0, unsigned char nSlaiveId = 255) { return _add_modbus_signal_ex2(_rbtCtrl, strSymbol.c_str(), strIpAddress.c_str(), nPort, eRegType, iRegIndex, nRegValue, nSlaiveId); };
		DRFL_RETURN add_tcp(string strSymbol, float fPostion[NUM_TASK]) { return _add_tcp_ex2(_rbtCtrl, strSymbol.c_str(), fPostion); };
        DRFL_RETURN add_tool(string strSymbol, float fWeight, float fCog[3], float fInertia[NUM_TASK]) { return _add_tool_ex2(_rbtCtrl, strSymbol.c_str(), fWeight, fCog, fInertia); };
		DRFL_RETURN calc_coord(unsigned short nCnt, unsigned short nInputMode, COORDINATE_SYSTEM eTargetRef, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fTargetPos4[NUM_TASK], LPROBOT_POSE pRoboPose){return _calc_coord_ex2(_rbtCtrl, nCnt, nInputMode, eTargetRef, fTargetPos1, fTargetPos2, fTargetPos3, fTargetPos4, pRoboPose);};
		DRFL_RETURN check_force_condition(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference ,bool* pCheckForceCondition) { return _check_force_condition_ex2(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, eForceReference, pCheckForceCondition); };
		DRFL_RETURN config_program_watch_variable(VARIABLE_TYPE eDivision, DATA_TYPE eType, string strName, string strData) { return _config_program_watch_variable_ex2(_rbtCtrl, eDivision, eType, strName.c_str(), strData.c_str()); };
		DRFL_RETURN move_home(MOVE_HOME eMode = MOVE_HOME_MECHANIC, unsigned char bRun = (unsigned)1) { return _move_home_ex2(_rbtCtrl, eMode, bRun); };
		DRFL_RETURN moveb(MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE) { return _moveb_ex2(_rbtCtrl, tTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference); };
		DRFL_RETURN movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movec_ex2(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fTargetAngle1, fTargetAngle2, fBlendingRadius, eBlendingType); };
		DRFL_RETURN movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movej_ex2(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
		DRFL_RETURN movejx(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movejx_ex2(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
		DRFL_RETURN movel(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movel_ex2(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); }
		DRFL_RETURN move_pause() { return _move_pause_ex2(_rbtCtrl); };
		DRFL_RETURN move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned int nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _move_periodic_ex2(_rbtCtrl, fAmplitude, fPeriodic, fAccelTime, nRepeat, eMoveReference); };
		DRFL_RETURN movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _movesj_ex2(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
		DRFL_RETURN move_spiral(TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _move_spiral_ex2(_rbtCtrl, eTaskAxis, fRevolution, fMaximuRadius, fMaximumLength, fTargetVel, fTargetAcc, fTargetTime, eMoveReference); };
		DRFL_RETURN movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT) { return _movesx_ex2(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eVelOpt); };
		DRFL_RETURN mwait() { return _mwait_ex2(_rbtCtrl); };
		DRFL_RETURN control_brake(unsigned char iTargetAxis, unsigned char bValue) { return _control_brake_ex2(_rbtCtrl, iTargetAxis, bValue); };
		DRFL_RETURN coord_transform(float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eInCoordSystem, COORDINATE_SYSTEM eOutCoordSystem, LPROBOT_POSE pRoboPose) { return _coord_transform_ex2(_rbtCtrl, fTargetPos, eInCoordSystem, eOutCoordSystem, pRoboPose); };
		DRFL_RETURN drl_pause() { return _drl_pause_ex2(_rbtCtrl); };
		DRFL_RETURN drl_resume() { return _drl_resume_ex2(_rbtCtrl); };
		DRFL_RETURN drl_start(ROBOT_SYSTEM eRobotSystem, string strDrlProgram) { return _drl_start_ex2(_rbtCtrl, eRobotSystem, strDrlProgram.c_str()); };
		DRFL_RETURN flange_serial_close(){ return _flange_serial_close_ex2(_rbtCtrl); };
		DRFL_RETURN flange_serial_open(int baudrate = 115200, BYTE_SIZE eByteSize = BYTE_SIZE_EIGHTBITS, PARITY_CHECK eParity = PARITY_CHECK_NONE, STOP_BITS eStopBits = STOPBITS_ONE){ return _flange_serial_open_ex2(_rbtCtrl, baudrate, eByteSize, eParity, eStopBits); };
		DRFL_RETURN flange_serial_read(LPFLANGE_SER_RXD_INFO pRx, float fTimeout = -1, int nPort = 1) { return _flange_serial_read_ex2(_rbtCtrl, pRx, fTimeout, nPort); };   
		DRFL_RETURN flange_serial_write(int nSize, char* pSendData, int nPort = 1){ return _flange_serial_write_ex2(_rbtCtrl, nSize, pSendData, nPort); };
		DRFL_RETURN focas_cnc_read_param(unsigned short hHandle, short iParamNumber, short iAxisNumber, short iDataLength, LPMACHINE_TENDING_RESPONSE_FOCAS_CNC_PARAM pCnc) { return _focas_cnc_read_param_ex2(_rbtCtrl, hHandle, iParamNumber, iAxisNumber, iDataLength, pCnc); };
		DRFL_RETURN focas_connect(short ErrorCode, const char szIpAddr[16], unsigned short iPort, unsigned short hHandle, float fTimeOut, LPMACHINE_TENDING_FOCAS_CONNECT pCnc) { return _focas_connect_ex2(_rbtCtrl, ErrorCode, szIpAddr, iPort, hHandle, fTimeOut, pCnc); };
		DRFL_RETURN focas_disconnect(short ErrorCode, unsigned short hHandle, LPMACHINE_TENDING_FOCAS_DISCONNECT pCnc) { return _focas_disconnect_ex2(_rbtCtrl, ErrorCode, hHandle, pCnc); };
		DRFL_RETURN focas_get_error_str(unsigned short hHandle, short ErrorCode, const char szErrorString[256], LPMACHINE_TENDING_FOCAS_ERR_STRING pCnc) { return _focas_get_error_str_ex2(_rbtCtrl, hHandle, ErrorCode, szErrorString, pCnc); };
		DRFL_RETURN focas_pmc_read_bit(short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset, MACHINE_TENDING_FOCAS_PMC_DATA tData, LPMACHINE_TENDING_FOCAS_PMC pCnc) { return _focas_pmc_read_bit_ex2(_rbtCtrl, ErrorCode, hHandle, iDataType, szAddressType, iStartAddressNum, iCount, iBitOffset, tData, pCnc); };
		DRFL_RETURN focas_pmc_write_bit(short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset, MACHINE_TENDING_FOCAS_PMC_DATA tData, LPMACHINE_TENDING_FOCAS_PMC pCnc) { return _focas_pmc_write_bit_ex2(_rbtCtrl, ErrorCode, hHandle, iDataType, szAddressType, iStartAddressNum, iCount, iBitOffset, tData, pCnc); };
		DRFL_RETURN focas_program_num(unsigned short nHandle, LPMACHINE_TENDING_RESPONSE_FOCAS_PROGRAM_NUMBER pCnc) { return _focas_program_num_ex2(_rbtCtrl, nHandle, pCnc); };
		DRFL_RETURN get_control_mode(CONTROL_MODE* pMode){ return _get_control_mode_ex2(_rbtCtrl, pMode);};
		DRFL_RETURN get_current_posj(LPROBOT_POSE pRoboPose) { return _get_current_posj_ex2(_rbtCtrl, pRoboPose); };
		DRFL_RETURN get_current_posx(LPROBOT_TASK_POSE pRoboTaskPose, COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE){return _get_current_posx_ex2(_rbtCtrl, pRoboTaskPose, eCoodType); };
		DRFL_RETURN get_curnet_tool_flange_posx(LPROBOT_POSE pRoboPose) { return _get_current_tool_flange_posx_ex2(_rbtCtrl, pRoboPose); };
		DRFL_RETURN get_current_velj(LPROBOT_VEL pRoboVel) { return _get_current_velj_ex2(_rbtCtrl, pRoboVel); };
		DRFL_RETURN get_current_velx(LPROBOT_VEL pRoboVel) { return _get_current_velx_ex2(_rbtCtrl, pRoboVel); };
		DRFL_RETURN change_operation_speed(float fSpeed) { return _change_operation_speed_ex2(_rbtCtrl, fSpeed); };
		DRFL_RETURN get_current_rotm(float (*fRotationMatrix)[3][3], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE) { return _get_current_rotm_ex2(_rbtCtrl, fRotationMatrix, eTargetRef); };
		// focas v2 start.
		DRFL_RETURN focas_cnc_read_param_v2(unsigned short hHandle, short iParamNumber, short iAxisNumber, short iDataLength, LPMACHINE_TENDING_RESPONSE_FOCAS_CNC_PARAM pCnc) { return _focas_cnc_read_param_ex2_v2(_rbtCtrl, hHandle, iParamNumber, iAxisNumber, iDataLength, pCnc); };
		DRFL_RETURN focas_connect_v2(short ErrorCode, const char szIpAddr[16], unsigned short iPort, unsigned short hHandle, float fTimeOut, LPMACHINE_TENDING_FOCAS_CONNECT pCnc) { return _focas_connect_ex2_v2(_rbtCtrl, ErrorCode, szIpAddr, iPort, hHandle, fTimeOut, pCnc); };
		DRFL_RETURN focas_disconnect_v2(short ErrorCode, unsigned short hHandle, LPMACHINE_TENDING_FOCAS_DISCONNECT pCnc) { return _focas_disconnect_ex2_v2(_rbtCtrl, ErrorCode, hHandle, pCnc); };
		DRFL_RETURN focas_get_error_str_v2(unsigned short hHandle, short ErrorCode, const char szErrorString[256], LPMACHINE_TENDING_FOCAS_ERR_STRING pCnc) { return _focas_get_error_str_ex2_v2(_rbtCtrl, hHandle, ErrorCode, szErrorString, pCnc); };
		DRFL_RETURN focas_pmc_read_bit_v2(short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset, MACHINE_TENDING_FOCAS_PMC_DATA tData, LPMACHINE_TENDING_FOCAS_PMC pCnc) { return _focas_pmc_read_bit_ex2_v2(_rbtCtrl, ErrorCode, hHandle, iDataType, szAddressType, iStartAddressNum, iCount, iBitOffset, tData, pCnc); };
		DRFL_RETURN focas_pmc_write_bit_v2(short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset, MACHINE_TENDING_FOCAS_PMC_DATA tData, LPMACHINE_TENDING_FOCAS_PMC pCnc) { return _focas_pmc_write_bit_ex2_v2(_rbtCtrl, ErrorCode, hHandle, iDataType, szAddressType, iStartAddressNum, iCount, iBitOffset, tData, pCnc); };
		DRFL_RETURN focas_program_num_v2(unsigned short nHandle, LPMACHINE_TENDING_RESPONSE_FOCAS_PROGRAM_NUMBER pCnc) { return _focas_program_num_ex2(_rbtCtrl, nHandle, pCnc); };
		
		// focas v2 end.


		/*open new connection
		 strPackageName: User Tcp Module Name
		 enType: SERVER/CLIENT
		 strIPAddress: 192.168.137.100
		 nPort: 1111
		 strUniqueID: TcpForGripper (must unique code)*/
		DRFL_RETURN open_user_tcp(const char* strUniqueID, EN_USER_MODULE_TYPE enType, const char* strIPAddress, int nPort)  { return _open_user_tcp(_rbtCtrl, strUniqueID, enType, strIPAddress, nPort); };

		// close exist connection by strUniqueID
		// strUniqueID: TcpForGripper (must unique code)
		DRFL_RETURN close_user_tcp(const char* strUniqueID)  { return _close_user_tcp(_rbtCtrl, strUniqueID); };

		// query exist connection contributes by connection code
		// tTcpInfo: pointer will point to struct USER_TCP
		DRFL_RETURN query_user_tcp_info(const char* strUniqueID, LPUSER_TCP tTcpInfo) { return _query_user_tcp_info_(_rbtCtrl, strUniqueID, tTcpInfo); };

		// query exist client contributes by ID
		// nClientID: client's ID
		// tClientInfo: pointer will point to struct USER_TCP_CLIENT
		DRFL_RETURN query_user_tcp_client_info(const char* strUniqueID, const int nClientID, LPUSER_TCP_CLIENT tClientInfo) { return _query_user_tcp_client_info_(_rbtCtrl, strUniqueID, nClientID, tClientInfo); };

		// write data from user DRFL to external device: TcpClient to TcpServer
		// pData: pointer is point to data will sent to external device
		DRFL_RETURN write_user_tcp(const char* strUniqueID, const char* pData) { return _write_user_tcp(_rbtCtrl, strUniqueID, pData); };

		// write data from user DRFL to external device: TcpServer to TcpClient
		// data: pointer is point to data will sent to external device
		// nClientID: client's ID
		DRFL_RETURN write_user_tcp(const char* strUniqueID, const char* pData, const int nClientID) { return _write_user_tcp_client(_rbtCtrl, strUniqueID, pData, nClientID);};

		// when there is new data from an external device. A callback function will be called
		// callback func for new data has been update to the sharedmem from external device
		void set_on_tcp_server_message_received(TOnTcpServerMessageReceivedCB pCallbackFunc) { _set_on_tcp_server_message_received(_rbtCtrl, pCallbackFunc); };

		void set_on_tcp_client_message_received(TOnTcpClientMessageReceivedCB pCallbackFunc) { _set_on_tcp_client_message_received(_rbtCtrl, pCallbackFunc); };

		void set_on_tcp_server_notify_new_connect(TOnTcpServerNotifyNewConnectCB pCallbackFunc) { _set_on_tcp_server_notify_new_connect(_rbtCtrl, pCallbackFunc); };

		void set_on_tcp_server_notify_disconnect(TOnTcpServerNotifyDisconnectCB pCallbackFunc) { _set_on_tcp_server_notify_disconnect(_rbtCtrl, pCallbackFunc); };

		void set_on_tcp_client_notify_new_connect(TOnTcpClientNotifyNewConnectCB pCallbackFunc) { _set_on_tcp_client_notify_new_connect(_rbtCtrl, pCallbackFunc); };

		void set_on_tcp_client_notify_disconnect(TOnTcpClientNotifyDisconnectCB pCallbackFunc) { _set_on_tcp_client_notify_disconnect(_rbtCtrl, pCallbackFunc); };

		void set_on_tcp_client_notify_reconnect(TOnTcpClientNotifyReconnectCB pCallbackFunc) { _set_on_tcp_client_notify_reconnect(_rbtCtrl, pCallbackFunc); };

		void set_on_open_connect_result(TOnOpenConnectResultCB pCallbackFunc) { _set_on_open_connect_result(_rbtCtrl, pCallbackFunc); };

		void set_on_close_connect_result(TOnCloseConnectResultCB pCallbackFunc) { _set_on_close_connect_result(_rbtCtrl, pCallbackFunc); };

		void set_on_tcp_client_disconnect(TOnTcpClientDisconnectCB pCallbackFunc) { _set_on_tcp_client_disconnect(_rbtCtrl, pCallbackFunc); };

		// API for user serial protocol
		/* open a serial connection based on the serial configure
		 * serialConfig: Configure use for creating new seial connection
		 * serialOpenStatus: response status open from DRCF
		 * moduleType: TCP SERVER/TCP CLIENT/SERIAL
		 */
		DRFL_RETURN open_user_serial(SERIAL_CONFIG serialConfig, bool* serialOpenStatus, EN_USER_MODULE_TYPE moduleType = EN_USER_SERIAL) { return _open_user_serial(_rbtCtrl, serialConfig, serialOpenStatus, moduleType); };
		/* API close serial need to change argument to serial port when fix feature unload in module manager */
		/* close a serial connection
		 * uniqueID: id exist unique in system
		 */
		DRFL_RETURN close_user_serial(char* uniqueID, bool* serialStatus) { return _close_user_serial(_rbtCtrl, uniqueID, serialStatus); };
		/* write data from DRFL to external device
		 * uniqueID: id exist unique in system
		 * serialData: data will be sent to DRCF
		 * serialWriteStatus: status write receive from DRCF
		 */
		DRFL_RETURN write_data_user_serial(const char* uniqueID, const char* serialData, bool* serialWriteStatus) {return _write_data_user_serial(_rbtCtrl, uniqueID, serialData, serialWriteStatus);};
		/* query serial data from DRFL to DRCF
		 * uniqueID: id exist unique in system
		 * serialInfo: data will be received from DRCF
		 */
		DRFL_RETURN query_user_serial(const char* uniqueID, LPUSER_SERIAL_INFO serialInfo) {return _query_user_serial(_rbtCtrl, uniqueID, serialInfo);};
		/* query serial data from DRFL to DRCF
		 * uniqueID: id exist unique in system
		 * serialInfo: data will be received from DRCF
		 */
		DRFL_RETURN query_all_user_serial(LPUSER_SERIAL_INFO serialInfo) {return _query_all_user_serial(_rbtCtrl, serialInfo);};
		/* callback function for receiving data from external device
		 * pCallbackFunc: callback function
		 */
		void set_on_serial_data_updated(TOnSerialDataUpdatedCB pCallbackFunc) { _set_on_serial_data_updated(_rbtCtrl, pCallbackFunc); };
		/* callback function for receiving connection status from drcf
		 * pCallbackFunc: callback function
		 */
		void set_on_serial_connection_status_updated(TOnSerialConnectionStatusUpdatedCB pCallbackFunc) { _set_on_serial_connection_status_updated(_rbtCtrl, pCallbackFunc); };

		// API for module manager
		/* send install command to module manager
		 * pModulePackageName: name of package
		 * nInstallType: install type
		 * retStatus: status install from DRCF
		 */ 
		DRFL_RETURN send_module_installation(char *pModulePackageName, LPMODULE_MANAGER_STATUS retStatus, int nInstallType = 0) { return _send_module_intallation_ex2(_rbtCtrl, pModulePackageName, retStatus, nInstallType); };
		/* send load command to module manager
		 * pModuleName: module name
		 * uniqueID: id exist unique in system
		 * retStatus: status load from DRCF
		 */ 
		DRFL_RETURN send_load_module(char *pModuleName, char* uniqueID, LPMODULE_MANAGER_STATUS retStatus) { return _send_load_module_ex2(_rbtCtrl, pModuleName, uniqueID, retStatus); };
		/* send unload command to module manager
		 * pModuleName: module name
		 * uniqueID: id exist unique in system
		 * retStatus: status unload from DRCF
		 */ 
		DRFL_RETURN send_unload_module(char *pModuleName, char* uniqueID, LPMODULE_MANAGER_STATUS retStatus) { return _send_unload_module_ex2(_rbtCtrl, pModuleName, uniqueID, retStatus); };
		/* send delete command to module manager
		 * pModuleName: module name
		 * retStatus: status delete from DRCF
		 */ 
		DRFL_RETURN send_delete_module(char *pModuleName, LPMODULE_MANAGER_STATUS retStatus) { return _send_delete_module_ex2(_rbtCtrl, pModuleName, retStatus); };
		////
        DRFL_RETURN app_weld_disable_digital(unsigned char bMode) { return _app_weld_disable_digital_ex2(_rbtCtrl, bMode); };
        DRFL_RETURN constrained_axis(unsigned char iTargetAxis[NUMBER_OF_JOINT], unsigned char iTargetRef, float fTargetStf, float fTargetDump) { return _constrained_axis_ex2(_rbtCtrl, iTargetAxis, iTargetRef, fTargetStf, fTargetDump); };
        DRFL_RETURN drl_save_main(unsigned char bMode, unsigned int uLength, string lpszString) { return _drl_save_main_ex2(_rbtCtrl, bMode, uLength, lpszString.c_str()); };
        DRFL_RETURN drl_stop(unsigned char eStopType = 0) { return _drl_stop_ex2(_rbtCtrl, eStopType); };
        DRFL_RETURN drl_start_main(ROBOT_SYSTEM eRobotSystem) { return _drl_start_main_ex2(_rbtCtrl, eRobotSystem); };
        DRFL_RETURN set_modbus_outputs(MODBUS_REGISTER_BURST pModbusregisterburst) { return _set_modbus_outputs_ex2(_rbtCtrl, pModbusregisterburst); };
        DRFL_RETURN add_modbus_rtu_signal(WRITE_MODBUS_RTU_DATA pWritemodbusrtudata) { return _add_modbus_rtu_signal_ex2(_rbtCtrl, pWritemodbusrtudata); };
        DRFL_RETURN add_modbus_multi_signal(const char szSymbol[MAX_SYMBOL_SIZE], const char szIpAddr[16], unsigned short iPort, int iSlaveID, unsigned char iRegType, unsigned short iRegIndex, unsigned char iRegCount) { return _add_modbus_multi_signal_ex2(_rbtCtrl, szSymbol, szIpAddr, iPort, iSlaveID, iRegType, iRegIndex, iRegCount); };
        DRFL_RETURN del_modbus_multi_signal(string szSymbol) { return _del_modbus_multi_signal_ex2(_rbtCtrl, szSymbol.c_str()); };
        DRFL_RETURN add_modbus_rtu_multi_signal(WRITE_MODBUS_RTU_MULTI_DATA pWritemodbusrtumultidata) { return _add_modbus_rtu_multi_signal_ex2(_rbtCtrl, pWritemodbusrtumultidata); };
        DRFL_RETURN set_output_register_bit(unsigned char iGprType, unsigned char iGprAddr, const char szData[128]) { return _set_output_register_bit_ex2(_rbtCtrl, iGprType, iGprAddr, szData); };
        DRFL_RETURN set_install_pose(float fGradient, float fRotation) { return _set_install_pose_ex2(_rbtCtrl, fGradient, fRotation); };
        DRFL_RETURN change_collision_sensitivity(float fSensitivity){ return _change_collision_sensitivity_ex2(_rbtCtrl, fSensitivity); };
        DRFL_RETURN set_safety_function(unsigned char iStopCode[SAFETY_FUNC_LAST]) { return _set_safety_function_ex2(_rbtCtrl, iStopCode); };
        DRFL_RETURN set_safety_mode(SAFETY_MODE eSafetyMode, SAFETY_MODE_EVENT eSafetyEvent){ return _set_safety_mode_ex2(_rbtCtrl, eSafetyMode, eSafetyEvent); };
        DRFL_RETURN set_cockpit(unsigned char bEnable, unsigned char iButton[2], unsigned char bRecoveryTeach) { return _set_cockpit_ex2(_rbtCtrl, bEnable, iButton, bRecoveryTeach); };
        DRFL_RETURN get_user_home(LPROBOT_POSE pPosition) { return _get_user_home_ex2(_rbtCtrl, pPosition); };
		DRFL_RETURN set_modbus_multi_output(string szSymbol, unsigned char iRegCount, unsigned short iRegValue[MAX_MODBUS_REGISTER_PER_DEVICE]){ return _set_modbus_multi_output_ex2(_rbtCtrl, szSymbol.c_str(), iRegCount, iRegValue); };
        DRFL_RETURN get_input_register_bit(unsigned char iGprType, unsigned char iGprAddr, unsigned char iInOut, LPIETHERNET_SLAVE_RESPONSE_DATA_EX pEthernetSlaveData) { return _get_input_register_bit_ex2(_rbtCtrl, iGprType, iGprAddr, iInOut, pEthernetSlaveData); };
        DRFL_RETURN set_general_range(GENERAL_RANGE tNormal, GENERAL_RANGE tReduced) { return _set_general_range_ex2(_rbtCtrl, tNormal, tReduced); };
        DRFL_RETURN set_joint_range(JOINT_RANGE tNormal, JOINT_RANGE tReduced) { return _set_joint_range_ex2(_rbtCtrl, tNormal, tReduced); };
        DRFL_RETURN set_safety_io(unsigned char iIO[TYPE_LAST][NUM_SAFETY]) { return _set_safety_io_ex2(_rbtCtrl, iIO); };
        DRFL_RETURN add_tool_shape(CONFIG_TOOL_SHAPE_SYMBOL pConfigtoolshapesymbol) { return _add_tool_shape_ex2(_rbtCtrl, pConfigtoolshapesymbol); };
        DRFL_RETURN add_safety_zone(string szIdentifier, string szAlias, unsigned char iZoneType, SAFETY_ZONE_PROPERTY_DATA tZoneProperty, SAFETY_ZONE_SHAPE tShape) { return _add_safety_zone_ex2(_rbtCtrl, szIdentifier.c_str(), szAlias.c_str(), iZoneType, tZoneProperty, tShape); };
        DRFL_RETURN config_setting_enable(unsigned short wPreviousCmdid, unsigned int iRefCrc32) { return _config_setting_enable_ex2(_rbtCtrl,wPreviousCmdid,iRefCrc32); };
        DRFL_RETURN config_configurable_io(unsigned char niIO[TYPE_LAST][NUM_DIGITAL]) { return _config_configurable_io_ex2(_rbtCtrl, niIO); };
        DRFL_RETURN safe_move_home(unsigned char bRun = (unsigned)1) { return _safe_move_home_ex2(_rbtCtrl, bRun); };
        DRFL_RETURN safe_movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _safe_movej_ex2(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
        DRFL_RETURN safe_drl_start(ROBOT_SYSTEM eRobotSystem) { return _safe_drl_start_ex2(_rbtCtrl,eRobotSystem); }
        DRFL_RETURN safe_move_userhome(unsigned char bRun = (unsigned)1) { return _safe_move_userhome_ex2(_rbtCtrl, bRun); };
        DRFL_RETURN set_system_power(unsigned char iTarget, unsigned char iPower) { return _set_system_power_ex2(_rbtCtrl, iTarget, iPower); };
		DRFL_RETURN get_desired_posj(LPROBOT_POSE robotPose) { return _get_desired_posj_ex2(_rbtCtrl, robotPose); };
        DRFL_RETURN get_desired_posx(LPROBOT_POSE robotPose, COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE){return _get_desired_posx_ex2(_rbtCtrl, robotPose, eCoodType);};
        DRFL_RETURN get_desired_velx(LPROBOT_VEL robotVel) { return _get_desired_velx_ex2(_rbtCtrl, robotVel); }; 
        DRFL_RETURN get_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool* retVal) { return _get_digital_output_ex2(_rbtCtrl, eGpioIndex, retVal); };
        DRFL_RETURN get_external_torque(LPROBOT_FORCE robotForce) { return _get_external_torque_ex2(_rbtCtrl, robotForce); };
        DRFL_RETURN get_joint_torque(LPROBOT_FORCE robotForce) { return _get_joint_torque_ex2(_rbtCtrl, robotForce); };
        DRFL_RETURN get_library_version(string* libVersion) { return _get_library_version_ex2(_rbtCtrl, libVersion); };
        DRFL_RETURN get_program_state(DRL_PROGRAM_STATE* programState) { return _get_program_state_ex2(_rbtCtrl, programState); };
        DRFL_RETURN get_robot_control_mode(unsigned char* controlMode) { return _get_robot_control_mode_ex2(_rbtCtrl, controlMode); };
        DRFL_RETURN get_robot_state(ROBOT_STATE* robotState) { return _get_robot_state_ex2(_rbtCtrl, robotState); };
        DRFL_RETURN get_robot_system(ROBOT_SYSTEM* robotSystem) { return _get_robot_system_ex2(_rbtCtrl, robotSystem); };
        DRFL_RETURN get_solution_space(float fTargetPos[NUM_JOINT], unsigned char* solutionSpace){ return _get_solution_space_ex2(_rbtCtrl, fTargetPos, solutionSpace);};
        DRFL_RETURN get_system_version(LPSYSTEM_VERSION pVersion, bool* retVal) { return _get_system_version_ex2(_rbtCtrl, pVersion, retVal); };
        DRFL_RETURN get_tcp(string* retTcp) { return _get_tcp_ex2(_rbtCtrl, retTcp); };
        DRFL_RETURN get_tool(string* retTool) { return _get_tool_ex2(_rbtCtrl, retTool); };
        DRFL_RETURN get_tool_analog_input(int nCh, float* retTool){ return _get_tool_analog_input_ex2(_rbtCtrl, nCh, retTool); };
        DRFL_RETURN get_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool* retVal) { return _get_tool_digital_output_ex2(_rbtCtrl, eGpioIndex, retVal); };
        DRFL_RETURN get_tool_force(LPROBOT_FORCE robotForce) { return _get_tool_force_ex2(_rbtCtrl, robotForce); };
        DRFL_RETURN get_tool_weight(unsigned char iMode, LPMEASURE_TOOL_RESPONSE retTool, float fWeight = -999.f, float fXYZ_X = -999.f, float fXYZ_Y = -999.f, float fXYZ_Z = -999.f ){ return _get_tool_weight_ex2(_rbtCtrl, iMode, retTool, fWeight, fXYZ_X, fXYZ_Y,fXYZ_Z ); };
        DRFL_RETURN is_focas_alive(unsigned short nHandle, LPFOCAS_IS_ALIVE_RESPONSE retFocasAlive) { return _is_focas_alive_ex2(_rbtCtrl, nHandle, retFocasAlive); };
        DRFL_RETURN manage_access_control(MANAGE_ACCESS_CONTROL eAccessControl = MANAGE_ACCESS_CONTROL_REQUEST) { return _manage_access_control_ex2(_rbtCtrl, eAccessControl); };
        DRFL_RETURN measure_friction(unsigned char iType, unsigned char iSelect[NUMBER_OF_JOINT], float fStart[NUMBER_OF_JOINT], float fRange[NUMBER_OF_JOINT], LPMEASURE_FRICTION_RESPONSE retFriction) { return _measure_friction_ex2(_rbtCtrl, iType, iSelect, fStart, fRange, retFriction); };
        DRFL_RETURN overwrite_user_cart_coord(bool bTargetUpdate, int iReqId, float fTargetPos[NUM_TASK], int* retOverWrite,COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE){return _overwrite_user_cart_coord_ex2(_rbtCtrl, bTargetUpdate, iReqId, fTargetPos, retOverWrite, eTargetRef); };
        DRFL_RETURN drl_check_syntax(unsigned int iTextLength, string iszTextString) { return _drl_check_syntax_ex2(_rbtCtrl, iTextLength, iszTextString.c_str()); };
        DRFL_RETURN reset_safety_config() { return _reset_safety_config_ex2(_rbtCtrl); };
        DRFL_RETURN reset_workpiece_weight(){return _reset_workpiece_weight_ex2(_rbtCtrl);};
        DRFL_RETURN get_last_alarm(LPLOG_ALARM retLogAlarm) { return _get_last_alarm_ex2(_rbtCtrl, retLogAlarm); };
        DRFL_RETURN enable_friction_compensation(unsigned char iSelect[NUMBER_OF_JOINT], float fPositive[4][NUMBER_OF_JOINT], float fNegative[4][NUMBER_OF_JOINT], float fTemperature[NUMBER_OF_JOINT]) { return _enable_friction_compensation_ex2(_rbtCtrl, iSelect, fPositive, fNegative, fTemperature); };
        DRFL_RETURN get_robot_mode(ROBOT_MODE* robotMode) { return _get_robot_mode_ex2(_rbtCtrl, robotMode); };
        DRFL_RETURN get_current_solution_space(unsigned char* retSolutionSpace) { return _get_current_solution_space_ex2(_rbtCtrl, retSolutionSpace); };
        DRFL_RETURN get_robot_speed_mode(SPEED_MODE* speedMode) { return _get_robot_speed_mode_ex2(_rbtCtrl, speedMode); };
        DRFL_RETURN save_sub_program(int iTargetType, string strFileName, string strDrlProgram){return _save_sub_program_ex2(_rbtCtrl, iTargetType, strFileName.c_str(), strDrlProgram.c_str());};
        DRFL_RETURN servoj(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime){ return _servoj_ex2(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };
        DRFL_RETURN servol(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime){ return _servol_ex2(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };
        DRFL_RETURN servo_off(STOP_TYPE eStopType, int* retServoOff) { return _servo_off_ex2(_rbtCtrl, eStopType, retServoOff); };
        DRFL_RETURN servo_on() { return _servo_on_ex2(_rbtCtrl); };
        DRFL_RETURN set_analog_output(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue) { return _set_analog_output_ex2(_rbtCtrl, eGpioIndex, fValue); };
        DRFL_RETURN set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff) { return _set_digital_output_ex2(_rbtCtrl, eGpioIndex, bOnOff); };
        DRFL_RETURN set_ip_address(unsigned char iUsage, unsigned char iIpType, string szHostIp, string szSubnet, string szGateway, string szPrimaryDNS, string szSecondaryDNS) { return _set_ip_address_ex2(_rbtCtrl, iUsage, iIpType, szHostIp.c_str(), szSubnet.c_str(), szGateway.c_str(), szPrimaryDNS.c_str(), szSecondaryDNS.c_str()); };
        DRFL_RETURN set_modbus_output(string strSymbol, unsigned short nValue) { return _set_modbus_output_ex2(_rbtCtrl, strSymbol.c_str(), nValue); };
        DRFL_RETURN set_mode_analog_input(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT) { return _set_mode_analog_input_ex2(_rbtCtrl, eGpioIndex, eAnalogType); };
        DRFL_RETURN set_mode_analog_output(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT) { return _set_mode_analog_output_ex2(_rbtCtrl, eGpioIndex, eAnalogType); };
        DRFL_RETURN set_mode_tool_analog_input(int nCh, GPIO_ANALOG_TYPE eAnalogType){ return _set_mode_tool_analog_input_ex2(_rbtCtrl, nCh, eAnalogType); };
        DRFL_RETURN set_robot_control(ROBOT_CONTROL eControl) { return _set_robot_control_ex2(_rbtCtrl, eControl); };
        DRFL_RETURN set_robot_system(ROBOT_SYSTEM eRobotSystem) { return _set_robot_system_ex2(_rbtCtrl, eRobotSystem); };
        DRFL_RETURN set_tcp(string strSymbol) { return _set_tcp_ex2(_rbtCtrl, strSymbol.c_str()); };
        DRFL_RETURN set_tool(string strSymbol) { return _set_tool_ex2(_rbtCtrl, strSymbol.c_str()); };
        DRFL_RETURN set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff) { return _set_tool_digital_output_ex2(_rbtCtrl, eGpioIndex, bOnOff); };
		//
		DRFL_RETURN jog(JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity){ return _jog_ex2( _rbtCtrl, eJogAxis, eMoveReference, fVelocity); };
		DRFL_RETURN del_modbus_signal(const char* lpszSymbol){ return _del_modbus_signal_ex2( _rbtCtrl, lpszSymbol); };
		DRFL_RETURN del_tcp( const char* lpszSymbol){ return _del_tcp_ex2( _rbtCtrl, lpszSymbol); };
		DRFL_RETURN del_tool( const char* lpszSymbol){ return _del_tool_ex2( _rbtCtrl, lpszSymbol); };
		DRFL_RETURN del_tool_shape( const char* pcArg){ return _del_tool_shape_ex2( _rbtCtrl, pcArg); };
		DRFL_RETURN get_analog_input( GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float *pGetAnalog){ return _get_analog_input_ex2( _rbtCtrl, eGpioIndex, pGetAnalog); };
		DRFL_RETURN get_digital_input( GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool* retVal){ return _get_digital_input_ex2(_rbtCtrl, eGpioIndex, retVal); };
		DRFL_RETURN get_modbus_input( const char* lpszSymbol, unsigned short *pModbusInput){ return _get_modbus_input_ex2( _rbtCtrl, lpszSymbol, pModbusInput); };
		DRFL_RETURN get_tool_digital_input( GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool* retVal){ return _get_tool_digital_input_ex2( _rbtCtrl, eGpioIndex, retVal); };
		DRFL_RETURN get_user_cart_coord( int iReqId, LPUSER_COORDINATE GetUserCartCoord){ return _get_user_cart_coord_ex2( _rbtCtrl, iReqId, GetUserCartCoord); };
		DRFL_RETURN drl_break2( int nLine, const char* szFile){ return _drl_break2_ex2( _rbtCtrl, nLine, szFile); };
		DRFL_RETURN get_current_pose( ROBOT_SPACE eSpaceType, LPROBOT_POSE GetCurrentPose){ return _get_current_pose_ex2( _rbtCtrl, eSpaceType, GetCurrentPose); };
		DRFL_RETURN parallel_axis( float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE){ return _parallel_axis1_ex2( _rbtCtrl, fTargetPos1,  fTargetPos2,  fTargetPos3, eTaskAxis, eSourceRef ); };
        DRFL_RETURN parallel_axis( float fTargetVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE){ return _parallel_axis2_ex2( _rbtCtrl,  fTargetVec, eTaskAxis, eSourceRef ); };
        DRFL_RETURN set_user_cart_coord( int iReqId, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eTargetRef, int *pSetUserCartCoord1) { return _set_user_cart_coord1_ex2( _rbtCtrl, iReqId, fTargetPos, eTargetRef, pSetUserCartCoord1 ); };
        DRFL_RETURN set_user_cart_coord( float fTargetPos[3][NUM_TASK], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef, int *pSetUserCartCoord2) { return _set_user_cart_coord2_ex2( _rbtCtrl, fTargetPos, fTargetOrg, fTargetRef, pSetUserCartCoord2); };
        DRFL_RETURN set_user_cart_coord( float fTargetVec[2][3], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef, int *pSetUserCartCoord3) { return _set_user_cart_coord3_ex2( _rbtCtrl, fTargetVec, fTargetOrg, fTargetRef, pSetUserCartCoord3); };
        DRFL_RETURN align_axis( float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE ){ return _align_axis1_ex2( _rbtCtrl, fTargetPos1, fTargetPos2, fTargetPos3, fSourceVec, eTaskAxis,  eSourceRef ); };
        DRFL_RETURN align_axis( float fTargetVec[3], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE){ return _align_axis2_ex2( _rbtCtrl, fTargetVec, fSourceVec, eTaskAxis, eSourceRef ); };
		DRFL_RETURN hold2run(){ return _hold2run_ex2( _rbtCtrl); };
		DRFL_RETURN stop( STOP_TYPE eStopType = STOP_TYPE_QUICK){ return _stop_ex2( _rbtCtrl, eStopType ); };
		//
		///
		DRFL_RETURN	set_nudge(unsigned char bEnable, float fInputForce, float fDelayTime) { return _set_nudge_ex2(_rbtCtrl, bEnable, fInputForce, fDelayTime); };
		DRFL_RETURN set_world_coord(unsigned char iType, float fPosition[NUMBER_OF_JOINT]) { return _set_world_coord_ex2(_rbtCtrl, iType, fPosition); };
		DRFL_RETURN	set_tool_shape(string szIdentifier) { return _set_tool_shape_ex2(_rbtCtrl, szIdentifier); };
		DRFL_RETURN	del_safety_zone(string szIdentifier) { return _del_safety_zone_ex2(_rbtCtrl, szIdentifier); };
		DRFL_RETURN	reset_sequence(unsigned char iIdentifier) { return _reset_sequence_ex2(_rbtCtrl, iIdentifier); };
		DRFL_RETURN	safe_movel(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _safe_movel_ex2(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
		DRFL_RETURN get_tool_shape(string *lpszString) { return _get_tool_shape_ex2(_rbtCtrl, lpszString); };
		DRFL_RETURN set_tool_digital_output_level(int nLv) {	 return _set_tool_digital_output_level_ex2(_rbtCtrl, nLv);	 };
		DRFL_RETURN set_tool_digital_output_type(int nPort, OUTPUT_TYPE eOutputType) { return _set_tool_digital_output_type_ex2(_rbtCtrl, nPort, eOutputType);	 };
		DRFL_RETURN set_user_home() {	 return _set_user_home_ex2(_rbtCtrl);	 };
		DRFL_RETURN speedj(float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime) {	 return _speedj_ex2(_rbtCtrl, fTargetVel, fTargetAcc, fTargetTime);	 };
		DRFL_RETURN speedl(float fTargetVel[NUM_TASK], float fTargetAcc[2], float fTargetTime) {	 return _speedl_ex2(_rbtCtrl, fTargetVel, fTargetAcc, fTargetTime);	 };
		DRFL_RETURN system_shut_down() {	 return _system_shut_down_ex2(_rbtCtrl);	 };
		DRFL_RETURN tp_get_user_input_response(string strUserInput) { return _tp_get_user_input_response_ex2(_rbtCtrl, strUserInput); };
		DRFL_RETURN tp_popup_response(POPUP_RESPONSE eRes) {	 return _tp_popup_response_ex2(_rbtCtrl, eRes); };
        DRFL_RETURN get_ip_address(unsigned char iIpType, LPSYSTEM_IPADDRESS ip) { return _get_ip_address_ex2(_rbtCtrl, iIpType, ip); };
		DRFL_RETURN trans(float fSourcePos[NUM_TASK], float fOffset[NUM_TASK], COORDINATE_SYSTEM eSourceRef, COORDINATE_SYSTEM eTargetRef, LPROBOT_POSE iRobot) {	return _trans_ex2(_rbtCtrl, fSourcePos, fOffset, eSourceRef, eTargetRef, iRobot); };
        DRFL_RETURN get_control_space(ROBOT_SPACE* iRobotSpace) { return _get_control_space_ex2(_rbtCtrl, iRobotSpace);	 };
		///

		//API Second Priority Start
		DRFL_RETURN app_weld_adj_motion_offset(float fOffsetY, float fOffsetZ) { return _app_weld_adj_motion_offset_ex2(_rbtCtrl, fOffsetY, fOffsetZ); };
		DRFL_RETURN app_weld_adj_welding_cond_analog(unsigned char bRealTime, unsigned char bResetFlag, float fTargetVol, float fFeedingVel, float fTargetVel, float fOffsetY, float fOffsetZ, float fWidthRate) { return _app_weld_adj_welding_cond_analog_ex2(_rbtCtrl, bRealTime, bResetFlag, fTargetVol, fFeedingVel, fTargetVel, fOffsetY, fOffsetZ, fWidthRate); };
		DRFL_RETURN app_weld_adj_welding_cond_digital(CONFIG_DIGITAL_WELDING_ADJUST pConfigdigitalweldingadjust){ return _app_weld_adj_welding_cond_digital_ex2(_rbtCtrl, pConfigdigitalweldingadjust); };
		DRFL_RETURN app_weld_enable_analog(CONFIG_ANALOG_WELDING_INTERFACE pConfiganalogweldinginterface){ return _app_weld_enable_analog_ex2(_rbtCtrl, pConfiganalogweldinginterface); };
		DRFL_RETURN app_weld_enable_digital(unsigned char bMode){ return _app_weld_enable_digital_ex2(_rbtCtrl, bMode); };		
		DRFL_RETURN app_weld_reset_interface(unsigned char bReset){ return _app_weld_reset_interface_ex2(_rbtCtrl, bReset); };
		DRFL_RETURN app_weld_set_interface_eip_m2r_monitoring(CONFIG_DIGITAL_WELDING_INTERFACE_MONITORING pConfigdigitalweldinginterfacemonitoring){ return _app_weld_set_interface_eip_m2r_monitoring_ex2(_rbtCtrl, pConfigdigitalweldinginterfacemonitoring); };
		DRFL_RETURN app_weld_set_interface_eip_m2r_other(CONFIG_DIGITAL_WELDING_INTERFACE_OTHER pConfigdigitalweldinginterfacemonitoring){ return _app_weld_set_interface_eip_m2r_other_ex2(_rbtCtrl, pConfigdigitalweldinginterfacemonitoring); };
		DRFL_RETURN app_weld_set_interface_eip_r2m_condition(CONFIG_DIGITAL_WELDING_INTERFACE_CONDITION pConfigdigitalweldinginterfacecondition){ return _app_weld_set_interface_eip_r2m_condition_ex2(_rbtCtrl, pConfigdigitalweldinginterfacecondition); };
		DRFL_RETURN app_weld_set_interface_eip_r2m_mode(CONFIG_DIGITAL_WELDING_INTERFACE_MODE pConfigdigitalweldinginterfacemode){ return _app_weld_set_interface_eip_r2m_mode_ex2(_rbtCtrl, pConfigdigitalweldinginterfacemode); };
		DRFL_RETURN app_weld_set_interface_eip_r2m_option(CONFIG_DIGITAL_WELDING_INTERFACE_OPTION pConfigdigitalweldinginterfaceoption){ return _app_weld_set_interface_eip_r2m_option_ex2(_rbtCtrl, pConfigdigitalweldinginterfaceoption); };
		DRFL_RETURN app_weld_set_interface_eip_r2m_process(CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS pConfigdigitalweldinginterfaceprocess){ return _app_weld_set_interface_eip_r2m_process_ex2(_rbtCtrl, pConfigdigitalweldinginterfaceprocess); };
		DRFL_RETURN app_weld_set_interface_eip_r2m_test(CONFIG_DIGITAL_WELDING_INTERFACE_TEST pConfigdigitalweldinginterfacetest){ return _app_weld_set_interface_eip_r2m_test_ex2(_rbtCtrl, pConfigdigitalweldinginterfacetest); };
		DRFL_RETURN app_weld_set_weld_cond_analog(unsigned char iVirtualWelding, float fTargetVoltage, float fTargetCurrent, float fTargetVel, float fMinVel, float fMaxVel, float fTargetFeedingSpeed){ return _app_weld_set_weld_cond_analog_ex2(_rbtCtrl, iVirtualWelding, fTargetVoltage, fTargetCurrent, fTargetVel, fMinVel, fMaxVel, fTargetFeedingSpeed); };
		DRFL_RETURN app_weld_set_weld_cond_digital(CONFIG_DIGITAL_WELDING_CONDITION pConfigdigitalweldingcondition){ return _app_weld_set_weld_cond_digital_ex2(_rbtCtrl, pConfigdigitalweldingcondition); };
		DRFL_RETURN app_weld_weave_cond_circular(float fOffsetY, float fOffsetZ, float fGradient, float fwWdt[2], float fwT[2]){ return _app_weld_weave_cond_circular_ex2(_rbtCtrl, fOffsetY, fOffsetZ, fGradient, fwWdt, fwT); };
		DRFL_RETURN app_weld_weave_cond_sinusoidal(float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle){ return _app_weld_weave_cond_sinusoidal_ex2(_rbtCtrl, fOffsetY, fOffsetZ, fGradient, fWeavingWidth, fWeavingCycle); };
		DRFL_RETURN app_weld_weave_cond_trapezoidal(CONFIG_TRAPEZOID_WEAVING_SETTING pConfigtrapezoidweavingsetting){ return _app_weld_weave_cond_trapezoidal_ex2(_rbtCtrl, pConfigtrapezoidweavingsetting); };
		DRFL_RETURN app_weld_weave_cond_zigzag(float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle){ return _app_weld_weave_cond_zigzag_ex2(_rbtCtrl, fOffsetY, fOffsetZ, fGradient, fWeavingWidth, fWeavingCycle); };
		DRFL_RETURN amoveb(MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, MOVE_MODE eMoveMode, MOVE_REFERENCE eMoveReference){ return _amoveb_ex2(_rbtCtrl, tTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference); };
		DRFL_RETURN amovec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE){ return _amovec_ex2(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime,eMoveMode, eMoveReference, fTargetAngle1, fTargetAngle2, eBlendingType); };
		DRFL_RETURN amovej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE){ return _amovej_ex2(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eBlendingType); };
		DRFL_RETURN amovejx(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE){ return _amovejx_ex2(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eBlendingType); };
		DRFL_RETURN amovel(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE){ return _amovel_ex2(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eBlendingType); };
		DRFL_RETURN amove_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned int nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL){ return _amove_periodic_ex2(_rbtCtrl, fAmplitude, fPeriodic, fAccelTime, nRepeat, eMoveReference); };
		DRFL_RETURN amovesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE){ return _amovesj_ex2( _rbtCtrl,  fTargetPos, nPosCount,  fTargetVel,  fTargetAcc, fTargetTime , eMoveMode); };
		DRFL_RETURN amove_spiral(TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL){  return _amove_spiral_ex2(_rbtCtrl, eTaskAxis, fRevolution, fMaximuRadius, fMaximumLength, fTargetVel, fTargetAcc, fTargetTime, eMoveReference);  };
		DRFL_RETURN amovesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT){ return _amovesx_ex2(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime,eMoveMode, eMoveReference); };
		DRFL_RETURN get_conveyor_object(unsigned char iConId, float fTimeout, unsigned char iContainerType, POSITION tPosObjCoord, int *pGetConveyorObj){ return _get_conveyor_object_ex2(_rbtCtrl, iConId, fTimeout, iContainerType, tPosObjCoord, pGetConveyorObj); };
		DRFL_RETURN get_cpu_usage(LPSYSTEM_CPUUSAGE iCpuUsage){ return _get_cpu_usage_ex2(_rbtCtrl, iCpuUsage); };
		DRFL_RETURN get_disk_usage(LPSYSTEM_DISKSIZE iDiskUsage){ return _get_disk_usage_ex2(_rbtCtrl, iDiskUsage); };
		DRFL_RETURN get_fts_value(LPCALIBRATE_FTS_RESPONSE iFtsValue){ return _get_fts_value_ex2(_rbtCtrl, iFtsValue); };
		DRFL_RETURN get_kernel_log(const char* iStrKernel){ return _get_kernel_log_ex2(_rbtCtrl, iStrKernel); };
		DRFL_RETURN measure_welding_tcp(unsigned char iMode, float fStickout, float fTargetPos[9][NUMBER_OF_JOINT]){ return _measure_welding_tcp_ex2(_rbtCtrl, iMode, fStickout, fTargetPos); };
		DRFL_RETURN get_conveyor_coord(POSITION tPosTeachPointQ, unsigned char nTeachCount, POSITION tPosTeachPointP[5], unsigned int EncoderCount[5], LPMEASURE_CONVEYOR_COORD_RESPONSE iConvCoord){ return _get_conveyor_coord_ex2(_rbtCtrl, tPosTeachPointQ, nTeachCount, tPosTeachPointP, EncoderCount, iConvCoord); };
		DRFL_RETURN get_conveyor_coord2(int nResolution, LPMEASURE_CONVEYOR_COORD_RESPONSE iConvCoord2){ return _get_conveyor_coord2_ex2(_rbtCtrl, nResolution, iConvCoord2); };
		DRFL_RETURN get_conveyor_distance(unsigned int nFilterSize, LPMEASURE_CONVEYOR_DISTANCE_RESPONSE iConvDist){ return _get_conveyor_distance_ex2(_rbtCtrl, nFilterSize, iConvDist); };
		DRFL_RETURN add_conveyor( const char* szName, CONVEYOR_BASIC tBASIC, CONVEYOR_COORD_EX tCoord, CONVEYOR_DISTANCE tDistance){ return _add_conveyor_ex2( _rbtCtrl, szName, tBASIC, tCoord, tDistance); };
		DRFL_RETURN del_conveyor( const char* szName){ return _del_conveyor_ex2(_rbtCtrl, szName); };
		DRFL_RETURN set_conveyor_monitoring( const char* szName, unsigned char bStart) { return _set_conveyor_monitoring_ex2(_rbtCtrl, szName, bStart); };
		DRFL_RETURN calc_conveyor_param( unsigned char iType, unsigned char iEncoderChannel, unsigned char iTriggerChannel, unsigned char iTriggerEdgeType, float fTriggerMuteTime, char iChannel[2], unsigned char iValue[2]){ return _calc_conveyor_param_ex2( _rbtCtrl, iType, iEncoderChannel, iTriggerChannel, iTriggerEdgeType, fTriggerMuteTime, iChannel, iValue); };
		DRFL_RETURN calc_conveyor_param2( int iDistance2Count, POSITION tPosConCoord, unsigned char iTargetRef){ return _calc_conveyor_param2_ex2( _rbtCtrl, iDistance2Count, tPosConCoord, iTargetRef); };
		DRFL_RETURN drl_step_run2( unsigned int nLine){ return _drl_step_run2_ex2( _rbtCtrl, nLine); };
		DRFL_RETURN reset_encorder( unsigned char iChannel){ return _reset_encorder_ex2( _rbtCtrl, iChannel); };
		DRFL_RETURN set_conveyor( const char* szName, unsigned char *pSetConveyor){ return _set_conveyor_ex2( _rbtCtrl,  szName, pSetConveyor); };
		DRFL_RETURN set_conveyor_ex( const char* szName, CONVEYOR_BASIC tBASIC, CONVEYOR_COORD_EX tCoord, CONVEYOR_DISTANCE tDistance, unsigned char *pSetConveyorex){ return _set_conveyor_ex_ex2( _rbtCtrl, szName, tBASIC, tCoord , tDistance,pSetConveyorex); };
		DRFL_RETURN set_conveyor_track( unsigned char iConId, unsigned char bTracking, unsigned char bMate, float fDuration, float fDummy[5]){ return _set_conveyor_track_ex2( _rbtCtrl, iConId, bTracking, bMate, fDuration, fDummy); };
		DRFL_RETURN set_digital_welding_monitoring_mode( unsigned char bEnable){ return _set_digital_welding_monitoring_mode_ex2( _rbtCtrl, bEnable); };
		DRFL_RETURN set_digital_welding_signal_output( unsigned char cDataType, float fData){ return _set_digital_welding_signal_output_ex2( _rbtCtrl, cDataType, fData); };
		DRFL_RETURN set_encorder_mode( unsigned char iChannel, unsigned char iABMode, unsigned char iZMode, unsigned char iSMode, unsigned char iInvMode, unsigned int  nPulseAZ){ return  _set_encorder_mode_ex2( _rbtCtrl, iChannel, iABMode, iZMode, iSMode, iInvMode, nPulseAZ); };
		DRFL_RETURN set_encorder_polarity( unsigned char iChannel, unsigned char iPolarity[ENCORDER_POLARITY_LAST]){ return _set_encorder_polarity_ex2( _rbtCtrl, iChannel, iPolarity); };
		DRFL_RETURN set_process_button( unsigned char iUsage){ return _set_process_button_ex2(_rbtCtrl, iUsage); };
		DRFL_RETURN set_remote_control( unsigned char bEnable, CONFIG_IO_FUNC tFunc[TYPE_LAST][NUM_REMOTE_CONTROL]){ return _set_remote_control_ex2( _rbtCtrl, bEnable, tFunc); };
		DRFL_RETURN set_system_time( const char* szData, const char* szTime){ return _set_system_time_ex2( _rbtCtrl, szData, szTime); };
		DRFL_RETURN set_time_sync( unsigned char iUsage){ return _set_time_sync_ex2( _rbtCtrl, iUsage); };
		DRFL_RETURN set_welding_cockpit_setting( unsigned char bEnable, unsigned char bWeldingType){ return _set_welding_cockpit_setting_ex2(_rbtCtrl, bEnable, bWeldingType); };
		DRFL_RETURN update_license( const char* lpszLicense){ return _update_license_ex2( _rbtCtrl, lpszLicense); };
		//API Second Priority End

		//API Low Priority Start
		DRFL_RETURN alter_motion( float fTargetPos[NUM_TASK]){ return _alter_motion_ex2( _rbtCtrl, fTargetPos); };
		DRFL_RETURN check_motion( int *CheckMotion){ return _check_motion_ex2( _rbtCtrl, CheckMotion); };
		DRFL_RETURN check_motion_ex( int *CheckMotionEx){ return _check_motion_ex_ex2(_rbtCtrl, CheckMotionEx); };
		DRFL_RETURN check_orientation_condition_abs( FORCE_AXIS eForceAxis, float fTargetMin[NUM_TASK], float fTargetMax[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL){ return _check_orientation_condition_abs_ex2( _rbtCtrl, eForceAxis, fTargetMin, fTargetMax, eForceReference); };
		DRFL_RETURN check_orientation_condition_rel( FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL){ return _check_orientation_condition_rel_ex2( _rbtCtrl, eForceAxis, fTargetMin, fTargetMax, fTargetPos, eForceReference); };
		DRFL_RETURN check_position_condition( FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], MOVE_MODE eMode = MOVE_MODE_ABSOLUTE, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL){ return _check_position_condition_ex2( _rbtCtrl, eForceAxis, fTargetMin, fTargetMax, fTargetPos, eMode, eForceReference); };
		DRFL_RETURN check_position_condition_abs( FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL){ return _check_position_condition_abs_ex2( _rbtCtrl, eForceAxis, fTargetMin, fTargetMax, eForceReference); };
		DRFL_RETURN check_position_condition_rel( FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL){ return _check_position_condition_rel_ex2( _rbtCtrl, eForceAxis, fTargetMin, fTargetMax, fTargetPos, eForceReference); };
		DRFL_RETURN config_kt_factory_makers( int bEnable, const char* szIpAddress, int nPort, const char* szDeviceId, const char* szDevicePw, const char* szGatewayId){ return _config_kt_factory_makers_ex2( _rbtCtrl, bEnable, szIpAddress, nPort, szDeviceId, szDevicePw, szGatewayId); };
		DRFL_RETURN multi_jog( float fTargetPos[NUM_TASK], MOVE_REFERENCE eMoveReference, float fVelocity){ return _multi_jog_ex2( _rbtCtrl, fTargetPos, eMoveReference, fVelocity); };
		DRFL_RETURN move_resume(){ return _move_resume_ex2( _rbtCtrl); };
		DRFL_RETURN delete_file( const char* szFileName){ return _delete_file_ex2(_rbtCtrl, szFileName); };
		DRFL_RETURN disable_alter_motion(){ return _disable_alter_motion_ex2(_rbtCtrl); };
		DRFL_RETURN drl_break( int nLineNum){ return _drl_break_ex2( _rbtCtrl, nLineNum); };
		DRFL_RETURN drl_step_run(){ return _drl_step_run_ex2(_rbtCtrl); };
		DRFL_RETURN enable_alter_motion( int iCycleTime, PATH_MODE ePathMode, COORDINATE_SYSTEM eTargetRef, float fLimitDpos[2], float fLimitDposPer[2]){ return _enable_alter_motion_ex2( _rbtCtrl, iCycleTime, ePathMode, eTargetRef, fLimitDpos, fLimitDposPer); };
		DRFL_RETURN fkin( float fSourcePos[NUM_JOINT], COORDINATE_SYSTEM eTargetRef , LPROBOT_POSE pFkin){ return _fkin_ex2( _rbtCtrl, fSourcePos, eTargetRef ,pFkin); };
		DRFL_RETURN get_normal( float fTargetPosX[NUMBER_OF_JOINT], float fTargetPosY[NUMBER_OF_JOINT], float fTargetPosZ[NUMBER_OF_JOINT],  LPNORMAL_VECTOR_RESPONSE pGetNormal){ return _get_normal_ex2( _rbtCtrl, fTargetPosX, fTargetPosY, fTargetPosZ, pGetNormal); };
		DRFL_RETURN get_orientation_error( float fPosition1[NUM_TASK], float fPosition2[NUM_TASK], TASK_AXIS eTaskAxis, float *pGetOrientationError){ return _get_orientation_error_ex2( _rbtCtrl, fPosition1, fPosition2, eTaskAxis, pGetOrientationError); };
		DRFL_RETURN ikin( float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef , LPROBOT_POSE pIkin){ return _ikin_ex2( _rbtCtrl, fSourcePos, iSolutionSpace, eTargetRef, pIkin); };
		DRFL_RETURN is_done_bolt_tightening( FORCE_AXIS eForceAxis, float fTargetTor = 0.f, float fTimeout = 0.f){ return _is_done_bolt_tightening_ex2( _rbtCtrl, eForceAxis, fTargetTor = 0.f, fTimeout = 0.f); };
		DRFL_RETURN release_compliance_ctrl(){ return _release_compliance_ctrl_ex2(_rbtCtrl); };
		DRFL_RETURN release_force( float fTargetTime = 0.f){ return _release_force_ex2(_rbtCtrl, fTargetTime = 0.f); };
		DRFL_RETURN report_tcp_client( unsigned char iIdentifier, LPREPORT_TCP_CLIENT pReportTcpClient){ return _report_tcp_client_ex2( _rbtCtrl, iIdentifier, pReportTcpClient); };
		DRFL_RETURN response_restore_controller(unsigned char iProcess) { return _response_restore_controller_ex2(_rbtCtrl, iProcess); };
		DRFL_RETURN set_deflection_comp_mode(unsigned char bEnable) { return _set_deflection_comp_mode_ex2(_rbtCtrl, bEnable); };
		DRFL_RETURN set_desired_force(float fTargetForce[NUM_TASK], unsigned char iTargetDirection[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f, FORCE_MODE eForceMode = FORCE_MODE_ABSOLUTE) { return _set_desired_force_ex2(_rbtCtrl, fTargetForce, iTargetDirection, eForceReference, fTargetTime, eForceMode); };
		//DRFL_RETURN set_license_option(unsigned int nOption) { return _set_license_option_ex2(_rbtCtrl, nOption); };
		DRFL_RETURN set_monitoring_control(unsigned char iControl) { return _set_monitoring_control_ex2(_rbtCtrl, iControl); };
		DRFL_RETURN set_oscillation_check(unsigned int puArg) { return _set_oscillation_check_ex2(_rbtCtrl, puArg); };
		DRFL_RETURN set_package_restore(const char szVersName[MAX_SYMBOL_SIZE]) { return _set_package_restore_ex2(_rbtCtrl, szVersName); };
		DRFL_RETURN set_ref_coord(COORDINATE_SYSTEM eTargetCoordSystem) { return _set_ref_coord_ex2(_rbtCtrl, eTargetCoordSystem); };
		DRFL_RETURN set_restore_list(LPPACKAGE_RESTORE_LIST iPackList) { return _set_restore_list_ex2(_rbtCtrl, iPackList); };
		DRFL_RETURN set_singularity_handling(SINGULARITY_AVOIDANCE eMode) { return _set_singularity_handling_ex2(_rbtCtrl, eMode); };
		DRFL_RETURN set_stiffnessx(float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f) { return _set_stiffnessx_ex2(_rbtCtrl, fTargetStiffness, eForceReference, fTargetTime); };
		DRFL_RETURN set_teach_mode(unsigned char bMode) { return _set_teach_mode_ex2(_rbtCtrl, bMode); };
		//DRFL_RETURN set_unzip(unsigned char iResult, const char szDirName[MAX_SYMBOL_SIZE]) { return _set_unzip_ex2(_rbtCtrl, iResult, szDirName); };
		DRFL_RETURN simulatior_keyin(unsigned char iKey) { return _simulatior_keyin_ex2(_rbtCtrl, iKey); };
		DRFL_RETURN task_compliance_ctrl(float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f) { return _task_compliance_ctrl_ex2(_rbtCtrl, fTargetStiffness, eForceReference, fTargetTime); };
		DRFL_RETURN twait(float fTime) { return _twait_ex2(_rbtCtrl, fTime); };
		DRFL_RETURN update_excute_package(unsigned char iExecute) { return _update_excute_package_ex2(_rbtCtrl, iExecute); };
		DRFL_RETURN update_fts_data(float fOffset[NUMBER_OF_JOINT]) { return _update_fts_data_ex2(_rbtCtrl, fOffset); };
		DRFL_RETURN update_gravity_param(COUNTER_BALANCE_PARAM_DATA tClParam, CALIBRATION_PARAM_DATA tCrParam) { return _update_gravity_param_ex2(_rbtCtrl, tClParam, tCrParam); };
		DRFL_RETURN update_jts_data(float fOffset[NUMBER_OF_JOINT], float fScale[NUMBER_OF_JOINT]) { return _update_jts_data_ex2(_rbtCtrl, fOffset, fScale); };
		DRFL_RETURN update_local_package(unsigned char iTarget[UPDATE_TARGET_LAST], const char szDirName[MAX_SYMBOL_SIZE]) { return _update_local_package_ex2(_rbtCtrl, iTarget, szDirName); };
		DRFL_RETURN update_network_package(unsigned char iTarget[UPDATE_TARGET_LAST], unsigned char iNetType, const char szIpAddress[16], const char szFileName[MAX_SYMBOL_SIZE]) { return _update_network_package_ex2(_rbtCtrl, iTarget, iNetType, szIpAddress, szFileName); };
		DRFL_RETURN update_process(unsigned char pProcessInfo, unsigned char pStatusInfo, LPSYSTEM_UPDATE_RESPONSE iUpdateProcs) { return _update_process_ex2(_rbtCtrl, pProcessInfo, pStatusInfo, iUpdateProcs); };
		//API Low Priority End
	protected:

	private:

	};	//end of class CDRFLEx2
#endif
}

