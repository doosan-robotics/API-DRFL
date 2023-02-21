/*    ========================================================================
    =                   Doosan Robot Framework Library                        =
    =                   Copyright (c) Doosan Robotics.                        =   
    =_______________________________________________________________________  =
    = Title             : Doosan Robot Framwork Library                       =
    = Author            : Lee Jeong-Woo<jeongwoo1.lee@doosan.com>             =
    = Description       : -                                                   =
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


namespace DRAFramework 
{
    typedef void (*TOnMonitoringDataExCB)(const LPMONITORING_DATA_EX);
    typedef void (*TOnMonitoringCtrlIOExCB)(const LPMONITORING_CTRLIO_EX);
    typedef void (*TOnTpPopupCB)(LPMESSAGE_POPUP);
    typedef void (*TOnTpLogCB)(const char[256]);
    typedef void (*TOnTpGetUserInputCB)(LPMESSAGE_INPUT);
    typedef void (*TOnTpProgressCB)(LPMESSAGE_PROGRESS); 

    typedef void (*TOnRTMonitoringDataCB)(const LPRT_OUTPUT_DATA_LIST);

    typedef void (*TOnMonitoringSafetyStateCB)(const SAFETY_STATE);
    typedef void (*TOnMonitoringRobotSystemCB)(const ROBOT_SYSTEM);
    typedef void (*TOnMonitoringSafetyStopTypeCB)(const unsigned char);

#if defined(_IS_PLATFORM)
    typedef void (*TOnMonitoringPowerButtonCB)(const unsigned char);
    typedef void (*TOnMonitoringCollisionSensitivityCB)(const float);
    typedef void (*TOnMonitoringWeldingCB)(const LPMONITORING_WELDING);
    typedef void (*TOnMonitoringModeCB)(const SAFETY_MODE);
    typedef void (*TOnMonitoringAnalogWeldingCB)(const LPMONITORING_ALALOG_WELDING);
    typedef void (*TOnMonitoringDigitalWeldingCB)(const LPMONITORING_DIGITAL_WELDING);
    typedef void (*TOnMonitoringDigitalWeldingCommStateCB)(const LPDIGITAL_WELDING_COMM_STATE);
    typedef void (*TOnMonitoringUserCoordExtForceCB)(const LPUSER_COORD_EXTERNAL_FORCE_PACKET);
    typedef void (*TOnMonitoringIeSlaveCB)(const LPMONITORING_IE_SLAVE);
    typedef void (*TOnMonitoringResponseCommandCB)(const unsigned short);

	typedef void (*TOnProgramStartCB)();
	typedef void (*TOnProgramWatchVariableCB)(const LPPROGRAM_WATCH_VARIABLE);
	typedef void (*TOnProgramErrorCB)(const LPPROGRAM_ERROR);

	typedef void (*TOnProgramDrlLineExCB)(const LPPROGRAM_EXECUTION_EX);
	typedef void (*TOnProgramStateCB)(const unsigned char);
    typedef void (*TOnMonitoringCtrlIOEx2CB)(const LPMONITORING_CTRLIO_EX2);
    typedef void(*TOnMonitoringProtectiveSafeOffCB)();
#endif

#ifdef __cplusplus
    extern "C" 
    {
#endif
        //////////////////////
        /////DRL Wrapping/////
        //////////////////////
        
        DRFL_API LPROBOTCONTROL _create_robot_control_udp();
        DRFL_API void _destroy_robot_control_udp(LPROBOTCONTROL pCtrl);

        ////////////////////////////////////////////////////////////////////////////
        // Instance                                                               //
        ////////////////////////////////////////////////////////////////////////////

        // connection
        //for ROS org DRFL_API bool _OpenConnection(LPROBOTCONTROL pCtrl, const char* lpszIpAddr = "192.168.137.100");
        DRFL_API bool _open_connection(LPROBOTCONTROL pCtrl, const char* lpszIpAddr = "192.168.137.100", unsigned int usPort = 12345);
        DRFL_API bool _close_connection(LPROBOTCONTROL pCtrl);

        DRFL_API bool _connect_rt_control(LPROBOTCONTROL pCtrl, const char* lpszIpAddr = "192.168.137.100", unsigned int usPort = 12347);
        DRFL_API bool _disconnect_rt_control(LPROBOTCONTROL pCtrl);

        ////////////////////////////////////////////////////////////////////////////
        // RT Control                                                             //
        ////////////////////////////////////////////////////////////////////////////   

        DRFL_API const char* _get_rt_control_output_version_list(LPROBOTCONTROL pCtrl);
        DRFL_API const char* _get_rt_control_input_version_list(LPROBOTCONTROL pCtrl);
        DRFL_API const char* _get_rt_control_input_data_list(LPROBOTCONTROL pCtrl, const char* szVersion);
        DRFL_API const char* _get_rt_control_output_data_list(LPROBOTCONTROL pCtrl, const char* szVersion);
        DRFL_API bool _set_rt_control_input(LPROBOTCONTROL pCtrl, const char* szVersion, float fPeriod, int nLossCnt);
        DRFL_API bool _set_rt_control_output(LPROBOTCONTROL pCtrl, const char* szVersion, float fPeriod, int nLossCnt);
        
        DRFL_API bool _start_rt_control(LPROBOTCONTROL pCtrl);
        DRFL_API bool _stop_rt_control(LPROBOTCONTROL pCtrl);

        DRFL_API bool _set_velj_rt(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_JOINT]);
        DRFL_API bool _set_accj_rt(LPROBOTCONTROL pCtrl, float fTargetAcc[NUM_JOINT]);
        DRFL_API bool _set_velx_rt(LPROBOTCONTROL pCtrl, float fTransVel, float fRotationVel = -10000);
        DRFL_API bool _set_accx_rt(LPROBOTCONTROL pCtrl, float fTransAcc, float fRotationAcc = -10000);

        DRFL_API LPRT_OUTPUT_DATA_LIST _read_data_rt(LPROBOTCONTROL pCtrl);
        DRFL_API bool _write_data_rt(LPROBOTCONTROL pCtrl, float fExternalForceTorque[NUM_JOINT], int iExternalDI, int iExternalDO, float fExternalAnalogInput[6], float fExternalAnalogOutput[6]);
        
        DRFL_API bool _servoj_rt(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
        DRFL_API bool _servol_rt(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[NUM_TASK], float fTargetAcc[NUM_TASK], float fTargetTime);
        DRFL_API bool _speedj_rt(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
        DRFL_API bool _speedl_rt(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_TASK], float fTargetAcc[NUM_TASK], float fTargetTime);
        DRFL_API bool _torque_rt(LPROBOTCONTROL pCtrl, float fMotorTor[NUM_JOINT], float fTargetTime);

        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////

        // get verion string
        DRFL_API bool _get_system_version(LPROBOTCONTROL pCtrl, LPSYSTEM_VERSION pVersion);
        DRFL_API const char* _get_library_version(LPROBOTCONTROL pCtrl);

        // get robot safety mode(manual, auto)
        DRFL_API ROBOT_MODE _get_robot_mode(LPROBOTCONTROL pCtrl);

        // set robot mode mode(manual, auto)
        DRFL_API bool _set_robot_mode(LPROBOTCONTROL pCtrl, ROBOT_MODE eMode);

       // get robot state( initial, standby, moving, safe-off, teach, ...) 
        DRFL_API ROBOT_STATE _get_robot_state(LPROBOTCONTROL pCtrl);
        // set robot control state
        DRFL_API bool _set_robot_control(LPROBOTCONTROL pCtrl, ROBOT_CONTROL eControl);
        DRFL_API CONTROL_MODE _get_control_mode(LPROBOTCONTROL pCtrl);
        
        // get robot system(real robot, virtrul robot)
        DRFL_API ROBOT_SYSTEM _get_robot_system(LPROBOTCONTROL pCtrl);
        // set robot system(real robot, virtrul robot)
        DRFL_API bool _set_robot_system(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem);

        // set robot speed mode(noraml reduced)
        DRFL_API bool _set_robot_speed_mode(LPROBOTCONTROL pCtrl, SPEED_MODE eSpeedMode);      
        // get robot speed mode(noraml reduced)
        DRFL_API SPEED_MODE _get_robot_speed_mode(LPROBOTCONTROL pCtrl);

        // get roobt axis data
        DRFL_API LPROBOT_POSE _get_current_pose(LPROBOTCONTROL pCtrl, ROBOT_SPACE eSpaceType = ROBOT_SPACE_JOINT);

        // get rotation matrix
        DRFL_API float(* _get_current_rotm(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eTargetRef))[3];
        
        // get current solution space
        DRFL_API unsigned char _get_current_solution_space(LPROBOTCONTROL pCtrl);
        
        // get current joint position list
        DRFL_API LPROBOT_POSE _get_current_posj(LPROBOTCONTROL pCtrl);
        // get current control space
        DRFL_API ROBOT_SPACE _get_control_space(LPROBOTCONTROL pCtrl);
        // get current joint velocity
        DRFL_API LPROBOT_VEL _get_current_velj(LPROBOTCONTROL pCtrl);
        // get target joint angle
        DRFL_API LPROBOT_POSE _get_desired_posj(LPROBOTCONTROL pCtrl);
        // get current flange task position
        DRFL_API LPROBOT_POSE _get_current_tool_flange_posx(LPROBOTCONTROL pCtrl);
        // get current task velocity
        DRFL_API LPROBOT_VEL _get_current_velx(LPROBOTCONTROL pCtrl);
        // get target task position
        //DRFL_API LPROBOT_POSE _get_desired_posx(LPROBOTCONTROL pCtrl);
        // get target task velocity
        DRFL_API LPROBOT_VEL _get_desired_velx(LPROBOTCONTROL pCtrl);
        // get current joint sensor torque value
        DRFL_API LPROBOT_FORCE _get_joint_torque(LPROBOTCONTROL pCtrl);
        // get current external force
        DRFL_API LPROBOT_FORCE _get_external_torque(LPROBOTCONTROL pCtrl);
        // get current external force in tool
        DRFL_API LPROBOT_FORCE _get_tool_force(LPROBOTCONTROL pCtrl);

        // get program running state
        DRFL_API DRL_PROGRAM_STATE _get_program_state(LPROBOTCONTROL pCtrl);

        // set safe-stop reset type
        DRFL_API bool _set_safe_stop_reset_type(LPROBOTCONTROL pCtrl, SAFE_STOP_RESET_TYPE eResetType = SAFE_STOP_RESET_TYPE_DEFAULT);

        // get robot system alarm
        DRFL_API LPLOG_ALARM _get_last_alarm(LPROBOTCONTROL pCtrl);        
                
        ////////////////////////////////////////////////////////////////////////////
        //  access control                                                       //
        ////////////////////////////////////////////////////////////////////////////

        // manage access control
        DRFL_API bool _manage_access_control(LPROBOTCONTROL pCtrl, MANAGE_ACCESS_CONTROL eAccessControl = MANAGE_ACCESS_CONTROL_REQUEST);
        
        ////////////////////////////////////////////////////////////////////////////
        // Callback operation                                                    //
        ////////////////////////////////////////////////////////////////////////////
        DRFL_API void _set_on_monitoring_state(LPROBOTCONTROL pCtrl, TOnMonitoringStateCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_data(LPROBOTCONTROL pCtrl, TOnMonitoringDataCB pCallbackFunc);   
        DRFL_API void _set_on_monitoring_data_ex(LPROBOTCONTROL pCtrl, TOnMonitoringDataExCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_ctrl_io(LPROBOTCONTROL pCtrl, TOnMonitoringCtrlIOCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_ctrl_io_ex(LPROBOTCONTROL pCtrl, TOnMonitoringCtrlIOExCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_modbus(LPROBOTCONTROL pCtrl, TOnMonitoringModbusCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_speed_mode(LPROBOTCONTROL pCtrl, TOnMonitoringSpeedModeCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_access_control(LPROBOTCONTROL pCtrl, TOnMonitoringAccessControlCB pCallbackFunc);
        DRFL_API void _set_on_log_alarm(LPROBOTCONTROL pCtrl, TOnLogAlarmCB pCallbackFunc);
        DRFL_API void _set_on_tp_popup(LPROBOTCONTROL pCtrl, TOnTpPopupCB pCallbackFunc);
        DRFL_API void _set_on_tp_log(LPROBOTCONTROL pCtrl, TOnTpLogCB pCallbackFunc);
        DRFL_API void _set_on_tp_progress(LPROBOTCONTROL pCtrl, TOnTpProgressCB pCallbackFunc);
        DRFL_API void _set_on_tp_get_user_input(LPROBOTCONTROL pCtrl, TOnTpGetUserInputCB pCallbackFunc);
        DRFL_API void _set_on_program_stopped(LPROBOTCONTROL pCtrl, TOnProgramStoppedCB pCallbackFunc);
        DRFL_API void _set_on_homming_completed(LPROBOTCONTROL pCtrl, TOnHommingCompletedCB pCallbackFunc);
        DRFL_API void _set_on_tp_initializing_completed(LPROBOTCONTROL pCtrl, TOnTpInitializingCompletedCB pCallbackFunc);
        DRFL_API void _set_on_mastering_need(LPROBOTCONTROL pCtrl, TOnMasteringNeedCB pCallbackFunc);
        DRFL_API void _set_on_disconnected(LPROBOTCONTROL pCtrl, TOnDisconnectedCB pCallbackFunc);

        DRFL_API void _set_on_monitoring_safety_state(LPROBOTCONTROL pCtrl, TOnMonitoringSafetyStateCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_robot_system(LPROBOTCONTROL pCtrl, TOnMonitoringRobotSystemCB pCallbackFunc);
        DRFL_API void _set_on_monitoring_safety_stop_type(LPROBOTCONTROL pCtrl, TOnMonitoringSafetyStopTypeCB pCallbackFunc);

        DRFL_API void _set_on_rt_monitoring_data(LPROBOTCONTROL pCtrl, TOnRTMonitoringDataCB pCallbackFunc);
        DRFL_API void _set_on_rt_log_alarm(LPROBOTCONTROL pCtrl,TOnLogAlarmCB pCallbackFunc);

        DRFL_API LPROBOT_POSE _trans(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_TASK], float fOffset[NUM_TASK], COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE, COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
        DRFL_API LPROBOT_POSE _ikin(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
		DRFL_API LPINVERSE_KINEMATIC_RESPONSE _ikin_ex(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef, unsigned char iRefPosOpt);
        DRFL_API LPROBOT_POSE _fkin(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_JOINT], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
        DRFL_API LPROBOT_POSE _addto(LPROBOTCONTROL pCtrl, float fSourcePos[NUM_JOINT], float fOffset[NUM_JOINT]);

        DRFL_API unsigned char _get_solution_space(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK]);

        DRFL_API LPROBOT_TASK_POSE _get_current_posx(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE);
        DRFL_API LPROBOT_POSE _get_desired_posx(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE);
        DRFL_API float _get_orientation_error(LPROBOTCONTROL pCtrl, float fPosition1[NUM_TASK], float fPosition2[NUM_TASK], TASK_AXIS eTaskAxis);

        DRFL_API bool _set_workpiece_weight(LPROBOTCONTROL pCtrl, float fWeight = 0.0, float fCog[3] = COG_DEFAULT, COG_REFERENCE eCogRef = COG_REFERENCE_TCP, ADD_UP eAddUp = ADD_UP_REPLACE, float fStartTime = -10000, float fTransitionTIme = -10000);

        DRFL_API float _get_workpiece_weight(LPROBOTCONTROL pCtrl);
        DRFL_API bool _reset_workpiece_weight(LPROBOTCONTROL pCtrl);
         
        DRFL_API bool _tp_popup_response(LPROBOTCONTROL pCtrl, POPUP_RESPONSE eRes);
        DRFL_API bool _tp_get_user_input_response(LPROBOTCONTROL pCtrl, const char* lpszTextString);
        ////////////////////////////////////////////////////////////////////////////
        //  motion Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////
        // basci motion(hold to run)
        DRFL_API bool _jog(LPROBOTCONTROL pCtrl, JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity);
        DRFL_API bool _multi_jog(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], MOVE_REFERENCE eMoveReference, float fVelocity);
        DRFL_API bool _move_home(LPROBOTCONTROL pCtrl, MOVE_HOME eMode = MOVE_HOME_MECHANIC, unsigned char bRun = (unsigned char)1);
        DRFL_API LPROBOT_POSE _get_user_home(LPROBOTCONTROL pCtrl);

        // stop motion
        DRFL_API bool _stop(LPROBOTCONTROL pCtrl, STOP_TYPE eStopType = STOP_TYPE_QUICK);
        // pause motion
        DRFL_API bool _move_pause(LPROBOTCONTROL pCtrl);
        // resume motion
        DRFL_API bool _move_resume(LPROBOTCONTROL pCtrl);
        // wait motion
        DRFL_API bool _mwait(LPROBOTCONTROL pCtrl);

        // joint motion
        DRFL_API bool _movej(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _movej_ex(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _amovej(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        // linear motion
        DRFL_API bool _movel(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _amovel(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        // circle motion
        DRFL_API bool _movec(LPROBOTCONTROL pCtrl, float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _amovec(LPROBOTCONTROL pCtrl, float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        // bleind motion
        DRFL_API bool _moveb(LPROBOTCONTROL pCtrl, MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE);
        DRFL_API bool _amoveb(LPROBOTCONTROL pCtrl, MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE);
        // joint motion as task information
        DRFL_API bool _movejx(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        DRFL_API bool _amovejx(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        // spline motion as joint information
        DRFL_API bool _movesj(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        DRFL_API bool _movesj_ex(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUMBER_OF_JOINT], float fTargetAcc[NUMBER_OF_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        DRFL_API bool _amovesj(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        DRFL_API bool _amovesj_ex(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUMBER_OF_JOINT], float fTargetAcc[NUMBER_OF_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE);
        // spline motion as task information
        DRFL_API bool _movesx(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);
        DRFL_API bool _amovesx(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);
        // spiral motion
        DRFL_API bool _move_spiral(LPROBOTCONTROL pCtrl, TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
        DRFL_API bool _amove_spiral(LPROBOTCONTROL pCtrl, TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
        // periodic motion
        DRFL_API bool _move_periodic(LPROBOTCONTROL pCtrl, float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned char nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);
        DRFL_API bool _amove_periodic(LPROBOTCONTROL pCtrl, float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned char nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL);

        // environment adaptive motion
        DRFL_API bool _servoj(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
        DRFL_API bool _servol(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime);

        DRFL_API bool _speedj(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
        DRFL_API bool _speedl(LPROBOTCONTROL pCtrl, float fTargetVel[NUM_TASK], float fTargetAcc[2], float fTargetTime);

		DRFL_API bool _servoj_g(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
        DRFL_API bool _servol_g(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime);
 
		DRFL_API bool _movesx_g(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime /* = 0.f */, MOVE_MODE eMoveMode /* = MOVE_MODE_ABSOLUTE */, MOVE_REFERENCE eMoveReference /* = MOVE_REFERENCE_BASE */, SPLINE_VELOCITY_OPTION eVelOpt /* = SPLINE_VELOCITY_OPTION_DEFAULT */);
		DRFL_API bool _movesj_g(LPROBOTCONTROL pCtrl, float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime /* = 0.f */, MOVE_MODE eMoveMode /* = MOVE_MODE_ABSOLUTE */);
		////////////////////////////////////////////////////////


        ////////////////////////////////////////////////////////////////////////////
        //  GPIO Operations                                                       //
        ////////////////////////////////////////////////////////////////////////////
        // set digital output on flange
        DRFL_API bool _set_tool_digital_output(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        // get digital input on flange
        DRFL_API bool _get_tool_digital_input(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex);
        DRFL_API bool _get_tool_digital_output(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex);
        // set digital ouput on control-box
        DRFL_API bool _set_digital_output(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        DRFL_API bool _get_digital_output(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex);
        // get digital input on control-box
        DRFL_API bool _get_digital_input(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex);

        // set analog ouput on control-box
        DRFL_API bool _set_analog_output(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue);
        // get analog inut on control-box
        DRFL_API float _get_analog_input(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex);

        // set analog input type on control-box
        DRFL_API bool _set_mode_analog_input(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT); 
        // set analog output type on control-box
        DRFL_API bool _set_mode_analog_output(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT); 

        DRFL_API float _get_tool_analog_input(LPROBOTCONTROL pCtrl, int nCh);
        DRFL_API bool _set_tool_digital_output_level(LPROBOTCONTROL pCtrl, int nLv);
        DRFL_API bool _set_tool_digital_output_type(LPROBOTCONTROL pCtrl, int nPort, OUTPUT_TYPE eOutputType);
        DRFL_API bool _set_mode_tool_analog_input(LPROBOTCONTROL pCtrl, int nCh, GPIO_ANALOG_TYPE eAnalogType);

        ////////////////////////////////////////////////////////////////////////////
        //  Modbus Operations                                                     //
		////////////////////////////////////////////////////////////////////////////
        // set modbus register 
        DRFL_API bool _set_modbus_output(LPROBOTCONTROL pCtrl, const char* lpszSymbol, unsigned short nValue);
        // get modbus register
        DRFL_API unsigned short _get_modbus_input(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // add modbus register
        DRFL_API bool _add_modbus_signal(LPROBOTCONTROL pCtrl, const char* lpszSymbol, const char* lpszIpAddress, unsigned short nPort, MODBUS_REGISTER_TYPE eRegType, unsigned short iRegIndex, unsigned short nRegValue = 0, unsigned char nSlaveId = 255);
        // del modbus register
        DRFL_API bool _del_modbus_signal(LPROBOTCONTROL pCtrl, const char* lpszSymbol);

        ////////////////////////////////////////////////////////////////////////////
        //  Flange Serial Operations                                              //
        ////////////////////////////////////////////////////////////////////////////
        DRFL_API bool _flange_serial_open(LPROBOTCONTROL pCtrl, int baudrate = 115200, BYTE_SIZE eByteSize = BYTE_SIZE_EIGHTBITS, PARITY_CHECK eParity = PARITY_CHECK_NONE, STOP_BITS eStopBits = STOPBITS_ONE);
        DRFL_API bool _flange_serial_close(LPROBOTCONTROL pCtrl);
        DRFL_API bool _flange_serial_write(LPROBOTCONTROL pCtrl, int nSize, char* pSendData, int nPort = 1);
        DRFL_API LPFLANGE_SER_RXD_INFO _flange_serial_read(LPROBOTCONTROL pCtrl, float fTimeout = -1, int nPort = 1);     
  
        ////////////////////////////////////////////////////////////////////////////
        //  Configuration Operations                                              //
        ////////////////////////////////////////////////////////////////////////////
        // set tool(end-effector) information
        DRFL_API bool _set_tool(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // add tool(end-effector) information
        DRFL_API bool _add_tool(LPROBOTCONTROL pCtrl, const char* lpszSymbol, float fWeight, float fCog[3], float fInertia[NUM_TASK]);
        // del tool(end-effector) informaiton
        DRFL_API bool _del_tool(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // get tool(end-effector) information
        DRFL_API const char* _get_tool(LPROBOTCONTROL pCtrl);

        // set robot tcp information
        DRFL_API bool _set_tcp(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // add robot tcp information
        DRFL_API bool _add_tcp(LPROBOTCONTROL pCtrl, const char* lpszSymbol, float fPostion[NUM_TASK]);
        // del robot tcp information
        DRFL_API bool _del_tcp(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        // get robot tcp information
        DRFL_API const char* _get_tcp(LPROBOTCONTROL pCtrl);  

        DRFL_API bool _set_tool_shape(LPROBOTCONTROL pCtrl, const char* lpszSymbol);
        DRFL_API const char* _get_tool_shape(LPROBOTCONTROL pCtrl);  
        DRFL_API bool _set_user_home(LPROBOTCONTROL pCtrl);

        DRFL_API int _servo_off(LPROBOTCONTROL pCtrl, STOP_TYPE eStopType);      
        DRFL_API int _check_motion(LPROBOTCONTROL pCtrl);
        DRFL_API bool _release_protective_stop(LPROBOTCONTROL pCtrl, RELEASE_MODE eReleaseMode);
        DRFL_API bool _set_safety_mode(LPROBOTCONTROL pCtrl, SAFETY_MODE eSafetyMode, SAFETY_MODE_EVENT eSafetyEvent);
        DRFL_API bool _set_auto_servo_off(LPROBOTCONTROL pCtrl, bool bFuncEnable, float fElapseTime);

        DRFL_API LPSAFETY_CONFIGURATION_EX _get_safety_configuration(LPROBOTCONTROL pCtrl);

        DRFL_API bool _change_collision_sensitivity(LPROBOTCONTROL pCtrl, float fSensitivity);
        ////////////////////////////////////////////////////////////////////////////
        //  drl program Operations                                                //
        ////////////////////////////////////////////////////////////////////////////
        // program start
        DRFL_API bool _drl_start(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem, const char* lpszDrlProgram);
        // program stop
        DRFL_API bool _drl_stop(LPROBOTCONTROL pCtrl, unsigned char eStopType = 0);
        // program Pause
        DRFL_API bool _drl_pause(LPROBOTCONTROL pCtrl);
        // program Resume
        DRFL_API bool _drl_resume(LPROBOTCONTROL pCtrl);
        // program speed
        DRFL_API bool _change_operation_speed(LPROBOTCONTROL pCtrl, float fSpeed);

        ////////////////////////////////////////////////////////////////////////////
        //  force control                                                        //
        ////////////////////////////////////////////////////////////////////////////

        DRFL_API bool _task_compliance_ctrl(LPROBOTCONTROL pCtrl, float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f);
        //DRFL_API bool _joint_compliance_ctrl(LPROBOTCONTROL pCtrl, float fTargetStiffness[NUM_TASK], float fTargetTime = 0.f);
        DRFL_API bool _set_stiffnessx(LPROBOTCONTROL pCtrl, float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f);
        DRFL_API bool _release_compliance_ctrl(LPROBOTCONTROL pCtrl);
        //DRFL_API bool _release_joint_compliance_ctrl(LPROBOTCONTROL pCtrl);
        DRFL_API bool _set_desired_force(LPROBOTCONTROL pCtrl, float fTargetForce[NUM_TASK], unsigned char iTargetDirection[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f, FORCE_MODE eForceMode = FORCE_MODE_ABSOLUTE);
        DRFL_API bool _release_force(LPROBOTCONTROL pCtrl, float fTargetTime = 0.f);

        DRFL_API bool _check_force_condition(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _check_position_condition_abs(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _check_position_condition_rel(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _check_position_condition(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], MOVE_MODE eMode = MOVE_MODE_ABSOLUTE, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _check_orientation_condition_abs(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin[NUM_TASK], float fTargetMax[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _check_orientation_condition_rel(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
        DRFL_API bool _is_done_bolt_tightening(LPROBOTCONTROL pCtrl, FORCE_AXIS eForceAxis, float fTargetTor = 0.f, float fTimeout = 0.f);

        DRFL_API bool _parallel_axis1(LPROBOTCONTROL pCtrl, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);
        DRFL_API bool _align_axis1(LPROBOTCONTROL pCtrl, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);
        DRFL_API bool _parallel_axis2(LPROBOTCONTROL pCtrl, float fTargetVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);
        DRFL_API bool _align_axis2(LPROBOTCONTROL pCtrl, float fTargetVec[3], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE);

        ////////////////////////////////////////////////////////////////////////////
        //  coordinate system control                                             //
        ////////////////////////////////////////////////////////////////////////////

        DRFL_API int _set_user_cart_coord1(LPROBOTCONTROL pCtrl, int iReqId, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
        DRFL_API int _set_user_cart_coord2(LPROBOTCONTROL pCtrl, float fTargetPos[3][NUM_TASK], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef);
        DRFL_API int _set_user_cart_coord3(LPROBOTCONTROL pCtrl, float fTargetVec[2][3], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef);
        DRFL_API LPROBOT_POSE _coord_transform(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eInCoordSystem, COORDINATE_SYSTEM eOutCoordSystem);
        DRFL_API bool _set_ref_coord(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eTargetCoordSystem);
        DRFL_API LPROBOT_POSE _calc_coord(LPROBOTCONTROL pCtrl, unsigned short nCnt, unsigned short nInputMode, COORDINATE_SYSTEM eTargetRef, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fTargetPos4[NUM_TASK]);
        DRFL_API LPUSER_COORDINATE _get_user_cart_coord(LPROBOTCONTROL pCtrl, int iReqId);
        DRFL_API int _overwrite_user_cart_coord(LPROBOTCONTROL pCtrl, bool bTargetUpdate, int iReqId, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
        DRFL_API bool _enable_alter_motion(LPROBOTCONTROL pCtrl, int iCycleTime, PATH_MODE ePathMode, COORDINATE_SYSTEM eTargetRef, float fLimitDpos[2], float fLimitDposPer[2]);
        DRFL_API bool _disable_alter_motion(LPROBOTCONTROL pCtrl);
        DRFL_API bool _alter_motion(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK]);
        DRFL_API bool _set_singularity_handling(LPROBOTCONTROL pCtrl, SINGULARITY_AVOIDANCE eMode);
        DRFL_API bool _config_program_watch_variable(LPROBOTCONTROL pCtrl, VARIABLE_TYPE eDivision, DATA_TYPE eType, const char* szName, const char* szData);
        DRFL_API bool _save_sub_program(LPROBOTCONTROL pCtrl, int iTargetType, const char* szFileName, const char* lpszTextString);
        DRFL_API bool _setup_monitoring_version(LPROBOTCONTROL pCtrl, int iVersion);
        DRFL_API bool _system_shut_down(LPROBOTCONTROL pCtrl);

#if defined(_IS_PLATFORM)
        //brief(159) :Set Time 명령                                      
        DRFL_API bool _set_system_time(LPROBOTCONTROL pCtrl, const char* szData, const char* szTime);
        
		//brief(160) :IP 주소 설정                                      
        DRFL_API bool _set_ip_address(LPROBOTCONTROL pCtrl, unsigned char iUsage, unsigned char iIpType, const char* szHostIp, const char* szSubnet, const char* szGateway, const char* szPrimaryDNS, const char* szSecondaryDNS);
        //brief(161) :전원 관리                                      
        DRFL_API unsigned char _set_system_power(LPROBOTCONTROL pCtrl, unsigned char iTarget, unsigned char iPower);
        //brief(162) :IP주소 정보 요청                                      
        DRFL_API LPSYSTEM_IPADDRESS _get_ip_address(LPROBOTCONTROL pCtrl, unsigned char iUsage);
        //brief(163) :시간 정보 요청                                      
        DRFL_API LPSYSTEM_TIME _get_system_time(LPROBOTCONTROL pCtrl);
        //brief(164) :CPU 사용량 정보 요청                                      
        DRFL_API LPSYSTEM_CPUUSAGE _get_cpu_usage(LPROBOTCONTROL pCtrl);
        //brief(165) :디스크 용량 정보 요청                                      
        DRFL_API LPSYSTEM_DISKSIZE _get_disk_usage(LPROBOTCONTROL pCtrl);
        //brief(166) :F/W 업데이트
        DRFL_API LPSYSTEM_UPDATE_RESPONSE _update_system(LPROBOTCONTROL pCtrl, const char* szIpAddress, const char* szFileName, unsigned char iFileType, unsigned char iInverter[6], unsigned char iResetType);
        //brief(167) :모니터링 정보 전송 제어                                      
        DRFL_API bool _set_monitoring_control(LPROBOTCONTROL pCtrl, unsigned char iControl);
        //brief(168) :라이선스 정보 변경                                      
        DRFL_API bool _update_license(LPROBOTCONTROL pCtrl, const char* lpszLicense);
        //brief(169) : 라이센스 정보 확인
        DRFL_API LPLICENSE_TEXT_PARAM _get_license(LPROBOTCONTROL pCtrl);
        
		////brief(170) :자동 Safe-Off 설정 변경
  //      DRFL_API bool _set_auto_safe_off(LPROBOTCONTROL pCtrl, unsigned char bFuncEnable, float fElapseTime);
        //brief(171) :시간 동기화 설정 변경                                      
        DRFL_API bool _set_time_sync(LPROBOTCONTROL pCtrl, unsigned char iUsage);
        //brief(172) :시리얼 통신 포트 정보 요청                                      
        DRFL_API LPSERIAL_SEARCH _get_serial_port(LPROBOTCONTROL pCtrl);
        //brief(173) :OS 커널 로그 정보 요청                                      
        DRFL_API const char* _get_kernel_log(LPROBOTCONTROL pCtrl);
        //brief(174) :JTS 교정정보 저장 명령                                      
        DRFL_API bool _update_jts_data(LPROBOTCONTROL pCtrl, float fOffset[NUMBER_OF_JOINT], float fScale[NUMBER_OF_JOINT]);
        //brief(175) :서브 시스템 정보 요청                                      
        DRFL_API LPINSTALL_SUB_SYSTEM _get_sub_system(LPROBOTCONTROL pCtrl);
        //brief(176) :FTS 교정정보 저장 명령                                      
        DRFL_API bool _update_fts_data(LPROBOTCONTROL pCtrl, float fOffset[NUMBER_OF_JOINT]);
        //brief(177) :F/W 업데이트 상태 정보 설정                                      
        DRFL_API LPSYSTEM_UPDATE_RESPONSE _update_process(LPROBOTCONTROL pCtrl, unsigned char pProcessInfo, unsigned char pStatusInfo);
        //brief(178) :KT 5G 연동 정보 설정                                      
        DRFL_API bool _config_kt_factory_makers(LPROBOTCONTROL pCtrl, int bEnable, const char* szIpAddress, int nPort, const char* szDeviceId, const char* szDevicePw, const char* szGatewayId);
        //brief(179) :폴더/파일 삭제 명령                                      
        DRFL_API bool _delete_file(LPROBOTCONTROL pCtrl, const char* szFileName);

        //brief(180) :운용모드 변경                                      
        DRFL_API bool _set_teach_mode(LPROBOTCONTROL pCtrl, unsigned char bMode);
        //brief(181) :브레이크 제어                                      
        DRFL_API bool _control_brake(LPROBOTCONTROL pCtrl, unsigned char iTargetAxis, unsigned char bValue);
        //brief(182) :충돌 감시 모드 변경                                      
        //DRFL_API bool _enable_collision_mode(LPROBOTCONTROL pCtrl, unsigned char bMode);
        //brief(183) :벡선벡터 계산 및 응답                                      
        DRFL_API LPNORMAL_VECTOR_RESPONSE _get_normal(LPROBOTCONTROL pCtrl, float fTargetPosX[NUMBER_OF_JOINT], float fTargetPosY[NUMBER_OF_JOINT], float fTargetPosZ[NUMBER_OF_JOINT]);
        //brief(184) :안전 TCP 좌표정보 자동측정 명령                                      
        DRFL_API LPMEASURE_TCP_RESPONSE _get_tcp_coord(LPROBOTCONTROL pCtrl, unsigned char iTargetRef, float fTargetPos1[NUMBER_OF_JOINT], float fTargetPos2[NUMBER_OF_JOINT], float fTargetPos3[NUMBER_OF_JOINT], float fTargetPos4[NUMBER_OF_JOINT]);
        //brief(185) : jts value 자동 측정
        DRFL_API LPCALIBRATE_JTS_RESPONSE _get_jts_value(LPROBOTCONTROL pCtrl);
        //brief(186) :안전 로봇설치 자세정보 자동측정 명령                                      
        DRFL_API LPCONFIG_INSTALL_POSE _get_install_pose(LPROBOTCONTROL pCtrl);
        //brief(187) :안전 FTS 교정정보 자동측정 명령                                      
        DRFL_API LPCALIBRATE_FTS_RESPONSE _get_fts_value(LPROBOTCONTROL pCtrl);
        //brief(188) :다중 Digital I/O 출력 설정                                      
        DRFL_API bool _set_digital_outputs(LPROBOTCONTROL pCtrl, unsigned char iLocation, unsigned short iCount, GPIO_PORT tPort[MAX_DIGITAL_BURST_SIZE]);
        //brief(189) :다중 Modbus I/O 출력 설정                                      
        DRFL_API bool _set_modbus_outputs(LPROBOTCONTROL pCtrl, MODBUS_REGISTER_BURST pModbusregisterburst);
        
		//brief(190) :모드버스 RTU 정의                                      
        DRFL_API bool _add_modbus_rtu_signal(LPROBOTCONTROL pCtrl, WRITE_MODBUS_RTU_DATA pWritemodbusrtudata);
        //brief(191) :모드버스 RTU FC15/16 I/O 추가                                      
        DRFL_API bool _add_modbus_rtu_multi_signal(LPROBOTCONTROL pCtrl, WRITE_MODBUS_RTU_MULTI_DATA pWritemodbusrtumultidata);
        //brief(192) :모드버스 FC15/16 I/O 삭제                                      
        DRFL_API bool _del_modbus_multi_signal(LPROBOTCONTROL pCtrl, const char* szSymbol);
        //brief(193) :모드버스 FC15/16 I/O 출력                                      
        DRFL_API bool _set_modbus_multi_output(LPROBOTCONTROL pCtrl, const char* szSymbol, unsigned char iRegCount, unsigned short iRegValue[MAX_MODBUS_REGISTER_PER_DEVICE]);
        //brief(194) :모드버스 RTU FC15/16 I/O 추가                                      
        /*DRFL_API bool _add_modbus_rtu_multi_signal2(LPROBOTCONTROL pCtrl, WRITE_MODBUS_RTU_MULTI_DATA pWritemodbusrtumultidata);*/

        ////brief(196) :Tool Shape 정보                                      
        DRFL_API bool _add_tool_shape(LPROBOTCONTROL pCtrl, CONFIG_TOOL_SHAPE_SYMBOL pConfigtoolshapesymbol);
        //brief(197) :del_tool_shape
        DRFL_API bool _del_tool_shape(LPROBOTCONTROL pCtrl, const char* pcArg);
        //brief(199) :모션 진행 상태 확인#2                                      
        DRFL_API int _check_motion_ex(LPROBOTCONTROL pCtrl);
        
		//brief(200) :Collaborative Zone 감속률 변경                                       
        DRFL_API int _change_collaborative_speed(LPROBOTCONTROL pCtrl, float fSpeed);
        //brief(201) :Safety Input Reduced 모드 감속률 변경                                       
        DRFL_API bool _change_reduced_speed(LPROBOTCONTROL pCtrl, float fSpeed);
        //brief(202) :로봇 설치 자세 정보                                      
        DRFL_API bool _set_install_pose(LPROBOTCONTROL pCtrl, float fGradient, float fRotation);
        //brief(203) :일반 제한(Safety) 정보                                      
        DRFL_API bool _set_general_range(LPROBOTCONTROL pCtrl, GENERAL_RANGE tNormal, GENERAL_RANGE tReduced);
        //brief(204) :조인트 제한(Safety) 정보                                      
        DRFL_API bool _set_joint_range(LPROBOTCONTROL pCtrl, JOINT_RANGE tNormal, JOINT_RANGE tReduced);

        //brief(213) :통합 안전 영역 정의(추가)                                      
        DRFL_API bool _add_safety_zone(LPROBOTCONTROL pCtrl, const char* szIdentifier, const char* szAlias, unsigned char iZoneType, SAFETY_ZONE_PROPERTY_DATA tZoneProperty, SAFETY_ZONE_SHAPE tShape);
        //brief(214) :통합 안전 영역 삭제                                      
        DRFL_API bool _del_safety_zone(LPROBOTCONTROL pCtrl, const char* szIdentifier);
        //brief(215) :충돌 민감도 정보                                      
        DRFL_API bool _change_collision_sensitivity(LPROBOTCONTROL pCtrl, float pfArg);
        //brief(216) :직접 교시 센서 정보                                      
        DRFL_API bool _set_teach_sensor(LPROBOTCONTROL pCtrl, unsigned char iSensorType);
        //brief(217) :안전 I/O 설정 정보                                      
        DRFL_API bool _set_safety_io(LPROBOTCONTROL pCtrl, unsigned char iIO[TYPE_LAST][NUM_SAFETY]);
        //brief(218) :Cockpit 교시버튼 정보                                      
        DRFL_API bool _set_cockpit(LPROBOTCONTROL pCtrl, unsigned char bEnable, unsigned char iButton[2], unsigned char bRecoveryTeach);
        //brief(219) :External Encoder 극성 설정 정보                                      
        DRFL_API bool _set_encorder_polarity(LPROBOTCONTROL pCtrl, unsigned char iChannel, unsigned char iPolarity[ENCORDER_POLARITY_LAST]);
        
		//brief(220) :External Encoder 모드 설정 정보                                      
        DRFL_API bool _set_encorder_mode(LPROBOTCONTROL pCtrl, unsigned char iChannel, unsigned char iABMode, unsigned char iZMode, unsigned char iSMode, unsigned char iInvMode, unsigned int  nPulseAZ);
        //brief(221) :External Encoder 리셋 명령                                      
        DRFL_API bool _reset_encorder(LPROBOTCONTROL pCtrl, unsigned char iChannel);
        //brief(222) :Nudge 설정 정보                                      
        DRFL_API bool _set_nudge(LPROBOTCONTROL pCtrl, unsigned char bEnable, float fInputForce, float fDelayTime);
        //brief(223) :원격 제어 모드 설정 정보                                      
        DRFL_API bool _set_remote_control(LPROBOTCONTROL pCtrl, unsigned char bEnable, CONFIG_IO_FUNC tFunc[TYPE_LAST][NUM_REMOTE_CONTROL]);
        //brief(224) :월드 좌표계 설정                                      
        DRFL_API bool _set_world_coord(LPROBOTCONTROL pCtrl, unsigned char iType, float fPosition[NUMBER_OF_JOINT]);
        //brief(225) :Safety 파라미터 리셋                                      
        DRFL_API bool _reset_safety_config(LPROBOTCONTROL pCtrl);
        //brief(226) :Process Button제어모드 설정 정보                                      
        DRFL_API bool _set_process_button(LPROBOTCONTROL pCtrl, unsigned char iUsage);
        //brief(227) :안전 순서번호 리셋 명령                                      
        DRFL_API bool _reset_sequence(LPROBOTCONTROL pCtrl, unsigned char iIdentifier);
        //brief(228) :drl_save_main
        DRFL_API bool _drl_save_main(LPROBOTCONTROL pCtrl, unsigned char bMode, unsigned int uLength, const char* lpszString);
        //brief(229) :drl_start_main
        DRFL_API bool _drl_start_main(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem);
        
		//brief(230) :중단점(Break) 설정 명령                                      
        DRFL_API bool _drl_break(LPROBOTCONTROL pCtrl, int nLineNum);
        //brief(231) :라인별 실행 명령                                      
        DRFL_API bool _drl_step_run(LPROBOTCONTROL pCtrl);
        //brief(232) :문법 검사 명령                                      
        DRFL_API bool _drl_check_syntax(LPROBOTCONTROL pCtrl, unsigned int iTextLength, const char* lpszString);
        //brief(233) :라인별 실행 명령 확장                                      
        DRFL_API bool _drl_step_run2(LPROBOTCONTROL pCtrl, unsigned int nLine);
        //brief(234) :중단점(Break) 설정 명령 확장                                      
        DRFL_API bool _drl_break2(LPROBOTCONTROL pCtrl, int nLine, const char* szFile);
        //brief(235) :관절 공간 순응 제어 모드 설정                                      
        //DRFL_API bool _joint_compliance_ctrl2(LPROBOTCONTROL pCtrl, JOINT_COMPLIANCE_CONTROL pJointcompliancecontrol);
        ////brief(236) :관절 공간 강성(Stiffness) 설정                                      
        //DRFL_API bool _set_stiffnessj(LPROBOTCONTROL pCtrl, float fTargetStf[NUMBER_OF_JOINT], float fTargetTIme);
        //brief(237) :관절 공간 순응 제어 모드 해제                                      
        //DRFL_API bool _release_joint_compliance_ctrl(LPROBOTCONTROL pCtrl, JOINT_COMPLIANCE_CONTROL pJointcompliancecontrol);
        //brief(238) :축 고정(Constrained Axis) 명령                                      
        DRFL_API bool _constrained_axis(LPROBOTCONTROL pCtrl, unsigned char iTargetAxis[NUMBER_OF_JOINT], unsigned char iTargetRef, float fTargetStf, float fTargetDump);
        //brief(239) :사용자 좌표계 벡터 정보 확인                                      
        DRFL_API LPUSER_COORDINATE_MATRIX_RESPONSE _get_user_cart_coord_matrix(LPROBOTCONTROL pCtrl, unsigned int iTargetRef);
        
		////brief(240) :NT 좌표계 설정                                      
  //      DRFL_API int _set_user_nt_coord1(LPROBOTCONTROL pCtrl, unsigned char iTargetRef, float fTargetPosX[6], float fTargetPosY[6], float fTargetPosZ[6]);
  //      //brief(241) :NT 좌표계 설정(벡터버전)                                      
  //      DRFL_API int _set_user_nt_coord2(LPROBOTCONTROL pCtrl, unsigned char iTargetRef, float fTargetPos[3]);
        //brief(242) :마찰계수(Friction) 정보 보상/저장                                       
        DRFL_API bool _enable_friction_compensation(LPROBOTCONTROL pCtrl, unsigned char iSelect[NUMBER_OF_JOINT], float fPositive[4][NUMBER_OF_JOINT], float fNegative[4][NUMBER_OF_JOINT], float fTemperature[NUMBER_OF_JOINT]);
 
        //brief(244) :컨베이어 연동 환경 설정                                      
        DRFL_API bool _calc_conveyor_param(LPROBOTCONTROL pCtrl, unsigned char iType, unsigned char iEncoderChannel, unsigned char iTriggerChannel, unsigned char iTriggerEdgeType, float fTriggerMuteTime, char iChannel[2], unsigned char iValue[2]);
		//brief(245) :컨베이어 좌표계 정보 설정 
        DRFL_API bool _calc_conveyor_param2(LPROBOTCONTROL pCtrl, int iDistance2Count, POSITION tPosConCoord, unsigned char iTargetRef);
        //brief(246) :컨베이어 좌표계 정보 측정                                      
        DRFL_API LPMEASURE_CONVEYOR_COORD_RESPONSE _get_conveyor_coord(LPROBOTCONTROL pCtrl, POSITION tPosTeachPointQ, unsigned char nTeachCount, POSITION tPosTeachPointP[5], unsigned int EncoderCount[5]);
        //brief(247) :컨베이어 좌표계 정보 측정 확장                                      
        DRFL_API LPMEASURE_CONVEYOR_COORD_RESPONSE _get_conveyor_coord2(LPROBOTCONTROL pCtrl, int nResolution);
        //brief(248) :컨베이어 거리 정보 측정                                      
        DRFL_API LPMEASURE_CONVEYOR_DISTANCE_RESPONSE _get_conveyor_distance(LPROBOTCONTROL pCtrl, unsigned int nFilterSize);
        //brief(249) :컨베이어 거리 정보 측정 확장                                      
        DRFL_API LPMEASURE_CONVEYOR_DISTANCE_RESPONSE _get_conveyor_distance_ex(LPROBOTCONTROL pCtrl, float fSpeed);
        
		//brief(250) :컨베이어 정보 추가                                      
        DRFL_API bool _add_conveyor(LPROBOTCONTROL pCtrl, const char* szName, CONVEYOR_BASIC tBASIC, CONVEYOR_COORD_EX tCoord, CONVEYOR_DISTANCE tDistance);
        //brief(251) :컨베이어 정보 삭제                                      
        DRFL_API bool _del_conveyor(LPROBOTCONTROL pCtrl, const char* szName);
        //brief(252) :컨베이어 모니터링 설정                                      
        DRFL_API bool _set_conveyor_monitoring(LPROBOTCONTROL pCtrl, const char* szName, unsigned char bStart);
        //brief(253) :현재 컨베이어 정보 설정                                      
        DRFL_API unsigned char _set_conveyor(LPROBOTCONTROL pCtrl, const char* szName);
        //brief(254) :현재 컨베이어 정보 설정 확장                                      
        DRFL_API unsigned char _set_conveyor_ex(LPROBOTCONTROL pCtrl, const char* szName, CONVEYOR_BASIC tBASIC, CONVEYOR_COORD_EX tCoord, CONVEYOR_DISTANCE tDistance);
        //brief(255) :컨베이어 작업물 정보 확인 요청                                      
        DRFL_API int _get_conveyor_object(LPROBOTCONTROL pCtrl, unsigned char iConId, float fTimeout, unsigned char iContainerType, POSITION tPosObjCoord);
        //brief(256) :컨베이어 트래킹 제어                                      
        DRFL_API bool _set_conveyor_track(LPROBOTCONTROL pCtrl, unsigned char iConId, unsigned char bTracking, unsigned char bMate, float fDuration, float fDummy[5]);
        //brief(270) :app_weld_weave_cond_trapezoidal
        DRFL_API bool _app_weld_weave_cond_trapezoidal(LPROBOTCONTROL pCtrl, CONFIG_TRAPEZOID_WEAVING_SETTING pConfigtrapezoidweavingsetting);
        //brief(271) :app_weld_weave_cond_zigzag
        DRFL_API bool _app_weld_weave_cond_zigzag(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle);
        //brief(272) :app_weld_weave_cond_circular
        DRFL_API bool _app_weld_weave_cond_circular(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ, float fGradient, float fwWdt[2], float fwT[2]);
        //brief(273) :app_weld_weave_cond_sinusoidal
        DRFL_API bool _app_weld_weave_cond_sinusoidal(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle);
        //brief(274) :app_weld_enable_analog
        DRFL_API bool _app_weld_enable_analog(LPROBOTCONTROL pCtrl, CONFIG_ANALOG_WELDING_INTERFACE pConfiganalogweldinginterface);
        //brief(275) :app_weld_set_weld_cond_analog
        DRFL_API bool _app_weld_set_weld_cond_analog(LPROBOTCONTROL pCtrl, unsigned char iVirtualWelding, float fTargetVoltage, float fTargetCurrent, float fTargetVel, float fMinVel, float fMaxVel, float fTargetFeedingSpeed);
        //brief(276) :app_weld_adj_welding_cond_analog
        DRFL_API bool _app_weld_adj_welding_cond_analog(LPROBOTCONTROL pCtrl, unsigned char bRealTime, unsigned char bResetFlag, float fTargetVol, float fFeedingVel, float fTargetVel, float fOffsetY, float fOffsetZ, float fWidthRate);
        //brief(277) :app_weld_set_interface_eip_r2m_process
        DRFL_API bool _app_weld_set_interface_eip_r2m_process(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS pConfigdigitalweldinginterfaceprocess);
        //brief(278) :app_weld_set_interface_eip_r2m_mode
        DRFL_API bool _app_weld_set_interface_eip_r2m_mode(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_MODE pConfigdigitalweldinginterfacemode);
        //brief(279) :app_weld_set_interface_eip_r2m_test
        DRFL_API bool _app_weld_set_interface_eip_r2m_test(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_TEST pConfigdigitalweldinginterfacetest);
        
        //brief(280) :app_weld_set_interface_eip_r2m_condition
        DRFL_API bool _app_weld_set_interface_eip_r2m_condition(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_CONDITION pConfigdigitalweldinginterfacecondition);
        //brief(281) :app_weld_set_interface_eip_r2m_option
        DRFL_API bool _app_weld_set_interface_eip_r2m_option(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_OPTION pConfigdigitalweldinginterfaceoption);
        //brief(282) :app_weld_set_interface_eip_m2r_process2
        DRFL_API bool _app_weld_set_interface_eip_m2r_process2(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS2 pConfigdigitalweldinginterfaceprocess2);
        //brief(283) :app_weld_set_interface_eip_m2r_monitoring
        DRFL_API bool _app_weld_set_interface_eip_m2r_monitoring(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_MONITORING pConfigdigitalweldinginterfacemonitoring);
        //brief(284) :app_weld_set_interface_eip_m2r_other
        DRFL_API bool _app_weld_set_interface_eip_m2r_other(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_INTERFACE_OTHER pConfigdigitalweldinginterfacemonitoring);
        //brief(285) :app_weld_reset_interface
        DRFL_API bool _app_weld_reset_interface(LPROBOTCONTROL pCtrl, unsigned char bReset);
        //brief(286) :app_weld_enable_digital
        DRFL_API bool _app_weld_enable_digital(LPROBOTCONTROL pCtrl, unsigned char bMode);
        //brief(286) :app_weld_disable_digital
        DRFL_API bool _app_weld_disable_digital(LPROBOTCONTROL pCtrl, unsigned char bMode);
        //brief(287) :app_weld_set_weld_cond_digital
        DRFL_API bool _app_weld_set_weld_cond_digital(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_CONDITION pConfigdigitalweldingcondition);
        //brief(288) :app_weld_adj_welding_cond_digital
        DRFL_API bool _app_weld_adj_welding_cond_digital(LPROBOTCONTROL pCtrl, CONFIG_DIGITAL_WELDING_ADJUST pConfigdigitalweldingadjust);
        //brief(289) :measure_welding_tcp
        DRFL_API bool _measure_welding_tcp(LPROBOTCONTROL pCtrl, unsigned char iMode, float fStickout, float fTargetPos[9][NUMBER_OF_JOINT]);
        
        //brief(290) :set_welding_cockpit_setting
        DRFL_API bool _set_welding_cockpit_setting(LPROBOTCONTROL pCtrl, unsigned char bEnable, unsigned char bWeldingType);
        //brief(291) :set_digital_welding_signal_output
        DRFL_API bool _set_digital_welding_signal_output(LPROBOTCONTROL pCtrl, unsigned char cDataType, float fData);
        //brief(292) :set_digital_welding_monitoring_mode
        DRFL_API bool _set_digital_welding_monitoring_mode(LPROBOTCONTROL pCtrl, unsigned char bEnable);
        //brief(293) :app_weld_adj_motion_offset
        DRFL_API bool _app_weld_adj_motion_offset(LPROBOTCONTROL pCtrl, float fOffsetY, float fOffsetZ);
        //brief(294) :set_output_register_bit
        DRFL_API bool _set_output_register_bit(LPROBOTCONTROL pCtrl, unsigned char iGprType, unsigned char iGprAddr, const char szData[128]);
        //brief(295) :get_input_register_bit
        DRFL_API LPIETHERNET_SLAVE_RESPONSE_DATA_EX _get_input_register_bit(LPROBOTCONTROL pCtrl, unsigned char iGprType, unsigned char iGprAddr, unsigned char iInOut);
        //brief(296) :focas_get_error_str
        DRFL_API LPMACHINE_TENDING_FOCAS_ERR_STRING _focas_get_error_str(LPROBOTCONTROL pCtrl, unsigned short hHandle, short ErrorCode);
        //brief(297) :focas_connect
        DRFL_API LPMACHINE_TENDING_FOCAS_CONNECT _focas_connect(LPROBOTCONTROL pCtrl, short ErrorCode, const char szIpAddr[16], unsigned short iPort, unsigned short hHandle, float fTimeOut);
        //brief(298) :focas_disconnect
        DRFL_API LPMACHINE_TENDING_FOCAS_DISCONNECT _focas_disconnect(LPROBOTCONTROL pCtrl, short ErrorCode, unsigned short hHandle);
        //brief(299) :focas_pmc_read_bit
        DRFL_API LPMACHINE_TENDING_FOCAS_PMC _focas_pmc_read_bit(LPROBOTCONTROL pCtrl, short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset);
        //brief(300) :focas_pmc_write_bit
        DRFL_API LPMACHINE_TENDING_FOCAS_PMC _focas_pmc_write_bit(LPROBOTCONTROL pCtrl, short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset, MACHINE_TENDING_FOCAS_PMC_DATA tData);
        //brief(301) :focas_cnc_read_param
        DRFL_API LPMACHINE_TENDING_RESPONSE_FOCAS_CNC_PARAM _focas_cnc_read_param(LPROBOTCONTROL pCtrl, unsigned short hHandle, short iParamNumber, short iAxisNumber, short iDataLength);
        //brief(302) :focas_program_num
        DRFL_API LPMACHINE_TENDING_RESPONSE_FOCAS_PROGRAM_NUMBER _focas_program_num(LPROBOTCONTROL pCtrl, unsigned short nHandle);
        //brief(303) :is_focas_alive
        DRFL_API LPFOCAS_IS_ALIVE_RESPONSE _is_focas_alive(LPROBOTCONTROL pCtrl, unsigned short nHandle);
		//brief(304) : config_setting_enable
		DRFL_API bool _config_setting_enable(LPROBOTCONTROL pCtrl, unsigned short wPreviousCmdid, unsigned int iRefCrc32);
		//brief(305) : config_configurable_io
		DRFL_API bool _config_configurable_io(LPROBOTCONTROL pCtrl, unsigned char niIO[TYPE_LAST][NUM_DIGITAL]);
		//brief(306) : safe_move_home
		DRFL_API bool _safe_move_home(LPROBOTCONTROL pCtrl, unsigned char bRun);
		//brief(307) : safe_movej
		DRFL_API bool _safe_movej(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        //brief(309) :safe_drl_start
		DRFL_API bool _safe_drl_start(LPROBOTCONTROL pCtrl, ROBOT_SYSTEM eRobotSystem);
		
        //brief(314) : get_tool_weight
		DRFL_API LPMEASURE_TOOL_RESPONSE _get_tool_weight(LPROBOTCONTROL pCtrl, unsigned char iMode, float fWeight, float fXYZ_X, float fXYZ_Y, float fXYZ_Z );
		//brief(317) : get_friction_value
		DRFL_API LPMEASURE_FRICTION_RESPONSE _get_friction_value(LPROBOTCONTROL pCtrl,unsigned char iType,unsigned char iSelect[NUMBER_OF_JOINT], float fStart[NUMBER_OF_JOINT], float fRange[NUMBER_OF_JOINT]);
		//brief(320) : safe_movel
        DRFL_API bool _safe_movel(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
        //brief(323) : safe_move_userhome
		DRFL_API bool _safe_move_userhome(LPROBOTCONTROL pCtrl, unsigned char bRun);
		//brief(324) : safe_movejx
        DRFL_API bool _safe_movejx(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);
		//brief(327) : hold2run 
		DRFL_API bool _hold2run(LPROBOTCONTROL pCtrl);
		//brief(329) :update_gravity_param
        DRFL_API bool _update_gravity_param(LPROBOTCONTROL pCtrl, COUNTER_BALANCE_PARAM_DATA tClParam, CALIBRATION_PARAM_DATA tCrParam);
        
        //brief(330) :operation_industrial_ethernet
        DRFL_API bool _operation_industrial_ethernet(LPROBOTCONTROL pCtrl, unsigned char nEtherNetIPOpMode, CONFIG_INDUSTRIAL_ETHERNET tConfig);
        //brief(331) :report_tcp_client
        DRFL_API LPREPORT_TCP_CLIENT _report_tcp_client(LPROBOTCONTROL pCtrl, unsigned char iIdentifier);
        //brief(332) :get_sysversion_ex
        DRFL_API LPSYSTEM_VERSION_EX _get_sysversion_ex(LPROBOTCONTROL pCtrl);
        //brief(333) :update_local_package
        DRFL_API bool _update_local_package(LPROBOTCONTROL pCtrl, unsigned char iTarget[UPDATE_TARGET_LAST], const char szDirName[MAX_SYMBOL_SIZE]);
        //brief(334) :update_network_package
        DRFL_API bool _update_network_package(LPROBOTCONTROL pCtrl, unsigned char iTarget[UPDATE_TARGET_LAST], unsigned char iNetType, const char szIpAddress[16], const char szFileName[MAX_SYMBOL_SIZE]);
        //brief(335) :update_local_svm
        DRFL_API bool _update_local_svm(LPROBOTCONTROL pCtrl, const char szFileName[MAX_SYMBOL_SIZE]);
        //brief(336) :update_network_svm
        DRFL_API bool _update_network_svm(LPROBOTCONTROL pCtrl, unsigned char iNetType, const char szIpAddress[16], const char szFileName[MAX_SYMBOL_SIZE]);
        //brief(337) :set_restore_list
        DRFL_API LPPACKAGE_RESTORE_LIST _set_restore_list(LPROBOTCONTROL pCtrl);
        //brief(338) :set_package_restore
        DRFL_API bool _set_package_restore(LPROBOTCONTROL pCtrl, const char szVersName[MAX_SYMBOL_SIZE]);
        //brief(339) :set_unzip
        //DRFL_API bool _set_unzip(LPROBOTCONTROL pCtrl, unsigned char iResult, const char szDirName[MAX_SYMBOL_SIZE]);
        
        //brief(340) :update_excute_package
        DRFL_API bool _update_excute_package(LPROBOTCONTROL pCtrl, unsigned char iExecute);
        //brief(341) :response_restore_controller
        DRFL_API bool _response_restore_controller(LPROBOTCONTROL pCtrl, unsigned char iProcess);
        //brief(342) :set_license_option
        //DRFL_API bool _set_license_option(LPROBOTCONTROL pCtrl, unsigned int nOption);
        //brief(343) :set_robot_control_mode
        DRFL_API unsigned char _set_robot_control_mode(LPROBOTCONTROL pCtrl, unsigned char nMode);
        //brief(344) :get_robot_control_mode
        DRFL_API unsigned char _get_robot_control_mode(LPROBOTCONTROL pCtrl);
        //brief(345) :get_pattern_point
        DRFL_API LPRESPONSE_TRANS_PALLET_POS _get_pattern_point(LPROBOTCONTROL pCtrl, CONTROL_TRANS_PALLET_POS pControltranspalletpos);
        //brief(346) :set_oscillation_check
        DRFL_API bool _set_oscillation_check(LPROBOTCONTROL pCtrl, unsigned int puArg);
        //brief(347) :drl_nudge
        DRFL_API bool _drl_nudge(LPROBOTCONTROL pCtrl);
        //brief(348) :get_user_coord_external_force
        DRFL_API bool _get_user_coord_external_force(LPROBOTCONTROL pCtrl, unsigned char bIsMonitoring, unsigned char iUserID[MAX_USER_COORD_MONITORING_EXT_FORCE_SIZE]);
        //brief(349) :set_deflection_comp_mode
        DRFL_API bool _set_deflection_comp_mode(LPROBOTCONTROL pCtrl, unsigned char bEnable);
        
        //brief(350) :set_ie_monitoring
        DRFL_API bool _set_ie_monitoring(LPROBOTCONTROL pCtrl, unsigned char bStart);
        //brief(351) :drl_teach_mode
        DRFL_API unsigned char _drl_teach_mode(LPROBOTCONTROL pCtrl, bool bOnOff);
        //brief(352) :query_modbus_data_list
        DRFL_API LPMODBUS_DATA_LIST _query_modbus_data_list(LPROBOTCONTROL pCtrl);
        //brief(354) : set_on_monitoring_power_button
        DRFL_API void _set_on_monitoring_power_button(LPROBOTCONTROL pCtrl, TOnMonitoringPowerButtonCB pCallbackFunc);
        //brief(355) : set_on_monitoring_safety_state
        //DRFL_API void set_on_monitoring_safety_state(LPROBOTCONTROL pCtrl);
        //brief(356) : set_on_monitoring_collision_sensitivity
        DRFL_API void _set_on_monitoring_collision_sensitivity(LPROBOTCONTROL pCtrl, TOnMonitoringCollisionSensitivityCB pCallbackFunc);
        //brief(357) : set_on_monitoring_system 
        //DRFL_API void set_on_monitoring_system(LPROBOTCONTROL pCtrl);
        //brief(358) : set_on_monitoring_welding
        DRFL_API void _set_on_monitoring_welding(LPROBOTCONTROL pCtrl, TOnMonitoringWeldingCB pCallbackFunc);
        
        //brief(360) : set_on_monitoring_mode
        DRFL_API void _set_on_monitoring_mode(LPROBOTCONTROL pCtrl, TOnMonitoringModeCB pCallbackFunc);
        //brief(361) : set_on_monitoring_analog_welding
        DRFL_API void _set_on_monitoring_analog_welding(LPROBOTCONTROL pCtrl, TOnMonitoringAnalogWeldingCB pCallbackFunc);
        //brief(362) : set_on_monitoring_digital_welding
        DRFL_API void _set_on_monitoring_digital_welding(LPROBOTCONTROL pCtrl, TOnMonitoringDigitalWeldingCB pCallbackFunc);
        //brief(363) : set_on_monitoring_digital_welding_comm_state
        DRFL_API void _set_on_monitoring_digital_welding_comm_state(LPROBOTCONTROL pCtrl, TOnMonitoringDigitalWeldingCommStateCB pCallbackFunc);
        //brief(364) : set_on_monitoring_user_coord_ext_force
        DRFL_API void _set_on_monitoring_user_coord_ext_force(LPROBOTCONTROL pCtrl, TOnMonitoringUserCoordExtForceCB pCallbackFunc);
        //brief(366) : set_on_monitoring_ie_slave
        DRFL_API void _set_on_monitoring_ie_slave(LPROBOTCONTROL pCtrl, TOnMonitoringIeSlaveCB pCallbackFunc);
        //brief(367) : set_on_monitoring_response_command
        DRFL_API void _set_on_monitoring_response_command(LPROBOTCONTROL pCtrl, TOnMonitoringResponseCommandCB pCallbackFunc);
        //brief(368) :query_operating_system
        DRFL_API ROBOT_SYSTEM _query_operating_system(LPROBOTCONTROL pCtrl);
        //brief(369) :query_operating_state
        DRFL_API ROBOT_STATE _query_operating_state(LPROBOTCONTROL pCtrl);
        
        //brief(371) :measure_tcp
        DRFL_API LPMEASURE_TCP_RESPONSE _measure_tcp(LPROBOTCONTROL pCtrl, unsigned char iTargetRef, float fTargetPos[4][NUMBER_OF_JOINT]);
        //brief(373) :measure_friction
        DRFL_API LPMEASURE_FRICTION_RESPONSE _measure_friction(LPROBOTCONTROL pCtrl, unsigned char iType, unsigned char iSelect[NUMBER_OF_JOINT], float fStart[NUMBER_OF_JOINT], float fRange[NUMBER_OF_JOINT]);

        //brief(375) :measure_install_pose
        DRFL_API bool _measure_install_pose(LPROBOTCONTROL pCtrl, float fGradient, float fRotation);
        //brief(376) :del_user_coord
        DRFL_API bool _del_user_coord(LPROBOTCONTROL pCtrl, unsigned char iIdentifier);
        //brief(377) :get_user_coord
        DRFL_API bool _get_user_coord(LPROBOTCONTROL pCtrl, unsigned char iIdentifier);
        //brief(378) :twait
        DRFL_API bool _twait(LPROBOTCONTROL pCtrl, float fTime);
        
        //brief(382) :add_modbus_multi_signal
        DRFL_API bool _add_modbus_multi_signal(LPROBOTCONTROL pCtrl, const char szSymbol[MAX_SYMBOL_SIZE], const char szIpAddr[16], unsigned short iPort, int iSlaveID, unsigned char iRegType, unsigned short iRegIndex, unsigned char iRegCount);
        //brief(383) :set_safety_function
        DRFL_API bool _set_safety_function(LPROBOTCONTROL pCtrl, unsigned char iStopCode[SAFETY_FUNC_LAST]);
        //brief(385) :simulatior_keyin
        DRFL_API bool _simulatior_keyin(LPROBOTCONTROL pCtrl, unsigned char iKey);

        DRFL_API bool _set_safe_stop_reset_type(LPROBOTCONTROL pCtrl, SAFE_STOP_RESET_TYPE eResetType);
        //brief(388) :set_user_cart_coord4
        DRFL_API int _set_user_cart_coord4(LPROBOTCONTROL pCtrl, COORDINATE_SYSTEM eTargetRef, float fTargetPos[3][3]);
        //brief(389) :set_palletizing_operation
        DRFL_API bool _set_palletizing_operation(LPROBOTCONTROL pCtrl, unsigned char iMode);
        
        //brief(390) :set_trajectory_end_mode
        DRFL_API bool _set_trajectory_end_mode(LPROBOTCONTROL pCtrl, unsigned char iMode);
        //brief(391) :setup_init_data_complete
        DRFL_API bool _setup_init_data_complete(LPROBOTCONTROL pCtrl);
        //brief(392) :set_system_config(deleted)
        //DRFL_API bool _set_system_config(LPROBOTCONTROL pCtrl);
        //brief(393) :get_flange_version
        DRFL_API LPFLANGE_VERSION _get_flange_version(LPROBOTCONTROL pCtrl);
        //brief(394) :set_workpiece_weight //complete
        //DRFL_API bool _set_workpiece_weight(LPROBOTCONTROL pCtrl);
        //brief(395) :wait_manual_guide_response
        DRFL_API bool _wait_manual_guide_response(LPROBOTCONTROL pCtrl);
		// brief(396) :  set_on_program_start
		DRFL_API void _set_on_program_start(LPROBOTCONTROL pCtrl, TOnProgramStartCB pCallbackFunc);
		//brief(397) :  set_on_program_watch_variable
		DRFL_API void _set_on_program_watch_variable(LPROBOTCONTROL pCtrl, TOnProgramWatchVariableCB pCallbackFunc);
		//brief(398) :  set_on_program_error
		DRFL_API void _set_on_program_error(LPROBOTCONTROL pCtrl, TOnProgramErrorCB pCallbackFunc);
		
        //brief(400) :  jog_h2r
		DRFL_API bool _jog_h2r(LPROBOTCONTROL pCtrl, JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity);
		//brief(401) :  safe_move_home_h2r
		DRFL_API bool _safe_move_home_h2r(LPROBOTCONTROL pCtrl, unsigned char bRun);
		//brief(402) :  safe_move_userhome_h2r
		DRFL_API bool _safe_move_userhome_h2r(LPROBOTCONTROL pCtrl, unsigned char bRun);
		//brief(403) :  safe_movej_h2r
		DRFL_API bool _safe_movej_h2r(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime, MOVE_MODE eMoveMode, float fBlendingRadius, BLENDING_SPEED_TYPE eBlendingType);
		//brief(404) :  safe_movel_h2r
		DRFL_API bool _safe_movel_h2r(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, MOVE_MODE eMoveMode, MOVE_REFERENCE eMoveReference, float fBlendingRadius, BLENDING_SPEED_TYPE eBlendingType);
		//brief(405) :  safe_movejx_h2r
		DRFL_API bool _safe_movejx_h2r(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime, MOVE_MODE eMoveMode, MOVE_REFERENCE eMoveReference, float fBlendingRadius, BLENDING_SPEED_TYPE eBlendingType);
		//brief(406) :  parallel_axis_h2r
		DRFL_API bool _parallel_axis_h2r(LPROBOTCONTROL pCtrl, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef /* = COORDINATE_SYSTEM_BASE*/);
		//brief(407) :  align_axis_h2r
		DRFL_API bool _align_axis_h2r(LPROBOTCONTROL pCtrl, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef /* = COORDINATE_SYSTEM_BASE*/);
		//brief(408) :  set_on_program_drl_line
		DRFL_API void _set_on_program_drl_line(LPROBOTCONTROL pCtrl, TOnProgramDrlLineExCB pCallbackFunc);
		//brief(409) :  set_on_program_state
		DRFL_API void _set_on_program_state(LPROBOTCONTROL pCtrl, TOnProgramStateCB pCallbackFunc);
        
        //brief(410) :  get_drl_program_state
        DRFL_API unsigned char _get_drl_program_state(LPROBOTCONTROL pCtrl);
		//brief(411) :  servo_on
		DRFL_API bool _servo_on(LPROBOTCONTROL pCtrl);
		//brief(412) :  set_config_user_home
		DRFL_API bool _set_config_user_home(LPROBOTCONTROL pCtrl, float fTargetPos[NUM_JOINT]);
		//brief(413) :  get_safety_mode
		DRFL_API SAFETY_MODE _get_safety_mode(LPROBOTCONTROL pCtrl);
		//brief(414) :  get_safety_state
		DRFL_API SAFETY_STATE _get_safety_state(LPROBOTCONTROL pCtrl);
		//brief(415) : _send_module_intallation
		DRFL_API bool _send_module_intallation(LPROBOTCONTROL pCtrl, int nInstallType, char *pModuleName);
		//brief(416) : _send_load_module
		DRFL_API bool _send_load_module(LPROBOTCONTROL pCtrl, char *pModuleName);
		//brief(417) : _send_unload_module
		DRFL_API bool _send_unload_module(LPROBOTCONTROL pCtrl, char *pModuleName);
		//brief(418) : _send_delete_module
		DRFL_API bool _send_delete_module(LPROBOTCONTROL pCtrl, char *pModuleName);
        //brief(419) : _set_safety_io_ex
        DRFL_API bool _set_safety_io_ex(LPROBOTCONTROL pCtrl, unsigned char iIO[TYPE_LAST][NUM_SAFETY * 2]);
        //brief(420) : _config_configurable_io_ex
        DRFL_API bool _config_configurable_io_ex(LPROBOTCONTROL pCtrl, unsigned char iIO[TYPE_LAST][NUM_DIGITAL * 2]);
        //brief(421) : _set_on_monitoring_ctrl_io_ex2
        DRFL_API void _set_on_monitoring_ctrl_io_ex2(LPROBOTCONTROL pCtrl, TOnMonitoringCtrlIOEx2CB pCallbackFunc);
        //brief(422) : _set_tool_digital_input
        DRFL_API bool _set_tool_digital_input(LPROBOTCONTROL pCtrl, GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        //brief(423) : _set_tool_analog_input
        DRFL_API bool _set_tool_analog_input(LPROBOTCONTROL pCtrl, GPIO_TOOL_ANALOG_INDEX eGpioIndex, float fValue);
        //brief(424) : _set_digital_input
        DRFL_API bool _set_digital_input(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff);
        //brief(425) : _set_analog_input
        DRFL_API bool _set_analog_input(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue);
        //brief(426) : _get_analog_output
        DRFL_API float _get_analog_output(LPROBOTCONTROL pCtrl, GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex);
        //brief(427) : _set_gpio_io_value
        DRFL_API bool _set_gpio_value(LPROBOTCONTROL pCtrl, unsigned char iLocation, unsigned char iType, unsigned char iSignal, unsigned char iIndex, float fValue);
        //brief(428) : _set_on_monitoring_protective_safe_off
        DRFL_API void _set_on_monitoring_protective_safe_off(LPROBOTCONTROL pCtrl, TOnMonitoringProtectiveSafeOffCB pCallbackFunc);
#endif

#ifdef __cplusplus
    };
#endif

#ifdef __cplusplus
    class CDRFLEx : public CDRFL
    {
    public:
        CDRFLEx() { _rbtCtrlUDP = _create_robot_control_udp(); }
        virtual ~CDRFLEx() { _destroy_robot_control_udp(_rbtCtrlUDP);   }
        // connection
        //for ROS org bool OpenConnection(string strIpAddr = "192.168.137.100") { return _OpenConnection(_rbtCtrl, strIpAddr.c_str()); };
        bool open_connection(string strIpAddr = "192.168.137.100", unsigned int usPort= 12345) { return _open_connection(_rbtCtrl, strIpAddr.c_str(), usPort); };
        bool close_connection() { return _close_connection(_rbtCtrl); };

        bool connect_rt_control(string strIpAddr = "192.168.137.100", unsigned int usPort= 12347) { return _connect_rt_control(_rbtCtrlUDP, strIpAddr.c_str(), usPort); };
        bool disconnect_rt_control() { return _disconnect_rt_control(_rbtCtrlUDP); };

        ////////////////////////////////////////////////////////////////////////////
        // RT Control                                                             //
        ////////////////////////////////////////////////////////////////////////////   

        string get_rt_control_output_version_list() { return _get_rt_control_output_version_list(_rbtCtrlUDP); };
        string get_rt_control_input_version_list() { return _get_rt_control_input_version_list(_rbtCtrlUDP); };
        string get_rt_control_input_data_list(string strVersion) { return _get_rt_control_input_data_list(_rbtCtrlUDP, strVersion.c_str()); };
        string get_rt_control_output_data_list(string strVersion) { return _get_rt_control_output_data_list(_rbtCtrlUDP, strVersion.c_str()); };
        bool set_rt_control_input(string strVersion, float fPeriod, int nLossCnt){ return _set_rt_control_input(_rbtCtrlUDP, strVersion.c_str(), fPeriod, nLossCnt); };
        bool set_rt_control_output(string strVersion, float fPeriod, int nLossCnt){ return _set_rt_control_output(_rbtCtrlUDP, strVersion.c_str(), fPeriod, nLossCnt); };

        bool start_rt_control(){ return _start_rt_control(_rbtCtrlUDP); };
        bool stop_rt_control(){ return _stop_rt_control(_rbtCtrlUDP); };
        
        bool set_velj_rt(float vel[NUM_JOINT]){ return _set_velj_rt(_rbtCtrlUDP, vel); };
        bool set_accj_rt(float acc[NUM_JOINT]){ return _set_accj_rt(_rbtCtrlUDP, acc); };
        bool set_velx_rt(float fTransVel, float fRotationVel = -10000){ return _set_velx_rt(_rbtCtrlUDP, fTransVel, fRotationVel); };
        bool set_accx_rt(float fTransAcc, float fRotationAcc = -10000){ return _set_accx_rt(_rbtCtrlUDP, fTransAcc, fRotationAcc); };

        LPRT_OUTPUT_DATA_LIST read_data_rt(){ return _read_data_rt(_rbtCtrlUDP); };
        bool write_data_rt(float fExternalForceTorque[NUM_JOINT], int iExternalDI, int iExternalDO, float fExternalAnalogInput[6], float fExternalAnalogOutput[6]){ return _write_data_rt(_rbtCtrlUDP, fExternalForceTorque, iExternalDI, iExternalDO, fExternalAnalogInput, fExternalAnalogOutput); };
        
        bool servoj_rt(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime){ return _servoj_rt(_rbtCtrlUDP, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };
        bool servol_rt(float fTargetPos[NUM_TASK], float fTargetVel[NUM_TASK], float fTargetAcc[NUM_TASK], float fTargetTime){ return _servol_rt(_rbtCtrlUDP, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };
        bool speedj_rt(float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime){ return _speedj_rt(_rbtCtrlUDP, fTargetVel, fTargetAcc, fTargetTime); };
        bool speedl_rt(float fTargetVel[NUM_TASK], float fTargetAcc[NUM_TASK], float fTargetTime){ return _speedl_rt(_rbtCtrlUDP, fTargetVel, fTargetAcc, fTargetTime); };
        bool torque_rt(float fMotorTor[NUM_JOINT], float fTargetTime){ return _torque_rt(_rbtCtrlUDP, fMotorTor, fTargetTime); };
        //bool change_operation_speed_rt(float fSpeedRate){ return _change_operation_speed_rt(_rbtCtrlUDP, fSpeedRate); }; //차후 개발


        ////////////////////////////////////////////////////////////////////////////
        // Callback operation                                                    //
        ////////////////////////////////////////////////////////////////////////////
        // robot status data
        void set_on_monitoring_state(TOnMonitoringStateCB pCallbackFunc) { _set_on_monitoring_state(_rbtCtrl, pCallbackFunc); };
        // robot operating data
        void set_on_monitoring_data(TOnMonitoringDataCB pCallbackFunc) { _set_on_monitoring_data(_rbtCtrl, pCallbackFunc); };
        // robot operating data : version 1
        void set_on_monitoring_data_ex(TOnMonitoringDataExCB pCallbackFunc) { _set_on_monitoring_data_ex(_rbtCtrl, pCallbackFunc); };
        // ctrl-box I/O data
        void set_on_monitoring_ctrl_io(TOnMonitoringCtrlIOCB pCallbackFunc) { _set_on_monitoring_ctrl_io(_rbtCtrl, pCallbackFunc); };
        // ctrl-box I/O data : version 1
        void set_on_monitoring_ctrl_io_ex(TOnMonitoringCtrlIOExCB pCallbackFunc) { _set_on_monitoring_ctrl_io_ex(_rbtCtrl, pCallbackFunc); };
        // modbus I/O data
        void set_on_monitoring_modbus(TOnMonitoringModbusCB pCallbackFunc) { _set_on_monitoring_modbus(_rbtCtrl, pCallbackFunc); };
        // robot speed mode event
        void set_on_monitoring_speed_mode(TOnMonitoringSpeedModeCB pCallbackFunc) { _set_on_monitoring_speed_mode(_rbtCtrl, pCallbackFunc); };
        // robot access control event
        void set_on_monitoring_access_control(TOnMonitoringAccessControlCB pCallbackFunc) { _set_on_monitoring_access_control(_rbtCtrl, pCallbackFunc); };
        // roobt alaram data
        void set_on_log_alarm(TOnLogAlarmCB pCallbackFunc)  { _set_on_log_alarm(_rbtCtrl, pCallbackFunc); };
        // tp popup message data
        void set_on_tp_popup(TOnTpPopupCB pCallbackFunc) { _set_on_tp_popup(_rbtCtrl, pCallbackFunc); };
        // tp log message data
        void set_on_tp_log(TOnTpLogCB pCallbackFunc) { _set_on_tp_log(_rbtCtrl, pCallbackFunc); };
        // tp progress message data
        void set_on_tp_progress(TOnTpProgressCB pCallbackFunc) { _set_on_tp_progress(_rbtCtrl, pCallbackFunc); };
        // tp user input message data
        void set_on_tp_get_user_input(TOnTpGetUserInputCB pCallbackFunc){ _set_on_tp_get_user_input(_rbtCtrl, pCallbackFunc); };
        // robot homing completed event
        void set_on_homming_completed(TOnHommingCompletedCB pCallbackFunc) { _set_on_homming_completed(_rbtCtrl, pCallbackFunc); };
        // Tp Initailzing completed
        void set_on_tp_initializing_completed(TOnTpInitializingCompletedCB pCallbackFunc) { _set_on_tp_initializing_completed(_rbtCtrl, pCallbackFunc); };
        // robot mastering needed event
        void set_on_mastering_need(TOnMasteringNeedCB pCallbackFunc) { _set_on_mastering_need(_rbtCtrl, pCallbackFunc); };
        // program stopeed event
        void set_on_program_stopped(TOnProgramStoppedCB pCallbackFunc) { _set_on_program_stopped(_rbtCtrl, pCallbackFunc); };
        // robot disconneted event
        void set_on_disconnected(TOnDisconnectedCB pCallbackFunc) { _set_on_disconnected(_rbtCtrl, pCallbackFunc); };    

        void set_on_monitoring_safety_state(TOnMonitoringSafetyStateCB pCallbackFunc) { _set_on_monitoring_safety_state(_rbtCtrl, pCallbackFunc); };
        void set_on_monitoring_robot_system(TOnMonitoringRobotSystemCB pCallbackFunc) { _set_on_monitoring_robot_system(_rbtCtrl, pCallbackFunc); };
        //void set_on_monitoring_update_module(TOnMonitoringUpdateModuleCB pCallbackFunc) { _set_on_monitoring_update_module(_rbtCtrl, pCallbackFunc); };
        void set_on_monitoring_safety_stop_type(TOnMonitoringSafetyStopTypeCB pCallbackFunc) { _set_on_monitoring_safety_stop_type(_rbtCtrl, pCallbackFunc); };

        void set_on_rt_monitoring_data(TOnRTMonitoringDataCB pCallbackFunc){ _set_on_rt_monitoring_data(_rbtCtrlUDP, pCallbackFunc); };
        void set_on_rt_log_alarm(TOnLogAlarmCB pCallbackFunc)  { _set_on_rt_log_alarm(_rbtCtrlUDP, pCallbackFunc); };

        LPROBOT_POSE trans(float fSourcePos[NUM_TASK], float fOffset[NUM_TASK], COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE, COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE){return _trans(_rbtCtrl, fSourcePos, fOffset, eSourceRef, eTargetRef);};
        LPROBOT_POSE ikin(float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE){return _ikin(_rbtCtrl, fSourcePos, iSolutionSpace, eTargetRef); };
		LPINVERSE_KINEMATIC_RESPONSE ikin(float fSourcePos[NUM_TASK], unsigned char iSolutionSpace, COORDINATE_SYSTEM eTargetRef, unsigned char iRefPosOpt){return _ikin_ex(_rbtCtrl, fSourcePos, iSolutionSpace, eTargetRef, iRefPosOpt); };
		LPROBOT_POSE fkin(float fSourcePos[NUM_JOINT], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE){return _fkin(_rbtCtrl, fSourcePos, eTargetRef); };        
        LPROBOT_POSE addto(float fSourcePos[NUM_JOINT], float fOffset[NUM_JOINT]) { return _addto(_rbtCtrl, fSourcePos, fOffset); };

        unsigned char get_solution_space(float fTargetPos[NUM_JOINT]){ return _get_solution_space(_rbtCtrl, fTargetPos);};
        LPROBOT_TASK_POSE get_current_posx(COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE){return _get_current_posx(_rbtCtrl, eCoodType); };
        LPROBOT_POSE get_desired_posx(COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE){return _get_desired_posx(_rbtCtrl, eCoodType);};
        float get_orientation_error(float fPosition1[NUM_TASK], float fPosition2[NUM_TASK], TASK_AXIS eTaskAxis){return _get_orientation_error(_rbtCtrl, fPosition1, fPosition2, eTaskAxis); };
        bool set_workpiece_weight(float fWeight = 0.0, float fCog[3] = COG_DEFAULT, COG_REFERENCE eCogRef = COG_REFERENCE_TCP, ADD_UP eAddUp = ADD_UP_REPLACE, float fStartTime = -10000, float fTransitionTIme = -10000){return _set_workpiece_weight(_rbtCtrl, fWeight, fCog, eCogRef, eAddUp, fStartTime, fTransitionTIme); };
        float get_workpiece_weight(){return _get_workpiece_weight(_rbtCtrl);};
        bool reset_workpiece_weight(){return _reset_workpiece_weight(_rbtCtrl);};
        bool tp_popup_response(POPUP_RESPONSE eRes){return _tp_popup_response(_rbtCtrl, eRes);};
        bool tp_get_user_input_response(string strUserInput){return _tp_get_user_input_response(_rbtCtrl, strUserInput.c_str());};
      
        ////////////////////////////////////////////////////////////////////////////
        // Attributes                                                             //
        ////////////////////////////////////////////////////////////////////////////
        // get verion string
        bool get_system_version(LPSYSTEM_VERSION pVersion) { return _get_system_version(_rbtCtrl, pVersion); };
        const char* get_library_version() { return _get_library_version(_rbtCtrl); };

        // get robot safety mode(manual, auto)
        ROBOT_MODE get_robot_mode() { return _get_robot_mode(_rbtCtrl); };
        // set robot mode mode(manual, auto)
        bool set_robot_mode(ROBOT_MODE eMode) { return _set_robot_mode(_rbtCtrl, eMode); };

        // get robot state( initial, standby, moving, safe-off, teach, ...) 
        ROBOT_STATE get_robot_state() { return _get_robot_state(_rbtCtrl); };
        // set robot state 
        bool set_robot_control(ROBOT_CONTROL eControl) { return _set_robot_control(_rbtCtrl, eControl); };
        CONTROL_MODE get_control_mode(){ return _get_control_mode(_rbtCtrl);};
        // get robot system(real robot, virtrul robot)
        ROBOT_SYSTEM get_robot_system() { return _get_robot_system(_rbtCtrl); };
        // set robot system(real robot, virtrul robot)
        bool set_robot_system(ROBOT_SYSTEM eRobotSystem) { return _set_robot_system(_rbtCtrl, eRobotSystem); };

        // set robot speed mode(noraml reduced)
        bool set_robot_speed_mode(SPEED_MODE eSpeedMode) { return _set_robot_speed_mode(_rbtCtrl, eSpeedMode); };
        // get robot speed mode(noraml reduced)
        SPEED_MODE get_robot_speed_mode() { return _get_robot_speed_mode(_rbtCtrl); };
        
        // get roobt axis data
        LPROBOT_POSE get_current_pose(ROBOT_SPACE eSpaceType = ROBOT_SPACE_JOINT) { return _get_current_pose(_rbtCtrl, eSpaceType); };
        float(* get_current_rotm(COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE))[3]{ return _get_current_rotm(_rbtCtrl, eTargetRef); };
        unsigned char get_current_solution_space() { return _get_current_solution_space(_rbtCtrl); };
        // get current joint position list
        LPROBOT_POSE get_current_posj() { return _get_current_posj(_rbtCtrl); };
        // get current control space
        ROBOT_SPACE get_control_space() { return _get_control_space(_rbtCtrl); };
        // get current joint velocity
        LPROBOT_VEL get_current_velj() { return _get_current_velj(_rbtCtrl); };
        // get target joint position
        LPROBOT_POSE get_desired_posj() { return _get_desired_posj(_rbtCtrl); };
        // get flange task position
        LPROBOT_POSE get_current_tool_flange_posx() { return _get_current_tool_flange_posx(_rbtCtrl); };
        // get current task velocity
        LPROBOT_VEL get_current_velx() { return _get_current_velx(_rbtCtrl); };
        // get target task position
        //LPROBOT_POSE get_desired_posx() { return _GetDesiredPosX(_rbtCtrl); };
        // get target task velocity
        LPROBOT_VEL get_desired_velx() { return _get_desired_velx(_rbtCtrl); }; 
        // get current joint sensor torque value
        LPROBOT_FORCE get_joint_torque() { return _get_joint_torque(_rbtCtrl); };
        // get current external force
        LPROBOT_FORCE get_external_torque() { return _get_external_torque(_rbtCtrl); };
        // get current external force in tool
        LPROBOT_FORCE get_tool_force() { return _get_tool_force(_rbtCtrl); };

        // get program running state
        DRL_PROGRAM_STATE get_program_state() { return _get_program_state(_rbtCtrl); };

        // set safe-stop reset type
        bool set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE eResetType = SAFE_STOP_RESET_TYPE_DEFAULT) { return _set_safe_stop_reset_type(_rbtCtrl, eResetType); }

        // get roobot system alarm
        LPLOG_ALARM get_last_alarm() { return _get_last_alarm(_rbtCtrl); };
        
        ////////////////////////////////////////////////////////////////////////////
        //  access control                                                        //
        ////////////////////////////////////////////////////////////////////////////

        // manage access control
        bool manage_access_control(MANAGE_ACCESS_CONTROL eAccessControl = MANAGE_ACCESS_CONTROL_REQUEST) { return _manage_access_control(_rbtCtrl, eAccessControl); };
        
        ////////////////////////////////////////////////////////////////////////////
        //  motion Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////
        // basic control(hold to run)
        bool jog(JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity) { return _jog(_rbtCtrl, eJogAxis, eMoveReference, fVelocity); };
        bool multi_jog(float fTargetPos[NUM_TASK], MOVE_REFERENCE eMoveReference, float fVelocity) { return _multi_jog(_rbtCtrl, fTargetPos, eMoveReference, fVelocity); };
        bool move_home(MOVE_HOME eMode = MOVE_HOME_MECHANIC, unsigned char bRun = (unsigned)1) { return _move_home(_rbtCtrl, eMode, bRun); };
        LPROBOT_POSE get_user_home(){ return _get_user_home(_rbtCtrl); };
        // motion control: move stop
        bool stop(STOP_TYPE eStopType = STOP_TYPE_QUICK) { return _stop(_rbtCtrl, eStopType); };
        // motion control: move pause
        bool move_pause() { return _move_pause(_rbtCtrl); };
        // motion control: move resume
        bool move_resume() { return _move_resume(_rbtCtrl); };
        // wait motion
        bool mwait() { return _mwait(_rbtCtrl); };

        
        // motion control: joint move
        bool movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movej(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
        bool movej(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movej_ex(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
        bool amovej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _amovej(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eBlendingType); };
        // motion control: linear move
        bool movel(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movel(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); }
        bool amovel(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _amovel(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eBlendingType); }
        // motion control: circle move
        bool movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movec(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fTargetAngle1, fTargetAngle2, fBlendingRadius, eBlendingType); };
        bool amovec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fTargetAngle1 = 0.f , float fTargetAngle2 = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _amovec(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fTargetAngle1, fTargetAngle2, eBlendingType); };
        // motion control: blending move
        bool moveb(MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE) { return _moveb(_rbtCtrl, tTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference); };
        bool amoveb(MOVE_POSB tTargetPos[MAX_MOVEB_POINT], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE) { return _amoveb(_rbtCtrl, tTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference); };
        // motion control: joint move as task information
        bool movejx(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _movejx(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
        bool amovejx(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _amovejx(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eBlendingType); };
        // spline motion as joint information
        bool movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _movesj(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        bool movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUMBER_OF_JOINT], float fTargetAcc[NUMBER_OF_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _movesj_ex(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        bool amovesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _amovesj(_rbtCtrl, fTargetPos ,nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        bool amovesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel[NUMBER_OF_JOINT], float fTargetAcc[NUMBER_OF_JOINT], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _amovesj_ex(_rbtCtrl, fTargetPos ,nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };
        // motion control: spline motin as task information
        bool movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT) { return _movesx(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eVelOpt); };
        bool amovesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT) { return _amovesx(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eVelOpt); };
        // motion control: move spiral motion
        bool move_spiral(TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _move_spiral(_rbtCtrl, eTaskAxis, fRevolution, fMaximuRadius, fMaximumLength, fTargetVel, fTargetAcc, fTargetTime, eMoveReference); };
        bool amove_spiral(TASK_AXIS eTaskAxis, float fRevolution, float fMaximuRadius, float fMaximumLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _amove_spiral(_rbtCtrl, eTaskAxis, fRevolution, fMaximuRadius, fMaximumLength, fTargetVel, fTargetAcc, fTargetTime, eMoveReference); };
        // motion control: move periodic motion
        bool move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned int nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _move_periodic(_rbtCtrl, fAmplitude, fPeriodic, fAccelTime, nRepeat, eMoveReference); };
        bool amove_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, unsigned int nRepeat, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL) { return _amove_periodic(_rbtCtrl, fAmplitude, fPeriodic, fAccelTime, nRepeat, eMoveReference); };
		
        // environment adaptive motion
        bool servoj(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime){ return _servoj(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };
        bool servol(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime){ return _servol(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };

        bool speedj(float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime){ return _speedj(_rbtCtrl, fTargetVel, fTargetAcc, fTargetTime); };
        bool speedl(float fTargetVel[NUM_TASK], float fTargetAcc[2], float fTargetTime){ return _speedl(_rbtCtrl, fTargetVel, fTargetAcc, fTargetTime); };

        bool servoj_g(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime=0.f){ return _servoj_g(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };
        bool servol_g(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime=0.f){ return _servol_g(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime); };

        bool movesx_g(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], unsigned char nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT) { return _movesx_g(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, eVelOpt); };
        bool movesj_g(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], unsigned char nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE) { return _movesj_g(_rbtCtrl, fTargetPos, nPosCount, fTargetVel, fTargetAcc, fTargetTime, eMoveMode); };



        ////////////////////////////////////////////////////////////////////////////
        //  GPIO Operations                                                       //
        ////////////////////////////////////////////////////////////////////////////
        // set digital output on flange
        bool set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff) { return _set_tool_digital_output(_rbtCtrl, eGpioIndex, bOnOff); };
        // get digital input on flange
        bool get_tool_digital_input(GPIO_TOOL_DIGITAL_INDEX eGpioIndex) { return _get_tool_digital_input(_rbtCtrl, eGpioIndex); };
        bool get_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX eGpioIndex) { return _get_tool_digital_output(_rbtCtrl, eGpioIndex); };
        // set digital ouput on control-box
        bool set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff) { return _set_digital_output(_rbtCtrl, eGpioIndex, bOnOff); };
        bool get_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex) { return _get_digital_output(_rbtCtrl, eGpioIndex); };
        // get digital input on control-box
        bool get_digital_input(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex) { return _get_digital_input(_rbtCtrl, eGpioIndex); };

        // set analog ouput on control-box
        bool set_analog_output(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue) { return _set_analog_output(_rbtCtrl, eGpioIndex, fValue); };
        // get analog inut on control-box
        float get_analog_input(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex) { return _get_analog_input(_rbtCtrl, eGpioIndex); };
        // set analog input type on control-box
        bool set_mode_analog_input(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT) { return _set_mode_analog_input(_rbtCtrl, eGpioIndex, eAnalogType); }; 
        // set analog output type on control-box
        bool set_mode_analog_output(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType = GPIO_ANALOG_TYPE_CURRENT) { return _set_mode_analog_output(_rbtCtrl, eGpioIndex, eAnalogType); };

        float get_tool_analog_input(int nCh){ return _get_tool_analog_input(_rbtCtrl, nCh); };
        bool set_tool_digital_output_level(int nLv){ return _set_tool_digital_output_level(_rbtCtrl, nLv); };
        bool set_tool_digital_output_type(int nPort, OUTPUT_TYPE eOutputType){ return _set_tool_digital_output_type(_rbtCtrl, nPort, eOutputType); };
        bool set_mode_tool_analog_input(int nCh, GPIO_ANALOG_TYPE eAnalogType){ return _set_mode_tool_analog_input(_rbtCtrl, nCh, eAnalogType); };
        

        ////////////////////////////////////////////////////////////////////////////
        //  Modbus Operations                                                     //
        ////////////////////////////////////////////////////////////////////////////
        // set modbus register
        bool set_modbus_output(string strSymbol, unsigned short nValue) { return _set_modbus_output(_rbtCtrl, strSymbol.c_str(), nValue); };
        // get modbus register
        unsigned short get_modbus_input(string strSymbol) { return _get_modbus_input(_rbtCtrl, strSymbol.c_str()); };
        // add modbus register
        bool add_modbus_signal(string strSymbol, string strIpAddress, unsigned short nPort, MODBUS_REGISTER_TYPE eRegType, unsigned short iRegIndex, unsigned short nRegValue = 0, unsigned char nSlaiveId = 255) { return _add_modbus_signal(_rbtCtrl, strSymbol.c_str(), strIpAddress.c_str(), nPort, eRegType, iRegIndex, nRegValue, nSlaiveId); };
        // del modbus register
        bool del_modbus_signal(string strSymbol) { return _del_modbus_signal(_rbtCtrl, strSymbol.c_str()); };

        ////////////////////////////////////////////////////////////////////////////
        //  Flange Serial Operations                                              //
        ////////////////////////////////////////////////////////////////////////////
        bool flange_serial_open(int baudrate = 115200, BYTE_SIZE eByteSize = BYTE_SIZE_EIGHTBITS, PARITY_CHECK eParity = PARITY_CHECK_NONE, STOP_BITS eStopBits = STOPBITS_ONE){ return _flange_serial_open(_rbtCtrl, baudrate, eByteSize, eParity, eStopBits); };
        bool flange_serial_close(){ return _flange_serial_close(_rbtCtrl); };
        bool flange_serial_write(int nSize, char* pSendData, int nPort = 1){ return _flange_serial_write(_rbtCtrl, nSize, pSendData, nPort); };
        LPFLANGE_SER_RXD_INFO flange_serial_read(float fTimeout = -1, int nPort = 1){ return _flange_serial_read(_rbtCtrl, fTimeout, nPort); };      

        ////////////////////////////////////////////////////////////////////////////
        //  Configuration Operations                                               //
        ////////////////////////////////////////////////////////////////////////////
        // set tool(end-effector) information
        bool set_tool(string strSymbol) { return _set_tool(_rbtCtrl, strSymbol.c_str()); };
        // get tool(end-effector) information
        string get_tool() { return string(_get_tool(_rbtCtrl)); };
        // add tool(end-effector) information
        bool add_tool(string strSymbol, float fWeight, float fCog[3], float fInertia[NUM_TASK]) { return _add_tool(_rbtCtrl, strSymbol.c_str(), fWeight, fCog, fInertia); };
        // del tool(end-effector) informaiton
        bool del_tool(string strSymbol) { return _del_tool(_rbtCtrl, strSymbol.c_str()); };
        
        // set robot tcp information
        bool set_tcp(string strSymbol) { return _set_tcp(_rbtCtrl, strSymbol.c_str()); };
        // get robot tcp information
        string get_tcp() { return string(_get_tcp(_rbtCtrl)); };  
        // add robot tcp information
        bool add_tcp(string strSymbol, float fPostion[NUM_TASK]) { return _add_tcp(_rbtCtrl, strSymbol.c_str(), fPostion); };
        // del robot tcp information
        bool del_tcp(string strSymbol) { return _del_tcp(_rbtCtrl, strSymbol.c_str()); };
        
        // set robot tool shape information
        bool set_tool_shape(string strSymbol){return _set_tool_shape(_rbtCtrl, strSymbol.c_str());};
        // get robot tool shape information 
        string get_tool_shape(){ return _get_tool_shape(_rbtCtrl);};

        bool set_user_home(){ return _set_user_home(_rbtCtrl); };

        int servo_off(STOP_TYPE eStopType) { return _servo_off(_rbtCtrl, eStopType); };
        bool release_protective_stop(RELEASE_MODE eReleaseMode){ return _release_protective_stop(_rbtCtrl, eReleaseMode); };


        bool set_safety_mode(SAFETY_MODE eSafetyMode, SAFETY_MODE_EVENT eSafetyEvent){ return _set_safety_mode(_rbtCtrl, eSafetyMode, eSafetyEvent); };
        bool set_auto_servo_off(bool bFuncEnable, float fElapseTime){ return _set_auto_servo_off(_rbtCtrl, bFuncEnable, fElapseTime); };
        bool change_collision_sensitivity(float fSensitivity){ return _change_collision_sensitivity(_rbtCtrl, fSensitivity); };

        LPSAFETY_CONFIGURATION_EX get_safety_configuration(){ return _get_safety_configuration(_rbtCtrl); };

        int check_motion() {return _check_motion(_rbtCtrl);};
        ////////////////////////////////////////////////////////////////////////////
        //  drl program Operations                                                //
        ////////////////////////////////////////////////////////////////////////////
        //program start
        bool drl_start(ROBOT_SYSTEM eRobotSystem, string strDrlProgram) { return _drl_start(_rbtCtrl, eRobotSystem, strDrlProgram.c_str()); };
        //program stop
        bool drl_stop(unsigned char eStopType = 0) { return _drl_stop(_rbtCtrl, eStopType); };
        //program pause
        bool drl_pause() { return _drl_pause(_rbtCtrl); };
        //program resume
        bool drl_resume() { return _drl_resume(_rbtCtrl); };
        bool change_operation_speed(float fSpeed) { return _change_operation_speed(_rbtCtrl, fSpeed); };

        ////////////////////////////////////////////////////////////////////////////
        //  force control                                                        //
        ////////////////////////////////////////////////////////////////////////////

        bool task_compliance_ctrl(float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f) { return _task_compliance_ctrl(_rbtCtrl, fTargetStiffness, eForceReference, fTargetTime); };
        //bool EnterJointCompliance(float fTargetStiffness[NUM_TASK], float fTargetTime = 0.f){ return _EnterJointCompliance(_rbtCtrl, fTargetStiffness, fTargetTime);};
        bool set_stiffnessx(float fTargetStiffness[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f) { return _set_stiffnessx(_rbtCtrl, fTargetStiffness, eForceReference, fTargetTime); };
        bool release_compliance_ctrl() { return _release_compliance_ctrl(_rbtCtrl); };
        //bool LeaveJointCompliance() { return _LeaveJointCompliance(_rbtCtrl);};
        bool set_desired_force(float fTargetForce[NUM_TASK], unsigned char iTargetDirection[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL, float fTargetTime = 0.f, FORCE_MODE eForceMode = FORCE_MODE_ABSOLUTE) { return _set_desired_force(_rbtCtrl, fTargetForce, iTargetDirection, eForceReference, fTargetTime, eForceMode); };
        bool release_force(float fTargetTime = 0.f) { return _release_force(_rbtCtrl, fTargetTime); };

        bool check_force_condition(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_force_condition(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, eForceReference); };
        bool check_position_condition_abs(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_position_condition_abs(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, eForceReference); };
        bool check_position_condition_rel(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_position_condition_rel(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, fTargetPos, eForceReference); };
        bool check_position_condition(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], MOVE_MODE eMode = MOVE_MODE_ABSOLUTE, COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_position_condition(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, fTargetPos, eMode, eForceReference); };

        bool check_orientation_condition(FORCE_AXIS eForceAxis, float fTargetMin[NUM_TASK], float fTargetMax[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_orientation_condition_abs(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, eForceReference); };
        bool check_orientation_condition(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL) { return _check_orientation_condition_rel(_rbtCtrl, eForceAxis, fTargetMin, fTargetMax, fTargetPos, eForceReference); };
        bool is_done_bolt_tightening(FORCE_AXIS eForceAxis, float fTargetTor = 0.f, float fTimeout = 0.f) { return _is_done_bolt_tightening(_rbtCtrl, eForceAxis, fTargetTor, fTimeout); };
        bool parallel_axis(float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE) { return _parallel_axis1(_rbtCtrl, fTargetPos1, fTargetPos2, fTargetPos3, eTaskAxis, eSourceRef); };
        bool align_axis(float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE) { return _align_axis1(_rbtCtrl, fTargetPos1, fTargetPos2, fTargetPos3, fSourceVec, eTaskAxis, eSourceRef); };
        bool parallel_axis(float fTargetVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef) { return _parallel_axis2(_rbtCtrl, fTargetVec, eTaskAxis, eSourceRef); };
        bool align_axis(float fTargetVec[3], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef) { return _align_axis2(_rbtCtrl, fTargetVec, fSourceVec, eTaskAxis, eSourceRef); };

        ////////////////////////////////////////////////////////////////////////////
        //  coordinate system control                                             //
        ////////////////////////////////////////////////////////////////////////////

        int set_user_cart_coord(int iReqId, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE) { return _set_user_cart_coord1(_rbtCtrl, iReqId, fTargetPos, eTargetRef); };
        int set_user_cart_coord(float fTargetPos[3][NUM_TASK], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef = COORDINATE_SYSTEM_BASE) { return _set_user_cart_coord2(_rbtCtrl, fTargetPos, fTargetOrg, fTargetRef); };
        int set_user_cart_coord(float fTargetVec[2][3], float fTargetOrg[3], COORDINATE_SYSTEM fTargetRef = COORDINATE_SYSTEM_BASE) { return _set_user_cart_coord3(_rbtCtrl, fTargetVec, fTargetOrg, fTargetRef); };
        LPROBOT_POSE coord_transform(float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eInCoordSystem, COORDINATE_SYSTEM eOutCoordSystem) { return _coord_transform(_rbtCtrl, fTargetPos, eInCoordSystem, eOutCoordSystem); };
        bool set_ref_coord(COORDINATE_SYSTEM eTargetCoordSystem) { return _set_ref_coord(_rbtCtrl, eTargetCoordSystem); };
        LPROBOT_POSE calc_coord(unsigned short nCnt, unsigned short nInputMode, COORDINATE_SYSTEM eTargetRef, float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fTargetPos4[NUM_TASK]) { return _calc_coord(_rbtCtrl, nCnt, nInputMode, eTargetRef, fTargetPos1, fTargetPos2, fTargetPos3, fTargetPos4); };
        LPUSER_COORDINATE get_user_cart_coord(int iReqId) { return _get_user_cart_coord(_rbtCtrl, iReqId); };
        int overwrite_user_cart_coord(bool bTargetUpdate, int iReqId, float fTargetPos[NUM_TASK], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE) { return _overwrite_user_cart_coord(_rbtCtrl, bTargetUpdate, iReqId, fTargetPos, eTargetRef); };
        bool enable_alter_motion(int iCycleTime, PATH_MODE ePathMode, COORDINATE_SYSTEM eTargetRef, float fLimitDpos[2], float fLimitDposPer[2]) { return _enable_alter_motion(_rbtCtrl, iCycleTime, ePathMode, eTargetRef, fLimitDpos, fLimitDposPer); };
        bool disable_alter_motion() { return _disable_alter_motion(_rbtCtrl); };
        bool alter_motion(float fTargetPos[NUM_TASK]) { return _alter_motion(_rbtCtrl, fTargetPos); };
        bool set_singularity_handling(SINGULARITY_AVOIDANCE eMode) { return _set_singularity_handling(_rbtCtrl, eMode); };
        bool config_program_watch_variable(VARIABLE_TYPE eDivision, DATA_TYPE eType, string strName, string strData) { return _config_program_watch_variable(_rbtCtrl, eDivision, eType, strName.c_str(), strData.c_str()); };
        bool save_sub_program(int iTargetType, string strFileName, string strDrlProgram) { return _save_sub_program(_rbtCtrl, iTargetType, strFileName.c_str(), strDrlProgram.c_str()); };
        bool setup_monitoring_version(int iVersion) { return _setup_monitoring_version(_rbtCtrl, iVersion); };
        bool system_shut_down() { return _system_shut_down(_rbtCtrl); };

#if defined(_IS_PLATFORM)
        //brief(159) :Set Time 명령                                      
        bool set_system_time(string szData, string szTime) { return _set_system_time(_rbtCtrl, szData.c_str(), szTime.c_str()); };
        
		//brief(160) :IP 주소 설정                                      
        bool set_ip_address(unsigned char iUsage, unsigned char iIpType, string szHostIp, string szSubnet, string szGateway, string szPrimaryDNS, string szSecondaryDNS) { return _set_ip_address(_rbtCtrl, iUsage, iIpType, szHostIp.c_str(), szSubnet.c_str(), szGateway.c_str(), szPrimaryDNS.c_str(), szSecondaryDNS.c_str()); };
        //brief(161) :전원 관리                                      
        unsigned char set_system_power(unsigned char iTarget, unsigned char iPower) { return _set_system_power(_rbtCtrl, iTarget, iPower); };
        //brief(162) :IP주소 정보 요청                                      
        LPSYSTEM_IPADDRESS get_ip_address(unsigned char iUsage) { return _get_ip_address(_rbtCtrl, iUsage); };
        //brief(163) :시간 정보 요청                                      
        LPSYSTEM_TIME get_system_time() { return _get_system_time(_rbtCtrl); };
        //brief(164) :CPU 사용량 정보 요청                                      
        LPSYSTEM_CPUUSAGE get_cpu_usage() { return _get_cpu_usage(_rbtCtrl); };
        //brief(165) :디스크 용량 정보 요청                                      
        LPSYSTEM_DISKSIZE get_disk_usage() { return _get_disk_usage(_rbtCtrl); };
        //brief(166) :F/W 업데이트
        LPSYSTEM_UPDATE_RESPONSE update_system(string szIpAddress, string szFileName, unsigned char iFileType, unsigned char iInverter[6], unsigned char iResetType) { return _update_system(_rbtCtrl, szIpAddress.c_str(), szFileName.c_str(), iFileType, iInverter, iResetType); };
        //brief(167) :모니터링 정보 전송 제어                                      
        bool set_monitoring_control(unsigned char iControl) { return _set_monitoring_control(_rbtCtrl, iControl); };
        //brief(168) :라이선스 정보 변경                                      
        bool update_license(string lpszLicense) { return _update_license(_rbtCtrl, lpszLicense.c_str()); };
        //brief(169) : 라이센스 정보 확인
        LPLICENSE_TEXT_PARAM get_license() { return _get_license(_rbtCtrl); };

        ////brief(170) :자동 Safe-Off 설정 변경
        //bool set_auto_safe_off(unsigned char bFuncEnable, float fElapseTime) { return _set_auto_safe_off(_rbtCtrl, bFuncEnable, fElapseTime); };
        //brief(171) :시간 동기화 설정 변경                                      
        bool set_time_sync(unsigned char iUsage) { return _set_time_sync(_rbtCtrl, iUsage); };
        //brief(172) :시리얼 통신 포트 정보 요청                                      
        LPSERIAL_SEARCH get_serial_port() { return _get_serial_port(_rbtCtrl); };
        //brief(173) :OS 커널 로그 정보 요청                                      
        const char* get_kernel_log() { return _get_kernel_log(_rbtCtrl); };
        //brief(174) :JTS 교정정보 저장 명령                                      
        bool update_jts_data(float fOffset[NUMBER_OF_JOINT], float fScale[NUMBER_OF_JOINT]) { return _update_jts_data(_rbtCtrl, fOffset, fScale); };
        //brief(175) :서브 시스템 정보 요청                                      
        LPINSTALL_SUB_SYSTEM get_sub_system() { return _get_sub_system(_rbtCtrl); };
        //brief(176) :FTS 교정정보 저장 명령                                      
        bool update_fts_data(float fOffset[NUMBER_OF_JOINT]) { return _update_fts_data(_rbtCtrl, fOffset); };
        //brief(177) :F/W 업데이트 상태 정보 설정                                      
        LPSYSTEM_UPDATE_RESPONSE update_process(unsigned char pProcessInfo, unsigned char pStatusInfo) { return _update_process(_rbtCtrl, pProcessInfo, pStatusInfo); };
        //brief(178) :KT 5G 연동 정보 설정                                      
        bool config_kt_factory_makers(int bEnable, string szIpAddress, int nPort, string szDeviceId, string szDevicePw, string szGatewayId) { return _config_kt_factory_makers(_rbtCtrl, bEnable, szIpAddress.c_str(), nPort, szDeviceId.c_str(), szDevicePw.c_str(), szGatewayId.c_str()); };
        //brief(179) :폴더/파일 삭제 명령                                      
        bool delete_file(string szFileName) { return _delete_file(_rbtCtrl, szFileName.c_str()); };
        
		//brief(180) :운용모드 변경                                      
        bool set_teach_mode(unsigned char bMode) { return _set_teach_mode(_rbtCtrl, bMode); };
        //brief(181) :브레이크 제어                                      
        bool control_brake(unsigned char iTargetAxis, unsigned char bValue) { return _control_brake(_rbtCtrl, iTargetAxis, bValue); };
        //brief(182) :충돌 감시 모드 변경                                      
        //bool enable_collision_mode(unsigned char bMode) { return _enable_collision_mode(_rbtCtrl, bMode); };
        //brief(183) :벡선벡터 계산 및 응답                                      
        LPNORMAL_VECTOR_RESPONSE get_normal(float fTargetPosX[NUMBER_OF_JOINT], float fTargetPosY[NUMBER_OF_JOINT], float fTargetPosZ[NUMBER_OF_JOINT]) { return _get_normal(_rbtCtrl, fTargetPosX, fTargetPosY, fTargetPosZ); };
        //brief(184) :안전 TCP 좌표정보 자동측정 명령                                      
        LPMEASURE_TCP_RESPONSE get_tcp_coord(unsigned char iTargetRef, float fTargetPos1[NUMBER_OF_JOINT], float fTargetPos2[NUMBER_OF_JOINT], float fTargetPos3[NUMBER_OF_JOINT], float fTargetPos4[NUMBER_OF_JOINT]) { return _get_tcp_coord(_rbtCtrl, iTargetRef, fTargetPos1, fTargetPos2, fTargetPos3, fTargetPos4); };
        //brief(185) : jts value 자동 측정
        LPCALIBRATE_JTS_RESPONSE get_jts_value() { return _get_jts_value(_rbtCtrl); };
        //main 브랜치에 추가
        //brief(186) :안전 로봇설치 자세정보 자동측정 명령                                      
        LPCONFIG_INSTALL_POSE get_install_pose() { return _get_install_pose(_rbtCtrl); };
        //brief(187) :안전 FTS 교정정보 자동측정 명령                                      
        LPCALIBRATE_FTS_RESPONSE get_fts_value() { return _get_fts_value(_rbtCtrl); };
        //brief(188) :다중 Digital I/O 출력 설정                                      
        bool set_digital_outputs(unsigned char iLocation, unsigned short iCount, GPIO_PORT tPort[MAX_DIGITAL_BURST_SIZE]) { return _set_digital_outputs(_rbtCtrl, iLocation, iCount, tPort); };
        //brief(189) :다중 Modbus I/O 출력 설정                                      
        bool set_modbus_outputs(MODBUS_REGISTER_BURST pModbusregisterburst) { return _set_modbus_outputs(_rbtCtrl, pModbusregisterburst); };
        
		//brief(190) :모드버스 RTU 정의                                      
        bool add_modbus_rtu_signal(WRITE_MODBUS_RTU_DATA pWritemodbusrtudata) { return _add_modbus_rtu_signal(_rbtCtrl, pWritemodbusrtudata); };
        //brief(191) :모드버스 RTU FC15/16 I/O 추가                                      
        bool add_modbus_rtu_multi_signal(WRITE_MODBUS_RTU_MULTI_DATA pWritemodbusrtumultidata) { return _add_modbus_rtu_multi_signal(_rbtCtrl, pWritemodbusrtumultidata); };
        //brief(192) :모드버스 FC15/16 I/O 삭제                                      
        bool del_modbus_multi_signal(string szSymbol) { return _del_modbus_multi_signal(_rbtCtrl, szSymbol.c_str()); };
        //brief(193) :모드버스 FC15/16 I/O 출력                                      
        bool set_modbus_multi_output(string szSymbol, unsigned char iRegCount, unsigned short iRegValue[MAX_MODBUS_REGISTER_PER_DEVICE]){ return _set_modbus_multi_output(_rbtCtrl, szSymbol.c_str(), iRegCount, iRegValue); };
        //main 브랜치에 추가
        //brief(194) :모드버스 RTU FC15/16 I/O 추가                                      
        //bool add_modbus_rtu_multi_signal2(WRITE_MODBUS_RTU_MULTI_DATA pWritemodbusrtumultidata){ return _add_modbus_rtu_multi_signal2(_rbtCtrl, pWritemodbusrtumultidata); };

        //brief(196) :Tool Shape 정보                                      
        bool add_tool_shape(CONFIG_TOOL_SHAPE_SYMBOL pConfigtoolshapesymbol) { return _add_tool_shape(_rbtCtrl, pConfigtoolshapesymbol); };
        //brief(197) :del_tool_shape
        bool del_tool_shape(string pcArg) { return _del_tool_shape(_rbtCtrl, pcArg.c_str()); };
        //brief(199) :모션 진행 상태 확인#2                                      
        int check_motion_ex() { return _check_motion_ex(_rbtCtrl); };
        
		//brief(200) :Collaborative Zone 감속률 변경                                       
        int change_collaborative_speed(float fSpeed) { return _change_collaborative_speed(_rbtCtrl, fSpeed); };
        //brief(201) :Safety Input Reduced 모드 감속률 변경                                       
        bool change_reduced_speed(float fSpeed) { return _change_reduced_speed(_rbtCtrl, fSpeed); };
        //brief(202) :로봇 설치 자세 정보                                      
        bool set_install_pose(float fGradient, float fRotation) { return _set_install_pose(_rbtCtrl, fGradient, fRotation); };
        //brief(203) :일반 제한(Safety) 정보                                      
        bool set_general_range(GENERAL_RANGE tNormal, GENERAL_RANGE tReduced) { return _set_general_range(_rbtCtrl, tNormal, tReduced); };
        //brief(204) :조인트 제한(Safety) 정보                                      
        bool set_joint_range(JOINT_RANGE tNormal, JOINT_RANGE tReduced) { return _set_joint_range(_rbtCtrl, tNormal, tReduced); };

        //brief(213) :통합 안전 영역 정의(추가)                                      
        bool add_safety_zone(string szIdentifier, string szAlias, unsigned char iZoneType, SAFETY_ZONE_PROPERTY_DATA tZoneProperty, SAFETY_ZONE_SHAPE tShape) { return _add_safety_zone(_rbtCtrl, szIdentifier.c_str(), szAlias.c_str(), iZoneType, tZoneProperty, tShape); };
        //brief(214) :통합 안전 영역 삭제                                      
        bool del_safety_zone(string szIdentifier) { return _del_safety_zone(_rbtCtrl, szIdentifier.c_str()); };
        //메인 브랜치에 추가
        //brief(215) :충돌 민감도 정보                                      
        //DRFL_API bool _change_collision_sensitivity(LPROBOTCONTROL pCtrl, float pfArg);
        //brief(216) :직접 교시 센서 정보                                      
        bool set_teach_sensor(unsigned char iSensorType) { return _set_teach_sensor(_rbtCtrl, iSensorType); };
        //brief(217) :안전 I/O 설정 정보                                      
		bool set_safety_io(unsigned char iIO[TYPE_LAST][NUM_SAFETY]) { return _set_safety_io(_rbtCtrl, iIO); };
        //brief(218) :Cockpit 교시버튼 정보                                      
        bool set_cockpit(unsigned char bEnable, unsigned char iButton[2], unsigned char bRecoveryTeach) { return _set_cockpit(_rbtCtrl, bEnable, iButton, bRecoveryTeach); };
        //brief(219) :External Encoder 극성 설정 정보                                      
        bool set_encorder_polarity(unsigned char iChannel, unsigned char iPolarity[ENCORDER_POLARITY_LAST]) { return _set_encorder_polarity(_rbtCtrl, iChannel, iPolarity); };
        
		//brief(220) :External Encoder 모드 설정 정보                                      
        bool set_encorder_mode(unsigned char iChannel, unsigned char iABMode, unsigned char iZMode, unsigned char iSMode, unsigned char iInvMode, unsigned int  nPulseAZ) { return _set_encorder_mode(_rbtCtrl, iChannel, iABMode, iZMode, iSMode, iInvMode, nPulseAZ); };
        //brief(221) :External Encoder 리셋 명령                                      
        bool reset_encorder(unsigned char iChannel) { return _reset_encorder(_rbtCtrl, iChannel); };
        //brief(222) :Nudge 설정 정보                                      
        bool set_nudge(unsigned char bEnable, float fInputForce, float fDelayTime) { return _set_nudge(_rbtCtrl, bEnable, fInputForce, fDelayTime); };
        //brief(223) :원격 제어 모드 설정 정보                                      
        bool set_remote_control(unsigned char bEnable, CONFIG_IO_FUNC tFunc[TYPE_LAST][NUM_REMOTE_CONTROL]) { return _set_remote_control(_rbtCtrl, bEnable, tFunc); };
        //brief(224) :월드 좌표계 설정                                      
        bool set_world_coord(unsigned char iType, float fPosition[NUMBER_OF_JOINT]) { return _set_world_coord(_rbtCtrl, iType, fPosition); };
        //brief(225) :Safety 파라미터 리셋                                      
        bool reset_safety_config() { return _reset_safety_config(_rbtCtrl); };
        //brief(226) :Process Button제어모드 설정 정보                                      
        bool set_process_button(unsigned char iUsage) { return _set_process_button(_rbtCtrl, iUsage); };
        //brief(227) :안전 순서번호 리셋 명령                                      
        bool reset_sequence(unsigned char iIdentifier) { return _reset_sequence(_rbtCtrl, iIdentifier); };
        //brief(228) :drl_save_main
        bool drl_save_main(unsigned char bMode, unsigned int uLength, string lpszString) { return _drl_save_main(_rbtCtrl, bMode, uLength, lpszString.c_str()); };
        //brief(229) :drl_start_main
        bool drl_start_main(ROBOT_SYSTEM eRobotSystem) { return _drl_start_main(_rbtCtrl, eRobotSystem); };
        
		//brief(230) :중단점(Break) 설정 명령                                      
		bool drl_break(int nLineNum) { return _drl_break(_rbtCtrl, nLineNum); };
        //brief(231) :라인별 실행 명령                                      
        bool drl_step_run() { return _drl_step_run(_rbtCtrl); };
        //brief(232) :문법 검사 명령                                      
        bool drl_check_syntax(unsigned int iTextLength, string iszTextString) { return _drl_check_syntax(_rbtCtrl, iTextLength, iszTextString.c_str()); };
        //brief(233) :라인별 실행 명령 확장                                      
        bool drl_step_run2(unsigned int nLine) { return _drl_step_run2(_rbtCtrl, nLine); };
        //brief(234) :중단점(Break) 설정 명령 확장                                      
        bool drl_break2(int nLine, string szFile) { return _drl_break2(_rbtCtrl, nLine, szFile.c_str()); };
        //brief(235) :관절 공간 순응 제어 모드 설정                                      
        //DRFL_API bool _joint_compliance_ctrl2(LPROBOTCONTROL pCtrl, JOINT_COMPLIANCE_CONTROL pJointcompliancecontrol);
        ////brief(236) :관절 공간 강성(Stiffness) 설정                                      
        //bool set_stiffnessj(float fTargetStf[NUMBER_OF_JOINT], float fTargetTIme) { return _set_stiffnessj(_rbtCtrl, fTargetStf, fTargetTIme); };
        //brief(237) :관절 공간 순응 제어 모드 해제                                      
        //DRFL_API bool _release_joint_compliance_ctrl(LPROBOTCONTROL pCtrl, JOINT_COMPLIANCE_CONTROL pJointcompliancecontrol);
        //brief(238) :축 고정(Constrained Axis) 명령                                      
        bool constrained_axis(unsigned char iTargetAxis[NUMBER_OF_JOINT], unsigned char iTargetRef, float fTargetStf, float fTargetDump) { return _constrained_axis(_rbtCtrl, iTargetAxis, iTargetRef, fTargetStf, fTargetDump); };
        //brief(239) :사용자 좌표계 벡터 정보 확인                                      
        LPUSER_COORDINATE_MATRIX_RESPONSE get_user_cart_coord_matrix(unsigned int iTargetRef) { return _get_user_cart_coord_matrix(_rbtCtrl, iTargetRef); };
        
		////brief(240) :NT 좌표계 설정                                      
  //      int set_user_nt_coord1(unsigned char iTargetRef, float fTargetPosX[6], float fTargetPosY[6], float fTargetPosZ[6]) { return _set_user_nt_coord1(_rbtCtrl, iTargetRef, fTargetPosX, fTargetPosY, fTargetPosZ); };
  //      //brief(241) :NT 좌표계 설정(벡터버전)                                      
  //      int set_user_nt_coord2(unsigned char iTargetRef, float fTargetPos[3]) { return _set_user_nt_coord2(_rbtCtrl, iTargetRef, fTargetPos); };
        //brief(242) :마찰계수(Friction) 정보 보상/저장                                       
        bool enable_friction_compensation(unsigned char iSelect[NUMBER_OF_JOINT], float fPositive[4][NUMBER_OF_JOINT], float fNegative[4][NUMBER_OF_JOINT], float fTemperature[NUMBER_OF_JOINT]) { return _enable_friction_compensation(_rbtCtrl, iSelect, fPositive, fNegative, fTemperature); };
        ////brief(243) :월드 좌표계 확인 요청                                      
        //LPCONFIG_WORLD_COORDINATE check_world_coord(unsigned char iType) { return _check_world_coord(_rbtCtrl, iType); }; // , fPosition);
        //brief(244) :컨베이어 연동 환경 설정                                      
        bool calc_conveyor_param(unsigned char iType, unsigned char iEncoderChannel, unsigned char iTriggerChannel, unsigned char iTriggerEdgeType, float fTriggerMuteTime, char iChannel[2], unsigned char iValue[2]) { return _calc_conveyor_param(_rbtCtrl, iType, iEncoderChannel, iTriggerChannel, iTriggerEdgeType, fTriggerMuteTime, iChannel, iValue); };
        //brief(245) :컨베이어 좌표계 정보 설정                                      
        bool calc_conveyor_param2(int iDistance2Count, POSITION tPosConCoord, unsigned char iTargetRef) { return _calc_conveyor_param2(_rbtCtrl, iDistance2Count, tPosConCoord, iTargetRef); };
        //brief(246) :컨베이어 좌표계 정보 측정                                      
        LPMEASURE_CONVEYOR_COORD_RESPONSE get_conveyor_coord(POSITION tPosTeachPointQ, unsigned char nTeachCount, POSITION tPosTeachPointP[5], unsigned int EncoderCount[5]) { return _get_conveyor_coord(_rbtCtrl, tPosTeachPointQ, nTeachCount, tPosTeachPointP, EncoderCount); };
        //brief(247) :컨베이어 좌표계 정보 측정 확장                                      
        LPMEASURE_CONVEYOR_COORD_RESPONSE get_conveyor_coord2(int nResolution) { return _get_conveyor_coord2(_rbtCtrl, nResolution); };
        //brief(248) :컨베이어 거리 정보 측정                                      
        LPMEASURE_CONVEYOR_DISTANCE_RESPONSE get_conveyor_distance(unsigned int nFilterSize) { return _get_conveyor_distance(_rbtCtrl, nFilterSize); };
        //brief(249) :컨베이어 거리 정보 측정 확장                                      
        LPMEASURE_CONVEYOR_DISTANCE_RESPONSE get_conveyor_distance_ex(float fSpeed) { return _get_conveyor_distance_ex(_rbtCtrl, fSpeed); };
        
		//brief(250) :컨베이어 정보 추가                                      
        bool add_conveyor(string szName, CONVEYOR_BASIC tBASIC, CONVEYOR_COORD_EX tCoord, CONVEYOR_DISTANCE tDistance) { return _add_conveyor(_rbtCtrl, szName.c_str(), tBASIC, tCoord, tDistance); };
        //brief(251) :컨베이어 정보 삭제                                      
        bool del_conveyor(string szName) { return _del_conveyor(_rbtCtrl, szName.c_str()); };
        //brief(252) :컨베이어 모니터링 설정                                      
        bool set_conveyor_monitoring(string szName, unsigned char bStart) { return _set_conveyor_monitoring(_rbtCtrl, szName.c_str(), bStart); };
        //brief(253) :현재 컨베이어 정보 설정                                      
		unsigned char set_conveyor(string szName) { return _set_conveyor(_rbtCtrl, szName.c_str()); };
        //brief(254) :현재 컨베이어 정보 설정 확장                                      
		unsigned char set_conveyor_ex(string szName, CONVEYOR_BASIC tBASIC, CONVEYOR_COORD_EX tCoord, CONVEYOR_DISTANCE tDistance) { return _set_conveyor_ex(_rbtCtrl, szName.c_str(), tBASIC, tCoord, tDistance); };
        //brief(255) :컨베이어 작업물 정보 확인 요청                                      
        int get_conveyor_object(unsigned char iConId, float fTimeout, unsigned char iContainerType, POSITION tPosObjCoord) { return _get_conveyor_object(_rbtCtrl, iConId, fTimeout, iContainerType, tPosObjCoord); };
        //brief(256) :컨베이어 트래킹 제어                                      
        bool set_conveyor_track(unsigned char iConId, unsigned char bTracking, unsigned char bMate, float fDuration, float fDummy[5]) { return _set_conveyor_track(_rbtCtrl, iConId, bTracking, bMate, fDuration, fDummy); };

        ////brief(263) :move_sine
        //bool move_sine(unsigned char iTargetAxs, unsigned char iTargetRef, float fTargetAcc, float fAmplitude, float fFrequency, unsigned int iRepeatCount, unsigned char iTargetAsyn) { return _move_sine(_rbtCtrl, iTargetAxs, iTargetRef, fTargetAcc, fAmplitude, fFrequency, iRepeatCount, iTargetAsyn); };
        ////brief(264) :move_lissajous
        //bool move_lissajous(unsigned char iMotionType, unsigned char iPlaneType, unsigned char iTargetRef, float fTargetAcc, float fAmplitude, float fFrequency, unsigned int iRepeatCount, unsigned char iTargetAsyn) { return _move_lissajous(_rbtCtrl, iMotionType, iPlaneType, iTargetRef, fTargetAcc, fAmplitude, fFrequency, iRepeatCount, iTargetAsyn); };
        
        //brief(270) :app_weld_weave_cond_trapezoidal
        bool app_weld_weave_cond_trapezoidal(CONFIG_TRAPEZOID_WEAVING_SETTING pConfigtrapezoidweavingsetting) { return _app_weld_weave_cond_trapezoidal(_rbtCtrl, pConfigtrapezoidweavingsetting); };
        //brief(271) :app_weld_weave_cond_zigzag
        bool app_weld_weave_cond_zigzag(float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle) { return _app_weld_weave_cond_zigzag(_rbtCtrl, fOffsetY, fOffsetZ, fGradient, fWeavingWidth, fWeavingCycle); };
        //brief(272) :app_weld_weave_cond_circular
        bool app_weld_weave_cond_circular(float fOffsetY, float fOffsetZ, float fGradient, float fwWdt[2], float fwT[2]) { return _app_weld_weave_cond_circular(_rbtCtrl, fOffsetY, fOffsetZ, fGradient, fwWdt, fwT); };
        //brief(273) :app_weld_weave_cond_sinusoidal
        bool app_weld_weave_cond_sinusoidal(float fOffsetY, float fOffsetZ, float fGradient, float fWeavingWidth, float fWeavingCycle) { return _app_weld_weave_cond_sinusoidal(_rbtCtrl, fOffsetY, fOffsetZ, fGradient, fWeavingWidth, fWeavingCycle); };
        //brief(274) :app_weld_enable_analog
        bool app_weld_enable_analog(CONFIG_ANALOG_WELDING_INTERFACE pConfiganalogweldinginterface) { return _app_weld_enable_analog(_rbtCtrl, pConfiganalogweldinginterface); };
        //brief(275) :app_weld_set_weld_cond_analog
        bool app_weld_set_weld_cond_analog(unsigned char iVirtualWelding, float fTargetVoltage, float fTargetCurrent, float fTargetVel, float fMinVel, float fMaxVel, float fTargetFeedingSpeed) { return _app_weld_set_weld_cond_analog(_rbtCtrl, iVirtualWelding, fTargetVoltage, fTargetCurrent, fTargetVel, fMinVel, fMaxVel, fTargetFeedingSpeed); };
        //brief(276) :app_weld_adj_welding_cond_analog
        bool app_weld_adj_welding_cond_analog(unsigned char bRealTime, unsigned char bResetFlag, float fTargetVol, float fFeedingVel, float fTargetVel, float fOffsetY, float fOffsetZ, float fWidthRate) { return _app_weld_adj_welding_cond_analog(_rbtCtrl, bRealTime, bResetFlag, fTargetVol, fFeedingVel, fTargetVel, fOffsetY, fOffsetZ, fWidthRate); };
        //brief(277) :app_weld_set_interface_eip_r2m_process
        bool app_weld_set_interface_eip_r2m_process(CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS pConfigdigitalweldinginterfaceprocess) { return _app_weld_set_interface_eip_r2m_process(_rbtCtrl, pConfigdigitalweldinginterfaceprocess); };
        //brief(278) :app_weld_set_interface_eip_r2m_mode
        bool app_weld_set_interface_eip_r2m_mode(CONFIG_DIGITAL_WELDING_INTERFACE_MODE pConfigdigitalweldinginterfacemode) { return _app_weld_set_interface_eip_r2m_mode(_rbtCtrl, pConfigdigitalweldinginterfacemode); };
        //brief(279) :app_weld_set_interface_eip_r2m_test
        bool app_weld_set_interface_eip_r2m_test(CONFIG_DIGITAL_WELDING_INTERFACE_TEST pConfigdigitalweldinginterfacetest) { return _app_weld_set_interface_eip_r2m_test(_rbtCtrl, pConfigdigitalweldinginterfacetest); };
        
        //brief(280) :app_weld_set_interface_eip_r2m_condition
        bool app_weld_set_interface_eip_r2m_condition(CONFIG_DIGITAL_WELDING_INTERFACE_CONDITION pConfigdigitalweldinginterfacecondition) { return _app_weld_set_interface_eip_r2m_condition(_rbtCtrl, pConfigdigitalweldinginterfacecondition); };
        //brief(281) :app_weld_set_interface_eip_r2m_option
        bool app_weld_set_interface_eip_r2m_option(CONFIG_DIGITAL_WELDING_INTERFACE_OPTION pConfigdigitalweldinginterfaceoption) { return _app_weld_set_interface_eip_r2m_option(_rbtCtrl, pConfigdigitalweldinginterfaceoption); };
        //brief(282) :app_weld_set_interface_eip_m2r_process2
        bool app_weld_set_interface_eip_m2r_process2(CONFIG_DIGITAL_WELDING_INTERFACE_PROCESS2 pConfigdigitalweldinginterfaceprocess2) { return _app_weld_set_interface_eip_m2r_process2(_rbtCtrl, pConfigdigitalweldinginterfaceprocess2); };
        //brief(283) :app_weld_set_interface_eip_m2r_monitoring
        bool app_weld_set_interface_eip_m2r_monitoring(CONFIG_DIGITAL_WELDING_INTERFACE_MONITORING pConfigdigitalweldinginterfacemonitoring) { return _app_weld_set_interface_eip_m2r_monitoring(_rbtCtrl, pConfigdigitalweldinginterfacemonitoring); };
        //brief(284) :app_weld_set_interface_eip_m2r_other
        bool app_weld_set_interface_eip_m2r_other(CONFIG_DIGITAL_WELDING_INTERFACE_OTHER pConfigdigitalweldinginterfacemonitoring) { return _app_weld_set_interface_eip_m2r_other(_rbtCtrl, pConfigdigitalweldinginterfacemonitoring); };
        //brief(285) :app_weld_reset_interface
        bool app_weld_reset_interface(unsigned char bReset) { return _app_weld_reset_interface(_rbtCtrl, bReset); };
        //brief(286) :app_weld_enable_digital
        bool app_weld_enable_digital(unsigned char bMode) { return _app_weld_enable_digital(_rbtCtrl, bMode); };
        //brief(286) :app_weld_disable_digital
        bool app_weld_disable_digital(unsigned char bMode) { return _app_weld_disable_digital(_rbtCtrl, bMode); };
        //brief(287) :app_weld_set_weld_cond_digital
        bool app_weld_set_weld_cond_digital(CONFIG_DIGITAL_WELDING_CONDITION pConfigdigitalweldingcondition) { return _app_weld_set_weld_cond_digital(_rbtCtrl, pConfigdigitalweldingcondition); };
        //brief(288) :app_weld_adj_welding_cond_digital
        bool app_weld_adj_welding_cond_digital(CONFIG_DIGITAL_WELDING_ADJUST pConfigdigitalweldingadjust) { return _app_weld_adj_welding_cond_digital(_rbtCtrl, pConfigdigitalweldingadjust); };
        //brief(289) :measure_welding_tcp
        bool measure_welding_tcp(unsigned char iMode, float fStickout, float fTargetPos[9][NUMBER_OF_JOINT]) { return _measure_welding_tcp(_rbtCtrl, iMode, fStickout, fTargetPos); };
        
        //brief(290) :set_welding_cockpit_setting
        bool set_welding_cockpit_setting(unsigned char bEnable, unsigned char bWeldingType) { return _set_welding_cockpit_setting(_rbtCtrl, bEnable, bWeldingType); };
        //brief(291) :set_digital_welding_signal_output
        bool set_digital_welding_signal_output(unsigned char cDataType, float fData) { return _set_digital_welding_signal_output(_rbtCtrl, cDataType, fData); };
        //brief(292) :set_digital_welding_monitoring_mode
        bool set_digital_welding_monitoring_mode(unsigned char bEnable) { return _set_digital_welding_monitoring_mode(_rbtCtrl, bEnable); };
        //brief(293) :app_weld_adj_motion_offset
        bool app_weld_adj_motion_offset(float fOffsetY, float fOffsetZ) { return _app_weld_adj_motion_offset(_rbtCtrl, fOffsetY, fOffsetZ); };
        //brief(294) :set_output_register_bit
        bool set_output_register_bit(unsigned char iGprType, unsigned char iGprAddr, const char szData[128]) { return _set_output_register_bit(_rbtCtrl, iGprType, iGprAddr, szData); };
        //brief(295) :get_input_register_bit
        LPIETHERNET_SLAVE_RESPONSE_DATA_EX get_input_register_bit(unsigned char iGprType, unsigned char iGprAddr, unsigned char iInOut) { return _get_input_register_bit(_rbtCtrl, iGprType, iGprAddr, iInOut); };
        //brief(296) :focas_get_error_str
        LPMACHINE_TENDING_FOCAS_ERR_STRING focas_get_error_str(unsigned short hHandle, short ErrorCode) { return _focas_get_error_str(_rbtCtrl, hHandle, ErrorCode); };
        //brief(297) :focas_connect
        LPMACHINE_TENDING_FOCAS_CONNECT focas_connect(short ErrorCode, const char szIpAddr[16], unsigned short iPort, unsigned short hHandle, float fTimeOut) { return _focas_connect(_rbtCtrl, ErrorCode, szIpAddr, iPort, hHandle, fTimeOut); };
        //brief(298) :focas_disconnect
        LPMACHINE_TENDING_FOCAS_DISCONNECT focas_disconnect(short ErrorCode, unsigned short hHandle) { return _focas_disconnect(_rbtCtrl, ErrorCode, hHandle); };
        //brief(299) :focas_pmc_read_bit
        LPMACHINE_TENDING_FOCAS_PMC focas_pmc_read_bit(short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset) { return _focas_pmc_read_bit(_rbtCtrl, ErrorCode, hHandle, iDataType, szAddressType, iStartAddressNum, iCount, iBitOffset); };
        //brief(300) :focas_pmc_write_bit
        LPMACHINE_TENDING_FOCAS_PMC focas_pmc_write_bit(short ErrorCode, unsigned short hHandle, short iDataType, const char szAddressType[2], unsigned short iStartAddressNum, unsigned short iCount, unsigned char iBitOffset, MACHINE_TENDING_FOCAS_PMC_DATA tData) { return _focas_pmc_write_bit(_rbtCtrl, ErrorCode, hHandle, iDataType, szAddressType, iStartAddressNum, iCount, iBitOffset, tData); };
        //brief(301) :focas_cnc_read_param
		LPMACHINE_TENDING_RESPONSE_FOCAS_CNC_PARAM focas_cnc_read_param(unsigned short hHandle, short iParamNumber, short iAxisNumber, short iDataLength) { return _focas_cnc_read_param(_rbtCtrl, hHandle, iParamNumber, iAxisNumber, iDataLength); };
        //brief(302) :focas_program_num
        LPMACHINE_TENDING_RESPONSE_FOCAS_PROGRAM_NUMBER focas_program_num(unsigned short nHandle) { return _focas_program_num(_rbtCtrl, nHandle); };
        //brief(303) :is_focas_alive
        LPFOCAS_IS_ALIVE_RESPONSE is_focas_alive(unsigned short nHandle) { return _is_focas_alive(_rbtCtrl, nHandle); };
		//brief(304) : config_setting_enable
		bool config_setting_enable(unsigned short wPreviousCmdid, unsigned int iRefCrc32) { return _config_setting_enable(_rbtCtrl,wPreviousCmdid,iRefCrc32); };
		//brief(305) : config_configurable_io
		bool config_configurable_io(unsigned char niIO[TYPE_LAST][NUM_DIGITAL]) { return _config_configurable_io(_rbtCtrl, niIO); };
		//brief(306) : safe_move_home
		bool safe_move_home(unsigned char bRun = (unsigned)1) { return _safe_move_home(_rbtCtrl, bRun); };
		//brief(307) : safe_movej
		bool safe_movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _safe_movej(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
		//brief(309) :safe_drl_start
		bool safe_drl_start(ROBOT_SYSTEM eRobotSystem) { return _safe_drl_start(_rbtCtrl,eRobotSystem); }
		
        //brief(314) : get_tool_weight
		LPMEASURE_TOOL_RESPONSE get_tool_weight(unsigned char iMode, float fWeight = -999.f, float fXYZ_X = -999.f, float fXYZ_Y = -999.f, float fXYZ_Z = -999.f ){ return _get_tool_weight(_rbtCtrl, iMode, fWeight, fXYZ_X, fXYZ_Y,fXYZ_Z ); };
		//brief(317) : get_friction_value
		LPMEASURE_FRICTION_RESPONSE get_friction_value(unsigned char iType,unsigned char iSelect[NUMBER_OF_JOINT], float fStart[NUMBER_OF_JOINT], float fRange[NUMBER_OF_JOINT]){ return _get_friction_value(_rbtCtrl ,iType, iSelect,  fStart, fRange);  };
		
        //brief(320) : safe_movel
		bool safe_movel(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _safe_movel(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); }
		//brief(323) : safe_move_userhome
		bool safe_move_userhome(unsigned char bRun = (unsigned)1) { return _safe_move_userhome(_rbtCtrl, bRun); };
		//brief(324) : safe_movejx
		bool safe_movejx(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, MOVE_MODE eMoveMode = MOVE_MODE_ABSOLUTE, MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE) { return _safe_movejx(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
		//brief(327) : hold2run 
		bool hold2run() { return _hold2run(_rbtCtrl); };
		//brief(329) :update_gravity_param
        bool update_gravity_param(COUNTER_BALANCE_PARAM_DATA tClParam, CALIBRATION_PARAM_DATA tCrParam) { return _update_gravity_param(_rbtCtrl, tClParam, tCrParam); };
        
        //brief(330) :operation_industrial_ethernet
        bool operation_industrial_ethernet(unsigned char nEtherNetIPOpMode, CONFIG_INDUSTRIAL_ETHERNET tConfig) { return _operation_industrial_ethernet(_rbtCtrl, nEtherNetIPOpMode, tConfig); };
        //brief(331) :report_tcp_client
        LPREPORT_TCP_CLIENT report_tcp_client(unsigned char iIdentifier) { return _report_tcp_client(_rbtCtrl, iIdentifier); };
        //brief(332) :get_sysversion_ex
        LPSYSTEM_VERSION_EX get_sysversion_ex() { return _get_sysversion_ex(_rbtCtrl); };
        //brief(333) :update_local_package
        bool update_local_package(unsigned char iTarget[UPDATE_TARGET_LAST], const char szDirName[MAX_SYMBOL_SIZE]) { return _update_local_package(_rbtCtrl, iTarget, szDirName); };
        //brief(334) :update_network_package
        bool update_network_package(unsigned char iTarget[UPDATE_TARGET_LAST], unsigned char iNetType, const char szIpAddress[16], const char szFileName[MAX_SYMBOL_SIZE]) { return _update_network_package(_rbtCtrl, iTarget, iNetType, szIpAddress, szFileName); };
        //brief(335) :update_local_svm
        bool update_local_svm(const char szFileName[MAX_SYMBOL_SIZE]) { return _update_local_svm(_rbtCtrl, szFileName); };
        //brief(336) :update_network_svm
        bool update_network_svm(unsigned char iNetType, const char szIpAddress[16], const char szFileName[MAX_SYMBOL_SIZE]) { return _update_network_svm(_rbtCtrl, iNetType, szIpAddress, szFileName); };
        //brief(337) :set_restore_list
        LPPACKAGE_RESTORE_LIST set_restore_list() { return _set_restore_list(_rbtCtrl); };
        //brief(338) :set_package_restore
        bool set_package_restore(const char szVersName[MAX_SYMBOL_SIZE]) { return _set_package_restore(_rbtCtrl, szVersName); };
        //brief(339) :set_unzip
        /*bool set_unzip(unsigned char iResult, const char szDirName[MAX_SYMBOL_SIZE]) { return _set_unzip(_rbtCtrl, iResult, szDirName); };*/
        
        //brief(340) :update_excute_package
        bool update_excute_package(unsigned char iExecute) { return _update_excute_package(_rbtCtrl, iExecute); };
        //brief(341) :response_restore_controller
        bool response_restore_controller(unsigned char iProcess) { return _response_restore_controller(_rbtCtrl, iProcess); };
        //brief(342) :set_license_option
        //bool set_license_option(unsigned int nOption) { return _set_license_option(_rbtCtrl, nOption); };
        //brief(343) :set_robot_control_mode
        unsigned char set_robot_control_mode(unsigned char nMode) { return _set_robot_control_mode(_rbtCtrl, nMode); };
        //brief(344) :get_robot_control_mode
        unsigned char get_robot_control_mode() { return _get_robot_control_mode(_rbtCtrl); };
        //brief(345) :get_pattern_point
        LPRESPONSE_TRANS_PALLET_POS get_pattern_point(CONTROL_TRANS_PALLET_POS pControltranspalletpos) { return _get_pattern_point(_rbtCtrl, pControltranspalletpos); };
        //brief(346) :set_oscillation_check
        bool set_oscillation_check(unsigned int puArg) { return _set_oscillation_check(_rbtCtrl, puArg); };
        //brief(347) :drl_nudge
        bool drl_nudge() { return _drl_nudge(_rbtCtrl); };
        //brief(348) :get_user_coord_external_force
        bool get_user_coord_external_force(unsigned char bIsMonitoring, unsigned char iUserID[MAX_USER_COORD_MONITORING_EXT_FORCE_SIZE]) { return _get_user_coord_external_force(_rbtCtrl, bIsMonitoring, iUserID); };
        //brief(349) :set_deflection_comp_mode
        bool set_deflection_comp_mode(unsigned char bEnable) { return _set_deflection_comp_mode(_rbtCtrl, bEnable); };
        
        //brief(350) :set_ie_monitoring
        bool set_ie_monitoring(unsigned char bStart) { return _set_ie_monitoring(_rbtCtrl, bStart); };
        //brief(351) :drl_teach_mode
        unsigned char drl_teach_mode(bool bOnOff) { return _drl_teach_mode(_rbtCtrl, bOnOff); };
        //brief(352) :query_modbus_data_list
        LPMODBUS_DATA_LIST query_modbus_data_list() { return _query_modbus_data_list(_rbtCtrl); };
        //brief(354) : set_on_monitoring_power_button
        void set_on_monitoring_power_button(TOnMonitoringPowerButtonCB pCallbackFunc) { return _set_on_monitoring_power_button(_rbtCtrl, pCallbackFunc); };
        //brief(355) : set_on_monitoring_safety_state
        //DRFL_API void set_on_monitoring_safety_state(LPROBOTCONTROL pCtrl);
        //brief(356) : set_on_monitoring_collision_sensitivity
        void set_on_monitoring_collision_sensitivity(TOnMonitoringCollisionSensitivityCB pCallbackFunc) { return _set_on_monitoring_collision_sensitivity(_rbtCtrl, pCallbackFunc); };
        //brief(357) : set_on_monitoring_system 
        //DRFL_API void set_on_monitoring_system(LPROBOTCONTROL pCtrl);
        //brief(358) : set_on_monitoring_welding
        void set_on_monitoring_welding(TOnMonitoringWeldingCB pCallbackFunc) { return _set_on_monitoring_welding(_rbtCtrl, pCallbackFunc); };
        
        //brief(360) : set_on_monitoring_mode
        void set_on_monitoring_mode(TOnMonitoringModeCB pCallbackFunc) { return _set_on_monitoring_mode(_rbtCtrl, pCallbackFunc); };
        //brief(361) : set_on_monitoring_analog_welding
        void set_on_monitoring_analog_welding(TOnMonitoringAnalogWeldingCB pCallbackFunc) { return _set_on_monitoring_analog_welding(_rbtCtrl, pCallbackFunc); };
        //brief(362) : set_on_monitoring_digital_welding
        void set_on_monitoring_digital_welding(TOnMonitoringDigitalWeldingCB pCallbackFunc) { return _set_on_monitoring_digital_welding(_rbtCtrl, pCallbackFunc); };
        //brief(363) : set_on_monitoring_digital_welding_comm_state
        void set_on_monitoring_digital_welding_comm_state(TOnMonitoringDigitalWeldingCommStateCB pCallbackFunc) { return _set_on_monitoring_digital_welding_comm_state(_rbtCtrl, pCallbackFunc); };
        //brief(364) : set_on_monitoring_user_coord_ext_force
        void set_on_monitoring_user_coord_ext_force(TOnMonitoringUserCoordExtForceCB pCallbackFunc) { return _set_on_monitoring_user_coord_ext_force(_rbtCtrl, pCallbackFunc); };
        //brief(366) : set_on_monitoring_ie_slave
        void set_on_monitoring_ie_slave(TOnMonitoringIeSlaveCB pCallbackFunc) { return _set_on_monitoring_ie_slave(_rbtCtrl, pCallbackFunc); };
        //brief(367) : set_on_monitoring_response_command
        void set_on_monitoring_response_command(TOnMonitoringResponseCommandCB pCallbackFunc) { return _set_on_monitoring_response_command(_rbtCtrl, pCallbackFunc); };
        //brief(368) :query_operating_system
        ROBOT_SYSTEM query_operating_system() { return _query_operating_system(_rbtCtrl); };
        //brief(369) :query_operating_state
        ROBOT_STATE query_operating_state() { return _query_operating_state(_rbtCtrl); };
        
        //brief(371) :measure_tcp
        LPMEASURE_TCP_RESPONSE measure_tcp(unsigned char iTargetRef, float fTargetPos[4][NUMBER_OF_JOINT]) { return _measure_tcp(_rbtCtrl, iTargetRef, fTargetPos); };
        //brief(373) :measure_friction
        LPMEASURE_FRICTION_RESPONSE measure_friction(unsigned char iType, unsigned char iSelect[NUMBER_OF_JOINT], float fStart[NUMBER_OF_JOINT], float fRange[NUMBER_OF_JOINT]) { return _measure_friction(_rbtCtrl, iType, iSelect, fStart, fRange); };

        //brief(375) :measure_install_pose
        bool measure_install_pose(float fGradient, float fRotation) { return _measure_install_pose(_rbtCtrl, fGradient, fRotation); };
        //brief(376) :del_user_coord
        bool del_user_coord(unsigned char iIdentifier) { return _del_user_coord(_rbtCtrl, iIdentifier); };
        //brief(377) :get_user_coord
        bool get_user_coord(unsigned char iIdentifier) { return _get_user_coord(_rbtCtrl, iIdentifier); };
        //brief(378) :twait
        bool twait(float fTime) { return _twait(_rbtCtrl, fTime); };
        
        //brief(382) :add_modbus_multi_signal
        bool add_modbus_multi_signal(const char szSymbol[MAX_SYMBOL_SIZE], const char szIpAddr[16], unsigned short iPort, int iSlaveID, unsigned char iRegType, unsigned short iRegIndex, unsigned char iRegCount) { return _add_modbus_multi_signal(_rbtCtrl, szSymbol, szIpAddr, iPort, iSlaveID, iRegType, iRegIndex, iRegCount); };
        //brief(383) :set_safety_function
        bool set_safety_function(unsigned char iStopCode[SAFETY_FUNC_LAST]) { return _set_safety_function(_rbtCtrl, iStopCode); };
        //brief(385) :simulatior_keyin
        bool simulatior_keyin(unsigned char iKey) { return _simulatior_keyin(_rbtCtrl, iKey); };
        //brief(385) :simulatior_keyin
        //bool set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE eResetType) { return _set_safe_stop_reset_type(_rbtCtrl, eResetType); };
        //bool joint_compliance_ctrl(float fTargetStiffness[NUM_TASK], float fTargetTime) { return _joint_compliance_ctrl(_rbtCtrl, fTargetStiffness, fTargetTime /* = 0.f */); };
		//brief(389) :set_palletizing_operation
		bool set_palletizing_operation(unsigned char iMode) { return _set_palletizing_operation(_rbtCtrl, iMode); };
		
        //brief(390) :set_trajectory_end_mode
		bool set_trajectory_end_mode(unsigned char iMode) { return _set_trajectory_end_mode(_rbtCtrl, iMode); };
		//brief(391) :setup_init_data_complete
		bool setup_init_data_complete() { return _setup_init_data_complete(_rbtCtrl); };
		//brief(393) :get_flange_version
		LPFLANGE_VERSION get_flange_version() { return _get_flange_version(_rbtCtrl); };
		//brief(395) :wait_manual_guide_response
		bool wait_manual_guide_response() { return _wait_manual_guide_response(_rbtCtrl); };
		// brief(396) :  set_on_program_start
		void set_on_program_start(TOnProgramStartCB pCallbackFunc) { return _set_on_program_start(_rbtCtrl, pCallbackFunc);  };
		//brief(397) :  set_on_program_watch_variable
		void set_on_program_watch_variable(TOnProgramWatchVariableCB pCallbackFunc) { return _set_on_program_watch_variable(_rbtCtrl, pCallbackFunc); };
		//brief(398) :  set_on_program_error
		void set_on_program_error(TOnProgramErrorCB pCallbackFunc) { return _set_on_program_error(_rbtCtrl, pCallbackFunc); };
		////brief(399) :  get_drl_line
		//LPPROGRAM_EXECUTION_EX get_drl_line() { return _get_drl_line(_rbtCtrl);  };
		
        //brief(400) :  jog_h2r
		bool jog_h2r(JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity) { return _jog_h2r(_rbtCtrl, eJogAxis, eMoveReference, fVelocity); };
		//brief(401) :  safe_move_home_h2r
		bool safe_move_home_h2r(unsigned char bRun) { return _safe_move_home_h2r(_rbtCtrl, bRun);  };
		//brief(402) :  safe_move_userhome_h2r
		bool safe_move_userhome_h2r(unsigned char bRun) { return _safe_move_userhome_h2r(_rbtCtrl, bRun);  };
		//brief(403) :  safe_movej_h2r
		bool safe_movej_h2r(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime, MOVE_MODE eMoveMode, float fBlendingRadius, BLENDING_SPEED_TYPE eBlendingType) { return _safe_movej_h2r(_rbtCtrl, fTargetPos, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, fBlendingRadius, eBlendingType); };
		//brief(404) :  safe_movel_h2r
		bool safe_movel_h2r(float fTargetPos[NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, MOVE_MODE eMoveMode, MOVE_REFERENCE eMoveReference, float fBlendingRadius, BLENDING_SPEED_TYPE eBlendingType) { return _safe_movel_h2r(_rbtCtrl, fTargetPos, fTargetVel,fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
		//brief(405) :  safe_movejx_h2r
		bool safe_movejx_h2r(float fTargetPos[NUM_JOINT], unsigned char iSolutionSpace, float fTargetVel, float fTargetAcc, float fTargetTime, MOVE_MODE eMoveMode, MOVE_REFERENCE eMoveReference, float fBlendingRadius, BLENDING_SPEED_TYPE eBlendingType) { return _safe_movejx_h2r(_rbtCtrl, fTargetPos, iSolutionSpace, fTargetVel, fTargetAcc, fTargetTime, eMoveMode, eMoveReference, fBlendingRadius, eBlendingType); };
		//brief(406) :  parallel_axis_h2r
		bool parallel_axis_h2r(float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef /* = COORDINATE_SYSTEM_BASE*/){ return _parallel_axis_h2r(_rbtCtrl, fTargetPos1, fTargetPos2, fTargetPos3, eTaskAxis, eSourceRef /* = COORDINATE_SYSTEM_BASE*/); };
		//brief(407) :  align_axis_h2r
		bool align_axis_h2r(float fTargetPos1[NUM_TASK], float fTargetPos2[NUM_TASK], float fTargetPos3[NUM_TASK], float fSourceVec[3], TASK_AXIS eTaskAxis, COORDINATE_SYSTEM eSourceRef /* = COORDINATE_SYSTEM_BASE*/){ return _align_axis_h2r(_rbtCtrl, fTargetPos1, fTargetPos2, fTargetPos3, fSourceVec, eTaskAxis, eSourceRef /* = COORDINATE_SYSTEM_BASE*/); };
		//brief(408) :  set_on_program_drl_line
		void set_on_program_drl_line(TOnProgramDrlLineExCB pCallbackFunc) { return _set_on_program_drl_line(_rbtCtrl, pCallbackFunc); };
		//brief(409) :  set_on_program_state
		void set_on_program_state(TOnProgramStateCB pCallbackFunc) { return _set_on_program_state(_rbtCtrl, pCallbackFunc); };
		
        //brief(410) :  get_drl_program_state
		unsigned char get_drl_program_state() { return _get_drl_program_state(_rbtCtrl); };
		//brief(411) :  servo_on
		bool servo_on() { return _servo_on(_rbtCtrl); };
		//brief(412) :  set_config_user_home
		bool set_config_user_home(float fTargetPos[NUM_JOINT]) { return _set_config_user_home(_rbtCtrl, fTargetPos);  };
		//brief(413) :  get_safety_mode
		SAFETY_MODE get_safety_mode() { return _get_safety_mode(_rbtCtrl); };
		//brief(414) :  get_safety_state
		SAFETY_STATE get_safety_state() { return _get_safety_state(_rbtCtrl); };
		//brief(415) : _send_module_intallation
		bool send_Module_Installation(char *pModulePackageName, int nInstallType=0) { return _send_module_intallation(_rbtCtrl, nInstallType, pModulePackageName); };
		//brief(416) : _send_load_module
		bool send_Load_Module(char *pModuleName) { return _send_load_module(_rbtCtrl, pModuleName); };
		//brief(417) : _send_unload_module
		bool send_Unload_Module(char *pModuleName) { return _send_unload_module(_rbtCtrl, pModuleName); };
		//brief(418) : _send_delete_module
		bool send_Delete_Module(char *pModuleName) { return _send_delete_module(_rbtCtrl, pModuleName); };
        //brief(419) : _set_safety_io_ex
        bool set_safety_io_ex(unsigned char iIO[TYPE_LAST][NUM_SAFETY * 2]) { return _set_safety_io_ex(_rbtCtrl, iIO);  };
        //brief(420) : _config_configurable_io_ex
        bool config_configurable_io_ex(unsigned char iIO[TYPE_LAST][NUM_DIGITAL * 2]) { return _config_configurable_io_ex(_rbtCtrl, iIO); };
        //brief(421) : _set_on_monitoring_ctrl_io_ex2
        void set_on_monitoring_ctrl_io_ex2(TOnMonitoringCtrlIOEx2CB pCallbackFunc) { _set_on_monitoring_ctrl_io_ex2(_rbtCtrl, pCallbackFunc); };
        //brief(422) : _set_tool_digital_input
        bool set_tool_digital_input(GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff) { return _set_tool_digital_input(_rbtCtrl, eGpioIndex, bOnOff);  };
        //brief(423) : _set_tool_analog_input
        bool set_tool_analog_input(GPIO_TOOL_ANALOG_INDEX eGpioIndex, float fValue) { return _set_tool_analog_input(_rbtCtrl, eGpioIndex, fValue); };
        //brief(424) : _set_digital_input
        bool set_digital_input(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff) { return _set_digital_input(_rbtCtrl, eGpioIndex, bOnOff); };
        //brief(425) : _set_analog_input
        bool set_analog_input(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue) { return _set_analog_input(_rbtCtrl, eGpioIndex, fValue); };
        //brief(426) : _get_analog_output
        float get_analog_output(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex) { return _get_analog_output(_rbtCtrl, eGpioIndex); };
        //brief(427) : _set_gpio_io_value
        bool set_gpio_value(unsigned char iLocation, unsigned char iType, unsigned char iSignal, unsigned char iIndex, float fValue) { return _set_gpio_value(_rbtCtrl, iLocation, iType, iSignal, iIndex, fValue); };
        //brief(428) : _set_on_monitoring_protective_safe_off
        void set_on_monitoring_protective_safe_off(TOnMonitoringProtectiveSafeOffCB pCallbackFunc) { _set_on_monitoring_protective_safe_off(_rbtCtrl, pCallbackFunc); };
#endif

protected:
        LPROBOTCONTROL _rbtCtrlUDP;
    };
#endif
}

