// DRFTWin32.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include <Windows.h>
#include <iostream>
#include <conio.h>
#include <process.h>

#include "include/DRFLEx.h"
using namespace DRAFramework;

#undef NDEBUG
#include <assert.h>

CDRFLEx Drfl;
bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool g_mStat = FALSE;
bool g_Stop = FALSE;
string strDrl = "\r\n\
loop = 0\r\n\
while loop < 1003:\r\n\
 movej(posj(10,10.10,10,10.10), vel=60, acc=60)\r\n\
 movej(posj(00,00.00,00,00.00), vel=60, acc=60)\r\n\
 loop+=1\r\n";
DWORD WINAPI ThreadFunc(void *arg);

bool bAlterFlag = FALSE;

void OnTpInitializingCompleted()
{
    // Tp 초기화 이후 제어권 요청.
    g_TpInitailizingComplted = TRUE;
    Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void OnHommingCompleted()
{
    // 50msec 이내 작업만 수행할 것.
    cout << "homming completed" << endl;
}

void OnProgramStopped(const PROGRAM_STOP_CAUSE)
{
    assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
    // 50msec 이내 작업만 수행할 것.
    //assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
    cout << "program stopped" << endl;
}

void OnMonitoringDataCB(const LPMONITORING_DATA pData)
{
    // 50msec 이내 작업만 수행할 것.

    return;
    cout << "# monitoring 0 data " 
    << pData->_tCtrl._tTask._fActualPos[0][0]
    << pData->_tCtrl._tTask._fActualPos[0][1]
    << pData->_tCtrl._tTask._fActualPos[0][2]
    << pData->_tCtrl._tTask._fActualPos[0][3]
    << pData->_tCtrl._tTask._fActualPos[0][4]
    << pData->_tCtrl._tTask._fActualPos[0][5] << endl;
}

void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData)
{
    return;
    cout << "# monitoring 1 data "
    << pData->_tCtrl._tWorld._fTargetPos[0]
    << pData->_tCtrl._tWorld._fTargetPos[1]
    << pData->_tCtrl._tWorld._fTargetPos[2]
    << pData->_tCtrl._tWorld._fTargetPos[3]
    << pData->_tCtrl._tWorld._fTargetPos[4]
    << pData->_tCtrl._tWorld._fTargetPos[5] << endl;
}

void OnMonitoringCtrlIOCB(const LPMONITORING_CTRLIO pData)
{
    return;
    cout << "# monitoring ctrl 0 data" << endl;
    for(int i=0; i<16; i++)
    {
        cout << (int)pData->_tInput._iActualDI[i] << endl;
    }
}

void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX pData)
{   
    return;
    cout << "# monitoring ctrl 1 data" << endl;
    for(int i=0; i<16; i++)
    {
        cout << (int)pData->_tInput._iActualDI[i] << endl;
    }
    for(int i=0; i<16; i++)
    {
        cout << (int)pData->_tOutput._iTargetDO[i] << endl;
    }
}


void OnMonitoringStateCB(const ROBOT_STATE eState)
{
    // 50msec 이내 작업만 수행할 것.
    switch((unsigned char)eState)
    {
#if 0  // TP 초기화시 사용하는 로직임으로 API 레벨에서는 사용하지 말것.(TP없이 단독 사용일 경우, 사용)
    case STATE_NOT_READY:
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_INIT_CONFIG);
        break;
    case STATE_INITIALIZING:
        // add initalizing logic
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_ENABLE_OPERATION);
        break;
#endif
    case STATE_EMERGENCY_STOP:
        // popup
        break;
    case STATE_STANDBY:
    case STATE_MOVING:
    case STATE_TEACHING:
        break;
    case STATE_SAFE_STOP:
        if (g_bHasControlAuthority) {
            Drfl.SetSafeStopResetType(SAFE_STOP_RESET_TYPE_DEFAULT);
            Drfl.SetRobotControl(CONTROL_RESET_SAFET_STOP);
        }
        break;
    case STATE_SAFE_OFF:
        //cout << "STATE_SAFE_OFF1" << endl;
        if (g_bHasControlAuthority) {
            //cout << "STATE_SAFE_OFF2" << endl;
            Drfl.SetRobotControl(CONTROL_SERVO_ON);
        }
        break;
    case STATE_SAFE_STOP2:
        if (g_bHasControlAuthority) Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
        break;
    case STATE_SAFE_OFF2:       
        if (g_bHasControlAuthority) {
            Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
        }
        break;
    case STATE_RECOVERY:
        //Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
        break;
    default:
        break;
    }
    return;
    cout << "current state: " << (int)eState << endl;
}

void OnMonitroingAccessControlCB(const MONITORING_ACCESS_CONTROL eTrasnsitControl)
{
    // 50msec 이내 작업만 수행할 것.

    switch(eTrasnsitControl)
    {
    case MONITORING_ACCESS_CONTROL_REQUEST:
        assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO));
        //Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
        break;
    case MONITORING_ACCESS_CONTROL_GRANT:
        g_bHasControlAuthority = TRUE;
        //cout << "GRANT1" << endl;
        //cout << "MONITORINGCB : " << (int)Drfl.GetRobotState() << endl;
        OnMonitoringStateCB(Drfl.GetRobotState());
        //cout << "GRANT2" << endl;
        break;
    case MONITORING_ACCESS_CONTROL_DENY:
    case MONITORING_ACCESS_CONTROL_LOSS:
        g_bHasControlAuthority = FALSE;
        if (g_TpInitailizingComplted) {
            //assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST));
            Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
        }
        break;
    default:
        break;
    }
}

void OnLogAlarm(LPLOG_ALARM tLog)
{
	g_mStat = true;
	cout << "Alarm Info: " << "group(" << (unsigned int)tLog->_iGroup << "), index("<< tLog->_iIndex
	<< "), param(" << tLog->_szParam[0] << "), param(" << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void OnTpPopup(LPMESSAGE_POPUP tPopup)
{
    cout << "Popup Message: " << tPopup->_szText << endl;
    cout << "Message Level: " << tPopup->_iLevel << endl;
    cout << "Button Type: " << tPopup->_iBtnType << endl;
}

void OnTpLog(const char* strLog)
{
    cout << "Log Message: " << strLog << endl;
}

void OnTpProgress(LPMESSAGE_PROGRESS tProgress)
{
    cout << "Progress cnt : " << (int)tProgress->_iTotalCount << endl;
    cout << "Current cnt : " << (int)tProgress->_iCurrentCount << endl;
}

void OnTpGetuserInput(LPMESSAGE_INPUT tInput)
{
    cout << "User Input : " << tInput->_szText << endl;
    cout << "Data Type : " << (int)tInput->_iType << endl;
}

DWORD WINAPI ThreadFunc(void *arg)
{
	while (true) {
		if (_kbhit())
		{	
			char ch = _getch();
			switch (ch) {
			case 's': 
				{
					printf("Stop!\n");
					g_Stop = true;
					Drfl.MoveStop(STOP_TYPE_SLOW);
				}
				break;
			case 'p' :
				{
					printf("Pause!\n");
					Drfl.MovePause();
				}
				break;
			case 'r' :
				{
					printf("Resume!\n");
					Drfl.MoveResume();
				}
                break;

            case 'y' :
                {
                    while(bAlterFlag){
                        float pos[6] = {10,0,0,10,0,0};
                        cout << "Alter Flag On !!!!!!" << endl;
                        Drfl.alter_motion(pos);
                    }
                }   
			}

		}
		Sleep(100);
	}

	return 0;
}

void OnDisConnected()
{
    while(!Drfl.open_connection("192.168.137.100")) {
        Sleep(1000);
    }
}

int _tmain(int argc, _TCHAR* argv[])
{
    // 콜백 등록(// 콜백 함수 내에서는 50msec 이내 작업만 수행할 것)
    Drfl.set_on_homming_completed(OnHommingCompleted);
    Drfl.set_on_monitoring_data(OnMonitoringDataCB);
    Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);
    Drfl.set_on_monitoring_ctrl_io(OnMonitoringCtrlIOCB);
    Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);
    Drfl.set_on_monitoring_state(OnMonitoringStateCB);
    Drfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
    Drfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
	Drfl.set_on_log_alarm(OnLogAlarm);
    Drfl.set_on_tp_popup(OnTpPopup);
    Drfl.set_on_tp_log(OnTpLog);
    Drfl.set_on_tp_progress(OnTpProgress);
    Drfl.set_on_tp_get_user_input(OnTpGetuserInput);

    Drfl.set_on_program_stopped(OnProgramStopped);
    Drfl.set_on_disconnected(OnDisConnected);

    // 연결 수립
    assert(Drfl.open_connection("192.168.137.100"));
    
    // 버전 정보 획득
    SYSTEM_VERSION tSysVerion = {'\0', };
    Drfl.get_system_version(&tSysVerion);
    // 모니터링 데이터 버전 변경
    assert(Drfl.setup_monitoring_version(1));
    Drfl.set_robot_control(CONTROL_SERVO_ON);
    Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_10, TRUE);
    cout << "System version: " << tSysVerion._szController << endl;
    cout << "Library version: " << Drfl.get_library_version() << endl;
    
    while ((Drfl.get_robot_state() != STATE_STANDBY) || !g_bHasControlAuthority)
        Sleep(1000);
    
    // 수동 모드 설정

    assert(Drfl.set_robot_mode(ROBOT_MODE_MANUAL));
    assert(Drfl.set_robot_system(ROBOT_SYSTEM_REAL));
        
    //Drfl.ConfigCreateModbus("mr1", "192.168.137.70", 552, MODBUS_REGISTER_TYPE_HOLDING_REGISTER, 3, 5);
      

    typedef enum {
        EXAMPLE_JOG,
        EXAMPLE_HOME,
        EXAMPLE_MOVEJ_ASYNC,
        EXAMPLE_MOVEL_SYNC,
        EXAMPLE_MOVEJ_SYNC,
        EXAMPLE_DRL_PROGRAM,
        EXAMPLE_GPIO,
        EXAMPLE_MODBUS,
        EXAMPLE_LAST,
        EXAMPLE_SERVO_OFF
    } EXAMPLE;

    EXAMPLE eExample = EXAMPLE_LAST;
	
	HANDLE hThread;
	DWORD dwThreadID;
	hThread = CreateThread(NULL, 0, ThreadFunc, NULL, 0, &dwThreadID);
	if (hThread == 0) {
		printf("Thread Error\n");
		return 0;
	}
    
    bool bLoop = TRUE;
    while (bLoop) {
        g_mStat = false;
		g_Stop  = false;
#if 0
        static char ch = '0';
        if (ch == '7') ch = '0';
        else if (ch == '0') ch = '7';
#else
        cout << "\ninput key : ";
        char ch = _getch();
        cout << ch << endl;
#endif    
        switch (ch)
        {
        case 'q':
            bLoop = FALSE;
            break;
        case '0':
            {
                switch ((int)eExample) {
                case EXAMPLE_JOG:
                    assert(Drfl.Jog(JOG_AXIS_JOINT_1, MOVE_REFERENCE_BASE, 0.f));
                    cout << "jog stop" << endl;
                    break;
                case EXAMPLE_HOME:
                    assert(Drfl.Home((unsigned char)0));
                    cout << "home stop" << endl;
                    break;
                case EXAMPLE_MOVEJ_ASYNC:
                    assert(Drfl.MoveStop(STOP_TYPE_SLOW));
                    cout << "movej async stop" << endl;
                    break;
                case EXAMPLE_MOVEL_SYNC:
                case EXAMPLE_MOVEJ_SYNC:
                    break;
                case EXAMPLE_DRL_PROGRAM:
                    assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
                    //assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
                    //assert(Drfl.SetRobotSystem(ROBOT_SYSTEM_REAL));
                    cout << "drl player stop" << endl;
                    break;
                case EXAMPLE_GPIO:
                    cout << "reset gpio" << endl;
                    for (int i = 0; i < NUM_DIGITAL; i++) {
                        assert(Drfl.SetCtrlBoxDigitalOutput((GPIO_CTRLBOX_DIGITAL_INDEX)i, FALSE));
                    }
                    break;
                case EXAMPLE_MODBUS:
                    cout << "reset modbus" << endl;
                    assert(Drfl.SetModbusValue("mr1", 0));
                    break;
                default:
                    break;
                }
            }
            break;
        case '1':
            {
                //cout << "jog start" << endl;
                //assert(Drfl.Jog(JOG_AXIS_JOINT_1, MOVE_REFERENCE_BASE,  60.f));
                //eExample = EXAMPLE_JOG;
                Drfl.set_robot_control(CONTROL_RESET_RECOVERY);
            }
            break;
        case '2':
            {
                //cout << "jog start" << endl;
                //assert(Drfl.Jog(JOG_AXIS_JOINT_1, MOVE_REFERENCE_BASE, -60.f));
                //eExample = EXAMPLE_JOG;

                Drfl.servo_off(STOP_TYPE_QUICK);
            }
            break;
        case '3':
            {
                //cout << "home start" << endl;
                //assert(Drfl.Home(TRUE));
                //eExample = EXAMPLE_HOME;
                Drfl.set_robot_control(CONTROL_RECOVERY_SAFE_OFF);
            }
            break;
        case '4':
            {
                float TargetStiffness[6] = { 3000, 3000, 3000, 200, 200, 200 };
                float TargetStiffness2[6] = { 20000, 20000, 1000, 200, 200, 200 };
                float force[6] = { 0, 0, 20, 0, 0, 0};
                unsigned char fdir[6] = {0, 0, 1, 0, 0, 0 };
                Drfl.task_compliance_ctrl(TargetStiffness, COORDINATE_SYSTEM_BASE, 0.5f);
                Drfl.set_stiffnessx(TargetStiffness2, COORDINATE_SYSTEM_TOOL, 0.5f);
                Drfl.set_desired_force(force, fdir, COORDINATE_SYSTEM_BASE, 0.1f, FORCE_MODE_ABSOLUTE);
                float pos1[6] = {100, 0, 0, 0, 0 , 0};
                float vel[2] = {60, 60};
                float acc[2] = {100, 100};
                Sleep(2000);
                //Drfl.movel(pos1, vel, acc, 0, MOVE_MODE_RELATIVE);
                Drfl.release_force();
                Drfl.release_compliance_ctrl();
            }
            break;
        case '5':
            {
             //   cout << "Serial Comm" << endl;
             //   Drfl.flange_serial_open(2400);
             //   Drfl.flange_serial_write(8, "test1234");
             //   LPFLANGE_SER_RXD_INFO temp = Drfl.flange_serial_read();
             //   cout << "serial read : " << temp->_cRxd << endl;
             //   cout << "serial cnt : " << (int)temp->_iSize << endl;
             //   Drfl.flange_serial_close();
                float px[6] = {500,550,600,10,20,30};
                ROBOT_POSE *rp = Drfl.ikin(px, 2, COORDINATE_SYSTEM_BASE);
                for(int i=0; i<6; i++){
                    cout << rp->_fPosition[i] << endl;
                }
            }

            break;
        case '6':
            {
                cout << "custom move home test" << endl;
                Drfl.move_home(MOVE_HOME_USER);
            }
            break;
        case '7':
            {   
                while(1){
                    Drfl.set_robot_mode(ROBOT_MODE_MANUAL);
                    Drfl.change_operation_speed(100);
                    cout << "DRFL All Teset" << endl;
                    cout << "CalTrans" << endl;
                    float cal_trans_point[6] = { 30, 30, 30, 30, 30, 30 };
                    float cal_trans_offset[6] = { 100, 100, 100, 100, 100, 100};
                    LPROBOT_POSE cal_trans_res = Drfl.trans(cal_trans_point, cal_trans_offset);
                    for(int i=0; i<6; i++){
                        cout << cal_trans_res->_fPosition[i] << endl;
                    }
                
                    cout << "CalFKin" << endl;
                    float cal_fkin_q1[6] = {0,0,90,0,90,0};
                    Drfl.movej(cal_fkin_q1, 60, 30);
                    float cal_fkin_q2[6] = {30, 0, 90, 0, 90, 0};
                    LPROBOT_POSE cal_fkin_res = Drfl.fkin(cal_fkin_q2, COORDINATE_SYSTEM_WORLD);
                    float cal_fkin_vel[2] = {100, 100};
                    float cal_fkin_acc[2] = {200, 200};
                    float cal_fkin_x2[6] = {0,};
                    Drfl.movej(cal_fkin_q1, 60, 30);
                    for(int i=0; i<6; i++){
                        cal_fkin_x2[i] = cal_fkin_res->_fPosition[i];
                    }
                    Drfl.movel(cal_fkin_x2, cal_fkin_vel, cal_fkin_acc);
                
                
                    cout << "CalIKin" << endl;
                    float cal_ikin_x1[6] = {370.9f, 719.7f, 651.5f, 90, -180, 0};
                    LPROBOT_POSE cal_ikin_res = Drfl.ikin(cal_ikin_x1, 2);
                    float cal_ikin_q1[6] = {0,};
                    for(int i=0; i<6; i++){
                        cal_ikin_q1[i] = cal_ikin_res->_fPosition[i];
                    }
                    Drfl.movej(cal_ikin_q1, 60, 30);
                
                    cout << "SetReferenceCoordinate" << endl;
                    float set_ref_coord_p1[6] = {0, 0, 90, 0, 90, 0};
                    Drfl.movej(set_ref_coord_p1, 60, 30);
                    float set_ref_coord_vec[2][3] = {{-1, 1, 1}, {1, 1, 0}};  
                    float set_ref_coord_org[3] = {370.9f, -419.7f, 651.5f};
          
                    //int set_ref_coord_user = Drfl.ConfigUserCoordinateSystemEx(set_ref_coord_vec, set_ref_coord_org); //2.5 핫픽스 이전 버전에서 작동 안할수 있음
                    Drfl.set_ref_coord((COORDINATE_SYSTEM)1);
                    Drfl.movej(cal_fkin_q1, 60, 30);
                
                    cout << "UserHome" << endl;
                    float user_home_p1[6] = {0,0,90,0,90,0};
                    Drfl.movej(user_home_p1, 60, 30);
                    bool user_home_res = Drfl.move_home(MOVE_HOME_USER);
                    cout << user_home_res << endl;
                
                    cout << "CheckMotion" << endl;
                    float check_motion_q0[6] = {0, 0, 90, 0, 90, 0};
                    float check_motion_q99[6] = {0, 0, 0, 0, 0, 0};
                    Drfl.MoveJAsync(check_motion_q0, 60, 30); //q0로 모션 및 즉시 다음명령 수행
                    int check_motion_res = Drfl.check_motion();
                    cout << check_motion_res << endl;
             
                    Drfl.mwait();   
    /*
                    cout << "PlayDrlSpeed" << endl;
                    bool play_drl_speed_res = Drfl.PlayDrlSpeed(10);
                    cout << play_drl_speed_res << endl;
                    string play_drl_speed_res_strDrlProgram = "loop = 0\nwhile loop < 3:\n  movej(posj(10,10.10,10,10.10), vel=60, acc=60)\n  movej(posj(00,00.00,00,00.00), vel=60, acc=60)\n  loop+=1\n  movej(posj(10,10.10,10,10.10), vel=60, acc=60)";

                    if (Drfl.GetRobotState() == STATE_STANDBY) {
                        Drfl.SetRobotMode(ROBOT_MODE_AUTONOMOUS);
                        if (Drfl.GetRobotMode() == ROBOT_MODE_AUTONOMOUS) {
                        // 자동모드
                        ROBOT_SYSTEM eTargetSystem = ROBOT_SYSTEM_VIRTUAL;
                        bool drl_start_res = Drfl.PlayDrlStart(eTargetSystem, play_drl_speed_res_strDrlProgram);
                        cout << drl_start_res << endl;
                        }
                    }*/
                    //Drfl.SetRobotMode(ROBOT_MODE_AUTONOMOUS);
                
                    Drfl.change_operation_speed(70);
                    Drfl.mwait();
                    cout << "EnableAlterMotion" << endl;
                    float enable_alter_motion_limit_dPOS[2] = {50, 90};
                    float enable_alter_motion_limit_dPOS_per[2] = {50, 50};
                    bool enable_alter_motion_res = Drfl.enable_alter_motion(5, PATH_MODE_DPOS, COORDINATE_SYSTEM_BASE, enable_alter_motion_limit_dPOS, enable_alter_motion_limit_dPOS_per);
                    cout << enable_alter_motion_res << endl;
                    cout << "AlterMotion" << endl;
                    float alter_motion_pos[6] = {10, 0, 0, 10, 0, 0};
                    bool alter_motion_res = Drfl.alter_motion(alter_motion_pos);
                    cout << alter_motion_res << endl;
                    cout << "DisableAlterMotion" << endl;
                    bool disable_alter_motion_res = Drfl.disable_alter_motion();
                    cout << disable_alter_motion_res << endl;
                
                    //cout << "SetCurrentToolShape" << endl;
                    //bool set_tool_shape_res = Drfl.SetCurrentToolShape("tool_shape1");
                    //cout << set_tool_shape_res << endl;
                
                    float ready_pos[6] = {0,0,90,0,90,0};
                    Drfl.movej(ready_pos, 60, 30);
                    cout << "MeasurePayload" << endl;
                    float get_workpiece_weight;
                    get_workpiece_weight = Drfl.get_workpiece_weight();
                    cout << get_workpiece_weight << endl;
                    cout << "ResetPayload" << endl;
                    bool reset_payload_res = Drfl.reset_workpiece_weight();
                    cout << reset_payload_res << endl;
                
                    Drfl.MoveJ(ready_pos, 60, 30);
                    cout << "SetSingularityHandling" << endl;
                    float set_singularity_handling_p1[6] = {400, 500, 800, 0 ,180 ,0};
                    float set_singularity_handling_p2[6] = {400, 500, 500, 0, 180, 0};
                    float set_singularity_handling_p3[6] = {400, 500, 200, 0, 180, 0};
                    bool set_singularity_handling_res1 = Drfl.set_singularity_handling(SINGULARITY_AVOIDANCE_AVOID);
                    cout << set_singularity_handling_res1 << endl;
                    float set_singularity_handling_v1[2] = {60, 60};
                    float set_singularity_handling_a1[2] = {30, 30};
                    Drfl.movel(set_singularity_handling_p1, set_singularity_handling_v1, set_singularity_handling_a1);
                    bool set_singularity_handling_res2 = Drfl.set_singularity_handling(SINGULARITY_AVOIDANCE_STOP);
                    cout << set_singularity_handling_res2 << endl;
                    Drfl.MoveL(set_singularity_handling_p2, set_singularity_handling_v1, set_singularity_handling_a1);
                    bool set_singularity_handling_res3 = Drfl.set_singularity_handling(SINGULARITY_AVOIDANCE_VEL);
                    cout << set_singularity_handling_res3 << endl;
                    Drfl.MoveL(set_singularity_handling_p3, set_singularity_handling_v1, set_singularity_handling_a1);
                    Drfl.mwait();
                
                    cout << "SetupMonitoringVersion" << endl;
                    bool setup_monitoring_version_res = Drfl.setup_monitoring_version(1);
                    cout << setup_monitoring_version_res << endl;
                
                    cout << "ConfigProgramWatchVariable" << endl;
                    bool config_program_watch_variable_res = Drfl.config_program_watch_variable(VARIABLE_TYPE_INSTALL, DATA_TYPE_FLOAT, "Pi", "3.14");
                    cout << config_program_watch_variable_res << endl;
                
                    cout << "GetCtrlBoxdigitalOutput" << endl;
                    bool get_digital_output = Drfl.get_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1);
                    Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_1, FALSE);
                    Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_2, FALSE);
                    cout << "get_Ctrl_box_digital_output : " <<  get_digital_output << endl;
                    bool get_digital_input_1 = Drfl.get_digital_input(GPIO_CTRLBOX_DIGITAL_INDEX_10);
                    cout << get_digital_input_1 << endl;

                    cout << "SaveSubProgram" << endl;
                    bool save_sub_program = Drfl.save_sub_program(1, "sub", "movej([0,0,90,0,90,0], 60, 30)");
                    cout << save_sub_program << endl;
                
                    Drfl.movej(ready_pos, 60, 30);
                    cout << "ParallelAxis1" << endl;
                    float parallel_axis1_x0[6] = {0, 0, 90, 0, 90, 0};
                    Drfl.MoveJ(parallel_axis1_x0, 60, 30);
                    float parallel_axis1_x1[6] = {0, 500, 700, 30, 0, 90};
                    float parallel_axis1_x2[6] = {500, 0, 700, 0, 0, 45};
                    float parallel_axis1_x3[6] = {300, 100, 500, 45, 0, 45};
                    bool parallel_axis1_res = Drfl.parallel_axis(parallel_axis1_x1, parallel_axis1_x2, parallel_axis1_x3, TASK_AXIS_X);
                    cout << parallel_axis1_res << endl;

                    Drfl.MoveJ(ready_pos, 60, 30);
                    cout << "ParallelAxis2" << endl;
                    float parallel_axis_2_v[3] = {1000, 700, 300};
                    bool parallel_axis_2_res = Drfl.parallel_axis(parallel_axis_2_v, TASK_AXIS_X, COORDINATE_SYSTEM_BASE);
                    cout << parallel_axis_2_res << endl;
            
                   /* Drfl.MoveJ(ready_pos, 60, 30);
                    cout << "AlignAxis1" << endl;
                    float align_axis_1_x1[6] = {0, 500, 700, 30, 0, 0};                
                    float align_axis_1_x2[6] = {500, 0, 700, 0, 0, 0};                 
                    float align_axis_1_x3[6] = {300, 100, 500, 0, 0, 0};                
                    float align_axis_1_pos[3] = {400, 400, 500};
                    bool align_axis_1_res = Drfl.AlignAxis1(align_axis_1_x1, align_axis_1_x2, align_axis_1_x3, align_axis_1_pos, TASK_AXIS_X, COORDINATE_SYSTEM_BASE);
                    cout << align_axis_1_res << endl;*/
    /*
                    Drfl.MoveJ(ready_pos, 60, 30);
                    cout << "AlignAxis2" << endl;
                    float align_axis_2_v1[3] = {400, 400, 500};
                    float align_axis_2_v2[3] = {350, 37, 430};
                    bool align_axis_2_res = Drfl.AlignAxis2(align_axis_2_v1, align_axis_2_v2, TASK_AXIS_X, COORDINATE_SYSTEM_BASE);
                    cout << align_axis_2_res << endl;*/

                    Drfl.MoveJ(ready_pos, 60, 30);
                    Drfl.mwait();
                    cout << "WaitForBoltTightening" << endl;
                    float wait_for_bolt_tight_p0[6] = {0,0,90,0,90,0};
                    Drfl.MoveJ(wait_for_bolt_tight_p0, 60, 30);
                    float wait_for_bolt_tight_stx[6] = {3000, 3000, 3000, 200, 200, 200};
                    Drfl.task_compliance_ctrl(wait_for_bolt_tight_stx);
                    float wait_for_bolt_tight_x1[6] = {559, 34.5, 651.5, 0, 180, 60};
                    float wait_for_bolt_tight_velx[2] = {50, 50};
                    float wait_for_bolt_tight_accx[2] = {50, 50};
                    Drfl.amovel(wait_for_bolt_tight_x1, wait_for_bolt_tight_velx, wait_for_bolt_tight_accx);
                    bool wait_for_bolt_tight_res = Drfl.is_done_bolt_tightening(FORCE_AXIS_Z, 10, 5);
                    cout << wait_for_bolt_tight_res << endl;
                    Drfl.release_compliance_ctrl();

                    cout << "EnterTaskCompliance" << endl;
                    float enter_task_compliance_p0[6] = {0, 0, 90, 0, 90, 0};
                    //Drfl.MoveJ(enter_task_compliance_p0, 60, 30);  ///??ignore command
                    float stx[6] = {500, 500, 500, 100, 100, 100};
                    bool enter_task_compliance_res = Drfl.task_compliance_ctrl(stx, COORDINATE_SYSTEM_TOOL, 1);
                    cout << enter_task_compliance_res << endl;
                
                    cout << "SetTaskStiffness" << endl;
                    float set_stiffness_stx2[6] = {1, 2, 3, 4, 5, 6};
                    bool set_stiffness_res = Drfl.set_stiffnessx(set_stiffness_stx2);
                    cout << set_stiffness_res << endl;
                    cout << "LeaveTaskCompliance" << endl;
                    bool release_compliance_res = Drfl.release_compliance_ctrl();
                    cout << release_compliance_res << endl;

                    //Drfl.MoveJ(ready_pos, 60, 30);
               
                    cout << "CalUserCoordinate" << endl;
                    float calc_coord_pos1[6] = {500, 30, 500, 0, 0, 0};
                    float calc_coord_pos2[6] = {400, 30, 500, 0, 0, 0};
                    float calc_coord_pos3[6] = {500, 30, 600, 45, 180, 45};
                    float calc_coord_pos4[6] = {500, -30, 600, 0, 180, 0};
                    ROBOT_POSE* calc_coord_pose_user1 = Drfl.calc_coord(4, 0, COORDINATE_SYSTEM_BASE, calc_coord_pos1, calc_coord_pos2, calc_coord_pos3, calc_coord_pos4);
                    for (int i=0; i<NUM_TASK; i++)
                    {
                        cout << calc_coord_pose_user1->_fPosition[i] << endl;
                    }

                    cout << "ConfigUserCoordinate" << endl;
                    float set_coord_1_pos1[6] = {500, 30, 500, 0, 0, 0};
                    int set_coord_1_id = Drfl.set_user_cart_coord(0, set_coord_1_pos1, COORDINATE_SYSTEM_BASE);
    /*
                    cout << "ConfigUserCoordinateSysteEx" << endl;
                    float set_coord_2_vec[2][3] = {{-1, 1, 1}, {1, 1, 0}};  
                    float set_coord_2_org[3] = {370.9, -419.7, 651.5};
            
                    int set_coord_2_user = Drfl.ConfigUserCoordinateSystemEx(set_coord_2_vec, set_coord_2_org);
    */
                    cout << "ConfigUserCoordinateSystem" << endl;
                    float set_coord_3_x1[6] = {0,500,700,0,0,0};
                    float set_coord_3_x2[6] = {500,0,700,0,0,0};
                    float set_coord_3_x3[6] = {300,100,500,0,0,0};
                    float set_coord_3_org[3] = {10, 20, 30};
                    float set_coord_3_x[3][6] = {{0,500,700,0,0,0}, {500,0,700,0,0,0}, {300,100,500,0,0,0}};
                    //int set_coord_3_id = Drfl.ConfigUserCoordinateSystem(set_coord_3_x, set_coord_3_org, COORDINATE_SYSTEM_BASE);
                    /*
                    cout << "UpdateUserCoordinate" << endl;

                    float update_coord_pos2[6] = {100, 150, 200, 45, 180, 0};
                    int update_coord_result = Drfl.overwrite_user_cart_coord(1, 102, update_coord_pos2);
             
                    cout << update_coord_result << endl;*/

                    cout << "GetUserCoordinate" << endl;
                    float get_user_coordinate_pos[6] = {10, 20, 30, 0, 0, 0};
                    int get_user_coordinate_id = Drfl.set_user_cart_coord(0, get_user_coordinate_pos);
                    USER_COORDINATE *get_user_coordinate_temp = Drfl.get_user_cart_coord(get_user_coordinate_id);
                    for(int i=0; i<6; i++){
                    cout << get_user_coordinate_temp->_fTargetPos[i] << endl;
                    }
                    cout << (int)get_user_coordinate_temp->_iReqId << endl;
                    cout << (int)get_user_coordinate_temp->_iTargetRef << endl;

                    Drfl.movej(ready_pos, 60, 30);

                    cout << "SetDesiredForce" << endl;
                    Drfl.set_ref_coord(COORDINATE_SYSTEM_TOOL);
                    float set_desired_force_x0[6] = {0, 0, 90, 0, 90, 0};
                    Drfl.MoveJ(set_desired_force_x0, 60, 30);
                    Drfl.MoveWait();
                    float set_desired_force_stx[6] = {500, 500, 500, 100, 100, 100};
                    Drfl.task_compliance_ctrl(set_desired_force_stx);
                    float set_desired_force_fd[6] = {0, 0, 0, 0, 0, 10};
                    unsigned char set_desired_force_fctrl_dir[6] = {0, 0 ,1, 0, 0, 1};
                    bool set_desired_force_res = Drfl.set_desired_force(set_desired_force_fd, set_desired_force_fctrl_dir);
                    cout << set_desired_force_res << endl;
                    cout << "ResetDesiredForce" << endl;
                    bool reset_desired_force_res = Drfl.release_force();
                    cout << reset_desired_force_res<< endl;
                    Drfl.release_compliance_ctrl();
    
                    cout << "WaitForPositionCondition" << endl;
                    bool wait_for_position_CON1 = Drfl.check_position_condition_abs(FORCE_AXIS_X, -5, 0, COORDINATE_SYSTEM_WORLD);
                    bool wait_for_position_CON2 = Drfl.check_position_condition_abs(FORCE_AXIS_Y, -10000, 700);
                    cout << wait_for_position_CON1 << endl;
                    cout << wait_for_position_CON2 << endl;

                    cout << "WaitForPositionConditionRel" << endl;
                    float wait_for_position_rel_posx1[6] = {400, 500, 800, 0, 180, 0};
                    bool wait_for_position_rel_CON3 = Drfl.check_position_condition_rel(FORCE_AXIS_Z, -10, -5, wait_for_position_rel_posx1, COORDINATE_SYSTEM_TOOL);
                    cout << wait_for_position_rel_CON3 << endl;

                    cout << "WaitForForceCondition" << endl;   
                    bool wait_for_force_fcon1 = Drfl.check_force_condition(FORCE_AXIS_Z, 5, 10, COORDINATE_SYSTEM_WORLD);
                    cout << wait_for_force_fcon1 << endl;
 
                    cout << "WaitForOrientationCondition" << endl;
                    float wait_for_orientation_posx1[6] = {400, 500, 800, 0, 180, 30};
                    float wait_for_orientation_posx2[6] = {400, 500, 500, 0, 180, 60};
                    bool wait_for_orientation_con1 = Drfl.check_orientation_condition(FORCE_AXIS_C, wait_for_orientation_posx1, wait_for_orientation_posx2);
                    cout << wait_for_orientation_con1 << endl;

                    cout << "WaitForOrientationConditionRel" << endl;
                    float wait_for_orientation_rel_posx1[6] = {400, 500, 800, 0, 180, 30};
                    bool wait_for_orientation_rel_con1 = Drfl.check_orientation_condition(FORCE_AXIS_C, 0, 5, wait_for_orientation_rel_posx1);

                    cout << "TransformCoordinateSystem" << endl;
                    float trans_coord_base_pos[6] = {400, 500, 800, 0, 180, 15};
                    ROBOT_POSE* trans_coord_tool_pos;
                    trans_coord_tool_pos = Drfl.coord_transform(trans_coord_base_pos, COORDINATE_SYSTEM_BASE, COORDINATE_SYSTEM_TOOL);
                    for(int i=0; i<6; i++)
                    {
                        cout << trans_coord_tool_pos->_fPosition[i] << endl;
                    }        
        
                    cout << "CalCurrentTaskPose" << endl;
                    ROBOT_TASK_POSE* cal_current_posx_result = Drfl.get_current_posx();
                    float* cal_current_posx_pos = new float[NUM_TASK];                
                    int cal_current_posx_sol = cal_current_posx_result->_iTargetSol;                
                    memcpy(cal_current_posx_pos, cal_current_posx_result->_fTargetPos, sizeof(float) * NUM_TASK);
                    for(int i=0; i<6; i++){
                        cout << cal_current_posx_pos[i] << endl;
                    }
                    cout << cal_current_posx_sol << endl;

                    cout << "CalDesiredTaskPose" << endl;
                    ROBOT_POSE* cal_target_posx_result = Drfl.get_desired_posx();
                    for(int i = 0; i < NUM_TASK; i++)
                    {
                        cout << cal_target_posx_result->_fPosition[i] << endl;
                    }
                    
                    cout << "GetSolutionSpace" << endl;
                    float get_solution_space_p1[6] = {0, 0, 0, 0, 0, 0};
                    int get_solution_space_sol = Drfl.get_solution_space(get_solution_space_p1);
                    cout << get_solution_space_sol << endl;
                
                    cout << "CalOrientationError" << endl;
                    float cal_orientation_err_x1[6] = {0, 0, 0, 0, 0, 0};
                    float cal_orientation_err_x2[6] = {10, 20, 30, 40, 50, 60};
                    float cal_orientation_err_diff = Drfl.get_orientation_error(cal_orientation_err_x1, cal_orientation_err_x2, TASK_AXIS_X);
                    cout << cal_orientation_err_diff << endl;

                    cout << "GetControlMode" << endl;
                    CONTROL_MODE get_control_mode =Drfl.get_control_mode();
                    cout << get_control_mode << endl;

                    cout << "GetCurrentRotationMatrix" << endl;
                    float(*get_current_rotm_result)[3] = Drfl.get_current_rotm();
                    for (int i=0; i<3; i++)
                    {
                        for (int j=0; j<3; j++)
                        {
                            cout << get_current_rotm_result[i][j] << " "  ;
                        }
                        cout << endl;
                    }
                }
            }
            break;	
        case '9':
            {
                float fCog[3] = {10, 10, 10};
                float fInertia[6] = {10, 10, 10, 10, 10, 10};
                Drfl.add_tool("tool2", 4, fCog, fInertia);
            }
            break;
        case 'a':
            {
                Drfl.set_tool("tool2");
                Sleep(1000);
                cout << Drfl.get_tool() << endl;
            }
            break;
        default:
            break;
        }
        Sleep(100);
    }

    Drfl.CloseConnection();
    
	return 0;
}

