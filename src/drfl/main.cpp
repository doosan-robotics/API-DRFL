// DRFTWin32.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#ifdef __XENO__
#include <rtdk.h>
#include <native/task.h>
#include <native/timer.h>
#else
#include "stdafx.h"
#include <Windows.h>
#include <process.h>
#include <conio.h>
#endif // __XENO__
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>

//#include "Eigen/Core"
//#include "Eigen/Dense"
//#include "Eigen/Geometry"
#include <cmath>

#define M_PI		3.14159265358979323846
#define MMETER          (1e-3)
const double DEGREE  = (M_PI/180);
const double RADIAN  = (180/M_PI);

#include "drfl/DRFLEx.h"
using namespace DRAFramework;

#undef NDEBUG
#include <assert.h>

CDRFLEx Drfl;
bool g_bHasControlAuthority = FALSE;
bool g_TpInitailizingComplted = FALSE;
bool g_mStat = FALSE;
bool g_Stop = FALSE;
bool moving = FALSE;
string strDrl =
    "\r\n\
loop = 0\r\n\
while loop < 1003:\r\n\
 movej(posj(10,10.10,10,10.10), vel=60, acc=60)\r\n\
 movej(posj(00,00.00,00,00.00), vel=60, acc=60)\r\n\
 loop+=1\r\n";

bool bAlterFlag = FALSE;

int linux_kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;

	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );


	ch = getchar();

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}

int getch()
{
    int c;
    struct termios oldattr, newattr;

    tcgetattr(STDIN_FILENO, &oldattr);           // 현재 터미널 설정 읽음
    newattr = oldattr;
    newattr.c_lflag &= ~(ICANON | ECHO);         // CANONICAL과 ECHO 끔
    newattr.c_cc[VMIN] = 1;                      // 최소 입력 문자 수를 1로 설정
    newattr.c_cc[VTIME] = 0;                     // 최소 읽기 대기 시간을 0으로 설정
    tcsetattr(STDIN_FILENO, TCSANOW, &newattr);  // 터미널에 설정 입력
    c = getchar();                               // 키보드 입력 읽음
    tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);  // 원래의 설정으로 복구
    return c;
}

void OnTpInitializingCompleted() {
  // Tp 초기화 이후 제어권 요청.
  g_TpInitailizingComplted = TRUE;
  Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
}

void OnHommingCompleted() {
  // 50msec 이내 작업만 수행할 것.
  cout << "homming completed" << endl;
}

void OnProgramStopped(const PROGRAM_STOP_CAUSE) {
  assert(Drfl.PlayDrlStop(STOP_TYPE_SLOW));
  // 50msec 이내 작업만 수행할 것.
  // assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
  cout << "program stopped" << endl;
}

void OnMonitoringDataCB(const LPMONITORING_DATA pData) {
  // 50msec 이내 작업만 수행할 것.

  return;
  cout << "# monitoring 0 data " << pData->_tCtrl._tTask._fActualPos[0][0]
       << pData->_tCtrl._tTask._fActualPos[0][1]
       << pData->_tCtrl._tTask._fActualPos[0][2]
       << pData->_tCtrl._tTask._fActualPos[0][3]
       << pData->_tCtrl._tTask._fActualPos[0][4]
       << pData->_tCtrl._tTask._fActualPos[0][5] << endl;
}

void OnMonitoringDataExCB(const LPMONITORING_DATA_EX pData) {
  return;
  cout << "# monitoring 1 data " << pData->_tCtrl._tWorld._fTargetPos[0]
       << pData->_tCtrl._tWorld._fTargetPos[1]
       << pData->_tCtrl._tWorld._fTargetPos[2]
       << pData->_tCtrl._tWorld._fTargetPos[3]
       << pData->_tCtrl._tWorld._fTargetPos[4]
       << pData->_tCtrl._tWorld._fTargetPos[5] << endl;
}

void OnMonitoringCtrlIOCB(const LPMONITORING_CTRLIO pData) {
  return;
  cout << "# monitoring ctrl 0 data" << endl;
  for (int i = 0; i < 16; i++) {
    cout << (int)pData->_tInput._iActualDI[i] << endl;
  }
}

void OnMonitoringCtrlIOExCB(const LPMONITORING_CTRLIO_EX pData) {
  return;
  cout << "# monitoring ctrl 1 data" << endl;
  for (int i = 0; i < 16; i++) {
    cout << (int)pData->_tInput._iActualDI[i] << endl;
  }
  for (int i = 0; i < 16; i++) {
    cout << (int)pData->_tOutput._iTargetDO[i] << endl;
  }
}

void OnMonitoringStateCB(const ROBOT_STATE eState) {
  // 50msec 이내 작업만 수행할 것.
  switch ((unsigned char)eState) {
#if 0  // TP 초기화시 사용하는 로직임으로 API 레벨에서는 사용하지 말것.(TP없이
       // 단독 사용일 경우, 사용)
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
      // cout << "STATE_SAFE_OFF1" << endl;
      if (g_bHasControlAuthority) {
        // cout << "STATE_SAFE_OFF2" << endl;
        Drfl.SetRobotControl(CONTROL_SERVO_ON);
      }
      break;
    case STATE_SAFE_STOP2:
      if (g_bHasControlAuthority)
        Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_STOP);
      break;
    case STATE_SAFE_OFF2:
      if (g_bHasControlAuthority) {
        Drfl.SetRobotControl(CONTROL_RECOVERY_SAFE_OFF);
      }
      break;
    case STATE_RECOVERY:
      // Drfl.SetRobotControl(CONTROL_RESET_RECOVERY);
      break;
    default:
      break;
  }
  return;
  cout << "current state: " << (int)eState << endl;
}

void OnMonitroingAccessControlCB(
    const MONITORING_ACCESS_CONTROL eTrasnsitControl) {
  // 50msec 이내 작업만 수행할 것.

  switch (eTrasnsitControl) {
    case MONITORING_ACCESS_CONTROL_REQUEST:
      assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_NO));
      // Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_RESPONSE_YES);
      break;
    case MONITORING_ACCESS_CONTROL_GRANT:
      g_bHasControlAuthority = TRUE;
      // cout << "GRANT1" << endl;
      // cout << "MONITORINGCB : " << (int)Drfl.GetRobotState() << endl;
      OnMonitoringStateCB(Drfl.GetRobotState());
      // cout << "GRANT2" << endl;
      break;
    case MONITORING_ACCESS_CONTROL_DENY:
    case MONITORING_ACCESS_CONTROL_LOSS:
      g_bHasControlAuthority = FALSE;
      if (g_TpInitailizingComplted) {
        // assert(Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_REQUEST));
        Drfl.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
      }
      break;
    default:
      break;
  }
}

void OnLogAlarm(LPLOG_ALARM tLog) {
  g_mStat = true;
  cout << "Alarm Info: "
       << "group(" << (unsigned int)tLog->_iGroup << "), index("
       << tLog->_iIndex << "), param(" << tLog->_szParam[0] << "), param("
       << tLog->_szParam[1] << "), param(" << tLog->_szParam[2] << ")" << endl;
}

void OnTpPopup(LPMESSAGE_POPUP tPopup) {
  cout << "Popup Message: " << tPopup->_szText << endl;
  cout << "Message Level: " << tPopup->_iLevel << endl;
  cout << "Button Type: " << tPopup->_iBtnType << endl;
}

void OnTpLog(const char* strLog) { cout << "Log Message: " << strLog << endl; }

void OnTpProgress(LPMESSAGE_PROGRESS tProgress) {
  cout << "Progress cnt : " << (int)tProgress->_iTotalCount << endl;
  cout << "Current cnt : " << (int)tProgress->_iCurrentCount << endl;
}

void OnTpGetuserInput(LPMESSAGE_INPUT tInput) {
  cout << "User Input : " << tInput->_szText << endl;
  cout << "Data Type : " << (int)tInput->_iType << endl;
}

void OnRTMonitoringData(LPRT_OUTPUT_DATA_LIST tData)
{
	return;
    static int td = 0;
    if (td++ == 1000) {
    	td = 0;
    	printf("timestamp : %.3f\n", tData->time_stamp);
    	printf("joint : %f %f %f %f %f %f\n", tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2], tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
		printf("q = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
				tData->actual_joint_position[0], tData->actual_joint_position[1], tData->actual_joint_position[2],
				tData->actual_joint_position[3], tData->actual_joint_position[4], tData->actual_joint_position[5]);
		printf("q_dot = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
				tData->actual_joint_velocity[0], tData->actual_joint_velocity[1], tData->actual_joint_velocity[2],
				tData->actual_joint_velocity[3], tData->actual_joint_velocity[4], tData->actual_joint_velocity[5]);
		printf("trq_g = %7.4f, %7.4f, %7.4f, %7.4f, %7.4f, %7.4f\n",
				tData->gravity_torque[0], tData->gravity_torque[1], tData->gravity_torque[2],
				tData->gravity_torque[3], tData->gravity_torque[4], tData->gravity_torque[5]);
    }
}


uint32_t ThreadFunc(void* arg) {
	printf("start ThreadFunc\n");

	while (true) {
		if(linux_kbhit()){
			char ch = getch();
			switch (ch) {
				case 's': {
					printf("Stop!\n");
					g_Stop = true;
					Drfl.MoveStop(STOP_TYPE_SLOW);
				} break;
				case 'p': {
					printf("Pause!\n");
					Drfl.MovePause();
				} break;
				case 'r': {
					printf("Resume!\n");
					Drfl.MoveResume();
				} break;
			}
		}

		//Sleep(100);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	std::cout << "exit ThreadFunc" << std::endl;

	return 0;
}

void OnDisConnected() {
  while (!Drfl.open_connection("192.168.137.100")) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

struct PlanParam
{
	float time;

	float ps[6];
	float vs[6];
	float as[6];
	float pf[6];
	float vf[6];
	float af[6];

	float A0[6];
	float A1[6];
	float A2[6];
	float A3[6];
	float A4[6];
	float A5[6];
};

struct TraParam
{
	float time;

	float pos[6];
	float vel[6];
	float acc[6];
};

void TrajectoryPlan(PlanParam* plan)
{
    float ps[6],vs[6],as[6];
    float pf[6],vf[6],af[6];
    float tf;

	tf = plan->time;

    for(int i=0; i<6; i++)
    {
        ps[i] = plan->ps[i];
        vs[i] = plan->vs[i];
        as[i] = plan->as[i];
        pf[i] = plan->pf[i];
        vf[i] = plan->vf[i];
        af[i] = plan->af[i];
    }

    for(int i=0; i<6; i++)
    {
        plan->A0[i] = ps[i];
        plan->A1[i] = vs[i];
        plan->A2[i] = as[i]/2;
        plan->A3[i] = (20*pf[i]-20*ps[i]-(8*vf[i]+12*vs[i])*tf-(3*as[i]-af[i])*tf*tf)/(2*tf*tf*tf);
        plan->A4[i] = (30*ps[i]-30*pf[i]+(14*vf[i]+16*vs[i])*tf+(3*as[i]-2*af[i])*tf*tf)/(2*tf*tf*tf*tf);
        plan->A5[i] = (12*pf[i]-12*ps[i]-(6*vf[i]+6*vs[i])*tf-(as[i]-af[i])*tf*tf)/(2*tf*tf*tf*tf*tf);
    }
}

void TrajectoryGenerator(PlanParam *plan, TraParam *tra)
{
    double A0[6],A1[6],A2[6],A3[6],A4[6],A5[6];
	double t = tra->time;

    for(int i=0; i<6; i++)
    {
        A0[i] = plan->A0[i];
        A1[i] = plan->A1[i];
        A2[i] = plan->A2[i];
        A3[i] = plan->A3[i];
        A4[i] = plan->A4[i];
        A5[i] = plan->A5[i];
    }

    for(int i=0; i<6; i++)
    {
        tra->pos[i] = A0[i] + A1[i]*t + A2[i]*t*t + A3[i]*t*t*t + A4[i]*t*t*t*t + A5[i]*t*t*t*t*t;
        tra->vel[i] = A1[i] + 2*A2[i]*t + 3*A3[i]*t*t + 4*A4[i]*t*t*t + 5*A5[i]*t*t*t*t;
        tra->acc[i] = 2*A2[i] + 6*A3[i]*t + 12*A4[i]*t*t + 20*A5[i]*t*t*t;
    }
}

void MoveJ_ServoJ_RT(float start_posj[6], float target_posj[6], float target_time, float sampling_time)
{
	const float None=-10000;
	const float st=sampling_time; // sampling time

	float count=0;
	float time=0;

	TraParam tra;

	// Plan1
	PlanParam plan1;
	plan1.time=target_time;
	memcpy(plan1.ps, start_posj, NUMBER_OF_JOINT*sizeof(float));
	memcpy(plan1.pf, target_posj, NUMBER_OF_JOINT*sizeof(float));

	plan1.vs[0]=0; plan1.vs[1]=0; plan1.vs[2]=0; plan1.vs[3]=0; plan1.vs[4]=0; plan1.vs[5]=0;
	plan1.vf[0]=0; plan1.vf[1]=0; plan1.vf[2]=0; plan1.vf[3]=0; plan1.vf[4]=0; plan1.vf[5]=0;
	plan1.as[0]=0; plan1.as[1]=0; plan1.as[2]=0; plan1.as[3]=0; plan1.as[4]=0; plan1.as[5]=0;
	plan1.af[0]=0; plan1.af[1]=0; plan1.af[2]=0; plan1.af[3]=0; plan1.af[4]=0; plan1.af[5]=0;
	TrajectoryPlan(&plan1);

	printf("Start MoveJ_ServoJ_RT motion, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
			plan1.ps[0],plan1.ps[1],plan1.ps[2],
			plan1.ps[3],plan1.ps[4],plan1.ps[5]);

	static float target_posj_old[6];
	memcpy(target_posj_old, plan1.ps, NUMBER_OF_JOINT*sizeof(float));

	while(1)
	{
		time=(++count)*st;
		tra.time=time;

		TrajectoryGenerator(&plan1,&tra);

		if(sampling_time < 0.002)
		{
			for(int i=0; i<6; i++)
			{
				//tra.vel[i]=(tra.pos[i]-target_posj_old[i])/sampling_time;
				tra.vel[i]=0.0;
				tra.acc[i]=None;
			}
		}
		else
		{
			for(int i=0; i<6; i++)
			{
				//tra.vel[i]=None;
				//tra.acc[i]=None;
			}
		}

		Drfl.servoj_rt(tra.pos, tra.vel, tra.acc, st);

		memcpy(target_posj_old, tra.pos, NUMBER_OF_JOINT*sizeof(float));

		if(time > plan1.time)
		{
			time=0;
//			Drfl.stop(STOP_TYPE_SLOW);
			printf("Finish MoveJ_ServoJ_RT with servoj_RT, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					target_posj[0],target_posj[1],target_posj[2],
					target_posj[3],target_posj[4],target_posj[5]);


			return;
		}

		rt_task_wait_period(NULL);
	}
}

void MoveL_ServoL_RT(float start_posx[6], float target_posx[6], float target_time, float sampling_time)
{
	const float None=-10000;
	const float st=sampling_time; // sampling time

	float count=0;
	float time=0;

	TraParam tra;

	// Plan1
	PlanParam plan1;
	plan1.time=target_time;
	memcpy(plan1.ps, start_posx, NUMBER_OF_JOINT*sizeof(float));
	memcpy(plan1.pf, target_posx, NUMBER_OF_JOINT*sizeof(float));

	plan1.vs[0]=0; plan1.vs[1]=0; plan1.vs[2]=0; plan1.vs[3]=0; plan1.vs[4]=0; plan1.vs[5]=0;
	plan1.vf[0]=0; plan1.vf[1]=0; plan1.vf[2]=0; plan1.vf[3]=0; plan1.vf[4]=0; plan1.vf[5]=0;
	plan1.as[0]=0; plan1.as[1]=0; plan1.as[2]=0; plan1.as[3]=0; plan1.as[4]=0; plan1.as[5]=0;
	plan1.af[0]=0; plan1.af[1]=0; plan1.af[2]=0; plan1.af[3]=0; plan1.af[4]=0; plan1.af[5]=0;
	TrajectoryPlan(&plan1);

	printf("Start MoveL_ServoL_RT motion, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
			plan1.ps[0],plan1.ps[1],plan1.ps[2],
			plan1.ps[3],plan1.ps[4],plan1.ps[5]);

	static float target_posx_old[6];
	memcpy(target_posx_old, plan1.ps, NUMBER_OF_JOINT*sizeof(float));

	while(1)
	{
		time=(++count)*st;
		tra.time=time;

		TrajectoryGenerator(&plan1,&tra);

		if(sampling_time < 0.002)
		{
			for(int i=0; i<6; i++)
			{
				//tra.vel[i]=(tra.pos[i]-target_posx_old[i])/sampling_time;
				tra.vel[i]=0.0;
				tra.acc[i]=None;
			}
		}
		else
		{
			for(int i=0; i<6; i++)
			{
				//tra.vel[i]=None;
				//tra.acc[i]=None;
			}
		}
		Drfl.servol_rt(tra.pos, tra.vel, tra.acc, st);

		memcpy(target_posx_old, tra.pos, NUMBER_OF_JOINT*sizeof(float));

		if(time > plan1.time)
		{
			time=0;
//			Drfl.stop(STOP_TYPE_SLOW);
			printf("Finish MoveL_ServoL_RT motion, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					target_posx[0],target_posx[1],target_posx[2],
					target_posx[3],target_posx[4],target_posx[5]);


			return;
		}

		rt_task_wait_period(NULL);
	}
}

void MoveJ_SpeedJ(float target_posj[6], float target_time)
{
	const float None=-10000;
	float vel[6] = {120, 120, 120, 120, 120, 120};
	float acc[6] = {120, 120, 120, 120, 120, 120};

	Drfl.set_velj_rt(vel);
	Drfl.set_accj_rt(acc);
	Drfl.set_velx_rt(500);
	Drfl.set_accx_rt(1000);

	const float st=0.001; // sampling time
	float count=0;
	float time=0;

	TraParam tra;

	// Plan1
	PlanParam plan1;
	plan1.time=target_time;
	memcpy(plan1.ps, Drfl.read_data_rt()->target_joint_position, NUMBER_OF_JOINT*sizeof(float));
	memcpy(plan1.pf, target_posj, NUMBER_OF_JOINT*sizeof(float));

	plan1.vs[0]=0; plan1.vs[1]=0; plan1.vs[2]=0; plan1.vs[3]=0; plan1.vs[4]=0; plan1.vs[5]=0;
	plan1.vf[0]=0; plan1.vf[1]=0; plan1.vf[2]=0; plan1.vf[3]=0; plan1.vf[4]=0; plan1.vf[5]=0;
	plan1.as[0]=0; plan1.as[1]=0; plan1.as[2]=0; plan1.as[3]=0; plan1.as[4]=0; plan1.as[5]=0;
	plan1.af[0]=0; plan1.af[1]=0; plan1.af[2]=0; plan1.af[3]=0; plan1.af[4]=0; plan1.af[5]=0;
	TrajectoryPlan(&plan1);

	printf("Start speedj motion, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
			plan1.ps[0],plan1.ps[1],plan1.ps[2],
			plan1.ps[3],plan1.ps[4],plan1.ps[5]);

	while(1)
	{
		time=(++count)*st;
		tra.time=time;

		TrajectoryGenerator(&plan1,&tra);

		Drfl.speedj_rt(tra.vel, tra.acc, st);

		if(time > plan1.time)
		{
			time=0;
			Drfl.stop(STOP_TYPE_SLOW);
			printf("Finish speedj motion, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					target_posj[0],target_posj[1],target_posj[2],
					target_posj[3],target_posj[4],target_posj[5]);
			float current_posj[6];
			memcpy(current_posj, Drfl.read_data_rt()->target_joint_position, NUMBER_OF_JOINT*sizeof(float));
			printf("	current posj, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					current_posj[0],current_posj[1],current_posj[2],
					current_posj[3],current_posj[4],current_posj[5]);

			return;
		}

		rt_task_wait_period(NULL);
	}
}

void MoveL_SpeedL(float start_posx[6], float target_posx[6], float target_time)
{
	const float None=-10000;
	float vel[6] = {120, 120, 120, 120, 120, 120};
	float acc[6] = {120, 120, 120, 120, 120, 120};

	Drfl.set_velj_rt(vel);
	Drfl.set_accj_rt(acc);
	Drfl.set_velx_rt(500);
	Drfl.set_accx_rt(1000);

	const float st=0.001; // sampling time

	float count=0;
	float time=0;

	TraParam tra1;

	// Plan1
	PlanParam plan1;
	plan1.time=target_time;
//	memcpy(plan1.ps, Drfl.read_data_rt()->target_tcp_position, NUMBER_OF_JOINT*sizeof(float));
	memcpy(plan1.ps, start_posx, NUMBER_OF_JOINT*sizeof(float));
	memcpy(plan1.pf, target_posx, NUMBER_OF_JOINT*sizeof(float));

	plan1.vs[0]=0; plan1.vs[1]=0; plan1.vs[2]=0; plan1.vs[3]=0; plan1.vs[4]=0; plan1.vs[5]=0;
	plan1.vf[0]=0; plan1.vf[1]=0; plan1.vf[2]=0; plan1.vf[3]=0; plan1.vf[4]=0; plan1.vf[5]=0;
	plan1.as[0]=0; plan1.as[1]=0; plan1.as[2]=0; plan1.as[3]=0; plan1.as[4]=0; plan1.as[5]=0;
	plan1.af[0]=0; plan1.af[1]=0; plan1.af[2]=0; plan1.af[3]=0; plan1.af[4]=0; plan1.af[5]=0;
	TrajectoryPlan(&plan1);

	printf("Start speedl motion, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
			plan1.ps[0],plan1.ps[1],plan1.ps[2],
			plan1.ps[3],plan1.ps[4],plan1.ps[5]);

	while(1)
	{
		time=(++count)*st;
		tra1.time=time;

		TrajectoryGenerator(&plan1,&tra1);

		Drfl.speedl_rt(tra1.vel, tra1.acc, st);

		if(time > plan1.time)
		{
			printf("Finish speedl motion, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					target_posx[0],target_posx[1],target_posx[2],
					target_posx[3],target_posx[4],target_posx[5]);
			float current_posx[6];
			memcpy(current_posx, Drfl.read_data_rt()->target_tcp_position, NUMBER_OF_TASK*sizeof(float));
			printf("	current posx, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
					current_posx[0],current_posx[1],current_posx[2],
					current_posx[3],current_posx[4],current_posx[5]);

			return;
		}
		rt_task_wait_period(NULL);
	}
}

int main(int argc, char** argv) {
#ifdef __XENO__
  RT_TASK main_task;
  char main_task_name[256] = {
      '\0',
  };
  sprintf(main_task_name, "drfl_main_t");
  if (rt_task_self() == 0) {
    rt_task_shadow(&main_task, main_task_name, 55, T_CPU(3));
  }
#endif  // __XENO__

#undef _RT_MONITORING_EX
#ifdef _RT_MONITORING_EX
  Drfl.set_on_rt_monitoring_data(OnRTMonitoringData);

  bool bLoop = TRUE;
  while (bLoop) {
#ifdef __XENO__
    unsigned long overrun = 0;
    const double tick = 20000000;;  // 1ms
    rt_task_set_periodic(nullptr, TM_NOW, tick);
    if (rt_task_wait_period(&overrun) == -ETIMEDOUT) {
      std::cout << __func__ << ": \x1B[37m\x1B[41mover-runs: " << overrun
                << "\x1B[0m\x1B[0K" << std::endl;
    }
#else
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
#endif  // __XENO__
#if 0
        static char ch = '0';
        if (ch == '7') ch = '0';
        else if (ch == '0') ch = '7';
#else
    cout << "\ninput key : ";
    // char ch = _getch();
    char ch;
    cin >> ch;
    cout << ch << endl;
#endif
    switch (ch) {
      case 'q':
        bLoop = FALSE;
        break;
      case '1':
          {
              //Drfl.connect_rt_control("127.0.0.1", 12348);
              Drfl.connect_rt_control();
          }
          break;
      case 'd':
          {
        	  Drfl.disconnect_rt_control();
          }
          break;
      case '2':
          {
              string version = "v1.0";
              float period = 0.001;
              int losscount = 4;
//              Drfl.set_rt_control_input(version, 1, losscount);
              Drfl.set_rt_control_output(version, period, losscount);
          }
          break;
      case '3':
          {
        	  Drfl.start_rt_control();
          }
          break;
      case '4':
		  {
			  Drfl.stop_rt_control();
		  }
		  break;
      default:
        break;
    }
	this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
#else

  // 콜백 등록(// 콜백 함수 내에서는 50msec 이내 작업만 수행할 것)
  Drfl.set_on_homming_completed(OnHommingCompleted);
  Drfl.set_on_monitoring_data(OnMonitoringDataCB);
  Drfl.set_on_monitoring_data_ex(OnMonitoringDataExCB);
  Drfl.set_on_monitoring_ctrl_io(OnMonitoringCtrlIOCB);
  Drfl.set_on_monitoring_ctrl_io_ex(OnMonitoringCtrlIOExCB);
  //Drfl.set_on_monitoring_state(OnMonitoringStateCB);
  Drfl.set_on_monitoring_access_control(OnMonitroingAccessControlCB);
  Drfl.set_on_tp_initializing_completed(OnTpInitializingCompleted);
  Drfl.set_on_log_alarm(OnLogAlarm);
  Drfl.set_on_tp_popup(OnTpPopup);
  Drfl.set_on_tp_log(OnTpLog);
  Drfl.set_on_tp_progress(OnTpProgress);
  Drfl.set_on_tp_get_user_input(OnTpGetuserInput);
  Drfl.set_on_rt_monitoring_data(OnRTMonitoringData);

  Drfl.set_on_program_stopped(OnProgramStopped);
  Drfl.set_on_disconnected(OnDisConnected);

  // 연결 수립
  assert(Drfl.open_connection("192.168.137.100"));

  // 버전 정보 획득
  SYSTEM_VERSION tSysVerion = {
      '\0',
  };
  Drfl.get_system_version(&tSysVerion);
  // 모니터링 데이터 버전 변경
  assert(Drfl.setup_monitoring_version(1));
  //Drfl.set_robot_control(CONTROL_SERVO_ON);
  //Drfl.set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX_10, TRUE);
  cout << "System version: " << tSysVerion._szController << endl;
  cout << "Library version: " << Drfl.get_library_version() << endl;

  while ((Drfl.get_robot_state() != STATE_STANDBY) || !g_bHasControlAuthority)
    // Sleep(1000);
    this_thread::sleep_for(std::chrono::milliseconds(1000));

  // 수동 모드 설정
  assert(Drfl.set_robot_mode(ROBOT_MODE_MANUAL));
  assert(Drfl.set_robot_system(ROBOT_SYSTEM_REAL));

  // Drfl.ConfigCreateModbus("mr1", "192.168.137.70", 552,
  // MODBUS_REGISTER_TYPE_HOLDING_REGISTER, 3, 5);

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

#ifdef __XENO__
  RT_TASK sub_task;
  char sub_task_name[256] = {
      '\0',
  };
  sprintf(sub_task_name, "drfl_sub_t");
  uint32_t stack_size = 1024 * 64;
  uint32_t prio = 50;
  if (rt_task_spawn(&sub_task, sub_task_name, stack_size, prio,
                    T_CPU(3) | /*T_SUSP |*/ T_JOINABLE,
                    (void (*)(void*)) & ThreadFunc, nullptr) != 0) {
    cout << "Can not create sub task" << endl;
  }
#else
  HANDLE hThread;
  DWORD dwThreadID;
  hThread = CreateThread(NULL, 0, ThreadFunc, NULL, 0, &dwThreadID);
  if (hThread == 0) {
    printf("Thread Error\n");
    return 0;
  }
#endif

  bool bLoop = TRUE;
  float k_ratio = 1.0;
  while (bLoop) {
    g_mStat = false;
    g_Stop = false;
#ifdef __XENO__
    unsigned long overrun = 0;
    const double tick = 20000000;  // 1ms
    rt_task_set_periodic(nullptr, TM_NOW, tick);
    if (rt_task_wait_period(&overrun) == -ETIMEDOUT) {
      std::cout << __func__ << ": \x1B[37m\x1B[41mover-runs: " << overrun
                << "\x1B[0m\x1B[0K" << std::endl;
    }
#else
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
#endif  // __XENO__
#if 0
        static char ch = '0';
        if (ch == '7') ch = '0';
        else if (ch == '0') ch = '7';
#else
    cout << "\ninput key : ";
    // char ch = _getch();
    char ch;
    cin >> ch;
    cout << ch << endl;
#endif

    switch (ch) {
      case 'q':
        bLoop = FALSE;
        break;
      case '0': {
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
            // assert(Drfl.SetRobotMode(ROBOT_MODE_MANUAL));
            // assert(Drfl.SetRobotSystem(ROBOT_SYSTEM_REAL));
            cout << "drl player stop" << endl;
            break;
          case EXAMPLE_GPIO:
            cout << "reset gpio" << endl;
            for (int i = 0; i < NUM_DIGITAL; i++) {
              assert(Drfl.SetCtrlBoxDigitalOutput((GPIO_CTRLBOX_DIGITAL_INDEX)i,
                                                  FALSE));
            }
            break;
          case EXAMPLE_MODBUS:
            cout << "reset modbus" << endl;
            assert(Drfl.SetModbusValue("mr1", 0));
            break;
          default:
            break;
        }
      } break;
      case '1':
          {
        	  //Drfl.connect_rt_control("127.0.0.1", 12348);
              Drfl.connect_rt_control();
          }
          break;
      case 'd':
          {
        	  Drfl.disconnect_rt_control();
          }
          break;
      case '2':
          {
              string version = "v1.0";
              float period = 0.001;
              int losscount = 4;
//              Drfl.set_rt_control_input(version, 0.001, losscount);
              Drfl.set_rt_control_output(version, period, losscount);
          }
          break;
      case '3':
          {
        	  Drfl.start_rt_control();
          }
          break;
      case '4':
		  {
			  Drfl.stop_rt_control();
		  }
		  break;
      case 'a':
		  {
			  k_ratio += 1.0;
			  printf("up ratio = %.3f", k_ratio);
		  }
		  break;
      case 'z':
		  {
			  k_ratio -= 1.0;
			  printf("down ratio = %.3f", k_ratio);
		  }
		  break;
      case '5': // servoj_rt
      {
    	  	//Drfl.set_tool("camera");
		  	float sampling_time = 0.02;
			float home[6] = {-90, 0, 90, 0, 90, 0};
			Drfl.movej(home, 30, 60);

			Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
			Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

			// move servoj
			const float None=-10000;
			float vel[6] = {50, 50, 50, 50, 50, 50};
			float acc[6] = {100, 100, 100, 100, 100, 100};

			Drfl.set_velj_rt(vel);
			Drfl.set_accj_rt(acc);
			Drfl.set_velx_rt(250);
			Drfl.set_accx_rt(1000);

			float posj_1[6];
			posj_1[0] = -90; posj_1[1] = 0; posj_1[2] = 90;
			posj_1[3] = 0; posj_1[4] = 90; posj_1[5] = 0;
			float posj_2[6];
			posj_2[0] = -80; posj_2[1] = 10; posj_2[2] = 100;
			posj_2[3] = 10; posj_2[4] = 100; posj_2[5] = 10;
			float posj_3[6];
			posj_3[0] = -100; posj_3[1] = -10; posj_3[2] = 80;
			posj_3[3] = -10; posj_3[4] = 80; posj_3[5] = -10;

			while(1)
			{
				MoveJ_ServoJ_RT(posj_1, posj_2, 3, sampling_time);
				MoveJ_ServoJ_RT(posj_2, posj_1, 3, sampling_time);
				MoveJ_ServoJ_RT(posj_1, posj_3, 3, sampling_time);
				MoveJ_ServoJ_RT(posj_3, posj_1, 3, sampling_time);
			}

			Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_STOP);
			Drfl.set_robot_mode(ROBOT_MODE_MANUAL);
      }
      break;
      case '6': // servol_rt
      {
		  	float sampling_time = 0.02;
			float home[6] = {-90, 0, 90, 0, 90, 0};
			Drfl.movej(home, 30, 60);

			Drfl.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
			Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_MOVE);

			const float None=-10000;
			float vel[6] = {50, 50, 50, 50, 50, 50};
			float acc[6] = {100, 100, 100, 100, 100, 100};

			Drfl.set_velj_rt(vel);
			Drfl.set_accj_rt(acc);
			Drfl.set_velx_rt(250);
			Drfl.set_accx_rt(1000);

			float posx_1[6], posx_2[6], posx_3[6];
			posx_1[0] = 9.9; posx_1[1] = -734; posx_1[2] = 1068.30;
			posx_1[3] = 135; posx_1[4] = -180; posx_1[5] = -135;

			posx_2[0] = -400; posx_2[1] = -734; posx_2[2] = 800;
			posx_2[3] = posx_1[3]; posx_2[4] = posx_1[4]; posx_2[5] = posx_1[5];

			posx_3[0] = 400; posx_3[1] = -734; posx_3[2] = 800;
			posx_3[3] = posx_1[3]; posx_3[4] = posx_1[4]; posx_3[5] = posx_1[5];

			MoveL_ServoL_RT(posx_1, posx_2, 4, sampling_time);

			while(1)
			{
				MoveL_ServoL_RT(posx_2, posx_3, 8, sampling_time);
				MoveL_ServoL_RT(posx_3, posx_2, 8, sampling_time);
				MoveL_ServoL_RT(posx_2, posx_3, 7, sampling_time);
				MoveL_ServoL_RT(posx_3, posx_2, 7, sampling_time);
				MoveL_ServoL_RT(posx_2, posx_3, 6, sampling_time);
				MoveL_ServoL_RT(posx_3, posx_2, 6, sampling_time);
				MoveL_ServoL_RT(posx_2, posx_3, 5, sampling_time);
				MoveL_ServoL_RT(posx_3, posx_2, 5, sampling_time);
				MoveL_ServoL_RT(posx_2, posx_3, 4, sampling_time);
				MoveL_ServoL_RT(posx_3, posx_2, 4, sampling_time);
				MoveL_ServoL_RT(posx_2, posx_3, 3, sampling_time);
				MoveL_ServoL_RT(posx_3, posx_2, 3, sampling_time);
			}

			Drfl.set_safety_mode(SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT_STOP);
			Drfl.set_robot_mode(ROBOT_MODE_MANUAL);
      }
      break;
      default:
        break;
    }
    // Sleep(100);
    this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  Drfl.disconnect_rt_control();
  Drfl.CloseConnection();

#ifdef __XENO__
  rt_task_join(&sub_task);
#endif // __XENO__


  return 0;
#endif
}
