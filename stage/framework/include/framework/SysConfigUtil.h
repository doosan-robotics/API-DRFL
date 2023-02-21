/*
 * SysConfigUtil.h
 *
 * This class is set of system configuration utility for DRCF.
 *
 *  Created on: 2016. 4. 4.
 *      Author: myungjin.kim
 */

#pragma once
#include <stddef.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#ifdef __XENO__
#include <native/timer.h>
#include <rtdk.h>
#endif

#include "../TaskIntegration/TI4UserOperation/TpSocket/TpDefine.h"

namespace DRAFramework {

    class CSysConfigUtil
    {
    public:
        ////////////////////////////////////////////////////////////////////////////
        // Constructors / Destructor                                              //
        ////////////////////////////////////////////////////////////////////////////
        CSysConfigUtil();
        virtual~CSysConfigUtil();

        ////////////////////////////////////////////////////////////////////////////
        // Operations                                                             //
        ////////////////////////////////////////////////////////////////////////////

        enum { USB_DISABLE, USB_ENABLE, USB_RESET }; 
        enum { REAL_IP, NETPLAN_IP};
#ifdef DB_ACCESS_AUTH
        static void DBAcessAuthorization();
#endif
        static void UsbReset(int nResetType = USB_RESET);
        static void ModbusAuthorization();
        static void IEAuthorization();
        static void LogFolderAuthorization();
        static void TimeSynchronized(bool flag);
        static int  SetSystemFileSystmeCheckFix(void);
        static void SyncFileToDisk();
        static void GetSerialPortName(LPSERIAL_SEARCH pSerialSearch);
        static void ConnectingUsbtethering();
#ifdef FUNC_PY_PACKAGE_UPDATE
        static void InstallPyPackage(vector<string> &libList, vector<string> &pyPackageList);
#endif
#ifdef _FUNC_IE_IP_ADDRESS_APPLY
        static int SetSystemIpAddress(SYSTEM_IPADDRESS param, char* lpszIeIP = NULL);
        static int GetSystemIpAddress(SYSTEM_IPADDRESS *pParam, int nIpType = REAL_IP , char* lpszIeIP = NULL);
#endif
#if defined (_FUNC_NETWORK_RUNNING_CHECK) && defined(__XENO__)
        static int GetNetworkDeviceStatus(char* pNetworkDevice);
        static void CheckPreparationCompleteOfNetworkDevice(char* pNetworkDevice);
#endif
        static int SetSystemTime(SYSTEM_TIME &timeData);
        static int GetSystemTime(SYSTEM_TIME *pTimeData);
        static int GetSystemTime(char* lpszDateTime);
        static int GetSystemTime(long &sec, long &usec);
        static int GetSystemTime(double &t);

        static void ExitSystem(void);
        static void RebootSystem(void);
        static int GetFreeSizeOfDisk(LPSYSTEM_DISKSIZE pSysDiskSize);
        static int GetUsageOfCPU(char* pszPorcessName, LPSYSTEM_CPUUSAGE pSysCpuUsage);
        static void GetExecuteTime(bool flag);
#if 0
        static void ClenanUpFile();
#endif

        static void RunSystem(char* szCommand);
#if defined (__XENO__)
        static double GetRealtimeTick();
#endif
        static void GetSmartVisionVersion();
    private:
#if defined (OS_LINUX)
#if defined (__XENO__)
        static RTIME preTime;
#endif
#else
        static __int64 preTime;
        static __int64 freq;
#endif
    };
}
