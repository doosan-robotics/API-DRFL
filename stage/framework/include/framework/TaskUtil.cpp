/*
 * TaskUtil.cpp
 *
 *  Created on: 2017. 1. 12.
 *      Author: sungjin.kim
 */
#include "TaskUtil.h"
#include <stdio.h>
#include <stdarg.h>

#if defined (OS_LINUX)
#if defined (__XENO__)
#include <rtdk.h>
#endif
#include "Gpio/GpioUtil.h"
#endif

#include "../TaskIntegration/TI4SystemConfig/EventManager.h"

//#define EXUTION_TIME_MEASURE

using namespace DRAFramework;

#if defined (__XENO__)
SRTIME CTaskTimeMeasure::m_lTickTime[TIME_LAST] = {'\0',};
SRTIME CTaskTimeMeasure::m_lExecTime[TIME_LAST] = {'\0',};
#endif

#ifdef NEW_EVENT_LOG
void CTaskUtil::DebugString(unsigned int nMajorCategory, unsigned int nMinorCategory, const char *format, ...)
#else
void CTaskUtil::DebugString(const char *format, ...)
#endif
{
    /**
     * vsnprintf, va_start, va_end 가 thread-safety 함수임으로
     * 불필요한 동기화 객체를 삭제함
     **/

    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

/*
    CPythonManager 안에 Linux POSIX 쓰레드에서 호출 되는 함수에서
    CTaskUtil::DebugString()을 쓰게 되면 rt_task_self() 가 사용 되어 세크멘테이션 폴트 발생!
    -> 일단 원래대로 원복 by kabdol2 2017/02/15
*/

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#ifdef NEW_EVENT_LOG
        CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, nMajorCategory, nMinorCategory, szString);
#else
        CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, szString);
#endif
    }
};

#ifdef NEW_EVENT_LOG
void CTaskUtil::TraceString(unsigned int nMajorCategory, unsigned int nMinorCategory, const char *format, ...)
#else
void CTaskUtil::TraceString(const char *format, ...)
#endif
{
    /**
     * vsnprintf, va_start, va_end 가 thread-safety 함수임으로
     * 불필요한 동기화 객체를 삭제함
     **/

    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

/*
    CPythonManager 안에 Linux POSIX 쓰레드에서 호출 되는 함수에서
    CTaskUtil::DebugString()을 쓰게 되면 rt_task_self() 가 사용 되어 세크멘테이션 폴트 발생!
    -> 일단 원래대로 원복 by kabdol2 2017/02/15
*/

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#ifdef NEW_EVENT_LOG
        CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, nMajorCategory, nMinorCategory, szString);
#else
        CEventManager::LogTrace(eLOG_GROUP_SYSTEMFMK, szString);
#endif
    }
}


//void CTaskUtil::DebugString(const char *format, ...)
//{
//
//}
//void CTaskUtil::TraceString(const char *format, ...)
//{
//
//}

void CTaskUtil::DebugStringUserTpServer(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_USER, eLOG_CT_FW_MINOR_TCP_SERVER, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringUserMBusSlave(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_USER, eLOG_CT_FW_MINOR_MODBUS_SLAVE, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}
void CTaskUtil::DebugStringUserIE(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_USER,eLOG_CT_FW_MINOR_IE, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}
void CTaskUtil::DebugStringUser5gTelecom(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_USER,eLOG_CT_FW_MINOR_5G_TELECOM, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}
void CTaskUtil::DebugStringUserUdpServer(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_USER,eLOG_CT_FW_MINOR_UDP_SERVER, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringCtrlInterpreter(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_CONTROL, eLOG_CT_FW_MINOR_INTERPRETER, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringCtrlMotion(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_CONTROL, eLOG_CT_FW_MINOR_MOTION, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringCtrlProgram(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_CONTROL, eLOG_CT_FW_MINOR_PROGRAM, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringCtrlIo(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_CONTROL, eLOG_CT_FW_MINOR_IO, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringCtrlDIo(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };
    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);
#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_CONTROL, eLOG_CT_FW_MINOR_DIO, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}
void CTaskUtil::DebugStringCtrlAIo(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };
    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);
#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_CONTROL, eLOG_CT_FW_MINOR_AIO, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}
void CTaskUtil::DebugStringCtrlConfig(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_CONTROL, eLOG_CT_FW_MINOR_CONFIG, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringCtrlState(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_CONTROL, eLOG_CT_FW_MINOR_STATE, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringDeviceMBusMaster(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_DEVICE, eLOG_CT_FW_MINOR_MODBUS_MASTER, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringDeviceECAT(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_DEVICE, eLOG_CT_FW_MINOR_ETHERCAT, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringDeviceSerial(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_DEVICE, eLOG_CT_FW_MINOR_SERIAL, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStringSysConfig(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_SYSTEM, eLOG_CT_FW_MINOR_SYSTEM_CONFIG, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}
void CTaskUtil::DebugStringSysFileManager(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_SYSTEM, eLOG_CT_FW_MINOR_FILE_MANAGER, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStrinFrwTask(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_FRAMEWORK, eLOG_CT_FW_MINOR_TASK, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStrinFrwFile(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_FRAMEWORK, eLOG_CT_FW_MINOR_FILE, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStrinFrwMBus(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_FRAMEWORK, eLOG_CT_FW_MINOR_MODBUS, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStrinFrwSerial(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_FRAMEWORK, eLOG_CT_FW_MINOR_SERIAL, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStrinFrwTFTP(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_FRAMEWORK, eLOG_CT_FW_MINOR_TFTP, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStrinFrwSystem(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_FRAMEWORK, eLOG_CT_FW_MINOR_SYSTEM, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStrinFrwIPC(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_FRAMEWORK, eLOG_CT_FW_MINOR_IPC, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStrinFrwECAT(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_FRAMEWORK, eLOG_CT_FW_MINOR_ETHERCAT, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStrinFrwParser(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_FRAMEWORK, eLOG_CT_FW_MINOR_PARSER, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

void CTaskUtil::DebugStrinFrwSocket(const char *format, ...)
{
    va_list   args;
    char szString[LOG_STRING_SIE]     = {'\0', };

    va_start(args, format);
    vsnprintf(szString, LOG_STRING_SIE, format, args);
    va_end(args);

#ifdef __XENO__
    if(rt_task_self() == 0) {
         rt_printf(szString);
    }
    else
#endif
    {
#if defined(NEW_EVENT_LOG)
    CEventManager::LogDebug(eLOG_GROUP_SYSTEMFMK, eLOG_CT_FW_MAJOR_FRAMEWORK, eLOG_CT_FW_MINOR_SOCKET, szString);
#else
    CTaskUtil::DebugString(szString);
#endif
    }
}

int CTaskUtil::GetAsrockGpio(int nPin)
{
    int nInput = 0;
#ifdef __XENO__
    AsrLibGetGpioValue(nPin, &nInput);
#endif
    return nInput;
}

void CTaskUtil::SetAsrockGpio(int nPin, int nValue)
{
#ifdef __XENO__
    AsrLibSetGpioValue(nPin, nValue);
#endif
}

void CTaskTimeMeasure::BeginEtherCATTime()
{
#if defined (__XENO__) && defined (EXUTION_TIME_MEASURE)
    m_lTickTime[TIME_ECAT] = rt_timer_read();
#endif
}

void CTaskTimeMeasure::EndEtherCATTime()
{
#if defined (__XENO__) && defined (EXUTION_TIME_MEASURE)
    SRTIME lExeTime = (rt_timer_read() - m_lTickTime[TIME_ECAT]);

    if (lExeTime >  m_lExecTime[TIME_ECAT]) {
        CTaskUtil::DebugStrinFrwTask("##Max Processing EtherCAT Time: %lld\n", lExeTime);
        m_lExecTime[TIME_ECAT] = lExeTime;
        if (lExeTime > 200000) m_lExecTime[TIME_ECAT] = 150000;
    }
#endif
}

void CTaskTimeMeasure::BeginSyncTime()
{
#if defined (__XENO__) && defined (EXUTION_TIME_MEASURE)
    SRTIME lTickTime = rt_timer_read();

    if ((lTickTime - m_lTickTime[TIME_SYNC]) > 1050000) {
        CTaskUtil::DebugStrinFrwTask("##Max Sync Signal Space(%lld)", (lTickTime - m_lTickTime[TIME_SYNC]));
    }
    m_lTickTime[TIME_SYNC] = lTickTime;
#endif
}

void CTaskTimeMeasure::EndSyncTime()
{
#if defined (__XENO__) && defined (EXUTION_TIME_MEASURE)
#endif
}

void CTaskTimeMeasure::BeginReadTime()
{
#if defined (__XENO__) && defined (EXUTION_TIME_MEASURE)
    SRTIME lTickTime = rt_timer_read();

    if ((lTickTime - m_lTickTime[TIME_INPT]) > 1050000) {
        CTaskUtil::DebugStrinFrwTask("##Max Read Signal Space(%lld)", (lTickTime - m_lTickTime[TIME_INPT]));
    }
    m_lTickTime[TIME_INPT] = lTickTime;
#endif
}

void CTaskTimeMeasure::EndReadTime()
{
#if defined (__XENO__) && defined (EXUTION_TIME_MEASURE)
    SRTIME lExeTime = (rt_timer_read() - m_lTickTime[TIME_INPT]);

    if (lExeTime >  m_lExecTime[TIME_INPT]) {
        CTaskUtil::DebugStrinFrwTask("##Max Processing Read Time: %lld\n", lExeTime);
        m_lExecTime[TIME_INPT] = lExeTime;
        if (lExeTime > 100000) m_lExecTime[TIME_INPT] = 50000;
        //if (lExeTime > 300000) m_lExecTime = 250000;
    }
#endif
}

void CTaskTimeMeasure::BeginControlTime()
{
#if defined (__XENO__) && defined (EXUTION_TIME_MEASURE)
    m_lTickTime[TIME_CTRL] = rt_timer_read();
#endif
}

void CTaskTimeMeasure::EndCtrlDataime()
{
#if defined (__XENO__) && defined (EXUTION_TIME_MEASURE)
    SRTIME lExeTime = (rt_timer_read() - m_lTickTime[TIME_CTRL]);

    if (lExeTime >  m_lExecTime[TIME_CTRL]) {
        CTaskUtil::DebugStrinFrwTask("##Max Processing Control Time: %lld\n", lExeTime);
        m_lExecTime[TIME_CTRL] = lExeTime;
        if (lExeTime > 900000) m_lExecTime[TIME_CTRL] = 800000;
    }
#endif
}

void CTaskTimeMeasure::BeginWriteTime()
{
#if defined (__XENO__) && defined (EXUTION_TIME_MEASURE)
    m_lTickTime[TIME_OUPT] = rt_timer_read();
#endif
}

void CTaskTimeMeasure::EndWriteTime()
{
#if defined (__XENO__) && defined (EXUTION_TIME_MEASURE)
    SRTIME lExeTime = (rt_timer_read() - m_lTickTime[TIME_OUPT]);

    if (lExeTime >  m_lExecTime[TIME_OUPT]) {
        CTaskUtil::DebugStrinFrwTask("##Max Processing Write Time: %lld\n", lExeTime);
        m_lExecTime[TIME_OUPT] = lExeTime;
        if (lExeTime > 100000) m_lExecTime[TIME_OUPT] = 50000;
    }
#endif
}
