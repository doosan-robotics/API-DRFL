/*
 * SysConfigUtil.cpp
 *
 * This class is set of system configuration utility for DRCF.
 *
 *  Created on: 2016. 4. 4.
 *      Author: myungjin.kim
 */

#if defined (OS_LINUX)
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <dirent.h>

#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <ifaddrs.h>

#include <spawn.h>
#include <wait.h>
#include <dirent.h>

#include <sys/socket.h> /* for socket() and bind() */
#include <arpa/inet.h>  /* for sockaddr_in */

#if defined (_FUNC_NETWORK_RUNNING_CHECK) && defined(__XENO__)
#include <ifaddrs.h>
#endif

extern char **environ;

#endif

#ifdef WIN32
#include <Windows.h>
#include <Pdh.h>
#pragma comment(lib,"Pdh.lib")
#include <iphlpapi.h>
#pragma comment(lib, "IPHLPAPI.lib")
#endif

#include "SysConfigUtil.h"
#include "File/File.h"
#if defined (__XENO__)
#include "../TaskIntegration/TI4UserOperation/IESlave/IESlave.h"
#endif

#define PATH_DHCLIENT "../../Common/Logs/dhclient.txt"
#define PATH_ROUTE    "../../Common/Logs/route.txt"

using namespace DRAFramework;
#if defined (OS_LINUX)
#if defined (__XENO__)
// Static variable
RTIME CSysConfigUtil::preTime;
#endif
#else
__int64 CSysConfigUtil::preTime;
__int64 CSysConfigUtil::freq;
#endif

#define IP_PRINT(a) \
      *((unsigned char *)(&a)), \
      *(((unsigned char *)(&a)) + 1), \
      *(((unsigned char *)(&a)) + 2), \
      *(((unsigned char *)(&a)) + 3)

//////////////////////////////////////////////////////////////////////////////////////
////////// CSysConfigUtil Class
//////////////////////////////////////////////////////////////////////////////////////
#ifdef DB_ACCESS_AUTH
void CSysConfigUtil::DBAcessAuthorization(void) {
    CTaskUtil::DebugStrinFrwSystem("DB Access Authorization");
    system("sh db_update.sh");
}
#endif
void CSysConfigUtil::UsbReset(int nResetType/* = USB_RESET*/) {

    //pid_t pid;
    //char *argv[] = {(char *)"../../UsbReset.sh", (char *)0};
    //posix_spawn(&pid, "../../UsbReset.sh", NULL, NULL, argv, environ);

    switch (nResetType)
    {
    case USB_DISABLE:
        {
            CTaskUtil::DebugStrinFrwSystem("USB Disable");

            system("echo -n ""0000:00:14.0"" > /sys/bus/pci/drivers/xhci_hcd/unbind");
        }
        break;
    case USB_ENABLE:
        {
            CTaskUtil::DebugStrinFrwSystem("USB Enable");

            system("echo -n ""0000:00:14.0"" > /sys/bus/pci/drivers/xhci_hcd/bind");
        }
        break;
    case USB_RESET:
    default:
        {
            CTaskUtil::DebugStrinFrwSystem("USB reset");

            system("echo -n ""0000:00:14.0"" > /sys/bus/pci/drivers/xhci_hcd/unbind");
            system("echo -n ""0000:00:14.0"" > /sys/bus/pci/drivers/xhci_hcd/bind");
        }
    break;
    }
}

/*
 * SyncFile
 *
 *     Description:  forces everything in temporary memory storage to be written to a persistent file storage (like a disk)
 *     Parameters:
 *         [IN] N/A
 *         [OUT] N/A
 *     History: TBD
 */

void
CSysConfigUtil::SyncFileToDisk()
{
    RunSystem((char*)"sync");
}

/*
 * ModbusAuthorization
 *
 *     Description:  change the access permissions to ModBusSubPrj
 *     Parameters:
 *         [IN] N/A
 *         [OUT] N/A
 *     History: TBD
 */
void
CSysConfigUtil::ModbusAuthorization() {
    system("chmod 755 ModBusSubPrj");
}

/*
 * IEAuthorization
 *
 *     Description:  change the access permissions to Industrial Ethernet process
 *     Parameters:
 *         [IN] N/A
 *         [OUT] N/A
 *     History: TBD
 */
void
CSysConfigUtil::IEAuthorization() {
#if defined (__XENO__)
    CFile ConverterEIP;
#if !defined (_FUNC_EIP_32BTYES)
    ConverterEIP.ConvertingWindowsToLinux(ETHERNETIP_SUB_PROGRAM_PATH);
#endif
    ConverterEIP.ConvertingWindowsToLinux(PROFINET_SUB_PROGRAM_PATH);
#endif
    system("chmod 755 interface/*");
    system("chmod 755 util/*");
}

/*
 * Log Folder Authorization
 *
 *     Description:  change the access permissions & owner to Log Folder
 *     Parameters:
 *         [IN] N/A
 *         [OUT] N/A
 *     History: TBD
 */
void
CSysConfigUtil::LogFolderAuthorization() {
    system("chown -R dra:dra /home/dra/Application/Common");
    system("chmod -R 755 /home/dra/Application/Common");
}


/*
 * TimeSynchronized
 *
 *     Description: NTP Time synchronized Enable/Disable
 *     Parameters:
 *         [IN] flag
 *         [OUT] N/A
 *     History: TBD
 */
void
CSysConfigUtil::TimeSynchronized(bool flag)
{
#if defined (OS_LINUX)
    if(flag == false) {
        // Control whether NTP is Disabled
        system("timedatectl set-ntp 0");
    }
    else {
        // Control whether NTP is Enabled
        system("timedatectl set-ntp 1");
    }
#endif
}
/*
 * SetSystemFileSystmeCheckFix
 *
 *     Description: enable file system check & fix
 *     Parameters:
 *         [IN] None
 *         [OUT] skip = 0, set = 1,  error= -1
 *     History:
 *         xubuntu version check : if exist rcS 14 otherwise 17
 *         xubuntu 14 : /etc/default/rcS  : insert FSCKFIX=yes
 *         xubuntu 17 : /etc/default/grub : change line (GRUB_CMDLINE_LINUX_DEFAULT="quiet splash vga=0362" -> GRUB_CMDLINE_LINUX_DEFAULT="quiet splash vga=0362 forcefsck fsckfix")
 */
int
CSysConfigUtil::SetSystemFileSystmeCheckFix(void)
{
#if defined (OS_LINUX)
    //CTaskUtil::DebugString("CSysConfigUtil::SetSystemFileSystmeCheckFix(void)\n");
    FILE *fp;
    FILE *original;
    FILE *modify;
    int ret = 0;
    char command[255] = { 0 };
    pid_t pid;
    char *argv[] = {(char *)"sh", (char *)"-c", command, 0};
    int nSkipCnt = 0;

    // Check xubuntu 14 or xubuntu 17
    fp = fopen("/etc/default/rcS", "r");
    if (fp != NULL) //xubuntu 14
    {
        while (!feof(fp))
        {
            fgets(command, 255, fp);
            if(strncmp(command, "FSCKFIX=yes", sizeof("FSCKFIX=yes")-1) ==0)
            {
                CTaskUtil::DebugStrinFrwSystem("Options(FSCKFIX=yes) already set !!!\n");
                fclose(fp);
                return 0;   //skip
            }
        }
        fclose(fp);

        // Reopen and add option
        fp = fopen("/etc/default/rcS", "a+");
        if (fp != NULL)
        {
            CTaskUtil::DebugStrinFrwSystem("add optoin : FSCKFIX=yes\n");
            fprintf(fp, "FSCKFIX=yes\n");
            fclose(fp);
            return 1;   //set
        }
    }
    else //xubuntu 17
    {
        fp = fopen("/etc/default/grub", "r");
        if (fp != NULL)
        {
            while (!feof(fp))
            {
                fgets(command, 255, fp);
#if defined(_FUNC_EXTERNAL_REALTIME_CONTROL)
                if (strstr(command, "forcefsck fsckfix") != NULL)
#else
                if(strncmp(command, "GRUB_CMDLINE_LINUX_DEFAULT=\"quiet splash vga=0362 forcefsck fsckfix\"",sizeof("GRUB_CMDLINE_LINUX_DEFAULT=\"quiet splash vga=0362 forcefsck fsckfix\"")-1) ==0)
#endif
                {
                    CTaskUtil::DebugStrinFrwSystem("Options(forcefsck fsckfix) already set !!!\n");
                    fclose(fp);
                    return 0;   //skip
                }
            }
            fclose(fp);

            // Change grub file after update-grub
            // GRUB_CMDLINE_LINUX_DEFAULT="quiet splash vga=0362" -> GRUB_CMDLINE_LINUX_DEFAULT="quiet splash vga=0362 forcefsck fsckfix"
            ret = rename("/etc/default/grub", "/etc/default/grub_bak");
            if (ret < 0)
            {
                CTaskUtil::DebugStrinFrwSystem("Rename failed : grub -> grub_bak\n");
            }
            else
            {
                // Create new grup file
                original = fopen("/etc/default/grub_bak", "r");
                modify = fopen("/etc/default/grub", "w");

                // Copy previous contents of original grub file to new one except GRUB_CMDLINE_LINUX_DEFAULT=
                nSkipCnt = 0;
                while (!feof(original))
                {
                    fgets(command, 255, original);

                    //CTaskUtil::DebugString("%s\n",command);
#if defined(_FUNC_EXTERNAL_REALTIME_CONTROL)
                    if(strstr(command,"GRUB_CMDLINE_LINUX_DEFAULT=") != NULL)
                    {
                        std::string strCmd(command);
                        std::string strCheck = " forcefsck fsckfix";
                        strCmd.insert(strCmd.begin() + strCmd.find_last_of("\""), strCheck.begin(), strCheck.end());

                        CTaskUtil::DebugStrinFrwSystem("%s\n",strCmd.c_str());
                        fputs(strCmd.c_str(), modify);
                    }
#else
                    if(strncmp(command,"GRUB_DISTRIBUTOR=",sizeof("GRUB_DISTRIBUTOR=")-1)==0)
                    {
                        fputs(command, modify);

                        strcpy(command, "GRUB_CMDLINE_LINUX_DEFAULT=\"quiet splash vga=0362 forcefsck fsckfix\"\n");
                        CTaskUtil::DebugStrinFrwSystem("%s\n",command);
                        fputs(command, modify);
                        nSkipCnt = 1;
                    }
#endif
                    else
                    {
                        if(nSkipCnt > 0)
                            nSkipCnt--;
                        else
                            if(command[0]) fputs(command, modify);
                    }

                    memset(command, 0, sizeof(char) * 255);
                }

                // Close files
                fclose(modify);
                fclose(original);

                // Execute update command
                memset(command, 0, sizeof(char) * 255);
                sprintf(command, "update-grub");
                ret = posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);
                CTaskUtil::DebugStrinFrwSystem("Execute update command: update-grub\n",ret);
                return 1;   //set
            }
        }
    }

    CTaskUtil::DebugStrinFrwSystem("CSysConfigUtil::SetSystemFileSystmeCheckFix(void) failed\n");
    return -1;
#endif
    return 0;
}

void
CSysConfigUtil::GetSerialPortName(LPSERIAL_SEARCH pSerialSearch)
{
#if defined (OS_LINUX)
    FILE *fp;
    pid_t pid;
    char command[255] = "ls /dev/ttyUSB* /dev/ttyACM* > serial.dat";
    char commandSymbolLink[255] = {0, };
    char *argv[] = {(char *)"sh", (char *)"-c", command, 0};
    char port[16] = {0, }, name[1024] = {0, };
    char *p, *nextptr;
    int cnt = 0, newAcmPort = 0;
    int nAcmOffset = 0;
    posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);
#if defined (__XENO__)
    rt_task_sleep(2e8);
#else
    CTaskUtil::SleepX(2e8);
#endif

    fp = fopen("serial.dat", "r");
    if (fp != NULL) {
        while(fgets(port, 16, fp) != NULL) {

            int nLength = strlen(port);
            if(strncmp(port, "/dev/ttyACM",11) == 0)///When dev/ttyACM is detected
            {
                port[strlen(port) - 1] = '\0';//remove nextline
                sscanf(port, "/dev/ttyACM%d", &newAcmPort);
                nAcmOffset = 80 + newAcmPort;
                sprintf(commandSymbolLink, "ln -fs %s /dev/ttyS%d", port, nAcmOffset);
                RunSystem(commandSymbolLink);
                memset(port, 0, sizeof(port));
                nLength = sprintf(port, "/dev/ttyS%d", nAcmOffset);
                strncpy(pSerialSearch->_tSerial[cnt]._szPort, port, nLength);
            }
            else
            {
                if(nLength > 2)
                    strncpy(pSerialSearch->_tSerial[cnt]._szPort, port, nLength-1);
            }
            //CTaskUtil::DebugString("### serial port search[%s]", pSerialSearch->_tSerial[cnt]._szPort);
            cnt++;
            pSerialSearch->_nCount = cnt;
        }
        fclose(fp);
    }
    memset(command, 0, sizeof(char) * 255);
    sprintf(command, "rm serial.dat");
    posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);

    cnt = 0;
    fp = fopen("/proc/tty/driver/usbserial", "r");
    if (fp != NULL) {

        fgets(name, 255, fp);                           //first sentence skip
        while(fgets(name, 255, fp)) {
            p = strtok_r(name, ":\" ", &nextptr);
            while(p != NULL) {
                if(strncmp(p, "name", 4) == 0) {
                    p = strtok_r(NULL, ":\"", &nextptr);
                    if(p != NULL)
                        strncpy(pSerialSearch->_tSerial[cnt]._szName, p, 128);
                    //CTaskUtil::DebugString("#### name[%s]", pSerialSearch->_tSerial[cnt]._szName);
                    break;
                }
                else
                    p = strtok_r(NULL, ":\" ", &nextptr);
            }
            cnt++;
        }
        fclose(fp);
    }
    //for(int i = 0; i < pSerialSearch->_nCount; i++)
        //CTaskUtil::DebugString("#### [%d] Serial Port[%s] Name[%s]", i, pSerialSearch->_tSerial[i]._szPort, pSerialSearch->_tSerial[i]._szName);
#endif
}

void CSysConfigUtil::RunSystem(char* szCommand)
{
#if defined (OS_LINUX)
    pid_t pid = 0;
    char *argv[] = {(char *)"sh", (char *)"-c", szCommand, NULL};
    posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);

    int status = 0;
    waitpid(pid, &status, 0);
#else
    system(szCommand);
#endif
}

void CSysConfigUtil::ConnectingUsbtethering()
{
#if defined (OS_LINUX)
    const char *szlo = "lo";           //network interfaceÁß local loopbackÀÌ¸§
    const char *szenp2s0 = "enp2s0";  //network interfaceÁß ¿ÜºÎ¿ÍÀÇ À¯¼±Á¢¼Ó½Ã »ç¿ëµÇ´Â ÀåÄ¡¸í(netplan¿¡ ÀÇÇÏ¿© ÇØ´ç ÀÌ¸§À¸·Î °íÁ¤µÊ)
    bool bIsDetectNewInterfaceDevice = false;

    char szCommand[255] = {'\0', };
    struct ifaddrs *iflist = NULL;

    if (getifaddrs(&iflist) != (-1)){

        for (struct ifaddrs* ifa = iflist; ifa != NULL; ifa = ifa->ifa_next) {

            if( ifa->ifa_addr->sa_family == 17/*AF_NETBIOS*/){

                if ((strcmp(ifa->ifa_name, szlo) == 0) || (strcmp(ifa->ifa_name, szenp2s0) == 0)) {
                    //ÀåÄ¡Áß ±âº»ÀåÄ¡ÀÏ°æ¿ì
                    continue;
                }
                else {
                    //±âº»ÀåÄ¡ ÀÌ¿ÜÀÇ Å×´õ¸µ ÀåÄ¡
                    bIsDetectNewInterfaceDevice = true;

                    sprintf(szCommand, "ip addr flush %s;dhclient -v %s > %s 2>&1", ifa->ifa_name, ifa->ifa_name, PATH_DHCLIENT);
                    CTaskUtil::DebugStrinFrwSystem("Register USB Tethering Device(%s)", ifa->ifa_name);
                    RunSystem(szCommand);

                    FILE *fp = fopen(PATH_DHCLIENT, "r");
                    if (fp != NULL){
                        char szLine[255] = {'\0', };
                        while(fgets(szLine, 255, fp)){

                            char *pszToken = strstr(szLine, "DHCPACK");
                            if (pszToken != NULL){
                                char szGateway[255] = {'\0', };
                                sscanf(szLine, "DHCPACK of %*s from %s", szGateway);
                                // À¯¹«¼± ÇÔ²² ¿¬°áµÇ¾î ÀÖÀ» ¶§ µÑÁß ÇÏ³ª°¡ ²÷¾îÁö¸é ÀÌÀü route °æ·Î¸¦ »õ·Î °»½Å
                                sprintf(szCommand, "route add default gw %s %s;route > %s 2>&1", szGateway, ifa->ifa_name, PATH_ROUTE);
                                CTaskUtil::DebugStrinFrwSystem("Add USB Tethering Gateway(%s) Device(%s)", szGateway, ifa->ifa_name);
                                RunSystem(szCommand);
                                break;
                            }
                        }
                        fclose(fp);
                        //remove(PATH_DHCLIENT);// º°µµ »èÁ¦ÇÏÁö ¾ÊÀ½.
                    }
                    break;

                }
            }
        }
        freeifaddrs(iflist);
    }
    if(bIsDetectNewInterfaceDevice == false)
        CTaskUtil::DebugStrinFrwSystem("Not Detected USB Tethering Device");
#endif
}

#ifdef FUNC_PY_PACKAGE_UPDATE
/*
 * InstallPyPackage
 *
 *     Description: Install python packages
 *     Parameters:
 *         [IN] pyPackageList : Python package list to install
 *     History: TBD
 */
void CSysConfigUtil::InstallPyPackage(vector<string> &libList, vector<string> &pyPackageList)
{
#if defined (OS_LINUX)
    struct stat temp;
    ostringstream buffer("");
    char command[255] = { 0 };
    char *argv[] = {(char *)"sh", (char *)"-c", command, 0};
    pid_t pid;
    bool bUpdateLib = false;

    // Library list
    for (vector<string>::iterator it = libList.begin(); it != libList.end(); it++) {
        // Check library
        buffer.str("");
        buffer << "/usr/local/lib/" << *it;
        if (stat(buffer.str().c_str(), &temp) != 0) {
            // Extract compression file
            sprintf(command, "tar zxf ./pys_linux/packages/%s.tar.gz -C /usr/local/lib/", it->c_str());
            CTaskUtil::DebugString("[InstallPyPackage] %s", command);
            system(command); // Don't use posix_spawn because next command (ldconfig) must be executed after complete this command

            // Generate dummy file
            sprintf(command, "touch /usr/local/lib/%s",  it->c_str());
            CTaskUtil::DebugString("[InstallPyPackage] %s", command);
            posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);
            bUpdateLib = true;
        }
    }

    // Update library search cache data
    if (bUpdateLib == true) {
        sprintf(command, "ldconfig");
        CTaskUtil::DebugString("[InstallPyPackage] %s", command);
        posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);
    }

    // Python Packages
    for (vector<string>::iterator it = pyPackageList.begin(); it != pyPackageList.end(); it++) {
        // Check python package
        buffer.str("");
        buffer << "/usr/local/lib/python3.2/dist-packages/" << *it << ".egg-info";
        if (stat(buffer.str().c_str(), &temp) != 0) {
            sprintf(command, "tar zxf ./pys_linux/packages/%s.tar.gz -C /usr/local/lib/python3.2/dist-packages/",  it->c_str());
            CTaskUtil::DebugString("[InstallPyPackage] %s", command);
            posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);
        }
    }
#endif
}
#endif

/*
 * SetSystemIpAddress
 *
 *     Description: Set IP address to system
 *     Parameters:
 *         [IN] index : index of interface (eth0, eth1, ...)
 *         [IN] param : information set about IP address (IP address, gateway, subnet, DNS)
 *         [OUT] it returns zero
 *     History: TBD
 */

int CSysConfigUtil::SetSystemIpAddress(SYSTEM_IPADDRESS param, char* lpszIeIP)
{
    int  ret = 0;

#if defined (OS_LINUX)
#ifdef _FUNC_IE_IP_ADDRESS_APPLY
    enum { DYNAMIC_IP, STATIC_IP };
    enum { IE_IP, USER_IP };
    pid_t pid;
    char command[255] = { 0 };
    char *argv[] = {(char *)"sh", (char *)"-c", command, 0};
    const char *pDefaultGatewayAddr = "0.0.0.0";
    //1. ¼³Á¤Àü¿¡ ³×Æ®¿öÅ©°¡ ´Ù¿îÀÌ ç´ÂÁö¸ÕÀú °Ë»ç
    struct ifaddrs *ifaddr, *ifa;//Network DownTest
    if (getifaddrs(&ifaddr) != (-1))
    {
        for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == NULL)    continue;

            if ((strcmp(ifa->ifa_name,"enp2s0") == 0) && ((ifa->ifa_flags & 0x10000) == 0))
            {

                ret = 1;//Notify error state 1
                if(lpszIeIP == NULL) //Immediately after IP modification in IE SW, NETWORK is down and exception handling
                {
                    freeifaddrs(ifaddr);
                    return ret;
                }
            }
        }
        freeifaddrs(ifaddr);
    }

    FILE *fp;
    char *p = NULL, *nextptr = NULL;
    char szNetplanIndustrialEthernetIP[16] = {0, };
    char szNetplanUserIP[16] = {0, };
    if(lpszIeIP == NULL)
    {
        fp = fopen("/etc/netplan/01-network-manager-all.yaml", "r");
        if (fp != NULL)
        {
            // Get current configuration
            while (fgets(command, 100, fp)) {
                p = strtok_r(command, ":[/, ", &nextptr);
                if(p!= NULL)
                {
                    if (strncmp(p, "addresses", 9) == 0)
                    {
                        p = strtok_r(NULL, ":[/, ", &nextptr);      // Get Netplan IndustrialEthernet IP address
                        strncpy(szNetplanIndustrialEthernetIP, p, 16);
                        strtok_r(NULL, ":[/, ", &nextptr);
                        p = strtok_r(NULL, ":[/, ", &nextptr);      // Get Netplan  User IP address
                        strncpy(szNetplanUserIP, p, 16);
                        break;
                    }
                }
                else
                    CTaskUtil::DebugStrinFrwSystem("Wrong details are written in the netplan file.");
            }
            // Close file
            fclose(fp);
        }
        else
            CTaskUtil::DebugStrinFrwSystem("Not exist netplan file");
    }

    struct in_addr subnet = {'\0', };
    int subnetBits = 0;
    // Calculate subnet mask
    if ((param._szSubnet[0] != 0) && (param._szSubnet[0] != '0')) {
        inet_aton(param._szSubnet, &subnet);
    }
    else {
        inet_aton("255.255.255.0", &subnet);
    }

    for (int i = 0; i < 32; i++) {
        if (subnet.s_addr & (1 << i)) {
            subnetBits++;
        }
    }

    //Network Disable Function
    if (param._iUsage == 0)
    {
    #if DISABLE_NETWORK
        memset(command, 0, sizeof(char) * 255);
        sprintf(command, "rm /etc/netplan/01-network-manager-all.yaml");
        posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);

        memset(command, 0, sizeof(char) * 255);
        sprintf(command, "ip link set enp2s0 down");
        posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);
    #else
        ret = 0;
    #endif
        return (ret);
    }

    // Reopen for update
    fp = fopen("/etc/netplan/01-network-manager-all.yaml", "w");

    if(fp != NULL)
    {
        // Create configuration file
        fprintf(fp, "network:\n");
        fprintf(fp, "  version: 2\n");
        fprintf(fp, "  renderer: networkd\n");
        fprintf(fp, "  ethernets:\n");
        fprintf(fp, "    enp2s0:\n");

        if (param._iIpType == DYNAMIC_IP)
        {
              // Dynamic IP
              fprintf(fp, "      dhcp4: yes\n");
              if (lpszIeIP != NULL)
                  fprintf(fp, "      addresses: [%s/24,0.0.0.0/24]\n", lpszIeIP);
              else
                  fprintf(fp, "      addresses: [%s/24,0.0.0.0/24]\n", szNetplanIndustrialEthernetIP);

              fclose(fp);
              memset(command, 0, sizeof(char) * 255);
              sprintf(command, "netplan apply");
              posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);
              return (ret);
          }
          else
          {
              // Static IP
              fprintf(fp, "      dhcp4: no\n");

              if (lpszIeIP != NULL)
                  fprintf(fp, "      addresses: [%s/24,%s/%d]\n", lpszIeIP, param._szHotsIp, subnetBits);
              else
                  fprintf(fp, "      addresses: [%s/24,%s/%d]\n", szNetplanIndustrialEthernetIP, param._szHotsIp, subnetBits);

              // Default gateway IP setting
              if ((param._szGateway[0] != 0) && (param._szGateway[0] != '0')) {
                  fprintf(fp, "      gateway4: %s\n", param._szGateway);
              }
              else {
                  fprintf(fp, "      gateway4: %s\n", pDefaultGatewayAddr);
              }
          }
          // DNS setting
          fprintf(fp, "      nameservers:\n");
          fprintf(fp, "        addresses: [");
          if ((param._szDNS[0][0] != 0) && (param._szDNS[0][0] != '0')) {
              fprintf(fp, "%s", param._szDNS[0]);
              if ((param._szDNS[1][0] != 0) && (param._szDNS[1][0] != '0'))
                  fprintf(fp, ",%s", param._szDNS[1]);
          }
          else {
              // default DNS
              fprintf(fp, "%s,%s", pDefaultGatewayAddr, pDefaultGatewayAddr);
          }
          fprintf(fp, "]\n");

          // Close file
          fclose(fp);

          // Execute update command
          #if DISABLE_NETWORK
          memset(command, 0, sizeof(char) * 255);
          sprintf(command, "ip link set enp2s0 up");
          posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);
          #endif
          memset(command, 0, sizeof(char) * 255);
          sprintf(command, "netplan apply");
          posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);
    }
    else
    {
        CTaskUtil::DebugStrinFrwSystem("netplan file Open fail::SetIpAddress");
    }
#endif
#endif
    return (ret);
}


/*
 * GetSystemIpAddress
 *
 *     Description: Get IP address from system
 *     Parameters:
 *         [IN] index : index of interface (eth0, eth1, ...)
 *         [IN] param : pointer to save information set about IP address (IP address, gateway, subnet, DNS)
 *         [OUT] If this operation is success, then it returns zero. Otherwise, it returns (-1)
 *     History: TBD
 */

int CSysConfigUtil::GetSystemIpAddress(SYSTEM_IPADDRESS *pParam, int nIpType , char* lpszIeIP)
{
#if defined (OS_LINUX)
#ifdef _FUNC_IE_IP_ADDRESS_APPLY
    enum { DYNAMIC_IP, STATIC_IP };
    enum { IE_IP, USER_IP };

    FILE *fp, *fp2;
    char line[100], line2[100];
    char host[255];
    char *p, *p2, *saveptr;
    struct ifaddrs *ifaddr, *ifa;
    int subnetBits = 0;
    struct in_addr subnet;
    char buf[256], iface[256];
    unsigned int destination, gateway;
    int ret, dnscnt = 0;

    //iUsage is always active
    pParam->_iUsage = 1;

    //1. check network interface state
    if (getifaddrs(&ifaddr) != (-1))
    {
        for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == NULL)    continue;

            if ((strcmp(ifa->ifa_name,"enp2s0") == 0) && ((ifa->ifa_flags & 0x10000) == 0))
            {
                pParam->_iUsage = 0;//Reply to 1 if normal, but 0 if abnormal (network disabled)
                break;

            }
        }
        freeifaddrs(ifaddr);
    }

    //2. Acquire all the information included in NetPlan
    fp = fopen("/etc/netplan/01-network-manager-all.yaml", "r");

    if (fp != NULL)
    {
        bool firstData = false;
        while (fgets(line, 100, fp))
        {
            p = strtok_r(line, ":[/, ", &saveptr);
            if (p != NULL)
            {
                // Get type
                if (strncmp(p, "dhcp4", 5) == 0)
                {
                    p = strtok_r(NULL, ":[/, ", &saveptr);
                    if (strncmp(p, "no", 2) == 0) {
                        pParam->_iIpType = STATIC_IP;
                    }
                    else {
                        pParam->_iIpType = DYNAMIC_IP;
                    }

                }

                // Get Subnet mask
                if (strncmp(p, "addresses", 9) == 0)
                {
                    if (firstData == false)
                    {
                        firstData = true;

                        p=strtok_r(NULL, ":[/,] ", &saveptr);           // Industrial Ethernet IP
                        if(lpszIeIP != NULL && p != NULL)
                            strncpy(lpszIeIP, p, 16);

                        p=strtok_r(NULL, ":[/,] ", &saveptr);             // Industrial Ethernet IP subnet mask (bits)

                        p=strtok_r(NULL, ":[/,] ", &saveptr);           // User IP address
                        if(p != NULL)
                            strncpy(pParam->_szHotsIp, p, 16);

                        p = strtok_r(NULL, ":[/,] ", &saveptr);         // User IP subnet mask (bits)

                        if(p != NULL)
                        {
                            // Convert subnet mask address from bits to string
                            subnetBits = atoi(p);
                            subnet.s_addr = 0;
                            while (subnetBits-- > 0) {
                                subnet.s_addr = subnet.s_addr | (1 << subnetBits);
                            }
                            strncpy(pParam->_szSubnet, inet_ntoa(subnet), 16);
                        }
                    }
                    else
                    {
                        // Get first DNS
                        p = strtok_r(NULL, ":[/,] ", &saveptr);
                        if (p != NULL) {
                            strncpy(pParam->_szDNS[0], p, 16);
                        }


                        // Get second DNS
                        p = strtok_r(NULL, ":[/,] ", &saveptr);
                        if (p != NULL) {
                            strncpy(pParam->_szDNS[1], p, 16);
                        }
                    }
                }

                // Get gateway IP address
                if (strncmp(p, "gateway4", 8) == 0) {
                    p = strtok_r(NULL, ":[/, ", &saveptr);
                    if (p != NULL)
                        strncpy(pParam->_szGateway, p, 16);
                    else
                        CTaskUtil::DebugStrinFrwSystem("Wrong details are written in the netplan file.(gateway4)");
                }
            }
        }
        fclose(fp);
    }
    else
    {
        CTaskUtil::DebugStrinFrwSystem("Not exist netplan file");
    }

    if(pParam->_iIpType == DYNAMIC_IP)
    {
        // Get real gateway
        fp2 = fopen("/proc/net/route", "r");

        if( fp2 != NULL)
        {
            while(fgets(buf, sizeof(buf), fp2))
            {
                if(!strncmp(buf, "Iface", 5))
                    continue;
                ret = sscanf(buf, "%s\t%x\t%x", iface, &destination, &gateway);
                if (ret < 3)
                {
                    fprintf(stderr, "ERROR: line read error\n");
                    continue;
                }
                if (destination != 0)
                    continue;
                sprintf(pParam->_szGateway, "%u.%u.%u.%u", IP_PRINT(gateway));
            }
            fclose(fp2);
        }
        else
        {
            CTaskUtil::DebugStrinFrwSystem("Not exist route file");
        }
        //Get real DNS
        //fp2 = fopen("/etc/resolv.conf", "r");
        fp2 = fopen("/run/systemd/resolve/resolv.conf", "r");

        if( fp2 != NULL)
        {
            while(fgets(line2, 100, fp2))
            {
                p2 = strtok_r(line2, " ", &saveptr);
                if(p2 != NULL){
                    if (strncmp(p2, "nameserver", 10) == 0) {
                        p2 = strtok_r(NULL, " ", &saveptr);
                        sprintf(pParam->_szDNS[dnscnt], "%s", p2);
                        dnscnt++;
                    }
                }
                else
                    CTaskUtil::DebugStrinFrwSystem("Wrong details are written in the netplan file.(nameserver)");
            }
            fclose(fp2);
        }
        else
        {
            CTaskUtil::DebugStrinFrwSystem("Not exist resolv.conf file");
        }
    }

    vector<string> vIpAddress;
    struct ifaddrs *ifaddr2, *ifa2;
    if(pParam->_iIpType == DYNAMIC_IP || nIpType == REAL_IP)
    {
        if (getifaddrs(&ifaddr2) != (-1))
        {
            for (ifa2 = ifaddr2; ifa2 != NULL; ifa2 = ifa2->ifa_next)
            {
                if (ifa2->ifa_addr == NULL)
                    continue;

                if ((strcmp(ifa2->ifa_name,"enp2s0") == 0) && (ifa2->ifa_addr->sa_family == AF_INET) && ((ifa2->ifa_flags & 0x10000) != 0))
                {
                    if (getnameinfo(ifa2->ifa_addr, sizeof(struct sockaddr_in), host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST) == 0)
                    {
                        if (strstr(host, "10.0.0") == NULL)
                        {
                            vIpAddress.push_back(host);
                        }
                    }
                }
            }
            freeifaddrs(ifaddr2);

            if (vIpAddress.size() == 0) {
                pParam->_iUsage = 0;
                memset(pParam->_szHotsIp,0,16);
                memset(pParam->_szDNS[0],0,16);
                memset(pParam->_szDNS[1],0,16);
                memset(pParam->_szGateway,0,16);
                memset(pParam->_szSubnet,0,16);
            }
            else
            {
                strncpy(pParam->_szHotsIp, vIpAddress[vIpAddress.size() -1].c_str(), 16);
            }
        }
    }
#endif
#endif
    return (0);
}

#if defined (_FUNC_NETWORK_RUNNING_CHECK) && defined(__XENO__)
/**
 * @brief      GetNetworkDeviceStatus
 * @details    Checking Network Device Status
 * @param      [IN] char pNetworkDevice[],
 * @return     [Status] NetworkDevice Status
 */
int CSysConfigUtil::GetNetworkDeviceStatus(char* pNetworkDevice)
{
    struct ifaddrs *ifaddr, *ifa;
    int nStatus = 0;
    if (getifaddrs(&ifaddr) != (-1)){
        for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
        {
            if (ifa->ifa_addr == NULL)    continue;

            if (strcmp(ifa->ifa_name,pNetworkDevice) == 0)
            {
                nStatus = ifa->ifa_flags;
                freeifaddrs(ifaddr);
                break;
            }
        }
    }
    return nStatus;
}

void CSysConfigUtil::CheckPreparationCompleteOfNetworkDevice(char* pNetworkDevice)
{
    unsigned int nPrevState = 0, nCurrentState = 0;
    static bool bNetworkCheck = false;
    int nMacCheckCnt = 100;
    if(bNetworkCheck == false)
    {
        nPrevState = CSysConfigUtil::GetNetworkDeviceStatus(pNetworkDevice);
        CTaskUtil::DebugStrinFrwSystem("Init NetworkState[%s][%d]", pNetworkDevice, nPrevState);
        while(1)
        {
            nCurrentState = CSysConfigUtil::GetNetworkDeviceStatus(pNetworkDevice);

            if((IFF_UP & nCurrentState) && (IFF_RUNNING  & nCurrentState))
            {
                CTaskUtil::DebugStrinFrwSystem("Cur NetworkState[%s][%d]", pNetworkDevice, nCurrentState);
                break;
            }
            CTaskUtil::SleepX(100);//Delay to reduce processing load
            if(--nMacCheckCnt == 0)    break;
        }
        bNetworkCheck = true;
    }
    CTaskUtil::SleepX(50);    //Delay for initial network operation
    CTaskUtil::DebugStrinFrwSystem("Final NetworkState[%s][%d]", pNetworkDevice, nCurrentState);
}
#endif

/*
 * SetSystemTime
 *
 *     Description: Set system time
 *     Parameters:
 *         [IN] timeData : time information to set
 *         [OUT] It returns zero
 *     History: TBD
 */

int
CSysConfigUtil::SetSystemTime(SYSTEM_TIME &timeData)
{
#if defined (OS_LINUX)
    struct tm newTime = {'\0', };
    struct timespec tp = {'\0', };
    int in_date = 0;
    int in_time = 0;
    pid_t pid ;
    char command[255] = { 0 };
    char *argv[] = {(char *)"sh", (char *)"-c", command, 0};

    in_date = atoi(timeData._szDate);
    in_time = atoi(timeData._szTime);

    // Get new time value
    newTime.tm_year = in_date / 10000 + 100;        // since 1900
    newTime.tm_mon = (in_date / 100) % 100 - 1;     // start 0
    newTime.tm_mday = in_date % 100;
    newTime.tm_hour = in_time / 10000;
    newTime.tm_min = (in_time / 100) % 100;
    newTime.tm_sec = in_time % 100;

    // Update system time
    clock_gettime(CLOCK_REALTIME, &tp);
    tp.tv_sec = mktime(&newTime);
    clock_settime(CLOCK_REALTIME, &tp);

    // Get system time
    clock_gettime(CLOCK_REALTIME, &tp);
    //CTaskUtil::DebugString("System Time(%ld): %s\n", tp.tv_nsec, ctime(&tp.tv_sec));

    // Update HW time
    //system("hwclock -w");
    sprintf(command, "hwclock -w");
    posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ);
#else
    int in_date;
    int in_time;

    in_date = atoi(timeData._szDate);
    in_time = atoi(timeData._szTime);

    SYSTEMTIME systemTime =
    {
        in_date / 10000,
        (in_date / 100) % 100,
        0,
        in_date % 100,
        in_time / 10000,
        (in_time / 100) % 100,
        0,
        0
    };

    SetLocalTime(&systemTime);
#endif
    return (0);
}


/*
 * GetSystemTime
 *
 *     Description: Get system time
 *     Parameters:
 *         [IN] pTimeData : pointer to buffer for saving time information
 *         [OUT] It returns zero
 *     History: TBD
 */

int
CSysConfigUtil::GetSystemTime(SYSTEM_TIME *pTimeData)
{
#if defined (OS_LINUX)
    struct tm *pCurrTime;
    struct timespec tp;
    int in_date;
    int in_time;

    // Get system time
    clock_gettime(CLOCK_REALTIME, &tp);

    pCurrTime = localtime(&tp.tv_sec);
    if(pCurrTime != NULL)
    {
        in_date = ((pCurrTime->tm_year % 100) * 10000) + (++pCurrTime->tm_mon * 100) + (pCurrTime->tm_mday);
        in_time = (pCurrTime->tm_hour * 10000) + (pCurrTime->tm_min * 100) + (pCurrTime->tm_sec);
        sprintf(pTimeData->_szDate, "%d", in_date);
        sprintf(pTimeData->_szTime, "%06d", in_time);
    }

#else
    int in_date;
    int in_time;

    SYSTEMTIME systemTime =
    {
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0
    };

    GetLocalTime(&systemTime);

    in_date = ((systemTime.wYear % 100) * 10000) + (systemTime.wMonth * 100) + (systemTime.wDay);
    in_time = (systemTime.wHour * 10000) + (systemTime.wMinute * 100) + (systemTime.wSecond);
    sprintf(pTimeData->_szDate, "%d", in_date);
    sprintf(pTimeData->_szTime, "%06d", in_time);
#endif
    return (0);
}


int CSysConfigUtil::GetSystemTime(char* lpszDateTime)
{
#if defined (OS_LINUX)
    struct timespec tp;
    struct tm *pCurrTime;

    // Get system time
    clock_gettime(CLOCK_REALTIME, &tp);

    pCurrTime = localtime(&tp.tv_sec);
    if( pCurrTime != NULL)
    {
        sprintf(lpszDateTime, "%04d%02d%02d%02d%02d%02d",
                pCurrTime->tm_year + 1900, pCurrTime->tm_mon + 1, pCurrTime->tm_mday,
                pCurrTime->tm_hour, pCurrTime->tm_min, pCurrTime->tm_sec);
    }


#else
    SYSTEMTIME systemTime = {'\0', };
    GetLocalTime(&systemTime);

    sprintf(lpszDateTime, "%04d%02d%02d%02d%02d%02d",
            systemTime.wYear, systemTime.wMonth, systemTime.wDay,
            systemTime.wHour, systemTime.wMinute, systemTime.wSecond);

#endif
    CTaskUtil::DebugStrinFrwSystem("System Time: %s\n", lpszDateTime);

    return (0);
}


int CSysConfigUtil::GetSystemTime(long &sec, long &usec)
{
#if defined (OS_LINUX)
    struct timespec tp;

    // Get system time
    clock_gettime(CLOCK_REALTIME, &tp);

    sec = tp.tv_sec;
    usec = tp.tv_nsec / 1e3;
#else
    static const uint64_t EPOCH = ((uint64_t) 116444736000000000ULL); // time distance between epoch time and 1601-Jan-01
    SYSTEMTIME  system_time;
    FILETIME    file_time;
    uint64_t    time; // 100-nanos since 1601-Jan-01
    ::GetSystemTime(&system_time);
    SystemTimeToFileTime( &system_time, &file_time );
    time =  ((uint64_t)file_time.dwLowDateTime )      ;
    time += ((uint64_t)file_time.dwHighDateTime) << 32;
    sec  = (long) ((time - EPOCH) / 10000000L);
    usec = (long) (((time - EPOCH) % 10000000L)/10.0);
    //sec -= 1483228800; // convert time since 2017.01.01 00:00:00
    //sec -= 1491004800;
#endif
    return (0);
}


int CSysConfigUtil::GetSystemTime(double &t)
{
#if defined (OS_LINUX)
    struct timespec tp;

    // Get system time
    clock_gettime(CLOCK_REALTIME, &tp);

    t = (double)tp.tv_sec + (double)tp.tv_nsec / 1e10;
#else
    static const uint64_t EPOCH = ((uint64_t) 116444736000000000ULL); // time distance between epoch time and 1601-Jan-01
    SYSTEMTIME  system_time;
    FILETIME    file_time;
    uint64_t    time; // 100-nanos since 1601-Jan-01
    ::GetSystemTime(&system_time);
    SystemTimeToFileTime( &system_time, &file_time );
    time =  ((uint64_t)file_time.dwLowDateTime )      ;
    time += ((uint64_t)file_time.dwHighDateTime) << 32;
    long sec  = (long) ((time - EPOCH) / 10000000L);
    long usec = (long) (((time - EPOCH) % 10000000L)/10.0);
    t = double(sec + usec/1e6);
#endif
    return (0);
}


/*
 * ExitSystem
 *
 *     Description: Shutdown system
 *     Parameters:
 *         [IN] N/A
 *         [OUT] N/A
 *     History: TBD
 */

void
CSysConfigUtil::ExitSystem(void)
{
#if defined (OS_LINUX)
    system ("shutdown -h now");        // Shutdown computer immediately
#else
#if 0
    system ("c:\\Windows\\System32\\shutdown /s");
#endif
#endif
}


/*
 * RebootSystem
 *
 *     Description: Reboot system
 *     Parameters:
 *         [IN] N/A
 *         [OUT] N/A
 *     History: TBD
 */

void
CSysConfigUtil::RebootSystem(void)
{
#if defined (OS_LINUX)
    system ("reboot");        // Reboot
#else
#if 0
    system ("c:\\Windows\\System32\\shutdown /r");
#endif
#endif
}


/*
 * GetFreeSizeOfDisk
 *
 *     Description: Return free size of disk (MB)
 *     Parameters:
 *         [IN] N/A
 *         [OUT] If this operation is success, then it returns size of free disk in MB. Otherwise, it returns (-1)
 *     History: TBD
 */

int
CSysConfigUtil::GetFreeSizeOfDisk(LPSYSTEM_DISKSIZE pSysDiskSize)
{
    int ret = 0;

#if defined (OS_LINUX)
    struct statvfs fiData = {'\0', };

    if (statvfs("/", &fiData) < 0) {
        CTaskUtil::DebugStrinFrwSystem("Failed to stat: /\n");
    }
    else {
        CTaskUtil::DebugStrinFrwSystem("\t Block size: %ld\n", fiData.f_bsize);
        CTaskUtil::DebugStrinFrwSystem("\t Total block: %ld\n", fiData.f_blocks);
        CTaskUtil::DebugStrinFrwSystem("\t Free block: %ld\n", fiData.f_bfree);
    }

    pSysDiskSize->_iTotalDiskSize = ((uint64_t)fiData.f_blocks * fiData.f_bsize) / 1000000UL;
    pSysDiskSize->_iUsedDiskSize = ((uint64_t)(fiData.f_blocks - fiData.f_bfree) * fiData.f_bsize) / 1000000UL;
#else
    ULARGE_INTEGER avail, total, free;
    avail.QuadPart = 0L;
    total.QuadPart = 0L;
    free.QuadPart = 0L;

    GetDiskFreeSpaceEx(TEXT("c:\\"), &avail, &total, &free);
    pSysDiskSize->_iTotalDiskSize = (int)(total.QuadPart >> 20);
    pSysDiskSize->_iUsedDiskSize = pSysDiskSize->_iTotalDiskSize - (unsigned int)(free.QuadPart >> 20);
#endif

    return (ret);
}


/*
 * GetUsageOfCPU
 *
 *     Description: Return usage of specific CPU core
 *     Parameters:
 *         [IN] coreNum : core number to get information
 *         [OUT] It returns usage of CPU in %
 *     History: TBD
 *
 *     TODO: This API isn't operated successfully. Because method to get CPU usage is not perfectly yet
 */

int
CSysConfigUtil::GetUsageOfCPU(char* pszPorcessName, LPSYSTEM_CPUUSAGE pSysCpuUsage)
{
    int ret = 0;

#if defined (OS_LINUX)
    FILE *pFp;
    char cpuId[5] = {0};
    static int jiffies[2][4] = {0};
    int totalJiffies;
    int diffjiffies[4];
    int idx;
    float cpuUsage;
    enum jiffy {PAST = 0, PRESENT, USER = 0, USER_NICE, SYSTEM, IDLE};

    pFp = fopen("/proc/stat", "r");
    fscanf(pFp, "%4s %d %d %d %d",
        cpuId, &jiffies[PRESENT][USER], &jiffies[PRESENT][USER_NICE], &jiffies[PRESENT][SYSTEM], &jiffies[PRESENT][IDLE]);
    for (idx = 0, totalJiffies = 0; idx < 4; ++idx) {
        diffjiffies[idx] = jiffies[PRESENT][idx] - jiffies[PAST][idx];
        totalJiffies = totalJiffies + diffjiffies[idx];
    }

    if ((jiffies[PAST][0] == 0) && (jiffies[PAST][1] == 0) && (jiffies[PAST][2] == 0) && (jiffies[PAST][3] == 0)) {
        cpuUsage = 0;
    }
    else {
        cpuUsage = 100.0 * (1.0 - (diffjiffies[IDLE] / (float)totalJiffies));
    }
    memcpy(jiffies[PAST], jiffies[PRESENT], sizeof(int) * 4);
    fclose(pFp);

    // Save data
    pSysCpuUsage->_iTotalUsage = cpuUsage;
    pSysCpuUsage->_iProcessUsage = cpuUsage;

    CTaskUtil::DebugStrinFrwSystem("usage(%s): %f / %f\n", pszPorcessName/*pSysCpuUsage->_processName*/, pSysCpuUsage->_iProcessUsage, pSysCpuUsage->_iTotalUsage);
#else
    // WIN32
    PDH_HQUERY cpuQuery;
    PDH_HCOUNTER cpuTotal;
    PDH_HCOUNTER cpuFree;
    PDH_FMT_COUNTERVALUE cpuTotalVal;
    PDH_FMT_COUNTERVALUE cpuFreeVal;

    // Get usage of specific process
    //if (pSysCpuUsage->_processName[0] == (char)NULL) {
    //    memcpy(pSysCpuUsage->_processName, "DRCFWin32", 9);
    //}

    // Open query
    PdhOpenQuery(NULL, NULL, &cpuQuery);
    PdhAddCounter(cpuQuery, TEXT("\\Processor(_Total)\\% Processor Time"), NULL, &cpuTotal);
    PdhAddCounter(cpuQuery, TEXT("\\Processor(_Total)\\% Idle Time"), NULL, &cpuFree);
    PdhCollectQueryData(cpuQuery);

    Sleep(1000);

    PdhCollectQueryData(cpuQuery);

    PdhGetFormattedCounterValue(cpuTotal, PDH_FMT_DOUBLE, NULL, &cpuTotalVal);
    PdhGetFormattedCounterValue(cpuFree, PDH_FMT_DOUBLE, NULL, &cpuFreeVal);

    pSysCpuUsage->_iTotalUsage  = 100.0f - (float)cpuFreeVal.doubleValue;
    pSysCpuUsage->_iProcessUsage = 0.0f;         // Not supported yet

    CTaskUtil::DebugStrinFrwSystem("CPU Usage: %f / %f\n", cpuTotalVal.doubleValue, cpuFreeVal.doubleValue);

    PdhRemoveCounter(cpuTotal);
    PdhRemoveCounter(cpuFree);
#endif

    return (ret);
}

#if defined (__XENO__)
double CSysConfigUtil::GetRealtimeTick()
{
    return rt_timer_read()/1e9;
}
#endif

/*
 * GetExecuteTime
 *
 *     Description: Calculate execution time
 *                  User must call firstly this function with FALSE parameter
 *                  And than, user must call this function with TRUE parameter
 *     Parameters:
 *         [IN] flag : if flag is false, then this function saves current time to preTime variable
 *                     if flag is true, then this function calculates execution time using preTime and currTime variables
 *         [OUT] N/A
 *     History: TBD
 */

void
CSysConfigUtil::GetExecuteTime(bool flag)
{
#if defined (OS_LINUX)
#if defined (__XENO__)
    RTIME currTime;
    RTIME execTime;

    // Set start time
    if (flag == false) {
        preTime = rt_timer_read();
    }
    // Get execution time
    else {
        currTime = rt_timer_read();
        if ((currTime < preTime) || (preTime == 0))
            CTaskUtil::DebugStrinFrwSystem("Invalid execution time\n");
        else {
            execTime = currTime - preTime;
            CTaskUtil::DebugStrinFrwSystem("ExecTime(%lld): %llds %lldms %lldus %lldns\n", execTime, execTime / 1000000000ULL, (execTime / 1000000ULL) % 1000, (execTime / 1000ULL) % 1000, execTime % 1000);
        }

        preTime = 0;
    }
#endif
#else
    __int64 currTime;
    __int64 execTime;
    if (flag == false) {
        if (QueryPerformanceFrequency((_LARGE_INTEGER *)&freq)) {
            QueryPerformanceCounter((_LARGE_INTEGER *)&preTime);
        }
    }
    else {
        QueryPerformanceCounter((_LARGE_INTEGER *)&currTime);
        execTime = (__int64)((double)(currTime - preTime) / freq * 1000000000ULL);
        CTaskUtil::DebugStrinFrwSystem("ExecTime(%lld): %llds %lldms %lldus %lldns\n", execTime, execTime / 1000000000ULL, (execTime / 1000000ULL) % 1000, (execTime / 1000ULL) % 1000, execTime % 1000);
    }
#endif
}

#if 0
void
CSysConfigUtil::ClenanUpFile()
{
#if defined (OS_LINUX)
#if defined (__XENO__)
    // Shell Command ºÎ¸£¸é ¸®´ª½º Ä¿³Î·Î º¯°æµÇ´Â °ÍÀ¸·Î º¸ÀÓ
    //system("find /home/user/dev/runtest/Release/*.log -mtime +3 -exec rm {} \\;");

    DIR           *dp = NULL;
    struct dirent *entry = NULL;
    struct stat   statbuf = {'\0', };
    time_t time = (rt_timer_read() / 1e9) - (3 * 86400);

    chdir("/home/user/dev/runtest/Release/");
    dp = opendir(".");
    if (dp == NULL) return;

    while ((entry = readdir(dp)) != NULL) {

        lstat(entry->d_name, &statbuf);
        // ÀÏ¹ÝÀûÀÎ ÆÄÀÏÀÌ¸é¼­
        if (S_ISREG(statbuf.st_mode)) {

            // ÆÄÀÏ È®ÀåÀÚ°¡ .log ÀÌ°í
            if (strstr(entry->d_name, ".log") != NULL) {

                // »ý¼ºµÈÁö 3ÀÏµÈ ÆÄÀÏ
                if (statbuf.st_atim.tv_sec < time) {

                    CTaskUtil::DebugStrinFrwSystem("%s: %s",entry->d_name, ctime(&statbuf.st_atim.tv_sec));
                    remove(entry->d_name);
                }
            }
        }

    }
    closedir(dp);
#endif
#endif
}
#endif

void CSysConfigUtil::GetSmartVisionVersion()
{
#if defined(__XENO__) && defined(_FUNC_SYTSTEM_UPDATE_INTEGRATION)
    // http://collab.doosanrobotics.com/pages/viewpage.action?pageId=51284878

    memset(SVMBOARD_VERSION,  0x00, MAX_SYMBOL_SIZE);
    memset(SVMBOARD_IPADDRESS, 0x00, MAX_SYMBOL_SIZE);

    int hSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (hSocket == -1) return;

    struct sockaddr_in tSvrSockAddr;
    socklen_t nSocklen = sizeof(struct sockaddr_in);

    memset(&tSvrSockAddr, 0, sizeof(struct sockaddr_in));
    tSvrSockAddr.sin_family      = AF_INET;
    tSvrSockAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    tSvrSockAddr.sin_port        = htons(12381);

    if (bind(hSocket,(sockaddr*)&tSvrSockAddr, nSocklen) == -1) {
        close(hSocket);
        return;
    }

    int nOption = 1;
    if (setsockopt(hSocket, SOL_SOCKET, SO_BROADCAST, (const char *)&nOption, sizeof(int)) == -1) {
        close(hSocket);
        return;
    }

    struct sockaddr_in tSndAddr;
    memset(&tSndAddr, 0, sizeof(struct sockaddr_in));
    tSndAddr.sin_family      = AF_INET;
    tSndAddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    tSndAddr.sin_port        = htons(12380);

    PACKET_DATA tPacket = { { HEADER_LENGTH, 1, 1, 0, 3001 }, } ;
    if (sendto(hSocket, (char*)&tPacket, HEADER_LENGTH, 0, (struct sockaddr *)&tSndAddr, nSocklen) == -1) {
        close(hSocket);
        return;
    }

    timeval timeout = { 1, 0 }; /* 1 second time-out */
    setsockopt(hSocket, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof(struct timeval));

    memset(&tSndAddr, 0, sizeof(struct sockaddr_in));

    char szBuffer[MAX_BUFFER_SIZE] = {'\0', };
    if (recvfrom(hSocket, szBuffer, MAX_BUFFER_SIZE, 0, (struct sockaddr *)&tSndAddr, &nSocklen) == -1) {
        close(hSocket);
        return;
    }

    LPPACKET_HEADER lpHDR = (LPPACKET_HEADER)szBuffer;
    if (lpHDR->_iCmdCode == 3002) {
        LPSEARCH_SVM lpSVM = (LPSEARCH_SVM)(szBuffer + HEADER_LENGTH);
        strncpy(SVMBOARD_VERSION,   lpSVM->_szVersion,   16);
        strncpy(SVMBOARD_IPADDRESS, lpSVM->_szIpAddress, 16);
        CTaskUtil::DebugStrinFrwSystem("SVM Search: vers(%s), addr(%s)", SVMBOARD_VERSION, SVMBOARD_IPADDRESS);

    }
    close(hSocket);
#endif
}
