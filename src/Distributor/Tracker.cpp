#include "./Tracker.hpp"

#include "stdio.h"
#include "string.h"
#include "sys/times.h"
//#include "sys/vtimes.h"
#include "sys/types.h"
#include "sys/sysinfo.h"



Tracker::Tracker()
{
    FILE* file;
    struct tms timeSample;
    char line[128];

    // CPU by Process
    lastCPU = times(&timeSample);
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;

    file = fopen("/proc/cpuinfo", "r");
    numProcessors = 0;
    while(fgets(line, 128, file) != NULL){
        if (strncmp(line, "processor", 9) == 0) numProcessors++;
    }
    fclose(file);

    // Total CPU
    file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %llu %llu %llu %llu", &lastTotalUser, &lastTotalUserLow,
        &lastTotalSys, &lastTotalIdle);
    fclose(file);
}

Tracker::~Tracker()
{

}

// main function
void Tracker::Run()
{
    mbFinished = false;
    std::cout << "System Tracker Thread has been started." << std::endl;
    
    while(1)
    {
      // Collect system utilization
      // CPU
      vdProcessUsageCPU_pct.push_back(GetCurrentProcessCPU());
      vdTotalUsageCPU_pct.push_back(GetTotalCPU());

      // Memory
      // Virtual Memory
      vnProcessVRAM_kb.push_back(GetCurrentProcessVMemory());
      vllCurrentVRAM_kb.push_back(GetCurrentVMemory());
      vllTotalVRAM_kb.push_back(GetTotalVMemory());

      // Physical RAM
      vnProcessPRAM_kb.push_back(GetCurrentProcessPMemory());
      vllCurrentPRAM_kb.push_back(GetCurrentPMemory());
      vllTotalPRAM_kb.push_back(GetTotalPMemory());

      vtTimes.push_back(std::chrono::steady_clock::now());


      usleep(100000);
      if(CheckFinish())
          break;
    }

    SetFinish();
}

double Tracker::GetCurrentProcessCPU()
{
    struct tms timeSample;
    clock_t now;
    double percent;

    now = times(&timeSample);
    if (now <= lastCPU || timeSample.tms_stime < lastSysCPU ||
        timeSample.tms_utime < lastUserCPU){
        //Overflow detection. Just skip this value.
        percent = -1.0;
    }
    else{
        percent = (timeSample.tms_stime - lastSysCPU) +
            (timeSample.tms_utime - lastUserCPU);
        percent /= (now - lastCPU);
        percent /= numProcessors;
        percent *= 100;
    }
    lastCPU = now;
    lastSysCPU = timeSample.tms_stime;
    lastUserCPU = timeSample.tms_utime;

    return percent;
}

double Tracker::GetTotalCPU()
{
    double percent;
    FILE* file;
    unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;

    file = fopen("/proc/stat", "r");
    fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow,
        &totalSys, &totalIdle);
    fclose(file);

    if (totalUser < lastTotalUser || totalUserLow < lastTotalUserLow ||
        totalSys < lastTotalSys || totalIdle < lastTotalIdle){
        //Overflow detection. Just skip this value.
        percent = -1.0;
    }
    else{
        total = (totalUser - lastTotalUser) + (totalUserLow - lastTotalUserLow) +
            (totalSys - lastTotalSys);
        percent = total;
        total += (totalIdle - lastTotalIdle);
        percent /= total;
        percent *= 100;
    }

    lastTotalUser = totalUser;
    lastTotalUserLow = totalUserLow;
    lastTotalSys = totalSys;
    lastTotalIdle = totalIdle;

    return percent;
}


int parseLine(char* line){
    // This assumes that a digit will be found and the line ends in " Kb".
    int i = strlen(line);
    const char* p = line;
    while (*p <'0' || *p > '9') p++;
    line[i-3] = '\0';
    i = atoi(p);
    return i;
}

int Tracker::GetCurrentProcessVMemory()
{
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmSize:", 7) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

int Tracker::GetCurrentProcessPMemory()
{
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];

    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmRSS:", 6) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

long long Tracker::GetTotalVMemory()
{
    struct sysinfo memInfo;

    sysinfo (&memInfo);
    long long totalVirtualMem = memInfo.totalram;
    //Add other values in next statement to avoid int overflow on right hand side...
    totalVirtualMem += memInfo.totalswap;
    totalVirtualMem *= memInfo.mem_unit;
    return totalVirtualMem;
}

long long Tracker::GetCurrentVMemory()
{
    struct sysinfo memInfo;

    sysinfo (&memInfo);
    long long virtualMemUsed = memInfo.totalram - memInfo.freeram;
    //Add other values in next statement to avoid int overflow on right hand side...
    virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
    virtualMemUsed *= memInfo.mem_unit;
    return virtualMemUsed;
}

long long Tracker::GetTotalPMemory()
{
    struct sysinfo memInfo;

    sysinfo (&memInfo);
    long long totalPhysMem = memInfo.totalram;
    //Multiply in next statement to avoid int overflow on right hand side...
    totalPhysMem *= memInfo.mem_unit;
    return totalPhysMem;
}

long long Tracker::GetCurrentPMemory()
{
    struct sysinfo memInfo;

    sysinfo (&memInfo);
    long long physMemUsed = memInfo.totalram - memInfo.freeram;
    //Multiply in next statement to avoid int overflow on right hand side...
    physMemUsed *= memInfo.mem_unit;
    return physMemUsed;
}

void Tracker::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Tracker::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Tracker::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    //unique_lock<mutex> lock2(mMutexStop);
    //mbStopped = true;
}

bool Tracker::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}
