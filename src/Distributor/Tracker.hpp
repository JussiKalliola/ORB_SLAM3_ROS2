#ifndef DISTRIBUTOR_TRACKER_H
#define DISTRIBUTOR_TRACKER_H

#include "System.h"

#include <mutex>
#include "stdlib.h"

class Tracker 
{
  public:  
    Tracker();
    ~Tracker();

    // main function
    void Run();

    void RequestFinish();
    bool isFinished();

    vector<double> vdProcessUsageCPU_pct;
    vector<double> vdTotalUsageCPU_pct;

    vector<int> vnProcessVRAM_kb;
    vector<long long> vllCurrentVRAM_kb;
    vector<long long> vllTotalVRAM_kb;

    vector<int> vnProcessPRAM_kb;
    vector<long long> vllCurrentPRAM_kb;
    vector<long long> vllTotalPRAM_kb;

    vector<std::chrono::steady_clock::time_point> vtTimes;


  protected:  
    double GetCurrentProcessCPU();
    double GetTotalCPU();
    
    int GetCurrentProcessVMemory();
    long long GetTotalVMemory();
    long long GetCurrentVMemory();

    int GetCurrentProcessPMemory();
    long long GetTotalPMemory();
    long long GetCurrentPMemory();

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;


    clock_t lastCPU, lastSysCPU, lastUserCPU;
    unsigned long long lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;
    int numProcessors;

};
//}

#endif // DISTRIBUTOR_MAP_HANDLER_H
