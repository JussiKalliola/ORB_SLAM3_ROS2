#include "./System.hpp"


#include "./Observer.hpp"
#include "./MapHandler.hpp"
#include "./KeyFramePublisher.hpp"
#include "./KeyFrameSubscriber.hpp"
#include "./Tracker.hpp"
#include "../slam/slam-wrapper-node.hpp"


//namespace TempDistributor
//{

System::System(std::string mStatSavePath):
    mStrStatSavePath(mStatSavePath)
{

    // initialize all the handlers
    mpMapHandler = new MapHandler();
    mpKeyFramePublisher = new KeyFramePublisher();
    mpKeyFrameSubscriber = new KeyFrameSubscriber();
    mpSystemTracker = new Tracker();

    
    // Start of a timer
    time_GlobalSystemStart = std::chrono::steady_clock::now();


    // Attach hnadlers to observer
    std::shared_ptr<Observer> pObserver(new Observer(mpMapHandler, mpKeyFramePublisher, mpKeyFrameSubscriber)); 
    mpObserver = pObserver;
    
    // Attach observer to handlers
    if(mpObserver)
    {
      mpMapHandler->AttachObserver(mpObserver);
      mpKeyFramePublisher->AttachObserver(mpObserver);
      mpKeyFrameSubscriber->AttachObserver(mpObserver);
    }
}

System::~System()
{

}

void System::LoopClosingStats2File()
{
    ofstream f;
    string sysId(std::getenv("SLAM_SYSTEM_ID"));
    std::string fileName=mStrStatSavePath + "PR_Stats-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "#Start time[ms], Total[ms]"<< endl; // vdDataQuery_ms, vdEstSim3_ms thse might be different sized vectors;
    for(int i=0; i<mpLoopCloser->vdPRTotal_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpLoopCloser->vtStartTimePR_ms[i];
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpLoopCloser->vdPRTotal_ms[i] << endl; 
    }

    f.close();

    fileName=mStrStatSavePath + "Merge_Stats-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "#Start time[ms], Merge Maps[ms], Welding BA[ms], Merge Opt.Ess.[ms], Total[ms], #KFs, #MPs" << endl; // vdDataQuery_ms, vdEstSim3_ms thse might be different sized vectors;
    for(int i=0; i<mpLoopCloser->vdMergeTotal_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpLoopCloser->vtStartTimeMerge_ms[i];
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpLoopCloser->vdMergeMaps_ms[i] << "," << mpLoopCloser->vdWeldingBA_ms[i] << "," << mpLoopCloser->vdMergeOptEss_ms[i] << "," << mpLoopCloser->vdMergeTotal_ms[i] << "," << mpLoopCloser->vnMergeKFs[i] << "," << mpLoopCloser->vnMergeMPs[i] << endl; 
    }


    f.close();



    fileName=mStrStatSavePath + "LoopClosure_Stats-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "#Start time[ms], Fusion[ms], Opt.Ess.[ms], Total[ms]" << endl; // vdDataQuery_ms, vdEstSim3_ms thse might be different sized vectors;
    for(int i=0; i<mpLoopCloser->vdLoopTotal_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpLoopCloser->vtStartTimeLoop_ms[i];
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpLoopCloser->vdLoopFusion_ms[i] << "," << mpLoopCloser->vdLoopOptEss_ms[i] << "," << mpLoopCloser->vdLoopTotal_ms[i] << endl; 
    }


    f.close();

    fileName=mStrStatSavePath + "GBA_Stats-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);


    f << "#Start time[ms], GBA[ms], Update map[ms], Total[ms]" << endl; // vdDataQuery_ms, vdEstSim3_ms thse might be different sized vectors;
    for(int i=0; i<mpLoopCloser->vdFGBATotal_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpLoopCloser->vtStartTimeFGBA_ms[i];
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpLoopCloser->vdGBA_ms[i] << "," << mpLoopCloser->vdUpdateMap_ms[i] << "," << mpLoopCloser->vdFGBATotal_ms[i]<< endl; 
    }


    f.close();
}

void System::LocalMapStats2File()
{
    ofstream f;
    string sysId(std::getenv("SLAM_SYSTEM_ID"));
    std::string fileName=mStrStatSavePath + "LocalMapTimeStats-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "#Start time[ms], Stereo rect[ms], MP culling[ms], MP creation[ms], LBA[ms], KF culling[ms], Total[ms]" << endl;
    for(int i=0; i<mpLocalMapper->vdLMTotal_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpLocalMapper->vtStartTimeLM_ms[i];
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpLocalMapper->vdKFInsert_ms[i] << "," << mpLocalMapper->vdMPCulling_ms[i] << ","
          << mpLocalMapper->vdMPCreation_ms[i] << "," << mpLocalMapper->vdLBASync_ms[i] << ","
          << mpLocalMapper->vdKFCullingSync_ms[i] <<  "," << mpLocalMapper->vdLMTotal_ms[i] << endl;
    }

    f.close();

    fileName=mStrStatSavePath + "LBA_Stats-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "#Start time[ms], LBA time[ms], KF opt[#], KF fixed[#], MP[#], Edges[#]" << endl;
    for(int i=0; i<mpLocalMapper->vdLBASync_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpLocalMapper->vtStartTimeLBA_ms[i];
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpLocalMapper->vdLBASync_ms[i] << "," << mpLocalMapper->vnLBA_KFopt[i] << ","
          << mpLocalMapper->vnLBA_KFfixed[i] << "," << mpLocalMapper->vnLBA_MPs[i] << ","
          << mpLocalMapper->vnLBA_edges[i] << endl;
    }


    f.close();
}

void System::TrackStats2File()
{
    ofstream f;
    string sysId(std::getenv("SLAM_SYSTEM_ID"));

    if(mpObserver->GetTaskModule() == 1)
    {
        std::string fileName=mStrStatSavePath + "LostTrackStats.txt";
        f.open(fileName);
        f << fixed << setprecision(6);

        f << "Time[ms]" << endl;

        for(int i=0;i<mpTracker->vtLostTrackTime_ms.size(); ++i)
        {
            std::chrono::steady_clock::time_point time_StartAction = mpTracker->vtLostTrackTime_ms[i];
            long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
            f << timeSinceStart << endl;
        }


        f.close();

    }
    //std::string fileName=mStrStatSavePath + "LostTrackStats-" + sysId + ".txt";
    //f.open(fileName);
    //f << fixed << setprecision(6);

    //f << "#Start time[ms], Image Rect[ms], Image Resize[ms], ORB ext[ms], Stereo match[ms], IMU preint[ms], Pose pred[ms], LM track[ms], KF dec[ms], Total[ms]" << endl;

    //for(int i=0; i<mpTracker->vtStartTime_ms.size(); ++i)
    //{
    //    std::chrono::steady_clock::time_point time_StartAction = mpTracker->vtStartTime_ms[i];
    //    long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();

    //    double stereo_rect = 0.0;
    //    if(!mpTracker->vdRectStereo_ms.empty())
    //    {
    //        stereo_rect = mpTracker->vdRectStereo_ms[i];
    //    }

    //    double resize_image = 0.0;
    //    if(!mpTracker->vdResizeImage_ms.empty())
    //    {
    //        resize_image = mpTracker->vdResizeImage_ms[i];
    //    }

    //    double orb_extract = 0.0;
    //    if(mpTracker->vdORBExtract_ms.size() > i)
    //    {
    //      
    //    }

    //    double stereo_match = 0.0;
    //    if(!mpTracker->vdStereoMatch_ms.empty())
    //    {
    //        stereo_match = mpTracker->vdStereoMatch_ms[i];
    //    }

    //    double imu_preint = 0.0;
    //    if(!mpTracker->vdIMUInteg_ms.empty())
    //    {
    //        imu_preint = mpTracker->vdIMUInteg_ms[i];
    //    }

    //    f << timeSinceStart << "," << stereo_rect << "," << resize_image << "," << mpTracker->vdORBExtract_ms[i] << "," << stereo_match << "," << imu_preint << ","
    //      << mpTracker->vdPosePred_ms[i] <<  "," << mpTracker->vdLMTrack_ms[i] << "," << mpTracker->vdNewKF_ms[i] << "," << mpTracker->vdTrackTotal_ms[i] << endl;
    //}

    //f.close();
}

void System::SystemStats2File()
{
    ofstream f;
    string sysId(std::getenv("SLAM_SYSTEM_ID"));
    std::string fileName=mStrStatSavePath + "SystemStats-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Time from start [ms], #CPU Process[pct], CPU Total[pct], VMem Process[kb], VMem Current[kb], VMem Total[kb], RAM Process[kb], RAM Current[kb], RAM Total[kb]" << endl;

    for(int i=0; i<mpSystemTracker->vdProcessUsageCPU_pct.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpSystemTracker->vtTimes[i];  
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpSystemTracker->vdProcessUsageCPU_pct[i] << "," << mpSystemTracker->vdTotalUsageCPU_pct[i] << "," <<  mpSystemTracker->vnProcessVRAM_kb[i] << "," << mpSystemTracker->vllCurrentVRAM_kb[i] << "," << mpSystemTracker->vllTotalVRAM_kb[i] << "," << mpSystemTracker->vnProcessPRAM_kb[i] << "," << mpSystemTracker->vllCurrentPRAM_kb[i] << "," << mpSystemTracker->vllTotalPRAM_kb[i] << endl;
    }

    f.close();
}

void System::TimeStats2File()
{
    // KF Publish
    ofstream f;
    string sysId(std::getenv("SLAM_SYSTEM_ID"));
    std::string fileName=mStrStatSavePath + "KeyFramePublishTime-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Time from start [ms], KF Conversion[ms], KeyFrame PreSave[ms], MPs PreSave[ms], Total[ms]" << endl;

    for(int i=0; i<mpKeyFramePublisher->vdOrb2RosProcKF_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpKeyFramePublisher->vtTimes[i];  
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpKeyFramePublisher->vdOrb2RosConvKF_ms[i] << "," << mpKeyFramePublisher->vdPreSaveKF_ms[i] << "," <<  mpKeyFramePublisher->vdPreSaveMP_ms[i] << "," << mpKeyFramePublisher->vdOrb2RosProcKF_ms[i] << endl;
    }

    f.close();

    // Map Publish
    fileName=mStrStatSavePath + "MapPublishTime-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Time from start [ms], Conversion(KF+MPs)[ms], Presave[ms], Total[ms]" << endl;

    for(int i=0; i<mpMapHandler->vdOrb2RosProcMap_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpMapHandler->vtTimesPubMap[i];  
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpMapHandler->vdOrb2RosConvMap_ms[i] << "," << mpMapHandler->vdPreSaveMap_ms[i] << "," <<  mpMapHandler->vdOrb2RosProcMap_ms[i]<< endl;
    }

    f.close();

    // Atlas Publish
    fileName=mStrStatSavePath + "AtlasPublishTime-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Time from start [ms], Conversion(KF+MPs)[ms], Presave[ms], Total[ms]" << endl;

    for(int i=0; i<mpMapHandler->vdOrb2RosProcAtlas_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpMapHandler->vtTimesPubAtlas[i];  
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpMapHandler->vdOrb2RosConvAtlas_ms[i] << "," << mpMapHandler->vdPreSaveAtlas_ms[i] << "," <<  mpMapHandler->vdOrb2RosProcAtlas_ms[i]<< endl;
    }

    f.close();

    // KF Subscription
    fileName=mStrStatSavePath + "KeyFrameSubscribeTime-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Time from start [ms], KF Conversion[ms], Prep data[ms], MPs Conversion[ms], KF PostLoad[ms], MPs PostLoad[ms], KF Injection[ms], MPs Injection[ms], Total[ms], #KFs, #MPs, #New MPs, #Update MPs" << endl;

    for(int i=0; i<mpKeyFrameSubscriber->vdRos2OrbProcKF_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpKeyFrameSubscriber->vtTimes[i];  
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpKeyFrameSubscriber->vdRos2OrbConvKF_ms[i] << "," << mpKeyFrameSubscriber->vdPrepDataKF_ms[i] << "," << mpKeyFrameSubscriber->vdRos2OrbConvMP_ms[i] << "," <<  mpKeyFrameSubscriber->vdPostLoadKF_ms[i] << "," << mpKeyFrameSubscriber->vdPostLoadMP_ms[i] << "," << mpKeyFrameSubscriber->vdInjectKF_ms[i] << "," << mpKeyFrameSubscriber->vdInjectMP_ms[i] << "," << mpKeyFrameSubscriber->vdRos2OrbProcKF_ms[i] << "," << mpKeyFrameSubscriber->vnKFAmount[i] << "," << mpKeyFrameSubscriber->vnMPAmount[i] << "," << mpKeyFrameSubscriber->vnNewMPAmount[i]<< "," << mpKeyFrameSubscriber->vnUpdateMPAmount[i]<< endl;
    }

    f.close();
        
    // Map Subscription
    fileName=mStrStatSavePath + "MapSubscribeTime-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Time from start [ms], #MPs PostLoad[ms], KFs PostLoad[ms], KFs Conversion[ms], MPs Conversion[ms], KFs+MPs Conversion[ms], MPs Injection[ms], KFs Injection[ms], Map Conversion[ms], Total[ms], #MPs/msg,  #KFs/msg, #MPs Total, #KFs Total" << endl;

    for(int i=0; i<mpMapHandler->vdRos2OrbProcMap_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpMapHandler->vtTimesSubMap[i];  
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpMapHandler->vdPostLoadMPMap_ms[i] << "," << mpMapHandler->vdPostLoadKFMap_ms[i] << "," << mpMapHandler->vdRos2OrbKFConvMap_ms[i] << "," << mpMapHandler->vdRos2OrbMPConvMap_ms[i] << "," <<  mpMapHandler->vdRos2OrbDataConvMap_ms[i] << "," << mpMapHandler->vdInjectMPMap_ms[i] << "," << mpMapHandler->vdInjectKFMap_ms[i] << "," << mpMapHandler->vdUpdateMap_ms[i] << "," << mpMapHandler->vdRos2OrbProcMap_ms[i] << "," << mpMapHandler->vnNumberOfMPs[i] << "," << mpMapHandler->vnNumberOfKFs[i] << "," << mpMapHandler->vnNumberOfMPsTotal[i] << "," << mpMapHandler->vnNumberOfKFsTotal[i]<< endl;
    }

    f.close();

    // Atlas Subscription
    fileName=mStrStatSavePath + "AtlasSubscribeTime-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Time from start [ms], #MPs PostLoad[ms], KFs PostLoad[ms], KFs+MPs Conversion[ms], MPs Injection[ms], KFs Injection[ms], Map Conversion[ms], Total[ms]" << endl;

    for(int i=0; i<mpMapHandler->vdRos2OrbProcAtlas_ms.size(); ++i)
    {
        std::chrono::steady_clock::time_point time_StartAction = mpMapHandler->vtTimesSubAtlas[i];  
        long int timeSinceStart = std::chrono::duration_cast<std::chrono::duration<long int,std::milli> >( time_StartAction - time_GlobalSystemStart).count();
        f << timeSinceStart << "," << mpMapHandler->vdPostLoadMPAtlas_ms[i] << "," << mpMapHandler->vdPostLoadKFAtlas_ms[i] << "," <<  mpMapHandler->vdRos2OrbDataConvAtlas_ms[i] << "," << mpMapHandler->vdInjectMPAtlas_ms[i] << "," << mpMapHandler->vdInjectKFAtlas_ms[i] << "," << mpMapHandler->vdUpdateAtlas_ms[i] << "," << mpMapHandler->vdRos2OrbProcAtlas_ms[i] << endl;
    }

    f.close();

    // KF Latency
    fileName=mStrStatSavePath + "KeyFrameLatencyTime-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Latency[ms]" << endl;

    for(int i=0; i<pSLAMNode->vdLatencyKF_ms.size(); ++i)
    {
        f << pSLAMNode->vdLatencyKF_ms[i]  << endl;
    }

    f.close();

    // Map Latency
    fileName=mStrStatSavePath + "MapLatencyTime-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Latency[ms]" << endl;

    for(int i=0; i<pSLAMNode->vdLatencyMap_ms.size(); ++i)
    {
        f << pSLAMNode->vdLatencyMap_ms[i]  << endl;
    }

    f.close();

    // Atlas Latency
    fileName=mStrStatSavePath + "AtlasLatencyTime-" + sysId + ".txt";
    f.open(fileName);
    f << fixed << setprecision(6);

    f << "Latency[ms]" << endl;

    for(int i=0; i<pSLAMNode->vdLatencyAtlas_ms.size(); ++i)
    {
        f << pSLAMNode->vdLatencyAtlas_ms[i]  << endl;
    }

    f.close();
}



double calcAverage(vector<double> v_times)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}

double calcAverage(vector<int> v_times)
{
    double accum = 0;
    int total = 0;
    for(double value : v_times)
    {
        if(value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(vector<int> v_values, double average)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}

double calcAverage(vector<long long> v_times)
{
    double accum = 0;
    int total = 0;
    for(double value : v_times)
    {
        if(value == 0)
            continue;
        accum += value;
        total++;
    }

    return accum / total;
}

double calcDeviation(vector<long long> v_values, double average)
{
    double accum = 0;
    int total = 0;
    for(double value : v_values)
    {
        if(value == 0)
            continue;
        accum += pow(value - average, 2);
        total++;
    }
    return sqrt(accum / total);
}



void System::PrintTimeStats()
{
    /************************************************/
    /*              PUBLISH                         */
    /************************************************/

    std::cout << std::endl << "TIME STATS in ms (mean$\\pm$std)" << std::endl;
    //f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Publish" << std::setprecision(5) << std::endl << std::endl;
    //f << "---------------------------" << std::endl;
    //f << "Tracking" << std::setprecision(5) << std::endl << std::endl;
    double average, deviation;
    std::cout << "KeyFrame" << std::endl;
    average = calcAverage(mpKeyFramePublisher->vdOrb2RosConvKF_ms);
    deviation = calcDeviation(mpKeyFramePublisher->vdOrb2RosConvKF_ms, average);
    std::cout << "KF ORB_SLAM3->ROS2: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpKeyFramePublisher->vdPreSaveKF_ms);
    deviation = calcDeviation(mpKeyFramePublisher->vdPreSaveKF_ms, average);
    std::cout << "KF PreSave: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    average = calcAverage(mpKeyFramePublisher->vdPreSaveMP_ms);
    deviation = calcDeviation(mpKeyFramePublisher->vdPreSaveMP_ms, average);
    std::cout << "MPs PreSave: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    //
    average = calcAverage(mpKeyFramePublisher->vdOrb2RosProcKF_ms);
    deviation = calcDeviation(mpKeyFramePublisher->vdOrb2RosProcKF_ms, average);
    std::cout << "Total KeyFrame: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    //average = calcAverage(mpObserver->vdOrb2RosConvMP_ms);
    //deviation = calcDeviation(mpObserver->vdOrb2RosConvMP_ms, average);
    //std::cout << "MapPoint ORB_SLAM3->ROS2 (Conversion): " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    std::cout<<std::endl;
    std::cout << "Local Map" << std::endl;
    average = calcAverage(mpMapHandler->vdOrb2RosConvMap_ms);
    deviation = calcDeviation(mpMapHandler->vdOrb2RosConvMap_ms, average);
    std::cout << "Map ORB_SLAM3->ROS2: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpMapHandler->vdPreSaveMap_ms);
    deviation = calcDeviation(mpMapHandler->vdPreSaveMap_ms, average);
    std::cout << "Map PreSave: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpMapHandler->vdOrb2RosProcMap_ms);
    deviation = calcDeviation(mpMapHandler->vdOrb2RosProcMap_ms, average);
    std::cout << "Total Map: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    std::cout<<std::endl;
    std::cout << "Global Map" << std::endl;
    average = calcAverage(mpMapHandler->vdOrb2RosConvAtlas_ms);
    deviation = calcDeviation(mpMapHandler->vdOrb2RosConvAtlas_ms, average);
    std::cout << "Atlas ORB_SLAM3->ROS2: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpMapHandler->vdPreSaveAtlas_ms);
    deviation = calcDeviation(mpMapHandler->vdPreSaveAtlas_ms, average);
    std::cout << "Atlas PreSave: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;


    average = calcAverage(mpMapHandler->vdOrb2RosProcAtlas_ms);
    deviation = calcDeviation(mpMapHandler->vdOrb2RosProcAtlas_ms, average);
    std::cout << "Total Atlas: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    std::cout << std::endl << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "KeyFrames published: " << mpKeyFramePublisher->vdOrb2RosProcKF_ms.size() << std::endl;
    std::cout << "Local Maps published: " << mpMapHandler->vdOrb2RosProcMap_ms.size() << std::endl;
    std::cout << "Global Maps published: " << mpMapHandler->vdOrb2RosProcAtlas_ms.size() << std::endl;

    /************************************************/
    /*              SUBSCRIPTION                    */
    /************************************************/

    // Subscription time stats
    std::cout << std::endl << std::endl << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Subscription" << std::endl << std::endl;

    std::cout << "KeyFrame" << std::endl;
    average = calcAverage(mpKeyFrameSubscriber->vdRos2OrbConvKF_ms);
    deviation = calcDeviation(mpKeyFrameSubscriber->vdRos2OrbConvKF_ms, average);
    std::cout << "KF ROS2->ORB_SLAM3: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpKeyFrameSubscriber->vdRos2OrbConvMP_ms);
    deviation = calcDeviation(mpKeyFrameSubscriber->vdRos2OrbConvMP_ms, average);
    std::cout << "MPs ROS2->ORB_SLAM3: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpKeyFrameSubscriber->vdPostLoadKF_ms);
    deviation = calcDeviation(mpKeyFrameSubscriber->vdPostLoadKF_ms, average);
    std::cout << "KF PostLoad: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    average = calcAverage(mpKeyFrameSubscriber->vdPostLoadMP_ms);
    deviation = calcDeviation(mpKeyFrameSubscriber->vdPostLoadMP_ms, average);
    std::cout << "MPs PostLoad: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    average = calcAverage(mpKeyFrameSubscriber->vdInjectKF_ms);
    deviation = calcDeviation(mpKeyFrameSubscriber->vdInjectKF_ms, average);
    std::cout << "KF Injection: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpKeyFrameSubscriber->vdInjectMP_ms);
    deviation = calcDeviation(mpKeyFrameSubscriber->vdInjectMP_ms, average);
    std::cout << "MPs Injection: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpKeyFrameSubscriber->vdRos2OrbProcKF_ms);
    deviation = calcDeviation(mpKeyFrameSubscriber->vdRos2OrbProcKF_ms, average);
    std::cout << "Total KeyFrame: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    std::cout << std::endl;
        
    std::cout << "Local Map" << std::endl;
    average = calcAverage(mpMapHandler->vdPostLoadMPMap_ms);
    deviation = calcDeviation(mpMapHandler->vdPostLoadMPMap_ms, average);
    std::cout << "MPs PostLoad: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        
    average = calcAverage(mpMapHandler->vdPostLoadKFMap_ms);
    deviation = calcDeviation(mpMapHandler->vdPostLoadKFMap_ms, average);
    std::cout << "KFs PostLoad: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        
    average = calcAverage(mpMapHandler->vdRos2OrbDataConvMap_ms);
    deviation = calcDeviation(mpMapHandler->vdRos2OrbDataConvMap_ms, average);
    std::cout << "KFs+MPs Conversion: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    average = calcAverage(mpMapHandler->vdInjectMPMap_ms);
    deviation = calcDeviation(mpMapHandler->vdInjectMPMap_ms, average);
    std::cout << "MPs Injection: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        
    average = calcAverage(mpMapHandler->vdInjectKFMap_ms);
    deviation = calcDeviation(mpMapHandler->vdInjectKFMap_ms, average);
    std::cout << "KFs Injection: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpMapHandler->vdUpdateMap_ms);
    deviation = calcDeviation(mpMapHandler->vdUpdateMap_ms, average);
    std::cout << "Map update (conversion, postload, injection): " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;    
        
    average = calcAverage(mpMapHandler->vdRos2OrbProcMap_ms);
    deviation = calcDeviation(mpMapHandler->vdRos2OrbProcMap_ms, average);
    std::cout << "Total Map: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    std::cout << std::endl;
    
    std::cout << "Global Map" << std::endl;

    average = calcAverage(mpMapHandler->vdPostLoadMPAtlas_ms);
    deviation = calcDeviation(mpMapHandler->vdPostLoadMPAtlas_ms, average);
    std::cout << "MPs PostLoad: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        
    average = calcAverage(mpMapHandler->vdPostLoadKFAtlas_ms);
    deviation = calcDeviation(mpMapHandler->vdPostLoadKFAtlas_ms, average);
    std::cout << "KFs PostLoad: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        
    average = calcAverage(mpMapHandler->vdRos2OrbDataConvAtlas_ms);
    deviation = calcDeviation(mpMapHandler->vdRos2OrbDataConvAtlas_ms, average);
    std::cout << "KFs+MPs Conversion: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    average = calcAverage(mpMapHandler->vdInjectMPAtlas_ms);
    deviation = calcDeviation(mpMapHandler->vdInjectMPAtlas_ms, average);
    std::cout << "MPs Injection: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
        
    average = calcAverage(mpMapHandler->vdInjectKFAtlas_ms);
    deviation = calcDeviation(mpMapHandler->vdInjectKFAtlas_ms, average);
    std::cout << "KFs Injection: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    average = calcAverage(mpMapHandler->vdUpdateAtlas_ms);
    deviation = calcDeviation(mpMapHandler->vdUpdateAtlas_ms, average);
    std::cout << "Atlas update (conversion, postload, injection): " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;    

    average = calcAverage(mpMapHandler->vdRos2OrbProcAtlas_ms);
    deviation = calcDeviation(mpMapHandler->vdRos2OrbProcAtlas_ms, average);
    std::cout << "Total Atlas: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    std::cout << std::endl << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "KeyFrames received: " << mpKeyFrameSubscriber->vdRos2OrbProcKF_ms.size() << std::endl;
    std::cout << "Local Maps received: " << mpMapHandler->vdRos2OrbProcMap_ms.size() << std::endl;
    std::cout << "Global Maps received: " << mpMapHandler->vdRos2OrbProcAtlas_ms.size() << std::endl;


    std::cout << std::endl <<std::endl << std::endl;


    /************************************************/
    /*                    SYSTEM                    */
    /************************************************/

    std::cout << std::endl << "SYSTEM UTILIZATION STATS (mean$\\pm$std)" << std::endl;
    //f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "CPU in percent" << std::setprecision(5) << std::endl;
    //f << "---------------------------" << std::endl;
    //f << "Tracking" << std::setprecision(5) << std::endl << std::endl;

    average = calcAverage(mpSystemTracker->vdProcessUsageCPU_pct);
    deviation = calcDeviation(mpSystemTracker->vdProcessUsageCPU_pct, average);
    std::cout << "Current process: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    average = calcAverage(mpSystemTracker->vdTotalUsageCPU_pct);
    deviation = calcDeviation(mpSystemTracker->vdTotalUsageCPU_pct, average);
    std::cout << "Total: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;

    std::cout << std::endl;
    std::cout << "Virtual Memory in kb" << std::endl;

    average = calcAverage(mpSystemTracker->vnProcessVRAM_kb);
    deviation = calcDeviation(mpSystemTracker->vnProcessVRAM_kb, average);
    std::cout << "Current Process: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    average = calcAverage(mpSystemTracker->vllCurrentVRAM_kb);
    deviation = calcDeviation(mpSystemTracker->vllCurrentVRAM_kb, average);
    std::cout << "Current: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    average = calcAverage(mpSystemTracker->vllTotalVRAM_kb);
    deviation = calcDeviation(mpSystemTracker->vllTotalVRAM_kb, average);
    std::cout << "Total: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;


    std::cout << std::endl;
    std::cout << "Physical Memory in kb" << std::endl;

    average = calcAverage(mpSystemTracker->vnProcessPRAM_kb);
    deviation = calcDeviation(mpSystemTracker->vnProcessPRAM_kb, average);
    std::cout << "Current Process: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    average = calcAverage(mpSystemTracker->vllCurrentPRAM_kb);
    deviation = calcDeviation(mpSystemTracker->vllCurrentPRAM_kb, average);
    std::cout << "Current: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    average = calcAverage(mpSystemTracker->vllTotalPRAM_kb);
    deviation = calcDeviation(mpSystemTracker->vllTotalPRAM_kb, average);
    std::cout << "Total: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    

    /************************************************/
    /*                    NETWORK                   */
    /************************************************/

    std::cout << std::endl << "NETWORK STATS (mean$\\pm$std)" << std::endl;
    //f << " TIME STATS in ms (mean$\\pm$std)" << std::endl;
    std::cout << "---------------------------" << std::endl;
    std::cout << "Latency in ms" << std::setprecision(5) << std::endl;
    //f << "---------------------------" << std::endl;
    //f << "Tracking" << std::setprecision(5) << std::endl << std::endl;

    average = calcAverage(pSLAMNode->vdLatencyKF_ms);
    deviation = calcDeviation(pSLAMNode->vdLatencyKF_ms, average);
    std::cout << "KFs: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    
    average = calcAverage(pSLAMNode->vdLatencyMap_ms);
    deviation = calcDeviation(pSLAMNode->vdLatencyMap_ms, average);
    std::cout << "Maps: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
    
    average = calcAverage(pSLAMNode->vdLatencyAtlas_ms);
    deviation = calcDeviation(pSLAMNode->vdLatencyAtlas_ms, average);
    std::cout << "Atlas: " << average << "$\\pm$" << deviation << std::endl;
    //f << "Stereo Rectification: " << average << "$\\pm$" << deviation << std::endl;
}

void System::ShutDown()
{
    mpMapHandler->RequestFinish();
    mpKeyFrameSubscriber->RequestFinish();
    mpKeyFramePublisher->RequestFinish();
    mpSystemTracker->RequestFinish();

    TrackStats2File();
    LoopClosingStats2File();
    LocalMapStats2File();
    TimeStats2File();
    SystemStats2File();

    PrintTimeStats();
}

void System::LaunchThreads()
{
    mptMapHandler = new thread(&MapHandler::Run, mpMapHandler);
    mptKeyFramePublisher = new thread(&KeyFramePublisher::Run, mpKeyFramePublisher);
    mptKeyFrameSubscriber = new thread(&KeyFrameSubscriber::Run, mpKeyFrameSubscriber);
    mptSystemTracker = new thread(&Tracker::Run, mpSystemTracker);
}
MapHandler* System::GetMapHandler()
{
  return mpMapHandler;
}

KeyFrameSubscriber* System::GetKeyFrameSubscriber()
{
  return mpKeyFrameSubscriber;
}

KeyFramePublisher* System::GetKeyFramePublisher()
{
  return mpKeyFramePublisher;
}
    


void System::AttachORBSLAMSystem(ORB_SLAM3::System* mSLAM)
{
    pSLAM = mSLAM;

    // So that we can save lm stats to file in sync with global time
    mpTracker=pSLAM->GetTrackerPtr();
    mpLocalMapper=pSLAM->GetMapperPtr();
    mpLoopCloser=pSLAM->GetLoopClosingPtr();

    // Attach node to the others
    mpObserver->AttachORBSLAMSystem(mSLAM);
    mpMapHandler->AttachORBSLAMSystem(mSLAM);
    mpKeyFramePublisher->AttachORBSLAMSystem(mSLAM);
    mpKeyFrameSubscriber->AttachORBSLAMSystem(mSLAM);
}

void System::AttachSLAMNode(std::shared_ptr<SlamWrapperNode> slam_node)
{
    pSLAMNode = slam_node;

    // Attach node to the others
    mpObserver->AttachSLAMNode(slam_node);
    mpMapHandler->AttachSLAMNode(slam_node);
    mpKeyFramePublisher->AttachSLAMNode(slam_node);
}


std::shared_ptr<Observer> System::GetObserver()
{
  return mpObserver;
}

//} // namespace TempDistributor
