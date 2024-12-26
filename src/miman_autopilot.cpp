#include "miman_config.h"
#include "miman_coms.h"
#include "miman_csp.h"
#include "miman_imgui.h"
#include "miman_orbital.h"
#include "miman_radial.h"
#include "miman_model.h"
#include "miman_ftp.h"
#include "miman_autopilot.h"


extern StateCheckUnit State;
extern Setup * setup;
extern Console console;
extern pthread_t p_thread[16];
extern int NowTLE;
extern int NowFTP;
extern bool sgp4check;
extern int PingCounter;
extern int BeaconCounter;
extern CmdGenerator_GS * SatCMD[256];

int FindValuablePath()
{
    int Valuable = 0;
    for(int i = 0 ; i < 64; i++)
    {
        if(State.Satellites[GetNowTracking()]->MaxElevation(i) > 10.0f)
        {
            Valuable = i;
            break;
        }
    }
    return Valuable;
}

void * AutoPilot(void *)
{
    console.AddLog("[Auto]Start AutoPilot.");
    int AutoRes = AutoPilot_task();
    if(AutoRes)
        console.AddLog("[ERROR]##[Auto]AutoPilot Stopped with Error Code : %d", AutoRes);
    if(AutoRes != -2)
    {
        State.Doppler = false;
        pthread_join(p_thread[7], NULL);
        console.AddLog("[OK]##[Auto]Stop Doppler Correction.");

        State.Engage = false;
        State.Doppler = false;
        pthread_join(p_thread[1], NULL);
        console.AddLog("[OK]##[Auto]Finish Engaging.");
        
        RotatorSetTo(0, 0);
        console.AddLog("[Auto]Beacon Success : %d, Ping Success : %d CMD Success : %d", BeaconCounter, PingCounter, State.CMDCount);
        BeaconCounter = 0;
        PingCounter = 0;
    }
    State.Successed = 0;
    return NULL;

}

static bool NowOn(int SatIndex, int NextValuablePath)
{
    return (DateTime::Now(false) - State.Satellites[SatIndex]->_nextlos[NextValuablePath]).TotalSeconds() < 0;
}


int AutoPilot_task()
{
    bool BeaconAvailable = false;
    bool PingAvailable = false;
    char NowTrackingNameBuffer[32];
    int NowTracking = GetNowTracking();

    if(NowTracking == -2)
    {
        console.AddLog("[ERROR]##[Auto]You Tried Autopilot without Tracking. Please Check Again.");
        State.Autopilot = false;
        return -2;
    }

    sprintf(NowTrackingNameBuffer, State.Satellites[NowTracking]->Name());

    while(State.Autopilot)
    {
        bool BeaconAvailable = false;
        bool PingAvailable = false;
        bool CalibrationSuccess = false; // Add
        printf("Start AutoPilot.\n");
        int NextValuablePath = FindValuablePath();
        int SatIndex = GetNowTracking();
        console.AddLog("[Auto]Start Autopilot. Next Valuable AOS : %d/%d %d:%d:%d",
                                                        State.Satellites[SatIndex]->_nextaos[NextValuablePath].AddHours(9).Month(), 
                                                        State.Satellites[SatIndex]->_nextaos[NextValuablePath].AddHours(9).Day(),
                                                        State.Satellites[SatIndex]->_nextaos[NextValuablePath].AddHours(9).Hour(),
                                                        State.Satellites[SatIndex]->_nextaos[NextValuablePath].AddHours(9).Minute(),
                                                        State.Satellites[SatIndex]->_nextaos[NextValuablePath].AddHours(9).Second());
        console.AddLog("[Auto]Max Elevation : %.2lf° Please Keep Devices On.", State.Satellites[SatIndex]->_max_elevation[NextValuablePath] * RAD_TO_DEG);
        // printf("%f\n", (State.Satellites[SatIndex]->_nextaos[NextValuablePath] -  DateTime::Now(true)).TotalMinutes());
        while(State.Autopilot)
        {
            if((State.Satellites[SatIndex]->_nextaos[NextValuablePath] -  DateTime::Now(true)).TotalMinutes() < 5.0f)
                break;
        }
        PathGenerator(GetNowTracking());

        if(!State.Autopilot)
        {
            console.AddLog("[OK]##[Auto]Autopilot Stopped by User.");
            return 0;
        }

        State.Engage = true;
        pthread_create(&p_thread[1], NULL, Sattracker, NULL);
        console.AddLog("[OK]##[Auto]Start Engaging.");

        State.Doppler = true;
        pthread_create(&p_thread[7], NULL, Doppler, NULL);
        console.AddLog("[OK]##[Auto]Start Doppler Correction.");

        //Task
        //Check Beacon Received
        while(State.Autopilot && State.DoBeaconCheck)
        {
            if(BeaconCounter >= State.iteration)
            {
                BeaconAvailable = true;
                break;
            }
                
            if(!NowOn(SatIndex, NextValuablePath))
            {
                console.AddLog("[ERROR]##[Auto]Beacon Success is too hard. Beacon Success : %d", BeaconCounter);
                break;
            }
        }

        if(!State.Autopilot)
        {
            console.AddLog("[OK]##[Auto]Autopilot Stopped by User.");
            return 0;
        }

        if(State.AMPmode)
        {
            while((State.Satellites[SatIndex]->_nextlos[NextValuablePath] - DateTime::Now(false)).TotalMinutes() > 15.0f)
                continue;
            if(State.AmpTime > 0)
            {
                console.AddLog("[ERRPR]##Cannot Turn on AMP Now. Wati until the timer goes on.");
                console.AddLog("[ERROR]##If you don't want it, restart the software.");
                console.AddLog("[ERROR]##Remind the AMP would be turned off after the timer goes on");
                console.AddLog("[ERROR]##even if you restart the software.");
                while(State.AmpTime > 0)
                    continue;
            }
                
            pid_t pid = fork();
            if (pid == -1) {
                console.AddLog("[ERROR]##Failed to fork process");
            }
            else if (pid == 0) {
                // Child process
                execl("./amp/ampcontrol", "ampcontrol", NULL);
                console.AddLog("[OK]##Forked Safe Amplifier Process.");
            }
            else
            {
                console.AddLog("[OK]##Forked Safe Amplifier Process.(ON, Timer)");
                pthread_create(&p_thread[6], NULL, AmpTimer, NULL);
            }
        }
        
        /*-----------------Calibration Code-------------------*/
        if (State.DoBaudCalibration) {
            pthread_create(&p_thread[5],NULL,csp_baud_calibration,NULL);
        }
        if (State.DoFreqCalibration) {
            pthread_create(&p_thread[5],NULL,csp_freq_calibration,NULL);
        }
        /*-----------------------------------------------------*/

        //Ping Try
        while(State.Autopilot && State.DoPing && NowOn(SatIndex, NextValuablePath))
        {
            sleep(State.SpanTime);
            
            pthread_create(&p_thread[4], NULL, csp_ping_console, NULL);
            if(PingCounter >= 1)
            {
                PingAvailable = true;
                break;
            }
                
            if(!NowOn(SatIndex, NextValuablePath))
            {
                console.AddLog("[ERROR]##[Auto]Ping Success is too hard. Ping Success : %d", PingCounter);
                break;
            }
            pthread_join(p_thread[4], NULL);
        }

        if(!State.Autopilot)
        {
            console.AddLog("[OK]##[Auto]Autopilot Stopped by User.");
            return 0;
        }

        uint32_t now_total_rx_bytes = get_rx_bytes();

        //Send Watchdog
        if(State.Autopilot && State.DoPing && NowOn(SatIndex, NextValuablePath))
        {
            console.AddLog("[Auto]Synchronizing Watchdog Counter GS100 - AX100");
            csp_transaction(CSP_PRIO_HIGH, setup->gs100_node, AX100_PORT_GNDWDT_RESET, 1000, NULL, 0, NULL, 0) > 0;
            while(State.Autopilot)
            {
                sleep(1);
                if(csp_transaction(CSP_PRIO_HIGH, setup->ax100_node, AX100_PORT_GNDWDT_RESET, 1000, NULL, 0, NULL, 0) > 0)
                console.AddLog("[OK]##[Auto]Watchdog Control OK.(No Reply)");
                else
                console.AddLog("[ERROR]##[Auto]Watchdog Control Failed.(No Reply)");
                if(now_total_rx_bytes != get_rx_bytes())
                {
                    console.AddLog("[OK]##Watchdog Reset Success. Total RX Bytes Start :%u, End : %u", now_total_rx_bytes, get_rx_bytes());
                    break;
                }
                else if (!NowOn(SatIndex, NextValuablePath))
                {
                    console.AddLog("[ERROR]##Watchdog Reset is too hard. Total RX Bytes Start :%u, End : %u", now_total_rx_bytes, get_rx_bytes());
                    break;
                }
            }

            

        }

        if(!State.Autopilot)
        {
            console.AddLog("[OK]##[Auto]Autopilot Stopped by User.");
            return 0;
        }

        bool ulsigdone = false;
        //UL SIGNAL
        if((State.DoFTPDL || State.DoFTPUL) && NowOn(SatIndex, NextValuablePath))
        {
            int NowBecCount = BeaconCounter;
            while(NowBecCount == BeaconCounter && State.Autopilot && NowOn(SatIndex, NextValuablePath))
                continue;
            uint32_t initial = get_rx_bytes();
            DateTime inittime = DateTime::Now(false);
            
            while(State.Autopilot && !ulsigdone)
            {
                if((inittime - DateTime::Now()).TotalSeconds() > 30)
                    console.AddLog("[Auto]No Beacon for 30 seconds. Try Next Task.");
                console.AddLog("[Auto]Try to Send UL Signal");
                
                DateTime BeaconStandard = DateTime::Now(false);
                for(int i = 0; i < 10; i++)
                {
                    sleep(State.SpanTime);
                    if((DateTime::Now(false) - State.Satellites[SatIndex]->_nextlos[NextValuablePath]).TotalSeconds() > 0)
                    {
                        // console.AddLog("[ERROR]##[Auto]Failed to Send UL Signal.");
                        break;
                    }
                    
                    // MIM_TLEinfo = FSWTleConverter(TrackingTLE[NowSat]);
                    packetsign* TestPacket = (packetsign *)malloc(9);
                    TestPacket->Identifier = MIM_ID;
                    TestPacket->PacketType = MIM_PT_SIGNAL;
                    TestPacket->Length = 1;
                    memset(TestPacket->Data, 0, TestPacket->Length);
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    pthread_join(p_thread[4], NULL);
                }

                console.AddLog("Try to config total rx bytes.");
                int trying = 0;
                while(State.Autopilot && NowOn(SatIndex, NextValuablePath))
                {
                    if(gs_rparam_get_uint32(setup->ax100_node, 4, 0x003C, 0xB00B, setup->default_timeout, get_rx_bytes_address()) == GS_OK)
                    {
                        if((get_rx_bytes() - initial) % 16 != 0)
                        {
                            console.AddLog("[OK]##[Auto]UL Signal Send Done. Init : %u, Now : %u", initial, get_rx_bytes());
                            ulsigdone = true;
                            break;
                        }
                    }
                    else if (trying == 5)
                        break;
                    else
                    {
                        trying ++;
                        continue;
                    }
                        

                }
                
                if(!NowOn(SatIndex, NextValuablePath))
                {
                    console.AddLog("[ERROR]##[Auto]Failed to Send UL Signal.");
                    break;
                }
            }
        }

        

        if(!State.Autopilot)
        {
            console.AddLog("[OK]##[Auto]Autopilot Stopped by User.");
            return 0;
        }

        //CMD
        if(State.Autopilot && State.DoCMD && NowOn(SatIndex, NextValuablePath))
        {
            for(int i = 0; i < 256; i++)
            {
                if(SatCMD[i] == NULL)
                    continue;
                if(State.Successed > State.CMDCount)
                {
                    State.CMDCount++;
                    continue;
                }
                while(State.Autopilot)
                {
                    sleep(State.SpanTime);
                    
                    if(SatCMD[i]->Checksum)
                        SatCMD[i]->GenerateChecksum();
                    packetsign * SendingCMD = SatCMD[i]->GenerateCMDPacket();
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)SendingCMD);
                    pthread_join(p_thread[4], NULL);
                    if(State.Successed < State.CMDCount)
                    {
                        State.Successed = State.CMDCount;
                        break;
                    }
                    if(!NowOn(SatIndex, NextValuablePath))
                    {
                        break;
                    }
                }
                if(!NowOn(SatIndex, NextValuablePath))
                {
                    break;
                }

            }
            if(!NowOn(SatIndex, NextValuablePath))
            {
                console.AddLog("[ERROR]##[Auto]Stop CMD after LOS. Success : %d", State.CMDCount);
                State.CMDCount = 0;
                break;
            }
        }

        if(!State.Autopilot)
        {
            console.AddLog("[OK]##[Auto]Autopilot Stopped by User.");
            return 0;
        }

        //FTP DL
        //Temporary condition : max_el > 30.0deg
        while(State.Autopilot && State.DoFTPDL && NowOn(SatIndex, NextValuablePath))
        {
            if(State.ftp_mode)
                continue;
            sleep(State.SpanTime);
            pthread_join(p_thread[8], NULL);
            if(!NowOn(SatIndex, NextValuablePath))
            {
                console.AddLog("[Auto]Stop FTP Download after LOS.");
                break;
            }
            if(State.ftp_version == 1)
                pthread_create(&p_thread[8], NULL, ftp_downlink_force, &State.ftplistup[NowFTP]);
            else
                pthread_create(&p_thread[8], NULL, ftp_downlink_onorbit, &State.ftplistup[NowFTP]);
            
        }

        if(!State.Autopilot)
        {
            console.AddLog("[OK]##[Auto]Autopilot Stopped by User.");
            return 0;
        }

        //FTP UL
        //Temporary condition : max_el > 30.0deg
        while(State.Autopilot && State.DoFTPUL && NowOn(SatIndex, NextValuablePath))
        {
            if(State.ftp_mode)
                continue;
            sleep(State.SpanTime);
            pthread_join(p_thread[8], NULL);
            if(!NowOn(SatIndex, NextValuablePath))
            {
                console.AddLog("[Auto]Stop FTP Upload after LOS.");
                break;
            }
            if(State.ftp_version == 1)
                pthread_create(&p_thread[8], NULL, ftp_uplink_force, &State.ftplistup[NowFTP]);
            else
                pthread_create(&p_thread[8], NULL, ftp_uplink_onorbit, &State.ftplistup[NowFTP]);
            
            
        }
        

        if(!State.Autopilot)
        {
            console.AddLog("[OK]##[Auto]Autopilot Stopped by User.");
            return 0;
        }

        //Wait Until LOS and return multhreads.
        pthread_join(p_thread[6], NULL);
        pthread_join(p_thread[8], NULL);
        pthread_join(p_thread[4], NULL);

        while(State.Autopilot && NowOn(SatIndex, NextValuablePath))
            continue;
        State.Doppler = false;
        pthread_join(p_thread[7], NULL);
        console.AddLog("[OK]##[Auto]Stop Doppler Correction.");

        State.Engage = false;
        State.Doppler = false;
        pthread_join(p_thread[1], NULL);
        console.AddLog("[OK]##[Auto]Finish Engaging.");
        
        RotatorSetTo(0, 0);
        console.AddLog("[Auto]Beacon Success : %d, Ping Success : %d CMD Success : %d", BeaconCounter, PingCounter, State.CMDCount);
        BeaconCounter = 0;
        PingCounter = 0;

        console.AddLog("[Auto]Update TLE...");
        for(int i = 0; i < 8192; i++)
        {
            State.Satellites[i]->use = false;
        }
        if(DownloadTLE(State.tleinfolistup[NowTLE]->remote, State.tleinfolistup[NowTLE]->local))
        {
            State.NowTracking = false;
            
            console.AddLog("[OK]##[Auto]TLE \"%s\" Download Completed.", State.tleinfolistup[NowTLE]->label);

            // pthread_create(&p_thread[15], NULL, &SatelliteInitialize,  &sgp4check);
            // SatelliteInitialize(NULL);
            
        }
        else
        {
            console.AddLog("[ERROR]##[Auto]TLE Download failed. Please check again.");
            return -1;
        }
        // pthread_join(p_thread[15], NULL);
        printf("Updated TLE.\n");
        sleep(3);

        //AMP OFF
        pid_t pid_off = fork();

        if (pid_off == -1) {
            console.AddLog("[ERROR]##Failed to fork process");
        }
        else if (pid_off == 0) {
            // Child process
            execl("./amp/ampoff", "ampoff", NULL);
        }
        else
        {
            console.AddLog("[OK]##Forked Safe Amplifier Process. (OFF)");
        }
        State.AMPON = false;

        csp_transaction(CSP_PRIO_HIGH, setup->gs100_node, AX100_PORT_GNDWDT_RESET, 1000, NULL, 0, NULL, 0) > 0;
        console.AddLog("Reset GS100 watchdog.");
        printf("Reset GS100 Watchdog.\n");
        sleep(3);
        int tlepopupindex = 0;
        
        printf("Search Samename Satellite.\n");

        State.tleallindex = 8192;
        if((State.tleallindex = ReadTLELines_Errorhandling(State.tleinfolistup[NowTLE]->local, NowTrackingNameBuffer,false, false)) <= 0)
            console.AddLog("[ERROR]##Invalid type of TLE File. Please check again.");
        State.Display_TLE = true;
       
        sleep(3);
        State.Satellites[NowTracking]->use = true;
        if(State.Satellites[NowTracking]->cal == false)
            State.Satellites[NowTracking]->Refresh(State.Satellites[NowTracking]->tle, State.Satellites[NowTracking]->obs, true, true);
        printf("Updated Propagator.\n");
        sleep(3);
        State.NowTracking = true;
        State.ModelRefreshRequired = true;
        printf("Model Refresh.\n");
        sleep(3);
        SetNowTracking(NowTracking);
        PathGenerator(GetNowTracking());
        State.Display_load = false;
        State.Display_TLE = false;
        printf("Finish Autopilot.\n");
    }
    return 0;
}

int TLE_Autoupdate_Test()
{
    int NowTracking = GetNowTracking();
    char NowTrackingNameBuffer[32];
    printf("[Auto]Update TLE...\n");
    sprintf(NowTrackingNameBuffer, State.Satellites[NowTracking]->Name());
    for(int i = 0; i < 8192; i++)
    {
        State.Satellites[i]->use = false;
    }
    if(DownloadTLE(State.tleinfolistup[NowTLE]->remote, State.tleinfolistup[NowTLE]->local))
    {
        State.NowTracking = false;
        
        printf("[OK]##[Auto]TLE \"%s\" Download Completed.\n", State.tleinfolistup[NowTLE]->label);

        // pthread_create(&p_thread[15], NULL, &SatelliteInitialize,  &sgp4check);
        // SatelliteInitialize(NULL);
        
    }
    else
    {
        printf("[ERROR]##[Auto]TLE Download failed. Please check again.\n");
        return -1;
    }
    // pthread_join(p_thread[15], NULL);
    printf("Updated TLE.\n");
    sleep(3);

    printf("Skip GS100 Watchdog");
    // csp_transaction(CSP_PRIO_HIGH, setup->gs100_node, AX100_PORT_GNDWDT_RESET, 1000, NULL, 0, NULL, 0) > 0;
    // console.AddLog("Reset GS100 watchdog.");
    // printf("Reset GS100 Watchdog.\n");
    // sleep(3);
    int tlepopupindex = 0;
    
    printf("Search Samename Satellite.\n");

    State.tleallindex = 32768;
    if((State.tleallindex = ReadTLELines_Errorhandling(State.tleinfolistup[NowTLE]->local, NowTrackingNameBuffer,false, false)) <= 0)
        printf("[ERROR]##Invalid type of TLE File. Please check again.\n");
    State.Display_TLE = true;
    
    sleep(3);
    State.Satellites[NowTracking]->use = true;
    if(State.Satellites[NowTracking]->cal == false)
        State.Satellites[NowTracking]->Refresh(State.Satellites[NowTracking]->tle, State.Satellites[NowTracking]->obs, true, true);
    printf("Updated Propagator.\n");
    sleep(3);
    State.NowTracking = true;
    State.ModelRefreshRequired = true;
    printf("Model Refresh.\n");
    sleep(3);
    SetNowTracking(NowTracking);
    PathGenerator(GetNowTracking());
    State.Display_load = false;
    State.Display_TLE = false;
    printf("Finish Autopilot.\n");
}