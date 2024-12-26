#include <pthread.h>
#include <inttypes.h>
#include <GL/glew.h> 
#include <GLFW/glfw3.h>

#include <gs/param/rparam.h>
#include <csp/switch.h>
#include <csp/delay.h>

#include "miman_config.h"
#include "miman_imgui.h"
#include "miman_csp.h"
#include "miman_radial.h"
#include "miman_coms.h"
#include "miman_orbital.h"
#include "miman_model.h"
#include "miman_s_control.h"
#include "miman_ftp.h"
#include "miman_autopilot.h"

Console console;
bool p_open;
ImGuiWindowFlags mim_winflags = ImGuiWindowFlags_NoMove |
                                ImGuiWindowFlags_NoResize |
                                ImGuiWindowFlags_NoSavedSettings |
                                ImGuiWindowFlags_NoCollapse |
                                ImGuiWindowFlags_NoBringToFrontOnFocus|
                                ImGuiWindowFlags_HorizontalScrollbar|
                                ImGuiWindowFlags_NoTitleBar;
extern pthread_t p_thread[16];
extern Ptable_0 param0;
extern Ptable_1 param1;
extern Ptable_5 param5;
extern StateCheckUnit State;
extern Command * command;
extern Beacon * beacon;
extern int BeaconCounter;
extern int PingCounter;
char callsignbuf[10];

int temptarget = 1;
int tempstatus = 1;
int tempfilenum = 1;
int tempoffset = 0;
int tempstep = 0;

extern dlreqdata * request;
static int freeze_cols = 1;
static int freeze_rows = 1;

bool booking;
uint32_t gen_msgid = 0;
uint16_t gen_fnccode = 0;

extern CmdGenerator_GS * SatCMD[256];
bool SchedulerState = false;
bool ChecksumState = true;
uint32_t ExecutionTimeBuf = 0;
uint32_t ExecutionWindowBuf = 0;
uint16_t EntryIDBuf = 0;
uint16_t GroupIDBuf = 0;
int NowCMD = 0;

extern struct Antenna Now;
int NowTLE = 0;
int Requested = -2;
extern int NowFTP;
extern Setup * setup;

Ptable_0 rparam0;
Ptable_1 rparam1;
Ptable_5 rparam5;
uint8_t remote_activeconf = 99;
int16_t remote_lastrssi = 999;
int16_t remote_rssibusy = 0;

int Satellite_row;
char TrackingButtonTextBuffer[32];
char InfoButtonTextBuffer[32];

char Sendinglabelbuf[16];
char Deletelabelbuf[16];
uint8_t msgidbuf[2];
uint16_t msgidinfo;

char SelectButtonTextBuf[32];
char DeleteButtonTextBuf[32];
char SearchBuf[256];
int tlepopupindex;
bool sgp4check = false;
char FDSFilePath[256] = "./bin/fds/fds.dat";
char SatWindowLabelBuf[64];
char inputbuf[1024];
extern Sband * sgs;

char ComboboxLabel[32];
char ADCSCMDLabels[128][64];
char CICMDLabels[128][64];
char EPSCMDLabels[128][64];
char FMCMDLabels[128][64];
char GPSCMDLabels[128][64];
char MTQCMDLabels[128][64];
char PAYCMDLabels[128][64];
char RWACMDLabels[128][64];
char SNSRCMDLabels[128][64];
char STXCMDLabels[128][64];
char TOCMDLabels[128][64];
char UTRXCMDLabels[128][64];
char TSCMDLabels[128][64];
char ESCMDLabels[128][64];
char SCHCMDLabels[128][64];
char ECMCMDLabels[128][64];

char AmpTimeBuf[32];

bool show_typingWindow = false;
typedef struct CMDInfo {
    uint16_t msgid;
    uint16_t fnccode;
}__attribute__((packed));

//CMDInfo *info = new CMDInfo();
uint16_t msgid;
uint16_t fnccode;

cmd_packet_t * shellcmd = new cmd_packet_t;
char repeated_label[64];

void ImGui_ClearColorBuf(GLFWwindow * window, ImVec4 clear_color)
{
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT);
}

void ImGui_MainMenu()
{
    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if(ImGui::MenuItem("FDS Manager"))
            {
                State.Display_FDS = true;
            }
            if(ImGui::MenuItem("TLE Manager"))
            {
                State.Display_TLE = true;
            }
            if(ImGui::MenuItem("S-band Manager"))
            {
                State.Display_Sband = true;
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Hardware"))
        {
            if(ImGui::MenuItem("GS100 Param 0"))
            {
                State.Display_paramt0 = true;
            }
            if(ImGui::MenuItem("GS100 Param 1"))
            {
                State.Display_paramt1 = true;
            }
            if(ImGui::MenuItem("GS100 Param 5"))
            {
                State.Display_paramt5 = true;
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Test"))
        {
            if (ImGui::MenuItem("Scan AX100"))
            {
                
                pthread_create(&p_thread[4], NULL, csp_ping_scan, NULL); 
            }
            if (ImGui::MenuItem("Start Signaltest"))
            {
                pthread_create(&p_thread[10], NULL, &SignalTest, NULL); 
            }
            if (ImGui::MenuItem("Stop Signaltest"))
            {
                State.Signaltest = false;
                pthread_join(p_thread[10], NULL); 
            }
            if(State.Debugmode)
            {
                if (ImGui::MenuItem("Debug Mode Off"))
                {
                    csp_log_info("Set Debug : False.");
                    State.Debugmode = false;
                    csp_debug_set_level(CSP_INFO, false);
                    // csp_debug_toggle_level(CSP_BUFFER);
                    csp_debug_set_level(CSP_PACKET, false);
                    csp_debug_set_level(CSP_PROTOCOL, false);
                    // csp_debug_toggle_level(CSP_LOCK);
                    
                }
                    
            }
            else
            {
                if (ImGui::MenuItem("Debug Mode On"))
                {
                    State.Debugmode = true;
                    csp_debug_set_level(CSP_INFO, true);
                    // csp_debug_toggle_level(CSP_BUFFER);
                    csp_debug_set_level(CSP_PACKET, true);
                    csp_debug_set_level(CSP_PROTOCOL, true);
                    // csp_debug_toggle_level(CSP_LOCK);
                    csp_log_info("Set Debug : True.");
                }
                    
            }
            if (ImGui::MenuItem("TLE Update(Auto)"))
            {
                TLE_Autoupdate_Test();
            }
            if (ImGui::MenuItem("Segfault"))
            {
                int* ptr = nullptr;
                *ptr = 42;
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}

void ImGui_ModelWindow(float fontscale)
{
    
    ImGui::Begin("Orbital Model", &p_open, mim_winflags);
    ImGui::SetWindowFontScale(fontscale);
    
    ImGui::SetWindowFontScale(1.0);
    ImGui::End();
}

void ImGui_TrackWindow(float fontscale)
{
    ImGui::Begin("Track Window", &p_open, mim_winflags);
    ImGui::SetWindowFontScale(fontscale * 0.8);
    
    ImGui::SameLine();
    Satellite_row = 0;
    
    if (ImGui::BeginTable("##SatelliteListup", 8, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable, ImVec2(ImGui::GetContentRegionAvail().x * 0.7 , ImGui::GetContentRegionAvail().y)))
    {
        ImGui::TableSetupScrollFreeze(freeze_cols, freeze_rows);
        ImGui::TableSetupColumn("Name",           ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("Next AOS",       ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("Next LOS",       ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("Max El",  ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("Control",  ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("Az",  ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("El",  ImGuiTableColumnFlags_NoHide, 0.0f);
        // ImGui::TableSetupColumn("Latitude",  ImGuiTableColumnFlags_NoHide, 0.0f);
        // ImGui::TableSetupColumn("Longitude",  ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("Altitude",  ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableHeadersRow();
        
        // ImGui::TableNextColumn();
        if(strlen(State.Fatellites->Name()) != 0)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.950f, 0.266f, 0.322f, 1.0f));
            ImGui::TableNextRow();
            
            ImGui::TableNextColumn();
            ImGui::Text("%s", State.Fatellites->Name());
            ImGui::TableNextColumn();
            ImGui::Text("%d/%d %d:%d:%d",   State.Fatellites->_nextaos[0].AddHours(9).Month(), 
                                            State.Fatellites->_nextaos[0].AddHours(9).Day(),
                                            State.Fatellites->_nextaos[0].AddHours(9).Hour(),
                                            State.Fatellites->_nextaos[0].AddHours(9).Minute(),
                                            State.Fatellites->_nextaos[0].AddHours(9).Second());
            ImGui::TableNextColumn();
            ImGui::Text("%d/%d %d:%d:%d",   State.Fatellites->_nextlos[0].AddHours(9).Month(), 
                                            State.Fatellites->_nextlos[0].AddHours(9).Day(),
                                            State.Fatellites->_nextlos[0].AddHours(9).Hour(),
                                            State.Fatellites->_nextlos[0].AddHours(9).Minute(),
                                            State.Fatellites->_nextlos[0].AddHours(9).Second());
            ImGui::TableNextColumn();
            ImGui::Text("%.2f", State.Fatellites->_max_elevation[0] * RAD_TO_DEG);
            ImGui::TableNextColumn();
            
            sprintf(InfoButtonTextBuffer, "Info##trackingwindow%d", Satellite_row);
            if(ImGui::Button(InfoButtonTextBuffer, ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
            {
                Requested = -1;
                State.Display_Satinfo = true;
            }
            ImGui::SameLine();
            if(GetNowTracking() == -1)
            {
                sprintf(TrackingButtonTextBuffer, "  On");
                ImGui::Text(TrackingButtonTextBuffer);
            }
            else
            {
                sprintf(TrackingButtonTextBuffer, "Track##trackingwindow%d", Satellite_row);
                if(ImGui::Button(TrackingButtonTextBuffer, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
                {
                    SetNowTracking(-1);
                    PathGenerator(GetNowTracking());
                }
            }
            
            ImGui::TableNextColumn();
            ImGui::Text("%.3lf°", State.Fatellites->topo.azimuth * RAD_TO_DEG);
            ImGui::TableNextColumn();
            ImGui::Text("%.3lf°", State.Fatellites->topo.elevation * RAD_TO_DEG);
            // ImGui::TableNextColumn();
            // ImGui::Text("%.2lfkm", State.Fatellites->topo.range);
            // ImGui::TableNextColumn();
            // ImGui::Text("%.2lfkm/s", State.Fatellites->topo.range_rate);
            // ImGui::TableNextColumn();
            // ImGui::Text("%.3lf°", State.Fatellites->geo.latitude * RAD_TO_DEG);
            // ImGui::TableNextColumn();
            // ImGui::Text("%.3lf°", State.Fatellites->geo.longitude * RAD_TO_DEG);
            ImGui::TableNextColumn();
            ImGui::Text("%.2lfkm", State.Fatellites->geo.altitude);
            ImGui::PopStyleColor();
            
        }
        for(Satellite_row = 0; Satellite_row < sizeof(State.Satellites) / sizeof(SatelliteObject *); Satellite_row++)
        {
            
            if(strlen(State.Satellites[Satellite_row]->Name()) == 0)
                break;
            else if (!State.Satellites[Satellite_row]->use)
            {
                continue;
            }
            else
            {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                ImGui::Text("%s", State.Satellites[Satellite_row]->Name());
                ImGui::TableNextColumn();
                ImGui::Text("%d/%d %d:%d:%d",   State.Satellites[Satellite_row]->_nextaos[0].AddHours(9).Month(), 
                                                State.Satellites[Satellite_row]->_nextaos[0].AddHours(9).Day(),
                                                State.Satellites[Satellite_row]->_nextaos[0].AddHours(9).Hour(),
                                                State.Satellites[Satellite_row]->_nextaos[0].AddHours(9).Minute(),
                                                State.Satellites[Satellite_row]->_nextaos[0].AddHours(9).Second());
                ImGui::TableNextColumn();
                ImGui::Text("%d/%d %d:%d:%d",   State.Satellites[Satellite_row]->_nextlos[0].AddHours(9).Month(), 
                                                State.Satellites[Satellite_row]->_nextlos[0].AddHours(9).Day(),
                                                State.Satellites[Satellite_row]->_nextlos[0].AddHours(9).Hour(),
                                                State.Satellites[Satellite_row]->_nextlos[0].AddHours(9).Minute(),
                                                State.Satellites[Satellite_row]->_nextlos[0].AddHours(9).Second());
                ImGui::TableNextColumn();
                ImGui::Text("%.2f", State.Satellites[Satellite_row]->_max_elevation[0] * RAD_TO_DEG);
                ImGui::TableNextColumn();
                
                sprintf(InfoButtonTextBuffer, "Info##trackingwindow%d", Satellite_row);
                if(ImGui::Button(InfoButtonTextBuffer, ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
                {
                    Requested = Satellite_row;
                    State.Display_Satinfo = true;
                }
                ImGui::SameLine();
                if(GetNowTracking() == Satellite_row)
                {
                    sprintf(TrackingButtonTextBuffer, "  On");
                    ImGui::Text(TrackingButtonTextBuffer);
                }
                else
                {
                    sprintf(TrackingButtonTextBuffer, "Track##trackingwindow%d", Satellite_row);
                    if(ImGui::Button(TrackingButtonTextBuffer, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
                    {
                        SetNowTracking(Satellite_row);
                        PathGenerator(GetNowTracking());
                    }
                }
                ImGui::TableNextColumn();
                ImGui::Text("%.3lf°", State.Satellites[Satellite_row]->topo.azimuth * RAD_TO_DEG);
                ImGui::TableNextColumn();
                ImGui::Text("%.3lf°", State.Satellites[Satellite_row]->topo.elevation * RAD_TO_DEG);
                // ImGui::TableNextColumn();
                // ImGui::Text("%.2lfkm", State.Satellites[Satellite_row]->topo.range);
                // ImGui::TableNextColumn();
                // ImGui::Text("%.2lfkm/s", State.Satellites[Satellite_row]->topo.range_rate);
                // ImGui::TableNextColumn();
                // ImGui::Text("%.3lf°", State.Satellites[Satellite_row]->geo.latitude * RAD_TO_DEG);
                // ImGui::TableNextColumn();
                // ImGui::Text("%.3lf°", State.Satellites[Satellite_row]->geo.longitude * RAD_TO_DEG);
                ImGui::TableNextColumn();
                ImGui::Text("%.2lfkm", State.Satellites[Satellite_row]->geo.altitude);
            }
        }
        
        ImGui::EndTable();
    }
    ImGui::SameLine();
    ImGui::SetWindowFontScale(fontscale);
    ImGui::BeginChild("##AutoPilot", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y), true, mim_winflags);
    ImGui::Text("AutoPilot");
    ImGui::Checkbox("Beacon##AutoPilot", &State.DoBeaconCheck);
    ImGui::SameLine();
    ImGui::Checkbox("Ping  ##AutoPilot", &State.DoPing);
    ImGui::SameLine();
    ImGui::Checkbox("CMD##AutoPilot", &State.DoCMD);
    ImGui::Checkbox("FTP DL##AutoPilot", &State.DoFTPDL);
    ImGui::SameLine();
    ImGui::Checkbox("FTP UL##AutoPilot", &State.DoFTPUL);
    ImGui::SameLine();
    ImGui::Checkbox("AMP##amponftp", &State.AMPmode);
    //Add for calibration
    ImGui::Checkbox("Baud Calibration##AutoPilot",&State.DoBaudCalibration);
    ImGui::SameLine();
    ImGui::Checkbox("Freq Calibration##AutoPilot",&State.DoFreqCalibration);
    //--------------------------------------
    ImGui::Text("iteration");
    ImGui::SameLine();
    ImGui::InputInt("##AutoPilotIter", &State.iteration);
    ImGui::Text("Time Span");
    ImGui::SameLine();
    ImGui::InputInt("##AutoPilotSpan", &State.SpanTime);
    if(!State.Autopilot)
    {
        if(ImGui::Button("Start AutoPilot", ImVec2(ImGui::GetContentRegionAvail().x , ImGui::GetContentRegionAvail().y)))
        {
            State.Autopilot = true;
            pthread_create(&p_thread[13], NULL, AutoPilot, NULL);
        }
    }
    else
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.650f, 0.266f, 0.322f, 1.0f));
        if(ImGui::Button("Stop AutoPilot", ImVec2(ImGui::GetContentRegionAvail().x , ImGui::GetContentRegionAvail().y)))
        {
            State.Autopilot = false;
            pthread_join(p_thread[13], NULL);
        }
        ImGui::PopStyleColor();
    }
    
    ImGui::EndChild();
    ImGui::SetWindowFontScale(1.0);
    ImGui::End();
   
}

void ImGui_FrequencyWindow(float fontscale)
{   
    
    ImGui::Begin("Frequency Window", &p_open, mim_winflags);
    ImGui::SetWindowFontScale(fontscale);
    ImGui::Text("Center Frequency");
    ImGui::InputDouble("##centerfrequency(Hz)", &setup->default_freq, NULL, NULL);
    ImGui::SameLine();
    if(ImGui::Button("Set##SetDefaultFrequency", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
    {
        SetRxFreq(setup->default_freq);
        SetTxFreq(setup->default_freq);
        SetRxBaud(4800);
        SetTxBaud(4800);
        UpdateFreq();
        UpdateBaud();
    }
    if (ImGui::BeginTable("##TRXspectable", 2, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 3.8)))
    {
        ImGui::TableSetupScrollFreeze(freeze_cols, freeze_rows);
        ImGui::TableSetupColumn("RX",       ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("TX",       ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableHeadersRow();
        ImGui::TableNextColumn();
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%f MHz", ((float)param1.freq) / 1000000);
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%f MHz", ((float)param5.freq) / 1000000);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("%"PRIu32" bps", param1.baud);
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%"PRIu32" bps", param5.baud);
        ImGui::EndTable();
    }
    ImGui::Text("Last RSSI(dBm) : ");
    ImGui::SameLine();
    ImGui::Text("%"PRId16"", Return_RSSI());
    if (ImGui::Button("Set to RX", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
    {
        switch_to_rx(setup->ax100_node);
    }
    ImGui::SameLine();
    if (ImGui::Button("Set to TX", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
    {
        switch_to_tx(setup->ax100_node);
    }

    if(State.Doppler == false)
    {

        if (ImGui::Button("Start Doppler Correction", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(State.GS100_Connection)
            {
                State.Doppler = true;
                pthread_create(&p_thread[7], NULL, Doppler, NULL);
                console.AddLog("[OK]##Start Doppler Correction.");
            }
            else
            {
                console.AddLog("[ERROR]##GS100 Connection Error - Cannot Start Doppler Correction.");
            }
            
        }
    }
    else
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.650f, 0.266f, 0.322f, 1.0f));
        if (ImGui::Button("Stop Doppler Correction", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            State.Doppler = false;
            pthread_join(p_thread[7], NULL);
        }
        ImGui::PopStyleColor();
    }
    ImGui::Text("Ping Size  ");
    ImGui::SameLine();
    ImGui::InputScalar("##pingsize", ImGuiDataType_U16, &setup->pingsize, NULL, NULL, "%u");
    ImGui::BeginChild("##DelaySettings", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y), true, mim_winflags);
    ImGui::Text("Default Delay");
    ImGui::SameLine();
    ImGui::InputScalar("##defaulttimeout", ImGuiDataType_U32, &setup->default_timeout, NULL, NULL, "%u");
    ImGui::Text("Guard Delay  ");
    ImGui::SameLine();
    ImGui::InputScalar("##guarddelay", ImGuiDataType_U32, &setup->guard_delay, NULL, NULL, "%u");
    ImGui::Text("Queue Delay  ");
    ImGui::SameLine();
    ImGui::InputScalar("##queuedelay", ImGuiDataType_U32, &setup->queue_delay, NULL, NULL, "%u");
    if(ImGui::Button("Apply##apply", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
    {
        set_serial_spec_micsec(setup->Transceiver_baud, setup->Switch_baud, 1000000, setup->queue_delay, setup->gs100_node, setup->default_timeout, setup->guard_delay);
        console.AddLog("Set Delay Spec : Default : %u, Guard : %u, Queue : %u", setup->default_timeout, setup->guard_delay, setup->queue_delay);
    }
    ImGui::EndChild();

    ImGui::SetWindowFontScale(1.0);
    ImGui::End();
}


void ImGui_RotatorWindow(float fontscale)
{
    ImGui::Begin("Rotator Window", &p_open, mim_winflags);
    ImGui::SetWindowFontScale(fontscale);
    if(State.Engage == false)
    {
        if (ImGui::Button("Start Rotator Engaging", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            State.Engage = true;
            pthread_create(&p_thread[1], NULL, Sattracker, NULL);
            console.AddLog("[OK]##Start Engaging.");
        }
    }
    else
    {
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.650f, 0.266f, 0.322f, 1.0f));
        if (ImGui::Button("Stop Rotator Engaging", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            State.Engage = false;
            State.Doppler = false;
            pthread_join(p_thread[1], NULL);
            console.AddLog("[OK]##Finish Engaging.");
            console.AddLog("Beacon Success : %d, Ping Success : %d", BeaconCounter, PingCounter);
            BeaconCounter = 0;
            PingCounter = 0;
        }
        ImGui::PopStyleColor();
    }
    
    
    ImGui::Text("Rotator Manual Control");
    ImGui::Text("AZ : ");
    ImGui::SameLine();
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
    ImGui::InputInt("##tg_az", &Now.az_tag);
    ImGui::PopItemWidth();
    ImGui::Text("EL : ");
    ImGui::SameLine();
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
    ImGui::InputInt("##tg_el", &Now.el_tag);
    ImGui::PopItemWidth();
    if (ImGui::Button("Set", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
    {
        RotatorSetTo(Now.az_tag, Now.el_tag);
        //memset(&ControlBuf,0,sizeof(ControlBuf));
    }
    ImGui::SameLine();
    if (ImGui::Button("Init", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
    {
        RotatorSetTo(0, 0);
    }
    if (ImGui::BeginTable("##Rotspectable", 3, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 3.8)))
    {
        ImGui::TableSetupScrollFreeze(freeze_cols, freeze_rows);
        ImGui::TableSetupColumn("Band",         ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("Azimuth",      ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("Elevation",    ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableHeadersRow();
        ImGui::TableNextColumn();
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("UHF");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("%d °", Now.az_now);
        ImGui::TableSetColumnIndex(2);
        ImGui::Text("%d °", Now.el_now);
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("S-band");
        ImGui::TableSetColumnIndex(1);
        // ImGui::Text("%d °", Now.az_now);
        ImGui::TableSetColumnIndex(2);
        // ImGui::Text("%d °", Now.el_now);
        ImGui::EndTable();
    }


    if(ImGui::Button("Rotator On", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
    {
        init_rotator();
        pthread_create(&p_thread[2], NULL, RotatorReadInfo, NULL);
    }
    ImGui::SameLine();
    if(ImGui::Button("Rotator OFF", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
    {
        State.RotatorReading = false;
        pthread_join(p_thread[2], NULL);
        // fin_rotator();
    }
    if(ImGui::Button("Rotator IO Buffer Free", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
    {
        RotatorEnter(NULL);
    }
    ImGui::SetWindowFontScale(1.0f);
    ImGui::End();

}

void StateWindowColumnManager(const char * info)
{
    ImGui::TableNextColumn();
    ImGui::Text(info);
    ImGui::TableNextColumn();
}


void ImGui_BeaconWindow(float fontscale)
{
            
    ImGui::Begin("Beacon GUI", &p_open, mim_winflags);
    ImGui::SetWindowFontScale(fontscale);

    ImGui::BeginTabBar("##InfoDesk");
    if(ImGui::BeginTabItem("Beacon"))
    {
        if (ImGui::BeginTable("##BeaconTables", 2, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable))
        {
            ImGui::TableSetupScrollFreeze(freeze_cols, freeze_rows);
            ImGui::TableSetupColumn("Parameter",           ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
            ImGui::TableSetupColumn("Data",       ImGuiTableColumnFlags_NoHide, 0.0f);
            ImGui::TableHeadersRow();
            ImGui::TableNextColumn();
            ImGui::Text("Call Sign");
            ImGui::TableNextColumn();
            memcpy(callsignbuf, beacon->CallSign, sizeof(beacon->CallSign));
            callsignbuf[6] = 0;
            // sprintf(callsignbuf, "%s\r\r\r\r", beacon->CallSign);
            ImGui::Text(callsignbuf);
            StateWindowColumnManager("Message ID");
            ImGui::Text("0x%x", csp_ntoh16(beacon->CCMessage_ID));
            StateWindowColumnManager("Sequence");
            ImGui::Text("0x%x", csp_ntoh16(beacon->CCSequence));
            StateWindowColumnManager("Length");
            ImGui::Text("%"PRIu16, csp_ntoh16(beacon->CCLength) + 7);
            StateWindowColumnManager("Time Code");
            ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", beacon->CCTime_code[0], beacon->CCTime_code[1], beacon->CCTime_code[2], beacon->CCTime_code[3], beacon->CCTime_code[4], beacon->CCTime_code[5]);
            StateWindowColumnManager("Spacecraft Time");
            ImGui::Text("%"PRIu32, beacon->FMSpaceTime);
            StateWindowColumnManager("Reboot Count");
            ImGui::Text("%"PRIu16, beacon->FMRebootCount);
            StateWindowColumnManager("Current Mode");
            ImGui::Text("%"PRIu8, beacon->FMCurMode);
            StateWindowColumnManager("Current Submode");
            ImGui::Text("%"PRIu8, beacon->FmCurSubMode);
            StateWindowColumnManager("Previous Mode");
            ImGui::Text("%"PRIu8, beacon->FMPreviousMode);
            StateWindowColumnManager("Previous SubMode");
            ImGui::Text("%"PRIu8, beacon->FMPreviousSubMode);
            StateWindowColumnManager("Mode");
            ImGui::Text("%"PRIu8, beacon->SYMode);
            StateWindowColumnManager("Reset Cause");
            ImGui::Text("%"PRIu8, beacon->SYResetCause);
            StateWindowColumnManager("Reset Count");
            ImGui::Text("%"PRIu16, beacon->SYResetcount);
            StateWindowColumnManager("Battery Status");
            ImGui::Text("0x%x", beacon->BAtteryStatus);
            StateWindowColumnManager("Battery In Voltage");
            ImGui::Text("%"PRId16, beacon->BAtteryInV);
            StateWindowColumnManager("Battery In Current");
            ImGui::Text("%"PRId16, beacon->BAtteryInI);
            StateWindowColumnManager("Battery Temperature");
            ImGui::Text("%"PRId16, beacon->BAtteryTemp);
            StateWindowColumnManager("Mboard MPPT In V");
            ImGui::Text("%"PRId16" %"PRId16" %"PRId16, beacon->MOMPPTInV[0], beacon->MOMPPTInV[1], beacon->MOMPPTInV[2]);
            StateWindowColumnManager("Mboard MPPT In I");
            ImGui::Text("%"PRId16" %"PRId16" %"PRId16, beacon->MOMPPTInI[0], beacon->MOMPPTInI[1], beacon->MOMPPTInI[2]);
            StateWindowColumnManager("Mboard Channel I 3V");
            ImGui::Text("%"PRId16" %"PRId16" %"PRId16" %"PRId16, beacon->MOChancur3V[0], beacon->MOChancur3V[1], beacon->MOChancur3V[2], beacon->MOChancur3V[3]);
            StateWindowColumnManager("Mboard Channel I 5V");
            ImGui::Text("%"PRId16" %"PRId16" %"PRId16" %"PRId16, beacon->MOChancur5V[0], beacon->MOChancur5V[1], beacon->MOChancur5V[2], beacon->MOChancur5V[3]);
            StateWindowColumnManager("Mboared Supply V");
            ImGui::Text("%"PRId16, beacon->MOBoardSupV);
            StateWindowColumnManager("Mboard Temperature");
            ImGui::Text("%"PRId16, beacon->MOBoardTemp);
            StateWindowColumnManager("Dboard MPPT In V");
            ImGui::Text("%"PRId16" %"PRId16, beacon->DAMPPTInV[0], beacon->DAMPPTInV[1]);
            StateWindowColumnManager("Dboard MPPT In I");
            ImGui::Text("%"PRId16" %"PRId16, beacon->DAMPPTInI[0], beacon->DAMPPTInI[1]);
            StateWindowColumnManager("Dboard Channel I 5V");
            ImGui::Text("%"PRId16, beacon->DAChancur5V);
            StateWindowColumnManager("Dboard Channel I 12V");
            ImGui::Text("%"PRId16, beacon->DAChancur12V);
            StateWindowColumnManager("Temperature");
            ImGui::Text("%.2f", beacon->IMTemp);
            StateWindowColumnManager("Angular Velocity");
            ImGui::Text("%.2f %.2f %.2f", beacon->IMAngV[0], beacon->IMAngV[1], beacon->IMAngV[2]);
            StateWindowColumnManager("RW Error Count");
            ImGui::Text("%"PRIu16, beacon->IMRWErrcount);
            StateWindowColumnManager("Total RX Bytes");
            ImGui::Text("%"PRIu32, beacon->UXTotRXByte);
            StateWindowColumnManager("Boot Count");
            ImGui::Text("%"PRIu16, beacon->UXBootCount);
            StateWindowColumnManager("Active Conf");
            ImGui::Text("%"PRIu8, beacon->UXActiveConf);
            
        }  
        ImGui::EndTable();
        ImGui::EndTabItem();
        ImGui::SetWindowFontScale(1.0 * fontscale);
    }
    if(ImGui::BeginTabItem("Telemetry"))
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.650f, 0.266f, 0.322f, 1.0f));
        ImGui::Text("Total RX Bytes(Beacon) : %"PRIu32, get_rx_bytes());
        ImGui::SetWindowFontScale(0.6 * fontscale);
        ImGui::PopStyleColor();

        ImGui::SetWindowFontScale(1.0 * fontscale);
        ImGui::EndTabItem();
    }
    if(ImGui::BeginTabItem("Shell"))
    {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.650f, 0.266f, 0.322f, 1.0f));
        ImGui::Text("Total RX Bytes(Beacon) : %"PRIu32, get_rx_bytes());
        ImGui::SetWindowFontScale(0.6 * fontscale);
        ImGui::PopStyleColor();
        
        ImGui::BeginChild("##AX100_shell_typeselect", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y * 0.8), true, NULL);
        
        ImGui::Text("RSSI busy        ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputScalar("##ax100_direct4_rssi_busy", ImGuiDataType_S16, &remote_rssibusy, NULL, NULL, "%d");
        if(ImGui::Button("Load##ax100_direct4_rssi_busy", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
        {
            sleep(0.3);
            if(gs_rparam_get_int16(setup->ax100_node, 5, 0x0048, 0xB00B, setup->default_timeout, &remote_rssibusy) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. rssi_busy : %"PRIu8, rparam0.csp_node);
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::SameLine();
        if(ImGui::Button("Save##ax100_direct4_rssi_busy", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            sleep(0.3);
            if(gs_rparam_set_int16(setup->ax100_node, 5, 0x0048, 0xB00B, setup->default_timeout, remote_rssibusy) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK.");
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::Spacing();

        if(get_rx_bytes() == 0)
            ImGui::Text("Total RX Bytes   ");
        else
            ImGui::Text("Total RX bytes : %"PRIu8"", get_rx_bytes());
        if(ImGui::Button("Load##ax100_direct4_total_rx_bytes", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_get_uint32(setup->ax100_node, 4, 0x003C, 0xB00B, setup->default_timeout, get_rx_bytes_address()) == GS_OK)
                console.AddLog("[OK]##Total RX Bytes : %u", get_rx_bytes());
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::Spacing();
        
        ImGui::Text("CAN enable       ");
        ImGui::SameLine();
        ImGui::Checkbox("##ax100_direct0_can_en", &rparam0.can_en);
        if(ImGui::Button("Load##ax100_direct0_can_en", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
        {
            sleep(0.3);
            if(gs_rparam_get_int8(setup->ax100_node, 0, rparam0.can_en_addr, 0xB00B, setup->default_timeout, (int8_t *)&rparam0.can_en) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. can_en : %s", rparam0.can_en ? "true" : "false");
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::SameLine();
        if(ImGui::Button("Save##ax100_direct0_can_en", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            sleep(0.3);
            if(gs_rparam_set_int8(setup->ax100_node, 0, rparam0.can_en_addr, 0xB00B, setup->default_timeout, (bool)rparam0.can_en) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK.");
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::Spacing();

        ImGui::Text("TX inhibit       ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputScalar("##ax100_direct0_tx_inhibit", ImGuiDataType_U32, &rparam0.tx_inhibit, NULL, NULL, "%u");
        if(ImGui::Button("Load##ax100_direct0_tx_inhibit", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
        {
            sleep(0.3);
            if(gs_rparam_get_uint32(setup->ax100_node, 0, rparam0.tx_inhibit_addr, 0xB00B, setup->default_timeout, &rparam0.tx_inhibit) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. tx_inhibit : %"PRIu32, rparam0.tx_inhibit);
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::SameLine();
        if(ImGui::Button("Save##ax100_direct0_tx_inhibit", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            sleep(0.3);
            if(gs_rparam_set_uint32(setup->ax100_node, 0, rparam0.tx_inhibit_addr, 0xB00B, setup->default_timeout, rparam0.tx_inhibit) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK.");
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
            
        }
        ImGui::Spacing();
        
        ImGui::Text("TX power         ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputScalar("##ax100_direct0_tx_pwr", ImGuiDataType_U8, &rparam0.tx_pwr, NULL, NULL, "%u");
        if(ImGui::Button("Load##ax100_direct0_tx_pwr", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_get_uint8(setup->ax100_node, 0, rparam0.tx_pwr_addr, 0xB00B, setup->default_timeout, &rparam0.tx_pwr) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. tx_pwr : %"PRIu8, rparam0.tx_pwr);
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::SameLine();
        if(ImGui::Button("Save##ax100_direct0_tx_pwr", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_set_uint8(setup->ax100_node, 0, rparam0.tx_pwr_addr, 0xB00B, setup->default_timeout, rparam0.tx_pwr) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK.");
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::Spacing();
        
        ImGui::Text("CSP rtable       ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##ax100_direct0_csp_rtable", rparam0.csp_rtable, sizeof(rparam0.csp_rtable));
        if(ImGui::Button("Load##ax100_direct0_csp_rtable", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_get_string(setup->ax100_node, 0, rparam0.csp_rtable_addr, 0xB00B, setup->default_timeout, rparam0.csp_rtable, sizeof(rparam0.csp_rtable)) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. csp_rtable : %s", rparam0.csp_rtable);
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::SameLine();
        if(ImGui::Button("Save##ax100_direct0_csp_rtable", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_set_string(setup->ax100_node, 0, rparam0.csp_rtable_addr, 0xB00B, setup->default_timeout, rparam0.csp_rtable, sizeof(rparam0.csp_rtable)) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK.");
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::Spacing();
        
        if(remote_activeconf > 3)
            ImGui::Text("Active Conf      ");
        else
            ImGui::Text("Active Conf : %"PRIu8"", remote_activeconf);
        if(ImGui::Button("Load##ax100_direct0_active_conf", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_get_uint8(setup->ax100_node, 4, 0x0018, 0xB00B, setup->default_timeout, &remote_activeconf) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. Active Conf : %"PRId8, remote_activeconf);
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
            
        }
        ImGui::Spacing();

        if(remote_lastrssi > 3)
            ImGui::Text("Last RSSI        ");
        else
            ImGui::Text("Last RSSI : %"PRId16"", remote_lastrssi);
        if(ImGui::Button("Load##ax100_direct0_last_rssi", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_get_int16(setup->ax100_node, 4, 0x0004, 0xB00B, setup->default_timeout, &remote_lastrssi) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. Last RSSI : %"PRId16, remote_lastrssi);
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
            
        }
        ImGui::Spacing();

        ImGui::Text("RX guard         ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputScalar("##ax100_direct1_rx_guard", ImGuiDataType_U16, &rparam1.guard, NULL, NULL, "%u");
        if(ImGui::Button("Load##ax100_direct1_rx_guard", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_get_uint16(setup->ax100_node, 1, rparam1.guard_addr, 0xB00B, setup->default_timeout, &rparam1.guard) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. rx_guard : %"PRIu16, rparam1.guard);
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::SameLine();
        if(ImGui::Button("Save##ax100_direct1_rx_guard", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_set_uint16(setup->ax100_node, 1, rparam1.guard_addr, 0xB00B, setup->default_timeout, rparam1.guard) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK.");
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::Spacing();
        
        ImGui::Text("TX guard         ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputScalar("##ax100_direct5_tx_guard", ImGuiDataType_U16, &rparam5.guard, NULL, NULL, "%u");
        if(ImGui::Button("Load##ax100_direct5_tx_guard", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_get_uint16(setup->ax100_node, 5, rparam5.guard_addr, 0xB00B, setup->default_timeout, &rparam5.guard) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. tx_guard : %"PRIu16, rparam5.guard);
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::SameLine();
        if(ImGui::Button("Save##ax100_direct5_tx_guard", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_set_uint16(setup->ax100_node, 5, rparam5.guard_addr, 0xB00B, setup->default_timeout, rparam5.guard) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK.");
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::Spacing();

        ImGui::Text("RX Baudrate         ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputScalar("##ax100_direct1_rxbaud", ImGuiDataType_U32, &rparam1.baud, NULL, NULL, "%u");
        if(ImGui::Button("Load##ax100_direct1_rx_baud", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_get_uint32(setup->ax100_node, 1, rparam1.baud_addr, 0xB00B, setup->default_timeout, &rparam1.baud) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. rx_baud : %"PRIu16, rparam1.baud);
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::SameLine();
        if(ImGui::Button("Save##ax100_direct1_rxbaud", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_set_uint32(setup->ax100_node, 1, rparam1.baud_addr, 0xB00B, setup->default_timeout, rparam1.baud) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK.");
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::Spacing();

        ImGui::Text("TX Baudrate         ");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputScalar("##ax100_direct5_txbaud", ImGuiDataType_U32, &rparam5.baud, NULL, NULL, "%u");
        if(ImGui::Button("Load##ax100_direct5_txbaud", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_get_uint32(setup->ax100_node, 5, rparam5.baud_addr, 0xB00B, setup->default_timeout, &rparam5.baud) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK. tx_baud : %"PRIu16, rparam5.baud);
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::SameLine();
        if(ImGui::Button("Save##ax100_direct5_txbaud", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(gs_rparam_set_uint32(setup->ax100_node, 5, rparam5.baud_addr, 0xB00B, setup->default_timeout, rparam5.baud) == GS_OK)
                console.AddLog("[OK]##Remote Param Control OK.");
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::Spacing();

        if(ImGui::Button("Save to FRAM##ax100_direct", ImVec2(ImGui::GetContentRegionAvail().x * 0.33, ImGui::GetContentRegionAvail().y)))
        {
            if(gs_rparam_save(setup->ax100_node, setup->default_timeout, 0, 0) == GS_OK)
            {
                gs_rparam_save(setup->ax100_node, setup->default_timeout, 1, 1);
                gs_rparam_save(setup->ax100_node, setup->default_timeout, 5, 5);
                set_guard_spec_micsec(param5.baud, param1.baud, param5.guard, param1.guard, rparam5.guard, rparam1.guard);
                console.AddLog("[OK]##Remote Param Control OK.");
            }
                
            else
                console.AddLog("[ERROR]##Cannot Load Remote Parameter.");
        }
        ImGui::SameLine();
        if(ImGui::Button("Reset Watchdog##ax100_direct", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetContentRegionAvail().y)))
        {
            if(csp_transaction(CSP_PRIO_HIGH, setup->ax100_node, AX100_PORT_GNDWDT_RESET, 1000, NULL, 0, NULL, 0) > 0)
                console.AddLog("Remote Param Control OK.(No Reply)");
            else
                console.AddLog("[ERROR]##Remote Param Control Failed.(No Reply)");
        }
        ImGui::SameLine();
        if(ImGui::Button("Reboot AX100##ax100_direct", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y)))
        {
            csp_reboot(setup->ax100_node);
            console.AddLog("Remote Param Control OK.(No Reply)");
        }
        ImGui::EndChild();

        ImGui::BeginChild("##MEOW_shell_typeselect", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y));
        ImGui::RadioButton("System CMD   ##shelltype_syscmd", &State.Shelltype, TYPE_SYSCMD);
        ImGui::SameLine();
        ImGui::RadioButton("Redirect CMD ##shelltype_rmdir ", &State.Shelltype, TYPE_SYSCMD_REDIR);
        ImGui::SameLine();
        ImGui::RadioButton("Remove Dir   ##shelltype_rmdirp", &State.Shelltype, TYPE_SET_REDIR_PATH);
        ImGui::SameLine();
        ImGui::RadioButton("DS Cleanup   ##shelltype_cleand", &State.Shelltype, TYPE_DS_CLEANUP);

        ImGui::RadioButton("HK Request   ##shelltype_requhk", &State.Shelltype, TYPE_HK_REQUEST);
        ImGui::SameLine();
        ImGui::RadioButton("Reserved     ##shelltype_reserv", &State.Shelltype, TYPE_RESERVED);
        ImGui::SameLine();
        ImGui::RadioButton("KILL         ##shelltype_killin", &State.Shelltype, TYPE_KILL);

        if(State.Shelltype != (int)shellcmd->type)
        {
            shellcmd->type = (uint8_t)State.Shelltype;
            memset(&shellcmd->required, 0, sizeof(shellcmd->required));
        }

        switch (State.Shelltype)
        {
        case TYPE_SYSCMD:
        {   
            ImGui::Text("Out U8   ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            ImGui::InputScalar("##shell_syscmd_redirout", ImGuiDataType_U8, &(shellcmd->required.syscmd.redir_out), NULL, NULL, "%u");
            ImGui::Text("CMD      ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            ImGui::InputText("##shell_syscmd_cmd", shellcmd->required.syscmd.cmd, sizeof(shellcmd->required.syscmd.cmd));
            break;
        }
        case TYPE_SYSCMD_REDIR:
        {   
            ImGui::Text("CMD      ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            ImGui::InputText("##shell_syscmd_cmd1", shellcmd->required.syscmd_redir.cmd, sizeof(shellcmd->required.syscmd_redir.cmd));
            ImGui::Text("Redirect ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            ImGui::InputText("##shell_syscmd_cmd2", shellcmd->required.syscmd_redir.redir_path, sizeof(shellcmd->required.syscmd_redir.redir_path));
            break;
        }
        case TYPE_SET_REDIR_PATH:
        {   
            ImGui::Text("Path     ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            ImGui::InputText("##shell_syscmd_cmd4", shellcmd->required.set_redir.redir_path, sizeof(shellcmd->required.set_redir.redir_path));
            ImGui::Text("");
            break;
        }
        case TYPE_DS_CLEANUP:
        {   
            ImGui::Text("Path     ");
            ImGui::SameLine();
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
            ImGui::InputScalar("##shell_syscmd_redirout5", ImGuiDataType_U8, &(shellcmd->required.ds_cleanup.opt), NULL, NULL, "%u");
            ImGui::Text("");
            break;
        }
        
        default:
        {
            ImGui::Text("ERROR!");
            break;
        }
        }

        if(ImGui::Button("Send##SendShellcmd", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 3)))
        {
            if(State.Debugmode)
            {
                printf("Input Shell Data : ");
                for(int i = 0; i < sizeof(cmd_packet_t); i++)
                    printf("%u\t", ((uint8_t *)shellcmd)[i]);
                printf("\n");
            }
            pthread_create(&p_thread[4], NULL, Direct_Shell, (void *)shellcmd); 
        }
        ImGui::EndChild();
        ImGui::SetWindowFontScale(1.0 * fontscale);
        ImGui::EndTabItem();
    }
    ImGui::EndTabBar();
    ImGui::SetWindowFontScale(1.0);

    ImGui::End();
}

void ImGui_ControlWindow(float fontscale)
{  
    ImGui::Begin("Control GUI", &p_open, mim_winflags);
    ImGui::SetWindowFontScale(fontscale);
    ImGui::BeginChild("PingRegion", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y * 0.3), true, mim_winflags);
    if(ImGui::Button("Ping(AX100)", ImVec2(ImGui::GetContentRegionAvail().x * 0.333, ImGui::GetContentRegionAvail().y)))
    {
        pthread_join(p_thread[4], NULL);
        pthread_create(&p_thread[4], NULL, csp_ping_console, NULL);
        //csp_ping_custom(NULL);
    }
    ImGui::SameLine();
    if(ImGui::Button("Ping(OBC)", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetContentRegionAvail().y)))
    {
        pthread_join(p_thread[4], NULL);
        FSWTle * MIM_TLEinfo = (FSWTle *)malloc(sizeof(FSWTle) + MIM_HAND_DATAFDSTART);
        int NowPingInfo = GetNowTracking();
        if(NowPingInfo < -1)
        {
            console.AddLog("[ERROR]##You Tried Ping without Tracking. Please Check Again.");
        }
        else if (NowPingInfo == -1)
        {
            FSWTleConverter(State.Fatellites->tle, MIM_TLEinfo);
            packetsign * TestPacket;
            TestPacket = PingInit(MIM_TLEinfo);
            pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket); 
        }
        else
        {
            FSWTleConverter(State.Satellites[NowPingInfo]->tle, MIM_TLEinfo);
            packetsign * TestPacket;
            TestPacket = PingInit(MIM_TLEinfo);
            pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket); 
        }
    }
    ImGui::SameLine();
    // if(ImGui::Button("UL Sig", ImVec2(ImGui::GetContentRegionAvail().x * 0.75, ImGui::GetContentRegionAvail().y)))
    // {
    //     pthread_join(p_thread[4], NULL);
    //     // MIM_TLEinfo = FSWTleConverter(TrackingTLE[NowSat]);
    //     packetsign* TestPacket = (packetsign *)malloc(9);
    //     TestPacket->Identifier = MIM_ID;
    //     TestPacket->PacketType = MIM_PT_SIGNAL;
    //     TestPacket->Length = 1;
    //     memset(TestPacket->Data, 0, TestPacket->Length);
    //     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);  
    //     //task_uplink_onorbit((void * )TestPacket);
    // }
    if(ImGui::Button("TMTC Test", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y))) {
        show_typingWindow = true;
        //memset(info, 0, sizeof(CMDInfo));
        msgid = 0;
        fnccode = 0;
    }
    if(show_typingWindow) {
        ImGui::Begin("##msgid_fnccode_TypeWindow");
        ImGui::Text("Type MsgID: ");
        ImGui::SameLine();
        ImGui::InputScalar("##Type MsgID",ImGuiDataType_U16,&msgid);
        ImGui::Text("Type Fnccode: ");
        ImGui::SameLine();
        ImGui::InputScalar("##Type Fnccode",ImGuiDataType_U16,&fnccode);
        if(ImGui::Button("Generate CMD")) {
            pthread_join(p_thread[4], NULL);
            packetsign * TestPacket = (packetsign *)malloc(2+2+4+8); // 8 is for data
            TestPacket->Identifier = HVD_TEST;
            TestPacket->PacketType = MIM_PT_TMTC_TEST;
            TestPacket->Length = 8;
            memset(TestPacket->Data,0,TestPacket->Length);
            uint8_t cmd[8] = {0,};
            memcpy(cmd,&msgid,sizeof(uint16_t));
            memcpy(cmd+sizeof(uint16_t),&fnccode,sizeof(uint16_t));
            //cmd[0]=0x18;
            //cmd[1]=0x80;
            //cmd[2]=0xc0;
            //cmd[3]=0x00;
            cmd[4]=0x00;
            cmd[5]=0x01;
            cmd[6]=0x00;
            cmd[7]=0x02;
            memcpy(TestPacket->Data,cmd,sizeof(cmd));
            //pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
            ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                    cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5],cmd[6],cmd[7]);
            //show_typingWindow = false;
        }
        ImGui::End();
    }

    ImGui::EndChild();
    if (ImGui::Button("Baud Rate Calibration",ImVec2(ImGui::GetContentRegionAvail().x*0.33,ImGui::GetFontSize()*1.5))) {
        pthread_join(p_thread[5],NULL);
        pthread_create(&p_thread[5],NULL,csp_baud_calibration,NULL);
    }
    if (ImGui::Button("Frequency Calibration",ImVec2(ImGui::GetContentRegionAvail().x*0.33,ImGui::GetFontSize()*1.5))) {
        pthread_join(p_thread[5],NULL);
        pthread_create(&p_thread[5],NULL,csp_freq_calibration,NULL);
    }
    if (ImGui::Button("CMD Manager", ImVec2(ImGui::GetContentRegionAvail().x * 0.33, ImGui::GetFontSize() * 1.5)))
    {
        State.Display_CMD = true;
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
    {
        NowCMD = 0;
        for(int i = 0; i < 256; i++)
        {
            if(SatCMD[i] != NULL)
            {
                delete SatCMD[i];
                SatCMD[i] = NULL;
            }    
        }
    }
    ImGui::SetWindowFontScale(0.9);
    if (ImGui::BeginTable("##CMDTables", 5, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y)))
    {
        ImGui::TableSetupScrollFreeze(freeze_cols, freeze_rows);
        ImGui::TableSetupColumn("ID",           ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("FuncCode",       ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("Scheduled",       ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("DataLength",  ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableSetupColumn("Order",  ImGuiTableColumnFlags_NoHide, 0.0f);
        ImGui::TableHeadersRow();
        ImGui::TableNextColumn();
        for(int i = 0; i < 256; i++)
        {
            if(SatCMD[i] == NULL)
                continue;
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            msgidbuf[1] = ((uint8_t *)SatCMD[i]->CmdHeader)[0];
            msgidbuf[0] = ((uint8_t *)SatCMD[i]->CmdHeader)[1];
            msgidinfo = *((uint16_t *)msgidbuf);
            ImGui::Text("%x", msgidinfo);
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%"PRIu16, SatCMD[i]->GetFncCode());
            ImGui::TableSetColumnIndex(2);
            if(SatCMD[i]->Scheduled)
                ImGui::Text("Yes");
            else
                ImGui::Text("No");
            ImGui::TableSetColumnIndex(3);
            ImGui::Text("%"PRIu16, SatCMD[i]->GetSize());
            ImGui::TableSetColumnIndex(4);
            sprintf(Sendinglabelbuf, "Send##%d", i);
            if(ImGui::Button(Sendinglabelbuf, ImVec2(ImGui::GetContentRegionAvail().x * 0.7, ImGui::GetFontSize() * 1.5)))
            {
                pthread_join(p_thread[4], NULL);
                if(SatCMD[i]->Checksum)
                    SatCMD[i]->GenerateChecksum();
                packetsign * SendingCMD = SatCMD[i]->GenerateCMDPacket();
                pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)SendingCMD);
            }
            ImGui::SameLine();
            sprintf(Deletelabelbuf, "X##%d", i);
            if(ImGui::Button(Deletelabelbuf, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
            {
                delete SatCMD[i];
                SatCMD[i] = NULL;
            }
        }  
    }
    ImGui::EndTable();
    ImGui::SetWindowFontScale(1.0);
    ImGui::End();
}

void ImGui_CommandWindow(float fontscale)
{  
    ImGui::Begin("Command GUI", &p_open, mim_winflags);
    ImGui::SetWindowFontScale(fontscale);
    ImGui::BeginChild("##FTPManager", ImVec2(ImGui::GetContentRegionAvail().x * 0.4, ImGui::GetContentRegionAvail().y), true, mim_winflags);
    ImGui::Text("File List");
    ImGui::Text("Telemetry : ");
    ImGui::SameLine();
    ImGui::RadioButton("HK ##ftpwindow", &temptarget, MIM_DLTYPE_HK);
    ImGui::SameLine();
    ImGui::RadioButton("AOD##ftpwindow", &temptarget, MIM_DLTYPE_AOD);
    ImGui::SameLine();
    ImGui::RadioButton("LOG##ftpwindow", &temptarget, MIM_DLTYPE_LOG);
    ImGui::SameLine();
    ImGui::RadioButton("SNSR##ftpwindow", &temptarget, MIM_DLTYPE_SNSR);
    ImGui::Text("            ");
    ImGui::SameLine();
    ImGui::RadioButton("GPS##ftpwindow", &temptarget, MIM_DLTYPE_GPS);
    ImGui::SameLine();
    ImGui::RadioButton("CTRLO##ftpwindow", &temptarget, MIM_DLTYPE_CTRLO);

    ImGui::Text("Status    : ");
    ImGui::SameLine();
    ImGui::RadioButton("NOW##ftpwindow", &tempstatus, MIM_DLSTAT_NEW);
    ImGui::SameLine();
    ImGui::RadioButton("OLD##ftpwindow", &tempstatus, MIM_DLSTAT_OLD);
    ImGui::SameLine();
    ImGui::RadioButton("CUR##ftpwindow", &tempstatus, MIM_DLSTAT_ING);
    ImGui::BeginTable("##filelisttable", 3, false, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 3.0f));
    
    ImGui::TableSetupScrollFreeze(freeze_cols, freeze_rows);
    ImGui::TableSetupColumn("Number##tle",                ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
    ImGui::TableSetupColumn("Step##tle",                 ImGuiTableColumnFlags_NoHide, 0.0f);
    ImGui::TableSetupColumn("Offset##tle",            ImGuiTableColumnFlags_NoHide, 0.0f);
    ImGui::TableHeadersRow();
    if(tempfilenum < 1)
        tempfilenum = 1;
    if(tempstep < 0)
        tempstep = 0;
    if(tempoffset < 0)
        tempoffset = 0;
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::InputInt("##ftpwindow_filenum", &tempfilenum);
    ImGui::TableSetColumnIndex(1);
    ImGui::InputInt("##ftpwindow_step", &tempstep);
    ImGui::TableSetColumnIndex(2);
    ImGui::InputInt("##ftpwindow_offset", &tempoffset);
    ImGui::EndTable();
    

    if (ImGui::Button("Save##ftpwindow", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetContentRegionAvail().y)))
    {
        request->target = (uint16_t) temptarget;
        request->filestatus = (uint16_t) tempstatus;
        request->filenum = (uint32_t) tempfilenum;
        request->offset = (uint32_t) tempoffset;
        request->step = (uint32_t) tempstep;
    }
    ImGui::SameLine();
    if (ImGui::Button("Request##ftpwindow", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y)))
    {
        packetsign * request_ = (packetsign *)malloc(sizeof(dlreqdata) + 8);
        request_->Identifier = MIM_ID;
        request_->PacketType = MIM_PT_DLREQ;
        request_->Length = sizeof(dlreqdata);
        memcpy(request_->Data, request, sizeof(dlreqdata));
        pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)request_); 
    }
    ImGui::EndChild();

    ImGui::SameLine();
    ImGui::BeginChild("##FTPSending", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y), true, mim_winflags);
    ImGui::Text("FTP  ");
    ImGui::SameLine();
    ImGui::RadioButton("V1      ##FTPVersionSelect1", &State.ftp_version, 1);
    ImGui::SameLine();
    ImGui::RadioButton("V2      ##FTPVersionSelect2", &State.ftp_version, 2);
    ImGui::SameLine();
    ImGui::Text("           ");
    ImGui::SameLine();
    ImGui::Text("Chunk Size ");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    ImGui::InputScalar("##ftpchunksize", ImGuiDataType_U32, &State.chunk_sz, NULL, NULL, "%u", NULL);
    if(State.chunk_sz < 1)
        State.chunk_sz = 1;
    ImGui::Text("Task ");
    ImGui::SameLine();
    ImGui::RadioButton("Upload  ##FTPTaskSelect00", &State.ftp_task, FTP_UPLOAD_REQUEST);
    ImGui::SameLine();
    ImGui::RadioButton("Download##FTPTaskSelect02", &State.ftp_task, FTP_DOWNLOAD_REQUEST);
    ImGui::SameLine();
    ImGui::RadioButton("Listup  ##FTPTaskSelect07", &State.ftp_task, FTP_LIST_REQUEST);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
    
    if(!State.AMPON)
    {
        if(State.AmpTime > 0)
        {
            strftime(AmpTimeBuf, sizeof(AmpTimeBuf), "Disabled  %M:%S", State.AmpTM);
            ImGui::PushItemFlag(ImGuiButtonFlags_Disabled, true);
            ImGui::Button(AmpTimeBuf, ImVec2(ImGui::GetContentRegionAvail().x , ImGui::GetFontSize() * 1.5));
            ImGui::PopItemFlag();
        }
            
        else
        {
            sprintf(AmpTimeBuf, "AMP ON");
            if (ImGui::Button(AmpTimeBuf, ImVec2(ImGui::GetContentRegionAvail().x , ImGui::GetFontSize() * 1.5)))
            {
                pthread_join(p_thread[6], NULL);
                pid_t pid_on = fork();

                if (pid_on == -1) {
                    console.AddLog("[ERROR]##Failed to fork process");
                }
                else if (pid_on == 0) {
                    // Child process
                    execl("./amp/ampcontrol", "ampcontrol", NULL);
                    
                }
                else
                {
                    console.AddLog("[OK]##Forked Safe Amplifier Process.(ON, Timer)");
                    pthread_create(&p_thread[6], NULL, AmpTimer, NULL);
                }
            }
        }
            
        
    }
    else
    {
        if(State.AmpTime > 0)
            strftime(AmpTimeBuf, sizeof(AmpTimeBuf), "AMP OFF %M:%S", State.AmpTM);
        else
            sprintf(AmpTimeBuf, "AMP OFF");
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.650f, 0.266f, 0.322f, 1.0f));
        if (ImGui::Button(AmpTimeBuf, ImVec2(ImGui::GetContentRegionAvail().x , ImGui::GetFontSize() * 1.5)))
        {
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
        }
        ImGui::PopStyleColor();
    }
    // ImGui::Text("     ");
    // ImGui::SameLine();
    // ImGui::RadioButton("Move    ##FTPTaskSelect10", &State.ftp_task, FTP_MOVE_REQUEST);
    // ImGui::SameLine();
    // ImGui::RadioButton("Remove  ##FTPTaskSelect12", &State.ftp_task, FTP_REMOVE_REQUEST);
    // ImGui::SameLine();
    // ImGui::RadioButton("Copy    ##FTPTaskSelect23", &State.ftp_task, FTP_COPY_REQUEST);
    // ImGui::SameLine();
    // ImGui::RadioButton("MKDIR   ##FTPTaskSelect25", &State.ftp_task, FTP_MKDIR_REQUEST);
    // ImGui::SameLine();
    // ImGui::RadioButton("RMDIR   ##FTPTaskSelect27", &State.ftp_task, FTP_RMDIR_REQUEST);
    switch(State.ftp_task)
    {
    case FTP_UPLOAD_REQUEST :{
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.8);
        if (ImGui::BeginCombo("##FTPCombobox", State.ftplistup[NowFTP].name))
        {
            for (int FTPindex = 0; FTPindex < IM_ARRAYSIZE(State.ftplistup); FTPindex++)
            {
                if (State.ftplistup[FTPindex].name[0] == 0)
                    continue;
                bool SelectedFTP = (NowFTP == FTPindex);
                char Selectedlabel[30];
                sprintf(Selectedlabel, "%s##%d", State.ftplistup[FTPindex].name, FTPindex);
                if (ImGui::Selectable(Selectedlabel, SelectedFTP))
                    NowFTP = FTPindex;
                        
                if (SelectedFTP)
                    ImGui::SetItemDefaultFocus();
                usleep(10);
            }
            ImGui::EndCombo();
        }
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button("Upload", ImVec2(ImGui::GetContentRegionAvail().x , ImGui::GetFontSize() * 1.5)))
        {
            if(State.ftp_mode)
                console.AddLog("[ERROR]##Still Uplink Thread is Running. Please Retry Later.");
            else
            {
                pthread_join(p_thread[8], NULL);
                if(State.ftp_version == 1)
                    pthread_create(&p_thread[8], NULL, ftp_uplink_force, &State.ftplistup[NowFTP]);
                else
                    pthread_create(&p_thread[8], NULL, ftp_uplink_onorbit, &State.ftplistup[NowFTP]);
            }
        }
        ImGui::Text("Local  ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_local_filepath : ", State.ftplistup[NowFTP].local_path, sizeof(State.ftplistup[NowFTP].local_path));
        ImGui::PopItemWidth();
        ImGui::Text("Remote ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_remote_filepath : ", State.ftplistup[NowFTP].remote_path, sizeof(State.ftplistup[NowFTP].remote_path));
        ImGui::PopItemWidth();
        ImGui::Text(State.FTPWindowBuffer);
        break;
    }
    case FTP_DOWNLOAD_REQUEST :{
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.8);
        if (ImGui::BeginCombo("##FTPCombobox", State.ftplistup[NowFTP].name))
        {
            for (int FTPindex = 0; FTPindex < IM_ARRAYSIZE(State.ftplistup); FTPindex++)
            {
                if (State.ftplistup[FTPindex].name[0] == 0)
                    continue;
                bool SelectedFTP = (NowFTP == FTPindex);
                char Selectedlabel[30];
                sprintf(Selectedlabel, "%s##%d", State.ftplistup[FTPindex].name, FTPindex);
                if (ImGui::Selectable(Selectedlabel, SelectedFTP))
                    NowFTP = FTPindex;
                        
                if (SelectedFTP)
                    ImGui::SetItemDefaultFocus();
                usleep(10);
            }
            ImGui::EndCombo();
        }
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (ImGui::Button("Download", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(State.ftp_mode)
                console.AddLog("[ERROR]##Still Uplink Thread is Running. Please Retry Later.");
            else
            {
                pthread_join(p_thread[8], NULL);
                if(State.ftp_version == 1)
                    pthread_create(&p_thread[8], NULL, ftp_downlink_force, &State.ftplistup[NowFTP]);
                else
                    pthread_create(&p_thread[8], NULL, ftp_downlink_onorbit, &State.ftplistup[NowFTP]);
            }
        }
        ImGui::Text("Local  ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_local_filepath : ", State.ftplistup[NowFTP].local_path, sizeof(State.ftplistup[NowFTP].local_path));
        ImGui::PopItemWidth();
        ImGui::Text("Remote ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_remote_filepath : ", State.ftplistup[NowFTP].remote_path, sizeof(State.ftplistup[NowFTP].remote_path));
        ImGui::PopItemWidth();
        ImGui::Text(State.FTPWindowBuffer);

        break;
    }
    case FTP_LIST_REQUEST :{
        if(State.ftp_version != 2)
            State.ftp_task = FTP_UPLOAD_REQUEST;
        ImGui::Text("Path   ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_listup_path : ", State.gslistup->fpathbuf, sizeof(State.gslistup->fpathbuf));
        ImGui::PopItemWidth();
        if (ImGui::Button("Listup", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(State.ftp_mode)
                console.AddLog("[ERROR]##Still Uplink Thread is Running. Please Retry Later.");
            else
            {
                pthread_join(p_thread[8], NULL);
                pthread_create(&p_thread[8], NULL, ftp_list_onorbit, NULL);
            }
        }
        ImGui::Text(State.FTPWindowBuffer);
        ImGui::Text(State.gslistup->fdispbuf);
        break;
    }
    case FTP_MOVE_REQUEST :{
        if(State.ftp_version != 2)
            State.ftp_task = FTP_UPLOAD_REQUEST;
        ImGui::Text("From   ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_movefrom_path : ", State.gsmove->from, sizeof(State.gsmove->from));
        ImGui::PopItemWidth();
        ImGui::Text("To     ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_moveto_path : ", State.gsmove->to, sizeof(State.gsmove->to));
        ImGui::PopItemWidth();
        if (ImGui::Button("Move", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(State.ftp_mode)
                console.AddLog("[ERROR]##Still Uplink Thread is Running. Please Retry Later.");
            else
            {
                pthread_join(p_thread[8], NULL);
                pthread_create(&p_thread[8], NULL, ftp_move_onorbit, NULL);
            }
        }
        ImGui::Text(State.FTPWindowBuffer);
        break;
    }
    case FTP_REMOVE_REQUEST :{
        if(State.ftp_version != 2)
            State.ftp_task = FTP_UPLOAD_REQUEST;
        ImGui::Text("Path   ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_REMOVE_path : ", State.gsremove->path, sizeof(State.gsremove->path));
        ImGui::PopItemWidth();
        if (ImGui::Button("Remove", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(State.ftp_mode)
                console.AddLog("[ERROR]##Still Uplink Thread is Running. Please Retry Later.");
            else
            {
                pthread_join(p_thread[8], NULL);
                pthread_create(&p_thread[8], NULL, ftp_remove_onorbit, NULL);
            }
        }
        ImGui::Text(State.FTPWindowBuffer);
        break;
    }
    case FTP_COPY_REQUEST :{
        if(State.ftp_version != 2)
            State.ftp_task = FTP_UPLOAD_REQUEST;
        ImGui::Text("From   ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_copyfrom_path : ", State.gscopy->from, sizeof(State.gscopy->from));
        ImGui::PopItemWidth();
        ImGui::Text("To     ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_copyto_path : ", State.gscopy->to, sizeof(State.gscopy->to));
        ImGui::PopItemWidth();
        if (ImGui::Button("Copy", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(State.ftp_mode)
                console.AddLog("[ERROR]##Still Uplink Thread is Running. Please Retry Later.");
            else
            {
                pthread_join(p_thread[8], NULL);
                pthread_create(&p_thread[8], NULL, ftp_copy_onorbit, NULL);
            }
        }
        ImGui::Text(State.FTPWindowBuffer);
        break;
    }
    case FTP_MKDIR_REQUEST :{
        if(State.ftp_version != 2)
            State.ftp_task = FTP_UPLOAD_REQUEST;
        ImGui::Text("Path   ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_mkdir_path : ", State.gsmkdir->path, sizeof(State.gsmkdir->path));
        ImGui::PopItemWidth();
        if (ImGui::Button("Make Directory", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(State.ftp_mode)
                console.AddLog("[ERROR]##Still Uplink Thread is Running. Please Retry Later.");
            else
            {
                pthread_join(p_thread[8], NULL);
                pthread_create(&p_thread[8], NULL, ftp_mkdir_onorbit, NULL);
            }
        }
        ImGui::Text(State.FTPWindowBuffer);
        break;
    }
    case FTP_RMDIR_REQUEST :{
        if(State.ftp_version != 2)
            State.ftp_task = FTP_UPLOAD_REQUEST;
        ImGui::Text("Path   ");
        ImGui::SameLine();
        ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::InputText("##FTP_rmdir_path : ", State.gsrmdir->path, sizeof(State.gsrmdir->path));
        ImGui::PopItemWidth();
        if (ImGui::Button("Remove Directory", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            if(State.ftp_mode)
                console.AddLog("[ERROR]##Still Uplink Thread is Running. Please Retry Later.");
            else
            {
                pthread_join(p_thread[8], NULL);
                pthread_create(&p_thread[8], NULL, ftp_rmdir_onorbit, NULL);
            }
        }
        ImGui::Text(State.FTPWindowBuffer);
        break;
    }
    default : {
        State.ftp_version = 2;
        State.ftp_task = FTP_UPLOAD_REQUEST;
        break;
    }
    }

    ImGui::EndChild();
    ImGui::SetWindowFontScale(1.0);
    ImGui::End();
}

bool LoadTextureFromFile(const char* filename, GLuint* out_texture, int width, int height)
{
    // Load from file
    int image_width = 0;
    int image_height = 0;
    unsigned char* image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);
    if (image_data == NULL)
        return false;

    // Create a OpenGL texture identifier
    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    // Setup filtering parameters for display
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE); // This is required on WebGL for non power-of-two textures
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE); // Same

    // Upload pixels into texture
#if defined(GL_UNPACK_ROW_LENGTH) && !defined(__EMSCRIPTEN__)
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
#endif
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);
    stbi_image_free(image_data);

    *out_texture = image_texture;

    return true;
}

void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}


int Console::Stricmp(const char* str1, const char* str2)
{
    int d;
    while (d = toupper(*str2) - toupper(*str1) == 0 && *str1)
    {
        str1++;
        str2++;
    }
    return d;
}

int Console::Strnicmp(const char* str1, const char* str2, int n)
{
    int d = 0;
    while(n > 0 && (d = toupper(*str2) - toupper(*str1)==0 && *str1) )
    {
        str1++;
        str2++;
        n--;
    }
    return d;
}
char* Console::Strdup(const char *str)
{
    size_t len = strlen(str) + 1;
    void* buf = malloc(len);
    IM_ASSERT(buf);
    return (char*)memcpy(buf, (const void*)str, len);
}
void Console::Strtrim(char* str)
{
    char* str_end = str + strlen(str);
    while (str_end > str && str_end[-1] == ' ')
        str_end--;
    *str_end = 0;
}

void Console::ClearLog()
{
    for (int i = 0; i < _items.Size; i++)
        free(_items[i]);
    _items.clear();
}

void Console::AddLog(const char* fmt, ...) IM_FMTARGS(2)
{
    char str[1024];
    char buf[1024];
    char tmp[1024];
    char * pos;
    char * bug;
    _now = time(NULL);
    _now_time = *localtime(&_now);

    strftime(tmp, sizeof(tmp), "[%H:%M:%S]  ", &_now_time);
    
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, IM_ARRAYSIZE(buf), fmt, args);
    buf[IM_ARRAYSIZE(buf) - 1] = 0;
    va_end(args);

    if ((pos = strstr(buf, "##")) != 0)
    {
        strcpy(str, tmp);
        strcat(str, pos + 2);
        strcpy(pos + 2, str);
        //Write to IMGUI Console.
        if ((bug = strstr(buf, "[DEBUG]##")) != 0)
        {
            if(State.Debugmode)
            {
                _debug_fs.write(str, strlen(str));
                _debug_fs.write("\n", 1);
            }
                
        }
        else if ((bug = strstr(buf, "[FTP]##")) != 0)
        {
            _history_fs.write(str, strlen(str));
            _history_fs.write("\n", 1);
        }
        else
        {
            _items.push_back(Strdup(buf));
            _history_fs.write(str, strlen(str));
            _history_fs.write("\n", 1);
        }
    }
    else
    {
        strcpy(str, tmp);
        strcat(str, buf);
        //Write to IMGUI Console.
        _items.push_back(Strdup(str));
        _history_fs.write(str, strlen(str));
        _history_fs.write("\n", 1);
    }

    this->_history_fs.flush();
    this->_debug_fs.flush();
    
}

void Console::DelStartingCharLog(const char* fmt)
{
    char buf[1024];
    strcpy(buf, fmt);
    for (int i = 0; i < _items.Size; i++)
    {
        char* item = _items[i];
        if(strstr(item, buf))
        {
            _items.erase(&_items[i]);
        }
    }
}

void Console::DelPrefixLog(const char* fmt)
{
    char buf[1024];
    strcpy(buf, fmt);
    for (int i = 0; i < _items.Size; i++)
    {
        char * item = _items[i];
        if (strstr(item, buf))
        {
            _items.insert(&_items[i], item + 1);
            _items.erase(&_items[i + 1]);
        }
    }
}

void Console::Draw(const char* title, bool* p_open, float fontscale)
{
    ImGui::SetNextWindowPos(ImVec2(this -> _x_pos, this -> _y_pos), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(this -> _width, this -> _height), ImGuiCond_Always);

    if (!ImGui::Begin(title, p_open, this -> _window_flags))
    {
        ImGui::End();
        return;
    }
    ImGui::SetWindowFontScale(fontscale);

    bool copy_to_clipboard = false;

    const float footer_height_to_reserve = 0.0;
    ImGui::BeginChild("ScrollingRegion", ImVec2(0, -footer_height_to_reserve), false, ImGuiWindowFlags_HorizontalScrollbar);
    //ImGui::BeginChild("scrolling");
    if(ImGui::BeginPopupContextWindow())
    {
        if (ImGui::Selectable("Clear"))
            ClearLog();
        copy_to_clipboard = ImGui::Selectable("Copy");
        ImGui::EndPopup();
    }
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4,1));
    if (copy_to_clipboard)
        ImGui::LogToClipboard();
    for(int i = 0; i < _items.Size; i++)
    {
        char*item = _items[i];

        bool pop_color = false;
        if(strstr(item, "[ERROR]##"))
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.70f, 0.38f, 0.36f, 1.0f));
            pop_color = true;
            item += 9;
        }
        else if (strstr(item, "[OK]##"))
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.35f, 0.60f, 0.70f, 1.0f));
            pop_color = true;
            item += 6;
        }
        else if (strncmp(item, "# ", 2) == 0)
        {
            ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.6f, 1.0f));
            pop_color = true;
        }
        else if (strstr(item, "$"))
        {
            item += 1;
        }
        ImGui::SetWindowFontScale(1.1);
        ImGui::TextUnformatted(item);
        if (pop_color)
        {
            ImGui::PopStyleColor();
        }
        ImGui::SetWindowFontScale(1.0);
    }

    if (copy_to_clipboard)
        ImGui::LogFinish();

    if (_push_to_bottom || ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.0f);

    ImGui::PopStyleVar();
    ImGui::EndChild();
    ImGui::SetWindowFontScale(1.0);

    ImGui::End();
    
}

void Console::ChangeWindowSize(float x_pos, float y_pos, float width, float height)
{
    this -> _x_pos = x_pos;
    this -> _y_pos = y_pos;
    this -> _width = width;
    this -> _height = height;
}


void ImGuiCustomStyle(ImGuiStyle* style)
{
    style->WindowPadding = ImVec2(15, 15);
	style->WindowRounding = 5.0f;
	style->FramePadding = ImVec2(5, 5);
	style->FrameRounding = 4.0f;
	style->ItemSpacing = ImVec2(12, 8);
	style->ItemInnerSpacing = ImVec2(8, 6);
	style->IndentSpacing = 25.0f;
	style->ScrollbarSize = 15.0f;
	style->ScrollbarRounding = 9.0f;
	style->GrabMinSize = 5.0f;
	style->GrabRounding = 3.0f;

    //Window Background
    ImVec4 Colorset_1 = ImVec4(0.141f, 0.364f, 0.376f, 1.00f);
    //Disabled
    ImVec4 Colorset_2 = ImVec4(0.24f, 0.23f, 0.29f, 1.00f);
    //Active Title
    ImVec4 Colorset_3 = ImVec4(0.235f, 0.603f, 0.623f, 1.00f);
    //General Title, Button
    ImVec4 Colorset_4 = ImVec4(0.180f, 0.462f, 0.478f, 1.00f);
    //ScrollBar
    ImVec4 Colorset_5 = ImVec4(0.80f, 0.80f, 0.83f, 0.31f);
    //Active Button
    ImVec4 Colorset_6 = ImVec4(0.56f, 0.56f, 0.58f, 1.00f);
 
	style->Colors[ImGuiCol_Text] = ImVec4(0.972f, 0.733f, 0.701f, 1.00f);
	style->Colors[ImGuiCol_TextDisabled] = Colorset_2;
	style->Colors[ImGuiCol_WindowBg] = Colorset_1;
	style->Colors[ImGuiCol_PopupBg] = Colorset_1;
	style->Colors[ImGuiCol_Border] = ImVec4(0.972f, 0.733f, 0.701f, 1.00f);
	style->Colors[ImGuiCol_BorderShadow] = ImVec4(0.92f, 0.91f, 0.88f, 0.00f);
	style->Colors[ImGuiCol_FrameBg] = Colorset_4;
	style->Colors[ImGuiCol_FrameBgHovered] = Colorset_2;
	style->Colors[ImGuiCol_FrameBgActive] = Colorset_6;
	style->Colors[ImGuiCol_TitleBg] = Colorset_4;
	style->Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(1.00f, 0.98f, 0.95f, 0.75f);
	style->Colors[ImGuiCol_TitleBgActive] = Colorset_3;
	style->Colors[ImGuiCol_MenuBarBg] = Colorset_4;
	style->Colors[ImGuiCol_ScrollbarBg] = Colorset_1;
	style->Colors[ImGuiCol_ScrollbarGrab] = Colorset_5 ;
	style->Colors[ImGuiCol_ScrollbarGrabHovered] = Colorset_6;
	style->Colors[ImGuiCol_ScrollbarGrabActive] = Colorset_5;
	style->Colors[ImGuiCol_CheckMark] = Colorset_5 ;
	style->Colors[ImGuiCol_SliderGrab] = Colorset_5 ;
	style->Colors[ImGuiCol_SliderGrabActive] = Colorset_1;
	style->Colors[ImGuiCol_Button] = Colorset_4;
	style->Colors[ImGuiCol_ButtonHovered] = Colorset_2;
	style->Colors[ImGuiCol_ButtonActive] = Colorset_6;
	style->Colors[ImGuiCol_Header] = Colorset_4;
	style->Colors[ImGuiCol_HeaderHovered] = Colorset_6;
	style->Colors[ImGuiCol_HeaderActive] = Colorset_1;
	style->Colors[ImGuiCol_ResizeGrip] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
	style->Colors[ImGuiCol_ResizeGripHovered] = Colorset_6;
	style->Colors[ImGuiCol_ResizeGripActive] = Colorset_1;
	style->Colors[ImGuiCol_PlotLines] = ImVec4(0.40f, 0.39f, 0.38f, 0.63f);
	style->Colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.25f, 1.00f, 0.00f, 1.00f);
	style->Colors[ImGuiCol_PlotHistogram] = ImVec4(0.40f, 0.39f, 0.38f, 0.63f);
	style->Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.25f, 1.00f, 0.00f, 1.00f);
	style->Colors[ImGuiCol_TextSelectedBg] = Colorset_3;
    style->Colors[ImGuiCol_Tab] = Colorset_4;
    style->Colors[ImGuiCol_TabActive] = Colorset_3;
}

bool popup_setup(Setup * setup)
{
    ImGui::OpenPopup("MIMAN Setup Page");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(720, 540), ImGuiCond_Appearing);

    if ( ImGui::BeginPopupModal("MIMAN Setup Page", NULL, ImGuiWindowFlags_AlwaysAutoResize) )
    {
        ImGui::SetWindowFontScale(0.8f);
        ImGui::Text("Transceiver");
        ImGui::BeginChild("##setup_transceiver", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 13.5), true, mim_winflags);
        ImGui::Text("Device Name");
        ImGui::InputText("##GS100_name", &setup->Transciever_devname[0], sizeof(setup->Transciever_devname));
        ImGui::Text("Default Frequency(Hz)");
        ImGui::InputDouble("##GS100_freq", &setup->default_freq);
        ImGui::Text("Baudrate");
        ImGui::InputScalar("##GS100_baud", ImGuiDataType_U32, &setup->Transceiver_baud, NULL, NULL, "%u");
        ImGui::Text("Delay(ms)");
        ImGui::InputScalar("##GS100_delay", ImGuiDataType_U32, &setup->default_timeout, NULL, NULL, "%u");
        ImGui::EndChild();

        ImGui::Text("Switch Box");
        ImGui::BeginChild("##setup_switch", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 7.5), true, mim_winflags);
        ImGui::Text("Device Name");
        ImGui::InputText("##KTA223_name", &setup->Switch_devname[0], sizeof(setup->Switch_devname));
        ImGui::Text("Baudrate");
        ImGui::InputScalar("##KTA223_baud", ImGuiDataType_U32, &setup->Switch_baud, NULL, NULL, "%u");
        ImGui::EndChild();

        ImGui::Text("Rotator Controller");
        ImGui::BeginChild("##setup_rotator", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 7.5), true, mim_winflags);
        ImGui::Text("Device Name");
        ImGui::InputText("##GS232B_name", &setup->Rotator_devname[0], sizeof(setup->Rotator_devname));
        ImGui::Text("Baudrate");
        ImGui::InputScalar("##GS232B_baud", ImGuiDataType_U32, &setup->Rotator_baud, NULL, NULL, "%u");
        ImGui::EndChild();

        ImGui::Text("Route Table");
        ImGui::BeginChild("##setup_node", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 8.0), true, mim_winflags);
        ImGui::Text("GS100 node");
        ImGui::SameLine();
        ImGui::InputScalar("##gs100_node", ImGuiDataType_U8, &setup->gs100_node, NULL, NULL, "%u");
        ImGui::Text("GS PC node");
        ImGui::SameLine();
        ImGui::InputScalar("##gspc_node", ImGuiDataType_U8, &setup->kiss_node, NULL, NULL, "%u");
        ImGui::Text("AX100 node");
        ImGui::SameLine();
        ImGui::InputScalar("##ax100_node", ImGuiDataType_U8, &setup->ax100_node, NULL, NULL, "%u");
        ImGui::Text("OBC   node");
        ImGui::SameLine();
        ImGui::InputScalar("##obd_node", ImGuiDataType_U8, &setup->obc_node, NULL, NULL, "%u");
        ImGui::EndChild();

        ImGui::Text("Use S-Band GS");
        ImGui::SameLine();
        ImGui::Checkbox("##s_band_use_state_check", &State.SbandUse);
        if(State.SbandUse)
        {
            ImGui::Text("S-Band");
            ImGui::BeginChild("##setup_s_band", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 6.5), true, mim_winflags);
            ImGui::Text("Username");
            ImGui::SameLine();
            ImGui::InputText("##S-BandUsername", &setup->S_Username[0], sizeof(setup->S_Username));
            ImGui::Text("IP Address");
            ImGui::SameLine();
            ImGui::InputText("##S-BandIP", &setup->S_Address[0], sizeof(setup->S_Address));
            ImGui::Text("Password");
            ImGui::SameLine();
            ImGui::InputText("##S-BandPW", &setup->S_passwd[0], sizeof(setup->S_passwd));
            ImGui::EndChild();
        }
        ImGui::SetWindowFontScale(1.0f);
        

        if (ImGui::Button("Start", ImVec2(ImGui::GetContentRegionAvail().x * 0.33, ImGui::GetFontSize() * 1.5)))
        {
            State.GUI = true;
            State.InitializeRequired = true;
            set_serial_spec_micsec(setup->Transceiver_baud, setup->Switch_baud, 1000000, setup->queue_delay, setup->gs100_node, setup->default_timeout, setup->guard_delay);


            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::SameLine();
        if(ImGui::Button("Reset Watchdog", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
        {
            ResetWatchdog();
        }
        ImGui::SameLine();
        if (ImGui::Button("Quit", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            State.GUI = false;
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        
        ImGui::EndPopup();
        return true;
    }
}

//PopupWindowFunctions
bool popup_param_table0()
{
    ImGui::OpenPopup("GS100 Parameter Table 0");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(720, 720), ImGuiCond_Appearing);
    if ( ImGui::BeginPopupModal("GS100 Parameter Table 0", NULL, ImGuiWindowFlags_AlwaysAutoResize) )
    {
        ImGui::BeginChild("##Ptable0", ImVec2(ImGui::GetContentRegionAvail().x, 650), true, mim_winflags);
        ImGui::PushItemWidth(300);
        ImGui::Text("RSSI Offset        : ");
        ImGui::SameLine();
        ImGui::InputScalar("##rssi_offset", ImGuiDataType_S8, &param0.rssi_offset, NULL, NULL, "%d");
        ImGui::Text("Max Temperature    : ");
        ImGui::SameLine();
        ImGui::InputScalar("°C##max_temp", ImGuiDataType_S16, &param0.max_temp, NULL, NULL, "%d");
        ImGui::Text("bgndrssl_ema       : ");
        ImGui::SameLine();
        ImGui::InputScalar("##bgndrssl_ema", ImGuiDataType_Float, &param0.bgndrssl_ema);
        ImGui::Text("CSP Node           : ");
        ImGui::SameLine();
        ImGui::InputScalar("##csp_node", ImGuiDataType_U8, &param0.csp_node, NULL, NULL, "%u");
        ImGui::Text("I2C                : ");
        ImGui::SameLine();
        ImGui::Checkbox("##i2c_enable", &param0.i2c_en);
        ImGui::Text("CAN                : ");
        ImGui::SameLine();
        ImGui::Checkbox("##can_enable", &param0.can_en);
        ImGui::Text("EXTPTT             : ");
        ImGui::SameLine();
        ImGui::Checkbox("##extptt_enable", &param0.extptt_en);
        ImGui::SameLine();
        ImGui::InputScalar("##max_temp", ImGuiDataType_S8, &param0.max_temp, NULL, NULL, "%d");
        ImGui::Text("I2C Address        : ");
        ImGui::SameLine();
        ImGui::InputScalar("##i2c_addr", ImGuiDataType_U8, &param0.i2c_addr, NULL, NULL, "%u");
        ImGui::Text("I2C Frequency      : ");
        ImGui::SameLine();
        ImGui::InputScalar("kHz##i2c_khz", ImGuiDataType_U16, &param0.i2c_khz, NULL, NULL, "%u");
        ImGui::Text("CAN Frequency      : ");
        ImGui::SameLine();
        ImGui::InputScalar("kHz##can_khz", ImGuiDataType_U16, &param0.can_khz, NULL, NULL, "%u");
        ImGui::Text("Reboot in          : ");
        ImGui::SameLine();
        ImGui::InputScalar("##reboot_in", ImGuiDataType_U16, &param0.reboot_in, NULL, NULL, "%u");
        ImGui::Text("TX Inhibit         : ");
        ImGui::SameLine();
        ImGui::InputScalar("##tx_inhibit", ImGuiDataType_U32, &param0.tx_inhibit, NULL, NULL, "%u");
        ImGui::Text("Store Log          : ");
        ImGui::SameLine();
        ImGui::InputScalar("##log_store", ImGuiDataType_U8, &param0.log_store, NULL, NULL, "%u");
        ImGui::Text("TX Power           : ");
        ImGui::SameLine();
        ImGui::InputScalar("##tx_power", ImGuiDataType_U8, &param0.tx_pwr, NULL, NULL, "%d");
        ImGui::Text("Maximum TX Time    : ");
        ImGui::SameLine();
        ImGui::InputScalar("second##max_tx_time", ImGuiDataType_U16, &param0.max_tx_time, NULL, NULL, "%u");
        ImGui::Text("Maximum IDLE Time  : ");
        ImGui::SameLine();
        ImGui::InputScalar("second##max_idle_time", ImGuiDataType_U16, &param0.max_idle_time, NULL, NULL, "%u");
        ImGui::Text("CSP Routing Table  : ");
        ImGui::SameLine();
        ImGui::InputText("##csp_rtable", param0.csp_rtable, sizeof(param0.csp_rtable));
        ImGui::PopItemWidth();
        ImGui::EndChild();

        if (ImGui::Button("Update", ImVec2(ImGui::GetContentRegionAvail().x * 0.25, 40)))
            pthread_create(&p_thread[0], NULL, &Load_Paramtable0, NULL);
        ImGui::SameLine();
        if (ImGui::Button("Save", ImVec2(ImGui::GetContentRegionAvail().x * 0.33, 40)))
            pthread_create(&p_thread[0], NULL, &Save_Paramtable0, NULL);
        ImGui::SameLine();
        if (ImGui::Button("Save to FRAM", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, 40)))
        {
            int8_t id = 0;
            pthread_create(&p_thread[0], NULL, &Save2FRAM, &id);
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, 40)))
        {
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        
        ImGui::EndPopup();
        return true;
    }
}

bool popup_param_table1()
{
    ImGui::OpenPopup("GS100 Parameter Table 1");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(720, 540), ImGuiCond_Appearing);
    if ( ImGui::BeginPopupModal("GS100 Parameter Table 1", NULL, ImGuiWindowFlags_AlwaysAutoResize) )
    {
        ImGui::BeginChild("##Ptable1", ImVec2(ImGui::GetContentRegionAvail().x, 650), true, mim_winflags);
        ImGui::PushItemWidth(300);
        ImGui::Text("Rx Frequency       : ");
        ImGui::SameLine();
        ImGui::InputScalar("##freq", ImGuiDataType_U32, &param1.freq, NULL, NULL, "%u");
        ImGui::Text("BaudRate           : ");
        ImGui::SameLine();
        ImGui::InputScalar("##baud", ImGuiDataType_U32, &param1.baud, NULL, NULL, "%u");
        ImGui::Text("modindex           : ");
        ImGui::SameLine();
        ImGui::InputScalar("##modeindex", ImGuiDataType_Float, &param1.modindex, NULL, NULL, "%f");
        ImGui::Text("Guard              : ");
        ImGui::SameLine();
        ImGui::InputScalar("##guard", ImGuiDataType_U16, &param1.guard, NULL, NULL, "%u");
        ImGui::Text("pllrang            : ");
        ImGui::SameLine();
        ImGui::InputScalar("##pllrang", ImGuiDataType_U8, &param1.pllrang, NULL, NULL, "%u");
        ImGui::Text("mode               : ");
        ImGui::SameLine();
        ImGui::InputScalar("##mode", ImGuiDataType_U8, &param1.mode, NULL, NULL, "%u");
        ImGui::Text("csp_hmac           : ");
        ImGui::SameLine();
        ImGui::Checkbox("##csp_hmac", &param1.csp_hmac);
        ImGui::Text("csp_rs             : ");
        ImGui::SameLine();
        ImGui::Checkbox("##csp_rs", &param1.csp_rs);
        ImGui::Text("csp_crc            : ");
        ImGui::SameLine();
        ImGui::Checkbox("##csp_crc", &param1.csp_crc);
        ImGui::Text("csp_rand           : ");
        ImGui::SameLine();
        ImGui::Checkbox("##csp_rand", &param1.csp_rand);
        ImGui::Text("AX25 Call          : ");
        ImGui::SameLine();
        ImGui::InputText("##ax25_call", param1.ax25_call, sizeof(param1.ax25_call));
        ImGui::Text("Bandwidth          : ");
        ImGui::SameLine();
        ImGui::InputScalar("##bw", ImGuiDataType_U32, &param1.bw, NULL, NULL, "%u");
        ImGui::Text("AFC Range          : ");
        ImGui::SameLine();
        ImGui::InputScalar("##afcrange", ImGuiDataType_S32, &param1.afcrange, NULL, NULL, "%d");
        ImGui::PopItemWidth();
        ImGui::EndChild();
        

        if (ImGui::Button("Update", ImVec2(ImGui::GetContentRegionAvail().x * 0.25, 40)))
            pthread_create(&p_thread[0], NULL, &Load_Paramtable1, NULL);
        ImGui::SameLine();
        if (ImGui::Button("Save", ImVec2(ImGui::GetContentRegionAvail().x * 0.33, 40)))
            pthread_create(&p_thread[0], NULL, &Save_Paramtable1, NULL);
        ImGui::SameLine();
        if (ImGui::Button("Save to FRAM", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, 40)))
        {
            int8_t id = 1;
            pthread_create(&p_thread[0], NULL, &Save2FRAM, &id);
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, 40)))
        {
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::EndPopup();
        return true;
    }
}

bool popup_param_table5()
{
    ImGui::OpenPopup("GS100 Parameter Table 5");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(720, 540), ImGuiCond_Appearing);
    if ( ImGui::BeginPopupModal("GS100 Parameter Table 5", NULL, ImGuiWindowFlags_AlwaysAutoResize) )
    {
        ImGui::BeginChild("##Ptable5", ImVec2(ImGui::GetContentRegionAvail().x, 650), true, mim_winflags);
        ImGui::PushItemWidth(300);
        ImGui::Text("Tx Frequency       : ");
        ImGui::SameLine();
        ImGui::InputScalar("##freq", ImGuiDataType_U32, &param5.freq, NULL, NULL, "%u");
        ImGui::Text("BaudRate           : ");
        ImGui::SameLine();
        ImGui::InputScalar("##baud", ImGuiDataType_U32, &param5.baud, NULL, NULL, "%u");
        ImGui::Text("modindex           : ");
        ImGui::SameLine();
        ImGui::InputScalar("##modeindex", ImGuiDataType_Float, &param5.modindex, NULL, NULL, "%f");
        ImGui::Text("Guard              : ");
        ImGui::SameLine();
        ImGui::InputScalar("##guard", ImGuiDataType_U16, &param5.guard, NULL, NULL, "%u");
        ImGui::Text("pllrang            : ");
        ImGui::SameLine();
        ImGui::InputScalar("##pllrang", ImGuiDataType_U8, &param5.pllrang, NULL, NULL, "%u");
        ImGui::Text("mode               : ");
        ImGui::SameLine();
        ImGui::InputScalar("##mode", ImGuiDataType_U8, &param5.mode, NULL, NULL, "%u");
        ImGui::Text("csp_hmac           : ");
        ImGui::SameLine();
        ImGui::Checkbox("##csp_hmac", &param5.csp_hmac);
        ImGui::Text("csp_rs             : ");
        ImGui::SameLine();
        ImGui::Checkbox("##csp_rs", &param5.csp_rs);
        ImGui::Text("csp_crc            : ");
        ImGui::SameLine();
        ImGui::Checkbox("##csp_crc", &param5.csp_crc);
        ImGui::Text("csp_rand           : ");
        ImGui::SameLine();
        ImGui::Checkbox("##csp_rand", &param5.csp_rand);
        ImGui::Text("AX25 Call          : ");
        ImGui::SameLine();
        ImGui::InputText("##ax25_call", param5.ax25_call, sizeof(param5.ax25_call));
        ImGui::Text("preamblen          : ");
        ImGui::SameLine();
        ImGui::InputScalar("##preamblen", ImGuiDataType_U8, &param5.preamblen, NULL, NULL, "%u");
        ImGui::Text("preambflags        : ");
        ImGui::SameLine();
        ImGui::InputScalar("##preambflags", ImGuiDataType_U8, &param5.preambflags, NULL, NULL, "%u");
        ImGui::Text("intfrmlen          : ");
        ImGui::SameLine();
        ImGui::InputScalar("##intfrmlen", ImGuiDataType_U8, &param5.intfrmlen, NULL, NULL, "%u");
        ImGui::Text("intfrmflags        : ");
        ImGui::SameLine();
        ImGui::InputScalar("##intfrmflags", ImGuiDataType_U8, &param5.intfrmflags, NULL, NULL, "%u");
        ImGui::Text("RSSI busy          : ");
        ImGui::SameLine();
        ImGui::InputScalar("##rssibusy", ImGuiDataType_S16, &param5.rssibusy, NULL, NULL, "%d");
        ImGui::Text("kup delay          : ");
        ImGui::SameLine();
        ImGui::InputScalar("##kup_delay", ImGuiDataType_U8, &param5.kup_delay, NULL, NULL, "%u");
        ImGui::Text("PA Level           : ");
        ImGui::SameLine();
        ImGui::InputScalar("##pa_level", ImGuiDataType_S16, &param5.pa_level, NULL, NULL, "%d");
        ImGui::PopItemWidth();
        ImGui::EndChild();

        if (ImGui::Button("Update", ImVec2(ImGui::GetContentRegionAvail().x * 0.25, 40)))
            pthread_create(&p_thread[0], NULL, &Load_Paramtable5, NULL);
        ImGui::SameLine();
        if (ImGui::Button("Save", ImVec2(ImGui::GetContentRegionAvail().x * 0.33, 40)))
            pthread_create(&p_thread[0], NULL, &Save_Paramtable5, NULL);
        ImGui::SameLine();
        if (ImGui::Button("Save to FRAM", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, 40)))
        {
            int8_t id = 5;
            pthread_create(&p_thread[0], NULL, &Save2FRAM, &id);
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, 40)))
        {
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::EndPopup();
        return true;
    }
}

bool popup_load()
{
    ImGui::OpenPopup("Load");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(500, 200), ImGuiCond_Appearing);
    if ( ImGui::BeginPopupModal("Load", NULL, ImGuiWindowFlags_AlwaysAutoResize) )
    {
        ImGui::Text("Progress : %d%c", State.loadindex * 100 / State.tleallindex, '%');
        ImGui::EndPopup();
    }
    return true;
}

bool popup_tle()
{
    ImGui::OpenPopup("TLE Manager");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(1080, 720), ImGuiCond_Appearing);
    if ( ImGui::BeginPopupModal("TLE Manager", NULL, ImGuiWindowFlags_AlwaysAutoResize) )
    {
        if (ImGui::BeginCombo("##TLECombobox", State.tleinfolistup[NowTLE]->label))
        {
            for (int TLEindex = 0; TLEindex < IM_ARRAYSIZE(State.ftplistup); TLEindex++)
            {
                if (!State.tleinfolistup[TLEindex])
                    break;
                bool SelectedTLE = (NowTLE == TLEindex);
                char Selectedlabel[64];
                strcpy(Selectedlabel, State.tleinfolistup[TLEindex]->label);
                if (ImGui::Selectable(Selectedlabel, SelectedTLE))
                {
                    NowTLE = TLEindex;
                }
                if (SelectedTLE)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        ImGui::InputText("##", SearchBuf, sizeof(SearchBuf));
        ImGui::SameLine();
        ImGui::Text("SGP4");
        ImGui::SameLine();
        ImGui::Checkbox("##sgp4checkbox", &sgp4check);
        if (ImGui::BeginTable("##TLESatelliteListup", 4, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Sortable, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y * 0.9)))
        {
            ImGui::TableSetupScrollFreeze(freeze_cols, freeze_rows);
            ImGui::TableSetupColumn("Name##tle",                ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
            ImGui::TableSetupColumn("Use##tle",                 ImGuiTableColumnFlags_NoHide, 0.0f);
            ImGui::TableSetupColumn("Next AOS##tle",            ImGuiTableColumnFlags_NoHide, 0.0f);
            ImGui::TableSetupColumn("Max Elevation##tle",       ImGuiTableColumnFlags_NoHide, 0.0f);
            ImGui::TableHeadersRow();
            tlepopupindex = 0;

            // if (ImGuiTableSortSpecs* sorts_specs = ImGui::TableGetSortSpecs())
            // {
            //     if (sorts_specs->SpecsDirty)
            //     {
            //         // If the sorting specs have changed, update the sorting state
            //         if (items.Size > 1)
            //             qsort(&items[0], (size_t)items.Size, sizeof(items[0]), MyItem::CompareWithSortSpecs);
            //         MyItem::s_current_sort_specs = NULL;
            //         sorts_specs->SpecsDirty = false;
            //         sort_column = sorts_specs->Specs[0].ColumnIndex;
            //         sort_ascending = sorts_specs->Specs[0].SortDirection == ImGuiSortDirection_Ascending;
            //     }
            // }

            while(1)
            {
                    
                if(strlen(State.Satellites[tlepopupindex]->Name())== NULL)
                    break;
                else
                {
                    if(State.Satellites[tlepopupindex]->_max_elevation[0] < 0 && sgp4check)
                    {
                        tlepopupindex += 1;
                        continue;
                    }
                    if(strlen(SearchBuf) != 0 && strstr(State.Satellites[tlepopupindex]->Name(), SearchBuf) == NULL)
                    {
                        tlepopupindex += 1;
                        continue;
                    }
                        
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%s", State.Satellites[tlepopupindex]->Name());
                    ImGui::TableSetColumnIndex(1);
                    if(State.Satellites[tlepopupindex]->use == false)
                    {
                        sprintf(SelectButtonTextBuf, "Select##TLE%d", tlepopupindex);
                        if(ImGui::Button(SelectButtonTextBuf, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
                        {
                            State.Satellites[tlepopupindex]->use = true;
                            if(State.Satellites[tlepopupindex]->cal == false)
                                State.Satellites[tlepopupindex]->Refresh(State.Satellites[tlepopupindex]->tle, State.Satellites[tlepopupindex]->obs, true, true);
                            SatelliteModelInitialize(tlepopupindex);
                        }
                    }
                    else
                    {
                        sprintf(DeleteButtonTextBuf, "Delete##TLE%d", tlepopupindex);
                        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.650f, 0.266f, 0.322f, 1.0f));
                        if(ImGui::Button(DeleteButtonTextBuf, ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
                        {
                            State.Satellites[tlepopupindex]->use = false;
                            SatelliteModelDelete(tlepopupindex);
                        }
                        ImGui::PopStyleColor();
                    }
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%d/%d %d:%d:%d",   State.Satellites[tlepopupindex]->_nextaos[0].AddHours(9).Month(), 
                                                    State.Satellites[tlepopupindex]->_nextaos[0].AddHours(9).Day(),
                                                    State.Satellites[tlepopupindex]->_nextaos[0].AddHours(9).Hour(),
                                                    State.Satellites[tlepopupindex]->_nextaos[0].AddHours(9).Minute(),
                                                    State.Satellites[tlepopupindex]->_nextaos[0].AddHours(9).Second());
                    ImGui::TableSetColumnIndex(3);
                    ImGui::Text("%.3lf°", State.Satellites[tlepopupindex]->_max_elevation[0] * RAD_TO_DEG);
                }
                tlepopupindex += 1;
            }

            
            ImGui::EndTable();
        }
        if(ImGui::Button("Load TLE", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetContentRegionAvail().y)))
        {
            
            if(DownloadTLE(State.tleinfolistup[NowTLE]->remote, State.tleinfolistup[NowTLE]->local))
            {
                State.NowTracking = false;
                
                console.AddLog("[OK]##TLE \"%s\" Download Completed.", State.tleinfolistup[NowTLE]->label);

                pthread_create(&p_thread[15], NULL, &SatelliteInitialize,  &sgp4check);
                // SatelliteInitialize(NULL);
                
            }
            else
            {
                console.AddLog("[ERROR]##TLE Download failed. Please check again.");
            }
            
        }
        ImGui::SameLine();
        if (ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y)))
        {
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::EndPopup();
        return true;
    }
}




void Initialize_CMDLabels()
{
    snprintf(ADCSCMDLabels[ADCS_NOOP_CC]                     , 64, "Noop CMD");
    snprintf(ADCSCMDLabels[ADCS_RESET_COUNTERS_CC]           , 64, "Reset Counters");
    snprintf(ADCSCMDLabels[ADCS_PROCESS_CC]                  , 64, "Process");
    snprintf(ADCSCMDLabels[ADCS_SETPARAM_SC]                 , 64, "Set Parameter SC");
    snprintf(ADCSCMDLabels[ADCS_SETPARAM_AUX]                , 64, "Set Parameter AUX");
    snprintf(ADCSCMDLabels[ADCS_SETPARAM_GAIN]               , 64, "Set Parameter Gain");
    snprintf(ADCSCMDLabels[ADCS_SETPARAM_TLE]                , 64, "Set Parameter TLE");
    snprintf(ADCSCMDLabels[ADCS_SETPARAM_TARGET]             , 64, "Set Parameter Target");

    snprintf(CICMDLabels[CI_NOOP_CC]                         , 64, "Noop CMD");
    snprintf(CICMDLabels[CI_RESET_COUNTERS_CC]               , 64, "Reset Counters");
    snprintf(CICMDLabels[CI_RESERVED_CC]                     , 64, "Reserved");

    snprintf(EPSCMDLabels[EPS_NOOP_CC]                       , 64, "Noop CMD");
    snprintf(EPSCMDLabels[EPS_RESET_COUNTERS_CC]             , 64, "Reset Counters");
    snprintf(EPSCMDLabels[EPS_RESERVED_CC]                   , 64, "Reserved");
    snprintf(EPSCMDLabels[EPS_SYSTEM_RESET_CC]               , 64, "System Reset");
    snprintf(EPSCMDLabels[EPS_HW_NOOP_CC]                    , 64, "HW Noop");
    snprintf(EPSCMDLabels[EPS_CANCEL_OPERATION_CC]           , 64, "Cancel Operation");
    snprintf(EPSCMDLabels[EPS_WATCHDOG_CC]                   , 64, "Watchdog");
    snprintf(EPSCMDLabels[EPS_CORRECT_TIME_CC]               , 64, "Correct Time");
    snprintf(EPSCMDLabels[EPS_OUTPUT_BUS_GROUP_ON_CC]        , 64, "Output Bus Group On");
    snprintf(EPSCMDLabels[EPS_OUTPUT_BUS_GROUP_OFF_CC]       , 64, "Output Bus Group Off");
    snprintf(EPSCMDLabels[EPS_OUTPUT_BUS_GROUP_STATE_CC]     , 64, "Output Bus Group State");
    snprintf(EPSCMDLabels[EPS_OUTPUT_BUS_CHANNEL_ON_CC]      , 64, "Output Bus Channel On");
    snprintf(EPSCMDLabels[EPS_OUTPUT_BUS_CHANNEL_OFF_CC]     , 64, "Output Bus Channel Off");
    snprintf(EPSCMDLabels[EPS_SWITCH_TO_NOMINAL_MODE_CC]     , 64, "Switch to Nominal Mode");
    snprintf(EPSCMDLabels[EPS_SWITCH_TO_SAFETY_MODE_CC]      , 64, "Switch to Safety Mode");
    snprintf(EPSCMDLabels[EPS_SET_CONFIG_PARAMETER_CC]       , 64, "Set Config Parameter");
    snprintf(EPSCMDLabels[EPS_RESET_CONFIG_PARAMETER_CC]     , 64, "Reset Config Parameter");
    snprintf(EPSCMDLabels[EPS_RESET_CONFIG_CC]               , 64, "Reset Config");
    snprintf(EPSCMDLabels[EPS_LOAD_CONFIG_CC]                , 64, "Load Config");
    snprintf(EPSCMDLabels[EPS_SAVE_CONFIG_CC]                , 64, "Save Config");

    snprintf(FMCMDLabels[FM_NOOP_CC]                         , 64, "Noop CMD");
    snprintf(FMCMDLabels[FM_RESET_COUNTERS_CC]               , 64, "Reset Counters");
    snprintf(FMCMDLabels[FM_RESERVED_CC]                     , 64, "Reserved");
    snprintf(FMCMDLabels[FM_RESET_FM_CC]                     , 64, "Reset FM");
    snprintf(FMCMDLabels[FM_RESET_APP_CC]                    , 64, "Reset APP");
    snprintf(FMCMDLabels[FM_RESET_PROCESSOR_CC]              , 64, "Reset Processor");
    snprintf(FMCMDLabels[FM_RESET_POWER_CC]                  , 64, "Reset Power");
    snprintf(FMCMDLabels[FM_MODE_TRANSFER_CC]                , 64, "Mode Transfer");
    snprintf(FMCMDLabels[FM_TERMINATE_EO_CC]                 , 64, "Terminate EO");
    snprintf(FMCMDLabels[FM_TERMINATE_EO_OVERRIDE_CC]        , 64, "Terminate EO Override");
    snprintf(FMCMDLabels[FM_START_OBSERVATION_CC]            , 64, "Start Observation");
    snprintf(FMCMDLabels[FM_START_COMMUNICATION_CC]          , 64, "Start Communication");
    snprintf(FMCMDLabels[FM_START_COMMISSIONING_CC]          , 64, "Start Commissioning");
    snprintf(FMCMDLabels[FM_SET_OPERATION_MODE_CC]           , 64, "Set Operation Mode");
    snprintf(FMCMDLabels[FM_SET_COMMISSIONING_PHASE_CC]      , 64, "Set Commissioning Phase");
    snprintf(FMCMDLabels[FM_INITIATE_BASELINE_CC]            , 64, "Initiate Baseline");
    snprintf(FMCMDLabels[FM_SET_SPACECRAFT_TIME_CC]          , 64, "Set Spacecraft Time");
    snprintf(FMCMDLabels[FM_DEPLOY_ANTENNA_CC]               , 64, "Deploy Antenna");
    snprintf(FMCMDLabels[FM_USE_NOMINAL_BAUDRATES_CC]        , 64, "Use Nomial Baudrates");
    snprintf(FMCMDLabels[FM_SET_ANTENNA_DEPLOY_FLAG_CC]      , 64, "Set Antenna Deploy Flag");
    snprintf(FMCMDLabels[FM_SET_DAYLIGHT_DETECTION_FLAG_CC]  , 64, "Set Daylight Detection Flag");
    snprintf(FMCMDLabels[FM_SET_COMM_MISSING_FLAG_CC]        , 64, "Set COMM Missing Flag");
    snprintf(FMCMDLabels[FM_STORE_OBSDATA_TO_CDS_CC]         , 64, "Store Obsdata to CDS");
    snprintf(FMCMDLabels[FM_RESTORE_OBSDATA_FROM_CDS_CC]     , 64, "Restore Obsdata from CDS");
    snprintf(FMCMDLabels[FM_STORE_OBSDATA_TO_SD_CC]          , 64, "Store Obsdata to SD");
    snprintf(FMCMDLabels[FM_RESTORE_OBSDATA_FROM_SD_CC]      , 64, "Restore Obsdata from SD");
    
    snprintf(GPSCMDLabels[GPS_NOOP_CC]                       , 64, "Noop CMD");
    snprintf(GPSCMDLabels[GPS_RESET_COUNTERS_CC]             , 64, "Reset Counters");
    snprintf(GPSCMDLabels[GPS_RESERVED_CC]                   , 64, "Reserved");
    snprintf(GPSCMDLabels[GPS_RESET_APP_CC]                  , 64, "Reset App");
    snprintf(GPSCMDLabels[GPS_RESET_HW_CC]                   , 64, "Reset HW");
    snprintf(GPSCMDLabels[GPS_CLEAR_LOGS_CC]                 , 64, "Clear Logs");
    snprintf(GPSCMDLabels[GPS_ENABLE_TIME_TONE_CC]           , 64, "Enable Time Tone");
    snprintf(GPSCMDLabels[GPS_DISABLE_TIME_TONE_CC]          , 64, "Disable Time Tone");
    snprintf(GPSCMDLabels[GPS_LOG_ONCE_CC]                   , 64, "Log Once");
    snprintf(GPSCMDLabels[GPS_LOG_ONTIME_CC]                 , 64, "Log Ontime");
    snprintf(GPSCMDLabels[GPS_LOG_ONCHANGE_CC]               , 64, "Log Onchange");
    snprintf(GPSCMDLabels[GPS_LOG_REQUEST_DFT_CC]            , 64, "Log Request DFT");
    snprintf(GPSCMDLabels[GPS_SET_IF_MODE_CC]                , 64, "Set If Mode");

    snprintf(MTQCMDLabels[MTQ_NOOP_CC]                       , 64, "Noop CMD");
    snprintf(MTQCMDLabels[MTQ_RESET_COUNTERS_CC]             , 64, "Reset Counters");
    snprintf(MTQCMDLabels[MTQ_RESERVED_CC]                   , 64, "Reserved");
    snprintf(MTQCMDLabels[MTQ_RESET]                         , 64, "Reset");
    snprintf(MTQCMDLabels[MTQ_RESET_COMP]                    , 64, "Reset Comp");
    snprintf(MTQCMDLabels[MTQ_SET_DUTY]                      , 64, "Set Duty");
    snprintf(MTQCMDLabels[MTQ_ENABLE]                        , 64, "Enable");
    snprintf(MTQCMDLabels[MTQ_DISABLE]                       , 64, "Disable");
    snprintf(MTQCMDLabels[MTQ_SET_PERIOD]                    , 64, "Set Period");
    snprintf(MTQCMDLabels[MTQ_SET_POLARITY]                  , 64, "Set Polarity");

    snprintf(PAYCMDLabels[PAY_NOOP_CC]                       , 64, "Noop CMD");
    snprintf(PAYCMDLabels[PAY_RESET_COUNTERS_CC]             , 64, "Reset Counters");
    snprintf(PAYCMDLabels[PAY_PROCESS_CC]                    , 64, "Process");
    snprintf(PAYCMDLabels[PAY_CAM_FIND_CC]                   , 64, "Cam Find");
    snprintf(PAYCMDLabels[PAY_CAM_CONNECT_CC]                , 64, "Cam Connect");
    snprintf(PAYCMDLabels[PAY_CAM_START_OPERATION_CC]        , 64, "Cam Start Operation");
    snprintf(PAYCMDLabels[PAY_CAM_STOP_OPERATION_CC]         , 64, "Cam Stop Operation");
    snprintf(PAYCMDLabels[PAY_CAM_DOWNLOAD_NEW_IMG_CC]       , 64, "Cam Download New Image");
    snprintf(PAYCMDLabels[PAY_CAM_DOWNLOAD_OLD_IMG_CC]       , 64, "Cam Download old Image");
    snprintf(PAYCMDLabels[PAY_CAM_SEND_NOARG_CC]             , 64, "Cam Send No Arg");
    snprintf(PAYCMDLabels[PAY_CAM_SET_EXPOSURE_CC]           , 64, "Cam Set Explosure");
    snprintf(PAYCMDLabels[PAY_CAM_SET_SCPD_CC]               , 64, "Cam Set SCPD");
    snprintf(PAYCMDLabels[PAY_BBE_SHUTDOWN_CC]               , 64, "BBE Shutdown");
    snprintf(PAYCMDLabels[PAY_INIT_DEVICE_CC]                , 64, "Init Device");

    snprintf(RWACMDLabels[RWA_NOOP_CC]                       , 64, "Noop CMD");
    snprintf(RWACMDLabels[RWA_RESET_COUNTERS_CC]             , 64, "Reset Counters");
    snprintf(RWACMDLabels[RWA_RESERVED_CC]                   , 64, "Reserved");
    snprintf(RWACMDLabels[RWA_RESET_ALL_CC]                  , 64, "Reset All");
    snprintf(RWACMDLabels[RWA_RESET_WHEEL_CC]                , 64, "Reset Wheel");
    snprintf(RWACMDLabels[RWA_SET_WHEEL_REFERENCE_SPEED_CC]  , 64, "Set Wheel Reference Speed");
    snprintf(RWACMDLabels[RWA_SET_WHEEL_REFERENCE_SPEED_ALL_AXIS_CC] , 64, "Set Wheel Reference Speed All Axis");
    snprintf(RWACMDLabels[RWA_SET_WHEEL_COMMANDED_TORQUE_CC] , 64, "Set Wheel Commanded Torque");
    snprintf(RWACMDLabels[RWA_SET_MOTOR_POWER_STATE_CC]      , 64, "Set Motor Power State");
    snprintf(RWACMDLabels[RWA_SET_ENCODER_POWER_STATE_CC]    , 64, "Set Encoder Power State");
    snprintf(RWACMDLabels[RWA_SET_HALL_POWER_STATE_CC]       , 64, "Set Hall Power State");
    snprintf(RWACMDLabels[RWA_SET_CONTROL_MODE_CC]           , 64, "Set Control Mode");
    snprintf(RWACMDLabels[RWA_SET_BACKUP_WHEEL_MODE_CC]      , 64, "Set Backup Wheel Mode");
    snprintf(RWACMDLabels[RWA_SET_CLEAR_ERRORS_CC]           , 64, "Set Clear Errors");
    snprintf(RWACMDLabels[RWA_SET_PWM_GAIN_CC]               , 64, "Set PWM Gain");
    snprintf(RWACMDLabels[RWA_SET_MAIN_GAIN_CC]              , 64, "Set Main Gain");
    snprintf(RWACMDLabels[RWA_SET_BACKUP_GAIN_CC]            , 64, "Set Backup Gain");

    snprintf(SNSRCMDLabels[SNSR_NOOP_CC]                     , 64, "Noop CMD");
    snprintf(SNSRCMDLabels[SNSR_RESET_COUNTERS_CC]           , 64, "Reset Counters");
    snprintf(SNSRCMDLabels[SNSR_RESERVED_CC]                 , 64, "Reserved");
    snprintf(SNSRCMDLabels[SNSR_SUN_ALARM_OFF_CC]            , 64, "Sun Alarm Off");
    snprintf(SNSRCMDLabels[SNSR_STT_BOOT_CC]                 , 64, "STT Boot");
    snprintf(SNSRCMDLabels[SNSR_STT_PING_CC]                 , 64, "STT Ping");
    snprintf(SNSRCMDLabels[SNSR_STT_UNLOCK_CC]               , 64, "STT Unlock");
    snprintf(SNSRCMDLabels[SNSR_STT_REBOOT_CC]               , 64, "STT Reboot");
    snprintf(SNSRCMDLabels[SNSR_STT_SET_PARAMS_DFT_CC]       , 64, "STT Set Params DFT");
    snprintf(SNSRCMDLabels[SNSR_STT_SET_PARAM_CC]            , 64, "STT Set Params");
    snprintf(SNSRCMDLabels[SNSR_STT_SEND_RS485_CC]           , 64, "STT Send RS485");
    snprintf(SNSRCMDLabels[SNSR_STT_INIT_DEVICE_CC]          , 64, "STT Init Device");
    snprintf(SNSRCMDLabels[SNSR_MMT_RESET_CC]                , 64, "MMT Reset");
    snprintf(SNSRCMDLabels[SNSR_MMT_SET_INTERNAL_CONTROL0_CC], 64, "MMT Set Internal Control 0");
    snprintf(SNSRCMDLabels[SNSR_MMT_SET_INTERNAL_CONTROL2_CC], 64, "MMT Set Internal Control 2");
    snprintf(SNSRCMDLabels[SNSR_MMT_WRITE_TO_REGISTER_CC]    , 64, "MMT Write to Register");
    snprintf(SNSRCMDLabels[SNSR_IMU_RESET_CC]                , 64, "IMU Reset");
    snprintf(SNSRCMDLabels[SNSR_IMU_SET_GYRO_OFFSET_CC]      , 64, "IMU Set Gyro Offset");
    snprintf(SNSRCMDLabels[SNSR_IMU_SET_CONFIGURATION_CC]    , 64, "IMU Set Configuration");
    snprintf(SNSRCMDLabels[SNSR_IMU_SET_GYRO_CONFIGURATION_CC], 64, "IMU Set Gyro Configuration");
    snprintf(SNSRCMDLabels[SNSR_IMU_SET_ACCEL_CONFIGURATION_CC], 64, "IMU Set Acceleration Configuration");
    snprintf(SNSRCMDLabels[SNSR_IMU_SET_ACCEL_CONFIGURATION2_CC], 64, "IMU Set Acceleration Configuration 2");
    snprintf(SNSRCMDLabels[SNSR_IMU_SET_POWER_MANAGEMENT_CC] , 64, "IMU Set Power Management");
    snprintf(SNSRCMDLabels[SNSR_IMU_WRITE_TO_REGISTER_CC]    , 64, "IMU Write to Register");
    snprintf(SNSRCMDLabels[SNSR_IMU_ISOLATE_CC]              , 64, "IMU Isolate");
    snprintf(SNSRCMDLabels[SNSR_IMU_RESTORE_CC]              , 64, "IMU Restore");
    snprintf(SNSRCMDLabels[SNSR_MMT_ISOLATE_CC]              , 64, "MMT Isolate");
    snprintf(SNSRCMDLabels[SNSR_MMT_RESTORE_CC]              , 64, "MMT Restore");
    snprintf(SNSRCMDLabels[SNSR_FSS_ISOLATE_CC]              , 64, "FSS Isolate");
    snprintf(SNSRCMDLabels[SNSR_FSS_RESTORE_CC]              , 64, "FSS Restore");
    snprintf(SNSRCMDLabels[SNSR_CSS_ISOLATE_CC]              , 64, "CSS Isolate");
    snprintf(SNSRCMDLabels[SNSR_CSS_RESTORE_CC]              , 64, "CSS Restore");
    snprintf(SNSRCMDLabels[SNSR_STT_ISOLATE_CC]              , 64, "STT Isolate");
    snprintf(SNSRCMDLabels[SNSR_STT_RESTORE_CC]              , 64, "STT Restore");
    snprintf(SNSRCMDLabels[SNSR_IMU_WHO_I_AM_CC]             , 64, "IMU Who I Am");
    snprintf(SNSRCMDLabels[SNSR_MMT_GET_PRODUCT_ID_CC]       , 64, "MMT Get Product ID");

    snprintf(STXCMDLabels[STX_NOOP_CC]                       , 64, "Noop CMD");
    snprintf(STXCMDLabels[STX_RESET_COUNTERS_CC]             , 64, "Reset Counters");
    snprintf(STXCMDLabels[STX_PROCESS_CC]                    , 64, "Process");
    snprintf(STXCMDLabels[STX_RESET_CC]                      , 64, "Reset");
    snprintf(STXCMDLabels[STX_SET_CONTROL_MODE_CC]           , 64, "Set Control Mode");
    snprintf(STXCMDLabels[STX_SET_ENCODER_RATE_CC]           , 64, "Set Encoder Rate");
    snprintf(STXCMDLabels[STX_SET_PA_POWER_CC]               , 64, "Set PA Power");
    snprintf(STXCMDLabels[STX_SET_SYNTH_OFFSET_CC]           , 64, "Set Synth Offset");
    snprintf(STXCMDLabels[STX_TRANSMIT_READY_CC]             , 64, "Transmit Ready");
    snprintf(STXCMDLabels[STX_TRANSMIT_DATA_CC]              , 64, "Transmit Data");
    snprintf(STXCMDLabels[STX_TRANSMIT_FILE_CC]              , 64, "Transmit File");
    snprintf(STXCMDLabels[STX_TRANSMIT_FILE_LONG_PATH_CC]    , 64, "Transmit File Log Path");
    snprintf(STXCMDLabels[STX_TRANSMIT_END_CC]               , 64, "Transmit End");

    snprintf(TOCMDLabels[TO_NOOP_CC]                         , 64, "Noop CMD");
    snprintf(TOCMDLabels[TO_RESET_COUNTERS_CC]               , 64, "Reset Counters");
    snprintf(TOCMDLabels[TO_PROCESS_CC]                      , 64, "Process");
    snprintf(TOCMDLabels[TO_ENABLE_BEACON_CC]                , 64, "Enable Beacon");
    snprintf(TOCMDLabels[TO_DISABLE_BEACON_CC]               , 64, "Disable Beacon");
    snprintf(TOCMDLabels[TO_DOWNLINK_QUERY_REPLY_CC]         , 64, "Downlink Query Reply");
    snprintf(TOCMDLabels[TO_ENABLE_CC]                       , 64, "Enable");

    snprintf(UTRXCMDLabels[UTRX_NOOP_CC]                     , 64, "Noop CMD");
    snprintf(UTRXCMDLabels[UTRX_RESET_COUNTERS_CC]           , 64, "Reset Counters");
    snprintf(UTRXCMDLabels[UTRX_RESERVED_CC]                 , 64, "Reserved");
    snprintf(UTRXCMDLabels[UTRX_REBOOT_CC]                   , 64, "Reboot");
    snprintf(UTRXCMDLabels[UTRX_SET_TX_FREQ_CC]              , 64, "Set TX Frequency");
    snprintf(UTRXCMDLabels[UTRX_SET_TX_BAUD_CC]              , 64, "Set TX Baudrate");
    snprintf(UTRXCMDLabels[UTRX_SET_TX_MODIND_CC]            , 64, "Set TX Mod Index");
    snprintf(UTRXCMDLabels[UTRX_SET_TX_MODE_CC]              , 64, "Set TX Mode");
    snprintf(UTRXCMDLabels[UTRX_SET_RX_FREQ_CC]              , 64, "Set RX Frequency");
    snprintf(UTRXCMDLabels[UTRX_SET_RX_BAUD_CC]              , 64, "Set RX Baudrate");
    snprintf(UTRXCMDLabels[UTRX_SET_RX_MODIND_CC]            , 64, "Set RX Mode Index");
    snprintf(UTRXCMDLabels[UTRX_SET_RX_MODE_CC]              , 64, "Set RX Mode");
    snprintf(UTRXCMDLabels[UTRX_SET_RX_BW_CC]                , 64, "Set RX Bandwidth");
    snprintf(UTRXCMDLabels[UTRX_STUB_PING_CC]                , 64, "Stup Ping");

    snprintf(TSCMDLabels[TS_NOOP_CC]                         , 64, "Noop CMD");
    snprintf(TSCMDLabels[TS_RESET_COUNTERS_CC]               , 64, "Reset Counters");
    snprintf(TSCMDLabels[TS_PROCESS_CC]                      , 64, "Process");
    snprintf(TSCMDLabels[TS_INSERT_SCHEDULE_ENTRY_CC]        , 64, "Insert Schedule Entry");
    snprintf(TSCMDLabels[TS_CLEAR_SCHEDULE_ENTRY_CC]         , 64, "Clear Schedule Entry");
    snprintf(TSCMDLabels[TS_CLEAR_SCHEDULE_GROUP_CC]         , 64, "Clear Schedule Group");
    snprintf(TSCMDLabels[TS_CLEAR_ALL_SCHEDULE_CC]           , 64, "Clear All Schedule");

    snprintf(ECMCMDLabels[ECM_GET_HK_AVG_CC]                 , 64, "Get HK AVG");
    snprintf(ECMCMDLabels[ECM_GET_SYSTEM_STATUS_CC]          , 64, "Get System Status");
    snprintf(ECMCMDLabels[ECM_GET_OCF_STATE_CC]              , 64, "Get OCF State");
    snprintf(ECMCMDLabels[ECM_READ_CC]                       , 64, "Read");

    snprintf(ESCMDLabels[CFE_ES_RESTART_CC]                  , 64, "Restart");
    snprintf(ESCMDLabels[CFE_ES_RESTART_APP_CC]              , 64, "Restart App");
    snprintf(ESCMDLabels[CFE_ES_STOP_APP_CC]                 , 64, "Stop App");
    snprintf(ESCMDLabels[CFE_ES_START_APP_CC]                , 64, "Start App");


    
    snprintf(SCHCMDLabels[SCH_ENABLE_CC]                     , 64, "SCH Enable");
    snprintf(SCHCMDLabels[SCH_DISABLE_CC]                    , 64, "SCH Disable");
}

int CMDDataGenerator(uint32_t msgid, uint16_t fnccode, void *Requested, size_t RequestedSize) 
{
    ImGui::Checkbox("Checksum", &ChecksumState);
    ImGui::SameLine();
	ImGui::Checkbox("Scheduler", &SchedulerState);
	if(SchedulerState)
	{
        ChecksumState = true;
		ImGui::Text("Execution Time     : ");
		ImGui::SameLine();
		ImGui::InputScalar("##ExecutionTime", ImGuiDataType_U32, &ExecutionTimeBuf, NULL, NULL, "%u");
		ImGui::Text("Execution Window   : ");
		ImGui::SameLine();
		ImGui::InputScalar("##ExecutionWindow", ImGuiDataType_U32, &ExecutionWindowBuf, NULL, NULL, "%u");
		ImGui::Text("Entry ID           : ");
		ImGui::SameLine();
		ImGui::InputScalar("##EntryID", ImGuiDataType_U16, &EntryIDBuf, NULL, NULL, "%u");
		ImGui::Text("GroupID            : ");
		ImGui::SameLine();
		ImGui::InputScalar("##GroupID", ImGuiDataType_U16, &GroupIDBuf, NULL, NULL, "%u");
	}
	if(ImGui::Button("Generate", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetFontSize() * 1.5)))
	{
		SatCMD[NowCMD] = new CmdGenerator_GS();
        console.AddLog("[DEBUG]##Generate : %u, %u, %d\n", msgid, fnccode, RequestedSize);
        if(RequestedSize == sizeof(CFE_MSG_CommandHeader))
            SatCMD[NowCMD]->GenerateCmdHeader(msgid, fnccode, RequestedSize, NULL);
        else
		    SatCMD[NowCMD]->GenerateCmdHeader(msgid, fnccode, RequestedSize, Requested + 8);
		if(SchedulerState)
        {
            SatCMD[NowCMD]->Scheduled = true;
            SatCMD[NowCMD]->Scheduling(ExecutionTimeBuf, ExecutionWindowBuf, EntryIDBuf, GroupIDBuf);
        }
        else if(!ChecksumState)
        {
            SatCMD[NowCMD]->Checksum = false;
        }
			
		NowCMD ++;

		console.AddLog("Generated Command Message.");
        memset(Requested, 0, RequestedSize);
		uint32_t ExecutionTimeBuf = 0;
		uint32_t ExecutionWindowBuf = 0;
		uint16_t EntryIDBuf = 0;
		uint16_t GroupIDBuf = 0;
        ChecksumState = true;
        SchedulerState = false;
	}
	ImGui::SameLine();
	if(ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
	{
		memset(Requested, 0, RequestedSize);
		return 0;
	}
    return 1;
	
}


bool popup_cmd()
{
    ImGui::OpenPopup("CMD Manager");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(720, 540), ImGuiCond_Appearing);
    if ( ImGui::BeginPopupModal("CMD Manager", NULL, ImGuiWindowFlags_AlwaysAutoResize) )
    {
        ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
        if (ImGui::BeginTabBar("CMDTabBar", tab_bar_flags))
        {
            if (ImGui::BeginTabItem("ADCS"))
            {
                if(gen_msgid != ADCS_CMD_MID)
                {
                    gen_msgid = ADCS_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, ADCSCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(ADCSCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(ADCSCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    
                    ImGui::EndCombo();
                }
                switch (gen_fnccode)
                {
                case ADCS_SETPARAM_SC : {
                    ImGui::PushItemWidth(200);
                    ImGui::Text("Contact to Manager. This packet is too long.");
                    ImGui::Text("Mass : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mass", &command->adcs_setparamsccmd.Params.mass);
                    ImGui::Text("MOI : ");
                    ImGui::Text("[0][0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##moi[0][0]", &command->adcs_setparamsccmd.Params.MOI[0][0]);
                    ImGui::SameLine();
                    ImGui::Text("[0][1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##moi[0][1]", &command->adcs_setparamsccmd.Params.MOI[0][1]);
                    ImGui::SameLine();
                    ImGui::Text("[0][2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##moi[0][2]", &command->adcs_setparamsccmd.Params.MOI[0][2]);
                    ImGui::Text("[1][0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##moi[1][0]", &command->adcs_setparamsccmd.Params.MOI[1][0]);
                    ImGui::SameLine();
                    ImGui::Text("[1][1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##moi[1][1]", &command->adcs_setparamsccmd.Params.MOI[1][1]);
                    ImGui::SameLine();
                    ImGui::Text("[1][2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##moi[1][2]", &command->adcs_setparamsccmd.Params.MOI[1][2]);
                    ImGui::Text("[2][0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##moi[2][0]", &command->adcs_setparamsccmd.Params.MOI[2][0]);
                    ImGui::SameLine();
                    ImGui::Text("[2][1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##moi[2][1]", &command->adcs_setparamsccmd.Params.MOI[2][1]);
                    ImGui::SameLine();
                    ImGui::Text("[2][2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##moi[2][2]", &command->adcs_setparamsccmd.Params.MOI[2][2]);
                    ImGui::Text("Inv MOI : ");
                    ImGui::Text("[0][0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##invmoi[0][0]", &command->adcs_setparamsccmd.Params.inv_MOI[0][0]);
                    ImGui::SameLine();
                    ImGui::Text("[0][1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##invmoi[0][1]", &command->adcs_setparamsccmd.Params.inv_MOI[0][1]);
                    ImGui::SameLine();
                    ImGui::Text("[0][2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##invmoi[0][2]", &command->adcs_setparamsccmd.Params.inv_MOI[0][2]);
                    ImGui::Text("[1][0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##invmoi[1][0]", &command->adcs_setparamsccmd.Params.inv_MOI[1][0]);
                    ImGui::SameLine();
                    ImGui::Text("[1][1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##invmoi[1][1]", &command->adcs_setparamsccmd.Params.inv_MOI[1][1]);
                    ImGui::SameLine();
                    ImGui::Text("[1][2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##invmoi[1][2]", &command->adcs_setparamsccmd.Params.inv_MOI[1][2]);
                    ImGui::Text("[2][0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##invmoi[2][0]", &command->adcs_setparamsccmd.Params.inv_MOI[2][0]);
                    ImGui::SameLine();
                    ImGui::Text("[2][1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##invmoi[2][1]", &command->adcs_setparamsccmd.Params.inv_MOI[2][1]);
                    ImGui::SameLine();
                    ImGui::Text("[2][2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##invmoi[2][2]", &command->adcs_setparamsccmd.Params.inv_MOI[2][2]);
                    ImGui::Text("CSS SF : ");
                    ImGui::Text("[0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##css_sf[0]", &command->adcs_setparamsccmd.Params.css_sf[0]);
                    ImGui::SameLine();
                    ImGui::Text("[1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##css_sf[0]", &command->adcs_setparamsccmd.Params.css_sf[1]);
                    ImGui::SameLine();
                    ImGui::Text("[2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##css_sf[0]", &command->adcs_setparamsccmd.Params.css_sf[2]);
                    ImGui::Text("[3]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##css_sf[0]", &command->adcs_setparamsccmd.Params.css_sf[3]);
                    ImGui::SameLine();
                    ImGui::Text("[4]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##css_sf[0]", &command->adcs_setparamsccmd.Params.css_sf[4]);
                    ImGui::Text("MMT_A : ");
                    ImGui::Text("[0][0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_A[0][0]", &command->adcs_setparamsccmd.Params.mmt_A[0][0]);
                    ImGui::SameLine();
                    ImGui::Text("[0][1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_A[0][1]", &command->adcs_setparamsccmd.Params.mmt_A[0][1]);
                    ImGui::SameLine();
                    ImGui::Text("[0][2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_A[0][2]", &command->adcs_setparamsccmd.Params.mmt_A[0][2]);
                    ImGui::Text("[1][0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_A[1][0]", &command->adcs_setparamsccmd.Params.mmt_A[1][0]);
                    ImGui::SameLine();
                    ImGui::Text("[1][1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_A[1][1]", &command->adcs_setparamsccmd.Params.mmt_A[1][1]);
                    ImGui::SameLine();
                    ImGui::Text("[1][2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_A[1][2]", &command->adcs_setparamsccmd.Params.mmt_A[1][2]);
                    ImGui::Text("[2][0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_A[2][0]", &command->adcs_setparamsccmd.Params.mmt_A[2][0]);
                    ImGui::SameLine();
                    ImGui::Text("[2][1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_A[2][1]", &command->adcs_setparamsccmd.Params.mmt_A[2][1]);
                    ImGui::SameLine();
                    ImGui::Text("[2][2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_A[2][2]", &command->adcs_setparamsccmd.Params.mmt_A[2][2]);
                    ImGui::Text("MMT bias : ");
                    ImGui::Text("[0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_bias[0]", &command->adcs_setparamsccmd.Params.mmt_bias[0]);
                    ImGui::SameLine();
                    ImGui::Text("[1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_bias[0]", &command->adcs_setparamsccmd.Params.mmt_bias[1]);
                    ImGui::SameLine();
                    ImGui::Text("[2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mmt_bias[0]", &command->adcs_setparamsccmd.Params.mmt_bias[2]);
                    ImGui::Text("Gyro bias A : ");
                    ImGui::Text("[0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##gyro_bias_a[0]", &command->adcs_setparamsccmd.Params.gyro_bias_a[0]);
                    ImGui::SameLine();
                    ImGui::Text("[1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##gyro_bias_a[0]", &command->adcs_setparamsccmd.Params.gyro_bias_a[1]);
                    ImGui::SameLine();
                    ImGui::Text("[2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##gyro_bias_a[0]", &command->adcs_setparamsccmd.Params.gyro_bias_a[2]);
                    ImGui::Text("Gyro bias B : ");
                    ImGui::Text("[0]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##gyro_bias_b[0]", &command->adcs_setparamsccmd.Params.gyro_bias_b[0]);
                    ImGui::SameLine();
                    ImGui::Text("[1]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##gyro_bias_b[0]", &command->adcs_setparamsccmd.Params.gyro_bias_b[1]);
                    ImGui::SameLine();
                    ImGui::Text("[2]");
                    ImGui::SameLine();
                    ImGui::InputDouble("##gyro_bias_b[0]", &command->adcs_setparamsccmd.Params.gyro_bias_b[2]);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->adcs_setparamsccmd, sizeof(ADCS_SetParamScCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                    }
                case(ADCS_SETPARAM_TLE) : {
                    ImGui::Text("Epoch Year : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##epochyear", ImGuiDataType_U32, &command->adcs_setparamtlecmd.Params.epochYear, NULL, NULL, "%u");
                    ImGui::Text("Epoch Day : ");
                    ImGui::SameLine();
                    ImGui::InputFloat("##epochday", &command->adcs_setparamtlecmd.Params.epochDay);
                    ImGui::Text("Bstar : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##bstar", &command->adcs_setparamtlecmd.Params.Bstar);
                    ImGui::Text("Inclination : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##bstar", &command->adcs_setparamtlecmd.Params.INC);
                    ImGui::Text("RAAN : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##bstar", &command->adcs_setparamtlecmd.Params.RAAN);
                    ImGui::Text("Eccentricity : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##bstar", &command->adcs_setparamtlecmd.Params.ECC);
                    ImGui::Text("AOP : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##bstar", &command->adcs_setparamtlecmd.Params.AOP);
                    ImGui::Text("Mean Anomoly : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##bstar", &command->adcs_setparamtlecmd.Params.MA);
                    ImGui::Text("Mean Motion : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##bstar", &command->adcs_setparamtlecmd.Params.MM);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->adcs_setparamtlecmd, sizeof(ADCS_SetParamTleCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(ADCS_SETPARAM_AUX) : {
                    ImGui::Text("mu_E: ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##mu_E", &command->adcs_setparamauxcmd.Params.mu_E);
                    ImGui::Text("R_E : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##R_E", &command->adcs_setparamauxcmd.Params.R_E);
                    ImGui::Text("Coef_J2 : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##coef_J2", &command->adcs_setparamauxcmd.Params.coef_J2);
                    ImGui::Text("w_E : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##w_E", &command->adcs_setparamauxcmd.Params.w_E);
                    ImGui::Text("wn : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##wn", &command->adcs_setparamauxcmd.Params.wn);
                    ImGui::Text("tstp : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##tstp", &command->adcs_setparamauxcmd.Params.tstp);
                    ImGui::Text("quit_TRIAD : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##quit_TRIAD", ImGuiDataType_U32, &command->adcs_setparamauxcmd.Params.quit_TRIAD, NULL, NULL, "%u");
                    ImGui::Text("num_AD_loop : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##num_AD_loop", ImGuiDataType_U8, &command->adcs_setparamauxcmd.Params.num_AD_loop, NULL, NULL, "%u");
                    ImGui::Text("num_AC_loop : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##num_AC_loop", ImGuiDataType_U8, &command->adcs_setparamauxcmd.Params.num_AC_loop, NULL, NULL, "%u");
                    ImGui::Text("option_prg_orb_est : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##option_prg_orb_est", ImGuiDataType_U8, &command->adcs_setparamauxcmd.Params.option_prg_orb_est, NULL, NULL, "%u");
                    ImGui::Text("option_gyroless : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##option_gyroless", ImGuiDataType_U8, &command->adcs_setparamauxcmd.Params.option_gyroless, NULL, NULL, "%u");
                    ImGui::Text("option_ignoreSTT : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##option_ignoreSTT", ImGuiDataType_U8, &command->adcs_setparamauxcmd.Params.option_ignoreSTT, NULL, NULL, "%u");
                    ImGui::Text("option_ignoreCSS : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##option_ignoreCSS", ImGuiDataType_U8, &command->adcs_setparamauxcmd.Params.option_ignoreCSS, NULL, NULL, "%u");
                    ImGui::Text("est_P_atd_STT_initial : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##est_P_atd_initial", &command->adcs_setparamauxcmd.Params.est_P_atd_STT_initial);
                    ImGui::Text("GS_sband[0] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##GS_sband[0]", &command->adcs_setparamauxcmd.Params.GS_sband[0]);
                    ImGui::Text("GS_sband[1] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##GS_sband[1]", &command->adcs_setparamauxcmd.Params.GS_sband[1]);
                    ImGui::Text("GS_sband[2] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##GS_sband[2]", &command->adcs_setparamauxcmd.Params.GS_sband[2]);
                    ImGui::Text("GS_uhf[0] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##GS_uhf[0]", &command->adcs_setparamauxcmd.Params.GS_uhf[0]);
                    ImGui::Text("GS_uhf[1] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##GS_uhf[1]", &command->adcs_setparamauxcmd.Params.GS_uhf[1]);
                    ImGui::Text("GS_uhf[2] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##GS_uhf[2]", &command->adcs_setparamauxcmd.Params.GS_uhf[2]);
                    ImGui::Text("wrpm_WHLdot_dump : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##wrpm_WHLdot_dump", &command->adcs_setparamauxcmd.Params.wrpm_WHLdot_dump);
                    ImGui::Text("wrpm_WHL_dump_crit : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##wrpm_WHL_dump_crit", &command->adcs_setparamauxcmd.Params.wrpm_WHL_dump_crit);
                    ImGui::Text("wrpm_WHL_dump_goal : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##wrpm_WHL_dump_goal", &command->adcs_setparamauxcmd.Params.wrpm_WHL_dump_goal);
                    ImGui::Text("w_stanby_detumbling_start : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##w_stanby_detumbling_start", &command->adcs_setparamauxcmd.Params.w_standby_detumbling_start);
                    ImGui::Text("w_stanby_detumbling_goal : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##w_stanby_detumblig_goal", &command->adcs_setparamauxcmd.Params.w_standby_detumbling_goal);
                    ImGui::Text("w_stanby_forcedSpin_start : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##w_stanby_forcedSpin_start", &command->adcs_setparamauxcmd.Params.w_standby_forcedSpin_start);
                    ImGui::Text("angle_stanby_forcedSpin_start : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##angle_stanby_forcedSpin_start", &command->adcs_setparamauxcmd.Params.angle_standby_forcedSpin_start);
                    ImGui::Text("M_stanby_forcedSpin[0] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##M_stanby_forcedSpin[0]", &command->adcs_setparamauxcmd.Params.M_standby_forcedSpin[0]);
                    ImGui::Text("M_stanby_forcedSpin[1] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##M_stanby_forcedSpin[1]", &command->adcs_setparamauxcmd.Params.M_standby_forcedSpin[1]);
                    ImGui::Text("M_stanby_forcedSpin[2] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##M_stanby_forcedSpin[2]", &command->adcs_setparamauxcmd.Params.M_standby_forcedSpin[2]);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->adcs_setparamauxcmd, sizeof(ADCS_SetParamAuxCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(ADCS_SETPARAM_TARGET) : {
                    ImGui::Text("r_Target[0] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##r_Target[0]", &command->adcs_setparamtargetcmd.Params.r_Target[0]);
                    ImGui::Text("r_Target[1] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##r_Target[1]", &command->adcs_setparamtargetcmd.Params.r_Target[1]);
                    ImGui::Text("r_Target[2] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##r_Target[2]", &command->adcs_setparamtargetcmd.Params.r_Target[2]);
                    ImGui::Text("q_Target[0] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##q_Target[0]", &command->adcs_setparamtargetcmd.Params.q_Target[0]);
                    ImGui::Text("q_Target[1] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##q_Target[1]", &command->adcs_setparamtargetcmd.Params.q_Target[1]);
                    ImGui::Text("q_Target[2] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##q_Target[2]", &command->adcs_setparamtargetcmd.Params.q_Target[2]);
                    ImGui::Text("q_Target[3] : ");
                    ImGui::SameLine();
                    ImGui::InputDouble("##q_Target[3]", &command->adcs_setparamtargetcmd.Params.q_Target[3]);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->adcs_setparamtargetcmd, sizeof(ADCS_SetParamTargetCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(ADCS_SETPARAM_GAIN) : {
                    ImGui::Text("WhichGain : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##adcs_setparamgain_whichgain", ImGuiDataType_U8, &command->adcs_setparamgaincmd.Payload.WhichGain, NULL, NULL, "%u");
                    ImGui::Text("Mode : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##adcs_setparamgain_Mode", ImGuiDataType_U8, &command->adcs_setparamgaincmd.Payload.Mode, NULL, NULL, "%u");
                    ImGui::Text("Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##adcs_setparamgain_num", ImGuiDataType_U16, &command->adcs_setparamgaincmd.Payload.Num, NULL, NULL, "%u");
                    for(int i = 0; i < 31; i++)
                    {
                        ImGui::Text("Gain[%d]", i);
                        ImGui::SameLine();
                        sprintf(repeated_label, "##adcs_setparamgain_gain_%d", i);
                        ImGui::InputDouble(repeated_label, &command->adcs_setparamgaincmd.Payload.Gain[i], NULL, NULL);
                        if(i % 4 != 3)
                            ImGui::SameLine();
                    }
                    ImGui::Text("");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->adcs_setparamgaincmd, sizeof(ADCS_SetParamGainCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default: {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->adcs_noargscmd, sizeof(ADCS_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("GPS"))
            {
                if(gen_msgid != GPS_CMD_MID)
                {
                    gen_msgid = GPS_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, GPSCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(GPSCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(GPSCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(GPS_LOG_ONCE_CC) :
                {
                    ImGui::Text("Msg ID : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##logoncemsgid", ImGuiDataType_S16, &command->gps_logoncecmd.MsgId, NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->gps_logoncecmd, sizeof(GPS_LogOnceCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(GPS_LOG_ONTIME_CC) :
                {
                    ImGui::Text("Msg ID : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##gps_logontime_msgid", ImGuiDataType_S16, &command->gps_logontimecmd.MsgId, NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->gps_logontimecmd, sizeof(GPS_LogOntimeCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(GPS_LOG_ONCHANGE_CC) :
                {
                    ImGui::Text("Msg ID : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##gps_logonchange_msgid", ImGuiDataType_S16, &command->gps_logonchangecmd.MsgId, NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->gps_logonchangecmd, sizeof(GPS_LogOnChangeCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->gps_noargscmd, sizeof(GPS_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("EPS"))
            {
                if(gen_msgid != EPS_CMD_MID)
                {
                    gen_msgid = EPS_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, EPSCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(EPSCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(EPSCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(EPS_SET_CONFIG_PARAMETER_CC) : {
                    ImGui::Text("Parameter ID : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##ParameterID", ImGuiDataType_U16, &command->eps_setconfig.ParameterID, NULL, NULL, "%u");
                    ImGui::Text("Parameter Length : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameter Length", ImGuiDataType_U8, &command->eps_setconfig.ParameterLength, NULL, NULL, "%u");
                    ImGui::Text("Parameter[0] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameters[0]", ImGuiDataType_U8, &command->eps_setconfig.Parameter[0], NULL, NULL, "%u");
                    ImGui::Text("Parameter[1] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameters[1]", ImGuiDataType_U8, &command->eps_setconfig.Parameter[1], NULL, NULL, "%u");
                    ImGui::Text("Parameter[2] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameters[2]", ImGuiDataType_U8, &command->eps_setconfig.Parameter[2], NULL, NULL, "%u");
                    ImGui::Text("Parameter[3] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameters[3]", ImGuiDataType_U8, &command->eps_setconfig.Parameter[3], NULL, NULL, "%u");
                    ImGui::Text("Parameter[4] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameters[4]", ImGuiDataType_U8, &command->eps_setconfig.Parameter[4], NULL, NULL, "%u");
                    ImGui::Text("Parameter[5] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameters[5]", ImGuiDataType_U8, &command->eps_setconfig.Parameter[5], NULL, NULL, "%u");
                    ImGui::Text("Parameter[6] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameters[6]", ImGuiDataType_U8, &command->eps_setconfig.Parameter[6], NULL, NULL, "%u");
                    ImGui::Text("Parameter[7] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameters[7]", ImGuiDataType_U8, &command->eps_setconfig.Parameter[7], NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->eps_setconfig, sizeof(EPS_SetConfigurationParameterCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(EPS_RESET_CONFIG_PARAMETER_CC) : {
                    ImGui::Text("Parameter Length : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameter Length", ImGuiDataType_U8, &command->eps_resetconfig.ParameterLength, NULL, NULL, "%u");
                    ImGui::Text("Parameter ID : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Parameter ID", ImGuiDataType_U16, &command->eps_resetconfig.ParameterID, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->eps_setconfig, sizeof(EPS_SetConfigurationParameterCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(EPS_CORRECT_TIME_CC) : {
                    ImGui::Text("Args : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##eps_correcttime_Arg : ", ImGuiDataType_S32, &command->eps_correcttimecmd.Arg, NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->eps_correcttimecmd, sizeof(EPS_CorrectTimeCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(EPS_OUTPUT_BUS_GROUP_ON_CC) : {
                    ImGui::Text("Args : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##eps_outputbusgroupon_arg", ImGuiDataType_U16, &command->eps_outputbusgrouponcmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->eps_outputbusgrouponcmd, sizeof(EPS_OutputBusGroupOnCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(EPS_OUTPUT_BUS_GROUP_OFF_CC) : {
                    ImGui::Text("Args : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##eps_outputbusgroupoff_arg", ImGuiDataType_U16, &command->eps_outputbusgroupoffcmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->eps_outputbusgroupoffcmd, sizeof(EPS_OUTPUT_BUS_GROUP_OFF_CC)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(EPS_OUTPUT_BUS_GROUP_STATE_CC) : {
                    ImGui::Text("Args : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##eps_outputbusgroupstate_arg", ImGuiDataType_U16, &command->eps_outputbusgroupstatecmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->eps_outputbusgroupstatecmd, sizeof(EPS_OutputBusGroupStateCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(EPS_OUTPUT_BUS_CHANNEL_ON_CC) : {
                    ImGui::Text("Args : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##eps_outputbuschannelon", ImGuiDataType_U8, &command->eps_outputbuschanneloffcmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->eps_outputbuschanneloffcmd, sizeof(EPS_OutputBusChannelOnCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(EPS_OUTPUT_BUS_CHANNEL_OFF_CC) : {
                    ImGui::Text("Args : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##eps_outputbuschanneloff_arg", ImGuiDataType_U8, &command->eps_outputbuschanneloffcmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->eps_outputbuschanneloffcmd, sizeof(EPS_OutputBusChannelOffCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default: {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->eps_noargscmd, sizeof(EPS_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("FM"))
            {
                if(gen_msgid != FM_CMD_MID)
                {
                    gen_msgid = FM_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, FMCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(FMCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(FMCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch (gen_fnccode)
                {
                case FM_RESET_APP_CC : {
                    ImGui::Text("Application ID             : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Application ID", ImGuiDataType_U32, &command->fm_resetappcmd.AppId, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->fm_resetappcmd, sizeof(FM_ResetAppCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case FM_MODE_TRANSFER_CC : {
                    ImGui::Text("Mode                       : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Mode", ImGuiDataType_U8, &command->fm_modetransfercmd.Mode, NULL, NULL, "%u");
                    ImGui::Text("Submode                    : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Submode", ImGuiDataType_U8, &command->fm_modetransfercmd.Submode, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->fm_modetransfercmd, sizeof(FM_ModeTransferCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case FM_SET_OPERATION_MODE_CC : {
                    ImGui::Text("Mode                       : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Mode", ImGuiDataType_U8, &command->fm_setoperationmodecmd.Mode, NULL, NULL, "%u");
                    ImGui::Text("Submode                    : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Submode", ImGuiDataType_U8, &command->fm_setoperationmodecmd.Submode, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->fm_setoperationmodecmd, sizeof(FM_SetOperationModeCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case FM_SET_COMMISSIONING_PHASE_CC : {
                    ImGui::Text("Commissioning Phase        : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Commissioning Phase", ImGuiDataType_U8, &command->fm_setcommissioningphasecmd.CommissioningPhase, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->fm_setcommissioningphasecmd, sizeof(FM_SetCommissioningPhaseCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case FM_SET_SPACECRAFT_TIME_CC : {
                    ImGui::Text("Second                     : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Second", ImGuiDataType_U32, &command->fm_setspacecrafttimecmd.Seconds, NULL, NULL, "%u");
                    ImGui::Text("Subsecond                  : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Subsecond", ImGuiDataType_U32, &command->fm_setspacecrafttimecmd.Subseconds, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->fm_setspacecrafttimecmd, sizeof(FM_SetSpacecraftTimeCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case FM_SET_ANTENNA_DEPLOY_FLAG_CC : {
                    ImGui::Text("Antenna Deploy Flag        : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Antenna Deploy Flag", ImGuiDataType_U8, &command->fm_setantennadeployflagcmd.AntennaDeployFlag, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->fm_setantennadeployflagcmd, sizeof(FM_SetAntennaDeployFlagCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case FM_SET_DAYLIGHT_DETECTION_FLAG_CC : {
                    ImGui::Text("Daylight Detection Flag    : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Daylight Detection Flag", ImGuiDataType_U8, &command->fm_setdaylightdetectionflagcmd.DaylightDetectionFlag, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->fm_setdaylightdetectionflagcmd, sizeof(FM_SetDaylightDetectionFlagCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case FM_SET_COMM_MISSING_FLAG_CC : {
                    ImGui::Text("Commissioning Flag         : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##Commissioning Flag", ImGuiDataType_U8, &command->fm_setcommmissingflagcmd.CommunicationMissing, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->fm_setcommmissingflagcmd, sizeof(FM_SetCommMissingFlagCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default: {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->fm_noargscmd, sizeof(FM_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("MTQ"))
            {
                if(gen_msgid != MTQ_CMD_MID)
                {
                    gen_msgid = MTQ_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, MTQCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(MTQCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(MTQCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(MTQ_RESET_COMP) : {
                    ImGui::Text("Args");
                    ImGui::Text("[0] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_resetcompcmd0", ImGuiDataType_U8, &command->mtq_resetcompcmd.Args[0], NULL, NULL, "%u");
                    ImGui::Text("[1] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_resetcompcmd1", ImGuiDataType_U8, &command->mtq_resetcompcmd.Args[1], NULL, NULL, "%u");
                    ImGui::Text("[2] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_resetcompcmd2", ImGuiDataType_U8, &command->mtq_resetcompcmd.Args[2], NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->mtq_resetcompcmd, sizeof(MTQ_ResetCompCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(MTQ_ENABLE) : {
                    ImGui::Text("Args");
                    ImGui::Text("[0] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_enablecmd0", ImGuiDataType_U8, &command->mtq_enablecmd.Args[0], NULL, NULL, "%u");
                    ImGui::Text("[1] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_enablecmd1", ImGuiDataType_U8, &command->mtq_enablecmd.Args[1], NULL, NULL, "%u");
                    ImGui::Text("[2] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_enablecmd2", ImGuiDataType_U8, &command->mtq_enablecmd.Args[2], NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->mtq_enablecmd, sizeof(MTQ_EnableCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(MTQ_DISABLE) : {
                    ImGui::Text("Args");
                    ImGui::Text("[0] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_diablecmd0", ImGuiDataType_U8, &command->mtq_disablecmd.Args[0], NULL, NULL, "%u");
                    ImGui::Text("[1] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_disablecmd1", ImGuiDataType_U8, &command->mtq_disablecmd.Args[1], NULL, NULL, "%u");
                    ImGui::Text("[2] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_disablecmd2", ImGuiDataType_U8, &command->mtq_disablecmd.Args[2], NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->mtq_disablecmd, sizeof(MTQ_DisableCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(MTQ_SET_POLARITY) : {
                    ImGui::Text("Args");
                    ImGui::Text("[0] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_setpolarity0", ImGuiDataType_U8, &command->mtq_setpolaritycmd.Args[0], NULL, NULL, "%u");
                    ImGui::Text("[1] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_setpolarity1", ImGuiDataType_U8, &command->mtq_setpolaritycmd.Args[1], NULL, NULL, "%u");
                    ImGui::Text("[2] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_setpolarity2", ImGuiDataType_U8, &command->mtq_setpolaritycmd.Args[2], NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->mtq_setpolaritycmd, sizeof(MTQ_SetPolarityCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(MTQ_SET_DUTY) : {
                    ImGui::Text("Args");
                    ImGui::Text("[0] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_setdutycmd0", ImGuiDataType_S8, &command->mtq_setdutycmd.Args[0], NULL, NULL, "%d");
                    ImGui::Text("[1] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_setdutycmd1", ImGuiDataType_S8, &command->mtq_setdutycmd.Args[1], NULL, NULL, "%d");
                    ImGui::Text("[2] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_setdutycmd2", ImGuiDataType_U8, &command->mtq_setdutycmd.Args[2], NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->mtq_setdutycmd, sizeof(MTQ_SetDutyCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(MTQ_SET_PERIOD) : {
                    ImGui::Text("Args");
                    ImGui::Text("[0] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_setperiodcmd0", ImGuiDataType_U32, &command->mtq_setperiodcmd.Args[0], NULL, NULL, "%u");
                    ImGui::Text("[1] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_setperiodcmd1", ImGuiDataType_U32, &command->mtq_setperiodcmd.Args[1], NULL, NULL, "%u");
                    ImGui::Text("[2] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##mtq_setperiodcmd2", ImGuiDataType_U32, &command->mtq_setperiodcmd.Args[2], NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->mtq_setperiodcmd, sizeof(MTQ_SetPeriodCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->mtq_noargscmd, sizeof(MTQ_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Payload"))
            {
                if(gen_msgid != PAY_CMD_MID)
                {
                    gen_msgid = PAY_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, PAYCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(PAYCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(PAYCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(PAY_CAM_DOWNLOAD_OLD_IMG_CC) : {
                    ImGui::Text("Index : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##pay_camdownloadoldimg", ImGuiDataType_U16, &command->pay_camdownloadoldimgcmd.Index, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->pay_camdownloadoldimgcmd, sizeof(PAY_CamDownloadOldImgCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(PAY_CAM_SEND_NOARG_CC) : {
                    ImGui::Text("CMD Code : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##pay_camsendnoargcmd", ImGuiDataType_U8, &command->pay_camsendnoargcmd.CmdCode, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->pay_camsendnoargcmd, sizeof(PAY_CamDownloadOldImgCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(PAY_CAM_SET_EXPOSURE_CC) : {
                    ImGui::Text("Exposure : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##pay_camsetexposure", ImGuiDataType_U32, &command->pay_camsetexposurecmd.Exposure, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->pay_camsetexposurecmd, sizeof(PAY_CamSetExposureCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(PAY_CAM_SET_SCPD_CC) : {
                    ImGui::Text("Index : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##pay_camsetscpd", ImGuiDataType_U32, &command->pay_camsetscpdcmd.SCPD, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->pay_camsetscpdcmd, sizeof(PAY_CamSetScpdCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->pay_noargscmd, sizeof(PAY_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("RWA"))
            {
                if(gen_msgid != RWA_CMD_MID)
                {
                    gen_msgid = RWA_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, RWACMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(RWACMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(RWACMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(RWA_RESET_WHEEL_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_resetwheel", ImGuiDataType_U8, &command->rwa_resetwheelcmd.WhlNum, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_resetwheelcmd, sizeof(RWA_ResetWheelCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_CLEAR_ERRORS_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_clearerrors", ImGuiDataType_U8, &command->rwa_clearerrorscmd.WhlNum, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_clearerrorscmd, sizeof(RWA_ClearErrorsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_MOTOR_POWER_STATE_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setmotorpowerstate_wheelnum", ImGuiDataType_U8, &command->rwa_setmotorpowerstatecmd.WhlNum, NULL, NULL, "%u");
                    ImGui::Text("Arg : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setmotorpowerstate_arg", ImGuiDataType_U8, &command->rwa_setmotorpowerstatecmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_setmotorpowerstatecmd, sizeof(RWA_SetMotorPowerStateCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_ENCODER_POWER_STATE_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setencoderpowerstate_wheelnum", ImGuiDataType_U8, &command->rwa_setencoderpowerstatecmd.WhlNum, NULL, NULL, "%u");
                    ImGui::Text("Arg : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setencoderpowerstate_arg", ImGuiDataType_U8, &command->rwa_setencoderpowerstatecmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_setencoderpowerstatecmd, sizeof(RWA_SetEncoderPowerStateCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_HALL_POWER_STATE_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_sethallpowerstate_wheelnum", ImGuiDataType_U8, &command->rwa_sethallpowerstatecmd.WhlNum, NULL, NULL, "%u");
                    ImGui::Text("Arg : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_sethallpowerstate_arg", ImGuiDataType_U8, &command->rwa_sethallpowerstatecmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_sethallpowerstatecmd, sizeof(RWA_SetHallPowerStateCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_CONTROL_MODE_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setcontrolmode_wheelnum", ImGuiDataType_U8, &command->rwa_setcontrolmodecmd.WhlNum, NULL, NULL, "%u");
                    ImGui::Text("Arg : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setcontrolmode_arg", ImGuiDataType_U8, &command->rwa_setcontrolmodecmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_setcontrolmodecmd, sizeof(RWA_SetControlModeCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_BACKUP_WHEEL_MODE_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setbackupwheelmode_wheelnum", ImGuiDataType_U8, &command->rwa_setbackupwheelmodecmd.WhlNum, NULL, NULL, "%u");
                    ImGui::Text("Arg : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setbackupwheelmode_arg", ImGuiDataType_U8, &command->rwa_setbackupwheelmodecmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_setbackupwheelmodecmd, sizeof(RWA_SetBackupWheelModeCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_WHEEL_REFERENCE_SPEED_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setwheelreferencespeed_wheelnum", ImGuiDataType_U8, &command->rwa_setwheelreferencespeedcmd.WhlNum, NULL, NULL, "%u");
                    ImGui::Text("Arg : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setwheelreferencespeed_arg", ImGuiDataType_S16, &command->rwa_setwheelreferencespeedcmd.Arg, NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_setwheelreferencespeedcmd, sizeof(RWA_SetWheelReferenceSpeedCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_WHEEL_COMMANDED_TORQUE_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setwheelcommandedtorque_wheelnum", ImGuiDataType_U8, &command->rwa_setwheelcommandedtorquecmd.WhlNum, NULL, NULL, "%u");
                    ImGui::Text("Arg : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setwheelcommandedtorque_arg", ImGuiDataType_S16, &command->rwa_setwheelcommandedtorquecmd.Arg, NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_setwheelcommandedtorquecmd, sizeof(RWA_SetWheelCommandedTorqueCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_PWM_GAIN_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setpwmgain_wheelnum", ImGuiDataType_U8, &command->rwa_setpwmgaincmd.WhlNum, NULL, NULL, "%u");
                    ImGui::Text("K : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setpwmgain_k", ImGuiDataType_S16, &command->rwa_setpwmgaincmd.Input.K, NULL, NULL, "%d");
                    ImGui::Text("Kmultiplier : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setpwmgain_kmultiplier", ImGuiDataType_U8, &command->rwa_setpwmgaincmd.Input.Kmultiplier, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_setpwmgaincmd, sizeof(RWA_SetPwmGainCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_MAIN_GAIN_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setmaingain_wheelnum", ImGuiDataType_U8, &command->rwa_setmaingaincmd.WhlNum, NULL, NULL, "%u");
                    ImGui::Text("Ki : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setmaingain_ki", ImGuiDataType_U16, &command->rwa_setmaingaincmd.Input.Ki, NULL, NULL, "%u");
                    ImGui::Text("Kimultiplier : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setmaingain_kimultiplier", ImGuiDataType_U8, &command->rwa_setmaingaincmd.Input.KiMultiplier, NULL, NULL, "%u");
                    ImGui::Text("Kd : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setmaingain_kd", ImGuiDataType_U16, &command->rwa_setmaingaincmd.Input.Kd, NULL, NULL, "%u");
                    ImGui::Text("Kdmultiplier : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setmaingain_kdmultiplier", ImGuiDataType_U8, &command->rwa_setmaingaincmd.Input.KdMultiplier, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_setmaingaincmd, sizeof(RWA_SetMainGainCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_BACKUP_GAIN_CC) : {
                    ImGui::Text("Wheel Num : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setbackupgain_wheelnum", ImGuiDataType_U8, &command->rwa_setbackupgaincmd.WhlNum, NULL, NULL, "%u");
                    ImGui::Text("Ki : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setbackupgain_ki", ImGuiDataType_U16, &command->rwa_setbackupgaincmd.Input.Ki, NULL, NULL, "%u");
                    ImGui::Text("Kimultiplier : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setbackupgain_kimultiplier", ImGuiDataType_U8, &command->rwa_setbackupgaincmd.Input.KiMultiplier, NULL, NULL, "%u");
                    ImGui::Text("Kd : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setbackupgain_kd", ImGuiDataType_U16, &command->rwa_setbackupgaincmd.Input.Kd, NULL, NULL, "%u");
                    ImGui::Text("Kdmultiplier : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setbackupgain_kdmultiplier", ImGuiDataType_U8, &command->rwa_setbackupgaincmd.Input.KdMultiplier, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_setbackupgaincmd, sizeof(RWA_SetBackupGainCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(RWA_SET_WHEEL_REFERENCE_SPEED_ALL_AXIS_CC) : {
                    ImGui::Text("Wheel Ref Speed[0]: ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setwheelreferencespeedallaxis[0]", ImGuiDataType_S16, &command->rwa_setwheelreferencespeedallaxiscmd.WheelRefSpeed[0], NULL, NULL, "%d");
                    ImGui::Text("Wheel Ref Speed[1]: ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setwheelreferencespeedallaxis[1]", ImGuiDataType_S16, &command->rwa_setwheelreferencespeedallaxiscmd.WheelRefSpeed[1], NULL, NULL, "%d");
                    ImGui::Text("Wheel Ref Speed[2]: ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##rwa_setwheelreferencespeedallaxis[2]", ImGuiDataType_S16, &command->rwa_setwheelreferencespeedallaxiscmd.WheelRefSpeed[2], NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_setwheelreferencespeedallaxiscmd, sizeof(RWA_SetWheelReferenceSpeedAllAxisCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->rwa_noargscmd, sizeof(RWA_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("Sensor"))
            {
                if(gen_msgid != SNSR_CMD_MID)
                {
                    gen_msgid = SNSR_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, SNSRCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(SNSRCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(SNSRCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(SNSR_STT_BOOT_CC) : {
                    ImGui::Text("Arg : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_sttboot_arg", ImGuiDataType_U8, &command->snsr_stt_bootcmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_stt_bootcmd, sizeof(SNSR_STT_BootCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_STT_PING_CC) : {
                    ImGui::Text("Arg : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_sttping_arg", ImGuiDataType_U32, &command->snsr_stt_pingcmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_stt_pingcmd, sizeof(SNSR_STT_PingCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_STT_UNLOCK_CC) : {
                    ImGui::Text("Image : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_sttunlock_image", ImGuiDataType_U8, &command->snsr_stt_unlockcmd.Image, NULL, NULL, "%u");
                    ImGui::Text("Code : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_sttunlock_code", ImGuiDataType_U32, &command->snsr_stt_unlockcmd.Code, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_stt_unlockcmd, sizeof(SNSR_STT_UnlockCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_STT_SET_PARAM_CC) : {
                    ImGui::Text("Param ID : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_sttsetparam_paramid", ImGuiDataType_U8, &command->snsr_stt_setparamcmd.ParamId, NULL, NULL, "%u");
                    ImGui::Text("Param Size : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_sttsetparam_paramsize", ImGuiDataType_U16, &command->snsr_stt_setparamcmd.ParamSize, NULL, NULL, "%u");
                    ImGui::Text("Param : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_sttsetparam_param", ImGuiDataType_U8, &command->snsr_stt_setparamcmd.Param[0], NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_stt_setparamcmd, sizeof(SNSR_STT_SetParamCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_STT_SEND_RS485_CC) : {
                    ImGui::Text("Length : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_sttsendrs485_length", ImGuiDataType_U16, &command->snsr_stt_sendrs485cmd.Length, NULL, NULL, "%u");
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_sttsendrs485_length", ImGuiDataType_U8, &command->snsr_stt_sendrs485cmd.Data[0], NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_stt_sendrs485cmd, sizeof(SNSR_STT_SendRS485Cmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_MMT_SET_INTERNAL_CONTROL0_CC) : {
                    ImGui::Text("TM_M : ");
                    ImGui::SameLine();
                    ImGui::Checkbox("##snsr_mmt_setinternalcontrol0cmd_ tm_m", &command->snsr_mmt_setinternalcontrol0cmd.TM_M);
                    ImGui::Text("TM_T : ");
                    ImGui::SameLine();
                    ImGui::Checkbox("##snsr_mmt_setinternalcontrol0cmd_ tm_t", &command->snsr_mmt_setinternalcontrol0cmd.TM_T);
                    ImGui::Text("Start_MDT : ");
                    ImGui::SameLine();
                    ImGui::Checkbox("##snsr_mmt_setinternalcontrol0cmd_ start_mdt", &command->snsr_mmt_setinternalcontrol0cmd.Start_MDT);
                    ImGui::Text("Set : ");
                    ImGui::SameLine();
                    ImGui::Checkbox("##snsr_mmt_setinternalcontrol0cmd_ set", &command->snsr_mmt_setinternalcontrol0cmd.Set);
                    ImGui::Text("Reset : ");
                    ImGui::SameLine();
                    ImGui::Checkbox("##snsr_mmt_setinternalcontrol0cmd_ reset", &command->snsr_mmt_setinternalcontrol0cmd.Reset);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_mmt_setinternalcontrol0cmd, sizeof(SNSR_MMT_SetInternalControl0Cmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;   
                }
                case(SNSR_MMT_SET_INTERNAL_CONTROL2_CC) : {
                    ImGui::Text("CM_Freq : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_mmt_setinternalcontrol2_cm_freq", ImGuiDataType_U8, &command->snsr_mmt_setinternalcontrol2cmd.CM_Freq, NULL, NULL, "%u");
                    ImGui::Text("INT_MDT_EN : ");
                    ImGui::SameLine();
                    ImGui::Checkbox("##snsr_mmt_setinternalcontrol2_intmdten", &command->snsr_mmt_setinternalcontrol2cmd.INT_MDT_EN);
                    ImGui::Text("INT_Meas_Done_EN : ");
                    ImGui::SameLine();
                    ImGui::Checkbox("##snsr_mmt_setinternalcontrol2_int_meas_done_en", &command->snsr_mmt_setinternalcontrol2cmd.INT_Meas_Done_EN);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_mmt_setinternalcontrol2cmd, sizeof(SNSR_MMT_SetInternalControl2Cmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_MMT_WRITE_TO_REGISTER_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_mmt_write_to_register_data", ImGuiDataType_U8, &command->snsr_mmt_writetoregistercmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_mmt_writetoregistercmd, sizeof(SNSR_MMT_WriteToRegisterCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_IMU_SET_GYRO_OFFSET_CC) : {
                    ImGui::Text("Offset[0] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setgyrooffsetcmd_offset0", ImGuiDataType_S16, &command->snsr_imu_setgyrooffsetcmd.Offset[0], NULL, NULL, "%d");
                    ImGui::Text("Offset[1] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setgyrooffsetcmd_offset1", ImGuiDataType_S16, &command->snsr_imu_setgyrooffsetcmd.Offset[1], NULL, NULL, "%d");
                    ImGui::Text("Offset[2] : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setgyrooffsetcmd_offset2", ImGuiDataType_S16, &command->snsr_imu_setgyrooffsetcmd.Offset[2], NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_imu_setgyrooffsetcmd, sizeof(SNSR_IMU_SetGyroOffsetCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_IMU_SET_CONFIGURATION_CC) : {
                    ImGui::Text("FifoMode : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setconfiguration_fifomode", ImGuiDataType_U8, &command->snsr_imu_setconfigurationcmd.FifoMode, NULL, NULL, "%u");
                    ImGui::Text("ExtSyncSet : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setconfiguration_extsyncset", ImGuiDataType_U8, &command->snsr_imu_setconfigurationcmd.ExtSyncSet, NULL, NULL, "%u");
                    ImGui::Text("Config DLPF : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setconfiguration_configdlpf", ImGuiDataType_U8, &command->snsr_imu_setconfigurationcmd.ConfigDLPF, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_imu_setconfigurationcmd, sizeof(SNSR_IMU_SetConfigurationCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_IMU_SET_GYRO_CONFIGURATION_CC) : {
                    ImGui::Text("Accel Full Scale : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setgyroconfiguration_accelfullscale", ImGuiDataType_U8, &command->snsr_imu_setgyroconfigurationcmd.AccelFullScale, NULL, NULL, "%u");
                    ImGui::Text("Filter Choice : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setgyroconfiguration_filterchoice", ImGuiDataType_U8, &command->snsr_imu_setgyroconfigurationcmd.FilterChoice, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_imu_setgyroconfigurationcmd, sizeof(SNSR_IMU_SetGyroConfigurationCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_IMU_SET_ACCEL_CONFIGURATION_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setaccelconfiguration_args", ImGuiDataType_U8, &command->snsr_imu_setaccelconfigurationcmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_imu_setaccelconfigurationcmd, sizeof(SNSR_IMU_SetAccelConfigurationCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_IMU_SET_ACCEL_CONFIGURATION2_CC) : {
                    ImGui::Text("Fifo Size : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setaccelconfiguration2_fifosize", ImGuiDataType_U8, &command->snsr_imu_setaccelconfiguration2cmd.FifoSize, NULL, NULL, "%u");
                    ImGui::Text("DEC2_CFG : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setaccelconfiguration2_dec2cfg", ImGuiDataType_U8, &command->snsr_imu_setaccelconfiguration2cmd.DEC2_CFG, NULL, NULL, "%u");
                    ImGui::Text("AccelFilter Choice : ");
                    ImGui::SameLine();
                    ImGui::Checkbox("##snsr_imu_setaccelconfiguration2_accelfilterchoice", &command->snsr_imu_setaccelconfiguration2cmd.AccelFilterChoice);
                    ImGui::Text("A_DLPF_CFG : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_setaccelconfiguration2_adlpfcfg", ImGuiDataType_U8, &command->snsr_imu_setaccelconfiguration2cmd.A_DLPF_CFG, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_imu_setaccelconfiguration2cmd, sizeof(SNSR_IMU_SetAccelConfiguration2Cmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_IMU_SET_POWER_MANAGEMENT_CC) : {
                    ImGui::Text("Device Reset : ");
                    ImGui::SameLine();
                    ImGui::Checkbox("##snsr_imu_setpowermanagement_devicereset", &command->snsr_imu_setpowermanagement1cmd.DEVICE_RESET);
                    ImGui::Text("Sleep : ");
                    ImGui::SameLine();
                    ImGui::Checkbox("##snsr_setpowermanagement_sleep", &command->snsr_imu_setpowermanagement1cmd.SLEEP);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_imu_setpowermanagement1cmd, sizeof(SNSR_IMU_SetPowerManagement1Cmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_IMU_WRITE_TO_REGISTER_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_imu_writetoregister_data", ImGuiDataType_U8 ,&command->snsr_imu_writetoregistercmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_imu_writetoregistercmd, sizeof(SNSR_IMU_WriteToRegisterCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_FSS_ISOLATE_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_fss_isolate", ImGuiDataType_U8 ,&command->snsr_fss_isolatedcmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_fss_isolatedcmd, sizeof(SNSR_FSS_IsolateCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_FSS_RESTORE_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_fss_restore", ImGuiDataType_U8 ,&command->snsr_fss_restorecmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_fss_restorecmd, sizeof(SNSR_FSS_RestoreCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_CSS_ISOLATE_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_css_isolate", ImGuiDataType_U8 ,&command->snsr_css_isolatedcmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_css_isolatedcmd, sizeof(SNSR_CSS_IsolateCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SNSR_CSS_RESTORE_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##snsr_css_restore", ImGuiDataType_U8 ,&command->snsr_css_restorecmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_css_restorecmd, sizeof(SNSR_CSS_RestoreCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->snsr_noargscmd, sizeof(SNSR_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("STX"))
            {
                if(gen_msgid != STX_CMD_MID)
                {
                    gen_msgid = STX_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, STXCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(STXCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(STXCMDLabels[i], SelectedGen))
                                gen_fnccode = i;
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(STX_TRANSMIT_DATA_CC) : {
                    ImGui::Text("Length : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_transmitdata_length", ImGuiDataType_S16 ,&command->stx_transmitdatacmd.Length, NULL, NULL, "%d");
                    ImGui::Text("Buf Push Delay : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_transmitdata_bufpushdelay", ImGuiDataType_U16 ,&command->stx_transmitdatacmd.BufPushDelay, NULL, NULL, "%u");
                    ImGui::Text("Data(ASCII, 16byte) : ");
                    ImGui::SameLine();
                    ImGui::InputText("##stx_transmitdata_data", (char *)command->stx_transmitdatacmd.Data ,sizeof(command->stx_transmitdatacmd.Data), NULL, NULL, NULL);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->stx_transmitdatacmd, sizeof(STX_TransmitDataCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(STX_SET_CONTROL_MODE_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_setcontrolmodecmd", ImGuiDataType_U8 ,&command->stx_setcontrolmodecmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->stx_setcontrolmodecmd, sizeof(STX_SetControlModeCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(STX_SET_ENCODER_RATE_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_setencoderrate", ImGuiDataType_U8 ,&command->stx_setencoderratecmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->stx_setencoderratecmd, sizeof(STX_SetEncoderRateCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(STX_SET_PA_POWER_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_setpapower", ImGuiDataType_U8 ,&command->stx_setpapowercmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->stx_setpapowercmd, sizeof(STX_SetPaPowerCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(STX_SET_SYNTH_OFFSET_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_setsynthoffset", ImGuiDataType_U8 ,&command->stx_setsynthoffsetcmd.Arg, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->stx_setsynthoffsetcmd, sizeof(STX_SetSynthOffsetCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(STX_TRANSMIT_FILE_CC) : {
                    ImGui::Text("Offset : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_transmitfile_offset", ImGuiDataType_U32 ,&command->stx_transmitfilecmd.Offset, NULL, NULL, "%u");
                    ImGui::Text("Length : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_transmitfile_length", ImGuiDataType_U32 ,&command->stx_transmitfilecmd.Length, NULL, NULL, "%u");
                    ImGui::Text("BufPushDelay : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_transmitfile_BufPushDelay", ImGuiDataType_U16 ,&command->stx_transmitfilecmd.BufPushDelay, NULL, NULL, "%u");
                    ImGui::Text("Path : ");
                    ImGui::SameLine();
                    ImGui::InputText("##stx_transmitfile_path", command->stx_transmitfilecmd.Path, sizeof(command->stx_transmitfilecmd.Path), NULL, NULL, NULL);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->stx_transmitfilecmd, sizeof(STX_TransmitFileCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(STX_TRANSMIT_FILE_LONG_PATH_CC) : {
                    ImGui::Text("Offset : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_transmitfilelongpath_offset", ImGuiDataType_U32 ,&command->stx_transmitfilelongpathcmd.Offset, NULL, NULL, "%u");
                    ImGui::Text("Length : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_transmitfilelongpath_length", ImGuiDataType_U32 ,&command->stx_transmitfilelongpathcmd.Length, NULL, NULL, "%u");
                    ImGui::Text("BufPushDelay : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##stx_transmitfilelongpath_BufPushDelay", ImGuiDataType_U16 ,&command->stx_transmitfilelongpathcmd.BufPushDelay, NULL, NULL, "%u");
                    ImGui::Text("Path : ");
                    ImGui::SameLine();
                    ImGui::InputText("##stx_transmitfile_path", command->stx_transmitfilelongpathcmd.Path, sizeof(command->stx_transmitfilelongpathcmd.Path), NULL, NULL, NULL);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->stx_transmitfilelongpathcmd, sizeof(STX_TransmitFileLongPathCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->stx_noargscmd, sizeof(STX_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("TO"))
            {
                if(gen_msgid != TO_CMD_MID)
                {
                    gen_msgid = TO_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, TOCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(TOCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(TOCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(TO_DOWNLINK_QUERY_REPLY_CC) : {
                    ImGui::Text("Target : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##to_downlinkqueryreply_target", ImGuiDataType_U16 ,&command->to_downlinkqueryreplycmd.Payload.Target, NULL, NULL, "%u");
                    ImGui::Text("FileStatus : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##to_downlinkqueryreply_filestatus", ImGuiDataType_U16 ,&command->to_downlinkqueryreplycmd.Payload.FileStatus, NULL, NULL, "%u");
                    ImGui::Text("NumFiles : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##to_downlinkqueryreply_numfiles", ImGuiDataType_U32 ,&command->to_downlinkqueryreplycmd.Payload.NumFiles, NULL, NULL, "%u");
                    ImGui::Text("Offset : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##to_downlinkqueryreply_offset", ImGuiDataType_U32 ,&command->to_downlinkqueryreplycmd.Payload.Offset, NULL, NULL, "%u");
                    ImGui::Text("Frequency : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##to_downlinkqueryreply_frequency", ImGuiDataType_U32 ,&command->to_downlinkqueryreplycmd.Payload.Frequency, NULL, NULL, "%u");
                    ImGui::Text("Conn(void pointer) : ");
                    ImGui::SameLine();
                    ImGui::Text("Contact To Manager.");
                
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->to_downlinkqueryreplycmd, sizeof(command->to_downlinkqueryreplycmd)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(TO_DISABLE_BEACON_CC): {
                    ImGui::Text("Timeoutsmin : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##to_disablereply_target", ImGuiDataType_S32 ,&command->to_disablebeaconcmd.timeoutsmin, NULL, NULL, "%d");

                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->to_disablebeaconcmd, sizeof(TO_DisableBeaconCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->to_noargscmd, sizeof(TO_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("UTRX"))
            {
                if(gen_msgid != UTRX_CMD_MID)
                {
                    gen_msgid = UTRX_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, UTRXCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(UTRXCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(UTRXCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(UTRX_SET_TX_FREQ_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##utrx_settxfreq", ImGuiDataType_U32 ,&command->utrx_settxfreqcmd.Arg, NULL, NULL, "%u");
                
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->utrx_settxfreqcmd, sizeof(UTRX_SetTxFreqCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(UTRX_SET_TX_BAUD_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##utrx_settxbaud", ImGuiDataType_U32 ,&command->utrx_settxbaudcmd.Arg, NULL, NULL, "%u");
                
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->utrx_settxbaudcmd, sizeof(UTRX_SetTxBaudCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(UTRX_SET_TX_MODIND_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputFloat("##utrx_settxmodind", &command->utrx_settxmodindex.Arg);
                
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->utrx_settxmodindex, sizeof(UTRX_SetTxModIndexCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(UTRX_SET_TX_MODE_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##utrx_settxmode", ImGuiDataType_U8 ,&command->utrx_settxmodecmd.Arg, NULL, NULL, "%u");
                
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->utrx_settxmodecmd, sizeof(UTRX_SetTxModeCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(UTRX_SET_RX_FREQ_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##utrx_setrxfreq", ImGuiDataType_U32 ,&command->utrx_setrxfreqcmd.Arg, NULL, NULL, "%u");
                
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->utrx_setrxfreqcmd, sizeof(UTRX_SetRxFreqCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(UTRX_SET_RX_BAUD_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##utrx_setrxbaud", ImGuiDataType_U32 ,&command->utrx_setrxbaudcmd.Arg, NULL, NULL, "%u");
                
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->utrx_setrxbaudcmd, sizeof(UTRX_SetRxBaudCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(UTRX_SET_RX_MODIND_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputFloat("##utrx_setrxmodindex", &command->utrx_setrxmodindexcmd.Arg);
                
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->utrx_setrxmodindexcmd, sizeof(UTRX_SetRxModIndexCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(UTRX_SET_RX_MODE_CC) : {
                    ImGui::Text("Data : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##utrx_setrxmode", ImGuiDataType_U8 ,&command->utrx_setrxmodecmd.Arg, NULL, NULL, "%u");                

                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->utrx_setrxmodecmd, sizeof(UTRX_SetRxModeCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->utrx_noargscmd, sizeof(UTRX_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("TS"))
            {
                if(gen_msgid != TS_CMD_MID)
                {
                    gen_msgid = TS_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, TSCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(TSCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(TSCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(TS_INSERT_SCHEDULE_ENTRY_CC) : {
                    ImGui::Text("Execution Time : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##ts_insertscheduleentry_executiontime", ImGuiDataType_U32 ,&command->ts_insertscheduleentrycmd.ExecutionTime, NULL, NULL, "%u");
                    ImGui::Text("Execution Window : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##ts_insertscheduleentry_executionwindow", ImGuiDataType_U32 ,&command->ts_insertscheduleentrycmd.ExecutionWindow, NULL, NULL, "%u");  
                    ImGui::Text("Entry ID : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##ts_insertscheduleentry_entryid", ImGuiDataType_U16 ,&command->ts_insertscheduleentrycmd.EntryId, NULL, NULL, "%u");  
                    ImGui::Text("Entry Group : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##ts_insertscheduleentry_entrygroup", ImGuiDataType_U16 ,&command->ts_insertscheduleentrycmd.EntryGroup, NULL, NULL, "%u");
                    ImGui::Text("Execution Msg : ");
                    ImGui::SameLine();
                    ImGui::Text("Contact to Manager.");

                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->ts_insertscheduleentrycmd, sizeof(TS_InsertScheduleEntryCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(TS_CLEAR_SCHEDULE_ENTRY_CC) : {
                    ImGui::Text("Entry ID : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##ts_clearscheduleentry_entryid", ImGuiDataType_U16 ,&command->ts_clearscheduleentrycmd.EntryId, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->ts_clearscheduleentrycmd, sizeof(TS_ClearScheduleEntryCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(TS_CLEAR_SCHEDULE_GROUP_CC) : {
                    ImGui::Text("Entry Group : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##ts_clearscheduleentry_entrygroup", ImGuiDataType_U16 ,&command->ts_clearschedulegroupcmd.EntryGroup, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->ts_clearschedulegroupcmd, sizeof(TS_ClearScheduleGroupCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->ts_noargscmd, sizeof(TS_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("ECM"))
            {
                if(gen_msgid != ECM_CMD_MID)
                {
                    gen_msgid = ECM_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, ECMCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(ECMCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(ECMCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                        
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case (ECM_READ_CC) : {
                    ImGui::Text("TXlen : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##ecm_read_txlen", ImGuiDataType_U8, &command->ecm_readcmd.txlen , NULL, NULL, "%u");
                    ImGui::Text("RXlen : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##ecm_read_rxlen", ImGuiDataType_U8, &command->ecm_readcmd.rxlen , NULL, NULL, "%u");
                    ImGui::Text("CC    : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##ecm_read_cc", ImGuiDataType_U8, &command->ecm_readcmd.cc , NULL, NULL, "%u");
                    ImGui::Text("data(ASCII): ");
                    ImGui::SameLine();
                    ImGui::InputText("##ecm_read_data", (char *)&command->ecm_readcmd.data, sizeof(command->ecm_readcmd.data));
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->ecm_readcmd, sizeof(ECM_Read_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->ecm_noargscmd, sizeof(ECM_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("ES"))
            {
                if(gen_msgid != ES_CMD_MID)
                {
                    gen_msgid = ES_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, ESCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(ESCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(ESCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                        
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case (CFE_ES_RESTART_CC) : {
                    ImGui::Text("RestartType : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##es_restart_restarttype", ImGuiDataType_U16, &command->es_restartcmd.Payload.RestartType, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->es_restartcmd, sizeof(CFE_ES_RestartCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case (CFE_ES_RESTART_APP_CC) : {
                    ImGui::Text("Appname : ");
                    ImGui::SameLine();
                    ImGui::InputText("##es_restartapp", command->es_restartappcmd.Payload.Application, CFE_MISSION_MAX_API_LEN);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->es_restartappcmd, sizeof(CFE_ES_AppNameCmd_t)) == 0)
                    {
                        printf("%s\n", command->es_restartappcmd.Payload.Application);
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                 case (CFE_ES_STOP_APP_CC) : {
                    ImGui::Text("Application : ");
                    ImGui::SameLine();
                    ImGui::InputText("##es_stopapp", command->es_stopappcmd.Payload.Application, 19);
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->es_stopappcmd, sizeof(CFE_ES_StopAppCmd_t)) == 0)
                    {
                        printf("%s\n", command->es_stopappcmd.Payload.Application);
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case (CFE_ES_START_APP_CC) : {
                    ImGui::Text("Application : ");
                    ImGui::SameLine();
                    ImGui::InputText("##es_startapp_application", command->es_startappcmd.Payload.Application, CFE_MISSION_MAX_API_LEN);
                    ImGui::Text("AppEntryPoint : ");
                    ImGui::SameLine();
                    ImGui::InputText("##es_startapp_appentrypoint", command->es_startappcmd.Payload.AppEntryPoint, CFE_MISSION_MAX_API_LEN);
                    ImGui::Text("AppFileName : ");
                    ImGui::SameLine();
                    ImGui::InputText("##es_startapp_appfilename", command->es_startappcmd.Payload.AppFileName, CFE_MISSION_MAX_PATH_LEN);
                    ImGui::Text("StackSize : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##es_startapp_stacksize", ImGuiDataType_U32, &command->es_startappcmd.Payload.StackSize, NULL, NULL, "%u");
                    ImGui::Text("ExceptionAction : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##es_startapp_exceptionaction", ImGuiDataType_U8, &command->es_startappcmd.Payload.ExceptionAction, NULL, NULL, "%u");
                    ImGui::Text("Priority : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##es_startapp_priority", ImGuiDataType_U16, &command->es_startappcmd.Payload.Priority, NULL, NULL, "%u");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->es_startappcmd, sizeof(CFE_ES_StartAppCmd_t)) == 0)
                    {
                        printf("%s\n", command->es_stopappcmd.Payload.Application);
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->es_noargscmd, sizeof(CFE_ES_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem("SCH"))
            {
                if(gen_msgid != SCH_CMD_MID)
                {
                    gen_msgid = SCH_CMD_MID;
                    gen_fnccode = 0;
                }

                sprintf(ComboboxLabel, "##ComboCMDGEN%d", gen_msgid);
                if (ImGui::BeginCombo(ComboboxLabel, SCHCMDLabels[gen_fnccode]))
                {
                    for (int i = 0; i < 128; i++)
                    {
                        if(strlen(SCHCMDLabels[i]) == 0)
                            continue;
                        else
                        {
                            bool SelectedGen = (gen_fnccode == i);
                            if (ImGui::Selectable(SCHCMDLabels[i], SelectedGen))
                            {
                                if(gen_fnccode != i)
                                {
                                    gen_fnccode = i;
                                }
                            }
                            if (SelectedGen)
                                ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                switch(gen_fnccode) {
                case(SCH_ENABLE_CC) :
                {
                    ImGui::Text("SlotNumber : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##sch_en_slotnumber", ImGuiDataType_U16, &command->sch_enablecmd.SlotNumber, NULL, NULL, "%d");
                    ImGui::Text("Entry : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##sch_en_entrynumber", ImGuiDataType_U16, &command->sch_enablecmd.EntryNumber, NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->sch_enablecmd, sizeof(SCH_EntryCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                case(SCH_DISABLE_CC) :
                {
                    ImGui::Text("SlotNumber : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##sch_dis_slotnumber", ImGuiDataType_U16, &command->sch_disablecmd.SlotNumber, NULL, NULL, "%d");
                    ImGui::Text("Entry : ");
                    ImGui::SameLine();
                    ImGui::InputScalar("##sch_dis_entrynumber", ImGuiDataType_U16, &command->sch_disablecmd.EntryNumber, NULL, NULL, "%d");
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->sch_disablecmd, sizeof(SCH_EntryCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                default : {
                    if(CMDDataGenerator(gen_msgid, gen_fnccode, &command->sch_noargscmd, sizeof(SCH_NoArgsCmd_t)) == 0)
                    {
                        ImGui::EndTabItem();
                        ImGui::EndTabBar();
                        ImGui::EndPopup();
                        return false;
                    }
                    break;
                }
                }
                ImGui::EndTabItem();
            }
        }
        ImGui::EndTabBar();
    }
    ImGui::EndPopup();
    return true;

}



bool popup_fds()
{
    ImGui::OpenPopup("FDS Manager");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(640, 300), ImGuiCond_Appearing);
    if ( ImGui::BeginPopupModal("FDS Manager", NULL, ImGuiWindowFlags_AlwaysAutoResize) )
    {
        ImGui::Text("FDS Path");
        ImGui::InputText("##FDSPath", FDSFilePath, sizeof(FDSFilePath), 0, NULL);
        if (ImGui::Button("Load FDS", ImVec2(ImGui::GetContentRegionAvail().x * 0.5, ImGui::GetContentRegionAvail().y)))
        {
            ReadTLELines(FDSFilePath, true);
            if(strlen(State.Fatellites->Name()) != 0)
            {
                State.Fatellites->use = true;
            }
            else
                console.AddLog("[ERROR]##Cannot Load FDS File.");
            // State.Fatellites = new SatelliteObject(Tle(), Yonsei, false);
            SatelliteModelInitialize(-1);
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetContentRegionAvail().y)))
        {
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::EndPopup();
        return true;
    }
}



bool popup_satinfo(int index)
{
    if(index < -1)
        return false;
    else if(index == -1)
        sprintf(SatWindowLabelBuf, "Details : %s", State.Fatellites->Name());
    else
        sprintf(SatWindowLabelBuf, "Details : %s", State.Satellites[index]->Name());

    ImGui::OpenPopup(SatWindowLabelBuf);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(1080, 640), ImGuiCond_Appearing);
    if ( ImGui::BeginPopupModal(SatWindowLabelBuf, NULL, ImGuiWindowFlags_AlwaysAutoResize) )
    {
        if (ImGui::BeginTable("##State.SatellitesInformations", 3, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_ScrollY | ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable, ImVec2(ImGui::GetContentRegionAvail().x, 480)));
        {
            ImGui::TableSetupScrollFreeze(freeze_cols, freeze_rows);
            ImGui::TableSetupColumn("AOS",           ImGuiTableColumnFlags_DefaultSort | ImGuiTableColumnFlags_NoHide, 0.0f);
            ImGui::TableSetupColumn("LOS",       ImGuiTableColumnFlags_NoHide, 0.0f);
            ImGui::TableSetupColumn("Max Elevation",  ImGuiTableColumnFlags_NoHide, 0.0f);
            ImGui::TableHeadersRow();
            
            // ImGui::TableNextColumn();
            if(index == -1)
            {
                for(int i = 0; i < 64; i++)
                {
                    if(State.Fatellites->_max_elevation[i] <= 0.0f || State.Satellites[i]->_max_elevation[i] > 90.0f)
                        break;
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%d/%d %d:%d:%d",   State.Fatellites->_nextaos[i].AddHours(9).Month(), 
                                                    State.Fatellites->_nextaos[i].AddHours(9).Day(),
                                                    State.Fatellites->_nextaos[i].AddHours(9).Hour(),
                                                    State.Fatellites->_nextaos[i].AddHours(9).Minute(),
                                                    State.Fatellites->_nextaos[i].AddHours(9).Second());
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%d/%d %d:%d:%d",   State.Fatellites->_nextlos[i].AddHours(9).Month(), 
                                                    State.Fatellites->_nextlos[i].AddHours(9).Day(),
                                                    State.Fatellites->_nextlos[i].AddHours(9).Hour(),
                                                    State.Fatellites->_nextlos[i].AddHours(9).Minute(),
                                                    State.Fatellites->_nextlos[i].AddHours(9).Second());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.2f", State.Fatellites->_max_elevation[i] * RAD_TO_DEG);
                }
            }
            else
            {
                for(int i = 0; i < 64; i++)
                {
                    if(State.Satellites[index]->_max_elevation[i] <= 0.0f || State.Satellites[index]->_max_elevation[i] > 90.0f)
                        break;
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::Text("%d/%d %d:%d:%d",   State.Satellites[index]->_nextaos[i].AddHours(9).Month(), 
                                                    State.Satellites[index]->_nextaos[i].AddHours(9).Day(),
                                                    State.Satellites[index]->_nextaos[i].AddHours(9).Hour(),
                                                    State.Satellites[index]->_nextaos[i].AddHours(9).Minute(),
                                                    State.Satellites[index]->_nextaos[i].AddHours(9).Second());
                    ImGui::TableSetColumnIndex(1);
                    ImGui::Text("%d/%d %d:%d:%d",   State.Satellites[index]->_nextlos[i].AddHours(9).Month(), 
                                                    State.Satellites[index]->_nextlos[i].AddHours(9).Day(),
                                                    State.Satellites[index]->_nextlos[i].AddHours(9).Hour(),
                                                    State.Satellites[index]->_nextlos[i].AddHours(9).Minute(),
                                                    State.Satellites[index]->_nextlos[i].AddHours(9).Second());
                    ImGui::TableSetColumnIndex(2);
                    ImGui::Text("%.2f", State.Satellites[index]->_max_elevation[i] * RAD_TO_DEG);
                }
            }
            ImGui::EndTable();
            
        }
        if (ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, ImGui::GetFontSize() * 1.5)))
        {
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::EndPopup();
        return true;
    }
}

bool popup_s_band()
{
    ImGui::OpenPopup("S-Band Manager");
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(720, 540), ImGuiCond_Appearing);
    if ( ImGui::BeginPopupModal("S-Band Manager", NULL, ImGuiWindowFlags_AlwaysAutoResize) )
    {
        ImGui::Text("S-Band GS PC Remote Control");
        
        ImGui::InputText("##s-bandcmd", inputbuf, sizeof(inputbuf), NULL, NULL);
        ImGui::SameLine();
        if(ImGui::Button("Send##s-band"))
        {
            sgs->cmd(inputbuf);
        }
        ImGui::BeginChild("##s-band", ImVec2(ImGui::GetContentRegionAvail().x, 650), true, mim_winflags);
        ImGui::Text(sgs->buffer);
        ImGui::EndChild();
        if (ImGui::Button("Close", ImVec2(ImGui::GetContentRegionAvail().x, 40)))
        {
            ImGui::EndPopup();
            ImGui::CloseCurrentPopup();
            return false;
        }
        ImGui::EndPopup();
        return true;
    }
}