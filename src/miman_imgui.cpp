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

char Templabels[128][64];

char AmpTimeBuf[32];

bool show_typingWindow = false;
typedef struct CMDInfo {
    uint16_t msgid;
    uint16_t fnccode;
}__attribute__((packed));

//CMDInfo *info = new CMDInfo();

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
    }
    if(show_typingWindow) {
        ImGui::Begin("##msgid_fnccode_TypeWindow");
        
        if (ImGui::BeginCombo("##Select CMD type", Templabels[gen_fnccode]))
        {
            for (int i = 0; i < 128; i++)
            {
                if(strlen(Templabels[i]) == 0)
                    continue;
                else                    {
                    bool SelectedGen = (gen_fnccode == i);
                    if (ImGui::Selectable(Templabels[i], SelectedGen))
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
            case 0: {  //no arg
                static uint16_t msgid=0;
                static uint8_t fnccode=0;
                
                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
                //ImGui::InputScalar("arg",ImGuiDataType_U8, &arg);
                if(ImGui::Button("Generate CMD")) {
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+8); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = 8;
                    memset(TestPacket->Data,0,TestPacket->Length);
                    uint8_t cmd[8] = {0,};
                    msgid = htons(msgid);
                    memcpy(cmd, &msgid, sizeof(uint16_t));
                    memcpy(cmd+6, &fnccode, sizeof(uint8_t));
                    cmd[2] = 0xc0;
                    cmd[3] = 0x00;
                    cmd[4] = 0x00;
                    cmd[5] = 0x01;
                    uint16_t len = 8;
                    const uint8_t *byteptr = cmd;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    cmd[7]=checksum; // checksum
                    memcpy(TestPacket->Data,cmd,sizeof(cmd));
                    for(int i=0; i<TestPacket->Length; i++) {
                        printf("0x%x ",cmd[i]);
                    } printf("\n");
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                
                break;
            }
            case 1:  {// u8
                static uint16_t msgid=0;
                static uint8_t fnccode=0;
                static uint8_t arg=0;
                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
                ImGui::InputScalar("arg u8",ImGuiDataType_U8, &arg);
                if(ImGui::Button("Generate CMD")) {
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+CFE_SB_CMD_HDR_SIZE+sizeof(uint8_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = CFE_SB_CMD_HDR_SIZE+sizeof(uint8_t);
                    memset(TestPacket->Data,0,TestPacket->Length);
                    uint8_t cmd2[CFE_SB_CMD_HDR_SIZE+sizeof(uint8_t)] = {0,};
                    msgid = htons(msgid);
                    memcpy(cmd2, &msgid, sizeof(uint16_t));
                    memcpy(cmd2 + 6, &fnccode, sizeof(uint8_t));
                    memcpy(cmd2 + 8, &arg, sizeof(uint8_t));
                    cmd2[2] = 0xc0;
                    cmd2[3] = 0x00;
                    cmd2[4] = 0x00;
                    cmd2[5] = 0x02;
                    uint16_t len = 9;
                    const uint8_t *byteptr = cmd2;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    cmd2[7]=checksum; // checksum
                    memcpy(TestPacket->Data,cmd2,sizeof(cmd2));
                    for(int i=0; i<TestPacket->Length; i++) {
                        if(i < 8) {
                            printf("0x%x ",cmd2[i]);
                        }
                        else {
                            printf("%u", cmd2[i]);
                        }
                    } printf("\n");
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                break;
            }
            case 2:  {// u16
                static uint16_t msgid=0;
                static uint8_t fnccode=0;
                static uint16_t arg=0;
                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
                ImGui::InputScalar("arg u16",ImGuiDataType_U16, &arg);
                if(ImGui::Button("Generate CMD")) { 
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+CFE_SB_CMD_HDR_SIZE+sizeof(uint16_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = CFE_SB_CMD_HDR_SIZE+sizeof(uint8_t);
                    memset(TestPacket->Data,0,TestPacket->Length);
                    uint8_t cmd2[CFE_SB_CMD_HDR_SIZE+sizeof(uint8_t)] = {0,};
                    msgid = htons(msgid);
                    memcpy(cmd2, &msgid, sizeof(uint16_t));
                    memcpy(cmd2 + 6, &fnccode, sizeof(uint8_t));
                    memcpy(cmd2 + 8, &arg, sizeof(uint16_t));
                    cmd2[2] = 0xc0;
                    cmd2[3] = 0x00;
                    cmd2[4] = 0x00;
                    cmd2[5] = 0x03;
                    uint16_t len = 10;
                    const uint8_t *byteptr = cmd2;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    cmd2[7]=checksum; // checksum
                    memcpy(TestPacket->Data,cmd2,sizeof(cmd2));
                    for(int i=0; i<TestPacket->Length; i++) {
                        if(i < 8) {
                            printf("0x%x ",cmd2[i]);
                        }
                        else {
                            printf("%u", arg);
                        }
                    } printf("\n");
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                break;
            }
            case 3:  {// u32
                static uint16_t msgid=0;
                static uint8_t fnccode=0;
                static uint32_t arg=0;
                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
                ImGui::InputScalar("arg u32",ImGuiDataType_U32, &arg);
                if(ImGui::Button("Generate CMD")) {
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+CFE_SB_CMD_HDR_SIZE+sizeof(uint32_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = CFE_SB_CMD_HDR_SIZE+sizeof(uint32_t);
                    memset(TestPacket->Data,0,TestPacket->Length);
                    uint8_t cmd2[CFE_SB_CMD_HDR_SIZE+sizeof(uint32_t)] = {0,};
                    msgid = htons(msgid);
                    memcpy(cmd2, &msgid, sizeof(uint16_t));
                    memcpy(cmd2 + 6, &fnccode, sizeof(uint8_t));
                    memcpy(cmd2 + 8, &arg, sizeof(uint32_t));
                    cmd2[2] = 0xc0;
                    cmd2[3] = 0x00;
                    cmd2[4] = 0x00;
                    cmd2[5] = 0x05;
                    uint16_t len = 12;
                    const uint8_t *byteptr = cmd2;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    cmd2[7]=checksum; // checksum
                    memcpy(TestPacket->Data,cmd2,sizeof(cmd2));
                    for(int i=0; i<TestPacket->Length; i++) {
                        printf("0x%x ",cmd2[i]);
                    } printf("\n");
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                break;
            }
            case 4:  {// u64
                static uint16_t msgid=0;
                static uint8_t fnccode=0;
                static uint64_t arg=0;
                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
                ImGui::InputScalar("arg u64",ImGuiDataType_U64, &arg);
                if(ImGui::Button("Generate CMD")) {
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+CFE_SB_CMD_HDR_SIZE+sizeof(uint64_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = CFE_SB_CMD_HDR_SIZE+sizeof(uint64_t);
                    memset(TestPacket->Data,0,TestPacket->Length);
                    uint8_t cmd2[CFE_SB_CMD_HDR_SIZE+sizeof(uint64_t)] = {0,};
                    msgid = htons(msgid);
                    memcpy(cmd2, &msgid, sizeof(uint16_t));
                    memcpy(cmd2 + 6, &fnccode, sizeof(uint8_t));
                    memcpy(cmd2 + 8, &arg, sizeof(uint64_t));
                    cmd2[2] = 0xc0;
                    cmd2[3] = 0x00;
                    cmd2[4] = 0x00;
                    cmd2[5] = 0x09;
                    uint16_t len = CFE_SB_CMD_HDR_SIZE+sizeof(uint64_t);
                    const uint8_t *byteptr = cmd2;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    cmd2[7]=checksum; // checksum
                    memcpy(TestPacket->Data,cmd2,sizeof(cmd2));
                    for(int i=0; i<TestPacket->Length; i++) {
                        printf("0x%x ",cmd2[i]);
                    } printf("\n");
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                break;
            }
            case 5:  {// s8
                static uint16_t msgid=0;
                static uint8_t fnccode=0;
                static int8_t arg=0;
                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
                ImGui::InputScalar("arg s8",ImGuiDataType_S8, &arg);
                if(ImGui::Button("Generate CMD")) {
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+CFE_SB_CMD_HDR_SIZE+sizeof(int8_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = CFE_SB_CMD_HDR_SIZE+sizeof(int8_t);
                    memset(TestPacket->Data,0,TestPacket->Length);
                    uint8_t cmd2[CFE_SB_CMD_HDR_SIZE+sizeof(int8_t)] = {0,};
                    msgid = htons(msgid);
                    memcpy(cmd2, &msgid, sizeof(uint16_t));
                    memcpy(cmd2 + 6, &fnccode, sizeof(uint8_t));
                    memcpy(cmd2 + 8, &arg, sizeof(int8_t));
                    cmd2[2] = 0xc0;
                    cmd2[3] = 0x00;
                    cmd2[4] = 0x00;
                    cmd2[5] = 0x02;
                    uint16_t len = CFE_SB_CMD_HDR_SIZE+sizeof(int8_t);
                    const uint8_t *byteptr = cmd2;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    cmd2[7]=checksum; // checksum
                    memcpy(TestPacket->Data,cmd2,sizeof(cmd2));
                    for(int i=0; i<TestPacket->Length; i++) {
                        printf("0x%x ",cmd2[i]);
                    } printf("\n");
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                break;
            }
            case 6:  {// s16
                static uint16_t msgid=0;
                static uint8_t fnccode=0;
                static int16_t arg=0;
                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
                ImGui::InputScalar("arg s16",ImGuiDataType_S16, &arg);
                if(ImGui::Button("Generate CMD")) {
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+CFE_SB_CMD_HDR_SIZE+sizeof(int16_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = CFE_SB_CMD_HDR_SIZE+sizeof(int16_t);
                    memset(TestPacket->Data,0,TestPacket->Length);
                    uint8_t cmd2[CFE_SB_CMD_HDR_SIZE+sizeof(int16_t)] = {0,};
                    msgid = htons(msgid);
                    memcpy(cmd2, &msgid, sizeof(uint16_t));
                    memcpy(cmd2 + 6, &fnccode, sizeof(uint8_t));
                    memcpy(cmd2 + 8, &arg, sizeof(int16_t));
                    cmd2[2] = 0xc0;
                    cmd2[3] = 0x00;
                    cmd2[4] = 0x00;
                    cmd2[5] = 0x03;
                    uint16_t len = 10;
                    const uint8_t *byteptr = cmd2;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    cmd2[7]=checksum; // checksum
                    memcpy(TestPacket->Data,cmd2,sizeof(cmd2));
                    for(int i=0; i<TestPacket->Length; i++) {
                        printf("0x%x ",cmd2[i]);
                    } printf("\n");
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                break;
            }
            case 7:  {// s32
                static uint16_t msgid=0;
                static uint8_t fnccode=0;
                static int32_t arg=0;
                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
                ImGui::InputScalar("arg s32",ImGuiDataType_S32, &arg);
                if(ImGui::Button("Generate CMD")) {
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+12); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = 12;
                    memset(TestPacket->Data,0,TestPacket->Length);
                    uint8_t cmd2[12] = {0,};
                    msgid = htons(msgid);
                    memcpy(cmd2, &msgid, sizeof(uint16_t));
                    memcpy(cmd2 + 6, &fnccode, sizeof(uint8_t));
                    memcpy(cmd2 + 8, &arg, sizeof(int32_t));
                    cmd2[2] = 0xc0;
                    cmd2[3] = 0x00;
                    cmd2[4] = 0x00;
                    cmd2[5] = 0x05;
                    uint16_t len = 12;
                    const uint8_t *byteptr = cmd2;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    cmd2[7]=checksum; // checksum
                    memcpy(TestPacket->Data,cmd2,sizeof(cmd2));
                    for(int i=0; i<TestPacket->Length; i++) {
                        printf("0x%x ",cmd2[i]);
                    } printf("\n");
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                break;
            }
            case 8:  {// s64
                static uint16_t msgid=0;
                static uint8_t fnccode=0;
                static int64_t arg=0;
                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
                ImGui::InputScalar("arg s64",ImGuiDataType_S64, &arg);
                if(ImGui::Button("Generate CMD")) {
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+16); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = 16;
                    memset(TestPacket->Data,0,TestPacket->Length);
                    uint8_t cmd2[16] = {0,};
                    msgid = htons(msgid);
                    memcpy(cmd2, &msgid, sizeof(uint16_t));
                    memcpy(cmd2 + 6, &fnccode, sizeof(uint8_t));
                    memcpy(cmd2 + 8, &arg, sizeof(int64_t));
                    cmd2[2] = 0xc0;
                    cmd2[3] = 0x00;
                    cmd2[4] = 0x00;
                    cmd2[5] = 0x09;
                    uint16_t len = 16;
                    const uint8_t *byteptr = cmd2;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    cmd2[7]=checksum; // checksum
                    memcpy(TestPacket->Data,cmd2,sizeof(cmd2));
                    for(int i=0; i<TestPacket->Length; i++) {
                        printf("0x%x ",cmd2[i]);
                    } printf("\n");
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                break;
            }
            case 9:  { // float
                static uint16_t msgid = 0;
                static uint8_t fnccode=0;
                static float arg =0;
                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
                ImGui::InputScalar("arg float",ImGuiDataType_Float, &arg);
                if(ImGui::Button("Generate CMD")) {
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+12); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = 12;
                    memset(TestPacket->Data,0,TestPacket->Length);
                    uint8_t cmd2[12] = {0,};
                    msgid = htons(msgid);
                    memcpy(cmd2, &msgid, sizeof(uint16_t));
                    memcpy(cmd2 + 6, &fnccode, sizeof(uint8_t));
                    memcpy(cmd2 + 8, &arg, sizeof(float));
                    cmd2[2] = 0xc0;
                    cmd2[3] = 0x00;
                    cmd2[4] = 0x00;
                    cmd2[5] = 0x05;
                    uint16_t len = 12;
                    const uint8_t *byteptr = cmd2;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    cmd2[7]=checksum; // checksum
                    memcpy(TestPacket->Data,cmd2,sizeof(cmd2));
                    for(int i=0; i<TestPacket->Length; i++) {
                        printf("0x%x ",cmd2[i]);
                    } printf("\n");
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                break;
            }
// // 여기까지 cc와 crc 순서 바꾸기 완료
//             }
//             case 10: { //u32u8bool
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
//                 ImGui::InputScalar("u32",ImGuiDataType_U32, &command->u32u8bool.Timeout);
//                 ImGui::InputScalar("u8",ImGuiDataType_U8, &command->u32u8bool.Channel);
//                 ImGui::InputScalar("bool",ImGuiDataType_U8, &command->u32u8bool.Status);
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(u32u8bool_t)-7};
//                     memcpy(command->u32u8bool.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->u32u8bool.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->u32u8bool.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->u32u8bool.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(u32u8bool_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(u32u8bool_t);
                    
//                     uint16_t len = sizeof(u32u8bool_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->u32u8bool;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->u32u8bool.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->u32u8bool, sizeof(u32u8bool_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                                 command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                                 command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);
//                 break;
//             }
//             case 11: { // u32u8
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
//                 ImGui::InputScalar("u32",ImGuiDataType_U32, &command->u32u8.Timeout);
//                 ImGui::InputScalar("u8",ImGuiDataType_U8, &command->u32u8.Channel);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(u32u8_t)-7};
//                     memcpy(command->u32u8.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->u32u8.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->u32u8.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->u32u8.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(u32u8_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(u32u8_t);
                    
//                     uint16_t len = sizeof(u32u8_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->u32u8;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->u32u8.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->u32u8, sizeof(u32u8_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u",command->u32u8.CmdHeader[0],
//                                 command->u32u8.CmdHeader[1], command->u32u8.CmdHeader[2], command->u32u8.CmdHeader[3], command->u32u8.CmdHeader[4], command->u32u8.CmdHeader[5], command->u32u8.CmdHeader[6], command->u32u8.CmdHeader[7],
//                                 command->u32u8.Timeout, command->u32u8.Channel);
//                 break;
//             }
//             case 12: { //u32u32u8u8
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);
//                 ImGui::InputScalar("size",ImGuiDataType_U32, &command->u32u32u8u8.Size);
//                 ImGui::InputScalar("timeout",ImGuiDataType_U32, &command->u32u32u8u8.Timeout);
//                 ImGui::InputScalar("node",ImGuiDataType_U8, &command->u32u32u8u8.Node);
//                 ImGui::InputScalar("tableid",ImGuiDataType_U8, &command->u32u32u8u8.TableId);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(u32u32u8u8_t)-7};
//                     memcpy(command->u32u32u8u8.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->u32u32u8u8.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->u32u32u8u8.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->u32u32u8u8.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(u32u32u8u8_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(u32u32u8u8_t);
                    
//                     uint16_t len = sizeof(u32u32u8u8_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->u32u32u8u8;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->u32u32u8u8.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->u32u8, sizeof(u32u32u8u8_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u %u",command->u32u32u8u8.CmdHeader[0],
//                                 command->u32u32u8u8.CmdHeader[1], command->u32u32u8u8.CmdHeader[2], command->u32u32u8u8.CmdHeader[3], command->u32u32u8u8.CmdHeader[4], command->u32u32u8u8.CmdHeader[5], command->u32u32u8u8.CmdHeader[6], command->u32u32u8u8.CmdHeader[7],
//                                 command->u32u32u8u8.Size, command->u32u32u8u8.Timeout, command->u32u32u8u8.Node, command->u32u32u8u8.TableId);
//                 break;
//             }
//             case 13: { //u32u8u8u8
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("timeout u32",ImGuiDataType_U32, &command->u32u8u8u8.Timeout);
//                 ImGui::InputScalar("node u8",ImGuiDataType_U8, &command->u32u8u8u8.Node);
//                 ImGui::InputScalar("tableid u8",ImGuiDataType_U8, &command->u32u8u8u8.TableId);
//                 ImGui::InputScalar("destination u8",ImGuiDataType_U8, &command->u32u8u8u8.Destination);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(u32u8u8u8_t)-7};
//                     memcpy(command->u32u8u8u8.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->u32u8u8u8.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->u32u8u8u8.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->u32u8u8u8.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(u32u8u8u8_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(u32u8u8u8_t);
                    
//                     uint16_t len = sizeof(u32u8u8u8_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->u32u8u8u8;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->u32u8u8u8.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->u32u8, sizeof(u32u8u8u8_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u %u",command->u32u8u8u8.CmdHeader[0],
//                                 command->u32u8u8u8.CmdHeader[1], command->u32u8u8u8.CmdHeader[2], command->u32u8u8u8.CmdHeader[3], command->u32u8u8u8.CmdHeader[4], command->u32u8u8u8.CmdHeader[5], command->u32u8u8u8.CmdHeader[6], command->u32u8u8u8.CmdHeader[7],
//                                 command->u32u8u8u8.Timeout, command->u32u8u8u8.Node, command->u32u8u8u8.TableId, command->u32u8u8u8.Destination);
//                 break;
//             }
//             case 14: { //u32u32u8u8u16u8
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("timeout u32",ImGuiDataType_U32, &command->u32u32u8u8u16u8.Timeout);
//                 ImGui::InputScalar("size u32",ImGuiDataType_U32, &command->u32u32u8u8u16u8.Size);
//                 ImGui::InputScalar("node u8",ImGuiDataType_U8, &command->u32u32u8u8u16u8.Node);
//                 ImGui::InputScalar("tableid u8",ImGuiDataType_U8, &command->u32u32u8u8u16u8.TableId);
//                 ImGui::InputScalar("address u16",ImGuiDataType_U16, &command->u32u32u8u8u16u8.Address);
//                 ImGui::InputScalar("type u8",ImGuiDataType_U8, &command->u32u32u8u8u16u8.Type);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(u32u32u8u8u16u8_t)-7};
//                     memcpy(command->u32u32u8u8u16u8.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->u32u32u8u8u16u8.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->u32u32u8u8u16u8.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->u32u32u8u8u16u8.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(u32u32u8u8u16u8_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(u32u32u8u8u16u8_t);
                    
//                     uint16_t len = sizeof(u32u32u8u8u16u8_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->u32u32u8u8u16u8;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->u32u32u8u8u16u8.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->u32u8, sizeof(u32u32u8u8u16u8_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u %u %u %u",command->u32u32u8u8u16u8.CmdHeader[0],
//                                 command->u32u32u8u8u16u8.CmdHeader[1], command->u32u32u8u8u16u8.CmdHeader[2], command->u32u32u8u8u16u8.CmdHeader[3], command->u32u32u8u8u16u8.CmdHeader[4], command->u32u32u8u8u16u8.CmdHeader[5], command->u32u32u8u8u16u8.CmdHeader[6], command->u32u32u8u8u16u8.CmdHeader[7],
//                                 command->u32u32u8u8u16u8.Timeout, command->u32u32u8u8u16u8.Size, command->u32u32u8u8u16u8.Node, command->u32u32u8u8u16u8.TableId, command->u32u32u8u8u16u8.Address, command->u32u32u8u8u16u8.Type);
//                 break;
//             }
//             case 15: { //u32u32u32u8u8u16u8
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("timeout u32",ImGuiDataType_U32, &command->u32u32u32u8u8u16u8.Timeout);
//                 ImGui::InputScalar("itemsize u32",ImGuiDataType_U32, &command->u32u32u32u8u8u16u8.ItemSize);
//                 ImGui::InputScalar("itemcount u32",ImGuiDataType_U32, &command->u32u32u32u8u8u16u8.ItemCount);
//                 ImGui::InputScalar("node u8",ImGuiDataType_U8, &command->u32u32u32u8u8u16u8.Node);
//                 ImGui::InputScalar("tableid u8",ImGuiDataType_U8, &command->u32u32u32u8u8u16u8.TableId);
//                 ImGui::InputScalar("address u16",ImGuiDataType_U16, &command->u32u32u32u8u8u16u8.Address);
//                 ImGui::InputScalar("type u8",ImGuiDataType_U8, &command->u32u32u32u8u8u16u8.Type);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(u32u32u32u8u8u16u8_t)-7};
//                     memcpy(command->u32u32u32u8u8u16u8.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->u32u32u32u8u8u16u8.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->u32u32u32u8u8u16u8.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->u32u32u32u8u8u16u8.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(u32u32u32u8u8u16u8_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(u32u32u32u8u8u16u8_t);
                    
//                     uint16_t len = sizeof(u32u32u32u8u8u16u8_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->u32u32u32u8u8u16u8;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->u32u32u32u8u8u16u8.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->u32u32u32u8u8u16u8, sizeof(u32u32u32u8u8u16u8_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u %u %u %u %u",command->u32u32u32u8u8u16u8.CmdHeader[0],
//                                 command->u32u32u32u8u8u16u8.CmdHeader[1], command->u32u32u32u8u8u16u8.CmdHeader[2], command->u32u32u32u8u8u16u8.CmdHeader[3], command->u32u32u32u8u8u16u8.CmdHeader[4], command->u32u32u32u8u8u16u8.CmdHeader[5], command->u32u32u32u8u8u16u8.CmdHeader[6], command->u32u32u32u8u8u16u8.CmdHeader[7],
//                                 command->u32u32u32u8u8u16u8.Timeout, command->u32u32u32u8u8u16u8.ItemSize, command->u32u32u32u8u8u16u8.ItemCount, command->u32u32u32u8u8u16u8.Node, command->u32u32u32u8u8u16u8.TableId, command->u32u32u32u8u8u16u8.Address, command->u32u32u32u8u8u16u8.Type);
//                 break;
//             }
//             case 16: { //u32bool13
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("timeout u32",ImGuiDataType_U32, &command->u32bool13.Timeout);
//                 for (uint8_t i =0; i<13; i++) {
//                     char temp[10] ={0,};
//                     sprintf(temp, "channel status bool%d",i);
//                     ImGui::Checkbox(temp, &command->u32bool13.ChannelStatus[i]);
//                 }

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(u32bool13_t)-7};
//                     memcpy(command->u32bool13.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->u32bool13.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->u32bool13.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->u32bool13.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(u32bool13_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(u32bool13_t);
                    
//                     uint16_t len = sizeof(u32bool13_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->u32bool13;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->u32bool13.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->u32bool13, sizeof(u32bool13_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %d %d %d %d %d %d %d %d %d %d %d %d %d",command->u32bool13.CmdHeader[0],
//                                 command->u32bool13.CmdHeader[1], command->u32bool13.CmdHeader[2], command->u32bool13.CmdHeader[3], command->u32bool13.CmdHeader[4], command->u32bool13.CmdHeader[5], command->u32bool13.CmdHeader[6], command->u32bool13.CmdHeader[7],
//                                 command->u32bool13.Timeout, command->u32bool13.ChannelStatus[0], command->u32bool13.ChannelStatus[1], command->u32bool13.ChannelStatus[2], command->u32bool13.ChannelStatus[3], command->u32bool13.ChannelStatus[4], command->u32bool13.ChannelStatus[5],
//                                 command->u32bool13.ChannelStatus[6], command->u32bool13.ChannelStatus[7], command->u32bool13.ChannelStatus[8], command->u32bool13.ChannelStatus[9], command->u32bool13.ChannelStatus[10], command->u32bool13.ChannelStatus[11], command->u32bool13.ChannelStatus[12], command->u32bool13.ChannelStatus[13]);
//                 break;
//             }
//             case 17: { //u32bool9
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("timeout u32",ImGuiDataType_U32, &command->u32bool9.Timeout);
//                 for (uint8_t i =0; i<9; i++) {
//                     char temp[10] ={0,};
//                     sprintf(temp, "channel status bool%d",i);
//                     ImGui::Checkbox(temp, &command->u32bool9.ChannelStatus[i]);
//                 }

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(u32bool9_t)-7};
//                     memcpy(command->u32bool9.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->u32bool9.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->u32bool9.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->u32bool9.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(u32bool9_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(u32bool9_t);
                    
//                     uint16_t len = sizeof(u32bool9_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->u32bool9;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->u32bool9.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->u32bool9, sizeof(u32bool9_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %d %d %d %d %d %d %d %d %d",command->u32bool9.CmdHeader[0],
//                                 command->u32bool9.CmdHeader[1], command->u32bool9.CmdHeader[2], command->u32bool9.CmdHeader[3], command->u32bool9.CmdHeader[4], command->u32bool9.CmdHeader[5], command->u32bool9.CmdHeader[6], command->u32bool9.CmdHeader[7],
//                                 command->u32bool9.Timeout, command->u32bool9.ChannelStatus[0], command->u32bool9.ChannelStatus[1], command->u32bool9.ChannelStatus[2], command->u32bool9.ChannelStatus[3], command->u32bool9.ChannelStatus[4], command->u32bool9.ChannelStatus[5],
//                                 command->u32bool9.ChannelStatus[6],command->u32bool9.ChannelStatus[7],command->u32bool9.ChannelStatus[8]);
//                 break;
//             }
//             case 18: { //grx assemblepublishcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("mid",ImGuiDataType_S16, &command->grxassemblepublishcmd.param.mid);
//                 ImGui::InputScalar("bodylength",ImGuiDataType_S16, &command->grxassemblepublishcmd.param.bodylength);
//                 ImGui::InputText("body (??)", (char *)&command->grxassemblepublishcmd.param.body,sizeof(command->grxassemblepublishcmd.param.body));

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_AssemblePublishCmd_t)-7};
//                     memcpy(command->grxassemblepublishcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxassemblepublishcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxassemblepublishcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxassemblepublishcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_AssemblePublishCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_AssemblePublishCmd_t);
                    
//                     uint16_t len = sizeof(GRX_AssemblePublishCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxassemblepublishcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxassemblepublishcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxassemblepublishcmd, sizeof(GRX_AssemblePublishCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 //ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u %u %u %u %u",command->grxassemblepublishcmd.CmdHeader[0],
//                 //                command->grxassemblepublishcmd.CmdHeader[1], command->grxassemblepublishcmd.CmdHeader[2], command->grxassemblepublishcmd.CmdHeader[3], command->grxassemblepublishcmd.CmdHeader[4], command->grxassemblepublishcmd.CmdHeader[5], command->grxassemblepublishcmd.CmdHeader[6], command->grxassemblepublishcmd.CmdHeader[7],
//                 //                command->grxassemblepublishcmd.Timeout, command->grxassemblepublishcmd.ItemSize, command->grxassemblepublishcmd.ItemCount, command->grxassemblepublishcmd.Node, command->grxassemblepublishcmd.TableId, command->grxassemblepublishcmd.Address, command->grxassemblepublishcmd.Type);
//                 break;
//             }
//             case 19: { //grx cmdlog
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &command->grxcmdlogcmd.param.msgId);
//                 ImGui::InputScalar("type",ImGuiDataType_U8, &command->grxcmdlogcmd.param.type);
//                 ImGui::InputScalar("port", ImGuiDataType_U32, &command->grxcmdlogcmd.param.port);
//                 ImGui::InputScalar("trigger", ImGuiDataType_U32, &command->grxcmdlogcmd.param.trigger);
//                 ImGui::InputScalar("hold", ImGuiDataType_U32, &command->grxcmdlogcmd.param.hold);
//                 ImGui::InputScalar("period", ImGuiDataType_Double, &command->grxcmdlogcmd.param.period);
//                 ImGui::InputScalar("offset", ImGuiDataType_Double, &command->grxcmdlogcmd.param.offset);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_CmdLOGCmd_t)-7};
//                     memcpy(command->grxcmdlogcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_CmdLOGCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_CmdLOGCmd_t);
                    
//                     uint16_t len = sizeof(GRX_CmdLOGCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxcmdlogcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxcmdlogcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxcmdlogcmd, sizeof(GRX_CmdLOGCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u %u %u %u %u",command->grxcmdlogcmd.CmdHeader[0],
//                                 command->grxcmdlogcmd.CmdHeader[1], command->grxcmdlogcmd.CmdHeader[2], command->grxcmdlogcmd.CmdHeader[3], command->grxcmdlogcmd.CmdHeader[4], command->grxcmdlogcmd.CmdHeader[5], command->grxcmdlogcmd.CmdHeader[6], command->grxcmdlogcmd.CmdHeader[7],
//                                 command->grxcmdlogcmd.param.msgId, command->grxcmdlogcmd.param.type, command->grxcmdlogcmd.param.port, command->grxcmdlogcmd.param.trigger, 
//                                 command->grxcmdlogcmd.param.hold, command->grxcmdlogcmd.param.period, command->grxcmdlogcmd.param.offset);
//                 break;
//             }
//             case 20: { //grx cmdlogontime
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &command->grxcmdlogontimecmd.param.msgId);
//                 ImGui::InputScalar("port", ImGuiDataType_U32, &command->grxcmdlogontimecmd.param.port);
//                 ImGui::InputScalar("period", ImGuiDataType_Double, &command->grxcmdlogontimecmd.param.period);
//                 ImGui::InputScalar("offset", ImGuiDataType_Double, &command->grxcmdlogontimecmd.param.offset);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_CmdLogOnTimeCmd_t)-7};
//                     memcpy(command->grxcmdlogontimecmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogontimecmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogontimecmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogontimecmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_CmdLogOnTimeCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_CmdLogOnTimeCmd_t);
                    
//                     uint16_t len = sizeof(GRX_CmdLogOnTimeCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxcmdlogontimecmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxcmdlogontimecmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxcmdlogontimecmd, sizeof(GRX_CmdLogOnTimeCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %lf %lf",command->grxcmdlogontimecmd.CmdHeader[0],
//                                 command->grxcmdlogontimecmd.CmdHeader[1], command->grxcmdlogontimecmd.CmdHeader[2], command->grxcmdlogontimecmd.CmdHeader[3], command->grxcmdlogontimecmd.CmdHeader[4], command->grxcmdlogontimecmd.CmdHeader[5], command->grxcmdlogontimecmd.CmdHeader[6], command->grxcmdlogontimecmd.CmdHeader[7],
//                                 command->grxcmdlogontimecmd.param.msgId, command->grxcmdlogontimecmd.param.port, command->grxcmdlogontimecmd.param.period, command->grxcmdlogontimecmd.param.offset);
//                 break;
//             }
//             case 21: { //grxcmdlogonchanged
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &command->grxcmdlogonchangedcmd.param.msgId);
//                 ImGui::InputScalar("port", ImGuiDataType_U32, &command->grxcmdlogonchangedcmd.param.port);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_CmdLogOnChangedCmd_t)-7};
//                     memcpy(command->grxcmdlogonchangedcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogonchangedcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogonchangedcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogonchangedcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_CmdLogOnChangedCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_CmdLogOnChangedCmd_t);
                    
//                     uint16_t len = sizeof(GRX_CmdLogOnChangedCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxcmdlogonchangedcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxcmdlogonchangedcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxcmdlogonchangedcmd, sizeof(GRX_CmdLogOnChangedCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u",command->grxcmdlogonchangedcmd.CmdHeader[0],
//                                 command->grxcmdlogonchangedcmd.CmdHeader[1], command->grxcmdlogonchangedcmd.CmdHeader[2], command->grxcmdlogonchangedcmd.CmdHeader[3], command->grxcmdlogonchangedcmd.CmdHeader[4], command->grxcmdlogonchangedcmd.CmdHeader[5], command->grxcmdlogonchangedcmd.CmdHeader[6], command->grxcmdlogonchangedcmd.CmdHeader[7],
//                                 command->grxcmdlogonchangedcmd.param.msgId, command->grxcmdlogonchangedcmd.param.port);
//                 break;
//             }
//             case 22: { //grxcmdlogonnew
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &command->grxcmdlogonnewcmd.param.msgId);
//                 ImGui::InputScalar("port", ImGuiDataType_U32, &command->grxcmdlogonnewcmd.param.port);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_CmdLogOnNewCmd_t)-7};
//                     memcpy(command->grxcmdlogonnewcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogonnewcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogonnewcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxcmdlogonnewcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_CmdLogOnNewCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_CmdLogOnNewCmd_t);
                    
//                     uint16_t len = sizeof(GRX_CmdLogOnNewCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxcmdlogonnewcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxcmdlogonnewcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxcmdlogonnewcmd, sizeof(GRX_CmdLogOnNewCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u",command->grxcmdlogonnewcmd.CmdHeader[0],
//                                 command->grxcmdlogonnewcmd.CmdHeader[1], command->grxcmdlogonnewcmd.CmdHeader[2], command->grxcmdlogonnewcmd.CmdHeader[3], command->grxcmdlogonnewcmd.CmdHeader[4], command->grxcmdlogonnewcmd.CmdHeader[5], command->grxcmdlogonnewcmd.CmdHeader[6], command->grxcmdlogonnewcmd.CmdHeader[7],
//                                 command->grxcmdlogonnewcmd.param.msgId, command->grxcmdlogonnewcmd.param.port);
//                 break;
//             }
//             case 23: { //grxcmdunlogcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &command->grxcmdunlogcmd.param.msgId);
//                 ImGui::InputScalar("msgtype",ImGuiDataType_U8, &command->grxcmdunlogcmd.param.msgType);
//                 ImGui::InputScalar("port", ImGuiDataType_U32, &command->grxcmdunlogcmd.param.port);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_CmdUnlogCmd_t)-7};
//                     memcpy(command->grxcmdunlogcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxcmdunlogcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxcmdunlogcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxcmdunlogcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_CmdUnlogCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_CmdUnlogCmd_t);
                    
//                     uint16_t len = sizeof(GRX_CmdUnlogCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxcmdunlogcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxcmdunlogcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxcmdunlogcmd, sizeof(GRX_CmdUnlogCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u",command->grxcmdunlogcmd.CmdHeader[0],
//                                 command->grxcmdunlogcmd.CmdHeader[1], command->grxcmdunlogcmd.CmdHeader[2], command->grxcmdunlogcmd.CmdHeader[3], command->grxcmdunlogcmd.CmdHeader[4], command->grxcmdunlogcmd.CmdHeader[5], command->grxcmdunlogcmd.CmdHeader[6], command->grxcmdunlogcmd.CmdHeader[7],
//                                 command->grxcmdunlogcmd.param.msgId, command->grxcmdunlogcmd.param.msgType, command->grxcmdunlogcmd.param.port);
//                 break;
//             }
//             case 24: { //grxcmdunlogallcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("port", ImGuiDataType_U32, &command->grxcmdunlogallcmd.param.port);
//                 ImGui::Checkbox("held",&command->grxcmdunlogallcmd.param.held);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_CmdUnlogAllCmd_t)-7};
//                     memcpy(command->grxcmdunlogallcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxcmdunlogallcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxcmdunlogallcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxcmdunlogallcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_CmdUnlogAllCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_CmdUnlogAllCmd_t);
                    
//                     uint16_t len = sizeof(GRX_CmdUnlogAllCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxcmdunlogallcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxcmdunlogallcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxcmdunlogallcmd, sizeof(GRX_CmdUnlogAllCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %d",command->grxcmdunlogallcmd.CmdHeader[0],
//                                 command->grxcmdunlogallcmd.CmdHeader[1], command->grxcmdunlogallcmd.CmdHeader[2], command->grxcmdunlogallcmd.CmdHeader[3], command->grxcmdunlogallcmd.CmdHeader[4], command->grxcmdunlogallcmd.CmdHeader[5], command->grxcmdunlogallcmd.CmdHeader[6], command->grxcmdunlogallcmd.CmdHeader[7],
//                                 command->grxcmdunlogallcmd.param.port, command->grxcmdunlogallcmd.param.held);
//                 break;
//             }
//             case 25: { //grxcmdelevationcutoffcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("constellation", ImGuiDataType_U32, &command->grxcmdelevationcutoffcmd.param.constellation);
//                 ImGui::InputScalar("cutoff", ImGuiDataType_Float, &command->grxcmdelevationcutoffcmd.param.cutoff);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_CmdElevationCutoffCmd_t)-7};
//                     memcpy(command->grxcmdelevationcutoffcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxcmdelevationcutoffcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxcmdelevationcutoffcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxcmdelevationcutoffcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_CmdElevationCutoffCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_CmdElevationCutoffCmd_t);
                    
//                     uint16_t len = sizeof(GRX_CmdElevationCutoffCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxcmdelevationcutoffcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxcmdelevationcutoffcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxcmdelevationcutoffcmd, sizeof(GRX_CmdElevationCutoffCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %f",command->grxcmdelevationcutoffcmd.CmdHeader[0],
//                                 command->grxcmdelevationcutoffcmd.CmdHeader[1], command->grxcmdelevationcutoffcmd.CmdHeader[2], command->grxcmdelevationcutoffcmd.CmdHeader[3], command->grxcmdelevationcutoffcmd.CmdHeader[4], command->grxcmdelevationcutoffcmd.CmdHeader[5], command->grxcmdelevationcutoffcmd.CmdHeader[6], command->grxcmdelevationcutoffcmd.CmdHeader[7],
//                                 command->grxcmdelevationcutoffcmd.param.constellation, command->grxcmdelevationcutoffcmd.param.cutoff);
//                 break;
//             }
//             case 26: { //grxcmdinterfacemodecmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("port", ImGuiDataType_U32, &command->grxcmdinterfacemodecmd.param.port);
//                 ImGui::InputScalar("rxtype", ImGuiDataType_U32, &command->grxcmdinterfacemodecmd.param.rxType);
//                 ImGui::InputScalar("txtype", ImGuiDataType_U32, &command->grxcmdinterfacemodecmd.param.txType);
//                 ImGui::InputScalar("response", ImGuiDataType_U32, &command->grxcmdinterfacemodecmd.param.responses);
                

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_CmdInterfaceModeCmd_t)-7};
//                     memcpy(command->grxcmdinterfacemodecmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxcmdinterfacemodecmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxcmdinterfacemodecmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxcmdinterfacemodecmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_CmdInterfaceModeCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_CmdInterfaceModeCmd_t);
                    
//                     uint16_t len = sizeof(GRX_CmdInterfaceModeCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxcmdinterfacemodecmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxcmdinterfacemodecmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxcmdinterfacemodecmd, sizeof(GRX_CmdInterfaceModeCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u %u",command->grxcmdinterfacemodecmd.CmdHeader[0],
//                                 command->grxcmdinterfacemodecmd.CmdHeader[1], command->grxcmdinterfacemodecmd.CmdHeader[2], command->grxcmdinterfacemodecmd.CmdHeader[3], command->grxcmdinterfacemodecmd.CmdHeader[4], command->grxcmdinterfacemodecmd.CmdHeader[5], command->grxcmdinterfacemodecmd.CmdHeader[6], command->grxcmdinterfacemodecmd.CmdHeader[7],
//                                 command->grxcmdinterfacemodecmd.param.port, command->grxcmdinterfacemodecmd.param.rxType, command->grxcmdinterfacemodecmd.param.txType, command->grxcmdinterfacemodecmd.param.responses);
//                 break;
//             }
//             case 27: { //grxcmdserialconfigcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("port", ImGuiDataType_U32, &command->grxcmdserialconfigcmd.param.port);
//                 ImGui::InputScalar("baud", ImGuiDataType_U32, &command->grxcmdserialconfigcmd.param.baud);
//                 ImGui::InputScalar("parity", ImGuiDataType_U32, &command->grxcmdserialconfigcmd.param.parity);
//                 ImGui::InputScalar("databits", ImGuiDataType_U32, &command->grxcmdserialconfigcmd.param.databits);
//                 ImGui::InputScalar("stopbits", ImGuiDataType_U32, &command->grxcmdserialconfigcmd.param.stopbits);
//                 ImGui::InputScalar("handshake", ImGuiDataType_U32, &command->grxcmdserialconfigcmd.param.handshake);
//                 ImGui::InputScalar("break", ImGuiDataType_U32, &command->grxcmdserialconfigcmd.param._break);
                

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_CmdSerialConfigCmd_t)-7};
//                     memcpy(command->grxcmdserialconfigcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxcmdserialconfigcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxcmdserialconfigcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxcmdserialconfigcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_CmdSerialConfigCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_CmdSerialConfigCmd_t);
                    
//                     uint16_t len = sizeof(GRX_CmdSerialConfigCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxcmdserialconfigcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxcmdserialconfigcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxcmdserialconfigcmd, sizeof(GRX_CmdSerialConfigCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u %u %u %u %u",command->grxcmdserialconfigcmd.CmdHeader[0],
//                                 command->grxcmdserialconfigcmd.CmdHeader[1], command->grxcmdserialconfigcmd.CmdHeader[2], command->grxcmdserialconfigcmd.CmdHeader[3], command->grxcmdserialconfigcmd.CmdHeader[4], command->grxcmdserialconfigcmd.CmdHeader[5], command->grxcmdserialconfigcmd.CmdHeader[6], command->grxcmdserialconfigcmd.CmdHeader[7],
//                                 command->grxcmdserialconfigcmd.param.port, command->grxcmdserialconfigcmd.param.baud, command->grxcmdserialconfigcmd.param.parity, command->grxcmdserialconfigcmd.param.databits, command->grxcmdserialconfigcmd.param.stopbits, command->grxcmdserialconfigcmd.param.handshake, command->grxcmdserialconfigcmd.param._break);
//                 break;
//             }
//             case 28: { //grxlogresisterhandlercmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("id", ImGuiDataType_U16, &command->grxlogresisterhandlercmd.param.id);
//                 ImGui::InputScalar("mlen", ImGuiDataType_U16, &command->grxlogresisterhandlercmd.param.mlen);
            
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_LogRegisterHandlerCmd_t)-7};
//                     memcpy(command->grxlogresisterhandlercmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxlogresisterhandlercmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxlogresisterhandlercmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxlogresisterhandlercmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_LogRegisterHandlerCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_LogRegisterHandlerCmd_t);
                    
//                     uint16_t len = sizeof(GRX_LogRegisterHandlerCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxlogresisterhandlercmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxlogresisterhandlercmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxlogresisterhandlercmd, sizeof(GRX_LogRegisterHandlerCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u",command->grxlogresisterhandlercmd.CmdHeader[0],
//                                 command->grxlogresisterhandlercmd.CmdHeader[1], command->grxlogresisterhandlercmd.CmdHeader[2], command->grxlogresisterhandlercmd.CmdHeader[3], command->grxlogresisterhandlercmd.CmdHeader[4], command->grxlogresisterhandlercmd.CmdHeader[5], command->grxlogresisterhandlercmd.CmdHeader[6], command->grxlogresisterhandlercmd.CmdHeader[7],
//                                 command->grxlogresisterhandlercmd.param.id, command->grxlogresisterhandlercmd.param.mlen);
//                 break;
//             }
//             case 29: { //grxlogunresisterhandlercmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("id", ImGuiDataType_U16, &command->grxlogunresisterhandlercmd.id);
                
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_LogUnregisterHandlerCmd_t)-7};
//                     memcpy(command->grxlogunresisterhandlercmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxlogunresisterhandlercmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxlogunresisterhandlercmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxlogunresisterhandlercmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_LogUnregisterHandlerCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_LogUnregisterHandlerCmd_t);
                    
//                     uint16_t len = sizeof(GRX_LogUnregisterHandlerCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxlogunresisterhandlercmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxlogunresisterhandlercmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxlogunresisterhandlercmd, sizeof(GRX_LogUnregisterHandlerCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u",command->grxlogunresisterhandlercmd.CmdHeader[0],
//                                 command->grxlogunresisterhandlercmd.CmdHeader[1], command->grxlogunresisterhandlercmd.CmdHeader[2], command->grxlogunresisterhandlercmd.CmdHeader[3], command->grxlogunresisterhandlercmd.CmdHeader[4], command->grxlogunresisterhandlercmd.CmdHeader[5], command->grxlogunresisterhandlercmd.CmdHeader[6], command->grxlogunresisterhandlercmd.CmdHeader[7],
//                                 command->grxlogunresisterhandlercmd.id);
//                 break;
//             }
//             case 30: { //grxlogaddcallbackcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("id", ImGuiDataType_U16, &command->grxlogaddcallbackcmd.param.id);
//                 ImGui::InputScalar("option", ImGuiDataType_S32, &command->grxlogaddcallbackcmd.param.options);
//                 ImGui::InputText("libpath", command->grxlogaddcallbackcmd.param.libpath, sizeof(command->grxlogaddcallbackcmd.param.libpath));
//                 ImGui::InputText("funcname", command->grxlogaddcallbackcmd.param.funcName, sizeof(command->grxlogaddcallbackcmd.param.funcName));
                
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_LogAddCallbackCmd_t)-7};
//                     memcpy(command->grxlogaddcallbackcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxlogaddcallbackcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxlogaddcallbackcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxlogaddcallbackcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_LogAddCallbackCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_LogAddCallbackCmd_t);
                    
//                     uint16_t len = sizeof(GRX_LogAddCallbackCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxlogaddcallbackcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxlogaddcallbackcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxlogaddcallbackcmd, sizeof(GRX_LogAddCallbackCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %d %s %s",command->grxlogaddcallbackcmd.CmdHeader[0],
//                                 command->grxlogaddcallbackcmd.CmdHeader[1], command->grxlogaddcallbackcmd.CmdHeader[2], command->grxlogaddcallbackcmd.CmdHeader[3], command->grxlogaddcallbackcmd.CmdHeader[4], command->grxlogaddcallbackcmd.CmdHeader[5], command->grxlogaddcallbackcmd.CmdHeader[6], command->grxlogaddcallbackcmd.CmdHeader[7],
//                                 command->grxlogaddcallbackcmd.param.id, command->grxlogaddcallbackcmd.param.options, command->grxlogaddcallbackcmd.param.libpath, command->grxlogaddcallbackcmd.param.funcName);
//                 break;
//             }
//             case 31: { //grxlogsethandlerstatuscmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode=0;
//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("id", ImGuiDataType_U16, &command->grxlogsethandlerstatuscmd.param.id);
//                 ImGui::InputScalar("status", ImGuiDataType_U8, &command->grxlogsethandlerstatuscmd.param.status);
//                 ImGui::Checkbox("override",&command->grxlogsethandlerstatuscmd.param.override);
                
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
//                     uint8_t length[2] = {0x00, sizeof(GRX_LogSetHandlerStatusCmd_t)-7};
//                     memcpy(command->grxlogsethandlerstatuscmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->grxlogsethandlerstatuscmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->grxlogsethandlerstatuscmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->grxlogsethandlerstatuscmd.CmdHeader + 7, &fnccode, sizeof(uint8_t));
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(GRX_LogSetHandlerStatusCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(GRX_LogSetHandlerStatusCmd_t);
                    
//                     uint16_t len = sizeof(GRX_LogSetHandlerStatusCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->grxlogsethandlerstatuscmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->grxlogsethandlerstatuscmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->grxlogsethandlerstatuscmd, sizeof(GRX_LogSetHandlerStatusCmd_t));
                    

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->grxlogsethandlerstatuscmd.CmdHeader[0],
//                                 command->grxlogsethandlerstatuscmd.CmdHeader[1], command->grxlogsethandlerstatuscmd.CmdHeader[2], command->grxlogsethandlerstatuscmd.CmdHeader[3], command->grxlogsethandlerstatuscmd.CmdHeader[4], command->grxlogsethandlerstatuscmd.CmdHeader[5], command->grxlogsethandlerstatuscmd.CmdHeader[6], command->grxlogsethandlerstatuscmd.CmdHeader[7],
//                                 command->grxlogsethandlerstatuscmd.param.id, command->grxlogsethandlerstatuscmd.param.status, command->grxlogsethandlerstatuscmd.param.override);
//                 break;
//             }

//             //ADCS
//             case 32: {
//                 //ID 2
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("unixTimeSeconds",ImGuiDataType_U32, &command->adcs_Unixtime.unixTimeSeconds);
//                 ImGui::InputScalar("unixTimeNanoSeconds",ImGuiDataType_U32, &command->adcs_Unixtime.unixTimeNanoSeconds);
                
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_CurrentUnixTimeCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_Unixtime.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_Unixtime.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_Unixtime.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_Unixtime.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_CurrentUnixTimeCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_CurrentUnixTimeCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_CurrentUnixTimeCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_Unixtime;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_Unixtime.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_Unixtime, sizeof(ADCS_CurrentUnixTimeCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;

//             }

//             case 33: {
//                 //ID 48
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("TargetLatitude",ImGuiDataType_Float, &command->adcs_RefLLHTarget.cmdTargetLatitude);
//                 ImGui::InputScalar("TargetLongitude",ImGuiDataType_Float, &command->adcs_RefLLHTarget.cmdTargetLongitude);
//                 ImGui::InputScalar("TargetAltitude",ImGuiDataType_Float, &command->adcs_RefLLHTarget.cmdTargetAltitude);
                
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_Reference_LLHTargetCommandCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_RefLLHTarget.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_RefLLHTarget.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_RefLLHTarget.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_RefLLHTarget.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_Reference_LLHTargetCommandCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_Reference_LLHTargetCommandCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_Reference_LLHTargetCommandCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_RefLLHTarget;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_RefLLHTarget.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_RefLLHTarget, sizeof(ADCS_Reference_LLHTargetCommandCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;

//             }

//             case 34: {
//                 //ID 49
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("gnssTimeSeconds(U32)",ImGuiDataType_U32, &command->adcs_GnssMeasurements.gnssTimeSeconds);
//                 ImGui::InputScalar("gnssTimeNs(U32)",ImGuiDataType_U32, &command->adcs_GnssMeasurements.gnssTimeNs);
//                 ImGui::InputScalar("gnssSatPosX(S32)",ImGuiDataType_S32, &command->adcs_GnssMeasurements.gnssSatPosX);
//                 ImGui::InputScalar("gnssSatPosY",ImGuiDataType_S32, &command->adcs_GnssMeasurements.gnssSatPosY);
//                 ImGui::InputScalar("gnssSatPosZ",ImGuiDataType_S32, &command->adcs_GnssMeasurements.gnssSatPosZ);
//                 ImGui::InputScalar("gnssSatVelX",ImGuiDataType_S32, &command->adcs_GnssMeasurements.gnssSatVelX);
//                 ImGui::InputScalar("gnssSatVelY",ImGuiDataType_S32, &command->adcs_GnssMeasurements.gnssSatVelY);
//                 ImGui::InputScalar("gnssSatVelZ",ImGuiDataType_S32, &command->adcs_GnssMeasurements.gnssSatVelZ);
//                 ImGui::InputScalar("syncTime",ImGuiDataType_U8, &command->adcs_GnssMeasurements.syncTime);
                
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_GnssMeasurementsCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_GnssMeasurements.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_GnssMeasurements.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_GnssMeasurements.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_GnssMeasurements.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_GnssMeasurementsCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_GnssMeasurementsCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_GnssMeasurementsCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_GnssMeasurements;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_GnssMeasurements.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_GnssMeasurements, sizeof(ADCS_GnssMeasurementsCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 35: {
//                 //ID 54
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("cmdRpyRoll(F32)",ImGuiDataType_U32, &command->adcs_RefRPY.cmdRpyRoll);
//                 ImGui::InputScalar("cmdRpyPitch(F32)",ImGuiDataType_U32, &command->adcs_RefRPY.cmdRpyPitch);
//                 ImGui::InputScalar("cmdRpyYaw(F32)",ImGuiDataType_S32, &command->adcs_RefRPY.cmdRpyYaw);
                
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_ReferenceRPYValueCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_RefRPY.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_RefRPY.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_RefRPY.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_RefRPY.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_ReferenceRPYValueCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_ReferenceRPYValueCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_ReferenceRPYValueCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_RefRPY;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_RefRPY.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_RefRPY, sizeof(ADCS_ReferenceRPYValueCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 36: {
//                 //ID 55
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("mtq0OnTimeCmd(S16)[ms]",ImGuiDataType_S16, &command->adcs_OpenLoopCmdMtq.mtq0OnTimeCmd);
//                 ImGui::InputScalar("mtq1OnTimeCmd(S16)[ms]",ImGuiDataType_S16, &command->adcs_OpenLoopCmdMtq.mtq1OnTimeCmd);
//                 ImGui::InputScalar("mtq2OnTimeCmd(S16)[ms]",ImGuiDataType_S16, &command->adcs_OpenLoopCmdMtq.mtq2OnTimeCmd);
                
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_OpenLoopCommandMtqCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_OpenLoopCmdMtq.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_OpenLoopCmdMtq.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_OpenLoopCmdMtq.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_OpenLoopCmdMtq.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_OpenLoopCommandMtqCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_OpenLoopCommandMtqCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_OpenLoopCommandMtqCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_OpenLoopCmdMtq;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_OpenLoopCmdMtq.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_OpenLoopCmdMtq, sizeof(ADCS_OpenLoopCommandMtqCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;

//             }

//             case 37: {
//                 //ID 58
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("controlMode(U8)",ImGuiDataType_U8, &command->adcs_ControlMode.controlMode);
//                 ImGui::InputScalar("magConTimeout(U16)[s]",ImGuiDataType_U16, &command->adcs_ControlMode.magConTimeout);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_ControlModeCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_ControlMode.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_ControlMode.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_ControlMode.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_ControlMode.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_ControlModeCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_ControlModeCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_ControlModeCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_ControlMode;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_ControlMode.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_ControlMode, sizeof(ADCS_ControlModeCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 38: {
//                 //ID 61
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("MOI ixx(F32)[kg.m^2]",ImGuiDataType_Float, &command->adcs_ConfigAdcsSat.ixx);
//                 ImGui::InputScalar("MOI iyy(F32)[kg.m^2]",ImGuiDataType_Float, &command->adcs_ConfigAdcsSat.iyy);
//                 ImGui::InputScalar("MOI izz(F32)[kg.m^2]",ImGuiDataType_Float, &command->adcs_ConfigAdcsSat.izz);
//                 ImGui::InputScalar("POI ixy(F32)[kg.m^2]",ImGuiDataType_Float, &command->adcs_ConfigAdcsSat.ixy);
//                 ImGui::InputScalar("POI ixz(F32)[kg.m^2]",ImGuiDataType_Float, &command->adcs_ConfigAdcsSat.ixz);
//                 ImGui::InputScalar("POI iyz(F32)[kg.m^2]",ImGuiDataType_Float, &command->adcs_ConfigAdcsSat.iyz);
//                 ImGui::InputScalar("sunPointBodyVecX(F64)",ImGuiDataType_Double, &command->adcs_ConfigAdcsSat.sunPointBodyVecX);
//                 ImGui::InputScalar("sunPointBodyVecY(F64)",ImGuiDataType_Double, &command->adcs_ConfigAdcsSat.sunPointBodyVecY);
//                 ImGui::InputScalar("sunPointBodyVecZ(F64)",ImGuiDataType_Double, &command->adcs_ConfigAdcsSat.sunPointBodyVecZ);
//                 ImGui::InputScalar("tgtTrackBodyVecX(F64)",ImGuiDataType_Double, &command->adcs_ConfigAdcsSat.tgtTrackBodyVecX);
//                 ImGui::InputScalar("tgtTrackBodyVecY(F64)",ImGuiDataType_Double, &command->adcs_ConfigAdcsSat.tgtTrackBodyVecY);
//                 ImGui::InputScalar("tgtTrackBodyVecZ(F64)",ImGuiDataType_Double, &command->adcs_ConfigAdcsSat.tgtTrackBodyVecZ);


//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_ConfigAdcsSatelliteCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_ConfigAdcsSat.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_ConfigAdcsSat.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_ConfigAdcsSat.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_ConfigAdcsSat.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_ConfigAdcsSatelliteCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_ConfigAdcsSatelliteCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_ConfigAdcsSatelliteCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_ConfigAdcsSat;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_ConfigAdcsSat.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_ConfigAdcsSat, sizeof(ADCS_ConfigAdcsSatelliteCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 39: {
//                 //ID 62
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("Default control mode(U8)",ImGuiDataType_U8, &command->adcs_ControllerConfig.conModeDefault);
//                 ImGui::InputScalar("Detumbling damping gain (Kd)(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kd);
//                 ImGui::InputScalar("Sun-spin control gain - sunlit part (KDsun)(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kdsun);
//                 ImGui::InputScalar("Sun-spin control gain - eclipse part (KDecl)(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kdecl);
//                 ImGui::InputScalar("Detumbling spin gain (Ks)(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.ks);
//                 ImGui::InputScalar("Fast B-dot detumbling gain (Kdf)(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kdf);
//                 ImGui::InputScalar("Y-momentum nutation damping gain (Kn)(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kn);
//                 ImGui::InputScalar("Y-momentum nutation damping quaternion gain (Kq)(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kq);
//                 ImGui::InputScalar("X-axis GG nutation damping quaternion gain (Kqx)(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kqx);
//                 ImGui::InputScalar("Y-axis GG nutation damping quaternion gain (Kqy)(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kqy);
//                 ImGui::InputScalar("Z-axis GG nutation damping quaternion gain (Kqz)(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kqz);
//                 ImGui::InputScalar("Wheel desaturation control gain(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kh);
//                 ImGui::InputScalar("Y-momentum proportional gain(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kp1);
//                 ImGui::InputScalar("Y-momentum derivative gain(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kd1);
//                 ImGui::InputScalar("RWheel proportional gain(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kp2);
//                 ImGui::InputScalar("RWheel derivative gain(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kd2);
//                 ImGui::InputScalar("Tracking proportional gain(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kp3);
//                 ImGui::InputScalar("Tracking derivative gain(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.kd3);
//                 ImGui::InputScalar("Tracking integral gain(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.ki3);
//                 ImGui::InputScalar("Reference spin rate(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.wy_ref);
//                 ImGui::InputScalar("Reference wheel momentum(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.h_ref);
//                 ImGui::InputScalar("Y-wheel bias momentum during XYZ-control(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.hy_bias);
//                 ImGui::InputScalar("Reference spin rate for RW spin control(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.wSunYawRef);
//                 ImGui::InputScalar("Sun keep-out angle(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.sunKeepoutAng);
//                 ImGui::InputScalar("Roll limit angle(F32)",ImGuiDataType_Float, &command->adcs_ControllerConfig.rollLimit);

//                 ImGui::InputScalar("Yaw compensation for earth rotation(Bool)",ImGuiDataType_U8, &command->adcs_ControllerConfig.yawCompensate);
//                 ImGui::InputScalar("Enable sun tracking in eclipse(Bool)",ImGuiDataType_U8, &command->adcs_ControllerConfig.sunTrackEclEn);
//                 ImGui::InputScalar("Enable sun avoidance(Bool)",ImGuiDataType_U8, &command->adcs_ControllerConfig.sunAvoidEn);

                


//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_ControllerConfigurationCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_ControllerConfig.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_ControllerConfig.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_ControllerConfig.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_ControllerConfig.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_ControllerConfigurationCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_ControllerConfigurationCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_ControllerConfigurationCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_ControllerConfig;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_ControllerConfig.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_ControllerConfig, sizeof(ADCS_ControllerConfigurationCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 40: {
//                 //ID 63
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("Magnetometer channel 1 offset(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.offset1);
//                 ImGui::InputScalar("Magnetometer channel 2 offset(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.offset2);
//                 ImGui::InputScalar("Magnetometer channel 3 offset(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.offset3);

//                 ImGui::InputScalar("Magnetometer sensitivity matrix S11(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.sensMatrix11);
//                 ImGui::InputScalar("Magnetometer sensitivity matrix S22(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.sensMatrix22);
//                 ImGui::InputScalar("Magnetometer sensitivity matrix S33(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.sensMatrix33);

//                 ImGui::InputScalar("Magnetometer sensitivity matrix S12(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.sensMatrix12);
//                 ImGui::InputScalar("Magnetometer sensitivity matrix S13(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.sensMatrix13);
//                 ImGui::InputScalar("Magnetometer sensitivity matrix S21(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.sensMatrix21);

//                 ImGui::InputScalar("Magnetometer sensitivity matrix S23(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.sensMatrix23);
//                 ImGui::InputScalar("Magnetometer sensitivity matrix S31(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.sensMatrix31);
//                 ImGui::InputScalar("Magnetometer sensitivity matrix S32(F32)",ImGuiDataType_Float, &command->adcs_ConfigMag0OrbitCal.sensMatrix32);


//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_ConfigMag0OrbitCalCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_ConfigMag0OrbitCal.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_ConfigMag0OrbitCal.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_ConfigMag0OrbitCal.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_ConfigMag0OrbitCal.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_ConfigMag0OrbitCalCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_ConfigMag0OrbitCalCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_ConfigMag0OrbitCalCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_ConfigMag0OrbitCal;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_ConfigMag0OrbitCal.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_ConfigMag0OrbitCal, sizeof(ADCS_ConfigMag0OrbitCalCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 41: {
//                 //ID 65
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("StackX mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountStackX);
//                 ImGui::InputScalar("StackY mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountStackY);
//                 ImGui::InputScalar("StackZ mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountStackZ);
//                 ImGui::InputScalar("MTQ0 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountMtq0);
//                 ImGui::InputScalar("MTQ1 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountMtq1);
//                 ImGui::InputScalar("MTQ2 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountMtq2);
//                 ImGui::InputScalar("Wheel0 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountRwl0);
//                 ImGui::InputScalar("Wheel1 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountRwl1);
//                 ImGui::InputScalar("Wheel2 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountRwl2);

//                 ImGui::InputScalar("CSS0 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountCss0);
//                 ImGui::InputScalar("CSS1 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountCss1);
//                 ImGui::InputScalar("CSS2 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountCss2);
//                 ImGui::InputScalar("CSS3 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountCss3);
//                 ImGui::InputScalar("CSS4 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountCss4);
//                 ImGui::InputScalar("CSS5 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountCss5);
//                 ImGui::InputScalar("CSS6 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountCss6);
//                 ImGui::InputScalar("CSS7 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountCss7);
//                 ImGui::InputScalar("CSS8 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountCss8);
//                 ImGui::InputScalar("CSS9 mounting(U8)",ImGuiDataType_U8, &command->adcs_MountingConfig.mountCss9);

//                 ImGui::InputScalar("FSS0 mounting alpha angle(F64)",ImGuiDataType_Double, &command->adcs_MountingConfig.mountFss0Alpha);
//                 ImGui::InputScalar("FSS0 mounting beta angle(F64)",ImGuiDataType_Double, &command->adcs_MountingConfig.mountFss0Beta);
//                 ImGui::InputScalar("FSS0 mounting gamma angle(F64)",ImGuiDataType_Double, &command->adcs_MountingConfig.mountFss0Gamma);
//                 ImGui::InputScalar("HSS0 mounting alpha angle(F64)",ImGuiDataType_Double, &command->adcs_MountingConfig.mountHss0Alpha);
//                 ImGui::InputScalar("HSS0 mounting beta angle(F64)",ImGuiDataType_Double, &command->adcs_MountingConfig.mountHss0Beta);
//                 ImGui::InputScalar("HSS0 mounting gamma angle(F64)",ImGuiDataType_Double, &command->adcs_MountingConfig.mountHss0Gamma);
//                 ImGui::InputScalar("MAG0 mounting alpha angle(F64)",ImGuiDataType_Double, &command->adcs_MountingConfig.mountMag0Alpha);
//                 ImGui::InputScalar("MAG0 mounting beta angle(F64)",ImGuiDataType_Double, &command->adcs_MountingConfig.mountMag0Beta);
//                 ImGui::InputScalar("MAG0 mounting gamma angle(F64)",ImGuiDataType_Double, &command->adcs_MountingConfig.mountMag0Gamma);


//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_MountingConfigurationCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_MountingConfig.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_MountingConfig.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_MountingConfig.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_MountingConfig.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_MountingConfigurationCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_MountingConfigurationCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_MountingConfigurationCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_MountingConfig;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_MountingConfig.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_MountingConfig, sizeof(ADCS_MountingConfigurationCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 42: {
//                 //ID 67
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("Default main estimator mode(U8)",ImGuiDataType_U8, &command->adcs_EstimatorConfig.estModeMainDefault);
//                 ImGui::InputScalar("Default backup estimator mode(U8)",ImGuiDataType_U8, &command->adcs_EstimatorConfig.estModeBackupDefault);

//                 ImGui::InputScalar("MAG measurement noise(Float)",ImGuiDataType_Float, &command->adcs_EstimatorConfig.magR);
//                 ImGui::InputScalar("CSS measurement noise(Float)",ImGuiDataType_Float, &command->adcs_EstimatorConfig.cssR);
//                 ImGui::InputScalar("FSS measurement noise(Float)",ImGuiDataType_Float, &command->adcs_EstimatorConfig.fssR);
//                 ImGui::InputScalar("HSS measurement noise(Float)",ImGuiDataType_Float, &command->adcs_EstimatorConfig.hssR);
//                 ImGui::InputScalar("STR measurement noise(Float)",ImGuiDataType_Float, &command->adcs_EstimatorConfig.strR);
//                 ImGui::InputScalar("Magnetometer RKF system noise(Float)",ImGuiDataType_Float, &command->adcs_EstimatorConfig.rkfQ);
//                 ImGui::InputScalar("EKF system noise(Float)",ImGuiDataType_Float, &command->adcs_EstimatorConfig.ekfQ);
//                 ImGui::InputScalar("Nutation Epsilon correction(Float)",ImGuiDataType_Float, &command->adcs_EstimatorConfig.nutDeps);
//                 ImGui::InputScalar("Nutation Psi correction(Float)",ImGuiDataType_Float, &command->adcs_EstimatorConfig.nutDpsi);

//                 ImGui::InputScalar("Use FSS in EKF(Bool)",ImGuiDataType_U8, &command->adcs_EstimatorConfig.ekfUseFss);
//                 ImGui::InputScalar("Use CSS in EKF(Bool)",ImGuiDataType_U8, &command->adcs_EstimatorConfig.ekfUseCss);
//                 ImGui::InputScalar("Use HSS in EKF(Bool)",ImGuiDataType_U8, &command->adcs_EstimatorConfig.ekfUseHss);
//                 ImGui::InputScalar("Use STR in EKF(Bool)",ImGuiDataType_U8, &command->adcs_EstimatorConfig.ekfUseStr);
                
//                 ImGui::InputScalar("Triad Vector 1(U8)",ImGuiDataType_U8, &command->adcs_EstimatorConfig.tVec1);
//                 ImGui::InputScalar("Triad Vector 2(U8)",ImGuiDataType_U8, &command->adcs_EstimatorConfig.tVec2);
   

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_EstimatorConfigurationCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_EstimatorConfig.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_EstimatorConfig.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_EstimatorConfig.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_EstimatorConfig.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_EstimatorConfigurationCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_EstimatorConfigurationCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_EstimatorConfigurationCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_EstimatorConfig;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_EstimatorConfig.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_EstimatorConfig, sizeof(ADCS_EstimatorConfigurationCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 43: {
//                 //ID 68
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("Orbit epoch(F64)",ImGuiDataType_Double, &command->adcs_ConfigOrbitSatParams.orbitEpoch);
//                 ImGui::InputScalar("Orbit inclination(F64)",ImGuiDataType_Double, &command->adcs_ConfigOrbitSatParams.orbitIncl);
//                 ImGui::InputScalar("Orbit RAAN(F64)",ImGuiDataType_Double, &command->adcs_ConfigOrbitSatParams.orbitRaan);
//                 ImGui::InputScalar("Orbit eccentricity(F64)",ImGuiDataType_Double, &command->adcs_ConfigOrbitSatParams.orbitEccen);
//                 ImGui::InputScalar("Orbit argument of perigee(F64)",ImGuiDataType_Double, &command->adcs_ConfigOrbitSatParams.orbitAP);
//                 ImGui::InputScalar("Orbit mean anomaly(F64)",ImGuiDataType_Double, &command->adcs_ConfigOrbitSatParams.orbitMA);
//                 ImGui::InputScalar("Orbit mean motion(F64)",ImGuiDataType_Double, &command->adcs_ConfigOrbitSatParams.orbitMM);
//                 ImGui::InputScalar("Orbit B-star drag term(F64)",ImGuiDataType_Double, &command->adcs_ConfigOrbitSatParams.orbitBstar);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_ConfigOrbitSatParamsCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_ConfigOrbitSatParams.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_ConfigOrbitSatParams.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_ConfigOrbitSatParams.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_ConfigOrbitSatParams.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_ConfigOrbitSatParamsCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_ConfigOrbitSatParamsCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_ConfigOrbitSatParamsCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_ConfigOrbitSatParams;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_ConfigOrbitSatParams.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_ConfigOrbitSatParams, sizeof(ADCS_ConfigOrbitSatParamsCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 44: {
//                 //ID 69
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("RWL selection flags(U8)",ImGuiDataType_U8, &command->adcs_NodeSelection.selectRwl);
//                 ImGui::InputScalar("MAG selection flags(U8)",ImGuiDataType_U8, &command->adcs_NodeSelection.selectMag);
//                 ImGui::InputScalar("FSS selection flags(U8)",ImGuiDataType_U8, &command->adcs_NodeSelection.selectFss);
//                 ImGui::InputScalar("HSS selection flags(U8)",ImGuiDataType_U8, &command->adcs_NodeSelection.selectHss);
//                 ImGui::InputScalar("GYRO selection flags(U8)",ImGuiDataType_U8, &command->adcs_NodeSelection.selectGyro);
//                 ImGui::InputScalar("STR selection flags(U8)",ImGuiDataType_U8, &command->adcs_NodeSelection.selectStr);
//                 ImGui::InputScalar("GNSS selection flags(U8)",ImGuiDataType_U8, &command->adcs_NodeSelection.selectGnss);
//                 ImGui::InputScalar("External sensor selection flags(U8)",ImGuiDataType_U8, &command->adcs_NodeSelection.selectExt);


//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_NodeSelectionCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_NodeSelection.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_NodeSelection.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_NodeSelection.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_NodeSelection.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_NodeSelectionCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_NodeSelectionCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_NodeSelectionCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_NodeSelection;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_NodeSelection.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_NodeSelection, sizeof(ADCS_NodeSelectionCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 45: {
//                 //ID 70
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("MTQ0 maximum dipole moment(F32)",ImGuiDataType_Float, &command->adcs_MtqConfig.mtq0Mmax);
//                 ImGui::InputScalar("MTQ1 maximum dipole moment(F32)",ImGuiDataType_Float, &command->adcs_MtqConfig.mtq1Mmax);
//                 ImGui::InputScalar("MTQ2 maximum dipole moment(F32)",ImGuiDataType_Float, &command->adcs_MtqConfig.mtq2Mmax);
//                 ImGui::InputScalar("Maximum magnetorquer on-time(U16)",ImGuiDataType_U16, &command->adcs_MtqConfig.onTimeMax);
//                 ImGui::InputScalar("Minimum magnetorquer on-time(U16)",ImGuiDataType_U16, &command->adcs_MtqConfig.onTimeMin);
//                 ImGui::InputScalar("LPF factor for magnetorquer commands(0: no filtering)(F32)",ImGuiDataType_Float, &command->adcs_MtqConfig.mtqFfac);


//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_MtqConfigCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_MtqConfig.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_MtqConfig.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_MtqConfig.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_MtqConfig.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_MtqConfigCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_MtqConfigCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_MtqConfigCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_MtqConfig;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_MtqConfig.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_MtqConfig, sizeof(ADCS_MtqConfigCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 46: {
//                 //ID 71
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("Main estimator mode(U8)",ImGuiDataType_U8, &command->adcs_EstMode.estModeMain);
//                 ImGui::InputScalar("Backup estimator mode(U8)",ImGuiDataType_U8, &command->adcs_EstMode.estModeBackup);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_EstimationModeCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_EstMode.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_EstMode.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_EstMode.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_EstMode.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_EstimationModeCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_EstimationModeCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_EstimationModeCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_EstMode;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_EstMode.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_EstMode, sizeof(ADCS_EstimationModeCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 47: {
//                 //ID 74
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("RWL0 open-loop speed(F32)",ImGuiDataType_Float, &command->adcs_OpenLoopCmdRwl.rwl0SpeedCmd);
//                 ImGui::InputScalar("RWL1 open-loop speed(F32)",ImGuiDataType_Float, &command->adcs_OpenLoopCmdRwl.rwl1SpeedCmd);
//                 ImGui::InputScalar("RWL2 open-loop speed(F32)",ImGuiDataType_Float, &command->adcs_OpenLoopCmdRwl.rwl2SpeedCmd);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_OpenLoopCommandRwlCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_OpenLoopCmdRwl.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_OpenLoopCmdRwl.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_OpenLoopCmdRwl.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_OpenLoopCmdRwl.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_OpenLoopCommandRwlCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_OpenLoopCommandRwlCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_OpenLoopCommandRwlCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_OpenLoopCmdRwl;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_OpenLoopCmdRwl.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_OpenLoopCmdRwl, sizeof(ADCS_OpenLoopCommandRwlCmd_t));
//                     // ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %d",command->u32u8bool.CmdHeader[0],
//                     //             command->u32u8bool.CmdHeader[1], command->u32u8bool.CmdHeader[2], command->u32u8bool.CmdHeader[3], command->u32u8bool.CmdHeader[4], command->u32u8bool.CmdHeader[5], command->u32u8bool.CmdHeader[6], command->u32u8bool.CmdHeader[7],
//                     //             command->u32u8bool.Timeout, command->u32u8bool.Channel, command->u32u8bool.Status);

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             case 48: {
//                 //ID 76
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("X-momentum open-loop speed(F32)",ImGuiDataType_Float, &command->adcs_OpenLoopCmdHxyzRw.hx);
//                 ImGui::InputScalar("Y-momentum open-loop speed(F32)",ImGuiDataType_Float, &command->adcs_OpenLoopCmdHxyzRw.hy);
//                 ImGui::InputScalar("Z-momentum open-loop speed(F32)",ImGuiDataType_Float, &command->adcs_OpenLoopCmdHxyzRw.hz);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(ADCS_OpenLoopCommandHxyzRwCmd_t)-7};
                    
                    
//                     memcpy(command->adcs_OpenLoopCmdHxyzRw.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->adcs_OpenLoopCmdHxyzRw.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->adcs_OpenLoopCmdHxyzRw.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->adcs_OpenLoopCmdHxyzRw.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(ADCS_OpenLoopCommandHxyzRwCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(ADCS_OpenLoopCommandHxyzRwCmd_t);
                    
//                     uint16_t len = sizeof(ADCS_OpenLoopCommandHxyzRwCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->adcs_OpenLoopCmdHxyzRw;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->adcs_OpenLoopCmdHxyzRw.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->adcs_OpenLoopCmdHxyzRw, sizeof(ADCS_OpenLoopCommandHxyzRwCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;

//             }
//             /* < End of ADCS Part > */

//             // ifc app
//             case 49: { //ifc handle noarg
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputText("handle name char[16]", command->ifchandlenoarg.HandleName, sizeof(command->ifchandlenoarg.HandleName));

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_HandleNoArgsCmd_t)-7};
                    
                    
//                     memcpy(command->ifchandlenoarg.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifchandlenoarg.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifchandlenoarg.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifchandlenoarg.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_HandleNoArgsCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_HandleNoArgsCmd_t);
                    
//                     uint16_t len = sizeof(IFC_HandleNoArgsCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifchandlenoarg;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifchandlenoarg.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifchandlenoarg, sizeof(IFC_HandleNoArgsCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 50: { // ifchandleu8arg
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputText("handle name char[16]", command->ifchandleu8arg.HandleName, sizeof(command->ifchandleu8arg.HandleName));
//                 ImGui::InputScalar("arg u8",ImGuiDataType_U8, &command->ifchandleu8arg.Arg);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_HandleU8ArgsCmd_t)-7};
                    
                    
//                     memcpy(command->ifchandleu8arg.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifchandleu8arg.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifchandleu8arg.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifchandleu8arg.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_HandleU8ArgsCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_HandleU8ArgsCmd_t);
                    
//                     uint16_t len = sizeof(IFC_HandleU8ArgsCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifchandleu8arg;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifchandleu8arg.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifchandleu8arg, sizeof(IFC_HandleU8ArgsCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 51: { // ifchandleu32arg
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputText("handle name char[16]", command->ifchandleu32arg.HandleName, sizeof(command->ifchandleu32arg.HandleName));
//                 ImGui::InputScalar("arg u32",ImGuiDataType_U32, &command->ifchandleu32arg.Arg);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_HandleU32ArgsCmd_t)-7};
                    
                    
//                     memcpy(command->ifchandleu32arg.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifchandleu32arg.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifchandleu32arg.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifchandleu32arg.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_HandleU32ArgsCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_HandleU32ArgsCmd_t);
                    
//                     uint16_t len = sizeof(IFC_HandleU32ArgsCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifchandleu32arg;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifchandleu32arg.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifchandleu32arg, sizeof(IFC_HandleU32ArgsCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 52: { // ifcwritecmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputText("handle name char[16]", command->ifcwritecmd.HandleName, sizeof(command->ifcwritecmd.HandleName));
//                 ImGui::InputScalar("size u16",ImGuiDataType_U16, &command->ifcwritecmd.Size);
//                 for(uint8_t i=0; i<sizeof(command->ifcwritecmd.Data); i++) {
//                     char temp[10] = {0,};
//                     sprintf(temp,"data[%u]",i);
//                     ImGui::InputScalar(temp,ImGuiDataType_U8, &command->ifcwritecmd.Data[i]);
//                 }

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_WriteCmd_t)-7};
                    
                    
//                     memcpy(command->ifcwritecmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifcwritecmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifcwritecmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifcwritecmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_WriteCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_WriteCmd_t);
                    
//                     uint16_t len = sizeof(IFC_WriteCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifcwritecmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifcwritecmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifcwritecmd, sizeof(IFC_WriteCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 53: { // ifcreadcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputText("handle name char[16]", command->ifcreadcmd.HandleName, sizeof(command->ifcreadcmd.HandleName));
//                 ImGui::InputScalar("size u16",ImGuiDataType_U16, &command->ifcreadcmd.Size);
//                 ImGui::InputScalar("timeout u16",ImGuiDataType_U16, &command->ifcreadcmd.Timeout);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_ReadCmd_t)-7};
                    
                    
//                     memcpy(command->ifcreadcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifcreadcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifcreadcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifcreadcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_ReadCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_ReadCmd_t);
                    
//                     uint16_t len = sizeof(IFC_ReadCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifcreadcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifcreadcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifcreadcmd, sizeof(IFC_ReadCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 54: { // ifcgpionumcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("gpio num int32",ImGuiDataType_S32, &command->ifcgpionumcmd.GpioNum);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_GpioNumCmd_t)-7};
                    
                    
//                     memcpy(command->ifcgpionumcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifcgpionumcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifcgpionumcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifcgpionumcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_GpioNumCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_GpioNumCmd_t);
                    
//                     uint16_t len = sizeof(IFC_GpioNumCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifcgpionumcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifcgpionumcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifcgpionumcmd, sizeof(IFC_GpioNumCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 55: { // ifcioopencmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputText("handle name char[16]",command->ifcioopencmd.HandleName, sizeof(command->ifcioopencmd.HandleName));
//                 ImGui::InputText("handle name char[16]",command->ifcioopencmd.DeviceName, sizeof(command->ifcioopencmd.DeviceName));
//                 ImGui::InputScalar("open opt int32",ImGuiDataType_S32, &command->ifcioopencmd.OpenOpt);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_IoOpenCmd_t)-7};
                    
                    
//                     memcpy(command->ifcioopencmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifcioopencmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifcioopencmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifcioopencmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_IoOpenCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_IoOpenCmd_t);
                    
//                     uint16_t len = sizeof(IFC_IoOpenCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifcioopencmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifcioopencmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifcioopencmd, sizeof(IFC_IoOpenCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 56: { // ifcgpiowritecmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

                
//                 ImGui::InputScalar("gpio num int32",ImGuiDataType_S32, &command->ifcgpiowritecmd.GpioNum);
//                 ImGui::Checkbox("Value bool",&command->ifcgpiowritecmd.Value);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_GpioWriteCmd_t)-7};
                    
                    
//                     memcpy(command->ifcgpiowritecmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifcgpiowritecmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifcgpiowritecmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifcgpiowritecmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_GpioWriteCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_GpioWriteCmd_t);
                    
//                     uint16_t len = sizeof(IFC_GpioWriteCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifcgpiowritecmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifcgpiowritecmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifcgpiowritecmd, sizeof(IFC_GpioWriteCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 57: { // ifcuartsettermioscmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

                
//                 ImGui::InputText("handle name char[16]", command->ifcuartsettermioscmd.HandleName, sizeof(command->ifcuartsettermioscmd.HandleName));
//                 for(uint8_t i=0; i<sizeof(command->ifcuartsettermioscmd.Termios); i++) {
//                     char temp[10] = {0,};
//                     sprintf(temp,"termios[%u]",i);
//                     ImGui::InputScalar(temp,ImGuiDataType_U8, &command->ifcuartsettermioscmd.Termios[i]);
//                 }

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_UartSetTermiosCmd_t)-7};
                    
                    
//                     memcpy(command->ifcuartsettermioscmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifcuartsettermioscmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifcuartsettermioscmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifcuartsettermioscmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_UartSetTermiosCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_UartSetTermiosCmd_t);
                    
//                     uint16_t len = sizeof(IFC_UartSetTermiosCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifcuartsettermioscmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifcuartsettermioscmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifcuartsettermioscmd, sizeof(IFC_UartSetTermiosCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 58: { // ifci2cduplextransfercmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

                
//                 ImGui::InputText("handle name char[16]", command->ifci2cduplextransfercmd.HandleName, sizeof(command->ifci2cduplextransfercmd.HandleName));
//                 ImGui::InputScalar("Tx Size u16",ImGuiDataType_U16, &command->ifci2cduplextransfercmd.TxSize);
//                 ImGui::InputScalar("Rx Size u16",ImGuiDataType_U16, &command->ifci2cduplextransfercmd.RxSize);
//                 ImGui::InputScalar("address u8",ImGuiDataType_U8, &command->ifci2cduplextransfercmd.Address);
//                 for(uint8_t i=0; i<sizeof(command->ifci2cduplextransfercmd.TxData); i++) {
//                     char temp[10] = {0,};
//                     sprintf(temp,"tx data[%u]",i);
//                     ImGui::InputScalar(temp,ImGuiDataType_U8, &command->ifci2cduplextransfercmd.TxData[i]);
//                 }

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_I2cDuplexTransferCmd_t)-7};
                    
                    
//                     memcpy(command->ifci2cduplextransfercmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifci2cduplextransfercmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifci2cduplextransfercmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifci2cduplextransfercmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_I2cDuplexTransferCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_I2cDuplexTransferCmd_t);
                    
//                     uint16_t len = sizeof(IFC_I2cDuplexTransferCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifci2cduplextransfercmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifci2cduplextransfercmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifci2cduplextransfercmd, sizeof(IFC_I2cDuplexTransferCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 59: { // ifcspiduplextransfercmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

                
//                 ImGui::InputText("handle name char[16]", command->ifcspiduplextransfercmd.HandleName, sizeof(command->ifcspiduplextransfercmd.HandleName));
//                 ImGui::InputScalar("Tx Size u16",ImGuiDataType_U16, &command->ifcspiduplextransfercmd.TxSize);
//                 ImGui::InputScalar("Rx Size u16",ImGuiDataType_U16, &command->ifcspiduplextransfercmd.RxSize);
//                 for(uint8_t i=0; i<sizeof(command->ifcspiduplextransfercmd.TxData); i++) {
//                     char temp[10] = {0,};
//                     sprintf(temp,"tx data[%u]",i);
//                     ImGui::InputScalar(temp,ImGuiDataType_U8, &command->ifcspiduplextransfercmd.TxData[i]);
//                 }

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_SpiDuplexTransferCmd_t)-7};
                    
                    
//                     memcpy(command->ifcspiduplextransfercmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifcspiduplextransfercmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifcspiduplextransfercmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifcspiduplextransfercmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_SpiDuplexTransferCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_SpiDuplexTransferCmd_t);
                    
//                     uint16_t len = sizeof(IFC_SpiDuplexTransferCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifcspiduplextransfercmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifcspiduplextransfercmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifcspiduplextransfercmd, sizeof(IFC_SpiDuplexTransferCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 60: { // ifciohandleallocatecmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

                
//                 ImGui::InputText("handle name char[16]", command->ifciohandleallocatecmd.HandleName, sizeof(command->ifciohandleallocatecmd.HandleName));
//                 ImGui::InputText("dev name char[32]", command->ifciohandleallocatecmd.DevName, sizeof(command->ifciohandleallocatecmd.DevName));
//                 ImGui::InputScalar("dev type u8",ImGuiDataType_U8, &command->ifciohandleallocatecmd.devType);
//                 ImGui::InputScalar("mutex id u8",ImGuiDataType_U8, &command->ifciohandleallocatecmd.MutexId);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(IFC_IoHandleAllocateCmd_t)-7};
                    
                    
//                     memcpy(command->ifciohandleallocatecmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->ifciohandleallocatecmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->ifciohandleallocatecmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->ifciohandleallocatecmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(IFC_IoHandleAllocateCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(IFC_IoHandleAllocateCmd_t);
                    
//                     uint16_t len = sizeof(IFC_IoHandleAllocateCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->ifciohandleallocatecmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->ifciohandleallocatecmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->ifciohandleallocatecmd, sizeof(IFC_IoHandleAllocateCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %s %s %u %u",command->ifciohandleallocatecmd.CmdHeader[0],
//                             command->ifciohandleallocatecmd.CmdHeader[1], command->ifciohandleallocatecmd.CmdHeader[2], command->ifciohandleallocatecmd.CmdHeader[3], command->ifciohandleallocatecmd.CmdHeader[4], command->ifciohandleallocatecmd.CmdHeader[5], command->ifciohandleallocatecmd.CmdHeader[6], command->ifciohandleallocatecmd.CmdHeader[7],
//                             command->ifciohandleallocatecmd.HandleName, command->ifciohandleallocatecmd.DevName, command->ifciohandleallocatecmd.devType, command->ifciohandleallocatecmd.MutexId);
//                 break;
//             }

//             // EPS P60 Param Set CMD
//             case 61: { // epsp60paramsetcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

                
//                 ImGui::InputScalar("timeout u32",ImGuiDataType_U32, &command->epsp60paramsetcmd.Timeout);
//                 ImGui::InputScalar("size u32",ImGuiDataType_U32, &command->epsp60paramsetcmd.Size);
//                 ImGui::InputScalar("node u8",ImGuiDataType_U8, &command->epsp60paramsetcmd.Node);
//                 ImGui::InputScalar("tableid u8",ImGuiDataType_U8, &command->epsp60paramsetcmd.TableId);
//                 ImGui::InputScalar("address u16",ImGuiDataType_U16, &command->epsp60paramsetcmd.Address);
//                 ImGui::InputScalar("type u8",ImGuiDataType_U8, &command->epsp60paramsetcmd.Type);
//                 for(uint8_t i=0; i<sizeof(command->epsp60paramsetcmd.Value); i++) {
//                     char temp[10] = {0,};
//                     sprintf(temp,"value[%u]",i);
//                     ImGui::InputScalar(temp,ImGuiDataType_U8, &command->epsp60paramsetcmd.Value[i]);
//                 }
//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(EPS_P60_ParamSetCmd_t)-7};
                    
                    
//                     memcpy(command->epsp60paramsetcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->epsp60paramsetcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->epsp60paramsetcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->epsp60paramsetcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(EPS_P60_ParamSetCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(EPS_P60_ParamSetCmd_t);
                    
//                     uint16_t len = sizeof(EPS_P60_ParamSetCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->epsp60paramsetcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->epsp60paramsetcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->epsp60paramsetcmd, sizeof(EPS_P60_ParamSetCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u %u %u %u",command->epsp60paramsetcmd.CmdHeader[0],
//                             command->epsp60paramsetcmd.CmdHeader[1], command->epsp60paramsetcmd.CmdHeader[2], command->epsp60paramsetcmd.CmdHeader[3], command->epsp60paramsetcmd.CmdHeader[4], command->epsp60paramsetcmd.CmdHeader[5], command->epsp60paramsetcmd.CmdHeader[6], command->epsp60paramsetcmd.CmdHeader[7],
//                             command->epsp60paramsetcmd.Timeout, command->epsp60paramsetcmd.Size, command->epsp60paramsetcmd.Node, command->epsp60paramsetcmd.TableId, command->epsp60paramsetcmd.Address, command->epsp60paramsetcmd.Type);
//                 for(uint8_t i=0; i<sizeof(command->epsp60paramsetcmd.Value); i++) {
//                     ImGui::Text("Value[%d]: %u\t", i, command->epsp60paramsetcmd.Value[i]);
//                     if(!(i%6) && i!=0) continue;
//                     ImGui::SameLine();
//                 }
//                 break;
//             }

//             //PAYS
//             case 62: { // paysd1064readsavestatuscmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("num reads u16",ImGuiDataType_U16, &command->paysd1064readsavestatuscmd.NumReads);
//                 ImGui::InputScalar("interval ms u16",ImGuiDataType_U16, &command->paysd1064readsavestatuscmd.IntervalMs);
//                 ImGui::Checkbox("ignore error bool",&command->paysd1064readsavestatuscmd.IgnoreErrors);
//                 ImGui::Checkbox("pack bool",&command->paysd1064readsavestatuscmd.Pack);
//                 ImGui::InputText("file name char[128]", command->paysd1064readsavestatuscmd.FileName, sizeof(command->paysd1064readsavestatuscmd.FileName));

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(PAYS_D1064_ReadSaveStatusCmd_t)-7};
                    
                    
//                     memcpy(command->paysd1064readsavestatuscmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->paysd1064readsavestatuscmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->paysd1064readsavestatuscmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->paysd1064readsavestatuscmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(PAYS_D1064_ReadSaveStatusCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(PAYS_D1064_ReadSaveStatusCmd_t);
                    
//                     uint16_t len = sizeof(PAYS_D1064_ReadSaveStatusCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->paysd1064readsavestatuscmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->paysd1064readsavestatuscmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->paysd1064readsavestatuscmd, sizeof(PAYS_D1064_ReadSaveStatusCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             //PAYC
//             case 63: { // paycsetconfcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("delay u16",ImGuiDataType_U16, &command->paycsetconfcmd.Payload.delay);
//                 ImGui::InputScalar("snap flag u32",ImGuiDataType_U32, &command->paycsetconfcmd.Payload.snap_flags);
//                 ImGui::InputScalar("snap format u16",ImGuiDataType_U8, &command->paycsetconfcmd.Payload.snap_format);
//                 ImGui::InputScalar("store flag u16",ImGuiDataType_U32, &command->paycsetconfcmd.Payload.store_flags);
//                 ImGui::InputScalar("store format u16",ImGuiDataType_U8, &command->paycsetconfcmd.Payload.store_format);
                

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(PAYC_SetConfCmd_t)-7};
                    
                    
//                     memcpy(command->paycsetconfcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->paycsetconfcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->paycsetconfcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->paycsetconfcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(PAYC_SetConfCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(PAYC_SetConfCmd_t);
                    
//                     uint16_t len = sizeof(PAYC_SetConfCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->paycsetconfcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->paycsetconfcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->paycsetconfcmd, sizeof(PAYC_SetConfCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 64: { // paycsnapcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("count u8",ImGuiDataType_U8, &command->paycsnapcmd.Payload.count);
//                 ImGui::InputScalar("width u16",ImGuiDataType_U16, &command->paycsnapcmd.Payload.width);
//                 ImGui::InputScalar("height u16",ImGuiDataType_U16, &command->paycsnapcmd.Payload.height);
//                 ImGui::InputScalar("left edge u16",ImGuiDataType_U16, &command->paycsnapcmd.Payload.leftedge);
//                 ImGui::InputScalar("top edge u16",ImGuiDataType_U16, &command->paycsnapcmd.Payload.topedge);
                

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(PAYC_SnapCmd_t)-7};
                    
                    
//                     memcpy(command->paycsnapcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->paycsnapcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->paycsnapcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->paycsnapcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(PAYC_SnapCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(PAYC_SnapCmd_t);
                    
//                     uint16_t len = sizeof(PAYC_SnapCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->paycsnapcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->paycsnapcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->paycsnapcmd, sizeof(PAYC_SnapCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 65: { // paycstorecmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("format u8",ImGuiDataType_U8, &command->paycstorecmd.Payload.format);
//                 ImGui::InputScalar("scale u8",ImGuiDataType_U16, &command->paycstorecmd.Payload.scale);
//                 ImGui::InputText("filename char[32]", command->paycstorecmd.Payload.filename, sizeof(command->paycstorecmd.Payload.filename));

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(PAYC_StoreCmd_t)-7};
                    
                    
//                     memcpy(command->paycstorecmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->paycstorecmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->paycstorecmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->paycstorecmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(PAYC_StoreCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(PAYC_StoreCmd_t);
                    
//                     uint16_t len = sizeof(PAYC_StoreCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->paycstorecmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->paycstorecmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->paycstorecmd, sizeof(PAYC_StoreCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 66: { // paycstorefilelistlocationcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputText("stored file list location char[64]", command->paycstorefilelistlocationcmd.Payload.StoredFileListLocation, sizeof(command->paycstorefilelistlocationcmd.Payload.StoredFileListLocation));

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(PAYC_StoreFileListLocationCmd_t)-7};
                    
                    
//                     memcpy(command->paycstorefilelistlocationcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->paycstorefilelistlocationcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->paycstorefilelistlocationcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->paycstorefilelistlocationcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(PAYC_StoreFileListLocationCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(PAYC_StoreFileListLocationCmd_t);
                    
//                     uint16_t len = sizeof(PAYC_StoreFileListLocationCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->paycstorefilelistlocationcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->paycstorefilelistlocationcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->paycstorefilelistlocationcmd, sizeof(PAYC_StoreFileListLocationCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 67: { // paycsetdirpathcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputText("remote dirname char[64]", command->paycsetdirpathcmd.Payload.remote_dirname, sizeof(command->paycsetdirpathcmd.Payload.remote_dirname));
//                 ImGui::InputText("local dirname char[64]", command->paycsetdirpathcmd.Payload.local_dirname, sizeof(command->paycsetdirpathcmd.Payload.local_dirname));

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(PAYC_SetDirPathCmd_t)-7};
                    
                    
//                     memcpy(command->paycsetdirpathcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->paycsetdirpathcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->paycsetdirpathcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->paycsetdirpathcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(PAYC_SetDirPathCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(PAYC_SetDirPathCmd_t);
                    
//                     uint16_t len = sizeof(PAYC_SetDirPathCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->paycsetdirpathcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->paycsetdirpathcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->paycsetdirpathcmd, sizeof(PAYC_SetDirPathCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 68: { // paycdownloadcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputText("file name char[32]", command->paycdownloadcmd.Payload.filename, sizeof(command->paycdownloadcmd.Payload.filename));

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(PAYC_DownloadCmd_t)-7};
                    
                    
//                     memcpy(command->paycdownloadcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->paycdownloadcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->paycdownloadcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->paycdownloadcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(PAYC_DownloadCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(PAYC_DownloadCmd_t);
                    
//                     uint16_t len = sizeof(PAYC_DownloadCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->paycdownloadcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->paycdownloadcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->paycdownloadcmd, sizeof(PAYC_DownloadCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 69: { // stxgenericsetvalue
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("reg u8",ImGuiDataType_U8,&command->stxgenericsetvalue.Reg);
//                 ImGui::InputScalar("val u8",ImGuiDataType_U8,&command->stxgenericsetvalue.Val);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(STX_PULSAR_GenericSetValueCmd_t)-7};
                    
                    
//                     memcpy(command->stxgenericsetvalue.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->stxgenericsetvalue.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->stxgenericsetvalue.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->stxgenericsetvalue.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(STX_PULSAR_GenericSetValueCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(STX_PULSAR_GenericSetValueCmd_t);
                    
//                     uint16_t len = sizeof(STX_PULSAR_GenericSetValueCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->stxgenericsetvalue;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->stxgenericsetvalue.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->stxgenericsetvalue, sizeof(STX_PULSAR_GenericSetValueCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 70: { // stxsetvaluebool
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::Checkbox("val", &command->stxsetvaluebool.Val);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(STX_PULSAR_SetValueBoolCmd_t)-7};
                    
                    
//                     memcpy(command->stxsetvaluebool.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->stxsetvaluebool.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->stxsetvaluebool.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->stxsetvaluebool.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(STX_PULSAR_SetValueBoolCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(STX_PULSAR_SetValueBoolCmd_t);
                    
//                     uint16_t len = sizeof(STX_PULSAR_SetValueBoolCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->stxsetvaluebool;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->stxsetvaluebool.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->stxsetvaluebool, sizeof(STX_PULSAR_SetValueBoolCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }
//             case 71: { // stxgenericgetvalue
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("reg u8",ImGuiDataType_U8,&command->stxgenericgetvalue.Reg);
//                 ImGui::InputScalar("size u8",ImGuiDataType_U8,&command->stxgenericgetvalue.Size);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(STX_PULSAR_GenericGetValueCmd_t)-7};
                    
                    
//                     memcpy(command->stxgenericgetvalue.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->stxgenericgetvalue.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->stxgenericgetvalue.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->stxgenericgetvalue.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(STX_PULSAR_GenericGetValueCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(STX_PULSAR_GenericGetValueCmd_t);
                    
//                     uint16_t len = sizeof(STX_PULSAR_GenericGetValueCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->stxgenericgetvalue;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->stxgenericgetvalue.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->stxgenericgetvalue, sizeof(STX_PULSAR_GenericGetValueCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 break;
//             }

//             // PAYR
//             case 72: { // payrburncmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("chip u8",ImGuiDataType_U8,&command->payrburncmd.Chip);
//                 ImGui::InputScalar("pins u8",ImGuiDataType_U8,&command->payrburncmd.Pins);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(PAYR_BurnCmd_t)-7};
                    
                    
//                     memcpy(command->payrburncmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->payrburncmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->payrburncmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->payrburncmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(PAYR_BurnCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(PAYR_BurnCmd_t);
                    
//                     uint16_t len = sizeof(PAYR_BurnCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->payrburncmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->payrburncmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->payrburncmd, sizeof(PAYR_BurnCmd_t));

//                     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u",command->payrburncmd.CmdHeader[0],
//                             command->payrburncmd.CmdHeader[1], command->payrburncmd.CmdHeader[2], command->payrburncmd.CmdHeader[3], command->payrburncmd.CmdHeader[4], command->payrburncmd.CmdHeader[5], command->payrburncmd.CmdHeader[6], command->payrburncmd.CmdHeader[7],
//                             command->payrburncmd.Chip, command->payrburncmd.Pins);
//                 break;
//             }
//             case 73: { // payrburnloofcmd
//                 static uint16_t msgid = 0;
//                 static uint8_t fnccode = 0;

//                 ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
//                 ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

//                 ImGui::InputScalar("chip u8",ImGuiDataType_U8,&command->payrburnloofcmd.Chip);
//                 ImGui::Checkbox("Mainburn bool", &command->payrburnloofcmd.MainBurn);
//                 ImGui::InputScalar("Burn time sec u8",ImGuiDataType_U8,&command->payrburnloofcmd.BurnTimeSeconds);

//                 if(ImGui::Button("Generate CMD")) {
//                     msgid = htons(msgid);
//                     uint8_t stream[2] ={0,};
//                     uint8_t sequence[2] = {0xC0,0x00};
                    
                    
//                     uint8_t length[2] = {0x00, sizeof(PAYR_BurnLoopCmd_t)-7};
                    
                    
//                     memcpy(command->payrburnloofcmd.CmdHeader, &msgid, sizeof(uint16_t));
//                     memcpy(command->payrburnloofcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
//                     memcpy(command->payrburnloofcmd.CmdHeader + 4, length, sizeof(uint16_t));
//                     memcpy(command->payrburnloofcmd.CmdHeader + 7, &fnccode, sizeof(uint8_t)); 
//                     pthread_join(p_thread[4], NULL);
//                     packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(PAYR_BurnLoopCmd_t)); // 8 is for data
//                     TestPacket->Identifier = HVD_TEST;
//                     TestPacket->PacketType = MIM_PT_TMTC_TEST;
//                     TestPacket->Length = sizeof(PAYR_BurnLoopCmd_t);
                    
//                     uint16_t len = sizeof(PAYR_BurnLoopCmd_t);
//                     const uint8_t *byteptr = (uint8_t *)&command->payrburnloofcmd;
//                     uint8_t checksum = 0xFF;
//                     while (len--) {
//                         checksum ^= *(byteptr++);
//                     }
//                     memcpy(command->payrburnloofcmd.CmdHeader+6, &checksum, sizeof(uint8_t));
//                     memcpy(TestPacket->Data, &command->payrburnloofcmd, sizeof(PAYR_BurnLoopCmd_t));

//                     //pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
//                     msgid = htons(msgid);
//                 }
//                 ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u",command->payrburnloofcmd.CmdHeader[0],
//                             command->payrburnloofcmd.CmdHeader[1], command->payrburnloofcmd.CmdHeader[2], command->payrburnloofcmd.CmdHeader[3], command->payrburnloofcmd.CmdHeader[4], command->payrburnloofcmd.CmdHeader[5], command->payrburnloofcmd.CmdHeader[6], command->payrburnloofcmd.CmdHeader[7],
//                             command->payrburnloofcmd.Chip, command->payrburnloofcmd.MainBurn, command->payrburnloofcmd.BurnTimeSeconds);
//                 break;
//             }

            case 74: { // COSMIC santsetsettingscmd command
                static uint16_t msgid = 0;
                static uint8_t fnccode = 0;

                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

                ImGui::InputScalar("min_deploy u16",ImGuiDataType_U16,&command->santsetsettingscmd.min_deploy);
                ImGui::InputScalar("backup u8", ImGuiDataType_U8, &command->santsetsettingscmd.backup);
                ImGui::InputScalar("max_burn_duration u8",ImGuiDataType_U8,&command->santsetsettingscmd.max_burn_duration);

                if(ImGui::Button("Generate CMD")) {
                    msgid = htons(msgid);
                    uint8_t stream[2] ={0,};
                    uint8_t sequence[2] = {0xC0,0x00};
                    
                    
                    uint8_t length[2] = {0x00, sizeof(SANT_SetSettingsCmd_t)-7};
                    
                    
                    memcpy(command->santsetsettingscmd.CmdHeader, &msgid, sizeof(uint16_t));
                    memcpy(command->santsetsettingscmd.CmdHeader + 2, sequence, sizeof(uint16_t));
                    memcpy(command->santsetsettingscmd.CmdHeader + 4, length, sizeof(uint16_t));
                    memcpy(command->santsetsettingscmd.CmdHeader + 6, &fnccode, sizeof(uint8_t)); 
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(SANT_SetSettingsCmd_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = sizeof(SANT_SetSettingsCmd_t);
                    
                    uint16_t len = sizeof(SANT_SetSettingsCmd_t);
                    const uint8_t *byteptr = (uint8_t *)&command->santsetsettingscmd;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    memcpy(command->santsetsettingscmd.CmdHeader+7, &checksum, sizeof(uint8_t));
                    memcpy(TestPacket->Data, &command->santsetsettingscmd, sizeof(SANT_SetSettingsCmd_t));

                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }

                ImGui::Text("Header: %02X %02X %02X %02X %02X %02X %02X %02X",
                            command->santsetsettingscmd.CmdHeader[0],
                            command->santsetsettingscmd.CmdHeader[1],
                            command->santsetsettingscmd.CmdHeader[2],
                            command->santsetsettingscmd.CmdHeader[3],
                            command->santsetsettingscmd.CmdHeader[4],
                            command->santsetsettingscmd.CmdHeader[5],
                            command->santsetsettingscmd.CmdHeader[6],
                            command->santsetsettingscmd.CmdHeader[7]);

                ImGui::Text("Params: min_deploy=%u, backup=%u, max_burn_duration=%u",
                            command->santsetsettingscmd.min_deploy,
                            command->santsetsettingscmd.backup,
                            command->santsetsettingscmd.max_burn_duration);
            
                break;
            }

            case 75: { // BEE-1000 UANT uantburnchannelcmd
                static uint16_t msgid = 0;
                static uint8_t fnccode = 0;

                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

                ImGui::InputScalar("addr u8",ImGuiDataType_U8,&command->uantburnchannelcmd.addr);
                ImGui::InputScalar("channel u8", ImGuiDataType_U8, &command->uantburnchannelcmd.channel);
                ImGui::InputScalar("duration u8",ImGuiDataType_U8,&command->uantburnchannelcmd.duration);

                if(ImGui::Button("Generate CMD")) {
                    msgid = htons(msgid);
                    uint8_t stream[2] ={0,};
                    uint8_t sequence[2] = {0xC0,0x00};
                    
                    
                    uint8_t length[2] = {0x00, sizeof(UANT_BurnChannelCmd_t)-7};
                    
                    
                    memcpy(command->uantburnchannelcmd.CmdHeader, &msgid, sizeof(uint16_t));
                    memcpy(command->uantburnchannelcmd.CmdHeader + 2, sequence, sizeof(uint16_t));
                    memcpy(command->uantburnchannelcmd.CmdHeader + 4, length, sizeof(uint16_t));
                    memcpy(command->uantburnchannelcmd.CmdHeader + 6, &fnccode, sizeof(uint8_t)); 
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(UANT_BurnChannelCmd_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = sizeof(UANT_BurnChannelCmd_t);
                    
                    uint16_t len = sizeof(UANT_BurnChannelCmd_t);
                    const uint8_t *byteptr = (uint8_t *)&command->uantburnchannelcmd;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    memcpy(command->uantburnchannelcmd.CmdHeader+7, &checksum, sizeof(uint8_t));
                    memcpy(TestPacket->Data, &command->uantburnchannelcmd, sizeof(UANT_BurnChannelCmd_t));

                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
               ImGui::Text("Header: %02X %02X %02X %02X %02X %02X %02X %02X",
                            command->uantburnchannelcmd.CmdHeader[0],
                            command->uantburnchannelcmd.CmdHeader[1],
                            command->uantburnchannelcmd.CmdHeader[2],
                            command->uantburnchannelcmd.CmdHeader[3],
                            command->uantburnchannelcmd.CmdHeader[4],
                            command->uantburnchannelcmd.CmdHeader[5],
                            command->uantburnchannelcmd.CmdHeader[6],
                            command->uantburnchannelcmd.CmdHeader[7]);

                ImGui::Text("Params: addr=%u, channel=%u, duration=%u",
                            command->uantburnchannelcmd.addr,
                            command->uantburnchannelcmd.channel,
                            command->uantburnchannelcmd.duration);



                break;
            }


            case 76: { // BEE-1000 UANT uantsetsettingscmd
                static uint16_t msgid = 0;
                static uint8_t fnccode = 0;

                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

                ImGui::InputScalar("addr u8",ImGuiDataType_U8,&command->uantsetsettingscmd.addr);
                ImGui::InputScalar("MinutesUntilDeploy u16", ImGuiDataType_U16, &command->uantsetsettingscmd.MinutesUntilDeploy);
                ImGui::InputScalar("BackupActive u8",ImGuiDataType_U8,&command->uantsetsettingscmd.BackupActive);
                ImGui::InputScalar("MaxBurnDuration u8",ImGuiDataType_U8,&command->uantsetsettingscmd.MaxBurnDuration);

                if(ImGui::Button("Generate CMD")) {
                    msgid = htons(msgid);
                    uint8_t stream[2] ={0,};
                    uint8_t sequence[2] = {0xC0,0x00};
                    
                    
                    uint8_t length[2] = {0x00, sizeof(UANT_SetSettingsCmd_t)-7};
                    
                    
                    memcpy(command->uantsetsettingscmd.CmdHeader, &msgid, sizeof(uint16_t));
                    memcpy(command->uantsetsettingscmd.CmdHeader + 2, sequence, sizeof(uint16_t));
                    memcpy(command->uantsetsettingscmd.CmdHeader + 4, length, sizeof(uint16_t));
                    memcpy(command->uantsetsettingscmd.CmdHeader + 6, &fnccode, sizeof(uint8_t)); 
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(UANT_SetSettingsCmd_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = sizeof(UANT_SetSettingsCmd_t);
                    
                    command->uantsetsettingscmd.CmdHeader[7] = 0x00;
                    uint16_t len = sizeof(UANT_SetSettingsCmd_t);
                    const uint8_t *byteptr = (uint8_t *)&command->uantsetsettingscmd;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    memcpy(command->uantsetsettingscmd.CmdHeader+7, &checksum, sizeof(uint8_t));
                    memcpy(TestPacket->Data, &command->uantsetsettingscmd, sizeof(UANT_SetSettingsCmd_t));

                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                ImGui::Text("Header: %02X %02X %02X %02X %02X %02X %02X %02X",
                            command->uantsetsettingscmd.CmdHeader[0],
                            command->uantsetsettingscmd.CmdHeader[1],
                            command->uantsetsettingscmd.CmdHeader[2],
                            command->uantsetsettingscmd.CmdHeader[3],
                            command->uantsetsettingscmd.CmdHeader[4],
                            command->uantsetsettingscmd.CmdHeader[5],
                            command->uantsetsettingscmd.CmdHeader[6],
                            command->uantsetsettingscmd.CmdHeader[7]);

                        ImGui::Text("Params: addr=%u, MinutesUntilDeploy=%u, BackupActive=%u, MaxBurnDuration=%u",
                            command->uantsetsettingscmd.addr,
                            command->uantsetsettingscmd.MinutesUntilDeploy,
                            command->uantsetsettingscmd.BackupActive,
                            command->uantsetsettingscmd.MaxBurnDuration);

                break;
            }


            case 77: { // BEE-1000 UANT uantautodeploycmd
                static uint16_t msgid = 0;
                static uint8_t fnccode = 0;

                ImGui::InputScalar("msgid",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode",ImGuiDataType_U8, &fnccode);

                ImGui::InputScalar("SecondsDelay u16",ImGuiDataType_U16,&command->uantautodeploycmd.SecondsDelay);
                ImGui::InputScalar("addrslave1 u8", ImGuiDataType_U8, &command->uantautodeploycmd.addrslave1);
                ImGui::InputScalar("addrslave2 u8",ImGuiDataType_U8,&command->uantautodeploycmd.addrslave2);

                if(ImGui::Button("Generate CMD")) {
                    msgid = htons(msgid);
                    uint8_t stream[2] ={0,};
                    uint8_t sequence[2] = {0xC0,0x00};
                    
                    
                    uint8_t length[2] = {0x00, sizeof(UANT_AutoDeployCmd_t)-7};
                    
                    
                    memcpy(command->uantautodeploycmd.CmdHeader, &msgid, sizeof(uint16_t));
                    memcpy(command->uantautodeploycmd.CmdHeader + 2, sequence, sizeof(uint16_t));
                    memcpy(command->uantautodeploycmd.CmdHeader + 4, length, sizeof(uint16_t));
                    memcpy(command->uantautodeploycmd.CmdHeader + 6, &fnccode, sizeof(uint8_t)); 
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(UANT_AutoDeployCmd_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = sizeof(UANT_AutoDeployCmd_t);
                    
                    uint16_t len = sizeof(UANT_AutoDeployCmd_t);
                    const uint8_t *byteptr = (uint8_t *)&command->uantautodeploycmd;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    memcpy(command->uantautodeploycmd.CmdHeader+7, &checksum, sizeof(uint8_t));
                    memcpy(TestPacket->Data, &command->uantautodeploycmd, sizeof(UANT_AutoDeployCmd_t));

                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                    msgid = htons(msgid);
                }
                ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u",command->uantautodeploycmd.CmdHeader[0],
                            command->uantautodeploycmd.CmdHeader[1], command->uantautodeploycmd.CmdHeader[2], command->uantautodeploycmd.CmdHeader[3], command->uantautodeploycmd.CmdHeader[4], command->uantautodeploycmd.CmdHeader[5], command->uantautodeploycmd.CmdHeader[6], command->uantautodeploycmd.CmdHeader[7],
                            command->uantautodeploycmd.SecondsDelay, command->uantautodeploycmd.addrslave1, command->uantautodeploycmd.addrslave2);
                ImGui::Text("Header: %02X %02X %02X %02X %02X %02X %02X %02X",
                            command->uantautodeploycmd.CmdHeader[0],
                            command->uantautodeploycmd.CmdHeader[1],
                            command->uantautodeploycmd.CmdHeader[2],
                            command->uantautodeploycmd.CmdHeader[3],
                            command->uantautodeploycmd.CmdHeader[4],
                            command->uantautodeploycmd.CmdHeader[5],
                            command->uantautodeploycmd.CmdHeader[6],
                            command->uantautodeploycmd.CmdHeader[7]);

                ImGui::Text("Params: SecondsDelay=%u, addrslave1=%u, addrslave2=%u",
                            command->uantautodeploycmd.SecondsDelay,
                            command->uantautodeploycmd.addrslave1,
                            command->uantautodeploycmd.addrslave2);

                
                
                break;
            }

            
            case 78: { 
                static uint16_t msgid = 0;
                static uint8_t  fnccode = 0;
                static char     name_buf[64] = "";


                ImGui::InputScalar("msgid",   ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode", ImGuiDataType_U8,  &fnccode);

                ImGui::InputText("name", name_buf, sizeof(name_buf));
                ImGui::InputScalar("start_byte u32", ImGuiDataType_U32, &command->ftpsendfilecmd.start_byte);
                ImGui::InputScalar("end_byte u32",   ImGuiDataType_U32, &command->ftpsendfilecmd.end_byte);
                ImGui::InputScalar("interval u8",    ImGuiDataType_U8,  &command->ftpsendfilecmd.interval);

                ImGui::Separator();
                ImGui::Text("Padding (3 bytes)");
                ImGui::Separator();

                ImGui::InputScalar("padding[0] u8", ImGuiDataType_U8, &command->ftpsendfilecmd.padding[0]);
                ImGui::InputScalar("padding[1] u8", ImGuiDataType_U8, &command->ftpsendfilecmd.padding[1]);
                ImGui::InputScalar("padding[2] u8", ImGuiDataType_U8, &command->ftpsendfilecmd.padding[2]);

                if (ImGui::Button("Generate CMD")) {
                    uint16_t msgid_be = htons(msgid);

                    memset(command->ftpsendfilecmd.name, 0, sizeof(command->ftpsendfilecmd.name));
                    strncpy(reinterpret_cast<char*>(command->ftpsendfilecmd.name), name_buf, sizeof(command->ftpsendfilecmd.name)-1);

                    uint8_t sequence[2] = {0xC0, 0x00};
                    uint8_t length[2]   = {0x00, 0x4D};

                    memcpy(command->ftpsendfilecmd.CmdHeader + 0, &msgid_be, sizeof(uint16_t));
                    memcpy(command->ftpsendfilecmd.CmdHeader + 2, sequence,  sizeof(uint16_t));
                    memcpy(command->ftpsendfilecmd.CmdHeader + 4, length,    sizeof(uint16_t));
                    memcpy(command->ftpsendfilecmd.CmdHeader + 6, &fnccode,  sizeof(uint8_t));

                    command->ftpsendfilecmd.CmdHeader[7] = 0x00;

                    pthread_join(p_thread[4], NULL);

                    packetsign* TestPacket = (packetsign*)malloc(2 + 2 + 4 + sizeof(FTP_sendfileCmd_t));
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length     = sizeof(FTP_sendfileCmd_t);

                    uint16_t len = sizeof(FTP_sendfileCmd_t);
                    const uint8_t* byteptr = reinterpret_cast<const uint8_t*>(&command->ftpsendfilecmd);
                    uint8_t checksum = 0xFF;
                    while (len--) checksum ^= *(byteptr++);

                    memcpy(command->ftpsendfilecmd.CmdHeader + 7, &checksum, sizeof(uint8_t));

                    memcpy(TestPacket->Data, &command->ftpsendfilecmd, sizeof(FTP_sendfileCmd_t));

                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void*)TestPacket);


                }

                ImGui::Text("Header: %02X %02X %02X %02X %02X %02X %02X %02X",
                    command->ftpsendfilecmd.CmdHeader[0],
                    command->ftpsendfilecmd.CmdHeader[1],
                    command->ftpsendfilecmd.CmdHeader[2],
                    command->ftpsendfilecmd.CmdHeader[3],
                    command->ftpsendfilecmd.CmdHeader[4],
                    command->ftpsendfilecmd.CmdHeader[5],
                    command->ftpsendfilecmd.CmdHeader[6],
                    command->ftpsendfilecmd.CmdHeader[7]);

                ImGui::Text("Params:");
                ImGui::Text("  name        = %s", name_buf);
                ImGui::Text("  start_byte  = %u", command->ftpsendfilecmd.start_byte);
                ImGui::Text("  end_byte    = %u", command->ftpsendfilecmd.end_byte);
                ImGui::Text("  interval    = %u", command->ftpsendfilecmd.interval);
                ImGui::Text("  padding[0..2] = {%u, %u, %u}",
                    command->ftpsendfilecmd.padding[0],
                    command->ftpsendfilecmd.padding[1],
                    command->ftpsendfilecmd.padding[2]);

                break;
            }            


            case 79: { // FTP_filenameCmd
                static uint16_t msgid = 0;
                static uint8_t fnccode = 0;
                static char path_buf[64] = "";

                ImGui::InputScalar("msgid", ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode", ImGuiDataType_U8, &fnccode);
                ImGui::InputText("path", path_buf, sizeof(path_buf));

                if (ImGui::Button("Generate CMD")) {
                    uint16_t msgid_be = htons(msgid);
                    memset(command->ftpfilenamecmd.path, 0, sizeof(command->ftpfilenamecmd.path));
                    strncpy(command->ftpfilenamecmd.path, path_buf, sizeof(command->ftpfilenamecmd.path) - 1);
                    uint8_t sequence[2] = {0xC0, 0x00};
                    uint8_t length[2] = {0x00, 0x41};
                    memcpy(command->ftpfilenamecmd.CmdHeader + 0, &msgid_be, sizeof(uint16_t));
                    memcpy(command->ftpfilenamecmd.CmdHeader + 2, sequence, sizeof(uint16_t));
                    memcpy(command->ftpfilenamecmd.CmdHeader + 4, length, sizeof(uint16_t));
                    memcpy(command->ftpfilenamecmd.CmdHeader + 6, &fnccode, sizeof(uint8_t));
                    command->ftpfilenamecmd.CmdHeader[7] = 0x00;
                    pthread_join(p_thread[4], NULL);
                    packetsign* TestPacket = (packetsign*)malloc(2 + 2 + 4 + sizeof(FTP_filenameCmd_t));
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = sizeof(FTP_filenameCmd_t);
                    uint16_t len = sizeof(FTP_filenameCmd_t);
                    const uint8_t* byteptr = reinterpret_cast<const uint8_t*>(&command->ftpfilenamecmd);
                    uint8_t checksum = 0xFF;
                    while (len--) checksum ^= *(byteptr++);
                    memcpy(command->ftpfilenamecmd.CmdHeader + 7, &checksum, sizeof(uint8_t));
                    memcpy(TestPacket->Data, &command->ftpfilenamecmd, sizeof(FTP_filenameCmd_t));
                    pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void*)TestPacket);
                }

                ImGui::Text("Header: %02X %02X %02X %02X %02X %02X %02X %02X",
                    command->ftpfilenamecmd.CmdHeader[0],
                    command->ftpfilenamecmd.CmdHeader[1],
                    command->ftpfilenamecmd.CmdHeader[2],
                    command->ftpfilenamecmd.CmdHeader[3],
                    command->ftpfilenamecmd.CmdHeader[4],
                    command->ftpfilenamecmd.CmdHeader[5],
                    command->ftpfilenamecmd.CmdHeader[6],
                    command->ftpfilenamecmd.CmdHeader[7]);

                ImGui::Text("Params: path = %s", path_buf);
                break;
            }
        }


        // if(ImGui::Button("Generate CMD")) {
        //     pthread_join(p_thread[4], NULL);
        //     packetsign * TestPacket = (packetsign *)malloc(2+2+4+8); // 8 is for data
        //     TestPacket->Identifier = HVD_TEST;
        //     TestPacket->PacketType = MIM_PT_TMTC_TEST;
        //     TestPacket->Length = 8;
        //     memset(TestPacket->Data,0,TestPacket->Length);
        //     uint8_t cmd[8] = {0,};
        //     memcpy(cmd, &msgid, sizeof(uint16_t)); // msgid
        //     memcpy(cmd + 7, &fnccode, sizeof(uint8_t)); // fnccode
        //     //memcpy(cmd + 8, &arg, sizeof(uint8_t)); // user input
        //     cmd[0] = cmd[0]^cmd[1];
        //     cmd[1] = cmd[0]^cmd[1];
        //     cmd[0] = cmd[0]^cmd[1];
        //     cmd[2]=0xc0;
        //     cmd[3]=0x00;
        //     cmd[4]=0x00;
        //     cmd[5]=0x01; // length = n+7
        //     //cmd[6]=0x00; // fnccode
        //     uint16_t len = 8;
        //     const uint8_t *byteptr = cmd;
        //     uint8_t checksum = 0xFF;
        //     while (len--) {
        //         checksum ^= *(byteptr++);
        //     }
        //     cmd[6]=checksum; // checksum
            
        //     memcpy(TestPacket->Data,cmd,sizeof(cmd));
        //     pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
        //     ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
        //             cmd[0],cmd[1],cmd[2],cmd[3],cmd[4],cmd[5],cmd[6],cmd[7],cmd[8]);
        //     //show_typingWindow = false;
        // }
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
{   // Common type cmd
    snprintf(Templabels[0], 64, "no arg");
    snprintf(Templabels[1], 64, "u8");
    snprintf(Templabels[2], 64, "u16");
    snprintf(Templabels[3], 64, "u32");
    snprintf(Templabels[4], 64, "u64");
    snprintf(Templabels[5], 64, "s8");
    snprintf(Templabels[6], 64, "s16");
    snprintf(Templabels[7], 64, "s32");
    snprintf(Templabels[8], 64, "s64");
    snprintf(Templabels[9], 64, "float");
    snprintf(Templabels[74], 64, "COSMIC SANT Set Settings Command");
    snprintf(Templabels[75], 64, "BEE-1000 UANT Burn Channel Command");
    snprintf(Templabels[76], 64, "BEE-1000 UANT Set Settings Command");
    snprintf(Templabels[77], 64, "BEE-1000 UANT Auto Deploy Command");
    snprintf(Templabels[78], 64, "COSMIC FTP Send File Command");
    snprintf(Templabels[79], 64, "COSMIC FTP File Name Command");
    // // EPS unusual
    // snprintf(Templabels[10], 64, "EPS_P60_Dock(PDU)SetChannelSingle");
    // snprintf(Templabels[11], 64, "EPS_P60_Dock(PDU)GetChannelSingle & AcuSetMpptMode");
    // snprintf(Templabels[12], 64, "EPS_P60_TableGet(Statistic)Cmd_t");
    // snprintf(Templabels[13], 64, "EPS_P60_TableSave(Load)Cmd_t");
    // snprintf(Templabels[14], 64, "EPS_P60_ParamGetCmd_t");
    // snprintf(Templabels[15], 64, "EPS_P60_ParamGetArrayCmd_t");
    // snprintf(Templabels[16], 64, "EPS_P60_DockSetChannelsCmd_t");
    // snprintf(Templabels[17], 64, "EPS_P60_PduSetChannelsCmd_t");

    // //GRX unusual
    // snprintf(Templabels[18], 64, "GRX assemblepublishcmd");
    // snprintf(Templabels[19], 64, "GRX cmdlog");
    // snprintf(Templabels[20], 64, "GRX cmdlogontime");
    // snprintf(Templabels[21], 64, "GRX cmdlogonchanged");
    // snprintf(Templabels[22], 64, "GRX cmdlogon new");
    // snprintf(Templabels[23], 64, "GRX cmd unlog");
    // snprintf(Templabels[24], 64, "GRX cmd unlog all");
    // snprintf(Templabels[25], 64, "GRX elevation cutoff");
    // snprintf(Templabels[26], 64, "GRX interface mode");
    // snprintf(Templabels[27], 64, "GRX serial config");
    // snprintf(Templabels[28], 64, "GRX logresister handler");
    // snprintf(Templabels[29], 64, "GRX logunresister handler");
    // snprintf(Templabels[30], 64, "GRX log add callback");
    // snprintf(Templabels[31], 64, "GRX logsethandlerstatus");

    // // ADCS unusual
    // snprintf(Templabels[32], 64, "ADCS unix time cmd");
    // snprintf(Templabels[33], 64, "ADCS reference LLH target command");
    // snprintf(Templabels[34], 64, "ADCS gnss measurement cmd");
    // snprintf(Templabels[35], 64, "ADCS referenceRPY value cmd");
    // snprintf(Templabels[36], 64, "ADCS openloop command MTQ cmd");
    // snprintf(Templabels[37], 64, "ADCS control mode cmd");
    // snprintf(Templabels[38], 64, "ADCS config adcs satellite cmd");
    // snprintf(Templabels[39], 64, "ADCS controller configuration cmd");
    // snprintf(Templabels[40], 64, "ADCS config mag0 orbital cmd");
    // snprintf(Templabels[41], 64, "ADCS mounting configuration cmd");
    // snprintf(Templabels[42], 64, "ADCS estimator configuration cmd");
    // snprintf(Templabels[43], 64, "ADCS config orbit sat param cmd");
    // snprintf(Templabels[44], 64, "ADCS node selection cmd");
    // snprintf(Templabels[45], 64, "ADCS MTQ config cmd");
    // snprintf(Templabels[46], 64, "ADCS Estimation Mode cmd");
    // snprintf(Templabels[47], 64, "ADCS openloop command RWL cmd");
    // snprintf(Templabels[48], 64, "ADCS openloop command HxyzRW cmd");

    // snprintf(Templabels[49], 64, "IFC handle no arg");
    // snprintf(Templabels[50], 64, "IFC handle u8 arg");
    // snprintf(Templabels[51], 64, "IFC handle u32 arg");
    // snprintf(Templabels[52], 64, "IFC write cmd");
    // snprintf(Templabels[53], 64, "IFC read cmd");
    // snprintf(Templabels[54], 64, "IFC gpio num cmd");
    // snprintf(Templabels[55], 64, "IFC io open cmd");
    // snprintf(Templabels[56], 64, "IFC gpio write cmd");
    // snprintf(Templabels[57], 64, "IFC uart set termios cmd");
    // snprintf(Templabels[58], 64, "IFC i2c duplex transfer cmd");
    // snprintf(Templabels[59], 64, "IFC spi duplex transfer cmd");
    // snprintf(Templabels[60], 64, "IFC io handle allocate cmd");

    // snprintf(Templabels[61], 64, "EPS P60 Param Set cmd");

    // snprintf(Templabels[62], 64, "PAYS d1064 read save status cmd");

    // snprintf(Templabels[63], 64, "PAYC set conf cmd");
    // snprintf(Templabels[64], 64, "PAYC snap cmd");
    // snprintf(Templabels[65], 64, "PAYC store cmd");
    // snprintf(Templabels[66], 64, "PAYC store file list location cmd");
    // snprintf(Templabels[67], 64, "PAYC set dir path cmd");
    // snprintf(Templabels[68], 64, "PAYC download cmd");

    // snprintf(Templabels[69], 64, "STX Generic Set value");
    // snprintf(Templabels[70], 64, "STX Set value bool");
    // snprintf(Templabels[71], 64, "STX Generic Get value");

    // snprintf(Templabels[72], 64, "PAYR burn cmd");
    // snprintf(Templabels[73], 64, "PAYR burn loof");

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