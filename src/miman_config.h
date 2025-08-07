#pragma once
#ifndef __MIMAN_CONFIG_H_
#define __MIMAN_CONFIG_H_

#define GS_FTP_INTERNAL_USE 1
#define DEFAULT_CHUNK_SIZE  100
#define FTP_CALLBACK_REQUEST_TOTAL_CHUNKS	1
#define FTP_CALLBACK_REQUEST_CURRENT_CHUNKS	2
#define FTP_CALLBACK_REQUEST_CHUNKSIZE		3
#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdarg.h>
#include <malloc.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <curl/curl.h>
#include <curl/easy.h>
#include <map>
#include <vector>
#include <stb_image.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <termios.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/time.h>

#include <csp/csp.h>
#include <csp/csp_error.h>
#include <csp/csp_endian.h>
#include <csp/csp_rtable.h>
#include <csp/csp_endian.h>
#include <csp_io.h>
#include <csp/arch/csp_time.h>
#include <csp/arch/csp_queue.h>
#include <csp/arch/csp_thread.h>
#include <csp/arch/csp_system.h>
#include <csp/interfaces/csp_if_kiss.h>
#include <csp/drivers/usart.h>
#include <csp/switch.h>
#include <csp/delay.h>
#include <csp/csp_endian.h>
#include <csp/csp_buffer.h>

#include <gs/csp/csp.h>
#include <gs/csp/port.h>
#include <gs/ftp/client.h>
#include <gs/ftp/types.h>
#include <gs/ftp/internal/types.h>
#include <gs/util/crc32.h>
#include <gs/util/string.h>
#include <gs/util/crc32.h>
#include <gs/util/clock.h>
#include <gs/util/log.h>
#include <gs/util/vmem.h>

#include <GL/glew.h> 
#include <GLFW/glfw3.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"

#include <CoordTopocentric.h>
#include <CoordGeodetic.h>
#include <Observer.h>
#include <SGP4.h>

#include <object/arcball.h>
#include <object/circle.h>
#include <object/cone.h>
#include <object/cube.h>
#include <object/keyframe.h>
#include <object/mass.h>
#include <object/Mesh.h>
#include <object/rectangle.h>
#include <object/spline.h>
#include <object/shader.h>

#include "components/fm.h"
#include "components/eps.h"
#include "components/rwa.h"
#include "components/mtq.h"
#include "components/snsr.h"
#include "components/pay.h"
#include "components/utrx.h"
#include "components/stx.h"
#include "components/ts.h"
#include "components/ccsds.h"

#define CFE_SB_CMD_HDR_SIZE 8

void todaystr(char * str, int length);
void * AmpTimer(void *);
void printftp(const char * input, ...);

/* Setup Configuration */
// Initial Settings
typedef struct {
    char Transciever_devname[256];
    char Rotator_devname[256];
    char Switch_devname[256];
    char DebugFilePath[256];
    char S_Username[64];
    char S_Address[64];
    char S_passwd[64];

    uint32_t Transceiver_baud;
    uint32_t Rotator_baud;
    uint32_t Switch_baud;

    uint8_t gs100_node;
    uint8_t kiss_node;
    uint8_t obc_node;
    uint8_t ax100_node;

    double default_freq;
    uint16_t pingsize;

    uint32_t default_timeout;
    uint32_t guard_delay;
    uint32_t queue_delay;
    
}__attribute__((packed)) Setup;

class Console {
private :
    float _x_pos, _y_pos, _width, _height;
    ImGuiWindowFlags    _window_flags;
    ImVector<char*>     _items;
    std::fstream        _history_fs;
    std::fstream        _debug_fs;
    time_t              _now;
    tm                  _now_time;
    bool                _push_to_bottom;
public:
    void Initializer()
    {
        this -> _window_flags = ImGuiWindowFlags_NoMove |
                                ImGuiWindowFlags_NoResize |
                                ImGuiWindowFlags_NoSavedSettings |
                                ImGuiWindowFlags_NoCollapse |
                                ImGuiWindowFlags_NoBringToFrontOnFocus|
                                ImGuiWindowFlags_HorizontalScrollbar|
                                ImGuiWindowFlags_NoTitleBar;
        ClearLog();

        char date_and_time_buf[32];
        _now = time(NULL);
        _now_time = *localtime(&_now);

        strftime(date_and_time_buf, sizeof(date_and_time_buf), "%Y-%m-%d %H:%M\n", &_now_time);
        _history_fs.write(date_and_time_buf, strlen(date_and_time_buf));
    }
    Console()
    : _x_pos(0), _y_pos(0), _width(200), _height(50), _push_to_bottom(false), _history_fs("./log/history.log", std::fstream::in | std::fstream::out | std::fstream::app), _debug_fs("./log/debug.log", std::fstream::in | std::fstream::out | std::fstream::app)
    {
        Initializer();
    }
    Console(float x_pos, float y_pos, float width, float height, const char * const log_name = "./log/history.log", const char * const debug_name = "./log/debug.log")
    : _x_pos(x_pos), _y_pos(y_pos), _width(width), _height(height), _push_to_bottom(false), _history_fs(log_name, std::fstream::in | std::fstream::out | std::fstream::app), _debug_fs(debug_name, std::fstream::in | std::fstream::out | std::fstream::app)
    {
        Initializer();
    }
    ~Console()
    {
        ClearLog();
        _history_fs.close();
        _debug_fs.close();
    }

    Console& operator = (const Console& rhs)
    {
        this -> _x_pos = rhs._x_pos;
        this -> _y_pos = rhs._y_pos;
        this -> _width = rhs._width;
        this -> _height = rhs._height;
        return *this;
    }

    int Stricmp(const char* str1, const char* str2);
    int Strnicmp(const char* str1, const char* str2, int n);
    char* Strdup(const char *str);
    void Strtrim(char* str);

    void ClearLog();
    void AddLog(const char* fmt, ...) IM_FMTARGS(2);
    void DelStartingCharLog(const char* fmt);
    void DelPrefixLog(const char* fmt);
    void Draw(const char* title, bool* p_open, float fontscale);
    void ChangeWindowSize(float x_pos, float y_pos, float width, float height);
};

class SatelliteObject{
private :
    char _name[30];
    DateTime start_date;
    DateTime end_date;

public :
    bool use, cal;
    Tle tle;
    SGP4 sgp4 = SGP4();
    Observer obs;
    Eci _eci, _eci_1secfuture;
    Eci AOSeci[64];
    CoordTopocentric topo, _topo_1secfuture;
    CoordTopocentric AOStopo[64];
    CoordGeodetic geo;
    DateTime _nextaos[64];
    DateTime _nextlos[64];
    double _max_elevation[64];

    double path_az[1024];
    double path_el[1024];

    double futuretime = 3.0f;
    int CrossPath = 0;


    SatelliteObject(Observer InputObserver)
    :obs(InputObserver)
    {
        this->obs = Observer(InputObserver.GetLocation());
    };

    SatelliteObject(Tle InputTLE, Observer InputObserver, bool usage = false, bool calculation = true)
    : _name(), use(usage), obs(InputObserver), cal(calculation)
    {
        for(int i = 0; i < 64; i ++)
            _max_elevation[i] = -1;
        tle = Tle(InputTLE);
        this->obs = Observer(InputObserver.GetLocation());
        strcpy(this->_name, this->tle.Name().c_str());
        this->sgp4.SetTle(this->tle);
        if(calculation)
            Initializer();
    };

    

    ~SatelliteObject()
    {
        for(int i = 0; i < 64; i ++)
            _max_elevation[i] = -1;
        this->use = false;
        this->cal = false;
        // this->tle = Tle();
        // this->sgp4 = SGP4();
        memset(&this->start_date, 0, sizeof(DateTime));
        memset(&this->end_date, 0, sizeof(DateTime));
        memset(this->_name, 0, sizeof(this->_name));
        memset(&this->tle, 0, sizeof(Tle));
        memset(&this->sgp4, 0, sizeof(sgp4));
        sgp4 = SGP4();
        memset(&this->obs, 0, sizeof(Observer));
        memset(&this->_eci, 0, sizeof(Eci));
        memset(this->AOSeci, 0, sizeof(AOSeci));
        memset(&this->topo, 0, sizeof(CoordTopocentric));
        memset(this->AOStopo, 0, sizeof(AOStopo));
        memset(&this->geo, 0, sizeof(CoordGeodetic));
        memset(this->_nextaos, 0, sizeof(_nextaos));
        memset(this->_nextlos, 0, sizeof(_nextlos));
        memset(this->path_az, 0, sizeof(path_az));
        memset(this->path_el, 0, sizeof(path_el));

        // this->Initializer();
    };

    void Refresh(Tle InputTLE, Observer InputObserver, bool usage = false, bool calculation = true)
    {
        for(int i = 0; i < 64; i ++)
            this->_max_elevation[i] = -1;
        this->use = false;
        tle = Tle(InputTLE);
        this->obs = Observer(InputObserver.GetLocation());
        strcpy(this->_name, this->tle.Name().c_str());
        this->sgp4.SetTle(this->tle);
        if(calculation)
        {
            Initializer();
            this->cal = true;
        }
        this->use = usage;
            
    };

    void Initializer();
    char * Name();
    int Azimuth();
    int Elevation();
    int Azimuth_1secfuture();
    int Elevation_1secfuture();
    int AOS_Azimuth(int i = 0);
    int AOS_Elevation(int i = 0);
    double MaxElevation(int i = 0);
    float DisplayPoint_Longitude();
    float DisplayPoint_Latitude();
    void Update();
    void GeneratePath();
    bool GeneratePassList(const int time_step);
    double FindMaxElevation(Observer obs, SGP4& sgp4, const DateTime& aos, const DateTime& los);
    DateTime FindCrossingPoint(Observer obs, SGP4& sgp4, const DateTime& initial_time1, const DateTime& initial_time2, bool finding_aos);
};

typedef struct {
    char label[64];
    char remote[256];
    char local[256];
}__attribute__((packed)) TLEinfo;

typedef struct{
	char name[30];
	char local_path[256];
	char remote_path[256];
}__attribute__((packed)) ftpinfo;

typedef struct{
    uint32_t TotalChunks = 0;
    uint32_t CurrentChunk = 0;
    uint32_t ChunkSize = 0;
}__attribute__((packed)) chunkstate_t;

typedef struct{
    FILE * fd;
    char flistbuf[64];
    char fpathbuf[64];
    char fdispbuf[2048];
}__attribute__((packed)) gsftp_listup_t;

typedef struct{
    char from[64];
    char to[64];
}__attribute__((packed)) gsftp_move_t;

typedef struct{
    char from[64];
    char to[64];
}__attribute__((packed)) gsftp_copy_t;

typedef struct{
    char path[64];
}__attribute__((packed)) gsftp_remove_t;

typedef struct{
    char path[64];
}__attribute__((packed)) gsftp_mkdir_t;

typedef struct{
    char path[64];
}__attribute__((packed)) gsftp_rmdir_t;

typedef enum __attribute__ ((packed)) {
    TYPE_SYSCMD         = 1,
    TYPE_SYSCMD_REDIR   = 2,
    TYPE_SET_REDIR_PATH = 3,
    TYPE_DS_CLEANUP     = 4,
    TYPE_HK_REQUEST     = 5,
    TYPE_RESERVED       = 6,
    TYPE_KILL           = 99,
} cmd_type_t;


// State Monitoring
struct StateCheckUnit{
    bool InitializeRequired = false;
    bool RadioInitialize = false;
    bool RotatorConnection = false;
    bool RotatorInitialize = false;
    bool RotatorReading = false;
    bool RotatorReadReady = false;
    bool SwitchConnection = false;
    bool ImGuiLoadState = true;
    bool GS100_Connection = false;
    bool GUI = true;
    bool Display_Setup = true;
    bool Display_paramt0 = false;
    bool Display_paramt1 = false;
    bool Display_paramt5 = false;
    bool Display_CMD = false;
    bool Display_TLE = false;
    bool Display_FDS = false;
    bool Display_Satinfo = false;
    bool Display_load = false;
    bool Display_Sband = false;
    bool SbandUse = false;
    bool TRx_mode = false;
    bool downlink_mode = false;
    bool uplink_mode = false;
    bool ftp_mode = false;
    bool Engage = false;
    bool Doppler = false;
    bool Signaltest = false;
    bool Debugmode = false;
    bool NowTracking = true;
    bool TrackThread = true;
    bool AllThread = true;
    bool Sleep = false;
    bool Autopilot = false;
    bool ModelRefreshRequired = false;
    bool AMPON = false;
    bool AMPmode = true;
    time_t AmpTime;
    struct tm * AmpTM;

    int loadindex;
    int tleallindex;
    int iteration = 3;
    int SpanTime = 1;
    int CMDCount = 0;
    int Successed = 0;
    bool DoBeaconCheck = true;
    bool DoPing = true;
    bool DoCMD = false;
    bool DoFTPDL = false;
    bool DoFTPUL = false;
    bool DoBaudCalibration = false;
    bool DoFreqCalibration = false;

    uint32_t chunk_sz = 180;
    int ftp_version = 2;
    int ftp_task = FTP_UPLOAD_REQUEST;
    char FTPWindowBuffer[64];

    int Paramtype = 0;
    int Shelltype = TYPE_SYSCMD;

    chunkstate_t * ChunkState;
    gsftp_listup_t * gslistup;
    gsftp_move_t * gsmove;
    gsftp_copy_t * gscopy;
    gsftp_remove_t * gsremove;
    gsftp_mkdir_t * gsmkdir;
    gsftp_rmdir_t * gsrmdir;
    

    SatelliteObject * Satellites[32768];
    SatelliteObject * Fatellites;
    TLEinfo * tleinfolistup[64];
    ftpinfo ftplistup[64];
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Calculate single step of crc32
 */
uint32_t chksum_crc32_step(uint32_t crc, uint8_t byte);

/**
 * Caluclate crc32 of a block
 */
uint32_t chksum_crc32(uint8_t *block, unsigned int length);

#ifdef __cplusplus
} /* extern "C" */
#endif

/* Node & Port Configurations */
// Node Configuration
#define TX_PORT                     13

// Port Configuration
#define RPARAM_PORT                 7
#define TEST_PORT		            10
#define	BCN_PORT                    31 //beacon port
#define TRX_PORT                    13
#define FTPFCD_PORT                 15
#define FTPRDP_PORT                 16
#define TASK_SYN	                0x01
#define TASK_ACK                    0x02
#define TASK_EAK                    0x04
#define TASK_RST	                0x08


// ADDED FOR COSMIC
#define RPT_PORT                    25



/* Time Settings*/
#define DOPPLER_TIMESTEP            500


/* Packet Configurations */
// Packet ID Configuration
#define MIM_ID                     42
#define HVD_TEST                   23

// Packet Type Configuration
#define MIM_PT_PING              	1
#define MIM_PT_SIGNAL             	2
#define MIM_PT_DLREQ             	3
#define MIM_PT_MDREQ				4
#define MIM_PT_GTCMD               	5
#define MIM_PT_CMD					6
#define MIM_PT_STCMD				7
#define MIM_PT_NCCMD				8
#define MIM_PT_TEST					0xF0
#define MIM_PT_TMTC_TEST            10

// Packet Parser
#define MIM_HAND_HEADERFIELD		4
#define MIM_HAND_OPTIONFIELD		4
#define MIM_HAND_DATAFDSTART		(MIM_HAND_HEADERFIELD + MIM_HAND_OPTIONFIELD)

// Packet Separator
#define MIM_DLTYPE_HK				1
#define MIM_DLTYPE_AOD				2
#define MIM_DLTYPE_LOG				3
#define MIM_DLTYPE_SNSR             4
#define MIM_DLTYPE_GPS              5
#define MIM_DLTYPE_CTRLO            6


#define MIM_DLSTAT_NEW				1
#define MIM_DLSTAT_OLD				2
#define MIM_DLSTAT_ING				3


/* Telecommunicaion Configurations */
// Telecommunication Length
#define MIM_LEN_PACKET              1024
#define MIM_LEN_BEACON              112

// Telecommunication Default Settings
#define MIM_DEFAULT_TIMEOUT         1000
#define MIM_DEFAULT_DATALEN 		1024


/* CCSDS Primary Standard definitions */
#define CFE_MSG_SIZE_OFFSET         7      /**< \brief CCSDS size offset */
#define CFE_MSG_CCSDSVER_MASK       0xE000 /**< \brief CCSDS version mask */
#define CFE_MSG_CCSDSVER_SHIFT      13     /**< \brief CCSDS version shift */
#define CFE_MSG_TYPE_MASK           0x1000 /**< \brief CCSDS type mask, command when set */
#define CFE_MSG_SHDR_MASK           0x0800 /**< \brief CCSDS secondary header mask, exists when set*/
#define CFE_MSG_APID_MASK           0x07FF /**< \brief CCSDS ApID mask */
#define CFE_MSG_SEGFLG_MASK         0xC000 /**< \brief CCSDS segmentation flag mask, all set = complete packet */
#define CFE_MSG_SEGFLG_CNT          0x0000 /**< \brief CCSDS Segment continuation flag */
#define CFE_MSG_SEGFLG_FIRST        0x4000 /**< \brief CCSDS Segment first flag */
#define CFE_MSG_SEGFLG_LAST         0x8000 /**< \brief CCSDS Segment last flag */
#define CFE_MSG_SEGFLG_UNSEG        0xC000 /**< \brief CCSDS Unsegmented flag */
#define CFE_MSG_SEQCNT_MASK         0x3FFF /**< \brief CCSDS Sequence count mask */


/* V1 Command Message IDs must be 0x18xx */
/* 0x1820 to 0x1827 for this App. */
#define ADCS_CMD_MID            0x1820
#define ADCS_SEND_HK_MID        0x1821
/* V1 Telemetry Message IDs must be 0x08xx */
/* 0x0820 to 0x0827 for this App. */
#define ADCS_HK_TLM_MID         0x0820
#define ADCS_REF_TLM_WHL_MID    0x0821
#define ADCS_REF_TLM_MTQ_MID    0x0822
#define ADCS_LOOP_WAKEUP_MID    0x0823
#define ADCS_OUT_TLM_MID        0x0824

/* V1 Command Message IDs must be 0x18xx */
#define GPS_CMD_MID         0x1828
#define GPS_SEND_HK_MID     0x1829
#define GPS_SEND_MEAS_MID   0x182A
/* V1 Telemetry Message IDs must be 0x08xx */
#define GPS_HK_TLM_MID      0x0828
#define GPS_MEAS_TLM_MID    0x0829

/* V1 Command Message IDs must be 0x18xx */
#define EPS_CMD_MID                 0x1848
#define EPS_SEND_HK_MID             0x1849
#define EPS_WATCHDOG_WAKEUP_MID     0x184A
/* V1 Telemetry Message IDs must be 0x08xx */
#define EPS_HK_TLM_MID              0x0848
#define EPS_BCON_TLM_MID            0x0849
#define EPS_HK_TLM_ENV_MID          0x084A

/* V1 Command Message IDs must be 0x18xx */
#define FM_CMD_MID                  0x1870
// #define FM_CMD_MID                  0x1806
#define FM_SEND_HK_MID              0x1871
#define FM_1HZ_WAKEUP_MID           0x1872
#define FM_SEND_SATMODE_MID         0x1873
/* V1 Telemetry Message IDs must be 0x08xx */
#define FM_HK_TLM_MID               0x0871
#define FM_SATMODE_MID              0x0873

/* V1 Command Message IDs must be 0x18xx */
#define MTQ_CMD_MID         0x1840
#define MTQ_SEND_HK_MID     0x1841
/* V1 Telemetry Message IDs must be 0x08xx */
#define MTQ_HK_TLM_MID      0x0840

/* V1 Command Message IDs must be 0x18xx */
#define PAY_CMD_MID         0x1818
#define PAY_SEND_HK_MID     0x1819
/* V1 Telemetry Message IDs must be 0x08xx */
#define PAY_HK_TLM_MID      0x0818

/* V1 Command Message IDs must be 0x18xx */
#define RWA_CMD_MID         0x1838
#define RWA_SEND_HK_MID     0x1839
#define RWA_SEND_WHL_SPEED_MID  0x183A
/* V1 Telemetry Message IDs must be 0x08xx */
#define RWA_HK_TLM_MID      0x0838
#define RWA_WHLSPEED_TLM_MID    0x0839

/* V1 Command Message IDs must be 0x18xx */
#define SNSR_CMD_MID        0x1830
#define SNSR_SEND_HK_MID    0x1831
#define SNSR_SEND_MEAS_MID    0x1832
/* V1 Telemetry Message IDs must be 0x08xx */
#define SNSR_HK_TLM_MID     0x0830
#define SNSR_MEAS_TLM_MID   0x0831
#define SNSR_SUNDETECTION_INFO_MID   0x0832

/* V1 Command Message IDs must be 0x18xx */
#define STX_CMD_MID         0x1858
#define STX_SEND_HK_MID     0x1859
/* V1 Telemetry Message IDs must be 0x08xx */
#define STX_HK_TLM_MID      0x0858

/* V1 Command Message IDs must be 0x18xx */
#define TO_CMD_MID      0x1890
#define TO_SEND_HK_MID  0x1891
/* V1 Telemetry Message IDs must be 0x08xx */
#define TO_HK_TLM_MID   0x0890

/* V1 Command Message IDs must be 0x18xx */
#define UTRX_CMD_MID        0x1850
#define UTRX_SEND_HK_MID    0x1851
/* V1 Telemetry Message IDs must be 0x08xx */
#define UTRX_HK_TLM_MID     0x0850
#define UTRX_BCON_TLM_MID   0x0851

/* V1 Command Message IDs must be 0x18xx */
#define TS_CMD_MID          0x1885
#define TS_SEND_HK_MID      0x1886
#define TS_1HZ_WAKEUP_MID   0x1887
/* V1 Telemetry Message IDs must be 0x08xx */
#define TS_HK_TLM_MID       0x0885

#define ECM_CMD_MID         0x18E0

/*
** ADCS App command codes
*/
#define ADCS_NOOP_CC            0
#define ADCS_RESET_COUNTERS_CC  1
#define ADCS_PROCESS_CC         2

#define ADCS_SETPARAM_SC        10
#define ADCS_SETPARAM_AUX       11
#define ADCS_SETPARAM_GAIN      12
#define ADCS_SETPARAM_TLE       13
#define ADCS_SETPARAM_TARGET    14

/*
** CI App command codes
*/
#define CI_NOOP_CC              0
#define CI_RESET_COUNTERS_CC    1
#define CI_RESERVED_CC          2


#define CI_GATE_NOOP_CC         20

/*
** EPS App command codes
*/
#define EPS_NOOP_CC                     0

#define EPS_RESET_COUNTERS_CC           1
#define EPS_RESERVED_CC                 2
#define EPS_SYSTEM_RESET_CC             3
#define EPS_HW_NOOP_CC                  4
#define EPS_CANCEL_OPERATION_CC         5
#define EPS_WATCHDOG_CC                 6
#define EPS_CORRECT_TIME_CC             7
#define EPS_OUTPUT_BUS_GROUP_ON_CC      8
#define EPS_OUTPUT_BUS_GROUP_OFF_CC     9
#define EPS_OUTPUT_BUS_GROUP_STATE_CC   10
#define EPS_OUTPUT_BUS_CHANNEL_ON_CC    11
#define EPS_OUTPUT_BUS_CHANNEL_OFF_CC   12
#define EPS_SWITCH_TO_NOMINAL_MODE_CC   13
#define EPS_SWITCH_TO_SAFETY_MODE_CC    14
#define EPS_SET_CONFIG_PARAMETER_CC     15
#define EPS_RESET_CONFIG_PARAMETER_CC   16
#define EPS_RESET_CONFIG_CC             17
#define EPS_LOAD_CONFIG_CC              18
#define EPS_SAVE_CONFIG_CC              19

/*
** FM App command codes
*/

/* Application control commands. */
#define FM_NOOP_CC                          0
#define FM_RESET_COUNTERS_CC                1
#define FM_RESERVED_CC                      2
#define FM_RESET_FM_CC                      3
#define FM_RESET_APP_CC                     4
#define FM_RESET_PROCESSOR_CC               5
#define FM_RESET_POWER_CC                   6

/* Mission scenario commands. */
#define FM_MODE_TRANSFER_CC                 10
#define FM_TERMINATE_EO_CC                  11
#define FM_TERMINATE_EO_OVERRIDE_CC         12
#define FM_START_OBSERVATION_CC             13
#define FM_START_COMMUNICATION_CC           14
#define FM_START_COMMISSIONING_CC           15
#define FM_SET_OPERATION_MODE_CC            16
#define FM_SET_COMMISSIONING_PHASE_CC       17
#define FM_INITIATE_BASELINE_CC             18

/* Ground based operation commands. */
#define FM_SET_SPACECRAFT_TIME_CC           20
#define FM_DEPLOY_ANTENNA_CC                21
#define FM_USE_NOMINAL_BAUDRATES_CC         22

/* Set flag commands. */
#define FM_SET_ANTENNA_DEPLOY_FLAG_CC       30
#define FM_SET_DAYLIGHT_DETECTION_FLAG_CC   31
#define FM_SET_COMM_MISSING_FLAG_CC         32

/* Operations data handling commands. */
#define FM_STORE_OBSDATA_TO_CDS_CC          40
#define FM_RESTORE_OBSDATA_FROM_CDS_CC      41
#define FM_STORE_OBSDATA_TO_SD_CC           42
#define FM_RESTORE_OBSDATA_FROM_SD_CC       43

/*
** GPS App command codes
*/
#define GPS_NOOP_CC                 0
#define GPS_RESET_COUNTERS_CC       1
#define GPS_RESERVED_CC             2
#define GPS_RESET_APP_CC            3
#define GPS_RESET_HW_CC             4
#define GPS_CLEAR_LOGS_CC           5
#define GPS_ENABLE_TIME_TONE_CC     6
#define GPS_DISABLE_TIME_TONE_CC    7
#define GPS_LOG_ONCE_CC             8
#define GPS_LOG_ONTIME_CC           9
#define GPS_LOG_ONCHANGE_CC         10
#define GPS_LOG_REQUEST_DFT_CC      11
#define GPS_SET_IF_MODE_CC          12

/*
** MTQ App command codes
*/
#define MTQ_NOOP_CC             0
#define MTQ_RESET_COUNTERS_CC   1
#define MTQ_RESERVED_CC         2

#define MTQ_RESET               3
#define MTQ_RESET_COMP          4
#define MTQ_SET_DUTY            5
#define MTQ_ENABLE              6
#define MTQ_DISABLE             7

#define MTQ_SET_PERIOD          8
#define MTQ_SET_POLARITY        9

/*
** PAY App command codes
*/
#define PAY_NOOP_CC                 0
#define PAY_RESET_COUNTERS_CC       1
#define PAY_PROCESS_CC              2
#define PAY_CAM_FIND_CC             3
#define PAY_CAM_CONNECT_CC          4
#define PAY_CAM_START_OPERATION_CC  5
#define PAY_CAM_STOP_OPERATION_CC   6
#define PAY_CAM_DOWNLOAD_NEW_IMG_CC 7
#define PAY_CAM_DOWNLOAD_OLD_IMG_CC 8
#define PAY_CAM_SEND_NOARG_CC       9
#define PAY_CAM_SET_EXPOSURE_CC     10
#define PAY_CAM_SET_SCPD_CC         11
#define PAY_BBE_SHUTDOWN_CC         12
#define PAY_INIT_DEVICE_CC          13

/*
** RWA App command codes
*/
#define RWA_NOOP_CC                         0
#define RWA_RESET_COUNTERS_CC               1
#define RWA_RESERVED_CC                     2
#define RWA_RESET_ALL_CC                    3
#define RWA_RESET_WHEEL_CC                  4
#define RWA_SET_WHEEL_REFERENCE_SPEED_CC    5
#define RWA_SET_WHEEL_REFERENCE_SPEED_ALL_AXIS_CC  6
#define RWA_SET_WHEEL_COMMANDED_TORQUE_CC   7
#define RWA_SET_MOTOR_POWER_STATE_CC        8
#define RWA_SET_ENCODER_POWER_STATE_CC      9
#define RWA_SET_HALL_POWER_STATE_CC         10
#define RWA_SET_CONTROL_MODE_CC             11
#define RWA_SET_BACKUP_WHEEL_MODE_CC        12
#define RWA_SET_CLEAR_ERRORS_CC             13
#define RWA_SET_PWM_GAIN_CC                 14
#define RWA_SET_MAIN_GAIN_CC                15
#define RWA_SET_BACKUP_GAIN_CC              16

/*
** SNSR App command codes
*/
#define SNSR_NOOP_CC                            0
#define SNSR_RESET_COUNTERS_CC                  1
#define SNSR_RESERVED_CC                        2
#define SNSR_SUN_ALARM_OFF_CC                   3

#define SNSR_STT_BOOT_CC                        10
#define SNSR_STT_PING_CC                        11
#define SNSR_STT_UNLOCK_CC                      12
#define SNSR_STT_REBOOT_CC                      13
#define SNSR_STT_SET_PARAMS_DFT_CC              14
#define SNSR_STT_SET_PARAM_CC                   15
#define SNSR_STT_SEND_RS485_CC                  16
#define SNSR_STT_INIT_DEVICE_CC                 17

#define SNSR_MMT_RESET_CC                       20
#define SNSR_MMT_SET_INTERNAL_CONTROL0_CC       21
#define SNSR_MMT_SET_INTERNAL_CONTROL2_CC       22
#define SNSR_MMT_WRITE_TO_REGISTER_CC           23

#define SNSR_IMU_RESET_CC                       30
#define SNSR_IMU_SET_GYRO_OFFSET_CC             31
#define SNSR_IMU_SET_CONFIGURATION_CC           32
#define SNSR_IMU_SET_GYRO_CONFIGURATION_CC      33
#define SNSR_IMU_SET_ACCEL_CONFIGURATION_CC     34
#define SNSR_IMU_SET_ACCEL_CONFIGURATION2_CC    35
#define SNSR_IMU_SET_POWER_MANAGEMENT_CC        36
#define SNSR_IMU_WRITE_TO_REGISTER_CC           37

#define SNSR_IMU_ISOLATE_CC                     50
#define SNSR_IMU_RESTORE_CC                     51
#define SNSR_MMT_ISOLATE_CC                     52
#define SNSR_MMT_RESTORE_CC                     53
#define SNSR_FSS_ISOLATE_CC                     54
#define SNSR_FSS_RESTORE_CC                     55
#define SNSR_CSS_ISOLATE_CC                     56
#define SNSR_CSS_RESTORE_CC                     57
#define SNSR_STT_ISOLATE_CC                     58
#define SNSR_STT_RESTORE_CC                     59

#define SNSR_IMU_WHO_I_AM_CC                    100
#define SNSR_MMT_GET_PRODUCT_ID_CC              101

/*
** STX App command codes
*/
#define STX_NOOP_CC                 0
#define STX_RESET_COUNTERS_CC       1
#define STX_PROCESS_CC              2
#define STX_RESET_CC                3
#define STX_SET_CONTROL_MODE_CC     4
#define STX_SET_ENCODER_RATE_CC     5
#define STX_SET_PA_POWER_CC         6
#define STX_SET_SYNTH_OFFSET_CC     7
#define STX_TRANSMIT_READY_CC       8
#define STX_TRANSMIT_DATA_CC        9
#define STX_TRANSMIT_FILE_CC        10
#define STX_TRANSMIT_FILE_LONG_PATH_CC        11
#define STX_TRANSMIT_END_CC         12

/*
** TO App command codes
*/
#define TO_BEACON_CALLSIGN         "MIMAN"
#define TO_BEACON_CALLSIGN_LENGTH  strlen(TO_BEACON_CALLSIGN)

#define TO_NOOP_CC              0
#define TO_RESET_COUNTERS_CC    1
#define TO_PROCESS_CC           2
#define TO_ENABLE_BEACON_CC     3
#define TO_DISABLE_BEACON_CC    4
#define TO_DOWNLINK_QUERY_REPLY_CC 5

#define TO_ENABLE_CC            10

/*
** UTRX App command codes
*/
#define UTRX_NOOP_CC                0
#define UTRX_RESET_COUNTERS_CC      1
#define UTRX_RESERVED_CC            2
#define UTRX_REBOOT_CC              3

#define UTRX_SET_TX_FREQ_CC         10
#define UTRX_SET_TX_BAUD_CC         11
#define UTRX_SET_TX_MODIND_CC       12
#define UTRX_SET_TX_MODE_CC         13

#define UTRX_SET_RX_FREQ_CC         20
#define UTRX_SET_RX_BAUD_CC         21
#define UTRX_SET_RX_MODIND_CC       22
#define UTRX_SET_RX_MODE_CC         23
#define UTRX_SET_RX_BW_CC           24

#define UTRX_STUB_PING_CC           99

/*
** TS App command codes
*/
#define TS_NOOP_CC                      0
#define TS_RESET_COUNTERS_CC            1
#define TS_PROCESS_CC                   2
#define TS_INSERT_SCHEDULE_ENTRY_CC     3
#define TS_CLEAR_SCHEDULE_ENTRY_CC      4
#define TS_CLEAR_SCHEDULE_GROUP_CC      5
#define TS_CLEAR_ALL_SCHEDULE_CC        6

#define ECM_GET_HK_AVG_CC               1
#define ECM_GET_SYSTEM_STATUS_CC        2
#define ECM_GET_OCF_STATE_CC            3
#define ECM_READ_CC                     4

// typedef struct {
//     uint8_t StreamId[2];
//     uint8_t Sequence[2];
//     uint8_t Length[2];
// } CCSDS_PrimaryHeader_t;

typedef struct {
    union {
        uint8_t StreamId[2];
        uint16_t stream;
    };
    union {
        uint8_t Sequence[2];
        uint16_t sequence;
    };
    union {
        uint8_t Length[2];
        uint16_t length;
    };
} __attribute__((packed)) CCSDS_PrimaryHeader_t;

typedef struct {
    CCSDS_PrimaryHeader_t Pri;
} CCSDS_SpacePacket_t;


typedef union {
    CCSDS_SpacePacket_t CCSDS;
    uint8_t Byte[sizeof(CCSDS_SpacePacket_t)];
} CFE_MSG_Message_t;

typedef struct {
    CFE_MSG_Message_t Msg;
    uint8_t FunctionCode;
    uint8_t Checksum;
} CommandHeader_t;

typedef struct {
    uint8_t FunctionCode;
    uint8_t Checksum;
} CFE_MSG_CommandSecondaryHeader_t;

typedef struct {
    uint8_t Time[6];
} CFE_MSG_TelemetrySecondaryHeader_t;

typedef struct {
    CFE_MSG_Message_t Msg;
    CFE_MSG_CommandSecondaryHeader_t Sec;
} CFE_MSG_CommandHeader;

typedef struct {
    CFE_MSG_Message_t                  Msg;
    CFE_MSG_TelemetrySecondaryHeader_t Sec;
    uint8_t                            Spare[4];
} CFE_MSG_TelemetryHeader;

/*************************************************************************/
/*
** Edited by JDM
** Converter of Struct Type between GS - FSW
** Warning! Struct Descriptor differs from FSW codes.
*/
/*************************************************************************/

/*************************************************************************/
/*
** ADCS system
*/
/*************************************************************************/ 

typedef struct
{
    CFE_MSG_CommandHeader CmdHeader; /**< \brief Command header */
} ADCS_NoArgsCmd_t;

typedef ADCS_NoArgsCmd_t ADCS_NoopCmd_t;
typedef ADCS_NoArgsCmd_t ADCS_ResetCountersCmd_t;
typedef ADCS_NoArgsCmd_t ADCS_ProcessCmd_t;


typedef struct {
	double mass, MOI[3][3], inv_MOI[3][3];
	double css_sf[5], mmt_A[3][3], mmt_bias[3], gyro_bias_a[3], gyro_bias_b[3];
} ADCS_ParamsSC;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    ADCS_ParamsSC           Params;
} ADCS_SetParamScCmd_t;


typedef struct {
	uint32_t epochYear;
	float epochDay;
	double Bstar, INC, RAAN, ECC, AOP, MA, MM;
} ADCS_ParamsTLE;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    ADCS_ParamsTLE          Params;
} ADCS_SetParamTleCmd_t;


typedef struct {
    double mu_E, R_E, coef_J2, w_E, wn;
	double tstp;
	uint32_t quit_TRIAD;
	uint8_t num_AD_loop, num_AC_loop, option_prg_orb_est, option_gyroless, option_ignoreSTT, option_ignoreCSS;
	double est_P_atd_STT_initial, est_P_atd_FSS_initial, est_P_atd_CSS_initial, est_P_atd_WSS_initial;
	double GS_sband[3], GS_uhf[3];
	double wrpm_WHLdot_dump, wrpm_WHL_dump_crit, wrpm_WHL_dump_goal;
	double w_standby_detumbling_start, w_standby_detumbling_goal, w_standby_forcedSpin_start, angle_standby_forcedSpin_start, M_standby_forcedSpin[3];
} ADCS_ParamsAux;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    ADCS_ParamsAux          Params;
} ADCS_SetParamAuxCmd_t;


typedef struct {
	double r_Target[3];
    double q_Target[4];
} ADCS_ParamsTarget;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    ADCS_ParamsTarget       Params;
} ADCS_SetParamTargetCmd_t;


typedef struct {
    uint8_t       WhichGain;
    uint8_t       Mode;
    uint16_t      Num;
    double      Gain[31];
} ADCS_SetParamGainPayload_t;

typedef struct {
    CFE_MSG_CommandHeader     CmdHeader;
    ADCS_SetParamGainPayload_t  Payload;
} ADCS_SetParamGainCmd_t;

/*************************************************************************/
/*
** CI system
*/
/*************************************************************************/


/*************************************************************************/
/*
** EPS system
*/
/*************************************************************************/

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
}__attribute__((packed)) EPS_NoArgsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t Arg;
} __attribute__((packed)) EPS_U8ArgCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint16_t Arg;
} __attribute__((packed)) EPS_U16ArgCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    int32_t Arg;
} __attribute__((packed)) EPS_I32ArgCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint16_t ParameterID;
    uint8_t ParameterLength;
    uint8_t Parameter[8];
} __attribute__((packed)) EPS_SetConfigurationParameterCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t ParameterLength;
    uint16_t ParameterID;
} __attribute__((packed)) EPS_ResetConfigurationParamterCmd_t;

typedef EPS_NoArgsCmd_t EPS_NoopCmd_t;
typedef EPS_NoArgsCmd_t EPS_ResetCountersCmd_t;
typedef EPS_NoArgsCmd_t EPS_SystemResetCmd_t;
typedef EPS_NoArgsCmd_t EPS_NoOperationCmd_t;
typedef EPS_NoArgsCmd_t EPS_CancelOperationCmd_t;
typedef EPS_NoArgsCmd_t EPS_WatchdogCmd_t;

typedef EPS_I32ArgCmd_t EPS_CorrectTimeCmd_t;

typedef EPS_U16ArgCmd_t EPS_OutputBusGroupOnCmd_t;
typedef EPS_U16ArgCmd_t EPS_OutputBusGroupOffCmd_t;
typedef EPS_U16ArgCmd_t EPS_OutputBusGroupStateCmd_t;

typedef EPS_U8ArgCmd_t  EPS_OutputBusChannelOnCmd_t;
typedef EPS_U8ArgCmd_t  EPS_OutputBusChannelOffCmd_t;

typedef EPS_NoArgsCmd_t EPS_SwitchToNominalModeCmd_t;
typedef EPS_NoArgsCmd_t EPS_SwitchToSafetyModeCmd_t;
typedef EPS_NoArgsCmd_t EPS_ResetConfigurationCmd_t;
typedef EPS_NoArgsCmd_t EPS_LoadConfigurationCmd_t;
typedef EPS_NoArgsCmd_t EPS_SaveConfigurationCmd_t;

/*************************************************************************/
/*
** FM system
*/
/*************************************************************************/

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; /**< \brief Command header */
} FM_NoArgsCmd_t;

typedef FM_NoArgsCmd_t FM_NoopCmd_t;
typedef FM_NoArgsCmd_t FM_ResetCountersCmd_t;
typedef FM_NoArgsCmd_t FM_ResetFmCmd_t;
typedef FM_NoArgsCmd_t FM_ResetProcessorCmd_t;
typedef FM_NoArgsCmd_t FM_ResetPowerCmd_t;
typedef FM_NoArgsCmd_t FM_TerminateEoCmd_t;
typedef FM_NoArgsCmd_t FM_TerminateEoOverrideCmd_t;
typedef FM_NoArgsCmd_t FM_InitiateBaselineCmd_t;
typedef FM_NoArgsCmd_t FM_DeploayAntennaCmd_t;
typedef FM_NoArgsCmd_t FM_UseNominalBaudRatesCmd_t;
typedef FM_NoArgsCmd_t FM_StoreObsDataToCdsCmd_t;
typedef FM_NoArgsCmd_t FM_RestoreObsDataFromCdsCmd_t;

/*
** FM_RESET_APP_CC
*/
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint32_t AppId;
} __attribute__((packed)) FM_ResetAppCmd_t;


/*
** FM_MODE_TRANSFER_CC
*/
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t Mode;
    uint8_t Submode;
} __attribute__((packed)) FM_ModeTransferCmd_t;


/*
** FM_SET_OPERATION_MODE_CC
*/
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t Mode;
    uint8_t Submode;
} __attribute__((packed)) FM_SetOperationModeCmd_t;


/*
** FM_SET_COMMISSIONING_PHASE_CC
*/
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t CommissioningPhase;
} __attribute__((packed)) FM_SetCommissioningPhaseCmd_t;


/*
** FM_SET_SPACECRAFT_TIME_CC
*/
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint32_t Seconds;
    uint32_t Subseconds;
} __attribute__((packed)) FM_SetSpacecraftTimeCmd_t;


/*
** FM_SET_ANTENNA_DEPLOY_FLAG_CC
*/
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t AntennaDeployFlag;
} __attribute__((packed)) FM_SetAntennaDeployFlagCmd_t;


/*
** FM_SET_DAYLIGHT_DETECTION_FLAG_CC
*/
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t DaylightDetectionFlag;
} __attribute__((packed)) FM_SetDaylightDetectionFlagCmd_t;


/*
** FM_SET_COMM_MISSING_FLAG_CC
*/
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t CommunicationMissing;
} __attribute__((packed)) FM_SetCommMissingFlagCmd_t;

/* Reset */
#define CFE_MISSION_MAX_API_LEN 20
#define CFE_MISSION_MAX_PATH_LEN 64
#define CFE_ES_START_APP_CC 4
#define CFE_ES_RESTART_APP_CC 6
#define CFE_ES_RESTART_CC 2
#define CFE_ES_STOP_APP_CC 5

#define CFE_PSP_RST_TYPE_PROCESSOR 1 /**< Volatile disk, CDS and User Reserved memory may be valid */
#define CFE_PSP_RST_TYPE_POWERON   2 /**< All memory has been cleared */

#define ES_CMD_MID                  0x1806

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; /**< \brief Command header */
} SCH_NoArgsCmd_t;

typedef struct
{
CFE_MSG_CommandHeader CmdHeader;

uint16_t SlotNumber; /**< \brief Slot Number of Activity whose state is to change */
/**< \details Valid Range is zero to (#SCH_TOTAL_SLOTS - 1) */
uint16_t EntryNumber; /**< \brief Entry Number of Activity whose state is to change
\details Valid Range is zero to (#SCH_ENTRIES_PER_SLOT - 1) */

} SCH_EntryCmd_t;


/* MESSAGE ID (CMD) */
#define SCH_CMD_MID                    0x1895 /**< \brief SCH Ground Commands Message ID */


/* COMMAND CODES */
#define SCH_ENABLE_CC           2   /* Enable Schedule Table Entry */
#define SCH_DISABLE_CC          3   /* disable schedule table entry */

// 전체 리스타트 명령.
typedef struct {
    CFE_MSG_CommandHeader CmdHeader; /**< \brief Command header */
} CFE_ES_NoArgsCmd_t;

typedef struct CFE_ES_RestartCmd_Payload
{
    uint16_t RestartType;  // 여기 1을 넣으세요
} CFE_ES_RestartCmd_Payload_t;


typedef struct CFE_ES_RestartCmd
{
    CFE_MSG_CommandHeader     CmdHeader;  // MID = 0x1806, CC = 2
    CFE_ES_RestartCmd_Payload_t Payload;
} CFE_ES_RestartCmd_t; 
 // 전체 리스타트 명령 끝.


 // 앱 리스타트 명령.

typedef struct CFE_ES_AppNameCmd_Payload
{
    char Application[CFE_MISSION_MAX_API_LEN];   // 여기 "FM" (F, M, \0)을 넣으세요.
} CFE_ES_AppNameCmd_Payload_t;


typedef struct CFE_ES_AppNameCmd
{
    CFE_MSG_CommandHeader     CmdHeader;   // MID = 0x1806, CC = 6
    CFE_ES_AppNameCmd_Payload_t Payload;
} CFE_ES_AppNameCmd_t;

typedef CFE_ES_AppNameCmd_t CFE_ES_StopAppCmd_t;
typedef CFE_ES_AppNameCmd_t CFE_ES_RestartAppCmd_t;

 // 앱 리스타트 명령 끝.

 /**
** \brief Start Application Command Payload
**
** For command details, see #CFE_ES_START_APP_CC
**
**/
typedef struct CFE_ES_StartAppCmd_Payload
{
    char Application[CFE_MISSION_MAX_API_LEN];   /**< \brief Name of Application to be started */
    char AppEntryPoint[CFE_MISSION_MAX_API_LEN]; /**< \brief Symbolic name of Application's entry point */
    char AppFileName[CFE_MISSION_MAX_PATH_LEN];  /**< \brief Full path and filename of Application's
                                                    executable image */

    uint32_t StackSize; /**< \brief Desired stack size for the new application */

    uint8_t ExceptionAction; /**< \brief #CFE_ES_ExceptionAction_RESTART_APP=On exception,
                                                       restart Application,
                                                       #CFE_ES_ExceptionAction_PROC_RESTART=On exception,
                                                       perform a Processor Reset */
    uint16_t Priority;           /**< \brief The new Applications runtime priority. */

} CFE_ES_StartAppCmd_Payload_t;

/**
 * \brief Start Application Command
 */
typedef struct CFE_ES_StartApp
{
    CFE_MSG_CommandHeader      CmdHeader; /**< \brief Command header */
    CFE_ES_StartAppCmd_Payload_t Payload;   /**< \brief Command payload */
} CFE_ES_StartAppCmd_t;


/*************************************************************************/
/*
** GPS system
*/
/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
    CFE_MSG_CommandHeader CmdHeader; /**< \brief Command header */
} GPS_NoArgsCmd_t;

/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef GPS_NoArgsCmd_t GPS_NoopCmd_t;
typedef GPS_NoArgsCmd_t GPS_ResetCountersCmd_t;
typedef GPS_NoArgsCmd_t GPS_ResetAppCmd_t;
typedef GPS_NoArgsCmd_t GPS_ResetHwCmd_t;
typedef GPS_NoArgsCmd_t GPS_ClearLogsCmd_t;
typedef GPS_NoArgsCmd_t GPS_EnableTimeToneCmd_t;
typedef GPS_NoArgsCmd_t GPS_DisableTimeToneCmd_t;
typedef GPS_NoArgsCmd_t GPS_LogRequestDftCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    int16_t MsgId;
} __attribute__((packed)) GPS_MsgIdCmd_t;

typedef GPS_MsgIdCmd_t GPS_LogOnceCmd_t;
typedef GPS_MsgIdCmd_t GPS_LogOntimeCmd_t;
typedef GPS_MsgIdCmd_t GPS_LogOnChangeCmd_t;

/*************************************************************************/
/*
** MTQ System
*/
/*************************************************************************/

typedef struct{
    CFE_MSG_CommandHeader CmdHeader;
} __attribute__((packed)) MTQ_NoArgsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t                   Args[3];
} __attribute__((packed)) MTQ_U8Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    int8_t                    Args[3];
} __attribute__((packed)) MTQ_C8ArrCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t                   Args[3];
} __attribute__((packed)) MTQ_U8ArrCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint32_t                  Args[3];
} __attribute__((packed)) MTQ_U32ArrCmd_t;

typedef MTQ_NoArgsCmd_t MTQ_NoopCmd_t;
typedef MTQ_NoArgsCmd_t MTQ_ResetCountersCmd_t;
typedef MTQ_NoArgsCmd_t MTQ_ResetCmd_t;

typedef MTQ_U8Cmd_t     MTQ_ResetCompCmd_t;
typedef MTQ_U8ArrCmd_t  MTQ_EnableCmd_t;
typedef MTQ_U8ArrCmd_t  MTQ_DisableCmd_t;
typedef MTQ_U8ArrCmd_t  MTQ_SetPolarityCmd_t;

typedef MTQ_C8ArrCmd_t  MTQ_SetDutyCmd_t;

typedef MTQ_U32ArrCmd_t MTQ_SetPeriodCmd_t;

/*************************************************************************/
/*
** RWA System
*/
/*************************************************************************/

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
} __attribute__((packed)) RWA_NoArgsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; 
    uint8_t WhlNum;
} __attribute__((packed)) RWA_DmaskCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t WhlNum;
    uint8_t Arg;
} __attribute__((packed)) RWA_U8Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t WhlNum;
    int16_t Arg;
} __attribute__((packed)) RWA_S16Cmd_t;

typedef RWA_NoArgsCmd_t RWA_NoopCmd_t;
typedef RWA_NoArgsCmd_t RWA_ResetCountersCmd_t;
typedef RWA_NoArgsCmd_t RWA_ProcessCmd_t;
typedef RWA_NoArgsCmd_t RWA_ResetAllCmd_t;

typedef RWA_DmaskCmd_t  RWA_ResetWheelCmd_t;
typedef RWA_DmaskCmd_t  RWA_ClearErrorsCmd_t;

typedef RWA_U8Cmd_t     RWA_SetMotorPowerStateCmd_t;
typedef RWA_U8Cmd_t     RWA_SetEncoderPowerStateCmd_t;
typedef RWA_U8Cmd_t     RWA_SetHallPowerStateCmd_t;
typedef RWA_U8Cmd_t     RWA_SetControlModeCmd_t;
typedef RWA_U8Cmd_t     RWA_SetBackupWheelModeCmd_t;

typedef RWA_S16Cmd_t    RWA_SetWheelReferenceSpeedCmd_t;
typedef RWA_S16Cmd_t    RWA_SetWheelCommandedTorqueCmd_t;

typedef struct {
    int16_t K;
    uint8_t Kmultiplier;
} __attribute__((packed)) PwmGain;


typedef struct {
    uint16_t Ki;
    uint8_t KiMultiplier;
    uint16_t Kd;
    uint8_t KdMultiplier;
} __attribute__((packed)) MainGain;


typedef struct {
    uint16_t Ki;
    uint8_t KiMultiplier;
    uint16_t Kd;
    uint8_t KdMultiplier;
} __attribute__((packed)) BackupGain;


typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t WhlNum;
    PwmGain    Input;
} __attribute__((packed)) RWA_SetPwmGainCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t WhlNum;
    MainGain    Input;
} __attribute__((packed)) RWA_SetMainGainCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t WhlNum;
    BackupGain    Input;
} __attribute__((packed)) RWA_SetBackupGainCmd_t;


/* Note that the telemetry header is already aligned to 64-bit. */
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    int16_t WheelRefSpeed[3];
} RWA_SetWheelReferenceSpeedAllAxisCmd_t;

/*************************************************************************/
/*
** Payload system
*/
/*************************************************************************/

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
} PAY_NoArgsCmd_t;

typedef PAY_NoArgsCmd_t PAY_InitDeviceCmd_t;
typedef PAY_NoArgsCmd_t PAY_NoopCmd_t;
typedef PAY_NoArgsCmd_t PAY_ResetCountersCmd_t;
typedef PAY_NoArgsCmd_t PAY_CamFindCmd_t;
typedef PAY_NoArgsCmd_t PAY_CamConnectCmd_t;
typedef PAY_NoArgsCmd_t PAY_CamStartOperationCmd_t;
typedef PAY_NoArgsCmd_t PAY_CamStopOperationCmd_t;
typedef PAY_NoArgsCmd_t PAY_CamDownloadNewImgCmd_t;
typedef PAY_NoArgsCmd_t PAY_BBEShutdownCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint16_t Index;
} PAY_CamDownloadOldImgCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t CmdCode;
} PAY_CamSendNoargCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint32_t Exposure;
} PAY_CamSetExposureCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint32_t SCPD;
} PAY_CamSetScpdCmd_t;

/*************************************************************************/
/*
** SNSR System
*/
/*************************************************************************/

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
} __attribute__((packed)) SNSR_NoArgsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t Arg;
} __attribute__((packed)) SNSR_U8Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    int8_t Arg;
} __attribute__((packed)) SNSR_C8Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint16_t Arg;
} __attribute__((packed)) SNSR_U16Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    int16_t Arg;
} __attribute__((packed)) SNSR_S16Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint32_t Arg;
} __attribute__((packed)) SNSR_U32Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    int32_t Arg;
} __attribute__((packed)) SNSR_I32Cmd_t;


typedef SNSR_NoArgsCmd_t SNSR_NoopCmd_t;
typedef SNSR_NoArgsCmd_t SNSR_ResetCountersCmd_t;
typedef SNSR_NoArgsCmd_t SNSR_SunAlarmOffCmd_t;

/*
** STT system
*/
typedef SNSR_NoArgsCmd_t    SNSR_STT_InitDeviceCmd_t;

typedef SNSR_U8Cmd_t        SNSR_STT_BootCmd_t;

typedef SNSR_U32Cmd_t       SNSR_STT_PingCmd_t;

typedef SNSR_NoArgsCmd_t    SNSR_STT_RebootCmd_t;
typedef SNSR_NoArgsCmd_t    SNSR_STT_SetParamsDftCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t Image;
    uint32_t Code;
} __attribute__((packed)) SNSR_STT_UnlockCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t ParamId;
    uint16_t ParamSize;
    uint8_t Param[1];
} __attribute__((packed)) SNSR_STT_SetParamCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint16_t Length;
    uint8_t Data[1];
} __attribute__((packed)) SNSR_STT_SendRS485Cmd_t;

/*
** MMT Command Messages. 
*/
typedef SNSR_NoArgsCmd_t    SNSR_MMT_ResetCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    bool TM_M;
    bool TM_T;
    bool Start_MDT;
    bool Set;
    bool Reset;
} __attribute__((packed)) SNSR_MMT_SetInternalControl0Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t CM_Freq;
    bool INT_MDT_EN;
    bool INT_Meas_Done_EN;
} __attribute__((packed)) SNSR_MMT_SetInternalControl2Cmd_t;

typedef SNSR_U8Cmd_t    SNSR_MMT_WriteToRegisterCmd_t;
typedef SNSR_NoArgsCmd_t    SNSR_MMT_GetProductIdCmd_t;

/*
** IMU Command Messages. 
*/
typedef SNSR_NoArgsCmd_t SNSR_IMU_ResetCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    int16_t Offset[3];
} __attribute__((packed)) SNSR_IMU_SetGyroOffsetCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t FifoMode;
    uint8_t ExtSyncSet;
    uint8_t ConfigDLPF;
} __attribute__((packed)) SNSR_IMU_SetConfigurationCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t AccelFullScale;
    uint8_t FilterChoice;
} __attribute__((packed)) SNSR_IMU_SetGyroConfigurationCmd_t;

typedef SNSR_U8Cmd_t    SNSR_IMU_SetAccelConfigurationCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t FifoSize;
    uint8_t DEC2_CFG;
    bool AccelFilterChoice;
    uint8_t A_DLPF_CFG;
} __attribute__((packed)) SNSR_IMU_SetAccelConfiguration2Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    bool DEVICE_RESET;
    bool SLEEP;
} __attribute__((packed)) SNSR_IMU_SetPowerManagement1Cmd_t;

typedef SNSR_U8Cmd_t        SNSR_IMU_WriteToRegisterCmd_t;
typedef SNSR_NoArgsCmd_t    SNSR_IMU_WhoAmICmd_t;

/*
** Isolate/restore Command Messages. 
*/
typedef SNSR_NoArgsCmd_t    SNSR_IMU_IsolateCmd_t;
typedef SNSR_NoArgsCmd_t    SNSR_IMU_RestoreCmd_t;
typedef SNSR_NoArgsCmd_t    SNSR_MMT_IsolateCmd_t;
typedef SNSR_NoArgsCmd_t    SNSR_MMT_RestoreCmd_t;

typedef SNSR_U8Cmd_t        SNSR_FSS_IsolateCmd_t;
typedef SNSR_U8Cmd_t        SNSR_FSS_RestoreCmd_t;
typedef SNSR_U8Cmd_t        SNSR_CSS_IsolateCmd_t;
typedef SNSR_U8Cmd_t        SNSR_CSS_RestoreCmd_t;

typedef SNSR_NoArgsCmd_t    SNSR_STT_IsolateCmd_t;
typedef SNSR_NoArgsCmd_t    SNSR_STT_RestoreCmd_t;

typedef struct {
    CFE_MSG_TelemetryHeader TlmHeader;
} __attribute__((packed)) SNSR_SunDetectionMsg_t;


/*************************************************************************/
/*
** STX system
*/
/*************************************************************************/

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; /**< \brief Command header */
} STX_NoArgsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; /**< \brief Command header */
    uint8_t                   Arg;
} STX_U8Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; /**< \brief Command header */
    int16_t                   Length;
    uint16_t                  BufPushDelay;
    uint8_t                   Data[16];
} STX_TransmitDataCmd_t;

typedef STX_NoArgsCmd_t STX_NoopCmd_t;
typedef STX_NoArgsCmd_t STX_ResetCountersCmd_t;
typedef STX_NoArgsCmd_t STX_ProcessCmd_t;
typedef STX_NoArgsCmd_t STX_ResetCmd_t;

typedef STX_U8Cmd_t     STX_SetControlModeCmd_t;
typedef STX_U8Cmd_t     STX_SetEncoderRateCmd_t;
typedef STX_U8Cmd_t     STX_SetPaPowerCmd_t;
typedef STX_U8Cmd_t     STX_SetSynthOffsetCmd_t;

typedef STX_NoArgsCmd_t STX_TransmitReadyCmd_t;
typedef STX_NoArgsCmd_t STX_TransmitEndCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint32_t Offset;
    uint32_t Length;
    uint16_t BufPushDelay;
    char Path[32];
} STX_TransmitFileCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint32_t Offset;
    uint32_t Length;
    uint16_t BufPushDelay;
    char Path[64];
} STX_TransmitFileLongPathCmd_t;

/*************************************************************************/
/*
** TO system
*/
/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
    CFE_MSG_CommandHeader CmdHeader; /**< \brief Command header */
} TO_NoArgsCmd_t;

/*
** The following commands all share the "NoArgs" format
**
** They are each given their own type name matching the command name, which
** allows them to change independently in the future without changing the prototype
** of the handler function
*/
typedef TO_NoArgsCmd_t TO_NoopCmd_t;
typedef TO_NoArgsCmd_t TO_ResetCountersCmd_t;
typedef TO_NoArgsCmd_t TO_ProcessCmd_t;
typedef TO_NoArgsCmd_t TO_EnableBeaconCmd_t;

/*************************************************************************/
/*
** Type definition (TO App housekeeping)
*/

typedef struct {
    uint16_t Target;
    uint16_t FileStatus;
    uint32_t NumFiles;
    uint32_t Offset;
    uint32_t Frequency;
    void* Conn;
} TO_DownlinkQueryReplyCmd_Payload_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    TO_DownlinkQueryReplyCmd_Payload_t Payload;
} TO_DownlinkQueryReplyCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    int32_t timeoutsmin;
} TO_DisableBeaconCmd_t;

/*************************************************************************/
/*
** UTRX system
*/
/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
} UTRX_NoArgsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; 
    uint8_t                   Arg;
} UTRX_u8Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; 
    uint32_t                  Arg;
} UTRX_u32Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; 
    float                   Arg;
} UTRX_fCmd_t;


typedef UTRX_NoArgsCmd_t    UTRX_NoopCmd_t;
typedef UTRX_NoArgsCmd_t    UTRX_ResetCountersCmd_t;
typedef UTRX_NoArgsCmd_t    UTRX_RebootCmd_t;

typedef UTRX_u32Cmd_t       UTRX_SetTxFreqCmd_t;
typedef UTRX_u32Cmd_t       UTRX_SetTxBaudCmd_t;

typedef UTRX_fCmd_t         UTRX_SetTxModIndexCmd_t;

typedef UTRX_u8Cmd_t        UTRX_SetTxModeCmd_t;

typedef UTRX_u32Cmd_t       UTRX_SetRxFreqCmd_t;
typedef UTRX_u32Cmd_t       UTRX_SetRxBaudCmd_t;

typedef UTRX_fCmd_t         UTRX_SetRxModIndexCmd_t;

typedef UTRX_u8Cmd_t        UTRX_SetRxModeCmd_t;

typedef UTRX_u32Cmd_t       UTRX_SetRxBandwidthCmd_t;

/*************************************************************************/
/*
** TS system
*/
/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
    CFE_MSG_CommandHeader CmdHeader; /**< \brief Command header */
} TS_NoArgsCmd_t;
typedef TS_NoArgsCmd_t TS_NoopCmd_t;
typedef TS_NoArgsCmd_t TS_ResetCountersCmd_t;
typedef TS_NoArgsCmd_t TS_ProcessCmd_t;

typedef TS_NoArgsCmd_t TS_ClearAllScheduleCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint32_t ExecutionTime;
    uint32_t ExecutionWindow;
    uint16_t EntryId;
    uint16_t EntryGroup;
    CFE_MSG_Message_t ExecutionMsg;
} TS_InsertScheduleEntryCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint16_t EntryId;
} TS_ClearScheduleEntryCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint16_t EntryGroup;
} TS_ClearScheduleGroupCmd_t;

/*************************************************************************/
/*
** ECM system
*/
/*************************************************************************/

/*
** Type definition (generic "no arguments" command)
*/
typedef struct
{
    CFE_MSG_CommandHeader CmdHeader;
} ECM_NoArgsCmd_t;

typedef ECM_NoArgsCmd_t ECM_GetHKAvgCmd_t;
typedef ECM_NoArgsCmd_t ECM_GetSystemStatusCmd_t;
typedef ECM_NoArgsCmd_t ECM_GetOcfStateCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
    uint8_t txlen;
    uint8_t rxlen;
    uint8_t cc;
    uint8_t data[5];
} ECM_Read_t;

/*************************************************************************/
/*
** FTP File Parser
*/
/*************************************************************************/

/*
** FTP File Header
*/
#define CFE_FS_HDR_DESC_MAX_LEN         32
#define DS_TOTAL_FNAME_BUFSIZE          64


typedef struct CFE_FS_Header
{
    uint32_t ContentType;   /**< \brief Identifies the content type (='cFE1'=0x63464531)*/
    uint32_t SubType;       /**< \brief Type of \c ContentType, if necessary */
                          /**< Standard SubType definitions can be found
                               \link #CFE_FS_SubType_ES_ERLOG here \endlink */
    uint32_t Length;        /**< \brief Length of this header to support external processing */
    uint32_t SpacecraftID;  /**< \brief Spacecraft that generated the file */
    uint32_t ProcessorID;   /**< \brief Processor that generated the file */
    uint32_t ApplicationID; /**< \brief Application that generated the file */

    uint32_t TimeSeconds;    /**< \brief File creation timestamp (seconds) */
    uint32_t TimeSubSeconds; /**< \brief File creation timestamp (sub-seconds) */

    char Description[CFE_FS_HDR_DESC_MAX_LEN]; /**< \brief File description */

} CFE_FS_Header_t;


typedef struct
{
    uint32_t  CloseSeconds;                               /**< \brief Time when file was closed */
    uint32_t  CloseSubsecs;        

    uint16_t  FileTableIndex;                             /**< \brief Destination file table index */
    uint16_t  FileNameType;                               /**< \brief Filename type - count vs time */

    char    FileName[DS_TOTAL_FNAME_BUFSIZE];           /**< \brief On-board filename */

} DS_FileHeader_t;

/*
** Ctrlo Type Definitions
*/
typedef struct {
    float AttitudeSolution[7];
    float AngularVelocityError[3];
    float AngleError;
} ADCS_OutputTlm_Attitude_t;

typedef struct {
    float BestPosition[3];
    float BestVelocity[3];
    uint8_t PositionStdStatus;
    uint8_t VelocityStdStatus;
    uint8_t Spare[2];   /* 32-bit Alignment. */
} ADCS_OutputTlm_Orbit_t;

typedef struct {
    uint16_t IntegratorErrFlag;
    uint8_t ControlMode;
    uint8_t ControlModeUpdated;
    uint8_t AcuatorFlagUpdated;
    uint8_t FlagDetumbling;
    uint8_t FlagForcedSpinning;
    uint8_t FlagSensors;
    uint8_t FlagCss;
    uint8_t FlagDumping[3];
} ADCS_OutputTlm_ControlFlags_t;

typedef struct {
    ADCS_OutputTlm_Attitude_t       Attitude;
    ADCS_OutputTlm_Orbit_t          Orbit;
    ADCS_OutputTlm_ControlFlags_t   Flags;
} ADCS_OutputTlm_Payload_t;

typedef struct {
    ccsds_tlm_header_t tlmHeader;
    ADCS_OutputTlm_Payload_t ctrlo;
} ctrlo_packet_t;

/*
** GPS_raw Type Definitions
*/
#define MAX_SAT_NUM             8


typedef struct {
    /* Reference week number in u32 to be aligned properly. */
    uint32_t          Week;
    int32_t           ms;
} GPS_HeaderReferenceTime;


/* RANGEGPSL1 log measurements. */
typedef struct {
    double                  Pseudorange;
    double                  CarrierPhase;
    float                   PseudorangeStd;
    float                   CarrierPhaseStd;
    float                   Doppler;
    float                   CNR;
    float                   Locktime;
    uint16_t                PRN;
    uint16_t                TrackingStatus;
} GPS_MeasurementsAux_RangeGPSL1_Comp_t;

typedef struct {
    GPS_HeaderReferenceTime Time;
    GPS_MeasurementsAux_RangeGPSL1_Comp_t Comp[MAX_SAT_NUM];
    uint8_t                 NumSat;
} GPS_MeasurementsAux_RangeGPSL1_t;


/* SATXYZ2 log measurements. */
typedef struct {
    double                  SatX;
    double                  SatY;
    double                  SatZ;
    double                  ClockCorrection;
    double                  IonosphereDelay;
    double                  TroposphereDelay;
    uint32_t                SatId;
} GPS_MeasurementsAux_SatXYZ2_Comp_t;

typedef struct {
    GPS_HeaderReferenceTime Time;
    GPS_MeasurementsAux_SatXYZ2_Comp_t Comp[MAX_SAT_NUM];
    uint8_t                 NumSat;
} GPS_MeasurementsAux_SatXYZ2_t;


/* Auxilary measurement package. */
typedef struct {
    GPS_MeasurementsAux_RangeGPSL1_t    RangeGPSL1;
    GPS_MeasurementsAux_SatXYZ2_t       SatXYZ2;
} GPS_MeasurementsAux_t_Payload_t;


typedef struct {
    ccsds_tlm_header_t              tlmHeader;
    GPS_MeasurementsAux_t_Payload_t gps;
} gps_packet_t;

/*
** HK Type Definitions
*/
typedef struct __attribute__((packed)) {
    ccsds_tlm_header_t      tlmHeader;
    FM_HkTlm_Payload_t      fm;
    EPS_HkTlm_Payload_t     eps;
    uint8_t padding[6];
    RWA_HkTlm_Payload_t     rwa;
    MTQ_HkTlm_Payload_t     mtq;
    SNSR_HkTlm_Payload_t    snsr;
    UTRX_HkTlm_Payload_t    utrx;
    STX_HkTlm_Payload_t     stx;
    //PAY_HkTlm_Payload_t     pay;
    //TS_HkTlm_Payload_t      ts;
    //uint8_t padding2[16];
} hk_packet_t;

/*
** SNSR_low Type Definitions
*/
typedef struct {
    uint32_t Ticks;
    uint32_t TS;
    float CalibratedQuaternion_Qw;
    float CalibratedQuaternion_Qx;
    float CalibratedQuaternion_Qy;
    float CalibratedQuaternion_Qz;
    float TRACK_Confidence;
    float TRACK_Qw;
    float TRACK_Qx;
    float TRACK_Qy;
    float TRACK_Qz;
    float LISA_Qw;
    float LISA_Qx;
    float LISA_Qy;
    float LISA_Qz;
    float LISA_Percentageclose;
    uint32_t StableCount;
    uint8_t IsTrustworthy;
    uint8_t SolutionStrategy;
    uint8_t Padding[2];
} SNSR_STT_Measurements_t;

typedef struct {
    float AngleX[2];
    float AngleY[2];
    float SunDetection[2];
    uint8_t ErrorCode[2];
    uint8_t Padding[2];
} SNSR_FSS_Measurements_t;

typedef struct {
    float RawVoltageOut[5];
} SNSR_CSS_Measurements_t;

typedef struct {
    float Temperature[5];
} SNSR_TMP_Measurements_t;

typedef struct {
    float IMU_AngularVelocity[3];
    float IMU_Temperature;
} SNSR_IMU_Measurements_t;

typedef struct {
    float MMT_FieldOutputs[3];
} SNSR_MMT_Measurements_t;

typedef struct {
    SNSR_STT_Measurements_t STT;
    SNSR_FSS_Measurements_t FSS;
    SNSR_IMU_Measurements_t IMU;
    SNSR_MMT_Measurements_t MMT;
    SNSR_CSS_Measurements_t CSS;
    SNSR_TMP_Measurements_t TMP;
    bool FaultySTT;
    bool FaultyIMU;
    bool FaultyMMT;
    bool FaultyFSS[2];
} SNSR_Meas_Payload_t;

typedef struct {
    ccsds_tlm_header_t  tlmHeader;
    SNSR_Meas_Payload_t snsr;
} snsr_packet_t;

#define CFE_SB_TLM_HDR_SIZE             12
#define CCSDS_TIME_SIZE                 6

#define OS_PACK         __attribute__ ((packed))
#define NATURALLY_ALIGNED
typedef int8_t                                int8;
typedef int16_t                               int16;
typedef int32_t                               int32;
typedef int64_t                               int64;
typedef uint8_t                               uint8;
typedef uint16_t                              uint16;
typedef uint32_t                              uint32;
typedef uint64_t                              uint64;



typedef struct {

   uint8  Time[CCSDS_TIME_SIZE];

} CCSDS_TlmSecHdr_t;

typedef struct {
    CCSDS_PrimaryHeader_t pri;
    CCSDS_TlmSecHdr_t sec;
} OS_PACK CCSDS_hdr_t;

typedef struct {
    uint8 RetCodeType;
    int32 RetCode;
    uint16 MsgId;
    uint16 CommandCode;
    uint16 DataSize;
    uint8 UsedState;
} OS_PACK HYVRID_CmdExecutionReportMsg_t;

typedef struct {
    union {
        uint8 TlmHeader[CFE_SB_TLM_HDR_SIZE];
        CCSDS_hdr_t Tlmhdr;
    };
    HYVRID_CmdExecutionReportMsg_t Report;
} OS_PACK HYVRID_TelemetryHeader_t;

/*----------------------------------------*/
/*                   EPS                  */
/*----------------------------------------*/
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
} OS_PACK HYVRID_NoArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8 arg;
} OS_PACK HYVRID_U8ArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint16 arg;
} OS_PACK HYVRID_U16ArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint32 arg;
} OS_PACK HYVRID_U32ArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint64 arg;
} OS_PACK HYVRID_U64ArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    int8 arg;
} OS_PACK HYVRID_S8ArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    int16 arg;
} OS_PACK HYVRID_S16ARgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    int32 arg;
} OS_PACK HYVRID_S32ArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    int64 arg;
} OS_PACK HYVRID_S64ArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    float arg;
} OS_PACK HYVRID_FloatArgsCmd_t;

typedef struct {
   uint8_t    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint32_t   Timeout;
   uint8_t    Channel;
   bool     Status;
}__attribute__((packed)) u32u8bool_t;

typedef struct {
   uint8_t    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint32_t   Timeout;
   uint8_t    Channel;
}__attribute__((packed)) u32u8_t;

typedef struct {
   uint8_t   CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint32_t   Size;
   uint32_t  Timeout;
   uint8_t    Node;
   uint8_t    TableId;
}__attribute__((packed)) u32u32u8u8_t;

typedef struct {
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint32   Timeout;
   uint8    Node;
   uint8    TableId;
   uint8    Destination;
}__attribute__((packed)) u32u8u8u8_t;

typedef struct {
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint32   Timeout;
   uint32   Size;
   uint8    Node;
   uint8    TableId;
   uint16   Address;
   uint8    Type;
}__attribute__((packed)) u32u32u8u8u16u8_t;

typedef struct {
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint32   Timeout;
   uint32   ItemSize;
   uint32   ItemCount;
   uint8    Node;
   uint8    TableId;
   uint16   Address;
   uint8    Type;
} OS_PACK u32u32u32u8u8u16u8_t;

typedef struct {
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint32   Timeout;
   bool     ChannelStatus[13];
} OS_PACK u32bool13_t;

typedef struct {
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint32   Timeout;
   bool     ChannelStatus[9];
} OS_PACK u32bool9_t;

typedef struct {
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint32   Timeout;
   uint32   Size;
   uint8    Node;
   uint8    TableId;
   uint16   Address;
   uint8    Type;
   uint8    Value[128];
} OS_PACK EPS_P60_ParamSetCmd_t;

/*----------------------------------------*/
/*                   GRX                  */
/*----------------------------------------*/
#define GRX_CMD_OEM_HANDLER_NAME_LEN            16

typedef struct {
    int16 mid;
    int16 bodylength;
    uint8 body[150];
} OS_PACK GRX_AssemblePublish_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_AssemblePublish_t param;
} OS_PACK GRX_AssemblePublishCmd_t;

typedef struct {
    uint16 msgId;
    uint8  type;
    uint8  padding[1];
    uint32 port;
    uint32 trigger;
    uint32 hold;
    double period;
    double offset;
} NATURALLY_ALIGNED GRX_CmdLOG_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_CmdLOG_t param;
} NATURALLY_ALIGNED GRX_CmdLOGCmd_t;

typedef struct {
    uint16 msgId;
    uint8  padding[2];
    uint32 port;
    double period;
    double offset;
} NATURALLY_ALIGNED GRX_CmdLogOnTime_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_CmdLogOnTime_t param;
} NATURALLY_ALIGNED GRX_CmdLogOnTimeCmd_t;

typedef struct {
    uint16 msgId;
    uint8  padding[2];
    uint32 port;
} NATURALLY_ALIGNED GRX_CmdLogOnChanged_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_CmdLogOnChanged_t param;
} NATURALLY_ALIGNED GRX_CmdLogOnChangedCmd_t;

typedef struct {
    uint16 msgId;
    uint8  padding[2];
    uint32 port;
} NATURALLY_ALIGNED GRX_CmdLogOnNew_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_CmdLogOnNew_t param;
} NATURALLY_ALIGNED GRX_CmdLogOnNewCmd_t;

typedef struct {
    uint16 msgId;
    uint8 msgType;
    uint8 padding[1];
    uint32 port;
} NATURALLY_ALIGNED GRX_CmdUnlog_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_CmdUnlog_t param;
} NATURALLY_ALIGNED GRX_CmdUnlogCmd_t;

typedef struct {
    uint32 port;
    bool held;
} OS_PACK GRX_CmdUnlogAll_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_CmdUnlogAll_t param;
} OS_PACK GRX_CmdUnlogAllCmd_t;

typedef struct {
    uint32 constellation;
    float cutoff;
} NATURALLY_ALIGNED GRX_CmdElevationCutoff_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_CmdElevationCutoff_t param;
} NATURALLY_ALIGNED GRX_CmdElevationCutoffCmd_t;

typedef struct {
    uint32 port;
    uint32 rxType;
    uint32 txType;
    uint32 responses;
} NATURALLY_ALIGNED GRX_CmdInterfaceMode_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_CmdInterfaceMode_t param;
} NATURALLY_ALIGNED GRX_CmdInterfaceModeCmd_t;

typedef struct {
    uint32 port;
    uint32 baud;
    uint32 parity;
    uint32 databits;
    uint32 stopbits;
    uint32 handshake;
    uint32 _break;
} NATURALLY_ALIGNED GRX_CmdSerialConfig_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_CmdSerialConfig_t param;
} NATURALLY_ALIGNED GRX_CmdSerialConfigCmd_t;

typedef struct {
    char name[GRX_CMD_OEM_HANDLER_NAME_LEN];
    uint16 id;
    uint16 mlen;
} NATURALLY_ALIGNED GRX_LogRegisterHandler_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_LogRegisterHandler_t param;
} NATURALLY_ALIGNED GRX_LogRegisterHandlerCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint16 id;
} NATURALLY_ALIGNED GRX_LogUnregisterHandlerCmd_t;

typedef struct {
    uint16 id;
    int32 options;
    char libpath[64];
    char funcName[32];
} OS_PACK GRX_LogAddCallback_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_LogAddCallback_t param;
} OS_PACK GRX_LogAddCallbackCmd_t;

typedef struct {
    uint16 id;
    uint8_t status;
    bool override;
} NATURALLY_ALIGNED GRX_LogSetHandlerStatus_t;
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    GRX_LogSetHandlerStatus_t param;
} NATURALLY_ALIGNED GRX_LogSetHandlerStatusCmd_t;

/*----------------------------------------*/
/*                  ADCS                  */
/*----------------------------------------*/
// ID 2
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint32 unixTimeSeconds;        /**< Current Unix time s  (measurment unit is [s]) */
    uint32 unixTimeNanoSeconds;    /**< Current Unix time ns  (measurment unit is [ns]. valid range is between 0 ns and 999999999 ns) */
} OS_PACK ADCS_CurrentUnixTimeCmd_t;

// ID 48
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    float cmdTargetLatitude;     /**< Target latitude command  (measurment unit is [deg]. valid range is between -90 deg and 90 deg) */
    float cmdTargetLongitude;    /**< Target longitude command  (measurment unit is [deg]. valid range is between -180 deg and 180 deg) */
    float cmdTargetAltitude;     /**< Target altitude command  (measurment unit is [km]. valid range is between -1000 km and 1000000 km) */
} OS_PACK ADCS_Reference_LLHTargetCommandCmd_t;

//ID 49
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint32 gnssTimeSeconds;    /**< GNSS Unix time integer seconds  (measurment unit is [s]) */
    uint32 gnssTimeNs;         /**< GNSS Unix time fraction nanoseconds  (measurment unit is [ns]) */
    int32 gnssSatPosX;        /**< Satellite position vector X component (GNSS frame)  (measurment unit is [cm]) */
    int32 gnssSatPosY;        /**< Satellite position vector Y component (GNSS frame)  (measurment unit is [cm]) */
    int32 gnssSatPosZ;        /**< Satellite position vector Z component (GNSS frame)  (measurment unit is [cm]) */
    int32 gnssSatVelX;        /**< Satellite velocity vector X component (GNSS frame)  (measurment unit is [cm/s]) */
    int32 gnssSatVelY;        /**< Satellite velocity vector Y component (GNSS frame)  (measurment unit is [cm/s]) */
    int32 gnssSatVelZ;        /**< Satellite velocity vector Z component (GNSS frame)  (measurment unit is [cm/s]) */
    uint8 syncTime;       /**< Flag to indicate if RTC should sync with unix time  */
} OS_PACK ADCS_GnssMeasurementsCmd_t;


// ID 54
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    float cmdRpyRoll;     /**< RPY Roll command  (measurment unit is [deg]. valid range is between -180 deg and 180 deg) */
    float cmdRpyPitch;    /**< RPY Pitch command  (measurment unit is [deg]. valid range is between -180 deg and 180 deg) */
    float cmdRpyYaw;      /**< RPY Yaw command  (measurment unit is [deg]. valid range is between -180 deg and 180 deg) */
} OS_PACK ADCS_ReferenceRPYValueCmd_t;

// ID 55
typedef struct
{
    uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
    int16 mtq0OnTimeCmd;    /**< MTQ0 open-loop on-time command  (measurment unit is [ms]. valid range is between -1000 ms and 1000 ms) */
    int16 mtq1OnTimeCmd;    /**< MTQ1 open-loop on-time command  (measurment unit is [ms]. valid range is between -1000 ms and 1000 ms) */
    int16 mtq2OnTimeCmd;    /**< MTQ2 open-loop on-time command  (measurment unit is [ms]. valid range is between -1000 ms and 1000 ms) */
} OS_PACK ADCS_OpenLoopCommandMtqCmd_t;

// ID 58
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8 controlMode;
    uint16 magConTimeout;
} OS_PACK ADCS_ControlModeCmd_t;

//ID 61
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    float ixx;                 /**< Moment of inertia Ixx  (measurment unit is [kg.m^2]. valid range is between 0 kg.m^2 and 100 kg.m^2) */
    float iyy;                 /**< Moment of inertia Iyy  (measurment unit is [kg.m^2]. valid range is between 0 kg.m^2 and 100 kg.m^2) */
    float izz;                 /**< Moment of inertia Izz  (measurment unit is [kg.m^2]. valid range is between 0 kg.m^2 and 100 kg.m^2) */
    float ixy;                 /**< Product of inertia Ixy  (measurment unit is [kg.m^2]. valid range is between -10 kg.m^2 and 10 kg.m^2) */
    float ixz;                 /**< Product of inertia Ixz  (measurment unit is [kg.m^2]. valid range is between -10 kg.m^2 and 10 kg.m^2) */
    float iyz;                 /**< Product of inertia Iyz  (measurment unit is [kg.m^2]. valid range is between -10 kg.m^2 and 10 kg.m^2) */
    double sunPointBodyVecX;    /**< Sun-pointing body vector X component  */
    double sunPointBodyVecY;    /**< Sun-pointing body vector Y component  */
    double sunPointBodyVecZ;    /**< Sun-pointing body vector Z component  */
    double tgtTrackBodyVecX;    /**< Target-tracking body vector X component  */
    double tgtTrackBodyVecY;    /**< Target-tracking body vector Y component  */
    double tgtTrackBodyVecZ;    /**< Target-tracking body vector Z component  */
} OS_PACK ADCS_ConfigAdcsSatelliteCmd_t;


// ID 62
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8 conModeDefault;    /**< Default control mode  */
    float kd;                                                           /**< Detumbling damping gain (Kd)  (valid range is between 0  and 10000 ) */
    float kdsun;                                                        /**< Sun-spin control gain (KDsun)  (valid range is between -1000  and 1000 ) */
    float kdecl;                                                        /**< Sun-spin control gain (KDecl)  (valid range is between -1000  and 1000 ) */
    float ks;                                                           /**< Detumbling spin gain (Ks)  (valid range is between 0  and 10000 ) */
    float kdf;                                                          /**< Fast B-dot detumbling gain (Kdf)  (valid range is between 0  and 100 ) */
    float kn;                                                           /**< Y-momentum nutation damping gain (Kn)  (valid range is between 0  and 10000 ) */
    float kq;                                                           /**< Y-momentum nutation damping quaternion gain (Kq)  (valid range is between 0  and 10000 ) */
    float kqx;                                                          /**< X-axis GG nutation damping quaternion gain (Kqx)  (valid range is between 0  and 10000 ) */
    float kqy;                                                          /**< Y-axis GG nutation damping quaternion gain (Kqy)  (valid range is between 0  and 10000 ) */
    float kqz;                                                          /**< Z-axis GG nutation damping quaternion gain (Kqz)  (valid range is between 0  and 10000 ) */
    float kh;                                                           /**< Wheel momentum dumping magnetic control gain (Kh)  (valid range is between 0  and 10000 ) */
    float kp1;                                                          /**< Y-momentum proportional gain (Kp1)  (valid range is between 0  and 100 ) */
    float kd1;                                                          /**< Y-momentum derivative gain (Kd1)  (valid range is between 0  and 100 ) */
    float kp2;                                                          /**< RWheel proportional gain (Kp2)  (valid range is between 0  and 100 ) */
    float kd2;                                                          /**< RWheel derivative gain (Kd2)  (valid range is between 0  and 100 ) */
    float kp3;                                                          /**< Tracking proportional gain (Kp3)  (valid range is between 0  and 100 ) */
    float kd3;                                                          /**< Tracking derivative gain (Kd3)  (valid range is between 0  and 100 ) */
    float ki3;                                                          /**< Tracking integral gain (Ki3)  (valid range is between 0  and 100 ) */
    float wy_ref;                                                       /**< Reference spin rate (wy-ref)  (measurment unit is [degps]. valid range is between -5 degps and 5 degps) */
    float h_ref;                                                        /**< Reference wheel momentum (H-ref). Must always be smaller than 0  (measurment unit is [Nms]. valid range is between -10 Nms and 0 Nms) */
    float hy_bias;                                                      /**< Y-wheel bias momentum during XYZ-control (Hy-bias)  (measurment unit is [Nms]. valid range is between -10 Nms and 10 Nms) */
    float wSunYawRef;                                                   /**< Reference spin rate for ConSunYawSpin RW control  (measurment unit is [degps]. valid range is between -20 degps and 20 degps) */
    float sunKeepoutAng;                                                /**< Sun keep-out angle  (measurment unit is [deg]. valid range is between 0 deg and 90 deg) */
    float rollLimit;                                                    /**< Limit roll angle in ConRollSun and ConRollTarget  (measurment unit is [deg]. valid range is between 0 deg and 180 deg) */
    bool yawCompensate;                                            /**< Perform yaw compensation for earth rotation in 3-axis RPY control  */
    bool sunTrackEclEn;                                            /**< Enable sun tracking during eclipse when using ConSunTrack  */
    bool sunAvoidEn;                                               /**< Enable sun avoidance  */
} OS_PACK ADCS_ControllerConfigurationCmd_t;

// ID 63
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    float offset1;         /**< Magnetometer channel 1 offset  */
    float offset2;         /**< Magnetometer channel 2 offset  */
    float offset3;         /**< Magnetometer channel 3 offset  */
    float sensMatrix11;    /**< Magnetometer sensitivity matrix S11  */
    float sensMatrix22;    /**< Magnetometer sensitivity matrix S22  */
    float sensMatrix33;    /**< Magnetometer sensitivity matrix S33  */
    float sensMatrix12;    /**< Magnetometer sensitivity matrix S12  */
    float sensMatrix13;    /**< Magnetometer sensitivity matrix S13  */
    float sensMatrix21;    /**< Magnetometer sensitivity matrix S21  */
    float sensMatrix23;    /**< Magnetometer sensitivity matrix S23  */
    float sensMatrix31;    /**< Magnetometer sensitivity matrix S31  */
    float sensMatrix32;    /**< Magnetometer sensitivity matrix int32  */
} OS_PACK ADCS_ConfigMag0OrbitCalCmd_t;

// ID 65
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8 mountStackX;          /**< StackX mounting  */
    uint8 mountStackY;          /**< StackY mounting  */
    uint8 mountStackZ;          /**< StackZ mounting  */
    uint8 mountMtq0;            /**< MTQ0 mounting  */
    uint8 mountMtq1;            /**< MTQ1 mounting  */
    uint8 mountMtq2;            /**< MTQ2 mounting  */
    uint8 mountRwl0;            /**< Wheel0 mounting  */
    uint8 mountRwl1;            /**< Wheel1 mounting  */
    uint8 mountRwl2;            /**< Wheel2 mounting  */
    uint8 mountCss0;            /**< CSS0 mounting  */
    uint8 mountCss1;            /**< CSS1 mounting  */
    uint8 mountCss2;            /**< CSS2 mounting  */
    uint8 mountCss3;            /**< CSS3 mounting  */
    uint8 mountCss4;            /**< CSS4 mounting  */
    uint8 mountCss5;            /**< CSS5 mounting  */
    uint8 mountCss6;            /**< CSS6 mounting  */
    uint8 mountCss7;            /**< CSS7 mounting  */
    uint8 mountCss8;            /**< CSS8 mounting  */
    uint8 mountCss9;            /**< CSS9 mounting  */
    double mountFss0Alpha;                                               /**< FSS0 mounting alpha angle  (measurment unit is [deg]) */
    double mountFss0Beta;                                                /**< FSS0 mounting beta angle  (measurment unit is [deg]) */
    double mountFss0Gamma;                                               /**< FSS0 mounting gamma angle  (measurment unit is [deg]) */
    double mountHss0Alpha;                                               /**< HSS0 mounting alpha angle  (measurment unit is [deg]) */
    double mountHss0Beta;                                                /**< HSS0 mounting beta angle  (measurment unit is [deg]) */
    double mountHss0Gamma;                                               /**< HSS0 mounting gamma angle  (measurment unit is [deg]) */
    double mountMag0Alpha;                                               /**< MAG0 mounting alpha angle  (measurment unit is [deg]) */
    double mountMag0Beta;                                                /**< MAG0 mounting beta angle  (measurment unit is [deg]) */
    double mountMag0Gamma;                                               /**< MAG0 mounting gamma angle  (measurment unit is [deg]) */
} OS_PACK ADCS_MountingConfigurationCmd_t;

//ID 67
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8 estModeMainDefault;      /**< Default main estimator mode  */
    uint8 estModeBackupDefault;    /**< Default backup estimator mode  */
    float magR;                                                               /**< Magnetometer measurement noise  (valid range is between 0  and 1 ) */
    float cssR;                                                               /**< Coarse sun sensor measurement noise  (valid range is between 0  and 10 ) */
    float fssR;                                                               /**< Fine sun sensor measurement noise  (valid range is between 0  and 1 ) */
    float hssR;                                                               /**< Horizon sensor measurement noise  (valid range is between 0  and 1 ) */
    float strR;                                                               /**< Star tracker measurement noise  (valid range is between 0  and 0.1 ) */
    float rkfQ;                                                               /**< Magnetometer RKF system noise  (valid range is between 0  and 0.01 ) */
    float ekfQ;                                                               /**< EKF system noise  (valid range is between 0  and 0.01 ) */
    float nutDeps;                                                            /**< Polar nutation Epsilon correction  (measurment unit is [rad]. valid range is between -0.1 rad and 0.1 rad) */
    float nutDpsi;                                                            /**< Polar nutation Psi correction  (measurment unit is [rad]. valid range is between -0.1 rad and 0.1 rad) */
    bool ekfUseFss;                                                      /**< Use fine sun sensor measurements in EKF  */
    bool ekfUseCss;                                                      /**< Use coarse sun sensor measurements in EKF  */
    bool ekfUseHss;                                                      /**< Use horizon sensor measurements in EKF  */
    bool ekfUseStr;                                                      /**< Use star tracker measurements in EKF  */
    uint8 tVec1;                       /**< Vector 1 selection for Triad  */
    uint8 tVec2;                       /**< Vector 2 selection for Triad  */
} OS_PACK ADCS_EstimatorConfigurationCmd_t;

// ID 68
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    double orbitEpoch;    /**< Orbit epoch  (measurment unit is [yyddd.ssssssss]. valid range is between 0 yyddd.ssssssss and 100000 yyddd.ssssssss) */
    double orbitIncl;     /**< Orbit inclination  (measurment unit is [deg]. valid range is between 0 deg and 180 deg) */
    double orbitRaan;     /**< Orbit RAAN  (measurment unit is [deg]. valid range is between 0 deg and 360 deg) */
    double orbitEccen;    /**< Orbit eccentricity  (valid range is between 0  and 1 ) */
    double orbitAP;       /**< Orbit argument of perigee  (measurment unit is [deg]. valid range is between 0 deg and 360 deg) */
    double orbitMA;       /**< Orbit mean anomaly  (measurment unit is [deg]. valid range is between 0 deg and 360 deg) */
    double orbitMM;       /**< Orbit mean motion  (measurment unit is [orbits/day]. valid range is between 0 orbits/day and 20 orbits/day) */
    double orbitBstar;    /**< Orbit B-star drag term  (valid range is between 0  and 1 ) */
} OS_PACK ADCS_ConfigOrbitSatParamsCmd_t;

// ID 69
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8 selectRwl;     /**< RWL selection flags  */
    uint8 selectMag;     /**< MAG selection flags  */
    uint8 selectFss;     /**< FSS selection flags  */
    uint8 selectHss;     /**< HSS selection flags  */
    uint8 selectGyro;    /**< GYR selection flags  */
    uint8 selectStr;     /**< STR selection flags  */
    uint8 selectGnss;    /**< GNSS selection flags  */
    uint8 selectExt;     /**< External sensor selection flags  */
} OS_PACK ADCS_NodeSelectionCmd_t;

// ID 70
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    float mtq0Mmax;     /**< MTQ0 maximum dipole moment  (measurment unit is [A.m^2]. valid range is between 0 A.m^2 and 400 A.m^2) */
    float mtq1Mmax;     /**< MTQ1 maximum dipole moment  (measurment unit is [A.m^2]. valid range is between 0 A.m^2 and 400 A.m^2) */
    float mtq2Mmax;     /**< MTQ2 maximum dipole moment  (measurment unit is [A.m^2]. valid range is between 0 A.m^2 and 400 A.m^2) */
    uint16 onTimeMax;    /**< Maximum magnetorquer on-time  (measurment unit is [ms]) */
    uint16 onTimeMin;    /**< Minimum magnetorquer on-time  (measurment unit is [ms]) */
    float mtqFfac;      /**< LPF factor for magnetorquer commands. Set to zero for no filtering  (valid range is between 0  and 1 ) */
} OS_PACK ADCS_MtqConfigCmd_t;

//ID 71
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8 estModeMain;      /**< Main estimator mode  */
    uint8 estModeBackup;    /**< Backup estimator mode  */
} OS_PACK ADCS_EstimationModeCmd_t;

// ID 74
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    float rwl0SpeedCmd;    /**< RWL0 open-loop speed command  (valid range is between -10000  and 10000 ) */
    float rwl1SpeedCmd;    /**< RWL1 open-loop speed command  (valid range is between -10000  and 10000 ) */
    float rwl2SpeedCmd;    /**< RWL2 open-loop speed command  (valid range is between -10000  and 10000 ) */
} OS_PACK ADCS_OpenLoopCommandRwlCmd_t;

// ID 76
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    float hx;    /**< X-momentum open-loop speed command  (measurment unit is [Nm]. valid range is between -0.1 Nm and 0.1 Nm) */
    float hy;    /**< Y-momentum open-loop speed command  (measurment unit is [Nm]. valid range is between -0.1 Nm and 0.1 Nm) */
    float hz;    /**< Z-momentum open-loop speed command  (measurment unit is [Nm]. valid range is between -0.1 Nm and 0.1 Nm) */
} OS_PACK ADCS_OpenLoopCommandHxyzRwCmd_t;
/*----------------------------*/
/*---- < End of ADCS > -------*/
/*----------------------------*/

/*----------------------------------------*/
/*                 IFC app                */
/*----------------------------------------*/
typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    char HandleName[16];
} IFC_HandleNoArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    char HandleName[16];
    uint8 Arg;
} IFC_HandleU8ArgsCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    char HandleName[16];
    uint32 Arg;
} IFC_HandleU32ArgsCmd_t;

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    char    HandleName[16];
    uint16  Size;
    uint8   Data[128]; 
} IFC_WriteCmd_t;

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    char    HandleName[16];
    uint16  Size;
    uint16  Timeout;
} IFC_ReadCmd_t;

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    int     GpioNum;
} IFC_GpioNumCmd_t;

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    char    HandleName[16];
    char    DeviceName[16];
    int     OpenOpt;
} IFC_IoOpenCmd_t;

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    int     GpioNum;
    bool    Value;
} IFC_GpioWriteCmd_t;

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    char    HandleName[16];
    uint8   Termios[64];
} IFC_UartSetTermiosCmd_t;

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    char    HandleName[16];
    uint16  TxSize;
    uint16  RxSize;
    uint8   Address;
    uint8   TxData[128];
} IFC_I2cDuplexTransferCmd_t;

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    char    HandleName[16];
    uint16  TxSize;
    uint16  RxSize;
    uint8   TxData[128];
} IFC_SpiDuplexTransferCmd_t;

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    char    HandleName[16];
    char    DevName[32];
    uint8   devType;
    uint8   MutexId;
} IFC_IoHandleAllocateCmd_t;

/*----------------------------------------*/
/*                  PAYS                  */
/*----------------------------------------*/
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint16  NumReads;
    uint16  IntervalMs;
    bool    IgnoreErrors;
    bool    Pack;
    char    FileName[128];
} OS_PACK PAYS_D1064_ReadSaveStatusCmd_t;


/*----------------------------------------*/
/*                  PAYC                  */
/*----------------------------------------*/
//Other Definitions
#define PAYC_STORE_FILENAME_SIZE        32
#define PAYC_FILE_LIST_PATH_SIZE        64
#define PAYC_REMOTE_DIRNAME_SIZE        64
#define PAYC_LOCAL_DIRNAME_SIZE         64
typedef struct {
    uint16_t delay;
    uint32_t snap_flags;
    uint8_t snap_format;
    uint32_t store_flags;
    uint8_t store_format;
} OS_PACK PAYC_Conf_t;

typedef struct{
    uint8_t count;
    uint16_t width;
    uint16_t height;
    uint16_t leftedge;
    uint16_t topedge;
} OS_PACK PAYC_Snap_t;

typedef struct{
    uint8_t format;
    uint8_t scale;
    char filename[PAYC_STORE_FILENAME_SIZE];
} OS_PACK PAYC_Store_t;

typedef struct{
    char StoredFileListLocation[PAYC_FILE_LIST_PATH_SIZE];
} OS_PACK PAYC_StoreFileListLocation_t;

typedef struct{
    char remote_dirname[PAYC_REMOTE_DIRNAME_SIZE];
    char local_dirname[PAYC_LOCAL_DIRNAME_SIZE];
} OS_PACK PAYC_DirPath_t;

typedef struct{
    char filename[PAYC_STORE_FILENAME_SIZE];
} OS_PACK PAYC_DownloadwithFileName_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    PAYC_Conf_t Payload;
}OS_PACK PAYC_SetConfCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    PAYC_Snap_t Payload;
} OS_PACK PAYC_SnapCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    PAYC_Store_t Payload;
} OS_PACK PAYC_StoreCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    PAYC_StoreFileListLocation_t Payload;
} OS_PACK PAYC_StoreFileListLocationCmd_t;

typedef struct {
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    PAYC_DirPath_t Payload;
} OS_PACK PAYC_SetDirPathCmd_t;

typedef struct
{
    uint8 CmdHeader[CFE_SB_CMD_HDR_SIZE];
    PAYC_DownloadwithFileName_t Payload;
} OS_PACK PAYC_DownloadCmd_t;

/*----------------------------------------*/
/*                  STX                   */
/*----------------------------------------*/
typedef struct {
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint8    Reg;
   uint8    Val;
} STX_PULSAR_GenericSetValueCmd_t;

typedef struct {
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   bool     Val;
} STX_PULSAR_SetValueBoolCmd_t;

typedef struct {
   uint8    CmdHeader[CFE_SB_CMD_HDR_SIZE];
   uint8    Reg;
   uint8    Size;
} STX_PULSAR_GenericGetValueCmd_t;


/*----------------------------------------*/
/*                  PAYR                  */
/*----------------------------------------*/
typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8   Chip;
    uint8   Pins;
} PAYR_BurnCmd_t;

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint8   Chip;
    bool    MainBurn;
    uint8_t BurnTimeSeconds;
} PAYR_BurnLoopCmd_t;












// THIS PART IS ADDED FOR COSMIC

/*----------------------------------------*/
/*                  SANT                  */
/*----------------------------------------*/

typedef struct {
    uint8   CmdHeader[CFE_SB_CMD_HDR_SIZE];
    uint16  min_deploy;
    uint8   backup;
    uint8   max_burn_duration;
} SANT_DeployBurnCmd_t;














typedef struct {
    /* PAYR */
    PAYR_BurnCmd_t payrburncmd;
    PAYR_BurnLoopCmd_t payrburnloofcmd;

    /* STX */
    STX_PULSAR_GenericSetValueCmd_t stxgenericsetvalue;
    STX_PULSAR_SetValueBoolCmd_t    stxsetvaluebool;
    STX_PULSAR_GenericGetValueCmd_t stxgenericgetvalue;
    
    /* PAYC*/
    PAYC_SetConfCmd_t paycsetconfcmd;
    PAYC_SnapCmd_t paycsnapcmd;
    PAYC_StoreCmd_t paycstorecmd;
    PAYC_StoreFileListLocationCmd_t paycstorefilelistlocationcmd;
    PAYC_SetDirPathCmd_t paycsetdirpathcmd;
    PAYC_DownloadCmd_t paycdownloadcmd;

    /* PAYS */
    PAYS_D1064_ReadSaveStatusCmd_t paysd1064readsavestatuscmd;

    /* IFC app */
    IFC_HandleNoArgsCmd_t ifchandlenoarg;
    IFC_HandleU8ArgsCmd_t ifchandleu8arg;
    IFC_HandleU32ArgsCmd_t ifchandleu32arg;
    IFC_WriteCmd_t ifcwritecmd;
    IFC_ReadCmd_t ifcreadcmd;
    IFC_GpioNumCmd_t ifcgpionumcmd;
    IFC_IoOpenCmd_t ifcioopencmd;
    IFC_GpioWriteCmd_t ifcgpiowritecmd;
    IFC_UartSetTermiosCmd_t ifcuartsettermioscmd;
    IFC_I2cDuplexTransferCmd_t ifci2cduplextransfercmd;
    IFC_SpiDuplexTransferCmd_t ifcspiduplextransfercmd;
    IFC_IoHandleAllocateCmd_t ifciohandleallocatecmd;

    //eps unusual case
    u32u8bool_t u32u8bool;
    u32u8_t u32u8;
    u32u32u8u8_t u32u32u8u8;
    u32u8u8u8_t u32u8u8u8;
    u32u32u8u8u16u8_t u32u32u8u8u16u8;
    u32u32u32u8u8u16u8_t u32u32u32u8u8u16u8;
    u32bool13_t u32bool13;
    u32bool9_t u32bool9;
    EPS_P60_ParamSetCmd_t epsp60paramsetcmd;

    //grx
    GRX_AssemblePublishCmd_t grxassemblepublishcmd;
    GRX_CmdLOGCmd_t grxcmdlogcmd;
    GRX_CmdLogOnTimeCmd_t grxcmdlogontimecmd;
    GRX_CmdLogOnChangedCmd_t grxcmdlogonchangedcmd;
    GRX_CmdLogOnNewCmd_t grxcmdlogonnewcmd;
    GRX_CmdUnlogCmd_t grxcmdunlogcmd;
    GRX_CmdUnlogAllCmd_t grxcmdunlogallcmd;
    GRX_CmdElevationCutoffCmd_t grxcmdelevationcutoffcmd;
    GRX_CmdInterfaceModeCmd_t grxcmdinterfacemodecmd;
    GRX_CmdSerialConfigCmd_t grxcmdserialconfigcmd;
    GRX_LogRegisterHandlerCmd_t grxlogresisterhandlercmd;
    GRX_LogUnregisterHandlerCmd_t grxlogunresisterhandlercmd;
    GRX_LogAddCallbackCmd_t grxlogaddcallbackcmd;
    GRX_LogSetHandlerStatusCmd_t grxlogsethandlerstatuscmd;

    /* ADCS */
    ADCS_CurrentUnixTimeCmd_t adcs_Unixtime;
    ADCS_Reference_LLHTargetCommandCmd_t adcs_RefLLHTarget;
    ADCS_GnssMeasurementsCmd_t adcs_GnssMeasurements;
    ADCS_ReferenceRPYValueCmd_t adcs_RefRPY;
    ADCS_OpenLoopCommandMtqCmd_t adcs_OpenLoopCmdMtq;
    ADCS_ControlModeCmd_t adcs_ControlMode;
    ADCS_ConfigAdcsSatelliteCmd_t adcs_ConfigAdcsSat;
    ADCS_ControllerConfigurationCmd_t adcs_ControllerConfig;
    ADCS_ConfigMag0OrbitCalCmd_t adcs_ConfigMag0OrbitCal;
    ADCS_MountingConfigurationCmd_t adcs_MountingConfig;
    ADCS_EstimatorConfigurationCmd_t adcs_EstimatorConfig;
    ADCS_ConfigOrbitSatParamsCmd_t adcs_ConfigOrbitSatParams;
    ADCS_NodeSelectionCmd_t adcs_NodeSelection;
    ADCS_MtqConfigCmd_t adcs_MtqConfig;
    ADCS_EstimationModeCmd_t adcs_EstMode;
    ADCS_OpenLoopCommandRwlCmd_t adcs_OpenLoopCmdRwl;
    ADCS_OpenLoopCommandHxyzRwCmd_t adcs_OpenLoopCmdHxyzRw;
    



    // THIS PART IS ADDED FOR COSMIC

    /* SANT*/
    SANT_DeployBurnCmd_t santdeployburncmd;






}__attribute__((packed)) Command;

typedef struct __attribute__ ((packed)) {
    uint8_t redir_out;
    char cmd[199];
} cmd_syscmd_t;

typedef struct __attribute__ ((packed)) {
    char cmd[100];
    char redir_path[100];
} cmd_syscmd_redir_t;

typedef struct __attribute__ ((packed)) {
    char redir_path[200];
} cmd_syscmd_set_redir_path_t;

typedef struct __attribute__ ((packed)) {
    uint8_t opt;
} cmd_ds_cleanup_t;

typedef struct __attribute__ ((packed)) {
    uint8_t type;
    union {
        cmd_syscmd_t        syscmd;
        cmd_syscmd_redir_t  syscmd_redir;
        cmd_syscmd_set_redir_path_t set_redir;
        cmd_ds_cleanup_t    ds_cleanup;
    } required;
} cmd_packet_t;



typedef struct __attribute__ ((packed)) {
    uint8_t type;
    uint8_t result;
    int retcode;
} reply_packet_t;

























// FOR COSMIC & BEE-1000 GS



/****************************************** For BOTH COSMIC & BEE-1000 ******************************************/
/**********************************/
/*                                */
/*   UT(R)X (COSMIC & BEE-1000)   */
/*        GomSpace AX100          */
/*                                */
/**********************************/

// Define MSGID

#define ADCS_HK_TLM_MID         0x0820
#define ADCS_REF_TLM_WHL_MID    0x0821
#define ADCS_REF_TLM_MTQ_MID    0x0822
#define ADCS_LOOP_WAKEUP_MID    0x0823
#define ADCS_OUT_TLM_MID        0x0824



// Define Function Codes

#define UTRX_APP_NOOP_CC                      0  //set
#define UTRX_APP_RESET_COUNTERS_CC            1  //set
#define UTRX_APP_RESET_APP_CMD_COUNTERS_CC    2  //set
#define UTRX_APP_RESET_DEVICE_CMD_COUNTERS_CC 3  //set
#define UTRX_APP_PROCESS_CC                   4 //set
#define UTRX_APP_DISPLAY_PARAM_CC             5 //set
#define UTRX_GNDWDT_CLEAR_CC                  6 //set
#define UTRX_REBOOT_CC                        7 //set
#define UTRX_RXCONF_SET_BAUD_CC               8 //set
#define UTRX_TXCONF_SET_BAUD_CC               9 //set
#define UTRX_SET_DEFAULT_BAUD_CC              10 //set
#define UTRX_RPARAM_SAVE_1_CC                 11 //set
#define UTRX_RPARAM_SAVE_5_CC                 12 //set
#define UTRX_RPARAM_SAVE_ALL_CC               13 //set
#define UTRX_RXCONF_SET_FREQ_CC               14 //set
#define UTRX_TXCONF_SET_FREQ_CC               15 //set
#define UTRX_CHECK_STATE_PING_CC              32 //set


#define UTRX_RXCONF_GET_BAUD_CC           16 //get
#define UTRX_RXCONF_GET_GUARD_CC          17 //get
#define UTRX_TXCONF_GET_BAUD_CC           18 //get
#define UTRX_TLM_GET_TEMP_BRD_CC          19 //get
#define UTRX_TLM_GET_LAST_RSSI_CC         20 //get
#define UTRX_TLM_GET_LAST_RFERR_CC        21 //get
#define UTRX_TLM_GET_ACTIVE_CONF_CC       22 //get
#define UTRX_TLM_GET_BOOT_COUNT_CC        23 //get
#define UTRX_TLM_GET_BOOT_CAUSE_CC        24 //get
#define UTRX_TLM_GET_LAST_CONTACT_CC      25 //get
#define UTRX_TLM_GET_TOT_TX_BYTES_CC      26 //get
#define UTRX_TLM_GET_TOT_RX_BYTES_CC      27 //get
#define UTRX_GET_STATUS_CONFIGURATION_CC  28 //get
#define UTRX_RXCONF_GET_FREQ_CC           29 //get
#define UTRX_TXCONF_GET_FREQ_CC           30 //get

#define UTRX_SEND_HK_CC                   31//hk data



// Define CMD Structure

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
} UTRX_NoArgsCmd_t;


typedef struct {
    CFE_MSG_CommandHeader CmdHeader; 
    uint32_t                  Arg;
} UTRX_u32Cmd_t;



typedef UTRX_NoArgsCmd_t    UTRX_NoopCmd_t;
typedef UTRX_NoArgsCmd_t    UTRX_ResetCountersCmd_t;
typedef UTRX_NoArgsCmd_t    UTRX_ResetAppCmdCountersCmd_t;
typedef UTRX_NoArgsCmd_t    UTRX_ResetDeviceCmdCountersCmd_t;
typedef UTRX_NoArgsCmd_t    UTRX_ProcessCmd_t;
typedef UTRX_NoArgsCmd_t    UTRX_DisplayParamCmd_t;
typedef UTRX_NoArgsCmd_t    UTRX_GndWdtClearCmd_t;
typedef UTRX_NoArgsCmd_t    UTRX_RebootCmd_t;

typedef UTRX_u32Cmd_t       UTRX_SetTxFreqCmd_t;
typedef UTRX_u32Cmd_t       UTRX_SetTxBaudCmd_t;
typedef UTRX_u32Cmd_t       UTRX_SetRxFreqCmd_t;
typedef UTRX_u32Cmd_t       UTRX_SetRxBaudCmd_t;






/****************************************** For COSMIC ******************************************/

/**********************************/
/*                                */
/*         STRX (COSMIC)          */
/*        GomSpace AX2150         */
/*                                */
/**********************************/

// Define MSGID

// Define Function Codes
#define STRX_APP_NOOP_CC                      0  //set
#define STRX_APP_RESET_COUNTERS_CC            1  //set
#define STRX_APP_RESET_APP_CMD_COUNTERS_CC    2  //set
#define STRX_APP_RESET_DEVICE_CMD_COUNTERS_CC 3  //set
// #define STRX_APP_PROCESS_CC                   4 //set
#define STRX_APP_DISPLAY_PARAM_CC             5 //set
#define STRX_GNDWDT_CLEAR_CC                  6 //set
#define STRX_REBOOT_CC                        7 //set
#define STRX_RXCONF_SET_BAUD_CC               8 //set
#define STRX_TXCONF_SET_BAUD_CC               9 //set
#define STRX_SET_DEFAULT_BAUD_CC              10 //set
#define STRX_RPARAM_SAVE_1_CC                 11 //set
#define STRX_RPARAM_SAVE_5_CC                 12 //set
#define STRX_RPARAM_SAVE_ALL_CC               13 //set
#define STRX_RXCONF_SET_FREQ_CC               14 //set
#define STRX_TXCONF_SET_FREQ_CC               15 //set
#define STRX_CHECK_STATE_PING_CC              32 //set


#define STRX_RXCONF_GET_BAUD_CC           16 //get
#define STRX_RXCONF_GET_GUARD_CC          17 //get
#define STRX_TXCONF_GET_BAUD_CC           18 //get
#define STRX_TLM_GET_TEMP_BRD_CC          19 //get
#define STRX_TLM_GET_LAST_RSSI_CC         20 //get
#define STRX_TLM_GET_LAST_RFERR_CC        21 //get
// #define STRX_TLM_GET_ACTIVE_CONF_CC       22 //get
#define STRX_TLM_GET_BOOT_COUNT_CC        23 //get
#define STRX_TLM_GET_BOOT_CAUSE_CC        24 //get
#define STRX_TLM_GET_LAST_CONTACT_CC      25 //get
#define STRX_TLM_GET_TOT_TX_BYTES_CC      26 //get
#define STRX_TLM_GET_TOT_RX_BYTES_CC      27 //get
#define STRX_GET_STATUS_CONFIGURATION_CC  28 //get
#define STRX_RXCONF_GET_FREQ_CC           29 //get
#define STRX_TXCONF_GET_FREQ_CC           30 //get
#define STRX_TLM_RXMODE_CC                32 //get
#define STRX_TLM_GET_GNDWDT_CNT_CC        33 //get
#define STRX_TLM_GET_GNDWDT_LEFT_CC       34 //get

#define STRX_SEND_HK_CC                   31//hk data

// Define CMD Structure




/*******************************************/
/*                                         */
/*              UANT (COSMIC)              */
/*    ISISpace Deployable Antenna System   */
/*                                         */
/*******************************************/

// Define MSGID

// Define Function Codes
#define UANT_APP_NOOP_CC                          1
#define UANT_APP_RESET_COUNTERS_CC                2
#define UANT_APP_RESET_CC                         3
#define UANT_APP_GET_DEPLOYMENT_STATUS_CC         4
#define UANT_APP_ARM_ANTENNA_SYSTEMS_CC           5
#define UANT_APP_DISARM_CC                        6
#define UANT_APP_AUTOMATED_DEPLOYMENT_CC          7
#define UANT_APP_CANCEL_DEPLOYMENT_ACTIVATION_CC  8
#define UANT_APP_DEPLOY_ANT1_CC                   9
#define UANT_APP_DEPLOY_ANT2_CC                   10
#define UANT_APP_DEPLOY_ANT3_CC                   11
#define UANT_APP_DEPLOY_ANT4_CC                   12
#define UANT_APP_DEPLOY_ANT1_OVERRIDE_CC          13
#define UANT_APP_DEPLOY_ANT2_OVERRIDE_CC          14
#define UANT_APP_DEPLOY_ANT3_OVERRIDE_CC          15
#define UANT_APP_DEPLOY_ANT4_OVERRIDE_CC          16
#define UANT_APP_REPORT_ANT_ACTIVATION_CNT_CC     17
#define UANT_APP_REPORT_ANT_ACTIVATION_TIME_CC    18
#define UANT_APP_MEASURE_SYSTEM_TEMPERATURE_CC    19

// Define CMD Structure
typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
} UANT_NoArgsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; 
    uint8_t                   Arg;
} UANT_u8Cmd_t;



typedef UANT_NoArgsCmd_t    UANT_NoopCmd_t;
typedef UANT_NoArgsCmd_t    UANT_ResetCountersCmd_t;
typedef UANT_NoArgsCmd_t    UANT_ResetCmd_t;
typedef UANT_NoArgsCmd_t    UANT_GetDeploymentStatusCmd_t;
typedef UANT_NoArgsCmd_t    UANT_ArmAntennaSystemsCmd_t;
typedef UANT_NoArgsCmd_t    UANT_DisarmCmd_t;
typedef UANT_NoArgsCmd_t    UANT_CancelDeploymentActivationCmd_t;
typedef UANT_NoArgsCmd_t    UANT_MeasureSystemTemperatureCmd_t;



typedef UANT_u8Cmd_t        UANT_AutomatedDeploymentCmd_t;
typedef UANT_u8Cmd_t        UANT_DeployAnt1Cmd_t;
typedef UANT_u8Cmd_t        UANT_DeployAnt2Cmd_t;
typedef UANT_u8Cmd_t        UANT_DeployAnt3Cmd_t;
typedef UANT_u8Cmd_t        UANT_DeployAnt4Cmd_t;
typedef UANT_u8Cmd_t        UANT_DeployAnt1OverrideCmd_t;
typedef UANT_u8Cmd_t        UANT_DeployAnt2OverrideCmd_t;
typedef UANT_u8Cmd_t        UANT_DeployAnt3OverrideCmd_t;
typedef UANT_u8Cmd_t        UANT_DeployAnt4OverrideCmd_t;
typedef UANT_u8Cmd_t        UANT_ReportAntActivationCntCmd_t;
typedef UANT_u8Cmd_t        UANT_ReportAntActivationTimeCmd_t;




/*******************************************/
/*                                         */
/*              SANT (COSMIC)              */
/*        GomSpace NanoCom AM2150-O        */
/*                                         */
/*******************************************/
// Define MSGID

// Define Function Codes
#define SANT_APP_NOOP_CC                0
#define SANT_APP_RESET_COUNTERS_CC      1
#define SANT_APP_SOFT_REBOOT_CC         2
#define SANT_APP_BURN_CC                3
#define SANT_APP_STOP_BURN_CC           4
#define SANT_APP_GET_STATUS_CC          5
#define SANT_APP_GET_BACKUP_STATUS_CC   6
#define SANT_APP_GET_BOARD_STATUS_CC    7
#define SANT_APP_GET_TEMPERATURE_CC     8
#define SANT_APP_GET_SETTINGS_CC        9
#define SANT_APP_SET_SETTINGS_CC        10


// Define CMD Structure

typedef struct {
    CFE_MSG_CommandHeader CmdHeader;
} SANT_NoArgsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; 
    uint8_t                   Arg;
} SANT_u8Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader CmdHeader; 
    uint16_t                 min_deploy;
    uint8_t                  backup;
    uint8_t                  max_burn_duration;
} SANT_SetSettingsCmd_t;


typedef SANT_NoArgsCmd_t        SANT_NoopCmd_t;
typedef SANT_NoArgsCmd_t        SANT_ResetCountersCmd_t;
typedef SANT_NoArgsCmd_t        SANT_SoftRebootCmd_t;
typedef SANT_NoArgsCmd_t        SANT_StopBurnCmd_t;
typedef SANT_NoArgsCmd_t        SANT_GetStatusCmd_t;
typedef SANT_NoArgsCmd_t        SANT_GetBackupStatusCmd_t;
typedef SANT_NoArgsCmd_t        SANT_GetTemperatureCmd_t;
typedef SANT_NoArgsCmd_t        SANT_GetSettingsCmd_t;

typedef SANT_u8Cmd_t            SANT_BurnCmd_t;









/*******************************************/
/*                                         */
/*               EPS (COSMIC)              */
/*        ?????????????????????????        */
/*                                         */
/*******************************************/
// Define MSGID

// Define Function Codes

/*
** EPS App command codes
*/
#define EPS_NOOP_CC                 0
#define EPS_RESET_COUNTERS_CC       1
#define EPS_GET_COUNTERS_CC         2
#define EPS_GET_APPDATA_CC          3

/*
** P31u device & channel control
*/
#define EPS_P31U_SET_OUT_SINGLE_CC  10
#define EPS_P31U_SET_OUTPUTS_CC     11
#define EPS_P31U_RESET_WDT_CC       20
#define EPS_P31U_RESET_COUNTERS_CC  21
#define EPS_P31U_HARD_RESET_CC      22

/*
** P31u housekeeping requests
*/
#define EPS_P31U_GETHK_ALL_CC       30
#define EPS_P31U_GETHK_OUT_CC       31
#define EPS_P31U_GETHK_VI_CC        32
#define EPS_P31U_GETHK_WDT_CC       33
#define EPS_P31U_GETHK_BASIC_CC     34
#define EPS_P31U_GETHK_OLD_CC       35
#define EPS_P31U_GETHK_CC           36

/*
** P31u config commands
*/
#define EPS_P31U_SET_PV_VOLT_CC     40
#define EPS_P31U_SET_PV_AUTO_CC     41
#define EPS_P31U_SET_HEATER_CC      42

#define EPS_P31U_GET_CONFIG_CC      50
#define EPS_P31U_SET_CONFIG_CC      51
#define EPS_P31U_CONFIG_CC          52
#define EPS_P31U_GET_CONFIG2_CC     53
#define EPS_P31U_SET_CONFIG2_CC     54
#define EPS_P31U_CONFIG2_CC         55
#define EPS_P31U_SET_CONFIG3_CC     56

/*
** Generic transaction (plumbing)
*/
#define EPS_P31U_TRANSACTION_CC     99


// Define CMD Structure
typedef struct {
    uint8_t  channel;
    uint8_t  value;
    uint16_t delay;
} EPS_P31U_SetOutputSingle_Payload_t;

typedef struct {
    uint8_t  mask;
} EPS_P31U_SetOutputs_Payload_t;

typedef struct {
    uint8_t  id;
} EPS_P31U_GetHk_Payload_t;
 
typedef struct {
    int16_t  voltage[3];
} EPS_P31U_SetPvVolt_Payload_t;

typedef struct {
    uint8_t  mode;
} EPS_P31U_SetPvAuto_Payload_t;

typedef struct {
    uint8_t  cmd;
    uint8_t  heater;
    uint8_t  mode;
} EPS_P31U_SetHeater_Payload_t;

typedef struct {
    uint8_t  ppt_mode;
    uint8_t  battheater_mode;
    int8_t   battheater_low;
    int8_t   battheater_high;
    uint8_t  output_normal_value[8];
    uint8_t  output_safe_value[8];
    uint16_t output_initial_on_delay[8];
    uint16_t output_initial_off_delay[8];
    uint16_t vboost[3];
} EPS_P31U_SetConfig_Payload_t;

typedef struct {
    uint16_t batt_maxvoltage;
    uint16_t batt_safevoltage;
    uint16_t batt_criticalvoltage;
    uint16_t batt_normalvoltage;
    uint32_t reserved1[2];
    uint8_t  reserved2[4];
} EPS_P31U_SetConfig2_Payload_t;

typedef struct {
    uint8_t  version;
    uint8_t  cmd;
    uint8_t  length;
    uint8_t  flags;
    uint16_t cur_lim[8];
    uint8_t  cur_ema_gain;
    uint8_t  cspwdt_channel[2];
    uint8_t  cspwdt_address[2];
} EPS_P31U_SetConfig3_Payload_t;

typedef struct {
    uint8_t  port;
    uint8_t  reserved;
    uint8_t  txSize;
    uint8_t  rxSize;
    uint8_t  tx[128];
} EPS_P31U_Transaction_Payload_t;

/*************************************************************************/
/*
** Type definition (EPS housekeeping)
*/

typedef struct {

} EPS_P31U_HkTlm_Payload_t;

typedef struct SAMPLE_APP_HkTlm_Payload {
    uint8_t CommandErrorCounter;
    uint8_t CommandCounter;
    uint8_t spare[2];
} EPS_HkTlm_Payload_t;



/**
 * Noarg cmd template.
 */
typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
} EPS_NoArgCmd_t;

typedef EPS_NoArgCmd_t  EPS_NoopCmd_t;
typedef EPS_NoArgCmd_t  EPS_ResetCountersCmd_t;
typedef EPS_NoArgCmd_t  EPS_GetCountersCmd_t;
typedef EPS_NoArgCmd_t  EPS_GetAppDataCmd_t;

typedef EPS_NoArgCmd_t  EPS_P31U_ResetCountersCmd_t;
typedef EPS_NoArgCmd_t  EPS_P31U_ResetWdtCmd_t;
typedef EPS_NoArgCmd_t  EPS_P31U_HardResetCmd_t;

typedef EPS_NoArgCmd_t  EPS_P31U_GetHkAllCmd_t;
typedef EPS_NoArgCmd_t  EPS_P31U_GetHkOutCmd_t;
typedef EPS_NoArgCmd_t  EPS_P31U_GetHkViCmd_t;
typedef EPS_NoArgCmd_t  EPS_P31U_GetHkWdtCmd_t;
typedef EPS_NoArgCmd_t  EPS_P31U_GetHkBasicCmd_t;
typedef EPS_NoArgCmd_t  EPS_P31U_GetHkOldCmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    EPS_P31U_GetHk_Payload_t Payload;
} EPS_P31U_GetHkCmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    EPS_P31U_SetOutputSingle_Payload_t Payload;
} EPS_P31U_SetOutputSingleCmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    EPS_P31U_SetOutputs_Payload_t Payload;
} EPS_P31U_SetOutputsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    EPS_P31U_SetPvVolt_Payload_t Payload;
} EPS_P31U_SetPvVoltCmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    EPS_P31U_SetPvAuto_Payload_t Payload;
} EPS_P31U_SetPvAutoCmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    EPS_P31U_SetHeater_Payload_t Payload;
} EPS_P31U_SetHeaterCmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    EPS_P31U_SetConfig_Payload_t Payload;
} EPS_P31U_SetConfigCmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    EPS_P31U_SetConfig2_Payload_t Payload;
} EPS_P31U_SetConfig2Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    EPS_P31U_SetConfig3_Payload_t Payload;
} EPS_P31U_SetConfig3Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader_t CommandHeader;
    EPS_P31U_Transaction_Payload_t Payload;
} EPS_P31U_TransactionCmd_t;


/*************************************************************************/
/*
** Type definition (EPS housekeeping)
*/
typedef EPS_NoArgCmd_t  EPS_SendHkCmd_t;


typedef struct {
    CFE_MSG_TelemetryHeader_t  TelemetryHeader;
    EPS_HkTlm_Payload_t Payload;
} EPS_HkTlm_t;


















/****************************************** For BEE-1000 ******************************************/

/**********************************/
/*                                */
/*         STX (BEE-1000)         */
/*       Endurohat S-band Tx      */
/*                                */
/**********************************/

// Define MSGID

// Define Function Codes
#define STX_NOOP_CC           0
#define STX_RESET_COUNTERS_CC 1
//#define STX_PROCESS_CC        2
#define STX_DISPLAY_PARAM_CC  3

/**set**/
#define STX_SET_SYMBOLRATE      10
#define STX_SET_TRANSMITPW      11
#define STX_SET_CENTERFREQ      12
#define STX_SET_MODCOD          13
#define STX_SET_ROLLOFF         14
#define STX_SET_PILOTSIG        15
#define STX_SET_FECFRAMESZ      16
#define STX_SET_PRETX_DELAY               17
#define STX_SET_ALL_PRAMETERS            18
#define STX_SET_RS485BAUD                19
#define STX_SET_MODULATOR_DATA_INTERFACE 20

/**get**/
#define STX_GET_SYMBOL_RATE     30
#define STX_GET_TX_POWER        31
#define STX_GET_CENTER_FREQ     32
#define STX_GET_MODCOD          33
#define STX_GET_ROLL_OFF        34
#define STX_GET_PILOT_SIGNAL    35
#define STX_GET_FEC_FRAME_SIZE  36
#define STX_GET_PRETX_DELAY     37
#define STX_GET_ALL_PRAMETERS           38
#define STX_GET_REPORT                  39
#define STX_GET_MODULATOR_DATA_INTERFACE 40

#define STX_FILESYS_CC_DIR               50
#define STX_FILESYS_CC_DIRNEXT           51
#define STX_FILESYS_CC_DELFILE           52
#define STX_FILESYS_CC_DELALLFILE        53
#define STX_FILESYS_CC_CREATEFILE        54
#define STX_FILESYS_CC_WRITEFILE         55
#define STX_FILESYS_CC_OPENFILE          56
#define STX_FILESYS_CC_READFILE          57
#define STX_FILESYS_CC_SENDFILE          58
// #define STX_FILESYS_CC_SENDFILEPI        59
// #define STX_FILESYS_CC_SENDFILERTI       60
// #define STX_FILESYS_CC_SENDFILEAI        61

#define STX_SYSCONF_CC_TRANSMITMODE      62
#define STX_SYSCONF_CC_IDLEMODE          63
#define STX_SYSCONF_CC_UPDATEFW          64
#define STX_SYSCONF_CC_SAFESHUTDOWN      65

// (주석 처리된 부분 기준)
#define STX_GETRES_CC_GETRES             66

// Define CMD Structure






/*******************************************/
/*                                         */
/*             UANT (BEE-1000)             */
/*      GomSpace NanoCom ANT-6F UHF VHF    */
/*                                         */
/*******************************************/

// Define MSGID

// Define Function Codes
#define   UANT_APP_NOOP_CC                 1         
#define   UANT_APP_RESET_COUNTERS_CC       2   
#define   UANT_APP_SOFT_REBOOT_CC          3      

    /* Burn control */
#define   UANT_APP_BURN_CHANNEL_CC         4      
#define   UANT_APP_STOP_BURN_CC            5        

    /* Telemetry */
#define   UANT_APP_GET_STATUS_CC           6        
#define   UANT_APP_GET_BACKUP_STATUS_CC    7 
#define   UANT_APP_GET_BOARD_STATUS_CC     8  
#define   UANT_APP_GET_TEMPERATURE_CC      9  
#define   UANT_APP_GET_SETTINGS_CC         10 
#define   UANT_APP_SET_SETTINGS_CC         11 
#define   UANT_APP_AUTODEPLOY_CC           12  

// Define CMD Structure

typedef struct {
    CFE_MSG_CommandHeader     CmdHeader;
} UANT_NoArgsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader     CmdHeader; 
    uint8_t                   Arg;
} UANT_u8Cmd_t;

typedef struct {
    CFE_MSG_CommandHeader     CmdHeader; 
    uint8_t                   addr;
    uint8_t                   channel;
    uint8_t                   duration;
} UANT_BurnChannelCmd_t;

typedef struct {
    CFE_MSG_CommandHeader     CmdHeader; 
    uint8_t                   addr;
    uint16_t                  MinutesUntilDeploy;
    uint8_t                   BackupActive;
    uint8_t                   MaxBurnDuration;
} UANT_SetSettingsCmd_t;

typedef struct {
    CFE_MSG_CommandHeader     CmdHeader; 
    uint16_t                  SecondsDelay;
    uint8_t                   addrslave1;
    uint8_t                   addrslave2;
} UANT_AutoDeployCmd_t;


typedef UANT_NoArgCmd_t    UANT_NoopCmd_t;
typedef UANT_NoArgCmd_t    UANT_ResetCountersCmd_t;

typedef UANT_u8Cmd_t       UANT_SoftRebootCmd_t;
typedef UANT_u8Cmd_t       UANT_StopBurnCmd_t;
typedef UANT_u8Cmd_t       UANT_GetStatusCmd_t;
typedef UANT_u8Cmd_t       UANT_GetBackupStatusCmd_t;
typedef UANT_u8Cmd_t       UANT_GetBoardStatusCmd_t;
typedef UANT_u8Cmd_t       UANT_GetTemperatureCmd_t;
typedef UANT_u8Cmd_t       UANT_GetSettingsCmd_t;







typedef struct {


    /* COSMIC SANT */
    SANT_SetSettingsCmd_t santsetsettingscmd;

    /* BEE-1000 UANT */
    UANT_BurnChannelCmd_t  uantburnchannelcmd;
    UANT_SetSettingsCmd_t  uantsetsettingscmd;
    UANT_AutoDeployCmd_t   uantautodeploycmd;

    

}__attribute__((packed)) Command;

#endif