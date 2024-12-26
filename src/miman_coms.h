#pragma once
#ifndef _MIMAN_COMS_H_
#define _MIMAN_COMS_H_

#include "miman_config.h"
#include "miman_orbital.h"

typedef struct{
	uint16_t Identifier;
	uint16_t PacketType;
	uint32_t Length;
	uint8_t Data[];

}__attribute__((packed)) packetsign;

typedef struct{
	uint16_t target;
	uint16_t filestatus;
	uint32_t filenum;
	uint32_t offset;
	uint32_t step;
}__attribute__((packed)) dlreqdata;

typedef struct{
	int32_t filenum;
	uint32_t file[];
}__attribute__((packed)) filelist;

// typedef enum {
//     GS_FTP_INFO_FILE  = 0,   /**< File size and checksum */
//     GS_FTP_INFO_CRC = 1,         /**< CRC of remote and local file */
//     GS_FTP_INFO_COMPLETED = 2,   /**< Completed and total chunks */
//     GS_FTP_INFO_PROGRESS = 3,    /**< Current chunk, total_chunks and chunk_size */
// } gs_ftp_info_type_t;

typedef struct {
    FILE       * fp;
    FILE       * fp_map;
    csp_conn_t * conn;
    uint32_t   timeout;
    char       file_name[GS_FTP_PATH_LENGTH];
    uint32_t   file_size;
    uint32_t   chunks;
    int        chunk_size;
    uint32_t   checksum;
    ftp_status_element_t last_status[GS_FTP_STATUS_CHUNKS];
    uint32_t   last_entries;
    gs_ftp_info_callback_t info_callback;
    void       * info_data;
} gs_ftp_state_t;

typedef struct {
    gs_ftp_backend_type_t backend;
    const char * path;
    uint32_t addr;
    uint32_t size;
} gs_ftp_url_t;



typedef struct {
	//Callsign
	char CallSign[5];

	//CCSDS Header
	uint16_t CCMessage_ID;
	uint16_t CCSequence;
	uint16_t CCLength;
	uint8_t CCTime_code[6];
	uint8_t CCPadding[4];

	//FM
	uint32_t FMSpaceTime;
	uint16_t FMRebootCount;
	uint8_t FMCurMode;
	uint8_t FmCurSubMode;
	uint8_t FMPreviousMode;
	uint8_t FMPreviousSubMode;

	//EPS->System
	uint8_t SYMode;
	uint8_t SYResetCause;
	uint16_t SYResetcount;

	//EPS->Battery
	uint16_t BAtteryStatus;
	int16_t BAtteryInV;
	int16_t BAtteryInI;
	int16_t BAtteryTemp;

	//EPS->Motherboard
	int16_t MOMPPTInV[3];
	int16_t MOMPPTInI[3];
	int16_t MOChancur3V[4];
	int16_t MOChancur5V[4];
	int16_t MOBoardSupV;
	int16_t MOBoardTemp;

	//EPS->Daughterboard
	int16_t DAMPPTInV[2];
	int16_t DAMPPTInI[2];
	int16_t DAChancur5V;
	int16_t DAChancur12V;

	//SNSR->IMU
	float IMTemp;
	float IMAngV[3];
	uint16_t IMRWErrcount;

	//UTRX
	uint32_t UXTotRXByte;
	uint16_t UXBootCount;
	uint8_t UXActiveConf;


	
}__attribute__((packed)) Beacon;

typedef struct {
	uint16_t Message_ID;
	uint16_t Sequence;
	uint16_t Length;
	uint8_t Time_code[6];
}__attribute__((packed)) CCSDS_Header_;

typedef struct {
	uint32_t Spacecraft_Time_seconds;
	uint32_t Spacecraft_Time_subseconds;
	uint16_t Reboot_Count;
	uint8_t Current_Mode;
	uint8_t Current_Submode;
	uint8_t Previous_Mode;
	uint8_t Previous_Submode;
	uint8_t Deployment_Phase;
	uint32_t Reserved;

}__attribute__((packed)) FM_HK_;

typedef struct {
	uint8_t Mode;
	uint8_t Reset_Cause;
	uint16_t Reset_Count;
	uint16_t Output_Bus_On_Field;
	uint16_t Output_Bus_Overcurrent_Fault_Field;
	uint16_t Battery_Status;
	int16_t Battery_Input_Voltage;
	int16_t Battery_Input_Current;
	int16_t Battery_Temperature;
	int16_t M_MPPT_Input_Voltage[3];
	int16_t M_MPPT_Input_Current[3];
	int16_t M_Channel_Current_3V3[4];
	int16_t M_Channel_Current_5V[4];
	int16_t M_Board_Supply_Voltage;
	int16_t M_Board_Temperature;
	int16_t D_MPPT_Input_Voltage[2];
	int16_t D_MPPT_Input_Current[2];
	int16_t D_Channel_Current_5V;
	int16_t D_Channel_Current_12V;
}__attribute__((packed)) EPS_;

typedef struct {
	uint8_t Temperature_Measurements[5];
	uint8_t Padding;
}__attribute__((packed)) TCS_;

typedef struct {
	uint8_t Wheel_Enabled_Field;
	uint8_t Wheel_Error_Flag[3];
	int16_t Wheel_Speed[3];
	int16_t Wheel_Reference_Speed[3];
}__attribute__((packed)) RWA_;

typedef struct {
	uint8_t Enable_Status;
	int8_t Duty[3];
}__attribute__((packed)) MTQ_;

typedef struct {
	float STT_MCU_Temperature;
	uint16_t STT_Error_Count;
	uint8_t STT_Padding[2];
	uint16_t IMU_Read_Write_Error_Count;
	uint8_t IMU_Sleep;
	uint8_t IMU_Padding;
	uint16_t MMT_Measurement_Error_Count;
	uint8_t CSS_Isolation_Status;
	uint8_t CSS_Padding;
}__attribute__((packed)) SNSR_;

typedef struct {
	uint32_t Total_TX_Byte;
	uint32_t Total_RX_Byte;
	uint32_t Last_Contact;
	uint16_t Boot_Count;
	uint16_t Boot_Cause;
	int16_t Last_RSSI;
	int16_t Last_RF_Error;
	int16_t Board_Temperature;
	uint8_t Active_Conf;
	uint8_t TX_Duty;
}__attribute__((packed)) UTRX_;

typedef struct {
	float PA_Temperature;
	float Output_Power;
	float Battery_Current;
	float Battery_Voltage;
	uint8_t Reset_Count;
	uint8_t Control_Mode;
	uint8_t Status;
	uint8_t Padding;
	uint16_t Tx_Ready_Error_Count;
	uint16_t Tx_End_Error_Count;
	uint16_t Last_Buffer_Overruns;
	uint16_t Last_Buffer_Underruns;
}__attribute__((packed)) STX_;

typedef struct {
	uint16_t Temprature;
	uint16_t Exposure_Time;
	uint16_t Snapshot_Execute_Count;
	uint16_t Snapeshot_Error_Count;
	uint16_t Reserved[4];
}__attribute__((packed)) PAY_;

typedef struct {
	uint16_t Message_ID;
	uint16_t Sequence;
	uint16_t Length;
	uint8_t Time_code[6];
}__attribute__((packed)) Header_;

typedef struct {
	uint32_t Spacecraft_Time_seconds;
	uint32_t Spacecraft_Time_subseconds;
	uint8_t ADCS_mode;
}__attribute__((packed)) FM_AOD_ ;

typedef struct {
	uint8_t AC_Mode_Flag;
	uint8_t Actuator_Flag;
	uint8_t Control_Input_Options[3];
	float Quaternions[4];
	float Angular_Velocity[3];
	float Angle_Error;
	float Angular_Rate_Error[3];
	uint8_t ADCS_Output_Flags[10];
	float Best_Position[3];
	float Position_Deviation[3];
	uint8_t Position_Solution_Status;
	float Best_Velocity[3];
	float Velocity_Deviation[3];
	uint8_t Velocity_Solution_Status;
}__attribute__((packed)) ADCS_;

typedef struct {
	CCSDS_Header_ CCSDS_Header;
	FM_HK_ FM;
	EPS_ EPS;
	TCS_ TCS;
	RWA_ RWA;
	MTQ_ MTQ;
	SNSR_ SNSR;
	UTRX_ UTRX;
	STX_ STX;
	PAY_ PAY;
}__attribute__((packed)) HK;

typedef struct {
	Header_ Header;
	FM_AOD_ FM;
	ADCS_ ADCS;
}__attribute__((packed)) AOD;

typedef struct {
	uint32_t ExTime;
	uint32_t ExWindow;
	uint16_t EntryID;
	uint16_t GroupID;
	uint8_t cmd[];
}__attribute__((packed)) Book;

void * TRxController(void *);
void * SignalTest(void*);
void now_rx_bytes_update();
void set_rx_bytes(uint32_t nowbytes);
uint32_t get_rx_bytes();
uint32_t * get_rx_bytes_address();
void buf_allclear();
void CalculateChecksum(CommandHeader_t* Cmd);
int32_t GenerateCmdMsg(CommandHeader_t* Cmd, uint16_t MsgId, uint8_t FcnCode, uint32_t ArgLen);
csp_socket_t *  DL_sock_initialize();
int BeaconSaver(Beacon * bec);
void * task_downlink_onorbit(void * socketinfo);
void * task_uplink_onorbit(void * sign_);

int PacketHandler(csp_packet_t *packet, int type, int NowCursor);
packetsign * PingInit(FSWTle * FSWTleinfo);
csp_packet_t * PacketEncoder(packetsign * sign, bool freeer = true);
packetsign * PacketDecoder(csp_packet_t * packet);


class CmdGenerator_GS {
private:
    void SetHeaderWord(uint8_t* Word16, uint16_t Value, uint16_t Mask);
    void SetHeaderByte(uint8_t* Byte, uint8_t Value, uint8_t Mask);
    void GetHeaderWord(const uint8_t* Word16, uint16_t& Value, uint16_t Mask);
    uint32_t ComputeCheckSum(void);

public:
	CFE_MSG_CommandHeader* CmdHeader;
	bool Scheduled = false;
	bool Checksum = true;

    CmdGenerator_GS(void);
    ~CmdGenerator_GS(void);

    int GenerateCmdHeader(uint32_t MsgId, uint16_t FncCode, uint32_t Size, void* Data);
    void CopyCmdHeaderToBuffer(uint8_t* Buffer);

    void InitHeader(void);
    void SetHeader(const CFE_MSG_CommandHeader* Header);
    const CFE_MSG_CommandHeader* GetHeader(void) const;

    int SetHasSecondaryHeader(bool HasSec);
    int SetMsgId(uint16_t MsgId);
    int SetSize(uint16_t Size);
    int SetSegmentationFlag(uint16_t SegFlag);
    int SetFncCode(uint16_t FncCode);

    bool HasSecondaryHeader(void) const;
    uint16_t GetSize(void);
    uint16_t GetFncCode(void) const;

    int GenerateChecksum(void);
	int Scheduling(uint32_t ExecutionTime, uint32_t ExecutionWindow, uint32_t EntryID, uint16_t GroupID);
	packetsign * GenerateCMDPacket(void);
};

void * Direct_Shell(void * data);

#endif _MIMAN_COMS_H_

