/* Customs */
#include "miman_config.h"
#include "miman_csp.h"
#include "miman_coms.h"
#include "miman_imgui.h"
#include "miman_config.h"
#include "miman_orbital.h"
#include "miman_radial.h"
#include "miman_ftp.h"

#include <netinet/in.h>

#define _CRT_SECURE_NO_WARNINGS

extern FILE *log_ptr;

//int signallen = 8;
Beacon* beacon = (Beacon *)malloc(MIM_LEN_BEACON);
int NowFTP = 0;

char HKbuf[202];
int HKbufCursor = 0;
char AODbuf[131];
int AODbufCufsor = 0;
CmdGenerator_GS * SatCMD[256];
extern StateCheckUnit State;
extern pthread_t p_thread[16];
extern pthread_mutex_t conn_lock;

HK *NowHK;
AOD *NowAOD;

extern Console console;
extern Setup * setup;

pthread_t LinkTrhead;

int BeaconCounter;
int PingCounter;
uint32_t remote_total_rx_bytes = 0;

void * TRxController(void *)
{
    csp_socket_t * DLsocket = DL_sock_initialize();
    while(State.TRx_mode)
    {
        if(State.downlink_mode)
        {
            printf("Create downlink onorbit...\n");
            pthread_create(&LinkTrhead, NULL, task_downlink_onorbit, DLsocket);
            pthread_join(LinkTrhead, NULL);
            State.uplink_mode = true;
        }
        else
            continue;
    }
}

void buf_allclear()
{
    memset(HKbuf, 0, sizeof(HKbuf));
    memset(AODbuf, 0, sizeof(HKbuf));
    HKbufCursor = 0;
    //AODBufCursor = 0;
}

void * SignalTest(void*)
{
    State.Signaltest = true;
    while(State.Signaltest)
    {
        if(csp_ping(19, 100, 1, 0))
            continue;
    }
}

void now_rx_bytes_update()
{
    if(beacon->UXTotRXByte != remote_total_rx_bytes)
        remote_total_rx_bytes = beacon->UXTotRXByte;
}

void set_rx_bytes(uint32_t nowbytes)
{
    remote_total_rx_bytes = nowbytes;
}

uint32_t get_rx_bytes()
{
    return remote_total_rx_bytes;
}

uint32_t * get_rx_bytes_address()
{
    return &remote_total_rx_bytes;
}

CmdGenerator_GS::CmdGenerator_GS(void) {
    uint8_t* pool = new uint8_t[MIM_DEFAULT_DATALEN];
    if (!pool) {
        console.AddLog("Generator init error: alloc failed");
        return;
    }
    memset(pool, 0, MIM_DEFAULT_DATALEN);
    this->CmdHeader = (CFE_MSG_CommandHeader*) pool;
}

CmdGenerator_GS::~CmdGenerator_GS(void) {
    uint8_t* pool = (uint8_t*) this->CmdHeader;
    delete[] CmdHeader;
}

int CmdGenerator_GS::GenerateCmdHeader(uint32_t MsgId, uint16_t FncCode, uint32_t Size, void* Data) {
    memset(this->CmdHeader->Msg.Byte, 0, Size);

    if (this->SetHasSecondaryHeader(true) != 0) {
        return -1;
    }
    if (this->SetSegmentationFlag(CFE_MSG_SEGFLG_UNSEG) != 0) {
        return -1;
    }
    if (this->SetMsgId(MsgId) != 0) {
        return -1;
    }
    if (this->SetSize(Size) != 0) {
        return -1;
    }
    if (this->SetFncCode(FncCode) != 0) {
        return -1;
    }
    if (this->GenerateChecksum() != 0) {
        return -1;
    }
    if (Data && (Size - sizeof(CFE_MSG_CommandHeader)) > 0)
        memcpy(this->CmdHeader->Msg.Byte + sizeof(CFE_MSG_CommandHeader), Data, Size - sizeof(CFE_MSG_CommandHeader));
    return 0;
}

void CmdGenerator_GS::CopyCmdHeaderToBuffer(uint8_t* Buffer) {
    memcpy(Buffer, this->CmdHeader->Msg.Byte, this->GetSize());
}

int CmdGenerator_GS::SetFncCode(uint16_t FncCode) {
    if (!this->HasSecondaryHeader()) {
        return -1;
    }
    this->CmdHeader->Sec.FunctionCode = FncCode;
    return 0;
}


void CmdGenerator_GS::SetHeaderWord(uint8_t* Word16, uint16_t Value, uint16_t Mask) {
    Word16[0] = (Word16[0] & ~(Mask >> 8)) | ((Value & Mask) >> 8);
    Word16[1] = ((Word16[1] & ~Mask) | (Value & Mask)) & 0xFF;
}


void CmdGenerator_GS::GetHeaderWord(const uint8_t* ByteField, uint16_t& Value, uint16_t Mask) {
    Value = (ByteField[0] << 8 | ByteField[1]) & Mask;
}

void CmdGenerator_GS::SetHeaderByte(uint8_t* Byte, uint8_t Value, uint8_t Mask) {
    Byte[0] = (Byte[0] & ~Mask) | (Value & Mask);
}

int CmdGenerator_GS::SetHasSecondaryHeader(bool HasSec) {
    SetHeaderByte(&this->CmdHeader->Msg.CCSDS.Pri.StreamId[0], HasSec, 0x08);
    return 0;
}

bool CmdGenerator_GS::HasSecondaryHeader(void) const {
    return (this->CmdHeader->Msg.CCSDS.Pri.StreamId[0] & (CFE_MSG_SHDR_MASK >> 8)) != 0;
}


int CmdGenerator_GS::SetSize(uint16_t Size) {
    if (Size < CFE_MSG_SIZE_OFFSET || Size > (0xFFFF + CFE_MSG_SIZE_OFFSET)) {
        return -1;
    }
    Size -= CFE_MSG_SIZE_OFFSET;
    SetHeaderWord(this->CmdHeader->Msg.CCSDS.Pri.Length, Size, 0xFFFF);
    return 0;
}

uint16_t CmdGenerator_GS::GetSize(void) {
    uint16_t RetVal;
    GetHeaderWord(this->CmdHeader->Msg.CCSDS.Pri.Length, RetVal, 0xFFFF);
    return RetVal + CFE_MSG_SIZE_OFFSET;
}

uint16_t CmdGenerator_GS::GetFncCode(void) const
{
    return this->CmdHeader->Sec.FunctionCode;
}

int CmdGenerator_GS::SetMsgId(uint16_t MsgId) {
    SetHeaderWord(this->CmdHeader->Msg.CCSDS.Pri.StreamId, MsgId, 0xFFFF);
    return 0;
}

int CmdGenerator_GS::SetSegmentationFlag(uint16_t SegFlag) {
    SetHeaderWord(this->CmdHeader->Msg.CCSDS.Pri.Sequence, SegFlag, CFE_MSG_SEGFLG_MASK);
    return 0;
}


int CmdGenerator_GS::GenerateChecksum(void) {
    CFE_MSG_CommandHeader* Cmd = this->CmdHeader;
    Cmd->Sec.Checksum = 0;
    Cmd->Sec.Checksum = this->ComputeCheckSum();
    return 0;
}

uint32_t CmdGenerator_GS::ComputeCheckSum(void) {
    uint16_t Len = this->GetSize();
    const uint8_t* BytePtr = this->CmdHeader->Msg.Byte;
    uint32_t chksum  = 0xFF;

    while (Len--) {
        chksum ^= *(BytePtr++);
    }
    return chksum;
}


csp_socket_t * DL_sock_initialize()
{
	csp_socket_t * sock = csp_socket(0);
	if(!csp_bind(sock, 13)) {
        console.AddLog("[OK]##TM Port 13 bind success.");
    }
    if(!csp_bind(sock, 31)) {
        console.AddLog("[OK]##BCN Port 31 bind success.");
    }

    while(true) {
        if (csp_bind(sock, 23) == 0) { // Add for HVD_TMTC_TEST
        console.AddLog("[OK]Bind Success.");
        break;
        };
    }
	csp_listen(sock, 10);
    //Fail preventation would be needed!

    return sock;
}

// void * PacketDebugMsg(csp_packet_t * packet)
// {
//     char DebugMsg[1024];
//     (char *)packet->data
//     for(int i = 0; i < packet->length; i++)
//     {
//         sprintf(DebugMsg + i, "%")
//     }
// }

int BeaconSaver(Beacon * bec)
{   
    BeaconCounter += 1;
    char beaconline[64];
    char beaconname[64];
    char binarybuf[1024];
    time_t tmtime = time(0);
    struct tm * local = localtime(&tmtime);
    sprintf(beaconname, "./data/beacon/Beacon--%04d-%02d-%02d-%02d-%02d-%02d--", local->tm_year+1900, local->tm_mon+1, local->tm_mday,local->tm_hour, local->tm_min, local->tm_sec);
    // sprintf(binarybuf, )
    FILE * beacon_fp;
    beacon_fp = fopen(beaconname, "wb");
    for(int i = 0 ; i < 5; i++)
        beaconline[i] = bec->CallSign[i];
    beaconline[6] = 0;
    // fprintf(beaconline);
    fprintf(beacon_fp, "Call Sign : %s\n", beaconline);
    fprintf(beacon_fp, "Message ID : 0x%x\n", csp_ntoh16(bec ->CCMessage_ID));
    fprintf(beacon_fp, "Sequence : 0x%x\n", csp_ntoh16(bec ->CCSequence));
    fprintf(beacon_fp, "Length : %"PRIu16"\n", csp_ntoh16(bec ->CCLength) + 7);
    fprintf(beacon_fp, "Time Code : 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", bec ->CCTime_code[0], bec ->CCTime_code[1], bec ->CCTime_code[2], bec ->CCTime_code[3], bec ->CCTime_code[4], bec ->CCTime_code[5]);
    fprintf(beacon_fp, "Spacecraft Time : %"PRIu32"\n", bec ->FMSpaceTime);
    fprintf(beacon_fp, "Reboot Count : %"PRIu16"\n", bec ->FMRebootCount);
    fprintf(beacon_fp, "Current Mode : %"PRIu8"\n", bec ->FMCurMode);
    fprintf(beacon_fp, "Current Submode : %"PRIu8"\n", bec ->FmCurSubMode);
    fprintf(beacon_fp, "Previous Mode : %"PRIu8"\n", bec ->FMPreviousMode);
    fprintf(beacon_fp, "Previous SubMode : %"PRIu8"\n", bec ->FMPreviousSubMode);
    fprintf(beacon_fp, "Mode : %"PRIu8"\n", bec ->SYMode);
    fprintf(beacon_fp, "Reset Cause : %"PRIu8"\n", bec ->SYResetCause);
    fprintf(beacon_fp, "Reset Count : %"PRIu16"\n", bec ->SYResetcount);
    fprintf(beacon_fp, "Battery Status : 0x%x\n", bec ->BAtteryStatus);
    fprintf(beacon_fp, "Battery IN Voltage : %"PRId16"\n", bec ->BAtteryInV);
    fprintf(beacon_fp, "Baterry In Current : %"PRId16"\n", bec ->BAtteryInI);
    fprintf(beacon_fp, "Battery Temperature : %"PRId16"\n", bec ->BAtteryTemp);
    fprintf(beacon_fp, "Mboard MPPT In V : %"PRId16" %"PRId16" %"PRId16"\n", bec ->MOMPPTInV[0], bec ->MOMPPTInV[1], bec ->MOMPPTInV[2]);
    fprintf(beacon_fp, "Mboard MPPT In I : %"PRId16" %"PRId16" %"PRId16"\n", bec ->MOMPPTInI[0], bec ->MOMPPTInI[1], bec ->MOMPPTInI[2]);
    fprintf(beacon_fp, "Mboard Channel I 3V : %"PRId16" %"PRId16" %"PRId16" %"PRId16"\n", bec ->MOChancur3V[0], bec ->MOChancur3V[1], bec ->MOChancur3V[2], bec ->MOChancur3V[3]);
    fprintf(beacon_fp, "Mboard Channel I 5V : %"PRId16" %"PRId16" %"PRId16" %"PRId16"\n", bec ->MOChancur5V[0], bec ->MOChancur5V[1], bec ->MOChancur5V[2], bec ->MOChancur5V[3]);
    fprintf(beacon_fp, "Mboard Supply V : %"PRId16"\n", bec ->MOBoardSupV);
    fprintf(beacon_fp, "Mboard Temperature : %"PRId16"\n", bec ->MOBoardTemp);
    fprintf(beacon_fp, "Dboard MPPT In V : %"PRId16" %"PRId16"\n", bec ->DAMPPTInV[0], bec ->DAMPPTInV[1]);
    fprintf(beacon_fp, "Dboard MPPT In I : %"PRId16" %"PRId16"\n", bec ->DAMPPTInI[0], bec ->DAMPPTInI[1]);
    fprintf(beacon_fp, "Dboard Channel I 5V : %"PRId16"\n", bec ->DAChancur5V);
    fprintf(beacon_fp, "Dboard Channel I 12V : %"PRId16"\n", bec ->DAChancur12V);
    fprintf(beacon_fp, "Temperature : %.2f\n", bec ->IMTemp);
    fprintf(beacon_fp, "Angular Velocity : %.2f %.2f %.2f\n", bec ->IMAngV[0], bec ->IMAngV[1], bec ->IMAngV[2]);
    fprintf(beacon_fp, "RW Error Count : %"PRIu16"\n", bec ->IMRWErrcount);
    fprintf(beacon_fp, "Total RX Bytes : %"PRIu32"\n", bec ->UXTotRXByte);
    fprintf(beacon_fp, "Boot Count : %"PRIu16"\n", bec ->UXBootCount);
    fprintf(beacon_fp, "Active Conf : %"PRIu8"\n", bec ->UXActiveConf);
    fprintf(beacon_fp, "\n\n\n\nBinary : \n");
    // fprintf(beacon_fp, )
    for(int i = 0; i < sizeof(Beacon) ; i++)
        fprintf(beacon_fp, "%x\t", ((char *)bec)[i]);
    fprintf(beacon_fp, "\n");
    fflush(beacon_fp);
    fclose(beacon_fp);
}
void * task_downlink_onorbit(void * socketinfo) 
{
    csp_socket_t * sock = (csp_socket_t *)socketinfo;

    // csp_bind(sock, CSP_PING);
	// csp_bind(sock, 12);
    // // csp_bind(sock, 31);
    // csp_bind(sock, 23);
	// csp_listen(sock, 10);

    csp_packet_t * packet = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
    packetsign * confirm = (packetsign *)malloc(MIM_LEN_PACKET);
    csp_conn_t * conn;

    //Need to copy pointer
    //This function must be on p_thread[3]
    float seconds = 0.0f;
	while (State.downlink_mode) {
        printf("Downlink ongoing...\n");
		if ((conn = csp_accept(sock, setup->default_timeout)) == NULL)
        {
            // printf("Running...but no comming...%f\n", seconds);
            seconds += 0.5;
            continue;
        }
            
        console.AddLog("Someone Comming...");
		while ((packet = csp_read(conn, setup->default_timeout)) != NULL) {
			switch(csp_conn_dport(conn)) {
                // For TMTC Test: Port 23
                case 23: {
                    char tmtcfilename[128];
                    time_t tmtime = time(0);
                    struct tm * local = localtime(&tmtime);
                    sprintf(tmtcfilename, "./data/tmtc/tmtc_test--%04d-%02d-%02d-%02d-%02d-%02d--", local->tm_year+1900, local->tm_mon+1, local->tm_mday,local->tm_hour, local->tm_min, local->tm_sec);
                    
                    uint16_t PacketLength = packet->length;
                    uint8_t data[96] = {0,};
                    memcpy(data, packet->data, PacketLength);
                    console.AddLog("TMTC Test Downlink requested.");
                    // FILE * TMTC_fp;
                    // TMTC_fp = fopen(tmtcfilename,"wb");
                    // for (int i=0; i<PacketLength; i++) {
                    //     fprintf(TMTC_fp, "Data %d: %u\n",i,packet->data[i]);
                    // }
                    // if (packet != NULL)
                    // {
                    //     csp_buffer_free(packet);
                    //     packet = NULL;
                    // }
                    // if (conn != NULL)
                    // {
                    //     csp_close(conn);
                    //     conn = NULL;
                    // }
                    // if(TMTC_fp != NULL)
                    // {
                    //     fclose(TMTC_fp);
                    // }
                    printf("Packet length: %u\n", PacketLength);

                    for(uint8_t i=0; i < packet->length; i++) {
                        printf("0x%02X\t", data[i]);
                        if (i%10 == 9) printf("\n");
                    }
                    break;
                    
                }
                case 12: {
                    console.AddLog("Signal Comming from Port 12.");
                    if (packet != NULL)
                        csp_buffer_free(packet);
                    if (conn != NULL)
                        csp_close(conn);
                    break;
                }
                //Case 13 : TM Packet Downlink
                case 13: {
                    printf("DL Length: %u\n",packet->length);
                    if(log_ptr == NULL) {
                        printf("Invalid file pointer.\n");
                        continue;
                    }
                    fprintf(log_ptr, "Downlink. Packet Length: %u\n",packet->length);
                    // Parsing & write header
                    //HYVRID_TelemetryHeader_t *hdr = (HYVRID_TelemetryHeader_t *)packet;
                    HYVRID_TelemetryHeader_t hdr = {0,};
                    memcpy(&hdr, packet->data, sizeof(HYVRID_TelemetryHeader_t));
                    fprintf(log_ptr, "Header Info.\n");
                    fprintf(log_ptr, "Stream ID: %x\n", htons(hdr.Tlmhdr.pri.stream));
                    fprintf(log_ptr, "Sequence: %x\n", htons(hdr.Tlmhdr.pri.sequence));
                    fprintf(log_ptr, "Length: %x\n", htons(hdr.Tlmhdr.pri.length));
                    fprintf(log_ptr, "Time Stamp: %x\t%x\t%x\t%x\t%x\t%x\n\n",
                            hdr.Tlmhdr.sec.Time[0],hdr.Tlmhdr.sec.Time[1],hdr.Tlmhdr.sec.Time[2],
                            hdr.Tlmhdr.sec.Time[3],hdr.Tlmhdr.sec.Time[4],hdr.Tlmhdr.sec.Time[5]);

                    fprintf(log_ptr, "HYVRID Header Info.\n");
                    fprintf(log_ptr, "RetCodeType: %x\n", hdr.Report.RetCodeType);
                    fprintf(log_ptr, "RetCode: %x\n", hdr.Report.RetCode);
                    fprintf(log_ptr, "MsgId: %x\t", hdr.Report.MsgId);
                    if (hdr.Report.MsgId <= 6260 || 6257 <= hdr.Report.MsgId) {
                        fprintf(log_ptr, "IFC (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6261 && hdr.Report.MsgId <= 6266) {
                        fprintf(log_ptr, "FM (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6267 && hdr.Report.MsgId <= 6272) {
                        fprintf(log_ptr, "EPS (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6273 && hdr.Report.MsgId <= 6279) {
                        fprintf(log_ptr, "ADCS (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6280 && hdr.Report.MsgId <= 6285) {
                        fprintf(log_ptr, "GRX (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6286 && hdr.Report.MsgId <= 6291) {
                        fprintf(log_ptr, "STX (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6292 && hdr.Report.MsgId <= 6297) {
                        fprintf(log_ptr, "UANT (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6298 && hdr.Report.MsgId <= 6303) {
                        fprintf(log_ptr, "UTRX (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6304 && hdr.Report.MsgId <= 6309) {
                        fprintf(log_ptr, "PAYC (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6310 && hdr.Report.MsgId <= 6315) {
                        fprintf(log_ptr, "PAYR (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6316 && hdr.Report.MsgId <= 6321) {
                        fprintf(log_ptr, "PAYS (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6322 && hdr.Report.MsgId <= 6324) {
                        fprintf(log_ptr, "CI (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6325 && hdr.Report.MsgId <= 6328) {
                        fprintf(log_ptr, "TO (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6329 && hdr.Report.MsgId <= 6330) {
                        fprintf(log_ptr, "SN (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6331 && hdr.Report.MsgId <= 6333) {
                        fprintf(log_ptr, "HK (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6334 && hdr.Report.MsgId <= 6335) {
                        fprintf(log_ptr, "SCH (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6336 && hdr.Report.MsgId <= 6337) {
                        fprintf(log_ptr, "DS (%x)\n",hdr.Report.MsgId);
                    }
                    else if(hdr.Report.MsgId >=6338 && hdr.Report.MsgId <= 6339) {
                        fprintf(log_ptr, "LOG (%x)\n",hdr.Report.MsgId);
                    }
                    else fprintf(log_ptr, "Unknown MsgID\n");
                    fprintf(log_ptr, "CC: %x\n", hdr.Report.CommandCode);
                    fprintf(log_ptr, "DataSize (Exclude header): %x\n", hdr.Report.DataSize);
                    fprintf(log_ptr, "Used State: %x\n\n", hdr.Report.UsedState);
                    fprintf(log_ptr, "Output Data.\n");
                    
                    
                    
                    for(int i=0; i< packet->length; i++) {
                        if(!(i%10) && i != 0) {
                            printf("\n");
                            fprintf(log_ptr, "\n");
                        }
                        printf("0x%x ",packet->data[i]);
                        fprintf(log_ptr, "%02hhx\t",packet->data[i]);
                    }
                    // for(int i = 0; i < packet->length / sizeof(uint32_t); i++)
                    //     packet->data32[i] = __builtin_bswap32(packet->data32[i]);
                    // fprintf(DL_fp, "CMD Cnt             : %02d\n", packet->data[0]);
                    // fprintf(DL_fp, "Reply Cnt           : %02d\n", packet->data[1]);
                    // fprintf(DL_fp, "Report Cnt          : %02d\n", packet->data[2]);
                    // fprintf(DL_fp, "CMD Err Cnt         : %02d\n", packet->data[3]);
                    // fprintf(DL_fp, "Reply Err Cnt       : %02d\n", packet->data[4]);
                    // fprintf(DL_fp, "Trans Err Cnt       : %02d\n", packet->data[5]);
                    // fprintf(DL_fp, "Packet Err Cnt      : %02d\n", packet->data[6]);
                    // fprintf(DL_fp, "Report Err Cnt      : %02d\n", packet->data[7]);
                    // fprintf(DL_fp, "Internal Last Err   : %02d\n", packet->data[8]);
                    // fprintf(DL_fp, "Last CC Reply       : %02d\n", packet->data[9]);
                    // fprintf(DL_fp, "Last Reply Length   : %02d\n", packet->data[10]);
                    // fprintf(DL_fp, "Last Reply Result   : %02d\n", packet->data[11]);
                    // fprintf(DL_fp, "Data                : ");
                    // if(packet->length > 12)
                    // {
                    //     for(int i = 12; i < packet->length; i++)
                    //         fprintf(DL_fp, "%02d ", packet->data[i]);
                    // }
                    fprintf(log_ptr, "\n");
                        

                    /* Clean up */
                    if (packet != NULL)
                    {
                        csp_buffer_free(packet);
                        packet = NULL;
                    }
                    if (conn != NULL)
                    {
                        csp_close(conn);
                        conn = NULL;
                    }
                    // if(log_ptr != NULL)
                    // {
                    //     fclose(DL_fp);
                    // }
                    printf("Report DL done.\n");
                    break;
                }
                
                //Case 31 : Beacon
				case 31: {
                    char bcnpktfilename[128];
                    time_t tmtime = time(0);
                    struct tm * local = localtime(&tmtime);
                    sprintf(bcnpktfilename, "../data/bcnpkt/bcnpkt--%04d-%02d-%02d-%02d-%02d-%02d--", local->tm_year+1900, local->tm_mon+1, local->tm_mday,local->tm_hour, local->tm_min, local->tm_sec);
                    console.AddLog("Received Beacon from port : %d.", csp_conn_dport(conn));
                    FILE * bcn_fp;
                    bcn_fp = fopen(bcnpktfilename, "wb");
                    printf("Beacon Length: %u",packet->length);
                    for(int i=0; i< packet->length; i++) {
                        if(!(i%10) && i!=0) {
                            printf("\n");
                            fprintf(bcn_fp, "\n");
                        }
                        printf("0x%x ",packet->data[i]);
                        fprintf(bcn_fp, "%02hhx\t",packet->data[i]);
                    }
                    // if(packet->length == MIM_LEN_BEACON)
                    // {
                    //     console.AddLog("Received Beacon from port : %d.", csp_conn_dport(conn));
                    //     if(State.Debugmode)
                    //     {
                    //         printf("Beacon Binary : \n");
                    //         for(int i = 0 ; i < packet->length; i++)
                    //         {
                    //             printf("%x\t", packet->data[i]);
                    //         }
                    //         printf("\n");
                    //     }
                    //     memcpy(beacon, packet->data, MIM_LEN_BEACON);		
                    //     //BeaconSaver(beacon);
                    // }
                    // else
                    // {
                    //     console.AddLog("Received Beacon but brocken.");
                    // }
					

                    /* Clean up */
                    if (packet != NULL)
                        csp_buffer_free(packet);
                    if (conn != NULL)
                        csp_close(conn);
                    if(bcn_fp != NULL)
                    {
                        fclose(bcn_fp);
                    }
                    break;
				}
				default: {
                    if (csp_conn_dport(conn) == 1)
                        console.AddLog("Ping Received through port 1.");
                    else
                    {
                        console.AddLog("Packet Received on unknown port %d", csp_conn_dport(conn));
                        console.AddLog("Packet length is %d", packet->length);
                    }
					csp_service_handler(conn, packet);
                    /* Clean up */
                    if (packet != NULL)
                        csp_buffer_free(packet);
                    if (conn != NULL)
                        csp_close(conn);
                    break;
				}
			}
		}
        RSSI_Monitoring();
    }
    printf("Downlink thread dead.\n");
    packet = NULL;
    conn = NULL;
    free(confirm);
    if (confirm != NULL)
        confirm = NULL;
}
void * task_uplink_onorbit(void * sign_)
{

    State.uplink_mode = true;
    //State.downlink_mode = false;
    pthread_mutex_lock(&conn_lock);
    //while(!State.RotatorReadReady)
    //    continue;
    bool dlstate = true;


    //This funcion must be on p_thread[4]
    if((dlstate))
    {
        State.downlink_mode = false;
    }
        
    while(!State.uplink_mode)
        continue;
    uint32_t start, time = 0;
    start = csp_get_ms();
    packetsign * sign;
    csp_packet_t * packet;
    csp_conn_t* txconn;
    console.AddLog("[DEBUG]## Start Uplink Task.");

    sign = (packetsign *) sign_;
    // for(int i =0; i < sign->Length; i++)
    //     printf("PacketByte : %d", sign->Data[i]);
    //packet = PacketEncoder(sign);

    

    uint16_t Ptype = sign->PacketType;
    uint32_t Plen = sign->Length;
    uint16_t filetype;
    uint16_t filestatus;

    if(Ptype == MIM_PT_DLREQ)
    {
        filetype = ((dlreqdata *)(sign->Data))->target;
        filestatus = ((dlreqdata *)(sign->Data))->filestatus;
    }
    
    
    if(State.Debugmode)
    {
        for(int i = 0 ; i < sign->Length; i++)
            printf("%x\t", sign->Data[i]);
        printf("\n");
    }
    uint16_t tx_length = (uint16_t)(Plen + MIM_HAND_DATAFDSTART + 4);
    packet = PacketEncoder(sign);
    console.AddLog("[DEBUG]##Send Packet of Ptype : %u. Check Terminal Log.", Ptype);
    

    switch(Ptype){
        //Ptype 1 : Ping
        case MIM_PT_PING : {
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            if((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->ax100_node))) != NULL && State.uplink_mode)
            {
                packetsign * confirm = PacketDecoder(confirm_);
                if(State.Debugmode)
                {
                    for(int i = 0; i < confirm->Length; i++)
                        printf("%d\t", confirm->Data[i]);
                    printf("\n");
                }
                

                uint16_t * retcode = (uint16_t *)&confirm->Data[0];
                uint16_t * pingcount = (uint16_t *)&confirm->Data[2];

                if(*retcode == 0)
                {
                    console.AddLog("[OK]##[OBC]Ping Success. retcode : %u, pingcount : %u", *retcode, *pingcount);
                    PingCounter ++;
                }
                else
                    console.AddLog("[ERROR]##[OBC]Ping Received but brocken. retcode : %u, pingcount : %u", *retcode, *pingcount);
                if(confirm != NULL)
                {
                    free(confirm);
                    confirm = NULL;
                }
            }
            else
                console.AddLog("[ERROR]##[OBC]Ping Failed.");
            
            if(confirm_!=NULL)
            {
                csp_buffer_free(confirm_);
                confirm_ = NULL;
            }
            break;
        }
        //Ptype 2 : Uplink Signal
        case MIM_PT_SIGNAL : {
            if(State.Debugmode)
            {
                for(int i = 0; i < packet->length; i++)
                {
                    printf("%x", packet[i]);
                }
                printf("\n");
            }
            
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                break;
            }
            console.AddLog("Uplink Signal TX Done.");
            //Ping Timeout : 10 sec
            break;
        }
        case MIM_PT_DLREQ : {
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            console.AddLog("Request FTP filelist.");
            if((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->ax100_node))) != NULL && State.uplink_mode)
            {
                filelist * confirm = (filelist *)malloc(confirm_->length);
                packetsign* confirmlist =  PacketDecoder(confirm_);
                memcpy(confirm, confirmlist->Data, confirmlist->Length);
                if(confirm->filenum > 0)
                {
                    for(int i = 0; i < 64; i++)
                    {
                        memset(&State.ftplistup[i], 0, sizeof(ftpinfo));
                    }
                    console.AddLog("[OK]##Received File list.");
                    for(int i = 0; i < 64; i++)
                    {
                        
                        if(i < confirm->filenum)
                        {
                            console.AddLog("[DEBUG]##Make Lists : %d\tFileName: %u", i, confirm->file[i]);
                            memcpy(&State.ftplistup[i], filelisthandler(confirm, filetype, filestatus, i), sizeof(ftpinfo));
                        }
                        else
                            continue;
                    }
                }
                else if(confirm->filenum == 0)
                    console.AddLog("[ERROR]##No more files.");
                else   
                    console.AddLog("[ERROR]##IO Error.");
                if(confirm!=NULL)
                {
                    free(confirm);
                    confirm = NULL;
                    sign = NULL;
                }
                    
            }
            else
            {
                console.AddLog("[ERROR]##Cannot Receive File List.");
            }
            //Ping Timeout : 10 sec
            if(confirm_!=NULL)
            {
                csp_buffer_free(confirm_);
                confirm_ = NULL;
            }
            break;
        }
        case MIM_PT_CMD : { //according to Ptype
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            if((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->ax100_node))) != NULL)
            {
                packetsign* confirmlist =  PacketDecoder(confirm_);
                uint8_t * retcode = (uint8_t *)&confirmlist->Data[0];
                console.AddLog("[OK]Received Command Reply. Retcode %d CMD Count %d", *retcode, State.CMDCount);
                State.CMDCount ++;
                if(confirmlist!=NULL)
                {
                    free(confirmlist);
                    confirmlist = NULL;
                    sign = NULL;
                } 
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
            }
            else
                console.AddLog("[ERROR]##No Command Reply.");
            break;
        }
        case MIM_PT_TMTC_TEST : { // == 10. Add for TMTC Test
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, 3, 13, MIM_DEFAULT_TIMEOUT, 0)) == NULL) {
                /*!!!!!!!!!!!Revise setup->obc_node!!!!!!!!!!*/ //
                // setup->
                /*!!!!!!!!!!!!Need to revise Port!!!!!!!!!!!*/
                    continue;
                }
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                console.AddLog("[OK]CMD Packet Header: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
                    packet->data[0],packet->data[1],packet->data[2],packet->data[3],packet->data[4],packet->data[5],packet->data[6],packet->data[7]);
                fprintf(log_ptr, "Uplink packet. Length: %d\n",packet->length);
                    for(int i=0; i<packet->length; i++) {
                        if(!(i%10) && i !=0) {
                            fprintf(log_ptr, "\n");
                        }
                        fprintf(log_ptr, "%02hhx\t",packet->data[i]);
                    } fprintf(log_ptr,"\n\n");
                if(csp_send(txconn, packet, setup->default_timeout)) // Success. then,
                {   
                    packet = NULL; // discard packet and,
                    break; // End process.
                }
                else // Fail. then,
                    continue; //Go to loof, and try again.
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            

            // // Reply(added for TCTM TEST), GS
            // if ((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->gs100_node))) != NULL)
            // {
            //     console.AddLog("[OK]Received Command Reply. Length = %d", confirm_->length);

            //     char hexstr[confirm_->length * 3 + 1]; // 각 바이트당 2자리 + 공백 + 널종료
            //     for (int i = 0; i < confirm_->length; i++) {
            //         sprintf(&hexstr[i * 3], "%02X ", confirm_->data[i]);
            //     }
            //     hexstr[confirm_->length * 3] = '\0';

            //     console.AddLog("[HEX] %s", hexstr);

            //     csp_buffer_free(confirm_);
            //     confirm_ = NULL;
            // }
            // else {
            //     console.AddLog("[ERROR]##No Command Reply.");
            // }










            // if((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->ax100_node))) != NULL)
            // {
            //     packetsign* confirmlist =  PacketDecoder(confirm_);
            //     uint8_t * retcode = (uint8_t *)&confirmlist->Data[0];
            //     console.AddLog("[OK]Received Command Reply. Retcode %d CMD Count %d", *retcode, State.CMDCount);
            //     State.CMDCount ++;
            //     if(confirmlist!=NULL)
            //     {
            //         free(confirmlist);
            //         confirmlist = NULL;
            //         sign = NULL;
            //     } 
            //     if(confirm_!=NULL)
            //     {
            //         csp_buffer_free(confirm_);
            //         confirm_ = NULL;
            //     }
            // }
            // else
            //     console.AddLog("[ERROR]##No Command Reply.");






            break;
        }
        case MIM_PT_STCMD : {
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            if((confirm_ = csp_read(txconn, rx_delay_ms(Plen, setup->ax100_node))) != NULL && State.uplink_mode)
            {
                if(State.uplink_mode)
                {
                    packetsign* confirmlist =  PacketDecoder(confirm_);
                    uint8_t * retcode = (uint8_t *)&confirmlist->Data[0];
                    console.AddLog("[OK]##Get Command Reply. Retcode : %d", *retcode);
                    if(confirmlist!=NULL)
                    {
                        free(confirmlist);
                        confirmlist = NULL;
                        sign = NULL;
                    } 
                    if(confirm_!=NULL)
                    {
                        csp_buffer_free(confirm_);
                        confirm_ = NULL;
                    }
                }
            }
            else
                console.AddLog("[ERROR]##Cannot Receive Command Reply.");
            break;
        }
        case MIM_PT_NCCMD : {
            csp_packet_t * confirm_ = (csp_packet_t *)csp_buffer_get(MIM_LEN_PACKET);
            packetsign* confirmlist =  PacketDecoder(confirm_);
            while(State.uplink_mode)
            {
                if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, TX_PORT, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
                    continue;
                else
                    break;
            }
            while (State.uplink_mode && txconn != NULL)
            {
                if(csp_send(txconn, packet, setup->default_timeout))
                {
                    packet = NULL;
                    break;
                }
                else
                    continue;
            }
            if(txconn == NULL)
            {
                console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
                if(confirm_!=NULL)
                {
                    csp_buffer_free(confirm_);
                    confirm_ = NULL;
                }
                break;
            }
            if((confirm_ = csp_read(txconn,rx_delay_ms(Plen, setup->ax100_node))) != NULL)
            {
                if(State.uplink_mode)
                {
                    console.AddLog("[OK]##Get Command Reply. No Checksum.");
                    if(confirmlist!=NULL)
                    {
                        free(confirmlist);
                        confirmlist = NULL;
                        sign = NULL;
                    } 
                    if(confirm_!=NULL)
                    {
                        csp_buffer_free(confirm_);
                        confirm_ = NULL;
                    }
                }
            }
            else
                console.AddLog("[ERROR]##Cannot Receive Command Reply.");
            break;
        }
        default : {
            console.AddLog("[ERROR]##Undefied type of uplink packetsign. Type : %"PRIu16, Ptype);
            break;
        }
    }
    
    if(txconn != NULL)
    {
        csp_close(txconn);
        txconn = NULL;
    }
    
    if(packet!=NULL)
    {
        csp_buffer_free(packet);
        packet = NULL;
    }
    time = csp_get_ms() - start;
    console.AddLog("[DEBUG]##Communication Time : %u", time);

    State.uplink_mode = false;
    if(dlstate)
    {
        State.downlink_mode = dlstate;
    }
    State.downlink_mode = true;
    pthread_mutex_unlock(&conn_lock);
}

int PacketHandler(csp_packet_t *packet, int type, int NowCursor)
{
    while(1) {
        switch(type) {
            case 0: {
                type = packet->data[NowCursor];
                NowCursor ++;
                return PacketHandler(packet, type, NowCursor);
            }
            case 1: {
                for (HKbufCursor; HKbufCursor < sizeof(HKbuf); HKbufCursor ++)
                {
                    HKbuf[HKbufCursor] = packet->data[NowCursor];
                    NowCursor ++;
                    if (NowCursor > MIM_LEN_PACKET)
                    {
                        //Read All Packet
                        //Need to go next packet
                        NowCursor = 0;
                        return 0;
                    }
                }
                NowHK = (HK *)HKbuf;
                buf_allclear();
                return NowCursor;
            }
            case 3: {
                // for (AODbufCursor; AODbufCursor < sizeof(AODbuf); AODbufCursor ++)
                // {
                //     AODbuf[AODbufCursor] = packet->data[NowCursor];
                //     NowCursor ++;
                //     if (NowCursor > MIM_LEN_PACKET)
                //     {
                //         //Read All Packet
                //         //Need to go next packet
                //         NowCursor = 0;
                //         return 0;
                //     }
                // }
                NowAOD = (AOD *)HKbuf;
                buf_allclear();
                return NowCursor;
            }
            default :{
                
            }
        }
    }
}

packetsign * PingInit(FSWTle * FSWTleinfo)
{
    packetsign * sign;
    sign = (packetsign *)malloc(sizeof(FSWTle) + MIM_HAND_DATAFDSTART);
    sign->Identifier = MIM_ID;
    sign->PacketType = MIM_PT_PING;
    sign->Length = sizeof(FSWTle);
    // printf("Start. Length : %u, Second : %u\n", sign->Length, FSWTleinfo->sec);
    memcpy(sign->Data, (void *)FSWTleinfo, sizeof(FSWTle));

    if(FSWTleinfo != NULL)
    {
        // free(FSWTleinfo);
        FSWTleinfo = NULL;
    }
    

    return sign;
}

csp_packet_t * PacketEncoder(packetsign * sign,bool freeer)
{
    if(sign == NULL)
    {
        console.AddLog("[DEBUG]##NULL POINTER GIVEN TO PACKETENCODER");
        return NULL;
    }
    uint32_t len = sign->Length;
    csp_packet_t *packet = (csp_packet_t *)csp_buffer_get(len);    
    packet->length = len;
    printf("packet len: %u\n", packet->length);
    memcpy(packet->data, sign->Data, len);
    // uint32_t packetsignlen = sign->Length +4; // Data size + Identifier(uint16) + PacketType(uint16)
    // csp_packet_t * packet = (csp_packet_t *)csp_buffer_get(packetsignlen);
    // packet->length = packetsignlen;
    // console.AddLog("[DEBUG]##Encoding Pakcets...Lnegth : %u", packetsignlen);
    // //Copy Header
    // memcpy((packet->data), sign, 4);
    // memcpy((packet->data) + 4, sign->Data, packetsignlen-4 ); // copy data
    for (int i =0; i < packet->length; i++) {
        printf("0x%x ", packet->data[i]);
    }printf("\n");
    if(freeer)
        sign = NULL;
    return packet;
}

packetsign * PacketDecoder(csp_packet_t * packet)
{
    uint32_t packetsignlen = packet->length +4;
    packetsign * sign = (packetsign *)malloc(packetsignlen);
    memcpy(sign, (packet->data), 4);
    sign->Length = packet->length-4;
    memcpy(sign->Data, packet->data + 4, sign->Length);
    return sign;
}

int CmdGenerator_GS::Scheduling(uint32_t ExecutionTime, uint32_t ExecutionWindow, uint32_t EntryID, uint16_t GroupID)
{
    uint16_t CMDlen = this->GetSize() + 12;
    Book * container = (Book *)malloc(CMDlen);
    memset(container, 0, CMDlen);
    container->ExTime = ExecutionTime;
    container->ExWindow = ExecutionWindow;
    container->EntryID = EntryID;
    container->GroupID = GroupID;
    this->CopyCmdHeaderToBuffer(container->cmd);
    this->GenerateCmdHeader(TS_CMD_MID, TS_INSERT_SCHEDULE_ENTRY_CC, sizeof(CFE_MSG_CommandHeader) + CMDlen, (void *)container);
    
}

packetsign * CmdGenerator_GS::GenerateCMDPacket(void)
{
    packetsign * ResultPacket = (packetsign * )malloc(this->GetSize() + MIM_HAND_DATAFDSTART);
    ResultPacket->Identifier = MIM_ID;
    if(this->Scheduled)
        ResultPacket->PacketType = MIM_PT_STCMD;
    else if(!this->Checksum)
        ResultPacket->PacketType = MIM_PT_NCCMD;
    else
        ResultPacket->PacketType = MIM_PT_CMD;
    ResultPacket->Length = this->GetSize();
    console.AddLog("[DEBUG]##Packetsign Length : %u", ResultPacket->Length);
    memcpy(ResultPacket->Data, this->CmdHeader, ResultPacket->Length);
    return ResultPacket;
}

void * Direct_Shell(void * data)
{
    pthread_mutex_lock(&conn_lock);
    while(!State.RotatorReadReady)
        continue;
    bool dlstate = true;


    //This funcion must be on p_thread[4]
    if((dlstate))
    {
        State.downlink_mode = false;
    }
        
    while(!State.uplink_mode)
        continue;

    cmd_packet_t * ResultCMD = (cmd_packet_t *)data;
    reply_packet_t * ResultReply = new reply_packet_t;
    csp_conn_t * txconn;
    csp_packet_t * packet = (csp_packet_t *)csp_buffer_get(sizeof(cmd_packet_t));
    csp_packet_t * confirm = (csp_packet_t *)csp_buffer_get(sizeof(reply_packet_t));

    uint32_t start, time = 0;
    start = csp_get_ms();

    memcpy(packet->data, ResultCMD, sizeof(cmd_packet_t));
    packet->length = sizeof(cmd_packet_t);

    if(State.Debugmode)
    {
        printf("Direct Shell Data : ");
        for(int i = 0; i < sizeof(cmd_packet_t); i++)
            printf("%u\t", ((uint8_t *)packet->data)[i]);
        printf("\n");
    }


    while(State.uplink_mode)
    {
        if ((txconn = csp_connect(CSP_PRIO_HIGH, setup->obc_node, 27, MIM_DEFAULT_TIMEOUT, 0)) == NULL)
            continue;
        else
            break;
    }
    while (State.uplink_mode && txconn != NULL)
    {
        if(csp_send(txconn, packet, setup->default_timeout *2))
        {
            packet = NULL;
            break;
        }
        else
            continue;
    }
    if(txconn == NULL)
    {
        console.AddLog("[ERROR]##Connection Buffer Busy. Skip this command.");
        if(confirm!=NULL)
        {
            csp_buffer_free(confirm);
            confirm = NULL;
        }
    }
    if((confirm = csp_read(txconn, 2 *rx_delay_ms(sizeof(MIM_LEN_PACKET), setup->ax100_node))) != NULL && State.uplink_mode)
    {
        if(State.uplink_mode)
        {
            memcpy(ResultReply, confirm->data, confirm->length);
            console.AddLog("[OK]## Received Shell Reply. Type : %"PRIu8", Result : %"PRIu8", Retcode : %d", ResultReply->type, ResultReply->result, ResultReply->retcode);
            if(confirm!=NULL)
            {
                csp_buffer_free(confirm);
                confirm = NULL;
            }
        }
    }
    else
        console.AddLog("[ERROR]##Cannot Receive Shell Reply.");
    
    if(txconn != NULL)
    {
        csp_close(txconn);
        txconn = NULL;
    }
    
    if(packet!=NULL)
    {
        csp_buffer_free(packet);
        packet = NULL;
    }
    time = csp_get_ms() - start;
    console.AddLog("[DEBUG]##Communication Time : %u", time);

    State.uplink_mode = false;
    if(dlstate)
    {
        State.downlink_mode = dlstate;
    }
    pthread_mutex_unlock(&conn_lock);
}