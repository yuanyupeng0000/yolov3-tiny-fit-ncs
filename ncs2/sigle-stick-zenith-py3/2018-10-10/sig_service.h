/*
 * sig_service.h
 *
 *  Created on: 2016??6??22??
 *      Author: Administrator
 */

#ifndef INCLUDE_SIG_SERVICE_H_
#define INCLUDE_SIG_SERVICE_H_
#include "g_define.h"


#include <pthread.h>
#pragma pack(push)
#pragma pack(1)

//
//#define  VERSION    0x01     //?∑⁄????
//#define  PROTTYPE    0x02         //ßø???∑⁄
//
////????ßø??????
//#define  SETCAMPARAM    0x0001      //????????????????
//#define  GETCAMPARAM    0x0002		  //???????????????
//#define  REPCAMPARAM    0x0003	  //??????????????????
//
//#define  SETDETECTDEVICE    0x0004    //???®π????ıÙ????????
//#define  GETDETECTDEVICE    0x0005    //????????ıÙ????????
//#define  REPDETECTDEVICE    0x0006    //????????ıÙ???????????
//
//#define  SETCHECHPARAM    0x0007    //??????????????
//#define  GETCHECHPARAM    0x0008    //?????????????
//#define  REPCHECHPARAM    0x0009    //????????????????
//
//#define  HEART    0x1000    //??????????ßø??
//#define  SHUTDOWN    0x1001    //???????
//#define  REPHEART    0x1002    //??????????ßø?????
//
//#define  REBOOTZEN   0x2001   //????????
//
//#define  FORKEXIT    0x3001   //?????????
//
//#define   HEARTTIME     60
//
//#define  USERNAMEMAX   20
//#define  IPADDRMAX     16
//#define  DEVNAMEMAX    50
//
////????????????
//#define Z_NONE 0x0
//#define Z_NORTH 0x1
//#define Z_NORTHEAST 0x2
//#define Z_EAST 0x3
//#define Z_SOUTHEAST 0x4
//#define Z_SOUTH 0x5
//#define Z_SOUTHWEST 0x6
//#define Z_WEST 0x7
//#define Z_NORTHWEST 0x8
//
//#define  COILPOINTMAX       4      //??????????
//#define  DETECTLANENUMMAX   4      //???????????
//#define  LANELINEMAX         8         //?????????
//#define  STANDPOINT          8        //????8??????
//#define  STANDARDVAULEMAX    4
//#define  ALGMAX               2    //??????????
//#define  MAXSUBSYSTEM         4
//
////#define  CAM_NUM           4    //????????????????
//
//typedef struct Command{
//	unsigned char version;       //?∑⁄????VERSION
//	unsigned char prottype;      //ßø???∑⁄ PROTTYPE
//	unsigned short objnumber;      //????????. ?°¿?index
//	unsigned short objtype;      //????????   ????: SETCAMPARAM
//	unsigned int objlen;         //???????????
//}mCommand; //????ßø???
//
//typedef struct{
//    unsigned short x;
//    unsigned short y;
//}IVDPoint;
///*-----------------??????????-----------------*/
//typedef struct{
//    unsigned char   state;                    //??????   //0????  1??
//    unsigned char   isCarInTail;              //??????????ß‘?, 0????, 1??
//    unsigned short  queueLength;              //????????
//    unsigned int    vehnum;                   //????????
//    //unsigned int    speed; 					  //?????????
//    unsigned short  uActualDetectLength;       //????????????  //?????
//    unsigned short  uActualTailLength;			//????????????  //????
//    IVDPoint        LineUp[2];                //?????????????? ??????????
//}mRealLaneInfo;
//
//typedef struct{
//    unsigned char   flag;          //??????0xFF
//    unsigned char   laneNum;       //??????????
//    unsigned char   curstatus;      //  1 ?????, 2 ?????
//    unsigned char   fuzzyflag;                //???????
//    unsigned char   visibility;		           //???????
//    unsigned short 	uDegreePoint[20][2];      //????????????. 0:x 1:y
//    //unsigned short 	uDegreePoint[4][2];      //????????????. 0:x 1:y
//    mRealLaneInfo   lane[DETECTLANENUMMAX];  //16
//}mRealStaticInfo;
//
//#if 0
////-----------------?????????????------------------
//typedef struct CammerControl{
//	unsigned char start;                    //??????????????   ????0x1 ??????0x0
//	unsigned char camdirect;              //??????????¶À
//	unsigned char cammerIp[IPADDRMAX];    //????IP???
//}mCammerControl;
//
//typedef struct DetectDeviceConfig{
//	unsigned int  detectport;
//	unsigned char camnum;
//	unsigned char detectip[IPADDRMAX];
//	unsigned char detectname[DEVNAMEMAX];
//	mCammerControl  camcontrol[CAM_NUM];
//}mDetectDeviceConfig;   //???????ıÙ????,????4??????????,
//#endif
//typedef struct DetectDeviceConfig{
//	unsigned int  deviceID;   //??????ID
//	unsigned int  detectport;
//	unsigned char camnum;
//	unsigned char mode;       //???????????   1 ???, 0??????????????????
//	//unsigned char camdirect1;              //??????????¶À
//	//unsigned char camdirect2;              //??????????¶À
//	//unsigned char camdirect3;              //??????????¶À
//	//unsigned char camdirect4;              //??????????¶À
//	//unsigned char cammerIp1[IPADDRMAX];    //????IP???
//	//unsigned char cammerIp2[IPADDRMAX];    //????IP???
//	//unsigned char cammerIp3[IPADDRMAX];    //????IP???
//	//unsigned char cammerIp4[IPADDRMAX];    //????IP???
//
//	unsigned char camstatus[CAM_NUM];
//	unsigned char camdirect[CAM_NUM];
//	unsigned char cammerIp[CAM_NUM][IPADDRMAX];
//
//	unsigned char detectip[IPADDRMAX];
//	unsigned char detectname[DEVNAMEMAX];
//}mDetectDeviceConfig;   //???????ıÙ????,????4??????????,
//
////-----------------????????----???????????????????-------------
//typedef struct CamAttributes{
//	unsigned char direction;
//	unsigned int  camID;      //??????ID
//	unsigned int  cammerport;
//	unsigned int  adjustport;
//	unsigned int  signalport;
//	unsigned char urlname[USERNAMEMAX];  //???????????????
//	unsigned char username[USERNAMEMAX];
//	unsigned char passwd[USERNAMEMAX];
//	unsigned char cammerIp[IPADDRMAX];
//	unsigned char adjustIp[IPADDRMAX];
//	unsigned char signalIp[IPADDRMAX];
//}mCamAttributes; //??????????
//
//typedef struct CamDemarcateParam{
//	unsigned short cam2stop;
//	unsigned short camheight;
//	unsigned short lannum;    //??????
//	unsigned short number;    //???????
//	unsigned short baselinelen;
//	unsigned short farth2stop;
//	unsigned short recent2stop;
//}mCamDemarcateParam; //??????????
//
//typedef struct ChannelVirtualcoil{
//	unsigned short number;      //???????
//	unsigned short farthCoillen;
//	unsigned short recentCoillen;
//}mChannelVirtualcoil; //???????????????
//
//typedef struct CamParam{
//	unsigned char coilnum;     //?????
//	mCamAttributes camattr;
//	mCamDemarcateParam camdem;
//	mChannelVirtualcoil channelcoil[DETECTLANENUMMAX];
//}mCamParam;

//---------------???????------????????????????????---------
//typedef struct Point{
//    unsigned short x;
//    unsigned short y;
//}mPoint;//??????
//
//typedef struct Line{
//    unsigned short startx;
//    unsigned short starty;
//    unsigned short endx;
//    unsigned short endy;
//}mLine; //??????
//
//typedef struct RearCoil{
//	mPoint RearCoil[COILPOINTMAX];  //?¶À???????
//	mPoint FrontCoil[COILPOINTMAX]; //??????
//}mChannelCoil;  //??????????????¶À??
//
//typedef struct CamDetectLane{
//	unsigned char lanenum;                 //??????
//	mChannelCoil virtuallane[DETECTLANENUMMAX];
//}mCamDetectLane;   //????????????????????, ?????????????????????? //???????????????????
//
//typedef struct VirtualLaneLine{
//	unsigned char lanelinenum;         //
//	mLine         laneline[LANELINEMAX];
//}mVirtualLaneLine;    //???????? ???????4??????5????
//
//typedef struct StandardPoint{
//	mPoint  coordinate;
//	unsigned short value;
//}mStandardPoint; //?????????????
//
//typedef struct DemDetectArea{
//	mPoint  vircoordinate[STANDPOINT];
//	mPoint  realcoordinate[STANDPOINT];
//}mDemDetectArea;  // ???????8????,?????????????????
//
//typedef struct DetectParam{
//	unsigned short uTransFactor;
//	unsigned int   uGraySubThreshold;
//	unsigned int   uSpeedCounterChangedThreshold;
//	unsigned int   uSpeedCounterChangedThreshold1;
//	unsigned int   uSpeedCounterChangedThreshold2;
//	unsigned short  uDayNightJudgeMinContiuFrame;//?ß›??????????
//	unsigned short  uComprehensiveSens;//??????????????
//	unsigned short  uDetectSens1;//?ßÿ?????????ß≥????
//	unsigned short  uDetectSens2;
//	unsigned short  uStatisticsSens1;
//	unsigned short  uStatisticsSens2;	//by david 20130910 from tagCfgs
//	unsigned short  uSobelThreshold;//sobel???
//	unsigned short  shutterMax;        // 1 2 3 4 5 6 7 8
//	unsigned short  shutterMin;        // 1 2 3 4 5 6 7 8
//}mDetectParam;
//
//typedef struct CamDetectParam{
//	unsigned int   timep[4]; //0 1?????Ÿí ??1????2   2 3???? ??? ??1????2
//	mCamDetectLane detectlane;        //????????
//	mVirtualLaneLine  laneline;      //??????????
//	mStandardPoint standpoint[STANDARDVAULEMAX];       //??????????
//	mDemDetectArea area;              //????????
//	mDetectParam detectparam[ALGMAX];   // 0  ?????????,  1???????????
//}mCamDetectParam;  //?????????ßﬁ????ß€????

//????????
enum LINKID {
    LINKID_ETH1=1,
    LINKID_ETH2,
    LINKID_UART1,
    LINKID_UART2,
    LINKID_UART3,
    LINKID_UART4,
    LINKID_UART5,
    LINKID_CAN,
    LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD,
    LINKID_BROARDCAST=0xFF
};
//ßø??????
enum PROTOCOLTYPE {
    PROTOCOTYPE_VIDEO_DETECTOR=0x01,
    PROTOCOTYPE_MAX
};
//?ıÙ??
enum DEVICECLASS {
    DEVICECLASS_VIDEO_SEPERATED_MACHINE=0x01,
    DEVICECLASS_IPCAMERAL,
    DEVICECLASS_VIDEO_INTEGRATED_MACHINE,
    DEVICECLASS_MAIN_CONTROL_BOARD,
    DEVICECLASS_COMMUNICATION_BOARD,
    DEVICECLASS_BROADCAST=0x0F,
};
//?????????
typedef struct {
    unsigned char mHeaderTag;
    unsigned char mSenderLinkId;
    unsigned char mRecieverLinkId;
    unsigned char mProtocolType;
    unsigned char mProtocolVersion;
    struct {
        unsigned char mDeviceIndex:4;
        unsigned char mDeviceClass:4;
    }mSenderDeviceID,mRecieverDeviceId;
    unsigned char mSessionID;
    unsigned char mDataType;
    //unsigned char pContent[1];
} FrameHeader;
//????¶¬????
typedef struct {
    unsigned char mXor;
    unsigned char mTailTag;
}FrameTail;
///////////////////////////////////////////////// //?????????/////////////
typedef struct EachChannelPack {
    unsigned char mDetectChannelIndex;
    unsigned char mQueueLength;
    unsigned char mRealTimeSingleSpeed;
    struct {
        unsigned char bOccupyStatus0:1;
        unsigned char bOccupyStatus1:1;
        unsigned char bFix0:2;
        unsigned char flow:2;
        unsigned char bVehicleType:2;
    }mDetectDataOfHeaderVirtualCoil;
    unsigned char mHeadwayTimeOfHeaderVirtualCoil;
    unsigned char mOccupancyOfHeaderVirtualCoil;
    struct {
        unsigned char bOccupyStatus0:1;
        unsigned char bOccupyStatus1:1;
        unsigned char bFix0:6;
    }mDetectDataOfTailVirtualCoil;
    unsigned char mHeadwayTimeOfTailVirtualCoil;
    unsigned char mOccupancyOfTailVirtualCoil;
    struct {
        unsigned char bFix0:7;
        unsigned char bDataIsValid:1;
    }mWorkStatusOfDetectChannle;
}EachChannelPackm;

typedef struct {
    unsigned char mDetectChannelCount;
    EachChannelPackm EachChannelPack[1];
}RealTimeTrafficData;

/////// //?ıÙ????????????? ////////////??????????? ///////////////////////////////////////////
//?ıÙ???????????? /////////////////////////////////////////

typedef	struct{
    struct {
        unsigned char mDeviceIndex:4;
        unsigned char mDeviceClass:4;
    }mCameralDeviceId;
    struct {
        unsigned char bNorth:1;
        unsigned char bEastNorth:1;
        unsigned char bEast:1;
        unsigned char bEastSouth:1;
        unsigned char bSouth:1;
        unsigned char bWestSouth:1;
        unsigned char bWest:1;
        unsigned char bWestNorth:1;
    }mCameralPosition;
    struct {
        unsigned char bWorkMode:2;
        unsigned char bBackgroundRefreshed:1;
        unsigned char bH264DecodeStatus:1;
        unsigned char bCameralOnLine:1;
        unsigned char bPictureStable:1;
        unsigned char bFix0:2;
    }mCameralStatus;
}EachCameralStatus;

typedef struct {
    struct {
        unsigned char mDeviceIndex:4;
        unsigned char mDeviceClass:4;
    }mDetectMainMachineDeviceId;
    union {
        struct {
            unsigned char bRegisted:1;
            unsigned char bTimeCorrected:1;
            unsigned char bFix0:6;
        }mStatusWhenSeperatedMachine;
        unsigned char mFix0WhenIntegratedMachine;
    }uDetectMainMachineStatus;
    unsigned char mCameralCount;
    EachCameralStatus mEachCameralStatus[1];
}DeviceWorkStatusQueryResponse;

//////////////////////////////////////// //????????? ////////
typedef struct {
    unsigned int mUTCTime;
}TimeBroadcastCommand;

///////////////////// //????????? /////////
typedef struct {
    unsigned char mIsRushHour;
}RushHourBroadcastCommand;

////////////////////// //??????????????? /////////////////// //???????????
/////////// //?????????????? ///
typedef struct {
    struct {
        unsigned short mListenPort; unsigned char mIp[4];
        unsigned char mNetmask[4]; unsigned char mGateway[4];
    }mEthernet[2];
    struct {
        unsigned char mWorkElectricLevel;
        unsigned char mBaudRate;
        unsigned char mDataBits;
        unsigned char mStopBits;
        unsigned char mParityBit;
    }mUart[5];
    unsigned char mCanBaudRate;
}CommunicationBoardArgumentQueryResponse;

///////////////////// //???????????????? ////////////
typedef struct {
    struct {
        unsigned short mListenPort;
        unsigned char mIp[4];
        unsigned char mNetmask[4];
        unsigned char mGateway[4];
    }mEthernet[2];
    struct {
        unsigned char mWorkElectricLevel;
        unsigned char mBaudRate;
        unsigned char mDataBits;
        unsigned char mStopBits;
        unsigned char mParityBit;
    }mUart[5];
    unsigned char mCanBaudRate;
}CommunicationBoardArgumentSetUpCommand;

///////////////////////////////// ////??????????????? ///////////////////////////
typedef struct {
    unsigned char mResult;
}CommunicationBoardArgumentSetUpResponse;
//////////////////////////////////////////////////////////////
//?ıÙ??????????????
typedef struct{
    struct {
        unsigned char mDeviceIndex:4;
        unsigned char mDeviceClass:4;
    }mEventDeviceId;
    union {
        struct { unsigned char bNorth:1;
                 unsigned char bEastNorth:1;
                      unsigned char bEast:1;
                           unsigned char bEastSouth:1;
                                unsigned char bSouth:1;
                                     unsigned char bWestSouth:1;
                                          unsigned char bWest:1;
                                               unsigned char bWestNorth:1;
               }mEventOccourPositionWhenCameralOrIntegratedCameral;
        unsigned char mFix0WhenSeperatedMachine;
    }uEventOccourPosition;
    unsigned char mEvent;
    unsigned char mReserverFix0;
    unsigned int mEventTime;
}DeviceEventsAutoReporte;

#define CHANNELMAXNUM 4
typedef struct car_info {
    int g_flow[4] ={0};
    int g_50frame1[4]={0};
    int g_50frame2[4]={0};
    int g_50frametail1[4]={0};
    int g_50frametail2[4]={0};
    int g_occupancyframe[4]={0};
    int g_occupancyframetail[4]={0};
} m_car_info;
typedef struct staticchannel{
    //unsigned char index;
    unsigned char status;   //??????????ßπ??, 0 ?????, ????????, 1???????, 2??????????????.
    //unsigned char mode;
    //unsigned char black;
    //unsigned char algswitch;
    EachCameralStatus EachStatus;//Camera status
    EachChannelPackm Eachchannel[CHANNELMAXNUM];//This is what signal machine wants
    int camera_state_change=0;
    int lane_num;
    m_car_info car_info;
    //unsigned int  frameNum;
}m_sig_data;
extern m_sig_data * get_sig_data(int index);
/*
 * v1.2
 ?????????????
1.??????????
2.?????????????????????????ßÿ?????
3.???????????
4,??????????????????????????????????????????????????????
 *
 *

????????????
1.?????????????????????
2.??????????
3.?????????
4.???????
 *
 * */


void reset_sig_machine();
void init_sig_service();void init_sig_service1();void init_sig_service2();
//void submit_sig_data(m_sig_data *p_channel_rst,int index);

extern m_sig_data * get_locked_sig_data(int index);
extern void submit_unlock_sig_data(int index);
void reboot_cmd();
void reset_sig_machine();

#define NANJING_CAM_NUM 4
typedef struct queue_info{
    unsigned char table_head[5];
    unsigned char table_no;
    unsigned char table_length;
    unsigned char detect_time[6];
    unsigned char detect_status;//whether device good or not
    unsigned char dir_no;//camera direction
    unsigned char lane_dir_type;//income or out come
    unsigned char queue_len[8];//---------
    unsigned char queue_start_pos[8];
    unsigned char queue_veh_num[8];//---------
    unsigned char veh_speed[8];//
    unsigned char crc;
}queue_info_t;


typedef struct radar_realtime{
    //unsigned char addr[2];
    unsigned char addr;
    unsigned char ver;
    unsigned char type;
    unsigned char obj;
}radar_head_t;

typedef struct radar_head{
    unsigned char time[4];
    unsigned char lane_count;
}radar_realtime_t;


typedef struct radar_realtime_lane{
    unsigned char lane_no;
    unsigned char queue_len;
    unsigned char head_len;
    unsigned char tail_len;
    unsigned char queue_no;
    unsigned char lane_vihicle_count;
    unsigned char ocuppy;
    unsigned char average_speed;
    unsigned char location;
    unsigned char head_pos;
    unsigned char head_speed;
    unsigned char tail_pos;
    unsigned char tail_speed;
}radar_realtime_lane_t;


typedef struct radar_result{
    unsigned char period[2];
    unsigned char time[4];
    unsigned char lane_count;

}radar_result_t;
#if 0
typedef struct radar_result_lane{
    unsigned char lane_no;
    unsigned char queue_len;
    unsigned char condition;
    unsigned char stop_count[4];
    unsigned char average_delay[4];
    unsigned char oil_use[4];
    unsigned char smoge[4];

}radar_result_lane_t;
#else
typedef struct radar_result_lane{
    unsigned char lane_no;
    unsigned char flowA;
    unsigned char flowB;
    unsigned char flowC;
    unsigned char flowSum;
    unsigned char Occupy_rate;
    unsigned char average_speed;
    unsigned char average_len;
    unsigned char average_head_time;
}radar_result_lane_t;
#endif

typedef struct radar_cycle_result_lane{  //ËΩ¶ÈÅìÂè∑ÂàíÂàÜÁöÑÁªüËÆ°Âë®ÊúüÊï∞ÊçÆ
    unsigned char lane_no;
    unsigned char queue_len_max;
    unsigned char lane_status;
    float car_stop_sum;
    float average_delay_ms;
    float oil_consume;
    float gas_emission; //Â∞æÊ∞îÊéíÊîæ
}radar_cycle_result_lane_t;

//
typedef struct radar_car_in_out_result {  //ËøõËΩ¶ÂíåÂá∫ËΩ¶ÈúÄË¶ÅÁªôÊåáÂÆöÁöÑudp ip
	unsigned char lane_no:5;
	unsigned char over_len_flag:1;
	unsigned char in_out_flag:2;
	
}radar_car_in_out_result_t;


typedef struct radar_rt_lane_car_in_out_item{
	unsigned char lane_no;
	unsigned char in_flag;
	unsigned char out_flag;
}radar_rt_lane_car_in_out_item_t;

typedef struct radar_rt_lane_car_in_out_info {
	radar_rt_lane_car_in_out_item_t lanes[MAX_LANE_NUM];
}radar_rt_lane_car_in_out_info_t;

//
typedef struct radar_car_in_out_status {  //‰∏ä‰∏ÄÊ¨°ËøõËΩ¶ÂíåÂá∫ËΩ¶Áä∂ÊÄÅ
	unsigned char flag;
}radar_car_in_out_status_t;

typedef struct flow_info{
    unsigned char table_head[5];
    unsigned char table_no;
    unsigned char table_length;
    unsigned char detect_time[20];
    unsigned char dir_no;
    unsigned char section_no;
    unsigned char flow[8];
    unsigned char average_speed[8];
    unsigned char ocuppy_percent[8];
    unsigned char crc;
}flow_info_t;

typedef struct outcar_info{
    unsigned char table_head[5];
    unsigned char table_no;
    unsigned char table_length;
    unsigned char pass_time[6];
    unsigned char dir_no;
    unsigned char lane_dir_type;
    unsigned char section_number;
    unsigned char lane_number;
    unsigned char veh_type;
    unsigned char veh_speed;
    unsigned int occupy_time;
    unsigned char crc;
}ourcar_info_t;
static int out_car_flag;
extern pthread_mutex_t out_car_lock;


typedef struct data_1s{
}data_1s_t;

#pragma pack(pop)

bool add_car_in_out_item(radar_rt_lane_car_in_out_info_t *io_item);


#endif /* INCLUDE_SIG_SERVICE_H_ */
