/*
 * protocol.h
 *
 *  Created on: 2016��8��17��
 *      Author: root
 */

#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "g_define.h"
#pragma pack(push)
#pragma pack(1)

#define  VERSION    0x01     //�汾����
#define  PROTTYPE    0x02         //Э��汾

//����Э������
#define  SETCAMPARAM    0x0001      //���������������
#define  GETCAMPARAM    0x0002		  //��ȡ�����������
#define  REPCAMPARAM    0x0003	  //��ȡ�������������Ӧ

#define  SETDETECTDEVICE    0x0004    //���ü���豸��������
#define  GETDETECTDEVICE    0x0005    //��ȡ����豸��������
#define  REPDETECTDEVICE    0x0006    //��ȡ����豸����������Ӧ

#define  SETCHECHPARAM    0x0007    //������Ƶ���߲���
#define  GETCHECHPARAM    0x0008    //��ȡ��Ƶ���߲���
#define  REPCHECHPARAM    0x0009    //��ȡ��Ƶ���߲�����Ӧ

#define  HEART    0x1000    //����������Э��
#define  SHUTDOWN    0x1001    //�ر�����
#define  REPHEART    0x1002    //����������Э���Ӧ

#define  REBOOTZEN   0x2001   //��������

#define  FORKEXIT    0x3001   //�����쳣�˳�


#define   HEARTTIME     60

#define  USERNAMEMAX   20
#define  IPADDRMAX     16
#define  DEVNAMEMAX    50

//����������
#define Z_NONE 0x0
#define Z_NORTH 0x1
#define Z_NORTHEAST 0x2
#define Z_EAST 0x3
#define Z_SOUTHEAST 0x4
#define Z_SOUTH 0x5
#define Z_SOUTHWEST 0x6
#define Z_WEST 0x7
#define Z_NORTHWEST 0x8

#define  COILPOINTMAX       4      //��Ȧ�ĸ�����
#define  DETECTLANENUMMAX   4      //���ͨ������
#define  LANELINEMAX         8         //�����߸���
#define  STANDPOINT          8        //�궨��8������
#define  STANDARDVAULEMAX    4
#define  ALGMAX               2    //�㷨��������
#define  MAXSUBSYSTEM         4

//#define  CAMMERNUM           4    //�������������

typedef struct Command{
	unsigned char version;       //�汾����VERSION
	unsigned char prottype;      //Э��汾 PROTTYPE
	unsigned short objnumber;      //��������. �±�index
	unsigned short objtype;      //��������   ����: SETCAMPARAM
	unsigned int objlen;         //������ĳ���
}mCommand; //����Э��ͷ

typedef struct{
    unsigned short x;
    unsigned short y;
}IVDPoint;
/*-----------------ʵʱ�������-----------------*/
typedef struct{
    unsigned char   state;                    //����״̬   //0����  1�복
    unsigned char   isCarInTail;              //ǰ����Ȧ�Ƿ��г�, 0����, 1�복
    unsigned short  queueLength;              //���ӵĳ���
    unsigned int    vehnum;                   //��������
    //unsigned int    speed; 					  //�������ٶ�
    unsigned short  uActualDetectLength;       //������Ȧ�ĳ���  //����Ȧ
    unsigned short  uActualTailLength;			//������Ȧ�ĳ���  //Զ��Ȧ
    IVDPoint        LineUp[2];                //��ǰ����ʵ������ ��ʼ����յ�
}mRealLaneInfo;
typedef struct{
    int x;
    int y;
    int w;
    int h;
}fvd_objects;
typedef struct{
    unsigned char   flag;          //���ݱ�־0xFF
    unsigned char   laneNum;       //ʵ�ʳ�������
    unsigned char   curstatus;      //  1 �ǰ���, 2 ��ҹ��
    unsigned char   fuzzyflag;                //��Ƶ�쳣״̬
    unsigned char   visibility;		           //�ܼ���״̬
    unsigned short 	uDegreePoint[20][2];      //���������. 0:x 1:y
    //unsigned short 	uDegreePoint[4][2];      //���������. 0:x 1:y
    mRealLaneInfo   lane[DETECTLANENUMMAX];  //16
    // nanjing....
    unsigned char   area_car_num[DETECTLANENUMMAX];//car amount
    unsigned char   queue_len[DETECTLANENUMMAX];// len
    IVDPoint queue_line[DETECTLANENUMMAX][2];
    int rcs_num;
    fvd_objects rcs[100];
}mRealStaticInfo;

#if 0
//-----------------�����ȫ������------------------
typedef struct CammerControl{
	unsigned char start;                    //����������ñ�־   ����0x1 ������0x0
	unsigned char camdirect;              //���������λ
	unsigned char cammerIp[IPADDRMAX];    //���IP��ַ
}mCammerControl;

typedef struct DetectDeviceConfig{
	unsigned int  detectport;
	unsigned char camnum;
	unsigned char detectip[IPADDRMAX];
	unsigned char detectname[DEVNAMEMAX];
	mCammerControl  camcontrol[CAMMERNUM];
}mDetectDeviceConfig;   //������豸����,����4��������,
#endif

#define CAM_CLOSED_STATUS 0
#define CAM_OPENED_STATUS 1
typedef struct caminfo {

	unsigned char camstatus;
	unsigned char camdirect;
	unsigned char cammerIp[IPADDRMAX];
} m_caminfo;

typedef struct DetectDeviceConfig{
	unsigned int  deviceID;   //�����ID
	unsigned int  detectport;
	unsigned char camnum;
//	unsigned char mode;       //�Ƿ�¼��ģʽ��־   1 ¼��, 0��¼��ģʽ��Ҳ�����ģʽ
	//unsigned char camdirect1;              //���������λ
	//unsigned char camdirect2;              //���������λ
	//unsigned char camdirect3;              //���������λ
	//unsigned char camdirect4;              //���������λ
	//unsigned char cammerIp1[IPADDRMAX];    //���IP��ַ
	//unsigned char cammerIp2[IPADDRMAX];    //���IP��ַ
	//unsigned char cammerIp3[IPADDRMAX];    //���IP��ַ
	//unsigned char cammerIp4[IPADDRMAX];    //���IP��ַ

//	unsigned char camstatus[CAMMERNUM];
//	unsigned char camdirect[CAMMERNUM];
//	unsigned char cammerIp[CAMMERNUM][IPADDRMAX];

	unsigned char detectip[IPADDRMAX];
	unsigned char detectname[DEVNAMEMAX];
	m_caminfo cam_info[CAM_NUM_1];

}mDetectDeviceConfig;   //������豸����,����4��������,

//-----------------��������----���������ز�������-------------
typedef struct CamAttributes{
	unsigned char direction;
	unsigned int  camID;      //�����ID
	unsigned int  cammerport;
	unsigned int  adjustport;
	unsigned int  signalport;
	unsigned char urlname[USERNAMEMAX];  //Ϊ���ʵ����ļ�����
	unsigned char username[USERNAMEMAX];
	unsigned char passwd[USERNAMEMAX];
	unsigned char cammerIp[IPADDRMAX];
	unsigned char adjustIp[IPADDRMAX];
	unsigned char signalIp[IPADDRMAX];
}mCamAttributes; //���������

typedef struct CamDemarcateParam{
	unsigned short cam2stop;
	unsigned short camheight;
	unsigned short lannum;    //������
	unsigned short number;    //ͨ�����
	unsigned short baselinelen;
	unsigned short farth2stop;
	unsigned short recent2stop;
}mCamDemarcateParam; //����궨����

typedef struct ChannelVirtualcoil{
	unsigned short number;      //ͨ�����
	unsigned short farthCoillen;
	unsigned short recentCoillen;
}mChannelVirtualcoil; //ͨ��������Ȧ�Ĳ���

typedef struct CamParam{
	unsigned char coilnum;     //ͨ����
	mCamAttributes camattr;
	mCamDemarcateParam camdem;
	mChannelVirtualcoil channelcoil[DETECTLANENUMMAX];
}mCamParam;

//---------------��Ȧ����------�����������������---------
typedef struct Point{
    unsigned short x;
    unsigned short y;
}mPoint;//������

typedef struct Line{
    unsigned short startx;
    unsigned short starty;
    unsigned short endx;
    unsigned short endy;
}mLine; //������

typedef struct RearCoil{
	mPoint RearCoil[COILPOINTMAX];  //ռλ�����Ȧ
	mPoint FrontCoil[COILPOINTMAX]; //ǰ����Ȧ
}mChannelCoil;  //��ͨ��������Ȧ��λ��

typedef struct CamDetectLane{
	unsigned char lanenum;                 //������
	mChannelCoil virtuallane[DETECTLANENUMMAX];
}mCamDetectLane;   //���������صĳ�������, ��������ͨ��������Ȧ //ÿһ����������������Ȧ

typedef struct VirtualLaneLine{
	unsigned char lanelinenum;         //
	mLine         laneline[LANELINEMAX];
}mVirtualLaneLine;    //���⳵���� ���֧��4������5����

typedef struct StandardPoint{
	mPoint  coordinate;
	unsigned short value;
}mStandardPoint; //�궨��������ֵ

typedef struct DemDetectArea{
	mPoint  vircoordinate[STANDPOINT];
	mPoint  realcoordinate[STANDPOINT];
}mDemDetectArea;  // �궨����ϵ8����,������ʵ�������

typedef struct DetectParam{
	unsigned short uTransFactor;
	unsigned int   uGraySubThreshold;
	unsigned int   uSpeedCounterChangedThreshold;
	unsigned int   uSpeedCounterChangedThreshold1;
	unsigned int   uSpeedCounterChangedThreshold2;
	unsigned short  uDayNightJudgeMinContiuFrame;//�л�ʱ��ֵ����ֵ
	unsigned short  uComprehensiveSens;//ȡ����������֡��
	unsigned short  uDetectSens1;//�ж��ǳ�ͷ����С����
	unsigned short  uDetectSens2;
	unsigned short  uStatisticsSens1;
	unsigned short  uStatisticsSens2;	//by david 20130910 from tagCfgs
	unsigned short  uSobelThreshold;//sobel��ֵ
	unsigned short  shutterMax;        // 1 2 3 4 5 6 7 8
	unsigned short  shutterMin;        // 1 2 3 4 5 6 7 8
}mDetectParam;

typedef struct CamDetectParam{
	unsigned int   timep[4]; //0 1�����賿 ��1����2   2 3���� �ƻ� ��1����2
	mCamDetectLane detectlane;        //��⳵����
	mVirtualLaneLine  laneline;      //�õ��ĳ�����
	mStandardPoint standpoint[STANDARDVAULEMAX];       //�궨�������
	mDemDetectArea area;              //�궨������
	mDetectParam detectparam[ALGMAX];   // 0  ����Ĳ���,  1�������ϲ���
}mCamDetectParam;  //��������м���йز���

#if 0
/*-------------------�źŻ��Խ�Э��---------------------------*/
typedef struct channelinfo{
	unsigned char channelNo;
	unsigned char queuelen;
	unsigned char speed;
	unsigned char usec0;
	unsigned char usec1;
	unsigned char bVehicleType;
	unsigned char channelstatus;
	unsigned int  flowdata;
}mChannelInfo;

typedef struct channelinfo1{
	unsigned char channelNo;
	unsigned char flowdata;
	unsigned char queuelen;
	unsigned char channelstatus;
}mChannelInfo1;

typedef struct
{
	unsigned char version;
	unsigned char sendID;
	unsigned char reciveID;
	unsigned char linkAddr;
	unsigned char externalbyte;
	mChannelInfo1 channel[4];
}ExternalProtocol;
#endif

////ͨ��֡����
//enum LINKID {
// LINKID_ETH1=1,
// LINKID_ETH2,
// LINKID_UART1,
// LINKID_UART2,
// LINKID_UART3,
// LINKID_UART4,
// LINKID_UART5,
// LINKID_CAN,
// LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD,
// LINKID_BROARDCAST=0xFF
//};
////Э������
//enum PROTOCOLTYPE {
//	PROTOCOTYPE_VIDEO_DETECTOR=0x01,
//	PROTOCOTYPE_MAX
//};
////�豸��
//enum DEVICECLASS {
//	DEVICECLASS_VIDEO_SEPERATED_MACHINE=0x01,
//	DEVICECLASS_IPCAMERAL,
//	DEVICECLASS_VIDEO_INTEGRATED_MACHINE,
//	DEVICECLASS_MAIN_CONTROL_BOARD,
//	DEVICECLASS_COMMUNICATION_BOARD,
//	DEVICECLASS_BROADCAST=0x0F,
//};
////ͨ��֡ͷ����
//typedef struct {
//	unsigned char mHeaderTag;
//	unsigned char mSenderLinkId;
//	unsigned char mRecieverLinkId;
//	unsigned char mProtocolType;
//	unsigned char mProtocolVersion;
//	struct {
//		unsigned char mDeviceIndex:4;
//		unsigned char mDeviceClass:4;
//	}mSenderDeviceID,mRecieverDeviceId;
//	unsigned char mSessionID;
//	unsigned char mDataType;
//	//unsigned char pContent[1];
//} FrameHeader;
////ͨ��֡β����
//typedef struct {
//	unsigned char mXor;
//	unsigned char mTailTag;
//}FrameTail;
/////////////////////////////////////////////////// //ʵʱ��ͨ����/////////////
//typedef struct EachChannelPack {
//		unsigned char mDetectChannelIndex;
//		unsigned char mQueueLength;
//		unsigned char mRealTimeSingleSpeed;
//		struct {
//			unsigned char bOccupyStatus0:1;
//			unsigned char bOccupyStatus1:1;
//			unsigned char bFix0:2;
//			unsigned char flow:2;
//			unsigned char bVehicleType:2;
//		}mDetectDataOfHeaderVirtualCoil;
//		unsigned char mHeadwayTimeOfHeaderVirtualCoil;
//		unsigned char mOccupancyOfHeaderVirtualCoil;
//		struct {
//		unsigned char bOccupyStatus0:1;
//		unsigned char bOccupyStatus1:1;
//		unsigned char bFix0:6;
//		}mDetectDataOfTailVirtualCoil;
//		unsigned char mHeadwayTimeOfTailVirtualCoil;
//		unsigned char mOccupancyOfTailVirtualCoil;
//		struct {
//		unsigned char bFix0:7;
//		unsigned char bDataIsValid:1;
//		}mWorkStatusOfDetectChannle;
//}EachChannelPackm;
//
//typedef struct {
//	unsigned char mDetectChannelCount;
//	EachChannelPackm EachChannelPack[1];
//}RealTimeTrafficData;
//
///////// //�豸����״̬��ѯ���� ////////////ֱ��ʹ��ͨ��֡ͷ ///////////////////////////////////////////
////�豸����״̬��ѯ��Ӧ /////////////////////////////////////////
//
//typedef	struct{
//		struct {
//			unsigned char mDeviceIndex:4;
//			unsigned char mDeviceClass:4;
//		}mCameralDeviceId;
//		struct {
//			unsigned char bNorth:1;
//			unsigned char bEastNorth:1;
//			unsigned char bEast:1;
//			unsigned char bEastSouth:1;
//			unsigned char bSouth:1;
//			unsigned char bWestSouth:1;
//			unsigned char bWest:1;
//			unsigned char bWestNorth:1;
//		}mCameralPosition;
//		struct {
//			unsigned char bWorkMode:2;
//			unsigned char bBackgroundRefreshed:1;
//			unsigned char bH264DecodeStatus:1;
//			unsigned char bCameralOnLine:1;
//			unsigned char bPictureStable:1;
//			unsigned char bFix0:2;
//		}mCameralStatus;
//}EachCameralStatus;
//
//typedef struct {
//	struct {
//	unsigned char mDeviceIndex:4;
//	unsigned char mDeviceClass:4;
//	}mDetectMainMachineDeviceId;
//	union {
//		struct {
//		unsigned char bRegisted:1;
//		unsigned char bTimeCorrected:1;
//		unsigned char bFix0:6;
//		}mStatusWhenSeperatedMachine;
//		unsigned char mFix0WhenIntegratedMachine;
//	}uDetectMainMachineStatus;
//	unsigned char mCameralCount;
//	EachCameralStatus mEachCameralStatus[1];
//}DeviceWorkStatusQueryResponse;
//
////////////////////////////////////////// //��ʱ�㲥���� ////////
//typedef struct {
//	unsigned int mUTCTime;
//}TimeBroadcastCommand;
//
/////////////////////// //�߷�㲥���� /////////
//typedef struct {
//	unsigned char mIsRushHour;
//}RushHourBroadcastCommand;
//
//////////////////////// //ͨѶ�������ѯ���� /////////////////// //ֱ��ʹ��ͨ��֡ͷ
///////////// //ͨѶ�������ѯ��Ӧ ///
//typedef struct {
//	struct {
//	unsigned short mListenPort; unsigned char mIp[4];
//	unsigned char mNetmask[4]; unsigned char mGateway[4];
//	}mEthernet[2];
//	struct {
//	unsigned char mWorkElectricLevel;
//	unsigned char mBaudRate;
//	unsigned char mDataBits;
//	unsigned char mStopBits;
//	unsigned char mParityBit;
//	}mUart[5];
//	unsigned char mCanBaudRate;
//}CommunicationBoardArgumentQueryResponse;
//
/////////////////////// //ͨѶ������������� ////////////
//typedef struct {
//	struct {
//		unsigned short mListenPort;
//		unsigned char mIp[4];
//		unsigned char mNetmask[4];
//		unsigned char mGateway[4];
//	}mEthernet[2];
//	struct {
//		unsigned char mWorkElectricLevel;
//		unsigned char mBaudRate;
//		unsigned char mDataBits;
//		unsigned char mStopBits;
//		unsigned char mParityBit;
//	}mUart[5];
//	unsigned char mCanBaudRate;
//}CommunicationBoardArgumentSetUpCommand;
//
/////////////////////////////////// ////ͨѶ�����������Ӧ ///////////////////////////
//typedef struct {
//	unsigned char mResult;
//}CommunicationBoardArgumentSetUpResponse;
////////////////////////////////////////////////////////////////
////�豸�����¼������ϱ�
//typedef struct{
//	struct {
//	unsigned char mDeviceIndex:4;
//	unsigned char mDeviceClass:4;
//	}mEventDeviceId;
//	union {
//		struct { unsigned char bNorth:1;
//		unsigned char bEastNorth:1;
//		unsigned char bEast:1;
//		unsigned char bEastSouth:1;
//		unsigned char bSouth:1;
//		unsigned char bWestSouth:1;
//		unsigned char bWest:1;
//		unsigned char bWestNorth:1;
//		}mEventOccourPositionWhenCameralOrIntegratedCameral;
//		unsigned char mFix0WhenSeperatedMachine;
//	}uEventOccourPosition;
//	unsigned char mEvent;
//	unsigned char mReserverFix0;
//	unsigned int mEventTime;
//}DeviceEventsAutoReporte;
//
//
//typedef struct staticchannel{
//	unsigned char index;
//	unsigned char status;   //��ʾ���ݵ���Ч��, 0 ��ʼ״̬, ������״̬, 1Ϊ����״̬, 2ǰ�����ݷ������.
//	//unsigned char mode;
//	//unsigned char black;
//	//unsigned char algswitch;
//	EachCameralStatus EachStatus;
//	EachChannelPackm Eachchannel[4];
//	//unsigned int  frameNum;
//}mStaticchannel;
#pragma pack(pop)
enum{
	CLASS_NULL,
	CLASS_char,
	CLASS_short,
	CLASS_int,
	CLASS_mCommand,
	CLASS_mDetectDeviceConfig,
	CLASS_mCamParam,
	CLASS_mCamDetectParam,
	CLASS_mChannelVirtualcoil,
	CLASS_mCamAttributes,
	CLASS_mCamDemarcateParam,
	CLASS_mCamDetectLane ,
	CLASS_mChannelCoil,
	CLASS_mPoint,
	CLASS_mLine,
	CLASS_mVirtualLaneLine,
	CLASS_mStandardPoint,
	CLASS_mDemDetectArea,
	CLASS_mDetectParam,

};
void net_decode_obj(unsigned char *bf,int type,int encode);
void net_decode_obj_n(unsigned char *addr,int type,int encode,int num,int size);
int get_obj_len(int class_type);
int prepare_pkt(unsigned char *p_start, int head_length,int reply_type, int class_type, int class_length,unsigned char *p_obj);
int handle_pkt(unsigned char *p_start, int head_length, int class_type, int class_length);
int get_pkt(unsigned char *p_start, int head_length,int reply_type, int class_type, int class_length,unsigned char *p_obj);

//int  handle_buffer(unsigned char *buf,int len,char *ip);
//void init_config();
//
//mCamDetectParam *get_mCamDetectParam(int index);
//mCamParam *get_mCamParam(int index);
//int get_obj_len(int class_type);
//int get_cam_status(int index);
//int get_cam_id(int index);
//int get_cam_direction(int index);
//int get_dev_id();
//void get_sig_ip(char *des);
//int  get_sig_port();
////void save_obj(unsigned char * p_obj,int class_type,int index);
//void client_output(int index);
//mRealStaticInfo *client_get_info(int index);
////#define CAM_MAX 4
//#define BUFFER_MAX 1000
//#define TCP_TIMEOUT 100
#endif /* PROTOCOL_H_ */
