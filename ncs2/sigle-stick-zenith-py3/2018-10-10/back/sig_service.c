
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include "client_net.h"
#include "sig_service.h"
#include "csocket.h"
#include "common.h"
#include "g_define.h"
#include <time.h>
#include <sys/time.h>
#include "udp_network.h"

//#include "cam_alg.h"
#define CYCLE_STATIS_TIM  300
#define LIST_IN_OUT_MAX   500

//////////////////////////////////////in out start
pthread_mutex_t mutex_in_out;

int list_count = 0;
int s_next_index = 0;
int g_next_index = 0;
radar_rt_lane_car_in_out_info_t list_in_out[LIST_IN_OUT_MAX];

//插入入车出车的实时数据
bool add_car_in_out_item(radar_rt_lane_car_in_out_info_t *io_item)
{

	//lock 
	pthread_mutex_lock(&mutex_in_out);
	
	if(s_next_index == g_next_index && list_count > 0) {
		pthread_mutex_unlock(&mutex_in_out);
		return false;
	}
	memcpy(&list_in_out[s_next_index], io_item, sizeof(radar_rt_lane_car_in_out_info_t));
	list_count++;
	s_next_index++;
	s_next_index = s_next_index % LIST_IN_OUT_MAX;
	
	pthread_mutex_unlock(&mutex_in_out);

	return true;
	//unlock
}

//获取入车出车的实时数据
bool get_car_in_out_item(radar_rt_lane_car_in_out_info_t *io_item)
{
	
	pthread_mutex_lock(&mutex_in_out);
	
	if (list_count == 0) {
		pthread_mutex_unlock(&mutex_in_out);
		return false;
	}

	memcpy(io_item, &list_in_out[g_next_index], sizeof(radar_rt_lane_car_in_out_info_t));
	list_count--;
	g_next_index++;
	g_next_index = g_next_index % LIST_IN_OUT_MAX;
	
	pthread_mutex_unlock(&mutex_in_out);
	
	return true;
	
}

///////////////////////////////////////////end in out

unsigned char get_crc(unsigned char *buf,int sz)
{
    unsigned char crc=0;
    for(int i=0;i<sz;i++){
        crc^=*(buf+i);
    }
    return crc;
}
void get_outcar_info()
{

}
void get_time_string(char *buf , int len)
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);

    sprintf(buf,"%d-%d-%d %d:%d:%d",t->tm_year+1900,t->tm_mon+1,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);

}
int get_year()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_year+1900;
}
int get_year_tail()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return (t->tm_year+1900)%2000;
}
int get_month()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_mon+1;
}
int get_day()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_mday;
}
int get_hour()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_hour;
}
int get_min()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_min;
}
int get_sec()
{
    time_t tt;
    tt=time(NULL);
    struct tm *t=  localtime(&tt);
    return t->tm_sec;
}

//pthread_mutex_t out_car_lock;
//pthread_mutex_t queue_lock;
//pthread_mutex_t flow_lock;

pthread_mutex_t mutex_lock;
typedef struct sig_holder{
    m_sig_data traffic_rst;
    pthread_mutex_t sig_data_lock;
}m_holder;
m_holder holder[ACTIVE_CAM_NUM];
int sig_state;
static pthread_mutex_t sig_state_lock;
enum{
    SIG_PRE_CONNECT,
    SIG_CONNECTED,
    SIG_NULL
};
int sig_fd;
char sig_ip[IP_LEN];
int sig_port;
//m_sig_data channel_rst[CAM_NUM];
void sig_set_state(int state)
{
    //	prt(stack,"set to %d",state);
    sig_state= state;
}
extern void submit_unlock_sig_data(int index)
{
    //	int ori_state=0;
    //	pthread_mutex_lock(&holder[index].sig_data_lock);
    ////	ori_state=holder[index].traffic_rst.camera_state_change;
    //	memcpy(&holder[index].traffic_rst,p_channel_rst,sizeof(m_sig_data));
    //	if(p_channel_rst->camera_state_change==1){
    //		prt(info,"state change ");
    //		p_channel_rst->camera_state_change=0;
    //	}
    pthread_mutex_unlock(&holder[index].sig_data_lock);
    //	prt(info,"index %d,src num %d,dst num %d",index,p_channel_rst->lane_num,holder[index].traffic_rst.lane_num);

}
extern m_sig_data * get_locked_sig_data(int index)
{
    pthread_mutex_lock(&holder[index].sig_data_lock);
    return &holder[index].traffic_rst;
}

int externalProtocolAddHeader(unsigned char *buff,int *size)
{
    buff[0]=0xC0;
    buff[*size+1]=0xC0;
    *size=*size+2;
    return 0;
}

void externalProtocolEncode(unsigned char *buff,int *size)
{
#if 0
    int i=0,j=0;
    for(i=0;i<*size;i++)
    {
        if(buff[i]==0XC0)
        {
            buff[i]=0XDB;
            *size=*size+1;
            for(j=*size-1;j>i+1;j--)
            {
                buff[j]=buff[j-1];
            }
            buff[i+1]=0XDC;

        }
        else if(buff[i]==0XDB)
        {
            buff[i]=0XDB;
            *size=*size+1;
            for(j=*size-1;j>i+1;j--)
            {
                buff[j]=buff[j-1];
            }
            buff[i+1]=0XDD;
        }
    }
#else
    int i=0,j=0;
    for(i=0;i<*size;i++)
    {
        if(buff[i]==0xC0)
        {
            buff[i]=0xDB;
            *size=*size+1;
            for(j=*size-1;j>i+1;j--)
            {
                buff[j]=buff[j-1];
            }
            buff[i+1]=0xDC;

        }
        else if(buff[i]==0xDB)
        {
            buff[i]=0xDB;
            *size=*size+1;
            for(j=*size-1;j>i+1;j--)
            {
                buff[j]=buff[j-1];
            }
            buff[i+1]=0xDD;
        }
    }
#endif
}

void externalProtocolDecode(unsigned char *buff,int *size)
{
#if 0
    int i=0,j=0;
    for(i=0;i<*size;i++)
    {
        if(buff[i]==0x7D&&buff[i+1]==0x5E){
            buff[i]=0x7E;
            //闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵鏁愭径濠勵吅闂佹寧绻傞幉娑㈠箻缂佹鍘搁梺鍛婁緱閸犳宕愰幇鐗堢厸鐎癸拷鐎ｎ剛鐦堥悗瑙勬礃鐢帟鐏掗梺缁樿壘閻°劎锟芥艾缍婇弻鈥愁吋鎼粹�插闂佺懓鍢查崲鏌ワ綖濠靛鏁嗛柛灞剧敖閵娾晜鈷戦柛婵嗗椤箓鏌涢弮锟介崹鍧楃嵁閸愵喖顫呴柕鍫濇噹缁愭稒绻濋悽闈浶㈤悗姘间簽濡叉劙寮撮姀鈾�鎷绘繛杈剧悼閸庛倝宕甸敓浠嬫⒑閹肩偛锟芥牠鎮ч悩鑽ゅ祦闊洦绋掗弲鎼佹煥閻曞倹瀚�?闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷???闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷??闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷
            for(j=i+1;j<*size;j++)
            {
                buff[j]=buff[j+1];
            }
            *size=*size-1;
        }else if(buff[i]==0x7D&&buff[i+1]==0x5D){
            buff[i]=0x7D;
            //闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵鏁愭径濠勵吅闂佹寧绻傞幉娑㈠箻缂佹鍘搁梺鍛婁緱閸犳宕愰幇鐗堢厸鐎癸拷鐎ｎ剛鐦堥悗瑙勬礃鐢帟鐏掗梺缁樿壘閻°劎锟芥艾缍婇弻鈥愁吋鎼粹�插闂佺懓鍢查崲鏌ワ綖濠靛鏁嗛柛灞剧敖閵娾晜鈷戦柛婵嗗椤箓鏌涢弮锟介崹鍧楃嵁閸愵喖顫呴柕鍫濇噹缁愭稒绻濋悽闈浶㈤悗姘间簽濡叉劙寮撮姀鈾�鎷绘繛杈剧悼閸庛倝宕甸敓浠嬫⒑閹肩偛锟芥牠鎮ч悩鑽ゅ祦闊洦绋掗弲鎼佹煥閻曞倹瀚�?闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷???闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷??闂傚倸鍊搁崐鎼佸磹瀹勬噴褰掑炊瑜夐弸鏍煛閸ャ儱鐏╃紒鎰殜閺岀喖鎮ч崼鐔哄嚒闂佸憡鍨规慨鎾煘閹达附鍋愰悗鍦Т椤ユ繄绱撴担鍝勵�岄柛銊ョ埣瀵濡搁埡鍌氫簽闂佺鏈粙鎴︻敂閿燂拷
            for(j=i+1;j<*size;j++)
            {
                buff[j]=buff[j+1];
            }
            *size=*size-1;
        }
    }
    return;
#else
    int i=0,j=0;
    for(i=0; i<*size; i++)
    {
        if(buff[i]==0xDB&&buff[i+1]==0xDC)
        {
            buff[i]=0xC0;
            for(j=i+1;j<*size;j++)
            {
                buff[j]=buff[j+1];
            }
            *size=*size-1;
        }
        else if(buff[i]==0xDB&&buff[i+1]==0xDD)
        {
            buff[i]=0xDB;
            for(j=i+1;j<*size;j++)
            {
                buff[j]=buff[j+1];
            }
            *size=*size-1;
        }
    }
    //return TRUE;
#endif
}

int externalProtocolAddCrcCode(unsigned char *buff,int *size)
{
#if 0
    unsigned char xor_crc=0;
    int i=0;
    for(i=0;i<*size;i++)
        xor_crc^=buff[i];

    *size=*size+1;
    buff[*size-1]=xor_crc;

    if(buff[*size-1]==0xC0){
        buff[*size-1]=0xDB;
        *size=*size+1;
        buff[*size-1]=0xDC;
    }else if(buff[*size-1]==0xDB){
        buff[*size-1]=0xDB;
        *size=*size+1;
        buff[*size-1]=0xDD;
    }
#else
    unsigned char xor_crc=0;
    int i=0;
    for(i=0;i<*size;i++)
        xor_crc^=buff[i];
    *size=*size+1;
    buff[*size-1]=xor_crc;
#endif
    return 0;
}

int externalProtocolCheckCrc(unsigned char *buff,int size)
{
#if 0
    unsigned char xor_crc=0;
    unsigned char my_xor=0;
    int i=0;
    int len = 0;

    if(buff[size-1]==0xDB&&buff[size-2]==0xDC){
        my_xor = 0xC0;
        len = size-2;
    }else if(buff[size-1]==0xDB&&buff[size-2]==0xDD){
        my_xor = 0xDB;
        len = size-2;
    }else{
        len = size-1;
        my_xor = buff[size-1];
    }

    for(i=0;i<len;i++)
        xor_crc^=buff[i];

    if(xor_crc==my_xor){
        return 0;
    }else{
        return -1;
    }
#else
    char xor_crc=0;
    int i=0;
    for(i=0;i<size;i++)
    {
        xor_crc^=buff[i];
    }
    if(xor_crc==buff[size])
    {
        return 1;
    }
    else
    {
        return 0;
    }
#endif
}

int ReportedCammerStatus(int sock, int mSessionID)
{
    int size = 0;
    unsigned char  sendbuff[100];
    m_sig_data *p_fvdstaticchannel;//=&p_detect_ctx->cam_ctx[i].thread_param.channel_rst;
    //	unsigned char *sendbuff=NULL;
    //	mSystemConfig * fsys = g_pSyscfg;

    ///	sendbuff = (unsigned char *)malloc(100);
    if(sendbuff==NULL)
        return 0;
    size = 0;
    memset(sendbuff, 0, sizeof(sendbuff));
    FrameHeader * fnewhead = (FrameHeader*)sendbuff;
    fnewhead->mSenderLinkId = 0x0;
    fnewhead->mRecieverLinkId = LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD;
    fnewhead->mProtocolType = PROTOCOTYPE_VIDEO_DETECTOR;
    fnewhead->mProtocolVersion = 0x12;
    fnewhead->mSenderDeviceID.mDeviceIndex = 0x1;
    fnewhead->mSenderDeviceID.mDeviceClass = 0x1;
    fnewhead->mRecieverDeviceId.mDeviceIndex = 0x0;
    fnewhead->mRecieverDeviceId.mDeviceClass = 0x4;
    fnewhead->mSessionID = mSessionID;
    fnewhead->mDataType = 0x04;

    size = sizeof(FrameHeader)-1;
    DeviceWorkStatusQueryResponse *res=(DeviceWorkStatusQueryResponse *)(sendbuff+sizeof(FrameHeader));

    res->mDetectMainMachineDeviceId.mDeviceClass = 0x1;
    res->mDetectMainMachineDeviceId.mDeviceIndex = get_dev_id();
    res->uDetectMainMachineStatus.mStatusWhenSeperatedMachine.bRegisted=0x1;
    res->uDetectMainMachineStatus.mStatusWhenSeperatedMachine.bTimeCorrected=0x1;
    res->uDetectMainMachineStatus.mStatusWhenSeperatedMachine.bFix0=0x0;
    res->mCameralCount = 0x0;
    size +=3;

    int i =0;
    for(i=0; i<ACTIVE_CAM_NUM; i++){
        p_fvdstaticchannel=&holder[i].traffic_rst;
        if(get_cam_status(i)){
            EachCameralStatus * cstatus=(EachCameralStatus*)(sendbuff+sizeof(FrameHeader)+3+res->mCameralCount*sizeof(EachCameralStatus));

            //	cstatus->mCameralDeviceId.mDeviceIndex = p_detect_ctx->cam_ctx[i].thread_param.cam_cfg.camattr.camID;

            cstatus->mCameralDeviceId.mDeviceIndex =get_cam_id(i);

            cstatus->mCameralDeviceId.mDeviceClass = DEVICECLASS_IPCAMERAL;
            //			prt(info,"get direction %d",get_cam_direction(i));
            switch(get_cam_direction(i)){

            case 0x1: cstatus->mCameralPosition.bNorth=0x1;break;

            case 0x2: cstatus->mCameralPosition.bEastNorth=0x1;break;

            case 0x3: cstatus->mCameralPosition.bEast=0x1;break;

            case 0x4: cstatus->mCameralPosition.bEastSouth=0x1;break;

            case 0x5: cstatus->mCameralPosition.bSouth=0x1;break;

            case 0x6: cstatus->mCameralPosition.bWestSouth=0x1;break;

            case 0x7: cstatus->mCameralPosition.bWest=0x1;break;

            case 0x8: cstatus->mCameralPosition.bWestNorth=0x1;break;

            default:  cstatus->mCameralPosition.bNorth=0x1;break;

            }
            cstatus->mCameralStatus.bWorkMode

                    = p_fvdstaticchannel->EachStatus.mCameralStatus.bWorkMode;

            cstatus->mCameralStatus.bBackgroundRefreshed

                    = p_fvdstaticchannel->EachStatus.mCameralStatus.bBackgroundRefreshed;

            cstatus->mCameralStatus.bH264DecodeStatus

                    =p_fvdstaticchannel->EachStatus.mCameralStatus.bH264DecodeStatus;

            if(0x1 == p_fvdstaticchannel->status || 0x2 == p_fvdstaticchannel->status)
                cstatus->mCameralStatus.bCameralOnLine=0x1;
            else
                cstatus->mCameralStatus.bCameralOnLine=0x0;
            cstatus->mCameralStatus.bPictureStable
                    = p_fvdstaticchannel->EachStatus.mCameralStatus.bPictureStable;
            cstatus->mCameralStatus.bFix0
                    =p_fvdstaticchannel->EachStatus.mCameralStatus.bFix0;
            res->mCameralCount++;
            size+=sizeof(EachCameralStatus);
        }
    }


#if 0
    externalProtocolEncode(sendbuff+1,&size);
    externalProtocolAddCrcCode(sendbuff+1,&size);
    externalProtocolAddHeader(sendbuff,&size);
#else
    externalProtocolAddCrcCode(sendbuff+1,&size);
    externalProtocolEncode(sendbuff+1,&size);
    externalProtocolAddHeader(sendbuff,&size);
#endif
    int tmp=0;
    //			prt(stack,"=========report cam status======");
    //			for (tmp = 0; tmp < size; ++tmp) {
    //				prt(info,"%x",sendbuff[tmp]);
    //			}
    return SendDataByTcp(sock, (char *)sendbuff, size);
}

int RequestPro(unsigned char *buffer, int len, int sock)
{
    printf("get request\n");
    unsigned char *sendbuff=NULL;
    int size = 0;
    int i=0;
    while(i<len){
        printf("%x\n",buffer[i]);
        i++;
    }
    if(buffer[0] != 0xC0 || buffer[len-1] != 0xC0){
        return 0;
    }

#if 0
    size=len-2;
    if(externalProtocolCheckCrc(buffer+1,size)){
        return 0;
    }
    size-=1;
    externalProtocolDecode(buffer+1,&size);
    if((size+1) != sizeof(FrameHeader) && (size+1) != (sizeof(FrameHeader)+4)){
        return 0;
    }
#else
    size=len-2;
    printf("check crc \n\n");
    externalProtocolDecode(buffer+1,&size);
    if(externalProtocolCheckCrc(buffer+1,size)){
        printf("check crc err \n\n");
        return 0;
    }
#endif

    FrameHeader * fhead = (FrameHeader*)buffer;
    if(fhead->mSenderLinkId != LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD
            || fhead->mRecieverLinkId != LINKID_BROARDCAST
            || fhead->mProtocolType != PROTOCOTYPE_VIDEO_DETECTOR
            || fhead->mProtocolVersion != 0x12){
        return 0;
    }

    if(fhead->mDataType == 0x05){
        char strTime[24];
        //time_t UTCTime = ntohl(*((unsigned int*)(buffer+sizeof(FrameHeader))))+28800;
        time_t UTCTime = *((unsigned int*)(buffer+sizeof(FrameHeader)))+28800;
        struct tm *ppltime = gmtime(&UTCTime);
        strftime(strTime,24,"%F %T",ppltime);
        char cmd[50]={0};
        sprintf(cmd, "date -s \"%s\";hwclock -w", strTime);
        system(cmd);
        printf("reply tm\n");
    }else if(fhead->mDataType == 0x03){
        printf("rply  \n");
        return ReportedCammerStatus(sock,fhead->mSessionID);
    }
    return 0;
}
#include "camera_service.h"
int ReportedEvent(int sock, char event)
{
    int size;
    unsigned char mysbuf[100] = {0};

    memset(mysbuf, 0, sizeof(mysbuf));
    FrameHeader * fhead= (FrameHeader *)mysbuf;
    fhead->mSenderLinkId = 0x00;
    fhead->mRecieverLinkId = LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD;
    fhead->mProtocolType = PROTOCOTYPE_VIDEO_DETECTOR;
    fhead->mProtocolVersion = 0x12;
    //fhead->mSenderDeviceID.mDeviceIndex = g_pSyscfg->devparam.deviceID;
    fhead->mSenderDeviceID.mDeviceClass = 0x1;
    fhead->mRecieverDeviceId.mDeviceIndex = 0x0;
    fhead->mRecieverDeviceId.mDeviceClass = 0x4;
    fhead->mSessionID = 0xFF;

    fhead->mDataType = 0x10;
    size = sizeof(FrameHeader)-1;
    DeviceEventsAutoReporte* report = (DeviceEventsAutoReporte*)(mysbuf+sizeof(FrameHeader));

    report->mEventDeviceId.mDeviceIndex = get_dev_id();
    report->mEventDeviceId.mDeviceClass = 0x1;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bNorth=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bEastNorth=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bEast=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bEastSouth=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bSouth=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bWestSouth=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bWest=0x0;
    report->uEventOccourPosition.mEventOccourPositionWhenCameralOrIntegratedCameral.bWestNorth=0x0;
    size+= sizeof(DeviceEventsAutoReporte);
    report->mEvent = event;
    report->mEventTime =(int )time(NULL);
#if 0
    externalProtocolEncode(mysbuf+1,&size);
    externalProtocolAddCrcCode(mysbuf+1,&size);
    externalProtocolAddHeader(mysbuf,&size);
#else
    externalProtocolAddCrcCode(mysbuf+1,&size);
    externalProtocolEncode(mysbuf+1,&size);
    externalProtocolAddHeader(mysbuf,&size);
#endif
    int tmp=0;
    //				prt(info,"=========report cam event======");
    //				for (tmp = 0; tmp < size; ++tmp) {
    //					prt(info,"%x",mysbuf[tmp]);
    //				}

    return SendDataByTcp(sock, (char *)mysbuf, size);
}
int ReportedRealStatic(int sock )
{
    int len=-1;
    int size, i, j, ret;
    unsigned char mysbuf[100] = { 0 };
    memset(mysbuf, 0, sizeof(mysbuf));
    FrameHeader * fhead = (FrameHeader *) mysbuf;
    size = sizeof(FrameHeader) - 1;
    fhead->mSenderLinkId = 0x00;
    fhead->mRecieverLinkId = LINKID_UART_BETWEEN_COMBOARD_AND_MAINCONTLBOARD;
    fhead->mProtocolType = PROTOCOTYPE_VIDEO_DETECTOR;
    fhead->mProtocolVersion = 0x12;
    fhead->mSenderDeviceID.mDeviceIndex =get_dev_id();
    fhead->mSenderDeviceID.mDeviceClass = 0x1;
    fhead->mRecieverDeviceId.mDeviceIndex = 0x0;
    fhead->mRecieverDeviceId.mDeviceClass = 0x4;
    fhead->mSessionID = 0xFF;
    fhead->mDataType = 0x01;

    for (i = 0; i < ACTIVE_CAM_NUM; i++) {
        if(get_cam_status(i)){
            size = sizeof(FrameHeader) - 1;
            //		if (p_detect_ctx->dev_cfg.camstatus[i] == 0)
            //			continue;
            //
            //		if(loops++%10==0){
            //			if(p_detect_ctx->cam_ctx[i].thread_param.algparam.framecount==frames_count_old[i]){
            //				 ReportedCammerStatus(p_detect_ctx->sig_fd, 0xFF);
            //			}
            //			frames_count_old[i]=p_detect_ctx->cam_ctx[i].thread_param.algparam.framecount;
            //		}
            //	traffic_rst
            RealTimeTrafficData*tmpReal = (RealTimeTrafficData*)(mysbuf+sizeof(FrameHeader));
            holder[i].traffic_rst.lane_num=get_lane_num(i);
            tmpReal->mDetectChannelCount = holder[i].traffic_rst.lane_num;
            //	prt(info,"%d",	tmpReal->mDetectChannelCount);
            size+=1;
            pthread_mutex_lock(&holder[i].sig_data_lock);
            //	prt(info,"lane num %d", tmpReal->mDetectChannelCount);
            for (j = 0;j< tmpReal->mDetectChannelCount;j++) {

                struct EachChannelPack *ptrChannel = (struct EachChannelPack *) (mysbuf + sizeof(FrameHeader) + 1
                                                                                 + j * sizeof(struct EachChannelPack));
                holder[i].traffic_rst.Eachchannel[j].mWorkStatusOfDetectChannle.bDataIsValid=get_cam_running_state(i);
                memcpy((void *) ptrChannel,
                       (void *) &holder[i].traffic_rst.Eachchannel[j],sizeof(EachChannelPackm));
                size += sizeof(EachChannelPackm);
                //	prt(info,"dealing cam %d , lane %d",i,j);
                //	dump_pack(i,j,ptrChannel);
            }
            pthread_mutex_unlock(&holder[i].sig_data_lock);
            externalProtocolAddCrcCode(mysbuf + 1, &size);
            externalProtocolEncode(mysbuf + 1, &size);
            externalProtocolAddHeader(mysbuf, &size);


            len = SendDataByTcp(sock, (char *) mysbuf, size);
            if (len <= 0) {
                prt(info,"sentd tcp sig machine err");
                return len;
            } else {
            }
        }
    }
    return len;
}
void reset_sig_machine()
{

    sig_set_state(SIG_NULL);
    close_socket(&sig_fd);
    get_sig_ip(sig_ip);
    prt(info,"reset sig ip to %s",sig_ip);
    sig_port=get_sig_port();
    sig_set_state(SIG_PRE_CONNECT);
}

void *report_data_callback_fun(void *data)
{
    if (sig_state == SIG_CONNECTED) {
        if (ReportedRealStatic(sig_fd) <= 0) {
            close_socket(&sig_fd);
            sig_set_state(SIG_PRE_CONNECT);
        }
    }

}
void *check_event_callback_fun(void *data)
{
    int i;
    for (i = 0; i < ACTIVE_CAM_NUM; i++) {
        if (holder[i].traffic_rst.camera_state_change&&sig_state==SIG_CONNECTED) {
            if(ReportedCammerStatus(sig_fd, 0xFF)>0)
            {
                holder[i].traffic_rst.camera_state_change = 0;
            }else{
                prt(info,"fail to send");
            }
        }
    }
}

extern void reboot_cmd()
{
    int retlen = ReportedEvent(sig_fd, 0x1);
    if (retlen < 0) {
        close_socket(&sig_fd);
        //		return ;
    }
    retlen = ReportedEvent(sig_fd, 0x4);
    if (retlen < 0) {
        close_socket(&sig_fd);
        //		continue;
    }
    usleep(200000);
    prt(info,"rebooting ");
    system("reboot");
}
int connect_sig()
{
    int retlen;
    if (sig_fd > 0) {
        usleep(1000000);
        prt(debug_sig, "try to connect signal machine,%s",sig_ip);
        retlen =ConnectTcpClient(sig_fd,
                                 (char *) sig_ip,
                                 sig_port);
        prt(debug_sig,"ret %d",retlen);
        if (-1 == retlen) {
            close_socket(&sig_fd);
        } else {
            prt(net, "ok to connect signal machine");
        }
    }
    return retlen;
}
void *sig_service_thread(void *data)
{
    int retlen = 0;
    while (1) {
        usleep(100000);
        switch (sig_state) {
        case SIG_PRE_CONNECT:
            prt(debug_sig,"connecting ");
		    if(sig_fd<=0){
				//sig_fd=CreateTcpClientSock(SIGNALPORTL, 0);
                sig_fd=CreateTcpClientSock(0, 0);// random port
                if(sig_fd<0)
                    continue;
                if(connect_sig()<=0)
                    continue;
            }
			
			
            // retlen = ReportedEvent(sig_fd, 0x2);
            if (retlen < 0) {
                close_socket(&sig_fd);
                prt(debug_sig,"send 2");
                break;
            }
            //  retlen = ReportedEvent(sig_fd, 0x3);
            if (retlen < 0) {
                close_socket(&sig_fd);
                prt(debug_sig,"send 3");
                break;
            }
            //  retlen = ReportedCammerStatus(sig_fd, 0xFF);
            if (retlen < 0) {
                close_socket(&sig_fd);
                prt(debug_sig,"send ff");
                break;
            }
			
		    sig_set_state( SIG_CONNECTED);
            prt(debug_sig,"sig connected ");
            break;
        case SIG_CONNECTED:
            break;
        case SIG_NULL:
            break;
        default:
            break;
        }
    }
}
static void init_data()
{
    int i;

    sig_set_state(SIG_NULL);
    sig_fd=-1;
    get_sig_ip(sig_ip);
    sig_port=get_sig_port();
    //	pthread_mutex_init(&sig_state_lock);
    for(i=0;i<ACTIVE_CAM_NUM;i++){
        pthread_mutex_init(&holder[i].sig_data_lock,NULL);
    }
    pthread_mutex_init(&mutex_lock,NULL);
    //    pthread_mutex_init(&out_car_lock,NULL);
    //    pthread_mutex_init(&queue_lock,NULL);
    //    pthread_mutex_init(&flow_lock,NULL);
    pthread_mutex_init(&mutex_in_out,NULL);
}
m_timed_func_data callback_data;
m_timed_func_data callback_data1;

void init_sig_service()
{
    init_data();
    create_detach_thread(sig_service_thread,1,NULL);
    callback_data.time=500000;
    callback_data.func=report_data_callback_fun;
    regist_timed_callback(&callback_data);

    callback_data1.time=100000;
    callback_data1.func=check_event_callback_fun;
    regist_timed_callback(&callback_data1);
    if(sig_ip[0]!='0'){
        sig_set_state(SIG_PRE_CONNECT);
    }
}


//####################nan jing #####################
#include "cam_alg.h"
data_60s_t d60[NANJING_CAM_NUM] = {0};
data_300s_t d300[NANJING_CAM_NUM] = {0};

//---------queue start----------------
flow_info_t flow_data[NANJING_CAM_NUM];
queue_info_t queue_data[NANJING_CAM_NUM];

radar_result_t  radar_result_tmp;
radar_result_lane_t radar_result_lane_tmp[MAX_LANE_NUM][NANJING_LANE_COIL_MAX];

radar_realtime_t radar_realtime_tmp;
radar_realtime_lane_t radar_realtime_lane_tmp[8][8];

radar_cycle_result_lane_t radar_cycle_lane_tmp[MAX_LANE_NUM];


#if 0
void get_queue_info()
{
    int i,j;
    for(i=0;i<NANJING_CAM_NUM;i++){
        for(j=0;j<NANJING_LANE_MAX;j++){
            queue_data[i].queue_len[j]=info.cams[i].lanes[j].queue_len;
            queue_data[i].crc=  get_crc((unsigned char *)&queue_data[i],sizeof(queue_info_t)-1);




            queue_data[i].detect_time[0]=get_year_tail();
            queue_data[i].detect_time[1]=get_month();
            queue_data[i].detect_time[2]=get_day();
            queue_data[i].detect_time[3]=get_hour();
            queue_data[i].detect_time[4]=get_min();
            queue_data[i].detect_time[5]=get_sec();

            queue_data[i].dir_no=get_direction(i);
            queue_data[i].lane_dir_type=0;
            queue_data[i].queue_start_pos[j]=0;
            queue_data[i].queue_veh_num[j]=info.cams[i].lanes[j].veh_no;

            for(int t=0;t<5;t++){
                queue_data[i].table_head[t]=0xfe;
            }
            queue_data[i].table_no=0x0e;
            queue_data[i].detect_status=(unsigned char)info.cams[i].lanes[j].det_status;
            queue_data[i].table_length=43;
            queue_data[i].veh_speed[j]=info.cams[i].lanes[j].speed;
        }
    }

}
#else
void get_queue_info()
{
    int i,j;
    for(i=0;i<1;i++){
        for(j=0;j<MAX_LANE_NUM;j++){
            queue_data[i].queue_len[j]=info.cams[i].lanes[j].queue_len;
            queue_data[i].crc=  get_crc((unsigned char *)&queue_data[i],sizeof(queue_info_t)-1);

            queue_data[i].detect_time[0]=get_year_tail();
            queue_data[i].detect_time[1]=get_month();
            queue_data[i].detect_time[2]=get_day();
            queue_data[i].detect_time[3]=get_hour();
            queue_data[i].detect_time[4]=get_min();
            queue_data[i].detect_time[5]=get_sec();

            queue_data[i].dir_no=get_direction(i);
            queue_data[i].lane_dir_type=0;
            queue_data[i].queue_start_pos[j]=0;
            queue_data[i].queue_veh_num[j]=info.cams[i].lanes[j].veh_no;

            for(int t=0;t<5;t++){
                queue_data[i].table_head[t]=0xfe;
            }
            queue_data[i].table_no=0x0e;
            queue_data[i].detect_status=(unsigned char)info.cams[i].lanes[j].det_status;
            queue_data[i].table_length=43;
            queue_data[i].veh_speed[j]=info.cams[i].lanes[j].speed;
        }
    }

}
#endif
int send_queue_info(int fd)
{
    printf("--->");fflush(stdout);
    int i=0;
    for(i=0;i<1;i++){
          //for(i=0;i<NANJING_CAM_NUM;i++){
        //   pthread_mutex_lock(&holder[i].sig_data_lock);
        int len = SendDataByTcp(fd, (char *) &queue_data[i], sizeof(queue_info_t));
        //  printf("---> %d",  info.cams[1].lanes[1].in_car);
        // pthread_mutex_unlock(&holder[i].sig_data_lock);
    }
    return 1;
}

int send_radar(int fd,char *buf,int sz)
{
    printf("--->");fflush(stdout);
    int i=0;
    ////for(i=0;i<1;i++){
          //for(i=0;i<NANJING_CAM_NUM;i++){
        //   pthread_mutex_lock(&holder[i].sig_data_lock);
   		////int len = SendDataByTcp(fd, (char *) buf, sz);
        //  printf("---> %d",  info.cams[1].lanes[1].in_car);
        // pthread_mutex_unlock(&holder[i].sig_data_lock);
    ////}
    return SendDataByTcp(fd, (char *) buf, sz);
}

m_timed_func_data camera_queue_info;// 1 s ---> focus on all camera
void *callback_camera_queue_info(void *data)
{
    pthread_mutex_lock(&mutex_lock);
    if (sig_state == SIG_CONNECTED) {
        get_queue_info();
        if (send_queue_info(sig_fd) <= 0) {
            close_socket(&sig_fd);
            sig_set_state(SIG_PRE_CONNECT);
        }
    }
    pthread_mutex_unlock(&mutex_lock);

}
//---------queue end----------------



//----------60s flow start----------------
#include <sys/time.h>
#include <time.h>
int send_flow_info(int fd)
{
    int i=0;
    int j=0;
    for(i=0;i<NANJING_CAM_NUM;i++){
        int len = SendDataByTcp(fd, (char *) &flow_data[i], sizeof(flow_info_t));
    }
    prt(info,"sent %x\n",flow_data[0].table_length);
    prt(info,"sent1 %x\n",flow_data[1].table_length);
    return 1;
}

void calculate_60s()
{

#if 0
    //cal
    //lock
    int i=0;
    int j=0;
    for( i=0;i<NANJING_CAM_NUM;i++){
        get_time_string((char *)flow_data[i].detect_time,sizeof(flow_data[i].detect_time));
        flow_data[i].crc=  get_crc((unsigned char *)&flow_data[i],sizeof(flow_info_t)-1);
        flow_data[i].dir_no=get_direction(i);
        flow_data[i].section_no=1;
        flow_data[i].table_length=48;
        flow_data[i].table_no=0x0c;
        for(int t=0;t<5;t++){
            flow_data[i].table_head[t]=0xfe;
        }

     //   prt(info,"%x\n",flow_data[i].table_length);
        //    if(d60[i].data_valid){//time up, stop acu,start cal and send
        for( j=0;j<NANJING_LANE_MAX;j++){
            flow_data[i].ocuppy_percent[j]=(unsigned char)d60[i].lane_data[j].exist_duration*100/60/1000;
            d60[i].lane_data[j].exist_duration=0;
            if(flow_data[i].flow[j])
                flow_data[i].average_speed[j]=(unsigned char)d60[i].lane_data[j].speed_sum/flow_data[i].flow[j];
            d60[i].lane_data[j].speed_sum=0;
            flow_data[i].flow[j]=(unsigned char)d60[i].lane_data[j].pass_number;
            d60[i].lane_data[j].pass_number=0;


            //            flow_data[i].detect_time[0]=get_year();
            //            flow_data[i].detect_time[1]=get_month();
            //            flow_data[i].detect_time[2]=get_day();
            //            flow_data[i].detect_time[3]=get_hour();
            //            flow_data[i].detect_time[4]=get_min();
            //            flow_data[i].detect_time[5]=get_sec();
      //  prt(info,"(%d %d )-> %x\n\n",i,j,flow_data[0].table_length);
        }
       // prt(info,"%x\n\n",flow_data[i].table_length);
  //  prt(info,"(%d %d )--> %x\n\n",i,j,flow_data[0].table_length);
    }
   // prt(info,"---> %x\n\n",flow_data[0].table_length);

  //  prt(info,"done\n");
	
#endif

}
void calculate_60s_radar()
{

    for(int i=0; i<MAX_LANE_NUM; i++){
       for(int j=0; j<NANJING_LANE_COIL_MAX; j++){

	   		radar_result_lane_tmp[i][j].flowA = (unsigned char)d60[0].lane_data[i][j].car_a_sum;
			radar_result_lane_tmp[i][j].flowB = (unsigned char)d60[0].lane_data[i][j].car_b_sum;
	   		radar_result_lane_tmp[i][j].flowC = (unsigned char)d60[0].lane_data[i][j].car_c_sum;
			radar_result_lane_tmp[i][j].flowSum=(unsigned char)d60[0].lane_data[i][j].pass_number;
	   		radar_result_lane_tmp[i][j].Occupy_rate=(unsigned char)(d60[0].lane_data[i][j].exist_duration*200/CYCLE_STATIS_TIM/1000);
            
			
        	if (radar_result_lane_tmp[i][j].flowSum > 0) { //平均车长 平均车速 平均车头时距
				radar_result_lane_tmp[i][j].average_len = (unsigned char)(d60[0].lane_data[i][j].veh_len_sum/radar_result_lane_tmp[i][j].flowSum) * 10;  //单位:0.1m
				radar_result_lane_tmp[i][j].average_speed = (unsigned char)d60[0].lane_data[i][j].speed_sum/radar_result_lane_tmp[i][j].flowSum;
				radar_result_lane_tmp[i][j].average_head_time = (unsigned char)d60[0].lane_data[i][j].head_time_sum/radar_result_lane_tmp[i][j].flowSum;
			}
			else {
				radar_result_lane_tmp[i][j].average_len = 0;
				radar_result_lane_tmp[i][j].average_speed = 0;
				radar_result_lane_tmp[i][j].average_head_time = 0;
			}

			d60[0].lane_data[i][j].exist_duration = 0;				
			d60[0].lane_data[i][j].pass_number = 0;
			d60[0].lane_data[i][j].head_len_sum = 0;
			d60[0].lane_data[i][j].car_a_sum = 0;
			d60[0].lane_data[i][j].car_b_sum = 0;
			d60[0].lane_data[i][j].car_c_sum = 0;
			d60[0].lane_data[i][j].veh_len_sum = 0;
			d60[0].lane_data[i][j].speed_sum = 0;
			d60[0].lane_data[i][j].head_time_sum = 0;
		
			
  		}

    }

}


void calculate_300s_radar()  //车道号划分的统计周期数据
{
	int m = 0;
	for(int i=0; i<MAX_LANE_NUM; i++){ 

		int occupy_rate = d300[m].lane_data[i].exist_duration *100/CYCLE_STATIS_TIM/1000;
		if (occupy_rate >70) { //拥堵
			radar_cycle_lane_tmp[i].lane_status = 2;
		}else if (occupy_rate > 40) { //缓行
			radar_cycle_lane_tmp[i].lane_status = 1;
		}else { //畅通
			radar_cycle_lane_tmp[i].lane_status = 0;
		}
		radar_cycle_lane_tmp[i].lane_no = d300[m].lane_data[i].lane_no;
		radar_cycle_lane_tmp[i].queue_len_max = d300[m].lane_data[i].queue_len_max;
		radar_cycle_lane_tmp[i].car_stop_sum = d300[m].lane_data[i].car_stop_sum;

	}

	memset(&d300[m], 0, sizeof(data_300s_t));

}


m_timed_func_data camera_flow_info;// 60 s---> focus on all camera
void *callback_camera_flow_info(void *data)
{
    pthread_mutex_lock(&mutex_lock);
    time_t tm;
    tm=time(NULL);
    if(tm%60==0){
        calculate_60s();
        send_flow_info(sig_fd);
    }
    pthread_mutex_unlock(&mutex_lock);
}
//----------60s flow end------------------
//--------- outcar start------------------
ourcar_info_t ourcar;
int send_outcar_info(int fd,int cam_index, int lane_index)
{

#if 0
    ourcar.veh_speed=info.cams[cam_index].lanes[lane_index].speed;
    ourcar.lane_number=get_lane_index(cam_index,lane_index);


    int i=0;
    int j=0;
    for(i=0;i<5;i++){
        ourcar.table_head[i]=0xfe;
    }
    ourcar.table_no=0x0f;
    ourcar.table_length=18;
    ourcar.pass_time[0]=get_year_tail();
    ourcar.pass_time[1]=get_month();
    ourcar.pass_time[2]=get_day();
    ourcar.pass_time[3]=get_hour();
    ourcar.pass_time[4]=get_min();
    ourcar.pass_time[5]=get_sec();
    ourcar.dir_no=get_direction(cam_index);
    ourcar.lane_dir_type=0;
    ourcar.section_number=1;
    ourcar.veh_type=1;
    ourcar.occupy_time=info.cams[cam_index].lanes[lane_index].out_car_time-info.cams[cam_index].lanes[lane_index].in_car_time;
    ourcar.crc=get_crc((unsigned char *)&ourcar,sizeof(ourcar_info_t)-1);
    // for(i=0;i<NANJING_CAM_NUM;i++){
    //   int len =
    SendDataByTcp(fd, (char *) &ourcar, sizeof(ourcar_info_t));

    //    }
#endif
    return 1;
}

m_timed_func_data outcar_check_info;// 1ms   --> focus on 1 lane
void *callback_outcar_check_info(void *data)
{

#if 0
    int i=0;
    int j=0;
    pthread_mutex_lock(&mutex_lock);
    for(i=0;i<NANJING_CAM_NUM;i++){
        for(j=0;j<NANJING_LANE_MAX;j++){
            if(info.cams[i].lanes[j].out_car){
                if (sig_state == SIG_CONNECTED) {
                    if (send_outcar_info(sig_fd,i,j) <= 0) {
                        close_socket(&sig_fd);
                        sig_set_state(SIG_PRE_CONNECT);
                    }
                }
                info.cams[i].lanes[j].out_car=0;
            }

        }
    }
    pthread_mutex_unlock(&mutex_lock);
#endif
}
//--------- outcar end------------------





typedef void *(func)(void*);
void regist_callback(m_timed_func_data *data,func fc,int ms)
{
    data->func=fc;
    data->time=ms*1000;
    regist_timed_callback(data);
}

void init_sig_service1()
{
    init_data();
    create_detach_thread(sig_service_thread,1,NULL);

    regist_callback(&camera_queue_info,callback_camera_queue_info,1000);
    regist_callback(&camera_flow_info,callback_camera_flow_info,1000);
    regist_callback(&outcar_check_info,callback_outcar_check_info,10);

    if(sig_ip[0]!='0'){
        sig_set_state(SIG_PRE_CONNECT);
    }
}

void encode_radar(unsigned char *buff,int *size)
{
    int i=0,j=0;
    for(i=0;i<*size;i++)
    {
        if(buff[i]==0X7D)
        {
            buff[i]=0X7D;
            *size=*size+1;
            for(j=*size-1;j>i+1;j--)
            {
                buff[j]=buff[j-1];
            }
            buff[i+1]=0X5D;

        }
        else if(buff[i]==0X7E)
        {
            buff[i]=0X7D;
            *size=*size+1;
            for(j=*size-1;j>i+1;j--)
            {
                buff[j]=buff[j-1];
            }
            buff[i+1]=0X5E;
        }
    }
}

int time_seconds()
{
 int  seconds = time(NULL);
 return seconds;
}

char *int2str(int t,char *buf)
{
    snprintf(buf,100,"%d",t);
    return buf;
}


unsigned char buf_radar_result[10000];
unsigned char buf_radar_realtime_result[10000];
unsigned char buf_radar_cycle_result[10000];
#define BUF_IN_OUT_MAX 9
unsigned char buf_radar_in_out_result[BUF_IN_OUT_MAX] = {0};
radar_car_in_out_status_t radar_in_out_status[MAX_LANE_NUM] = {0};    

int get_result_info()//zhouqi  tong ji
{
    radar_head_t  head;
	int sz_head=sizeof(radar_head_t);
	unsigned char dev_id = 0;
	dev_id = (unsigned char)get_dev_id();
	dev_id = (dev_id << 2) || 0x01;
    // head data
    head.addr=dev_id;
    head.ver=0x20;
    head.type=0x82;
    head.obj=0x54;
    //
    int lanesize=get_lane_num(0);
    radar_result_t rst;
	int sz_rst=sizeof(radar_result_t);
 	//
 	rst.period[1] = (CYCLE_STATIS_TIM >>8) & 0xFF;
	rst.period[0] = CYCLE_STATIS_TIM & 0xFF;//5分钟
	int tm_run = time_seconds();
	memcpy(rst.time, (unsigned char *)&tm_run, 4);
    rst.lane_count = (unsigned char)lanesize * 2;
	//
    memset(buf_radar_result,0,10000);
    memcpy(buf_radar_result+1,&head,sz_head);
    memcpy(buf_radar_result+1+sz_head,&rst,sz_rst);
	//
    radar_result_lane_t result_lane;
	int sz_lane=sizeof(radar_result_lane_t);
	
	
    for(int i=0; i<lanesize; i++){
		for(int j = 0; j < NANJING_LANE_COIL_MAX; j++) {
			memset(&result_lane,0,sizeof(radar_result_lane_t)); //没有的值就填255     by roger
			result_lane.lane_no = (i + 1) + j*lanesize; //检测器线圈编号
			result_lane.flowA = radar_result_lane_tmp[i][j].flowA;
			result_lane.flowB = radar_result_lane_tmp[i][j].flowB;
			result_lane.flowC = radar_result_lane_tmp[i][j].flowC;
			result_lane.Occupy_rate = radar_result_lane_tmp[i][j].Occupy_rate;
			result_lane.flowSum = radar_result_lane_tmp[i][j].flowSum;
			result_lane.average_speed = radar_result_lane_tmp[i][j].average_speed;
			result_lane.average_len = radar_result_lane_tmp[i][j].average_len;
			result_lane.average_head_time = radar_result_lane_tmp[i][j].average_head_time;

			memcpy(buf_radar_result+1+sz_head+sz_rst+sz_lane*(i*2 + j), &result_lane, sz_lane);
			//memset(&result_lane,0xFF,sizeof(radar_result_lane_t));
			//result_lane.lane_no = i*2 + 2;
			//memcpy(buf_radar_result+1+sz_head+sz_rst+sz_lane*(i*2 + 1), &result_lane, sz_lane);
	        //memcpy(buf_radar_result+1+sz_head+sz_rst+i*sz_lane, &result_lane, sz_lane);
		}
    }
	
    return sz_head+sz_rst+lanesize*sz_lane*2; //有双线圈 by roger

}


int get_cycle_statis_result_info()//add by roger 20190109
{
    radar_head_t  head;
	int sz_head=sizeof(radar_head_t);
	unsigned char dev_id = 0;
	dev_id = (unsigned char)get_dev_id();
	dev_id = (dev_id << 2) || 0x01;
    // head data
	head.addr=dev_id;
    head.ver=0x20;
    head.type=0x82;
    head.obj=0x55;
    //
    int lanesize=get_lane_num(0);
    radar_result_t rst;
	int sz_rst=sizeof(radar_result_t);
 	//
 	rst.period[1] = (CYCLE_STATIS_TIM >>8) & 0xFF;
	rst.period[0] = CYCLE_STATIS_TIM & 0xFF;//5分钟
	int tm_run = time_seconds();
	memcpy(rst.time, (unsigned char *)&tm_run, 4);
    rst.lane_count = (unsigned char)lanesize;
	//
    memset(buf_radar_cycle_result,0,10000);
    memcpy(buf_radar_cycle_result+1,&head,sz_head);
    memcpy(buf_radar_cycle_result+1+sz_head,&rst,sz_rst);
	//
    radar_cycle_result_lane_t result_lane;
	int sz_lane=sizeof(radar_cycle_result_lane_t);

    for(int i=0; i<lanesize; i++){
		memset(&result_lane,0,sizeof(radar_cycle_result_lane_t)); //没有的值就填255     by roger
		result_lane.lane_no = get_lane_index(0,i); //车道号
		result_lane.queue_len_max = radar_cycle_lane_tmp[i].queue_len_max;
		result_lane.lane_status = radar_cycle_lane_tmp[i].lane_status;
		result_lane.car_stop_sum = radar_cycle_lane_tmp[i].car_stop_sum;
		result_lane.average_delay_ms = 0;
		result_lane.oil_consume = 0;
		result_lane.gas_emission = 0;

		memcpy(buf_radar_cycle_result+1+sz_head+sz_rst+sz_lane*i, &result_lane, sz_lane);
    }
	
    return sz_head+sz_rst+lanesize*sz_lane; 

}


int get_realtime_result_info()//tong ji
{
    radar_head_t  head;
	int sz_head=sizeof(radar_head_t);
	unsigned char dev_id = 0;
	dev_id = (unsigned char)get_dev_id();
	dev_id = (dev_id << 2) || 0x01;
    // head data
        head.ver=0x20;
        head.type=0x82;
        head.obj=0x56;
		head.addr= dev_id;
    //

    int lanesize=get_lane_num(0);
    radar_realtime_t rst;
	int sz_rst=sizeof(radar_realtime_t);

    int tm_run = time_seconds();
	memcpy(rst.time, &tm_run, 4);
    rst.lane_count=lanesize;
	radar_realtime_lane_t lane[lanesize] = {0};
    int i;
    for(i=0;i<lanesize;i++){
		lane[i].lane_no=get_lane_index(0,i);
        lane[i].queue_len=info.cams[0].lanes[i].queue_len;
  		lane[i].head_len = info.cams[0].lanes[i].queue_head_len;
		lane[i].tail_len = info.cams[0].lanes[i].queue_tail_len;
		lane[i].queue_no = info.cams[0].lanes[i].queue_no;
		lane[i].lane_vihicle_count = info.cams[0].lanes[i].veh_no;
		lane[i].ocuppy  = info.cams[0].lanes[i].ocupation_ratio;
		lane[i].average_speed = info.cams[0].lanes[i].average_speed;
		lane[i].location = info.cams[0].lanes[i].locate;
		lane[i].head_pos = info.cams[0].lanes[i].head_veh_pos;
		lane[i].head_speed = info.cams[0].lanes[i].head_veh_speed;
		lane[i].tail_pos = info.cams[0].lanes[i].tail_veh_pos;
		lane[i].tail_speed = info.cams[0].lanes[i].tail_veh_speed;
    }

    int sz_lane=sizeof(radar_realtime_lane_t);

    memset(buf_radar_realtime_result,0,10000);
    memcpy(buf_radar_realtime_result+1,&head,sz_head);
    memcpy(buf_radar_realtime_result+1+sz_head,&rst,sz_rst);

    for(int i=0;i<lanesize;i++){
        memcpy(buf_radar_realtime_result+1+sz_head+sz_rst+i*sz_lane,&lane[i],sz_lane);
    }

		
    return sz_head+sz_rst+lanesize*sz_lane;
}

int get_car_in_out_info() //获取入车和出车的状态和数据
{
	bool send_flag = false;
	int pack_len = 0;
	int lane_size=get_lane_num(0);
	radar_car_in_out_result_t  car_in_out;
	radar_rt_lane_car_in_out_info_t list_in_out_item;
	
	if ( !get_car_in_out_item(&list_in_out_item) )
		return 0;
	
	memset(buf_radar_in_out_result, 0, BUF_IN_OUT_MAX);
	buf_radar_in_out_result[0] = 0xEE; //head flag
		
	for(int i = 0; i < lane_size; i++) {
		memset(&car_in_out, 0, sizeof(radar_car_in_out_result_t));
		if (list_in_out_item.lanes[i].in_flag > 0)
			car_in_out.in_out_flag = 1; //入车
		else if (list_in_out_item.lanes[i].out_flag > 0)
			car_in_out.in_out_flag = 2; //出车
		else
			car_in_out.in_out_flag = 0;

		if (radar_in_out_status[i].flag != car_in_out.in_out_flag) //车道值有不同才发送udp数据
			send_flag = true;
	
		radar_in_out_status[i].flag = car_in_out.in_out_flag;
		car_in_out.lane_no = get_lane_index(0,i);
		memcpy(buf_radar_in_out_result +1+ pack_len, &car_in_out, sizeof(radar_car_in_out_result_t));
		pack_len += sizeof(radar_car_in_out_result_t);
		
	}

	if (!send_flag) //数据没变化
		return 0;
	
	return BUF_IN_OUT_MAX; //只有9个字节
}



m_timed_func_data radar_result_info;// 1 s ---> focus on all camera
m_timed_func_data radar_realtime_result_info;// 1 s ---> focus on all camera
m_timed_func_data radar_cycle_statis_result_info;// 1 s ---> focus on all camera
m_timed_func_data radar_car_in_out_result_info;// 1 s ---> 车辆入出


void *callback_radar_realtime_result(void *data)
{
    pthread_mutex_lock(&mutex_lock);
    if (sig_state == SIG_CONNECTED) {
        int len=get_realtime_result_info();
		
        buf_radar_realtime_result[0]=0x7e;
        *(buf_radar_realtime_result+1+len)=get_crc(buf_radar_realtime_result+1,len);
		len++;//校验码
     	encode_radar(buf_radar_realtime_result+1,&len);
		*(buf_radar_realtime_result+1+len)=0x7e;
        if (send_radar(sig_fd,(char *)buf_radar_realtime_result,len+2) <= 0) {
            close_socket(&sig_fd);
            sig_set_state(SIG_PRE_CONNECT);
		}
    }

    pthread_mutex_unlock(&mutex_lock);

}


void *callback_radar_result(void *data)
{
    pthread_mutex_lock(&mutex_lock);
    time_t tm;
    tm=time(NULL);

    if(tm%CYCLE_STATIS_TIM==0){
		calculate_60s_radar();
        if (sig_state == SIG_CONNECTED) {
            int len=get_result_info();
			buf_radar_result[0] = 0x7e;
		    *(buf_radar_result+1+len)=get_crc(buf_radar_result+1,len);
			len++;//校验码
            encode_radar(buf_radar_result+1,&len);
			*(buf_radar_result+1+len)=0x7e;
            if (send_radar(sig_fd,(char *)buf_radar_result,len+2) <= 0) {
                close_socket(&sig_fd);
                sig_set_state(SIG_PRE_CONNECT);
				
            }
        }
    }
    pthread_mutex_unlock(&mutex_lock);
}


void *callback_cycle_statis_result(void *data)
{
    pthread_mutex_lock(&mutex_lock);
    time_t tm;
    tm=time(NULL);

    if(tm%CYCLE_STATIS_TIM==0){
		calculate_300s_radar();
        if (sig_state == SIG_CONNECTED) {
            int len=get_cycle_statis_result_info();
			buf_radar_cycle_result[0] = 0x7e;
		    *(buf_radar_cycle_result+1+len)=get_crc(buf_radar_cycle_result+1,len);
			len++;//校验码
            encode_radar(buf_radar_cycle_result+1,&len);
			*(buf_radar_cycle_result+1+len)=0x7e;
			
            if (send_radar(sig_fd,(char *)buf_radar_cycle_result,len+2) <= 0) {
                close_socket(&sig_fd);
                sig_set_state(SIG_PRE_CONNECT);
				
            }
        }
    }
    pthread_mutex_unlock(&mutex_lock);
}


extern int fd_udp_car_in_out;
void *callback_car_in_out_result(void *data)
{
   // pthread_mutex_lock(&mutex_lock);

		int len = get_car_in_out_info();
   		char ip[IPADDRMAX];
		unsigned short port = 0;
		get_udp_info(ip, &port);

		if (len > 0 && fd_udp_car_in_out > 0) {
			UdpSendData(fd_udp_car_in_out , ip, port, (char *)buf_radar_in_out_result, len);
		}
		
   // pthread_mutex_unlock(&mutex_lock);
}



void init_sig_service2()
{
	init_data();
    create_detach_thread(sig_service_thread,1,NULL);

    regist_callback(&radar_realtime_result_info,callback_radar_realtime_result,1000);
    regist_callback(&radar_result_info,callback_radar_result,1000);
	regist_callback(&radar_cycle_statis_result_info,callback_cycle_statis_result,1000);
	regist_callback(&radar_car_in_out_result_info,callback_car_in_out_result,0);
	
    if(sig_ip[0]!='0'){
        sig_set_state(SIG_PRE_CONNECT);
    }
}
