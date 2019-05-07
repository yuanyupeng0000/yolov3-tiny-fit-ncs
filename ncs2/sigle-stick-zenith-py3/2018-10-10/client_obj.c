/*
 * protocol.c
 *
 *  Created on: 2016年8月17日
 *      Author: root
 */
#include "common.h"
#include "client_obj.h"

#include "g_define.h"
#include <arpa/inet.h>
//#include "file_op.h"
//
//#include "camera_service.h"
//#include "client_file.h"
//typedef struct cam_cfg {
//	mCamParam cam_param;
//	mCamDetectParam det_param;
//} m_cam_cfg;
//m_cam_cfg g_cam_cfg[CAM_NUM];
//mDetectDeviceConfig g_dev_cfg;
//mDetectDeviceConfig device_default_config={
//		.deviceID=1,
//		.detectport=8888,
//		.camnum=1,
//		.mode=0,
//		.camstatus={1,0,0,0},
//		.camdirect={1,1,1,1},
//		.cammerIp={"192.168.1.81","192.168.1.81","192.168.1.81","192.168.1.81"},
//		"192.168.1.113",
//		"testname"
//};
//mCamAttributes attr_dft={
//		.direction=2,
//		.camID=22,
//		.cammerport=5000,
//		.adjustport=5000,
//		5000,
//		"url:test",
//		"admin",
//		"admin",
//		.cammerIp={'1','9','2','.','1','6','8','.','1','.','8','9'},
//		"0.0.0.1",
//		"0.0.0.2",
//};
//mCamDemarcateParam demarcate_dft={
//		1,
//		2,
//		3,
//		4,
//		5,
//		6,
//		7
//};
//mChannelVirtualcoil virtual_coil_dft={
//		0,
//		0,
//		0
//};
//int a=2;
//mCamParam camera_default_config={
//		.coilnum=2,
//		.camattr=attr_dft,
//		demarcate_dft,
////		virtual_coil_dft
//};
//
//unsigned int   g_timep[4]; //0 1代表凌晨 点1至点2   2 3代表 黄昏 点1至点2
//mCamDetectLane g_detectlane;        //检测车道数
//mVirtualLaneLine  g_laneline;      //用到的车道线
//mStandardPoint g_standpoint[STANDARDVAULEMAX];       //标定点和坐标
//mDemDetectArea g_area;              //标定的区域
//mDetectParam g_detectparam[ALGMAX];   // 0  白天的参数,  1代表晚上参数
//mCamDetectParam camera_det_default_config={
//		g_timep[0]=0,
//		g_timep[0]=1,
//		g_timep[0]=2,
//		g_timep[0]=3,
//		g_detectlane,
//		g_laneline,
//		g_standpoint[0],
//		g_standpoint[1],
//		g_standpoint[2],
//		g_standpoint[3],
//		g_area,
//		g_detectparam[0],
//		g_detectparam[1],
//};
//void net_decode_obj_n(unsigned char *addr,int type,int encode,int num,int size);
//#define WEAK __attribute__((weak))
//void net_decode_obj(char *bf,int type,int encode) WEAK;
typedef struct pointers {
	mCamAttributes *p_mCamAttributes;
	mCommand *p_mCommand;
	short *p_short;
	int *   p_int;
	mDetectDeviceConfig *p_mDetectDeviceConfig;
	mCamParam *p_mCamParam;
	mCamDemarcateParam *p_mCamDemarcateParam;
	mChannelVirtualcoil *p_mChannelVirtualcoil;
	mCamDetectParam *p_mCamDetectParam;
	mCamDetectLane *p_mCamDetectLane;
	mChannelCoil *p_mChannelCoil;
	mPoint *p_mPoint;
	mLine *p_mLine;
	mVirtualLaneLine *p_mVirtualLaneLine;
	mStandardPoint *p_mStandardPoint;
	mDemDetectArea *p_mDemDetectArea;
	mDetectParam *p_mDetectParam;

} m_pointers;
//m_list_info *ip_list;
//typedef struct client_ip_data {
//	char ip[16];
//	int camera_index;
//	int time;
//	int fd;
//} m_client_ip_data;
//enum{
//	CLIENT_IP_SAME=0,
//	CLIENT_CAM_INDEX_SAME
//};
//int client_info_match(void *ori_info,void *new_info)
//{
//	m_client_ip_data *p1=(m_client_ip_data *)ori_info;
//	m_client_ip_data *p2=(m_client_ip_data *)new_info;
//
//	if(!memcmp(p1->ip,p2->ip,16)){
//		if(p1->camera_index==p2->camera_index)
//			return CLIENT_CAM_INDEX_SAME;
//		else
//			return CLIENT_IP_SAME;
//	}
//	return -1;
//}

void net_decode_obj_n(unsigned char *addr,int type,int encode,int num,int size);
int get_obj_len(int class_type)
{
	int len=0;
	switch(class_type){
	case CLASS_NULL:
		break;
	case CLASS_char:
		break;
	case CLASS_short:
		len=2;
		break;
	case CLASS_int:
		len=4;
		break;
	case CLASS_mCommand:
	    len=sizeof(mCommand);
				break;
	case CLASS_mDetectDeviceConfig:
		len=sizeof(mDetectDeviceConfig);
		break;
	case CLASS_mCamParam:
		len=sizeof(mCamParam);
		break;
	case CLASS_mCamAttributes:
		len=sizeof(mCamAttributes);
		break;

	case CLASS_mCamDemarcateParam:
		len=sizeof(mCamDemarcateParam);
				break;
	case CLASS_mChannelVirtualcoil:
		len=sizeof(mChannelVirtualcoil);
		break;
	case CLASS_mCamDetectParam:
		len=sizeof(mCamDetectParam);

		break;

	case	CLASS_mCamDetectLane :
		len=sizeof(mCamDetectLane );
		break;
	case	CLASS_mChannelCoil:
		len=sizeof(mChannelCoil);
		break;
	case	CLASS_mPoint:
		len=sizeof(mPoint);
		break;
	case	CLASS_mLine:
		len=sizeof(mLine);
		break;
	case	CLASS_mVirtualLaneLine:
		len=sizeof(mVirtualLaneLine);
		break;
	case	CLASS_mStandardPoint:
		len=sizeof(mStandardPoint);
		break;
	case	CLASS_mDemDetectArea:
		len=sizeof(mDemDetectArea);
		break;
	case	CLASS_mDetectParam:
		len=sizeof(mDetectParam);
		break;
	default:
		prt(info,"not recognize");
		break;
	}
	return len;
}
void net_decode_obj(unsigned char *bf,int type,int encode)
{
	m_pointers pt;
//	char *p;
	switch(type){
	case CLASS_NULL:
		break;
	case CLASS_char:
		break;
	case CLASS_short:
	    pt.p_short=(short *)bf;
		if (!encode)
			{
		//		prt(info,"from short %d",*(pt.p_short));
			 *pt.p_short = ntohs(*pt.p_short);
		//		prt(info,"to short %d",*pt.p_short);
			}
		else
			{
		// 	prt(info,"a %d",*pt.p_short);
			*pt.p_short = htons(*pt.p_short);
		// 	prt(info,"b %d",*pt.p_short);
			}
		break;
	case CLASS_int:
		pt.p_int = (int *) bf;
		if (!encode)
			*pt.p_int = ntohl(*pt.p_int);
		else
			*pt.p_int = htonl(*pt.p_int);
		break;
	case CLASS_mCommand:
	    pt.p_mCommand=(mCommand *)bf;
		net_decode_obj((unsigned char *)& pt.p_mCommand->version,CLASS_char,encode);
		net_decode_obj((unsigned char *)& pt.p_mCommand->prottype,CLASS_char,encode);
		net_decode_obj((unsigned char *)& pt.p_mCommand->objnumber,CLASS_short,encode);
		net_decode_obj((unsigned char *)& pt.p_mCommand->objtype,CLASS_short,encode);
		net_decode_obj((unsigned char *)& pt.p_mCommand->objlen,CLASS_int,encode);
		break;
	case CLASS_mDetectDeviceConfig:
		pt.p_mDetectDeviceConfig=(mDetectDeviceConfig *)bf;
		net_decode_obj((unsigned char *)&pt.p_mDetectDeviceConfig->deviceID,CLASS_int,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectDeviceConfig->detectport,CLASS_int,encode);
		break;
	case CLASS_mCamParam:
		pt.p_mCamParam=(mCamParam *)bf;
		net_decode_obj((unsigned char *)&pt.p_mCamParam->camattr,CLASS_mCamAttributes,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamParam->camdem,CLASS_mCamDemarcateParam,encode);
		net_decode_obj_n((unsigned char *)pt.p_mCamParam->channelcoil,CLASS_mChannelVirtualcoil,encode,4,sizeof(mChannelVirtualcoil));
		break;
	case CLASS_mCamAttributes:
		pt.p_mCamAttributes=(mCamAttributes *)bf;
		net_decode_obj((unsigned char *)&(pt.p_mCamAttributes->camID),CLASS_int,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamAttributes->cammerport,CLASS_int,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamAttributes->adjustport,CLASS_int,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamAttributes->signalport,CLASS_int,encode);
		break;

	case CLASS_mCamDemarcateParam:
		pt.p_mCamDemarcateParam=(mCamDemarcateParam *)bf;
		net_decode_obj((unsigned char *)&pt.p_mCamDemarcateParam->cam2stop,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamDemarcateParam->camheight,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamDemarcateParam->lannum,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamDemarcateParam->number,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamDemarcateParam->baselinelen,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamDemarcateParam->farth2stop,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamDemarcateParam->recent2stop,CLASS_short,encode);
				break;
	case CLASS_mChannelVirtualcoil:
		pt.p_mChannelVirtualcoil=(mChannelVirtualcoil *)bf;
		net_decode_obj((unsigned char *)&pt.p_mChannelVirtualcoil->farthCoillen,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mChannelVirtualcoil->recentCoillen,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mChannelVirtualcoil->number,CLASS_short,encode);
		break;
	case CLASS_mCamDetectParam:
		pt.p_mCamDetectParam=(mCamDetectParam *)bf;
	//	prt(info,"!!!%d",pt.p_mCamDetectParam->area.vircoordinate[2].x);
		//prt(info,"!!!%d",pt.p_mCamDetectParam->area.vircoordinate[2].y);
		net_decode_obj_n((unsigned char *)pt.p_mCamDetectParam->timep,CLASS_int,encode,4,sizeof(int));

		net_decode_obj((unsigned char *)&pt.p_mCamDetectParam->detectlane,CLASS_mCamDetectLane,encode);
		net_decode_obj((unsigned char *)&pt.p_mCamDetectParam->laneline,CLASS_mVirtualLaneLine,encode);

		net_decode_obj_n((unsigned char *)&pt.p_mCamDetectParam->standpoint,CLASS_mStandardPoint,encode,4,sizeof(mStandardPoint));

		net_decode_obj((unsigned char *)&pt.p_mCamDetectParam->area,CLASS_mDemDetectArea,encode);
		net_decode_obj_n((unsigned char *)pt.p_mCamDetectParam->detectparam,CLASS_mDetectParam,encode,2,sizeof(mDetectParam));

	//	prt(info,"!!%d",pt.p_mCamDetectParam->area.vircoordinate[2].x);
	//	prt(info,"!!%d",pt.p_mCamDetectParam->area.vircoordinate[2].y);
				break;

	case	CLASS_mCamDetectLane :
		pt.p_mCamDetectLane=(mCamDetectLane *)bf;
		net_decode_obj((unsigned char *)&pt.p_mCamDetectLane->lanenum,CLASS_char,encode);
		net_decode_obj_n((unsigned char *)pt.p_mCamDetectLane->virtuallane,CLASS_mChannelCoil,encode,DETECTLANENUMMAX,sizeof(mChannelCoil));

		break;
	case	CLASS_mChannelCoil:
		pt.p_mChannelCoil=(mChannelCoil *)bf;
		net_decode_obj_n((unsigned char *)pt.p_mChannelCoil->RearCoil,CLASS_mPoint,encode,COILPOINTMAX,sizeof(mPoint));
		net_decode_obj_n((unsigned char *)pt.p_mChannelCoil->FrontCoil,CLASS_mPoint,encode,COILPOINTMAX,sizeof(mPoint));

		break;
	case	CLASS_mPoint:
		pt.p_mPoint=(mPoint *)bf;
	//	prt(info,"from point %d,encode %d,addr %p",pt.p_mPoint->x,encode,&pt.p_mPoint);
		net_decode_obj((unsigned char *)&pt.p_mPoint->x,CLASS_short,encode);
	//	prt(info,"to point %d",pt.p_mPoint->x);
		net_decode_obj((unsigned char *)&pt.p_mPoint->y,CLASS_short,encode);

		break;
	case	CLASS_mLine:
		pt.p_mLine=(mLine *)bf;
		net_decode_obj((unsigned char *)&pt.p_mLine->startx,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mLine->starty,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mLine->endx,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mLine->endy,CLASS_short,encode);

		break;
	case	CLASS_mVirtualLaneLine:
		pt.p_mVirtualLaneLine=(mVirtualLaneLine *)bf;
		net_decode_obj((unsigned char *)&pt.p_mVirtualLaneLine->lanelinenum,CLASS_char,encode);
		net_decode_obj_n((unsigned char *)&pt.p_mVirtualLaneLine->laneline,CLASS_mLine,encode,LANELINEMAX,sizeof(mLine));

		break;
	case	CLASS_mStandardPoint:
		pt.p_mStandardPoint=(mStandardPoint *)bf;

		net_decode_obj((unsigned char *)&pt.p_mStandardPoint->coordinate,CLASS_mPoint,encode);
		net_decode_obj((unsigned char *)&pt.p_mStandardPoint->value,CLASS_short,encode);


		break;
	case	CLASS_mDemDetectArea:
		pt.p_mDemDetectArea=(mDemDetectArea *)bf;

	//	prt(info," ###########################from !!!!!!!%d,size %d,addr %p",
		//		(unsigned short)pt.p_mDemDetectArea->vircoordinate[0].x,sizeof(mPoint),
	//			&(pt.p_mDemDetectArea->vircoordinate[0]));
		net_decode_obj_n((unsigned char *)&(pt.p_mDemDetectArea->vircoordinate[0]),CLASS_mPoint,encode,STANDPOINT,sizeof(mPoint));

	//	prt(info," ###################to !!!!!!!%d",pt.p_mDemDetectArea->vircoordinate[0].x);
		net_decode_obj_n((unsigned char *)pt.p_mDemDetectArea->realcoordinate,CLASS_mPoint,encode,STANDPOINT,sizeof(mPoint));

		break;
	case	CLASS_mDetectParam:
		pt.p_mDetectParam=(mDetectParam *)bf;
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uTransFactor,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uGraySubThreshold,CLASS_int,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uSpeedCounterChangedThreshold,CLASS_int,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uSpeedCounterChangedThreshold1,CLASS_int,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uSpeedCounterChangedThreshold2,CLASS_int,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uDayNightJudgeMinContiuFrame,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uComprehensiveSens,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uDetectSens1,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uDetectSens2,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uStatisticsSens1,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uStatisticsSens2,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->uSobelThreshold,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->shutterMax,CLASS_short,encode);
		net_decode_obj((unsigned char *)&pt.p_mDetectParam->shutterMin,CLASS_short,encode);
		break;
	default:
		prt(info,"not recognize");
		break;
	}
}
void net_decode_obj_n(unsigned char *addr,int type,int encode,int num,int size)
{
	int i=0;
	for(i=0;i<num;i++){
	//	prt(info,"add  %p,size %d,num %d",(unsigned char *)(addr+i*size),size,num);
		net_decode_obj((unsigned char *)addr+i*size,type,encode);
	}
}

int prepare_pkt(unsigned char *p_start, int head_length,int reply_type, int class_type, int class_length,unsigned char *p_obj)
{
	int total_length;
	mCommand *p_head_obj=(mCommand *)p_start;
	p_head_obj->objtype=reply_type;
	unsigned char *p_obj_start=p_start+head_length;
	memcpy(p_obj_start,p_obj,class_length);
    net_decode_obj(p_obj_start,class_type,1);
    total_length=head_length+class_length;
	return total_length;
}
int handle_pkt(unsigned char *p_start, int head_length, int class_type, int class_length)
{
	int total_length;
	mCommand *p_head_obj=(mCommand *)p_start;
//	p_head_obj->objtype=reply_type;
	unsigned char *p_obj_start=p_start+head_length;
	//memcpy(p_obj_start,p_obj,class_length);
    net_decode_obj(p_obj_start,class_type,0);
   // total_length=head_length+class_length;
	return 0;
}
int get_pkt(unsigned char *p_start, int head_length,int reply_type, int class_type, int class_length,unsigned char *p_obj)
{
	int total_length;
	mCommand *p_head_obj=(mCommand *)p_start;
	p_head_obj->objtype=reply_type;
	unsigned char *p_obj_start=p_start+head_length;
	memcpy(p_obj_start,p_obj,class_length);
    net_decode_obj(p_obj_start,class_type,1);
    total_length=head_length+class_length;
	return total_length;
}

//#define FILE_NAME_LENGTH 20
//char cfg_dir[]="./cfg/";
//char *cfg_file_name(char *filename,int class_type)
//{
//	switch(class_type){
//	case CLASS_mCamParam:
//		sprintf(filename,"./cfg/%s","cam_param");
//		break;
//	case CLASS_mDetectDeviceConfig:
//		sprintf(filename,"./cfg/%s","dev_config");
//		break;
//	case CLASS_mCamDetectParam:
//		sprintf(filename,"./cfg/%s","det_param");
//		break;
//
//	default :
//		prt(info,"unsupported save class %d",class_type);
//		break;
//	}
//	return filename;
//}
//void save_obj(unsigned char * p_obj,int class_type,int index)
//{
//	int pos=0;
//	char filename[FILE_NAME_LENGTH];
//	int len=get_obj_len(class_type);
//	if(class_type==CLASS_mDetectDeviceConfig){
//		pos=0;
//	}else{
//		pos=index*len;
//	}
//	char *p_dst=NULL;
//	switch(class_type){
//	case CLASS_mCamParam:
//		//memcpy(&g_cam_cfg[index],p_obj,len);
//		p_dst=(char *)&g_cam_cfg[index].cam_param;
//		sprintf(filename,"./cfg/%s","cam_param");
//		break;
//	case CLASS_mDetectDeviceConfig:
//		p_dst=(char *)&g_dev_cfg;
//		sprintf(filename,"./cfg/%s","dev_config");
//		break;
//	case CLASS_mCamDetectParam:
//		p_dst=(char *)&g_cam_cfg[index].det_param;
//		sprintf(filename,"./cfg/%s","det_param");
//		prt(info,"===11111>%d====>>> %d",g_cam_cfg[index].det_param.area.vircoordinate[2].x,((mCamDetectParam*)p_obj)->area.vircoordinate[2].x);
//		break;
//
//	default :
//		prt(info,"unsupported save class %d",class_type);
//		break;
//	}
//	memcpy(p_dst,p_obj,len);
//	save_buf(filename,(char *)p_obj,pos,len);
//}
//void handle_change(unsigned char * p_obj,int class_type,int index)
//{
//	char *p_old_obj=NULL;
//	switch(class_type){
//	case CLASS_mCamParam:
//		prt(info,"saving cam param,checking alg change,sigmachine change , ntp change");
//		//memcpy(&g_cam_cfg[index],p_obj,len);
////		prt(info,"11change ip  %s  to %s ", g_cam_cfg[index].cam_param.camattr.cammerIp ,
////						(char *)((mCamParam *)p_obj)->camattr.cammerIp);
//		p_old_obj=(char *)&g_cam_cfg[index].cam_param;
//		camera_ctrl(CAMERA_CONTROL_RESET_ALG,index,0,NULL);
//		if(strcmp((char *)g_cam_cfg[index].cam_param.camattr.cammerIp,
//				(char *)((mCamParam *)p_obj)->camattr.cammerIp)){
//			prt(info,"change ip  %s  to %s ", g_cam_cfg[index].cam_param.camattr.cammerIp ,
//					(char *)((mCamParam *)p_obj)->camattr.cammerIp);
//			mCamParam *tmp=(mCamParam *)p_obj;
//		//	camera_open(index,(char *)tmp->camattr.cammerIp,tmp->camattr.cammerport,(char *)tmp->camattr.username,(char *)tmp->camattr.passwd);
////			camera_open(index,(char *)((mCamParam *)p_obj).cam_param.camattr.cammerIp,g_cam_cfg[index].cam_param.camattr.cammerport,
////					(char *)g_cam_cfg[index].cam_param.camattr.username,(char *)g_cam_cfg[index].cam_param.camattr.passwd);
//
//			camera_ctrl(CAMERA_CONTROL_RESET,index,0,NULL);
//		}
//
//		break;
//		/*
//		 *配置相机开、关状态。检测机名称等
//		 */
//	case CLASS_mDetectDeviceConfig:
//		prt(info,"saving det dev,checking camera status , direction");
//		p_old_obj=(char *)&g_dev_cfg;
//		if(strcmp((char *)g_dev_cfg.detectname,(char *)((mDetectDeviceConfig *)p_obj)->detectname)){
//			prt(info,"change name  %s  to %s ",g_dev_cfg.detectname,((mDetectDeviceConfig *)p_obj)->detectname);
//		}
//
//
//		for (int i = 0; i < CAM_NUM; i++) {
//			if ((g_dev_cfg.camstatus[i] == 0)
//					&& ((mDetectDeviceConfig *) p_obj)->camstatus[i] == 1) {
//				prt(info,"open cam %d",i);
//
//				camera_open(i,(char *)g_cam_cfg[i].cam_param.camattr.cammerIp,
//						g_cam_cfg[i].cam_param.camattr.cammerport,
//						(char *)g_cam_cfg[i].cam_param.camattr.username,
//						(char *)g_cam_cfg[i].cam_param.camattr.passwd);
//
//			}
//
//			if ((g_dev_cfg.camstatus[i] == 1)
//					&& ((mDetectDeviceConfig *) p_obj)->camstatus[i] == 0) {
//				prt(info,"close cam %d",i);
//
//				camera_close(i);
//			}
//
//			if(strcmp((char *)g_dev_cfg.cammerIp[i],(char *)((mDetectDeviceConfig *)p_obj)->cammerIp[i])){
//
//				if(((mDetectDeviceConfig *) p_obj)->camstatus[i] == 1){
//					prt(info," ip from  %s  to %s ",(char *)g_dev_cfg.cammerIp[i],(char *)((mDetectDeviceConfig *)p_obj)->cammerIp[i]);
//					camera_ctrl(CAMERA_CONTROL_RESET,i,0,NULL);
//				}
//			}
//
//
//		}
//	//	camera_ctrl(CAMERA_CONTROL_RESET_ALG,index,0,NULL);
//		break;
//	case CLASS_mCamDetectParam:
//		prt(info,"saving det param reset alg,checking alg change");
//		prt(info,"===11111>%d",g_cam_cfg[index].det_param.area.vircoordinate[2].x);
//		p_old_obj=(char *)&g_cam_cfg[index].det_param;
//		camera_ctrl(CAMERA_CONTROL_RESET_ALG,index,0,NULL);
//		break;
//	default :
//		break;
//	}
//}
//mRealStaticInfo static_info[CAM_NUM];
//mRealStaticInfo *client_get_info(int index)
//{
//	return &static_info[index];
//}
//#include "csocket.h"
//void *client_send_udp(void *node_data,void *data)
//{
//	m_client_ip_data *p_data=(m_client_ip_data *)node_data;
//	int *p_index=(int *)data;
//	int index=*p_index;
//	//prt(info,"get index %d",index);
//		if (p_data->camera_index == index) {
//			UdpSendData(p_data->fd, p_data->ip, SERVERPORT, (char*) &static_info[index],sizeof(mRealStaticInfo));
//		}
//}
//
//void client_output(int index)
//{
//	list_operate_node_all(ip_list,(p_func )client_send_udp,(void *)&index);
////	if (fd <= 0) {
////		fd = UdpCreateSocket(UDP_LOCAL_PORT + index);
////	}
////
////	m_client_ip_data *p_data = (m_client_ip_data *) data;
////	if (p_data->camera_index == index) {
////		UdpSendData(fd, p_data->ip, SERVERPORT, (char*) p_real_data,sizeof(mRealStaticInfo));
////	}
//}
//#include "csocket.h"
//void add_client(char *ip,int index)
//{
//	m_client_ip_data *p_tmp;
//	list_node_alloc_tail(ip_list);
//	p_tmp=(m_client_ip_data *)ip_list->tail->data;
//	p_tmp->camera_index=index;
//	memcpy(p_tmp->ip, ip,16);
//	prt(info,"add :%s",p_tmp->ip);
//	p_tmp->fd = UdpCreateSocket(UDP_LOCAL_PORT + index);
//}
//void del_client()
//{
//
//}
//void sync_obj(unsigned char * p_obj,int class_type,int index)
//{
//	int pos=0;
//	char filename[FILE_NAME_LENGTH];
//	int len=get_obj_len(class_type);
//	if(class_type==CLASS_mDetectDeviceConfig){
//		pos=0;
//	}else{
//		pos=index*len;
//	}
//	char *p_dst=NULL;
//	switch(class_type){
//	case CLASS_mCamParam:
//		p_dst=(char *)&g_cam_cfg[index].cam_param;
//		break;
//	case CLASS_mDetectDeviceConfig:
//		p_dst=(char *)&g_dev_cfg;
//		break;
//	case CLASS_mCamDetectParam:
//		p_dst=(char *)&g_cam_cfg[index].det_param;
//		break;
//
//	default :
//		prt(info,"unsupported save class %d",class_type);
//		break;
//	}
//	memcpy(p_dst,p_obj,len);
//}
//int handle_buffer(unsigned char *buf, int len, char *ip)
//{
//	int type;
//	int length;
//	int ret=-1;
//	int match_ret;
//	mCommand *cmd_p;
// 	cmd_p = (mCommand *) buf;
//	net_decode_obj((unsigned char *) cmd_p, CLASS_mCommand, 0);
//	type = cmd_p->objtype;
//	int cmd_len = sizeof(mCommand);
//	int index=cmd_p->objnumber;
//	prt(info, "get cmd type %x, length %d,index %d", type,len,cmd_p->objnumber);
//	m_client_ip_data tmp_data;
//	m_client_ip_data *current_data;
//	tmp_data.camera_index=index;
//	memcpy(tmp_data.ip,ip,16);
//	if((match_ret=list_node_seek(ip_list,(void *)&tmp_data))>=CLIENT_IP_SAME){
//	//	prt(info,":%s    ",tmp_data.ip);
//		if(match_ret!=CLIENT_CAM_INDEX_SAME){
//			list_overwirte_current_data(ip_list,(void *)&tmp_data);
//		//	current_data=(m_client_ip_data *)list_get_current_data(ip_list);
//
//		//	current_data->camera_index=tmp_data.camera_index;
//		//	memcpy(current_data->ip,tmp_data.ip,16);
//		//	prt(info,":%s index change to %d  ",current_data->ip,tmp_data.camera_index);
//		}else{
//			//prt(info,":%s index change to %d  ",current_data->ip,tmp_data.camera_index);
//		}
//
//	}else{
////		list_node_alloc_tail(ip_list);
////		current_data=(m_client_ip_data *)ip_list->tail->data;
////		current_data->camera_index=tmp_data.camera_index;
////		memcpy(current_data->ip,tmp_data.ip,16);
////		prt(info,"add :%s",current_data->ip);
//
//		add_client(tmp_data.ip,tmp_data.camera_index);
//	}
//	int reply_type=0;
//	int class_type=0;
//	unsigned char *p_obj=NULL;
//	int class_len=0;
//	int flg=NOOP_FLAG;
//	switch (type) {
//
////	m_cam_cfg g_cam_cfg[CAM_NUM];
////	mDetectDeviceConfig g_dev_cfg;
//
//	case SETCAMPARAM:
//		class_type=CLASS_mCamParam;
//		flg=SET_FLAG;
//		break;
//	case SETDETECTDEVICE:
//		class_type=CLASS_mDetectDeviceConfig;
//		flg=SET_FLAG;
//		break;    //设置检测设备参数命名
//	case SETCHECHPARAM:
//		class_type=CLASS_mCamDetectParam;
////		class_len=sizeof(mDetectDeviceConfig);
////		class_len=sizeof(mCamParam);
//		flg=SET_FLAG;
//		break;   //设置视频画线参数
//
////	case REPCAMPARAM:
////		break;	  //获取相机参数命名回应
////	case REPDETECTDEVICE:
////		break;    //获取检测设备参数命名回应
//	case GETCAMPARAM:
//		reply_type=REPCAMPARAM;
//		class_type=CLASS_mCamParam;
//	//	p_obj=(unsigned char *) &camera_default_config;
//		p_obj=(unsigned char *)&g_cam_cfg[index].cam_param;
//	//	class_len=sizeof(mCamParam);
//		flg=GET_FLAG;
//		break;	  //获取相机参数命名
//	case GETDETECTDEVICE:
//		reply_type=REPDETECTDEVICE;
//		class_type=CLASS_mDetectDeviceConfig;
//	//	p_obj=(unsigned char *) &device_default_config;
//		p_obj=(unsigned char *)&g_dev_cfg;
//		//class_len=sizeof(mDetectDeviceConfig);
//		flg=GET_FLAG;
//		break;   //获取检测设备参数命名
//
//	case GETCHECHPARAM:
//		reply_type=REPCHECHPARAM;
//		class_type=CLASS_mCamDetectParam;
//	//	p_obj=(unsigned char *) &camera_det_default_config;
//		p_obj=(unsigned char *)&g_cam_cfg[index].det_param;
//		//class_len=sizeof(mCamDetectParam);
//		flg=GET_FLAG;
//
//		break;   //获取视频画线参数
////	case REPCHECHPARAM:
////		break;   //获取视频画线参数回应
//
//	case HEART:
//		reply_type=REPHEART;
//		class_type=CLASS_NULL;
//		p_obj=(unsigned char *) 0;
//		class_len=0;
//		break;  //心跳包数据协议
//	case SHUTDOWN:
//		cmd_len=0;
//		break; //关闭命令
//	case REPHEART:
//		break;   //心跳包数据协议回应
//	case REBOOTZEN:
//		break;   //重启命令
//	case FORKEXIT:
//		break;  //程序异常退出
//	default:
//		break;
//	}
//	switch(flg){
//	case GET_FLAG:
//        class_len=get_obj_len(class_type);
//		ret = prepare_pkt(buf, cmd_len, reply_type, class_type,class_len,p_obj);
//		net_decode_obj((unsigned char *) cmd_p, CLASS_mCommand, 1);
//		break;
//	case SET_FLAG:
//		net_decode_obj((unsigned char *) (buf+cmd_len), class_type, 0);
//
//		handle_change(buf+cmd_len,class_type,index);
//		sync_obj((unsigned char *) (buf+cmd_len),class_type,index);
//		break;
//	default: break;
//	}
//
//	prt(protocol,"ret length %d",ret);
//	return ret;
//}
//TODO external

//void init_config()
//{
//	ip_list=new_list(sizeof(m_client_ip_data),(void *)client_info_match);
//
//
//	int i=0;
//	//load cfg
//	char fname[FILE_NAME_LENGTH];
//	int pos,len;
//	len=get_obj_len(CLASS_mDetectDeviceConfig);
//	pos=0;
//	load_buf(cfg_file_name(fname,CLASS_mDetectDeviceConfig),(char *)&g_dev_cfg,pos,len);
//	for(i=0;i<CAM_NUM;i++){
//		len=get_obj_len(CLASS_mCamParam);
//		pos=i*len;
//		load_buf(cfg_file_name(fname,CLASS_mCamParam),(char *)&g_cam_cfg[i].cam_param,pos,len);
//		len=get_obj_len(CLASS_mCamDetectParam);
//	//	save_obj((unsigned char *) &camera_det_default_config,CLASS_mCamDetectParam,i);
//		pos=i*len;
//		load_buf(cfg_file_name(fname,CLASS_mCamDetectParam),(char *)&g_cam_cfg[i].det_param,pos,len);
//
////		prt(info,"===11111>%d",g_cam_cfg[i].det_param.area.vircoordinate[2].x);
//		if(	g_dev_cfg.camstatus[i]==1){
//			camera_open(i,(char *)g_cam_cfg[i].cam_param.camattr.cammerIp,
//					g_cam_cfg[i].cam_param.camattr.cammerport,
//					(char *)g_cam_cfg[i].cam_param.camattr.username,
//					(char *)g_cam_cfg[i].cam_param.camattr.passwd);
//		}
//		//memcpy((unsigned char *)&g_cam_cfg[i].cam_param,(unsigned char *)&camera_default_config,sizeof(mCamParam));
//	 //	memcpy((unsigned char *)&g_cam_cfg[i].det_param,(unsigned char *)&camera_det_default_config,sizeof(mCamDetectParam));
//	}
//
//
//	//memcpy( (unsigned char *)&g_dev_cfg,(unsigned char *)&device_default_config,sizeof(mDetectDeviceConfig));
//}
//mCamDetectParam *get_mCamDetectParam(int index)
//{
//	return &g_cam_cfg[index].det_param;
//}
//mCamParam *get_mCamParam(int index)
//{
//	return &g_cam_cfg[index].cam_param;
//}

//int get_cam_status(int index)
//{
//	return g_dev_cfg.camstatus[index];
//}
//int get_cam_id(int index)
//{
//	return  g_cam_cfg[index].cam_param.camattr.camID;
//}
//int get_cam_direction(int index)
//{
//	return  g_dev_cfg.camdirect[index];
//}
//int get_dev_id()
//{
//	return  g_dev_cfg.deviceID;
//}
//void get_sig_ip(char *des)
//{
//	strcpy( des,(char *) g_cam_cfg[0].cam_param.camattr.signalIp);
//}
//int  get_sig_port()
//{
//	return g_cam_cfg[0].cam_param.camattr.signalport;
//}
