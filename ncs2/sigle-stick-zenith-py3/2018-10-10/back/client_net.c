#include "client_net.h"
#include "client_obj.h"
#include "common.h"
#include "g_define.h"

#include <arpa/inet.h>
#include "file_op.h"

#include "camera_service.h"
#include "client_file.h"
#include "sig_service.h"
#define CHECK_TIMES 15*60//log???????
typedef struct client_ip_data {
        char ip[16];
        int camera_index;
        int time;
        int fd;
        int times;//
} m_client_ip_data;
enum{
        CLIENT_IP_SAME=0,
        CLIENT_CAM_INDEX_SAME
};
typedef struct cam_cfg {
        mCamParam cam_param;
        mCamDetectParam det_param;
} m_cam_cfg;
m_cam_cfg g_cam_cfg[ACTIVE_CAM_NUM];
mDetectDeviceConfig g_dev_cfg;
mRealStaticInfo static_info[ACTIVE_CAM_NUM];
mRealStaticInfo *client_get_info(int index)
{
        return &static_info[index];
}
#include "csocket.h"
enum {
        NOOP_FLAG,
        SET_FLAG,
        GET_FLAG
};
m_list_info *ip_list;
void *client_send_udp(void *node_data,void *data)
{
        m_client_ip_data *p_data = (m_client_ip_data *) node_data;

        int *p_index = (int *) data;
        int index = *p_index;

        //prt(info,"get index %d",index);
        //	prt(info,"fd %d(%p),sending dpu to ip %s,index %d,coming index %d",p_data->fd,&p_data->fd,p_data->ip,p_data->camera_index,index);
        if (p_data->camera_index == index) {
                if(p_data->times++==CHECK_TIMES){
                        prt(clients_msg,"cam %d output to client %s",index,p_data->ip);
                        p_data->times=0;
                }
     //   static_info[index].lane[0].vehnum;

        //prt(info,"$$$$$$$$$$$$$$$$$$$$$$$$ %d",static_info[index].area_car_num[0]);
        int sended=UdpSendData(p_data->fd, p_data->ip, SERVERPORT,
                        (char*) &static_info[index], sizeof(mRealStaticInfo));


   // prt(info,"send size %d to%s",sended,p_data->ip);
        }


}
void client_output(int index)
{
        list_operate_node_all(ip_list,(p_func )client_send_udp,(void *)&index);
}
#include "csocket.h"
void add_client(char *ip,int index)
{
        m_client_ip_data *p_tmp;
        list_node_alloc_tail(ip_list);
        p_tmp=(m_client_ip_data *)ip_list->tail->data;
        p_tmp->camera_index=index;
        memcpy(p_tmp->ip, ip,16);
        p_tmp->fd=-1;
        while(p_tmp->fd<=0)
        {
        //	p_tmp->fd = UdpCreateSocket(get_random_port());
                p_tmp->fd = UdpCreateSocket(get_random_port());
                //	prt(info,"create udp socket");
        }
        prt(clients_msg,"add client :%s, index %d  ",p_tmp->ip, index );
        prt(clients_msg,"total num of clients is %d",ip_list->number);
}
void del_client(char *ip,int index)
{
        prt(clients_msg,"delclient  %s",ip);
//	prt(info,"total num of clients is %d",ip_list->number);
        list_node_del_cur(ip_list);
        prt(clients_msg,"total num of clients is %d",ip_list->number);
}

int client_info_match(void *ori_info,void *new_info)
{
        m_client_ip_data *p1=(m_client_ip_data *)ori_info;
        m_client_ip_data *p2=(m_client_ip_data *)new_info;

        if(!memcmp(p1->ip,p2->ip,16)){
                if(p1->camera_index==p2->camera_index)
                        return CLIENT_CAM_INDEX_SAME;
                else
                        return CLIENT_IP_SAME;
        }
        return -1;
}
enum {
        CHANGE_NOTHING,
        CHANGE_ALG,
        CHANGE_SIG_IP,
        CHANGE_CAM_IP,
        CHANGE_ADJUST_IP
};
int client_cmd_flag=CHANGE_NOTHING;
void sync_obj(unsigned char * p_obj,int class_type,int index)
{
        int pos=0;
        char filename[FILE_NAME_LENGTH];
        int len=get_obj_len(class_type);
        if(class_type==CLASS_mDetectDeviceConfig){
                pos=0;
        }else{
                pos=index*len;
        }
        char *p_dst=NULL;
        switch(class_type){
        case CLASS_mCamParam:
                p_dst=(char *)&g_cam_cfg[index].cam_param;
                break;
        case CLASS_mDetectDeviceConfig:
                p_dst=(char *)&g_dev_cfg;
                break;
        case CLASS_mCamDetectParam:
                p_dst=(char *)&g_cam_cfg[index].det_param;
                break;

        default :
                prt(info,"unsupported save class %d",class_type);
                break;
        }
        memcpy(p_dst,p_obj,len);
}
void handle_change(unsigned char * p_obj,int class_type,int index)
{
        char *p_old_obj=NULL;
        switch(class_type){
        case CLASS_mCamParam:
                prt(debug_client_cmds,"get CLASS_mCamParam");
//		prt(info,"saving cam param,checking alg change,sigmachine change , ntp change");
//		//memcpy(&g_cam_cfg[index],p_obj,len);
// 		prt(info,"11change ip  %s  to %s ", g_cam_cfg[index].cam_param.camattr.cammerIp ,
// 						(char *)((mCamParam *)p_obj)->camattr.cammerIp);
//		p_old_obj=(char *)&g_cam_cfg[index].cam_param;
//		camera_ctrl(CAMERA_CONTROL_RESET_ALG,index,0,NULL);
//		prt(info,"####change ip  %s  to %s ", g_cam_cfg[index].cam_param.camattr.cammerIp ,
//				(char *)((mCamParam *)p_obj)->camattr.cammerIp);
//		if(strcmp((char *)g_cam_cfg[index].cam_param.camattr.cammerIp,
//				(char *)((mCamParam *)p_obj)->camattr.cammerIp)){
//			prt(info,"change ip  %s  to %s ", g_cam_cfg[index].cam_param.camattr.cammerIp ,
//					(char *)((mCamParam *)p_obj)->camattr.cammerIp);
//			mCamParam *tmp=(mCamParam *)p_obj;
//		//	camera_open(index,(char *)tmp->camattr.cammerIp,tmp->camattr.cammerport,(char *)tmp->camattr.username,(char *)tmp->camattr.passwd);
////			camera_open(index,(char *)((mCamParam *)p_obj).cam_param.camattr.cammerIp,g_cam_cfg[index].cam_param.camattr.cammerport,
////					(char *)g_cam_cfg[index].cam_param.camattr.username,(char *)g_cam_cfg[index].cam_param.camattr.passwd);
//
//		//	camera_ctrl(CAMERA_CONTROL_RESET,index,0,NULL);
//			prt(info,"warning ----------------- ip changed checkit");
//			// client_cmd_flag=CHANGE_CAM_IP;
//
//		}
//
                if(index==0&&(strcmp((char *)g_cam_cfg[index].cam_param.camattr.signalIp,(char *)((mCamParam *)p_obj)->camattr.signalIp)||((int)g_cam_cfg[index].cam_param.camattr.signalport!=(int)((mCamParam *)p_obj)->camattr.signalport)))
                {
                                prt(info,"change  sig ip  %s  to %s ", g_cam_cfg[index].cam_param.camattr.signalIp ,
                                                (char *)((mCamParam *)p_obj)->camattr.signalIp);
                        //	reset_sig_machine();

                                 client_cmd_flag=CHANGE_SIG_IP;
                }
            prt(info,"index (%d) , direction  %d",index,((mCamParam *)p_obj)->camattr.direction);
                break;
                /*
                 *?????????????????????????????
                 */
        case CLASS_mDetectDeviceConfig:
                prt(debug_client_cmds,"get CLASS_mDetectDeviceConfig");
        //	prt(info,"saving det dev,checking camera status , direction");
                p_old_obj=(char *)&g_dev_cfg;
                if(strcmp((char *)g_dev_cfg.detectname,(char *)((mDetectDeviceConfig *)p_obj)->detectname)){
                //	prt(info,"change name  %s  to %s ",g_dev_cfg.detectname,((mDetectDeviceConfig *)p_obj)->detectname);
                        prt(info,"change det name");
                }

                for (int i = 0; i < ACTIVE_CAM_NUM; i++) {
                        if(strcmp((char *)g_dev_cfg.cam_info[i].cammerIp,(char *)((mDetectDeviceConfig *)p_obj)->cam_info[i].cammerIp)){

//				if(((mDetectDeviceConfig *) p_obj)->cam_info[i].camstatus == 1){
                //		prt(info," ip from  %s  to %s ",(char *)g_dev_cfg.cam_info[i].cammerIp,(char *)((mDetectDeviceConfig *)p_obj)->cam_info[i].cammerIp);
//				//	camera_ctrl(CAMERA_CONTROL_RESET,i,0,NULL);
//				    client_cmd_flag=CHANGE_CAM_IP;
//				}
                                camera_close(i);
                                g_dev_cfg.cam_info[i].camstatus = CAM_CLOSED_STATUS;
                        }

                        if ((g_dev_cfg.cam_info[i].camstatus == CAM_CLOSED_STATUS)
                                        && ((mDetectDeviceConfig *) p_obj)->cam_info[i].camstatus == CAM_OPENED_STATUS) {
                                prt(info,"open cam %d",i);
                                prt(info," ip from  %s  to %s ",(char *)g_dev_cfg.cam_info[i].cammerIp,(char *)((mDetectDeviceConfig *)p_obj)->cam_info[i].cammerIp);

                                camera_open(i,(char *)((mDetectDeviceConfig *)p_obj)->cam_info[i].cammerIp,
                                                g_cam_cfg[i].cam_param.camattr.cammerport,
                                                (char *)g_cam_cfg[i].cam_param.camattr.username,
                                                (char *)g_cam_cfg[i].cam_param.camattr.passwd);

                        }

                        if ((g_dev_cfg.cam_info[i].camstatus == CAM_OPENED_STATUS)
                                        && ((mDetectDeviceConfig *) p_obj)->cam_info[i].camstatus == CAM_CLOSED_STATUS) {
                                prt(info,"close cam %d",i);
                                camera_close(i);
                        }

                }
        //	camera_ctrl(CAMERA_CONTROL_RESET_ALG,index,0,NULL);
                break;
        case CLASS_mCamDetectParam:
                prt(debug_client_cmds,"get CLASS_mCamDetectParam");
//		prt(info,"===11111>%d",g_cam_cfg[index].det_param.area.vircoordinate[2].x);
                p_old_obj=(char *)&g_cam_cfg[index].det_param;
                 client_cmd_flag=CHANGE_ALG;
        //	camera_ctrl(CAMERA_CONTROL_RESET_ALG,index,0,NULL);
                break;
        default :
                break;
        }
}
int handle_buffer(unsigned char *buf, int len, char *ip)
{
        int type;
        int length;
        int ret=-1;
        int match_ret=-1;
        mCommand *cmd_p;
        cmd_p = (mCommand *) buf;
        net_decode_obj((unsigned char *) cmd_p, CLASS_mCommand, 0);
        type = cmd_p->objtype;
        int cmd_len = sizeof(mCommand);
        int index=cmd_p->objnumber;
        prt(debug_client_cmds, "get cmd type %x, length %d,index %d", type,len,cmd_p->objnumber);
        static m_client_ip_data tmp_data;
        m_client_ip_data *current_data;
        tmp_data.camera_index=index;
        memcpy(tmp_data.ip,ip,16);
        if(type==GETCAMPARAM||type==HEART||type==SHUTDOWN)
        if((match_ret=list_node_seek(ip_list,(void *)&tmp_data))>=CLIENT_IP_SAME){
                if(match_ret!=CLIENT_CAM_INDEX_SAME){
                //	prt(info,"deferent index " );
                        //list_overwirte_current_data(ip_list,(void *)&tmp_data);
                        current_data=(m_client_ip_data *)list_get_current_data(ip_list);
                        prt(info,"ip %s instead index %d",current_data->ip,index);
                        current_data->camera_index=index;
                //	current_data->camera_index=tmp_data.camera_index;
                //	memcpy(current_data->ip,tmp_data.ip,16);
                //	prt(info,":%s index change to %d  ",current_data->ip,tmp_data.camera_index);
                }else{
                        //prt(info,":%s index change to %d  ",current_data->ip,tmp_data.camera_index);
                        //prt(info,"same index " );
                }

        }else{
//		list_node_alloc_tail(ip_list);
//		current_data=(m_client_ip_data *)ip_list->tail->data;
//		current_data->camera_index=tmp_data.camera_index;
//		memcpy(current_data->ip,tmp_data.ip,16);
//		prt(info,"add :%s",current_data->ip);
                prt(info,"get index %d",tmp_data.camera_index);
                add_client(tmp_data.ip,tmp_data.camera_index);
        }
        int reply_type=0;
        int class_type=0;
        unsigned char *p_obj=NULL;
        int class_len=0;
        int flg=NOOP_FLAG;
        switch (type) {

//	m_cam_cfg g_cam_cfg[CAM_NUM];
//	mDetectDeviceConfig g_dev_cfg;

        case SETCAMPARAM:
                class_type=CLASS_mCamParam;
                flg=SET_FLAG;
                break;
        case SETDETECTDEVICE:
                class_type=CLASS_mDetectDeviceConfig;
                flg=SET_FLAG;
                break;    //????????????????
        case SETCHECHPARAM:
                class_type=CLASS_mCamDetectParam;
//		class_len=sizeof(mDetectDeviceConfig);
//		class_len=sizeof(mCamParam);
                flg=SET_FLAG;
                break;   //??????????????

//	case REPCAMPARAM:
//		break;	  //??????????????????
//	case REPDETECTDEVICE:
//		break;    //????????????????????
        case GETCAMPARAM:
                reply_type=REPCAMPARAM;
                class_type=CLASS_mCamParam;
        //	p_obj=(unsigned char *) &camera_default_config;
                p_obj=(unsigned char *)&g_cam_cfg[index].cam_param;
        //	class_len=sizeof(mCamParam);
                flg=GET_FLAG;
                break;	  //???????????????
        case GETDETECTDEVICE:
                reply_type=REPDETECTDEVICE;
                class_type=CLASS_mDetectDeviceConfig;
        //	p_obj=(unsigned char *) &device_default_config;
                p_obj=(unsigned char *)&g_dev_cfg;
                //class_len=sizeof(mDetectDeviceConfig);
                flg=GET_FLAG;
                break;   //?????????????????

        case GETCHECHPARAM:
                reply_type=REPCHECHPARAM;
                class_type=CLASS_mCamDetectParam;
        //	p_obj=(unsigned char *) &camera_det_default_config;
                p_obj=(unsigned char *)&g_cam_cfg[index].det_param;
                //class_len=sizeof(mCamDetectParam);
                flg=GET_FLAG;

                break;   //?????????????
//	case REPCHECHPARAM:
//		break;   //????????????????

        case HEART:
                reply_type=REPHEART;
                class_type=CLASS_NULL;
                p_obj=(unsigned char *) 0;
                class_len=0;
                break;  //?????????????
        case SHUTDOWN:
                del_client(tmp_data.ip,index);

                cmd_len=0;
                break; //???????
        case REPHEART:
                prt(info,"1");
                break;   //????????????????
        case REBOOTZEN:
                reboot_cmd();
                break;   //????????
        case FORKEXIT:
                prt(info,"1");
                break;  //?????????
        default:
                break;
        }
        switch(flg){
        case GET_FLAG:
        class_len=get_obj_len(class_type);
                ret = prepare_pkt(buf, cmd_len, reply_type, class_type,class_len,p_obj);
                net_decode_obj((unsigned char *) cmd_p, CLASS_mCommand, 1);
                break;
        case SET_FLAG:
                net_decode_obj((unsigned char *) (buf+cmd_len), class_type, 0);
                client_cmd_flag=CHANGE_NOTHING;
                handle_change(buf+cmd_len,class_type,index);
                sync_obj((unsigned char *) (buf+cmd_len),class_type,index);
                save_obj((unsigned char *) (buf+cmd_len),class_type,index);
                switch(client_cmd_flag){
                case CHANGE_NOTHING:break;
                case CHANGE_SIG_IP:
                        reset_sig_machine();
                        break;
//		case CHANGE_CAM_IP:
//			prt(info,"reset ip");
//			camera_ctrl(CAMERA_CONTROL_RESET,index,0,NULL);
//			break;
                case CHANGE_ALG:
                        camera_ctrl(CAMERA_CONTROL_RESET_ALG,index,0,NULL);
                        break;
                case CHANGE_ADJUST_IP:break;
                        default:break;
                }
                break;
        default: break;
        }

        prt(protocol,"ret length %d",ret);
        return ret;
}

void init_config()
{
        ip_list=new_list(sizeof(m_client_ip_data),(void *)client_info_match);
        int i=0;
        //load cfg
        char fname[FILE_NAME_LENGTH];
        int pos,len;
        len=get_obj_len(CLASS_mDetectDeviceConfig);
        pos=0;
        load_buf(cfg_file_name(fname,CLASS_mDetectDeviceConfig),(char *)&g_dev_cfg,pos,len); //加载检测设备配置到g_dev_cfg
//	prt(info,"get ip %s",g_dev_cfg.cam_info[0].cammerIp);;
//	prt(info,"get ip %s",g_dev_cfg.cam_info[1].cammerIp);;
//	prt(info,"get ip %s",g_dev_cfg.cam_info[2].cammerIp);;
//	prt(info,"get ip %s",g_dev_cfg.cam_info[3].cammerIp);;
//	prt(info,"get ip %s",g_dev_cfg.cam_info[4].cammerIp);;
//	prt(info,"get ip %s",g_dev_cfg.cam_info[5].cammerIp);;
//	prt(info,"get ip %s",g_dev_cfg.cam_info[6].cammerIp);;
//	prt(info,"get ip %s",g_dev_cfg.cam_info[7].cammerIp);;
#ifdef USE_FILE
        for(i=0;i<1;i++){
#else

        for(i=0;i<ACTIVE_CAM_NUM;i++){
#endif
                len=get_obj_len(CLASS_mCamParam);
                pos=i*len;
                load_buf(cfg_file_name(fname,CLASS_mCamParam),(char *)&g_cam_cfg[i].cam_param,pos,len); //加载相机参数到cam_param
                len=get_obj_len(CLASS_mCamDetectParam);
                pos=i*len;
                load_buf(cfg_file_name(fname,CLASS_mCamDetectParam),(char *)&g_cam_cfg[i].det_param,pos,len); //加检测机参数到det_param

                if(	g_dev_cfg.cam_info[i].camstatus==CAM_OPENED_STATUS){
                        prt(camera_msg,"cam %d, port %d,name %s,ip %s",i,(int)g_cam_cfg[i].cam_param.camattr.cammerport,
                                        (char *)g_cam_cfg[i].cam_param.camattr.username,
                                        (char *)g_dev_cfg.cam_info[i].cammerIp);
						
                        camera_open(i,(char *)g_dev_cfg.cam_info[i].cammerIp,
                                        (int)g_cam_cfg[i].cam_param.camattr.cammerport,
                                        (char *)g_cam_cfg[i].cam_param.camattr.username,
                                        (char *)g_cam_cfg[i].cam_param.camattr.passwd);   //打开相机
                }
        }
}
mCamDetectParam *get_mCamDetectParam(int index)
{
        return &g_cam_cfg[index].det_param;
}
mCamParam *get_mCamParam(int index)
{
        return &g_cam_cfg[index].cam_param;
}

int get_cam_status(int index)
{
        return g_dev_cfg.cam_info[index].camstatus;
}
int get_cam_id(int index)
{
        return  g_cam_cfg[index].cam_param.camattr.camID;
}
int get_cam_direction(int index)
{
        return  g_dev_cfg.cam_info[index].camdirect;
}
int get_dev_id()
{
        return  g_dev_cfg.deviceID;
}
void get_sig_ip(char *des)
{
    strcpy( des,(char *) g_cam_cfg[0].cam_param.camattr.signalIp);
 //   strcpy( des,"192.168.1.216");
}
int  get_sig_port()
{
   return g_cam_cfg[0].cam_param.camattr.signalport;
   //  return 5000;
}
int get_lane_num(int index)
{
        return g_cam_cfg[index].det_param.detectlane.lanenum;
}
int get_lane_index(int index,int lane_i)
{
    return g_cam_cfg[index].cam_param.channelcoil[lane_i].number;
}

void get_udp_info(char *ip, unsigned short *port)
{
	memcpy(ip, g_cam_cfg[0].cam_param.camattr.adjustIp, IPADDRMAX);
	*port = g_cam_cfg[0].cam_param.camattr.adjustport;
}

// 7- xi   --3
// 1 - bei  --0
// 5 - nan  --2
// 3 - dong  -- 1
int get_direction(int index)
{

  //  return g_cam_cfg[index].cam_param.camattr.direction;
  //  int dir=g_cam_cfg[index].cam_param.camattr.direction;

       int dir= g_dev_cfg.cam_info[index].camdirect;
     prt(info,"client dir %x(camera %d)",dir,index);

#if 0
    if(dir==0x40) dir=3;
    if(dir==0x10) dir=2;
    if(dir==0x04) dir=1;
    if(dir==0x01) dir=0;
#else
//      if(dir==0x07) dir=3;
//      if(dir==0x05) dir=2;
//      if(dir==0x03) dir=1;
//      if(dir==0x01) dir=0;

      if(dir==0x07) {dir=3;  return dir;}
      if(dir==0x05) {dir=2;  return dir;}
      if(dir==0x03) {dir=1;  return dir;}
      if(dir==0x01) {dir=0;  return dir;}
#endif

}
