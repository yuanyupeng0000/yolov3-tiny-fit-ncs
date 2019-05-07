/*
 * author ryder jia
 * date 2016??10??11??
 * File introduction: ??????????????
 * service init?????????
 * ??????open??close??control????????????????????????????????????
 * ???????????????process fun
 * ????????????????????
 */
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include "camera_service.h"
#include "common.h"
#include "cam_net.h"
#include "cam_alg.h"

#include "cam_net.h"

#include "cam_codec.h"
#include "glplayer.h"
#include "h264_stream_file.h"
#include "client_net.h"


#define FRAME_COLS 1920 //640
#define FRAME_ROWS 1080 //480
/*

 * ??????????????????????????
 * */
extern  int stoped;
typedef struct camera_info {
    int updated;
    int index;
//	int stop_run;//TODO: lock this
    pthread_mutex_t run_lock;
    int open_flg;
    int cam_running_state;
    int cmd;
    unsigned char ip[16];
    unsigned char name[16];
    unsigned char passwd[16];
    int port;
    pthread_mutex_t cmd_lock;
    m_cam_context * p_cam;
#ifdef USE_FILE
    m_h264_file_common h264_file_common;
#endif
#ifdef PLAY_BACK
    m_gl_common gl_common;
#endif
    m_timed_func_data *p_data;
    unsigned char *oubuf;
    unsigned char *oubufu;
    unsigned char *oubufv;
    int watchdog_value;
    int watchdog_frames;

    unsigned char ybuf[FRAME_COLS*FRAME_ROWS];
    unsigned char ubuf[FRAME_COLS*FRAME_ROWS/4];
    unsigned char vbuf[FRAME_COLS*FRAME_ROWS/4];
    pthread_mutex_t frame_lock;
    pthread_mutex_t frame_lock_ex;
    pthread_t process_thread;
} m_camera_info;
enum{
    CAM_RUNNING,
    CAM_STOPED
};
enum{
    DOG_HUNGRY_STATE,
    DOG_FULL_STATE
};

void *move_pic(void *data)
{
    m_camera_info *p_info=(m_camera_info*)data;
     usleep(20);
    pthread_mutex_lock(&p_info->frame_lock);
  //   prt(info,"--------------------------------------a-------------------------------->send out");

    run_alg(p_info->index, p_info->ybuf, p_info->ubuf, p_info->vbuf);
    memcpy(p_info->ybuf,p_info->oubuf,FRAME_COLS*FRAME_ROWS);
    memcpy(p_info->ubuf,p_info->oubufu,FRAME_COLS*FRAME_ROWS/4);
    memcpy(p_info->vbuf,p_info->oubufv,FRAME_COLS*FRAME_ROWS/4);
   //  prt(info,"--------------------------------------b-------------------------------->send out");
    pthread_mutex_unlock(&p_info->frame_lock);
   usleep(20);
}
//#define CAMERA_NUMBER 4
#define CHECK_DURATION 1000//US
#define WATCHDOG_CHECK_DURATION 1000*1000*10//10s
#define FRAME_DURATION 100000//US
m_camera_info cam_info[ACTIVE_CAM_NUM];
/* Function Description
 * name:
 * return:
 * args:
 * comment:???????????????????§Ý??????????
 * todo:
 */
//static int test1=0;
void *process_fun(int index, char *data, int size)
{
 //  while(test1++>300){
//	prt(info,"stop");
//}
  //  prt(info,"get  pkt %d %d",index,size);

    FRAME_HEAD	*pFrameHead = (FRAME_HEAD *)data;
    EXT_FRAME_HEAD *exfream = (EXT_FRAME_HEAD *)(data + sizeof(FRAME_HEAD));

    //printf("myStreamCallBack----------------\n");

    if (FRAME_FLAG_A == pFrameHead->streamFlag)
    {
        return NULL;
    }
    int ret=0;
    //prt(info,"running %d",index);
    m_camera_info *p_info = &cam_info[index];
//	prt(info, "call back %d", p_info->index);
    p_info->watchdog_value=DOG_FULL_STATE;
    pthread_mutex_lock(&p_info->cmd_lock);

#ifdef PLAY_BACK
    if (p_info->gl_common.window_id < 0)
    {
        //	prt(info,"start  gl window");
        start_gl_window(&p_info->gl_common.window_id);
    }
#endif
    //	prt(info, "call back %d", p_info->index);
#ifdef USE_FILE
    //	prt(info,"gettiing file");
    get_file_264buf1(&p_info->h264_file_common);
    //	prt(info,"gettiing file done");
    data =(char *) p_info->h264_file_common.h264_pkt.data;
    size =p_info->h264_file_common.h264_pkt.size;
    prt(info,"get %d %d",index,size);
#else
    //	p_info->codec_common.av_pkt.data =(uint8_t *) data;
    //	p_info->codec_common.av_pkt.size = size;
#endif
    data=data+ sizeof(FRAME_HEAD)+sizeof(EXT_FRAME_HEAD);
//size= pFrameHead->nByteNum ;
    pthread_mutex_lock(&p_info->frame_lock);
    h264_decode(index, data-100, size, &p_info->oubuf, &p_info->oubufu,
            &p_info->oubufv);
    //if(p_info->oubuf!=NULL){
     //   p_info->updated=1;
    //}
    pthread_mutex_unlock(&p_info->frame_lock);
#ifdef PLAY_BACK
    copy_frame(p_info->oubuf, p_info->oubufu,
            p_info->oubufv, WIDTH, HEIGHT,
            (unsigned char *) p_info->gl_common.bf_src);
    //	memset(p_info->gl_common.bf_src,0,WIDTH*HEIGHT*3/2);

    diaplay_frames(p_info->gl_common.bf_src, WIDTH, HEIGHT,
            p_info->gl_common.bf_dst, p_info->gl_common.window_id);
#endif
	usleep(30000);
if(p_info->oubuf!=NULL){
    printf("decode ok\n");fflush(NULL);
//     if((ret=run_alg(p_info->index, p_info->oubuf, p_info->oubufu, p_info->oubufv))>0 ){
//         cam_set_shutter(index, get_mCamDetectParam(index)->detectparam[ret -1].shutterMax);
//         reset_alg(index);
//     }
//    pthread_t td;
//    pthread_create(&td,NULL,move_pic,(void *)p_info);
//    pthread_detach(td);
//     client_output(p_info->index);
}else{
    printf("decode err\n");fflush(NULL);
}
  //	 submit_sig_data(&p_channel_rst[index],index);
        //send_sig_traffic_data(index);
        //	send_client_data();
        //send_sig_machine_data();
     pthread_mutex_unlock(&p_info->cmd_lock);
}

int get_cmd(int index)
{
    return cam_info[index].cmd;
}
void set_cmd(int cmd, int index)
{
//	prt(info,"cam %d  set cmd to %d",index,cmd);
    cam_info[index].cmd = cmd;
}
void *file_process_fun(int *index, char *data, int size)
{
    m_camera_info *p_info = (m_camera_info *) data;
    process_fun(*index, data, size);
//	start_detached_func((void *)real_process_fun,data);
}
#include "cam_alg.h"
static int test3=0;

void *start_process(void *data)
{
	unsigned char updated = 0;
	m_camera_info *p_info=(m_camera_info*)data;
    while(1){
	    pthread_mutex_lock(&p_info->frame_lock);
	    if(p_info->oubuf){
	    memcpy(p_info->ybuf,p_info->oubuf,FRAME_COLS*FRAME_ROWS);
	    memcpy(p_info->ubuf,p_info->oubufu,FRAME_COLS*FRAME_ROWS/4);
	    memcpy(p_info->vbuf,p_info->oubufv,FRAME_COLS*FRAME_ROWS/4);
	    }
	    updated = 1;
	    p_info->oubuf = NULL;
	    pthread_mutex_unlock(&p_info->frame_lock);
	    pthread_mutex_lock(&p_info->frame_lock_ex);
	    if(!((++test3)%10000)){
	    	prt(info,"##process alg %d",test3);
		}
		
	    if(updated){
	    	run_alg(p_info->index, p_info->ybuf, p_info->ubuf, p_info->vbuf);
			p_info->watchdog_frames++;
		updated = 0;
	    }
		
		if(!((++test3)%10000)){
		    prt(info,"##process alg  done%d",test3);
		}

	    //p_info->updated=0;
	    pthread_mutex_unlock(&p_info->frame_lock_ex);
	    client_output(p_info->index);
	    usleep(10);
    }
}

/* Function Description
 * name:
 * return:
 * args:
 * 		index?????????????????????????????????????????ctrl?? ??reset??
 * 		ip port username passwd
 * comment:
 * 		?????????????????????????????????????????????????????????
 * todo:
 */
int camera_open(int index, char ip[], int port, char username[], char passwd[])
{
    int ret=-1;
    prt(camera_msg,"open camera %d",index);
    //????????????????????reset??????
    if (cam_info[index].open_flg) {
        camera_ctrl(CAMERA_CONTROL_RESET, index, 0, NULL);
        return 0;
    }
    //???????
    open_alg(index);
    //?????????
    open_h264_decoder(index);
#ifdef USE_FILE
    char filename[]="/mediafiles/test.2640";
    //start_gl_window(&cam_info[index].gl_common.window_id);
    memcpy(cam_info[index].h264_file_common.file_name,filename,sizeof(filename));
//	cam_info[index].h264_file_common.h264_pkt=&cam_info[index].codec_common.av_pkt;
    open_h264_file(&cam_info[index].h264_file_common);
    cam_info[index].p_data=regist_timed_func(FRAME_DURATION,(void *)file_process_fun,(void *)&index);
    start_timed_func(cam_info[index].p_data);

#else
    //prt(info,"ip %s ",ip);
    memset(cam_info[index].ip,0,16);
    memset(cam_info[index].name,0,16);
    memset(cam_info[index].passwd,0,16);
    memcpy(cam_info[index].ip, ip, strlen(ip));
//	prt(info,"ip %s ",cam_info[index].ip);
    memcpy(cam_info[index].name, username, strlen(username));
    memcpy(cam_info[index].passwd, passwd, strlen(passwd));
    cam_info[index].port = port;

//	cam_info[index].p_cam = prepare_camera((char *) cam_info[index].ip,
//			cam_info[index].port, (char *) cam_info[index].name,
//			(char *) cam_info[index].passwd, (void *) frame_process_fun,
//			&cam_info[index]);
    ret=net_open_camera(index, (char *) cam_info[index].ip, cam_info[index].port,
            (char *) cam_info[index].name, (char *) cam_info[index].passwd,
            (void *) process_fun, &cam_info[index]);
//	open_camera(cam_info[index].p_cam);
    if(!ret){
        cam_info[index].cam_running_state = CAM_RUNNING;
    }else{
        cam_info[index].cam_running_state = CAM_STOPED;
    }
#endif
    cam_info[index].open_flg = 1;
prt(info,"creating thread ............................");
    pthread_create( &cam_info[index].process_thread,NULL,start_process, (void *)&cam_info[index]);
    return 0;
}
int camera_close(int index)
{
    prt(info,"close camera %d",index);
    m_camera_info *p_info = (m_camera_info *) &cam_info[index];
#ifdef PLAY_BACK
    stop_gl_window(&p_info->gl_common.window_id);
    p_info->gl_common.window_id=-1;
#endif
//		close_h264_file(&h264_file_common);
//		close_h264_decoder1(&codec_common);

#ifdef USE_FILE
    stop_timed_func(cam_info[index].p_data);
    close_h264_file(&cam_info[index].h264_file_common);
#else
    net_close_camera(index);
//	close_camera(cam_info[index].p_cam);
    if (cam_info[index].p_cam != NULL) {
        free(cam_info[index].p_cam);
    }
#endif
    close_h264_decoder(index);
    release_alg(index);
    cam_info[index].open_flg = 0;
    cam_info[index].cam_running_state = CAM_STOPED;
    return 0;
}
int camera_ctrl(int cmd, int index, int blocked, void *data)
{
    pthread_mutex_lock(&cam_info[index].cmd_lock);
    set_cmd(cmd, index);
    pthread_mutex_unlock(&cam_info[index].cmd_lock);
    return 0;
}

void *watchdog_func(void *data)
{
        prt(camera_msg,"watch dog ------------>...");
    m_camera_info *info = (m_camera_info *) data;
	prt(info,"*********************watchdog dog for cam %d , fp10s:%d****************",info->index,info->watchdog_frames);
        //if(info->watchdog_frames<2&&info->index==0&&stoped==0){

        if(0){

        prt(camera_msg,"watch dog reboot ");
//		alg_mem_free();
        release_alg(info->index);
		system("reboot");
	}
	info->watchdog_frames=0;
    if(info->watchdog_value==DOG_HUNGRY_STATE&&info->open_flg){
        prt(camera_msg,"watch dog for cam %d:camera loop stopped,resetting...",info->index);
        camera_ctrl(CAMERA_CONTROL_RESET,info->index,1,NULL);
        info->cam_running_state = CAM_STOPED;
    }else{
        info->cam_running_state = CAM_RUNNING;
    //	prt(camera_msg,"watch dog for cam %d:camera running normally",info->index);
    }
    info->watchdog_value = DOG_HUNGRY_STATE;
}
int get_cam_running_state(int index)
{
    return cam_info[index].cam_running_state ;
}

/* Function Description
 * name:
 * return:
 * args:data????????????????????‰^
 * comment:?????????????????????????????????????þŸ
 * todo:
 */
void *camera_main(void *data)
{
    int ret;
    m_camera_info *info = (m_camera_info *) data;
    int index = info->index;
    pthread_mutex_lock(&info->cmd_lock);
    switch (get_cmd(info->index)) {
    case CAMERA_CONTROL_NULL:
        break;
    case CAMERA_CONTROL_RESET:
//		break;
#ifndef USE_FILE
        prt(info, "reseting............ camera %d",index);
    //	pthread_mutex_lock(&info->cmd_lock);
        //	close_camera(info->p_cam);
        //open_camera(info->p_cam);
        prt(info, "closing camera %d",index);
        net_close_camera(index);
        prt(info, "  camera %d closed",index);
        ret=net_open_camera(index, (char *) cam_info[index].ip,
                cam_info[index].port, (char *) cam_info[index].name,
                (char *) cam_info[index].passwd, (void *) process_fun,
                &cam_info[index]);
        prt(info, "  camera %d opened",index);
        if(ret<0){
            prt(debug_long,"login %s (port %d) fail",cam_info[index].ip,cam_info[index].port);
        }
       pthread_mutex_lock(&info->frame_lock_ex);
        reset_alg(info->index);
       pthread_mutex_unlock(&info->frame_lock_ex);
        prt(info, "  camera %d reset done",index);
    //	pthread_mutex_unlock(&info->cmd_lock);
#endif
        break;
    case CAMERA_CONTROL_SET_NTP:
        prt(info, "set cam")
        ;
        break;
    case CAMERA_CONTROL_RESET_ALG:
    //	pthread_mutex_lock(&info->cmd_lock);
    pthread_mutex_lock(&info->frame_lock_ex);
        reset_alg(info->index);
    pthread_mutex_unlock(&info->frame_lock_ex);
    //	pthread_mutex_unlock(&info->cmd_lock);
        prt(info, "reset alg");
        break;
    default:
        break;
    }
    if (get_cmd(info->index) != CAMERA_CONTROL_NULL)
        set_cmd(CAMERA_CONTROL_NULL, info->index);
    pthread_mutex_unlock(&info->cmd_lock);
}
/* Function Description
 * name:
 * return:
 * args:
 * comment:?????????????????????????????????????????????????????????????????????????cpu§¹???????§Õ????????î•
 * ?????????????????????????????????????
 * todo:
 */
void camera_service_init()
{
    int i;
    //?????????????sdk
    if (open_sdk()) {
        prt(info, "err in open sdk");
    } else {
        prt(info, "ok to open net camera  sdk");
    }
#ifdef PLAY_BACK
    init_gl();	//?????opengl
#endif
    //????????????????
#ifdef USE_FILE
    for(i=0;i<1;i++){
#else
    for (i = 0; i < ACTIVE_CAM_NUM; i++) {
#endif
        init_alg(i);
#ifdef PLAY_BACK
        cam_info[i].gl_common.window_id=-1;
#endif
        cam_info[i].open_flg = 0;
        cam_info[i].index = i;
        cam_info[i].cam_running_state = CAM_STOPED;
            set_cmd(CAMERA_CONTROL_NULL, i);	//????????????????????????????????????????????????????????
        pthread_mutex_init(&cam_info[i].cmd_lock, NULL);
        pthread_mutex_init(&cam_info[i].run_lock, NULL);
        pthread_mutex_init(&cam_info[i].frame_lock, NULL);
        pthread_mutex_init(&cam_info[i].frame_lock_ex, NULL);
               //prt(info,"init addr %p,i %d",&cam_info[i].cmd_lock,i);
        m_timed_func_data *p_data = regist_timed_func(CHECK_DURATION,
                (void *) camera_main, (void *) &cam_info[i]);//?1/1000???????????????????
        start_timed_func(p_data);

        cam_info[i].watchdog_value=DOG_HUNGRY_STATE;
        m_timed_func_data *p_data_watchdog = regist_timed_func(WATCHDOG_CHECK_DURATION,
                (void *) watchdog_func, (void *) &cam_info[i]);//?10??????????????????????????
        start_timed_func(p_data_watchdog);
    }
}
