/*
 * camera_service.h
 *
 *  Created on: 2016年9月9日
 *      Author: root
 */

#ifndef CAMERA_SERVICE_H_
#define CAMERA_SERVICE_H_
#include <pthread.h>
#include "g_define.h"

enum{
	CAMERA_CONTROL_NULL,
	CAMERA_CONTROL_OPEN,
	CAMERA_CONTROL_CLOSE,
	CAMERA_CONTROL_RESET,
	CAMERA_CONTROL_SET_NTP,
	CAMERA_CONTROL_RESET_ALG
};
//
//#include "cam_net.h"
//
//#include "cam_codec.h"
//#include "glplayer.h"
//#include "h264_stream_file.h"
///*
//
// * 代表一个相机的全部资源或者信息
// * */
//typedef struct camera_info {
//	int index;
//	int stop_run;//TODO: lock this
//	int open_flg;
//	int cam_state;
//	int cmd;
//	unsigned char ip[16];
//	unsigned char name[16];
//	unsigned char passwd[16];
//	int port;
//	pthread_mutex_t cmd_lock;
//	m_cam_context * p_cam;
//
//#ifdef USE_FILE
//	m_h264_file_common h264_file_common;
//#endif
//#ifdef PLAY_BACK
//	m_gl_common gl_common;
//#endif
////	m_codec_common codec_common;
//	m_timed_func_data *p_data;
//	unsigned char *oubuf;
//	unsigned char *oubufu;
//	unsigned char *oubufv;
//} m_camera_info;
//

void camera_service_init();
int camera_ctrl(int cmd, int index,int blocked, void *data);
int camera_open(int index,char ip[], int port, char username[],char passwd[]);
int camera_close(int index);
int get_cam_running_state(int index);
#endif /* CAMERA_SERVICE_H_ */
