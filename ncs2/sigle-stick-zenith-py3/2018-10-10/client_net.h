/*
 * client_net.h
 *
 *  Created on: 2016年10月19日
 *      Author: root
 */

#ifndef CLIENT_NET_H_
#define CLIENT_NET_H_
#include "client_obj.h"
int handle_buffer(unsigned char *buf, int len, char *ip);
int  handle_buffer(unsigned char *buf,int len,char *ip);
void init_config();

mCamDetectParam *get_mCamDetectParam(int index);
mCamParam *get_mCamParam(int index);

int get_cam_status(int index);
int get_cam_id(int index);
int get_cam_direction(int index);
int get_dev_id();
void get_sig_ip(char *des);
int  get_sig_port();
int get_lane_num(int index);
//void save_obj(unsigned char * p_obj,int class_type,int index);
void client_output(int index);
mRealStaticInfo *client_get_info(int index);

int get_lane_index(int index,int lane_i);
int get_direction(int index);
void get_udp_info(char *ip, unsigned short *port); //获取udp ip和port

//#define CAM_MAX 4
#define BUFFER_MAX 1000
#define TCP_TIMEOUT 100
#endif /* CLIENT_NET_H_ */
